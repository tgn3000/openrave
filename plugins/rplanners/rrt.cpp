// -*- coding: utf-8 -*-
// Copyright (C) 2006-2010 Rosen Diankov (rosen.diankov@gmail.com)
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

#include <boost/algorithm/string.hpp>
#include "rrt.h"

bool RrtPlanner::_InitPlan(RobotBasePtr pbase, PlannerParametersPtr params)
{
    params->Validate();
    _goalindex = _startindex = -1;
    const auto penv = this->GetEnv();
    const int envId = penv->GetId();
    const EnvironmentMutex::scoped_lock lock(penv->GetMutex());
    if( !_uniformsampler ) {
        _uniformsampler = RaveCreateSpaceSampler(penv, "mt19937");
    }
    _robot = pbase;

    _uniformsampler->SetSeed(params->_nRandomGeneratorSeed);
    for(const SpaceSamplerBasePtr& psampler : params->_listInternalSamplers) {
        psampler->SetSeed(params->_nRandomGeneratorSeed);
    }

    const PlannerParameters::StateSaver savestate(params);
    const auto pcc = penv->GetCollisionChecker();
    const CollisionOptionsStateSaver optionstate(pcc, pcc->GetCollisionOptions() | CO_ActiveDOFs, false);
    const int dof = params->GetDOF();

    const std::vector<dReal>& vinitialconfig_ = params->vinitialconfig;
    const int configsize = vinitialconfig_.size();

    if( configsize % dof ) {
        RAVELOG_ERROR_FORMAT("env=%d, initial config wrong dim: %d %% %d != 0", envId % configsize % dof);
        return false;
    }

    _vecInitialNodes.clear();
    _sampleConfig.resize(dof);
    // TODO perhaps distmetricfn should take into number of revolutions of circular joints
    _treeForward.Init(shared_planner(), dof, params->_distmetricfn, params->_fStepLength, params->_distmetricfn(params->_vConfigLowerLimit, params->_vConfigUpperLimit));
    std::vector<dReal> vinitialconfig(dof);

    for(auto it = begin(vinitialconfig_); it != end(vinitialconfig_); advance(it, dof)) {
        vinitialconfig = std::vector<dReal>(it, it + dof);
        _filterreturn->Clear();
        if( !!params->CheckPathAllConstraints(vinitialconfig, vinitialconfig, {}, {}, 0, IT_OpenStart, CFO_FillCollisionReport, _filterreturn) ) {
            RAVELOG_DEBUG_FORMAT("env=%d, initial configuration for rrt does not satisfy constraints: %s", envId % _filterreturn->_report.__str__());
            continue;
        }
        _vecInitialNodes.push_back(_treeForward.InsertNode(NULL, vinitialconfig, _vecInitialNodes.size()));
    }

    if( _treeForward.empty() && !params->_sampleinitialfn ) {
        RAVELOG_WARN_FORMAT("env=%d, no initial configurations", envId);
        return false;
    }

    return true;
}

/// \brief simple path optimization given a path of dof values. Every _parameters->GetDOF() are one point in the path
void RrtPlanner::_SimpleOptimizePath(std::deque<dReal>& path, int niter)
{
    const PlannerParametersConstPtr params = this->GetParameters();
    const int dof = params->GetDOF();
    const int npath = path.size();
    if( npath <= 2 * dof ) {
        return;
    }

    if( !_filterreturn ) {
        _filterreturn.reset(new ConstraintFilterReturn());
    }

    std::vector<dReal> vstart(dof), vend(dof), vdiff0(dof), vdiff1(dof);
    for(int curiter = niter, nrejected = 0; curiter > 0 && nrejected < npath + 4 * dof; --curiter, ++nrejected) {
        // pick a random node on the path, and a random jump ahead
        // e.g. npath=102, dof=6, npath/dof-2=15, endIndex in 2+[0,14]=[2, 16], 0<=startIndex<=endIndex-2
        const int endIndex = 2 + (_uniformsampler->SampleSequenceOneUInt32() % (npath / dof - 2));
        const int startIndex = _uniformsampler->SampleSequenceOneUInt32() % (endIndex - 1);
        const int dist = endIndex - startIndex;

        std::deque<dReal>::const_iterator itstart = path.begin();
        advance(itstart, startIndex * dof);
        std::deque<dReal>::const_iterator itend = itstart;
        advance(itend, dist * dof); ///< itend = begin(path) + endIndex*dof

        // check if the nodes are in a straight line and if yes, then check different node
        vstart = std::vector<dReal>(itstart, itstart + dof); //?? std::copy(itstart, itstart+dof, vstart.begin());
        vdiff0 = vend = std::vector<dReal>(itend - dof, itend); //?? // std::copy(itend-dof, itend, vend.begin());
        params->_diffstatefn(vdiff0, vstart);

        // take the midpoint since that is the most stable and check if it is collinear
        const std::deque<dReal>::const_iterator itconfig = itstart + dist / 2 * dof;
        vdiff1 = std::vector<dReal>(itconfig, itconfig + dof);
        params->_diffstatefn(vdiff1, vstart);

        dReal dotproduct = 0, x0length2 = 0, x1length2 = 0;
        for(int idof = 0; idof < dof; ++idof) {
            dotproduct += vdiff0[idof] * vdiff1[idof];
            x0length2  += vdiff0[idof] * vdiff0[idof];
            x1length2  += vdiff1[idof] * vdiff1[idof];
        }
        if( RaveFabs(dotproduct * dotproduct - x0length2*x1length2) > g_fEpsilonDotProduct ) {
            return; // not colinear
        }
    }
}

bool BirrtPlanner::InitPlan(RobotBasePtr pbase, PlannerParametersConstPtr pparams)
{
    const auto penv = this->GetEnv();
    const int envId = penv->GetId();
    const EnvironmentMutex::scoped_lock lock(penv->GetMutex());
    _parameters.reset(new RRTParameters());
    _parameters->copy(pparams);
    if( !RrtPlanner::_InitPlan(pbase, _parameters) ) { // call base class's method
        _parameters.reset();
        return false;
    }

    _fGoalBiasProb = 0.01;
    const PlannerParameters::StateSaver savestate(_parameters);
    auto pcc = penv->GetCollisionChecker();
    const CollisionOptionsStateSaver optionstate(pcc, pcc->GetCollisionOptions() | CO_ActiveDOFs, false);

    // TODO perhaps distmetricfn should take into number of revolutions of circular joints
    const size_t dof = _parameters->GetDOF();
    _treeBackward.Init(shared_planner(), dof, _parameters->_distmetricfn, _parameters->_fStepLength, 
                       _parameters->_distmetricfn(_parameters->_vConfigLowerLimit, _parameters->_vConfigUpperLimit));

    //read in all goals
    const std::vector<dReal>& vgoalconfig_ = _parameters->vgoalconfig;
    if( ((int) vgoalconfig_.size() % dof) != 0 ) {
        RAVELOG_ERROR_FORMAT("env=%d, BirrtPlanner::InitPlan - Error: goals are improperly specified", envId);
        _parameters.reset();
        return false;
    }

    std::vector<dReal> vgoal(dof);
    _vecGoalNodes.clear();
    _nValidGoals = 0;
    for(auto it = begin(vgoalconfig_); it != end(vgoalconfig_); advance(it, dof)) {
        vgoal = std::vector<dReal>(it, it + dof);
        const int ret = _parameters->CheckPathAllConstraints(vgoal, vgoal, {}, {}, 0, IT_OpenStart);
        if( ret == 0 ) {
            _vecGoalNodes.push_back(_treeBackward.InsertNode(NULL, vgoal, _vecGoalNodes.size()));
            _nValidGoals++;
        }
        else {
            RAVELOG_WARN_FORMAT("env=%d, goal %d fails constraints with 0x%x", envId % (std::distance(begin(vgoalconfig_), it)/dof) % ret);
            if( IS_DEBUGLEVEL(Level_Verbose) ) {
                /*const int ret = */_parameters->CheckPathAllConstraints(vgoal, vgoal, {}, {}, 0, IT_OpenStart);
            }
            _vecGoalNodes.push_back(NULL); // have to push back dummy or else indices will be messed up
        }
    }

    if( _treeBackward.empty() && !_parameters->_samplegoalfn ) {
        RAVELOG_WARN_FORMAT("env=%d, no goals specified", envId);
        _parameters.reset();
        return false;
    }

    if( _parameters->_nMaxIterations <= 0 ) {
        _parameters->_nMaxIterations = 10000;
    }

    _vgoalpaths.clear();
    if( _vgoalpaths.capacity() < _parameters->_minimumgoalpaths ) {
        _vgoalpaths.reserve(_parameters->_minimumgoalpaths);
    }
    RAVELOG_DEBUG_FORMAT("env=%d, BiRRT Planner Initialized, initial=%d, goal=%d, step=%f",
                         envId % _vecInitialNodes.size() % _treeBackward.GetNumNodes() % _parameters->_fStepLength);
    return true;
}

PlannerStatus BirrtPlanner::PlanPath(TrajectoryBasePtr ptraj, int planningoptions)
{
    _goalindex = _startindex = -1;
    const auto penv = GetEnv();
    const int envId = penv->GetId();
    if(!_parameters) {
        return OPENRAVE_PLANNER_STATUS(str(boost::format("env=%d, BirrtPlanner::PlanPath - Error, planner not initialized") % envId), PS_Failed);
    }

    const EnvironmentMutex::scoped_lock lock(penv->GetMutex());
    const uint32_t basetime = utils::GetMilliTime();

    // the main planning loop
    const PlannerParameters::StateSaver savestate(_parameters);
    const auto pcc = penv->GetCollisionChecker();
    const CollisionOptionsStateSaver optionstate(pcc, pcc->GetCollisionOptions() | CO_ActiveDOFs, false);

    SpatialTreeBase* TreeA = &_treeForward;
    SpatialTreeBase* TreeB = &_treeBackward;
    SimpleNodePtr iConnectedA = NULL;
    SimpleNodePtr iConnectedB = NULL;
    int iter = 0;

    bool bSampleGoal = true;
    PlannerProgress progress;
    PlannerAction callbackaction = PA_None;
    while(_vgoalpaths.size() < _parameters->_minimumgoalpaths && iter < 3*_parameters->_nMaxIterations) {
        RAVELOG_VERBOSE_FORMAT("env=%d, iter=%d, forward=%d, backward=%d", 
                               envId % (iter/3) % _treeForward.GetNumNodes() % _treeBackward.GetNumNodes());
        ++iter;

        // have to check callbacks at the beginning since code can continue
        callbackaction = _CallCallbacks(progress);
        if( callbackaction ==  PA_Interrupt ) {
            return OPENRAVE_PLANNER_STATUS(str(boost::format("env=%d, Planning was interrupted")%envId), PS_Interrupted);
        }
        else if( callbackaction == PA_ReturnWithAnySolution ) {
            if( !_vgoalpaths.empty() ) {
                break;
            }
        }

        if( _parameters->_nMaxPlanningTime > 0 ) {
            const uint32_t elapsedtime = utils::GetMilliTime() - basetime;
            if( elapsedtime >= _parameters->_nMaxPlanningTime ) {
                RAVELOG_DEBUG_FORMAT("env=%d, time exceeded (%dms) so breaking. iter=%d < %d",
                                     envId % elapsedtime % (iter/3) % _parameters->_nMaxIterations);
                break;
            }
        }

        if( !!_parameters->_samplegoalfn ) {
            std::vector<dReal> vgoal;
            if( _parameters->_samplegoalfn(vgoal) ) {
                RAVELOG_VERBOSE_FORMAT("env=%d, inserting new goal index %d", envId % _vecGoalNodes.size());
                _vecGoalNodes.push_back(_treeBackward.InsertNode(NULL, vgoal, _vecGoalNodes.size()));
                _nValidGoals++;
            }
        }

        if( !!_parameters->_sampleinitialfn ) {
            std::vector<dReal> vinitial;
            if( _parameters->_sampleinitialfn(vinitial) ) {
                RAVELOG_VERBOSE_FORMAT("env=%d, inserting new initial %d", envId % _vecInitialNodes.size());
                _vecInitialNodes.push_back(_treeForward.InsertNode(NULL, vinitial, _vecInitialNodes.size()));
            }
        }

        _sampleConfig.clear();
        if( (bSampleGoal || _uniformsampler->SampleSequenceOneReal() < _fGoalBiasProb) && _nValidGoals > 0 ) {
            bSampleGoal = false; ///< true when first enters this while loop
            uint32_t bestgoalindex = -1;
            for(size_t testiter = 0; testiter < _vecGoalNodes.size() * 3; ++testiter) {
                const uint32_t goalindex = _uniformsampler->SampleSequenceOneUInt32() % _vecGoalNodes.size();
                if( !_vecGoalNodes.at(goalindex) ) {
                    continue; // dummy: we pushed back NULL before as it violated constraints
                }
                // make sure goal is not already found
                bool bfound = false;
                for(const GOALPATH& goalpath : _vgoalpaths) {
                    bfound = goalindex == (uint32_t)goalpath.goalindex;
                    if( bfound ) {
                        break;
                    }
                }
                if( !bfound) {
                    bestgoalindex = goalindex;
                    break;
                }
            }
            if( bestgoalindex != uint32_t(-1) ) {
                _treeBackward.GetVectorConfig(_vecGoalNodes.at(bestgoalindex), _sampleConfig);
            }
        }

        if( _sampleConfig.empty() ) {
            if( !_parameters->_samplefn(_sampleConfig) ) {
                continue;
            }
        }

        // extend A
        ExtendType et = TreeA->Extend(_sampleConfig, iConnectedA);

        // although check isn't necessary, having it improves running times
        if( et == ET_Failed ) {
            // necessary to increment iterator in case spaces are not connected
            if( iter > 3*_parameters->_nMaxIterations ) {
                RAVELOG_WARN_FORMAT("env=%d, iterations exceeded", envId);
                break;
            }
            continue;
        }

        et = TreeB->Extend(TreeA->GetVectorConfig(iConnectedA), iConnectedB);     // extend B toward A

        if( et == ET_Connected ) {
            // connected, process goal
            _vgoalpaths.emplace_back();
            GOALPATH& lastpath = _vgoalpaths.back();
            (TreeA == &_treeForward) ? _ExtractPath(lastpath, iConnectedA, iConnectedB)
                                     : _ExtractPath(lastpath, iConnectedB, iConnectedA);
            const int goalindex = lastpath.goalindex;
            const int startindex = lastpath.startindex;
            if( IS_DEBUGLEVEL(Level_Debug) ) {
                std::stringstream ss;
                ss << std::setprecision(std::numeric_limits<dReal>::digits10+1);
                ss << "env=" << envId << ", found a goal, start index=" << startindex << " goal index=" << goalindex << ", path length=" << lastpath.length << ", startvalues=[";
                const int dof = _parameters->GetDOF();
                for(int i = 0; i < dof; ++i) {
                    ss << lastpath.qall.at(i) << ", ";
                }
                ss << "]; goalvalues=[";
                for(int i = 0; i < dof; ++i) {
                    ss << lastpath.qall.at(lastpath.qall.size()-dof+i) << ", ";
                }
                ss << "];";
                RAVELOG_DEBUG(ss.str());
            }
            if( _vgoalpaths.size() >= _parameters->_minimumgoalpaths || _vgoalpaths.size() >= _nValidGoals ) {
                break;
            }
            bSampleGoal = true;
            // more goals requested, so make sure to remove all the nodes pointing to the current found goal
            _treeBackward.InvalidateNodesWithParent(_vecGoalNodes.at(goalindex));
        }

        std::swap(TreeA, TreeB);
        iter += 3;
        if( iter > 3*_parameters->_nMaxIterations ) {
            RAVELOG_WARN_FORMAT("env=%d, iterations exceeded %d", envId%_parameters->_nMaxIterations);
            break;
        }

        progress._iteration = iter/3;
    }

    if( _vgoalpaths.empty() ) {
        const std::string description = str(boost::format(_("env=%d, plan failed in %fs, iter=%d, nMaxIterations=%d"))
                                            % envId % (0.001*(utils::GetMilliTime()-basetime)) % (iter/3) % _parameters->_nMaxIterations);
        RAVELOG_WARN(description);
        return OPENRAVE_PLANNER_STATUS(description, PS_Failed);
    }

    size_t ibestpath = 0;
    for(size_t ipath = 0; ipath < _vgoalpaths.size(); ++ipath) {
        if(_vgoalpaths[ipath].length < _vgoalpaths[ibestpath].length) {
            ibestpath = ipath;
        }
    }
    const GOALPATH& bestpath = _vgoalpaths.at(ibestpath);
    _goalindex = bestpath.goalindex;
    _startindex = bestpath.startindex;
    if( ptraj->GetConfigurationSpecification().GetDOF() == 0 ) {
        ptraj->Init(_parameters->_configurationspecification);
    }
    ptraj->Insert(ptraj->GetNumWaypoints(), bestpath.qall, _parameters->_configurationspecification);
    const std::string description = str(boost::format(_("env=%d, plan success, iters=%d, path=%d points, computation time=%fs\n"))
                                        % envId % progress._iteration % ptraj->GetNumWaypoints() % (0.001*(utils::GetMilliTime()-basetime)));
    RAVELOG_DEBUG(description);
    PlannerStatus status = _ProcessPostPlanners(_robot,ptraj);
    //TODO should use accessor to change description
    status.description = description;
    return status;
}

void BirrtPlanner::_ExtractPath(GOALPATH& goalpath, SimpleNodePtr pforward, SimpleNodePtr pbackward)
{
    const int dof = _parameters->GetDOF();
    _cachedpath.clear();

    // add nodes from the forward tree
    _cachedpath.insert(_cachedpath.begin(), pforward->q, pforward->q+dof);
    while(pforward->rrtparent) {
        pforward = pforward->rrtparent;
        _cachedpath.insert(_cachedpath.begin(), pforward->q, pforward->q+dof);
    }
    goalpath.startindex = pforward->_userdata;

    // add nodes from the backward tree
    _cachedpath.insert(_cachedpath.end(), pbackward->q, pbackward->q+dof);
    while(pbackward->rrtparent) {
        pbackward = pbackward->rrtparent;
        _cachedpath.insert(_cachedpath.end(), pbackward->q, pbackward->q+dof);
    }
    goalpath.goalindex = pbackward->_userdata;

    std::stringstream ss;
    ss << std::setprecision(16);
    ss << "jvals = [";
    for(auto it = _cachedpath.begin(); it != _cachedpath.end(); advance(it, dof)) {
        auto jt = it;
        ss << "[";
        for(int i = 0; i < dof; ++i, ++jt) {
            ss << *jt << ", ";
        }
        ss << "], " << std::endl;
    }
    ss << "]" << std::endl;
    RAVELOG_WARN_FORMAT("%s", ss.str());

    BOOST_ASSERT( goalpath.goalindex >= 0 && goalpath.goalindex < (int)_vecGoalNodes.size() );
    _SimpleOptimizePath(_cachedpath,10);

    std::vector<dReal>& qall = goalpath.qall;
    qall = std::vector<dReal>(begin(_cachedpath), end(_cachedpath));
    goalpath.length = 0;
    std::vector<dReal> vivel(dof);
    for(size_t i = 0; i < dof; ++i) {
        const dReal veli = _parameters->_vConfigVelocityLimit.at(i);
        vivel[i] = (veli == 0.0) ? 1.0 : (1.0/veli);
    }

    // take distance scaled with respect to velocities with the first and last points only!
    // this is because rrt paths can initially be very complex but simplify down to something simpler.
    std::vector<dReal> vdiff(qall.begin(), qall.begin() + dof);
    const std::vector<dReal> lastpoint(qall.end() - dof, qall.end());
    _parameters->_diffstatefn(vdiff, lastpoint);
    for(size_t i = 0; i < dof; ++i) {
        goalpath.length += RaveFabs(vdiff[i])*vivel[i];
    }
}
