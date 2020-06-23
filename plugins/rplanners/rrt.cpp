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

RrtPlanner::RrtPlanner(EnvironmentBasePtr penv) : PlannerBase(penv), _treeForward(0)
{
    __description = "\
:Interface Author:  Rosen Diankov\n\n\
Uses the Rapidly-Exploring Random Trees Algorithm.\n\
";
    RegisterCommand("GetGoalIndex",boost::bind(&RrtPlanner::GetGoalIndexCommand,this,_1,_2),
                    "returns the goal index of the plan");
    RegisterCommand("GetInitGoalIndices",boost::bind(&RrtPlanner::GetInitGoalIndicesCommand,this,_1,_2),
                    "returns the start and goal indices");
    _filterreturn.reset(new ConstraintFilterReturn());
}

bool RrtPlanner::_InitPlan(RobotBasePtr pbase, PlannerParametersPtr params)
{
    params->Validate();
    _goalindex = -1;
    _startindex = -1;
    EnvironmentMutex::scoped_lock lock(GetEnv()->GetMutex());
    if( !_uniformsampler ) {
        _uniformsampler = RaveCreateSpaceSampler(GetEnv(),"mt19937");
    }
    _robot = pbase;

    _uniformsampler->SetSeed(params->_nRandomGeneratorSeed);
    FOREACH(it, params->_listInternalSamplers) {
        (*it)->SetSeed(params->_nRandomGeneratorSeed);
    }

    PlannerParameters::StateSaver savestate(params);
    CollisionOptionsStateSaver optionstate(GetEnv()->GetCollisionChecker(),GetEnv()->GetCollisionChecker()->GetCollisionOptions()|CO_ActiveDOFs,false);

    if( (int)params->vinitialconfig.size() % params->GetDOF() ) {
        RAVELOG_ERROR_FORMAT("env=%d, initial config wrong dim: %d %% %d != 0", GetEnv()->GetId()%params->vinitialconfig.size()%params->GetDOF());
        return false;
    }

    _vecInitialNodes.clear();
    _sampleConfig.resize(params->GetDOF());
    // TODO perhaps distmetricfn should take into number of revolutions of circular joints
    _treeForward.Init(shared_planner(), params->GetDOF(), params->_distmetricfn, params->_fStepLength, params->_distmetricfn(params->_vConfigLowerLimit, params->_vConfigUpperLimit));
    std::vector<dReal> vinitialconfig(params->GetDOF());
    for(size_t index = 0; index < params->vinitialconfig.size(); index += params->GetDOF()) {
        std::copy(params->vinitialconfig.begin()+index,params->vinitialconfig.begin()+index+params->GetDOF(),vinitialconfig.begin());
        _filterreturn->Clear();
        if( params->CheckPathAllConstraints(vinitialconfig,vinitialconfig, {}, {}, 0, IT_OpenStart, CFO_FillCollisionReport, _filterreturn) != 0 ) {
            RAVELOG_DEBUG_FORMAT("env=%d, initial configuration for rrt does not satisfy constraints: %s", GetEnv()->GetId()%_filterreturn->_report.__str__());
            continue;
        }
        _vecInitialNodes.push_back(_treeForward.InsertNode(NULL, vinitialconfig, _vecInitialNodes.size()));
    }

    if( _treeForward.GetNumNodes() == 0 && !params->_sampleinitialfn ) {
        RAVELOG_WARN_FORMAT("env=%d, no initial configurations", GetEnv()->GetId());
        return false;
    }

    return true;
}

/// \brief simple path optimization given a path of dof values. Every _parameters->GetDOF() are one point in the path
void RrtPlanner::_SimpleOptimizePath(std::deque<dReal>& path, int numiterations)
{
    PlannerParametersConstPtr params = this->GetParameters();
    const int dof = params->GetDOF();
    if( (int)path.size() <= 2*dof ) {
        return;
    }

    std::deque<dReal>::iterator startNode, endNode, itconfig;
    if( !_filterreturn ) {
        _filterreturn.reset(new ConstraintFilterReturn());
    }
    int nrejected = 0;
    int curiter = numiterations;
    std::vector<dReal> vstart(dof), vend(dof), vdiff0(dof), vdiff1(dof);
    while(curiter > 0 && nrejected < (int)path.size()+4*dof ) {
        --curiter;

        // pick a random node on the path, and a random jump ahead
        const int endIndex = 2+(_uniformsampler->SampleSequenceOneUInt32()%((int)path.size()/dof-2));
        const int startIndex = _uniformsampler->SampleSequenceOneUInt32()%(endIndex-1);

        startNode = path.begin();
        advance(startNode, startIndex*dof);
        endNode = startNode;
        advance(endNode, (endIndex-startIndex)*dof);
        nrejected++;

        // check if the nodes are in a straight line and if yes, then check different node
        std::copy(startNode, startNode+dof, vstart.begin());
        std::copy(endNode-dof, endNode, vend.begin());
        vdiff0 = vend;
        params->_diffstatefn(vdiff0, vstart);
        bool bcolinear = true;
        // take the midpoint since that is the most stable and check if it is collinear
        {
            itconfig = startNode + ((endIndex-startIndex)/2)*dof;
            std::copy(itconfig, itconfig+dof, vdiff1.begin());
            params->_diffstatefn(vdiff1, vstart);
            dReal dotproduct=0,x0length2=0,x1length2=0;
            for(int idof = 0; idof < dof; ++idof) {
                dotproduct += vdiff0[idof]*vdiff1[idof];
                x0length2 += vdiff0[idof]*vdiff0[idof];
                x1length2 += vdiff1[idof]*vdiff1[idof];
            }
            if( RaveFabs(dotproduct * dotproduct - x0length2*x1length2) > g_fEpsilonDotProduct ) {
                //RAVELOG_INFO_FORMAT("env=%d, colinear: %.15e, %.15e", GetEnv()->GetId()%RaveFabs(dotproduct * dotproduct - x0length2*x1length2)%(dotproduct/RaveSqrt(x0length2*x1length2)));
                bcolinear = false;
                break;
            }
        }

        if( bcolinear ) {
            continue;
        }

        // check if the nodes can be connected by a straight line
        _filterreturn->Clear();
        if ( params->CheckPathAllConstraints(vstart, vend, {}, {}, 0, IT_Open, 0xffff|CFO_FillCheckedConfiguration, _filterreturn) != 0 ) {
            if( nrejected++ > (int)path.size()+8 ) {
                break;
            }
            continue;
        }

        startNode += dof;
        OPENRAVE_ASSERT_OP(_filterreturn->_configurations.size()%dof,==,0);
        // need to copy _filterreturn->_configurations between startNode and endNode
        size_t ioffset=endNode-startNode;
        if( ioffset > 0 ) {
            if( ioffset <= _filterreturn->_configurations.size() ) {
                std::copy(_filterreturn->_configurations.begin(), _filterreturn->_configurations.begin()+ioffset, startNode);
            }
            else {
                std::copy(_filterreturn->_configurations.begin(), _filterreturn->_configurations.end(), startNode);
                // have to remove nodes
                path.erase(startNode+_filterreturn->_configurations.size(), endNode);
            }
        }
        if( ioffset < _filterreturn->_configurations.size() ) {
            // insert the rest of the continue
            path.insert(endNode, _filterreturn->_configurations.begin()+ioffset, _filterreturn->_configurations.end());
        }

        nrejected = 0;
        if( (int)path.size() <= 2*dof ) {
            return;
        }
    }
}

bool RrtPlanner::GetGoalIndexCommand(std::ostream& os, std::istream& is)
{
    os << _goalindex;
    return !!os;
}

bool RrtPlanner::GetInitGoalIndicesCommand(std::ostream& os, std::istream& is)
{
    os << _startindex << " " << _goalindex;
    return !!os;
}

RrtPlannerPtr RrtPlanner::shared_planner() {
    return boost::static_pointer_cast<RrtPlanner>(shared_from_this());
}

RrtPlannerConstPtr RrtPlanner::shared_planner_const() const {
    return boost::static_pointer_cast<RrtPlanner const>(shared_from_this());
}

BirrtPlanner::BirrtPlanner(EnvironmentBasePtr penv) : RrtPlanner(penv), _treeBackward(1)
{
    __description += "Bi-directional RRTs. See\n\n\
- J.J. Kuffner and S.M. LaValle. RRT-Connect: An efficient approach to single-query path planning. In Proc. IEEE Int'l Conf. on Robotics and Automation (ICRA'2000), pages 995-1001, San Francisco, CA, April 2000.";
    RegisterCommand("DumpTree", boost::bind(&BirrtPlanner::_DumpTreeCommand,this,_1,_2),
                    "dumps the source and goal trees to $OPENRAVE_HOME/birrtdump.txt. The first N values are the DOF values, the last value is the parent index.\n\
Some python code to display data::\n\
\n\
sourcetree=loadtxt(os.path.join(RaveGetHomeDirectory(),'sourcetree.txt'),delimiter=' ,')\n\
hs=env.plot3(sourcetree,5,[1,0,0])\n\
sourcedist = abs(sourcetree[:,0]-x[0]) + abs(sourcetree[:,1]-x[1])\n\
robot.SetActiveDOFValues(sourcetree[argmin(sourcedist)])\n\
\n\
");
    _nValidGoals = 0;
}

bool BirrtPlanner::InitPlan(RobotBasePtr pbase, PlannerParametersConstPtr pparams)
{
    EnvironmentMutex::scoped_lock lock(GetEnv()->GetMutex());
    _parameters.reset(new RRTParameters());
    _parameters->copy(pparams);
    if( !RrtPlanner::_InitPlan(pbase,_parameters) ) {
        _parameters.reset();
        return false;
    }

    _fGoalBiasProb = 0.01;
    PlannerParameters::StateSaver savestate(_parameters);
    CollisionOptionsStateSaver optionstate(GetEnv()->GetCollisionChecker(),GetEnv()->GetCollisionChecker()->GetCollisionOptions()|CO_ActiveDOFs,false);

    // TODO perhaps distmetricfn should take into number of revolutions of circular joints
    const size_t dof = _parameters->GetDOF();
    _treeBackward.Init(shared_planner(), dof, _parameters->_distmetricfn, _parameters->_fStepLength, _parameters->_distmetricfn(_parameters->_vConfigLowerLimit, _parameters->_vConfigUpperLimit));

    //read in all goals
    if( (_parameters->vgoalconfig.size() % dof) != 0 ) {
        RAVELOG_ERROR_FORMAT("env=%d, BirrtPlanner::InitPlan - Error: goals are improperly specified", GetEnv()->GetId());
        _parameters.reset();
        return false;
    }

    std::vector<dReal> vgoal(dof);
    _vecGoalNodes.clear();
    _nValidGoals = 0;
    for(size_t igoal = 0; igoal < _parameters->vgoalconfig.size(); igoal += dof) {
        std::copy(_parameters->vgoalconfig.begin()+igoal,_parameters->vgoalconfig.begin()+igoal+dof,vgoal.begin());
        int ret = _parameters->CheckPathAllConstraints(vgoal,vgoal,{}, {}, 0, IT_OpenStart);
        if( ret == 0 ) {
            _vecGoalNodes.push_back(_treeBackward.InsertNode(NULL, vgoal, _vecGoalNodes.size()));
            _nValidGoals++;
        }
        else {
            RAVELOG_WARN_FORMAT("env=%d, goal %d fails constraints with 0x%x", GetEnv()->GetId()%igoal%ret);
            if( IS_DEBUGLEVEL(Level_Verbose) ) {
                int ret = _parameters->CheckPathAllConstraints(vgoal,vgoal,{}, {}, 0, IT_OpenStart);
            }
            _vecGoalNodes.push_back(NULL); // have to push back dummy or else indices will be messed up
        }
    }

    if( _treeBackward.GetNumNodes() == 0 && !_parameters->_samplegoalfn ) {
        RAVELOG_WARN_FORMAT("env=%d, no goals specified", GetEnv()->GetId());
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
    RAVELOG_DEBUG_FORMAT("env=%d, BiRRT Planner Initialized, initial=%d, goal=%d, step=%f", GetEnv()->GetId()%_vecInitialNodes.size()%_treeBackward.GetNumNodes()%_parameters->_fStepLength);
    return true;
}

PlannerStatus BirrtPlanner::PlanPath(TrajectoryBasePtr ptraj, int planningoptions)
{
    _goalindex = -1;
    _startindex = -1;
    if(!_parameters) {
        return OPENRAVE_PLANNER_STATUS(str(boost::format("env=%d, BirrtPlanner::PlanPath - Error, planner not initialized")%GetEnv()->GetId()), PS_Failed);
    }

    EnvironmentMutex::scoped_lock lock(GetEnv()->GetMutex());
    uint32_t basetime = utils::GetMilliTime();

    // the main planning loop
    PlannerParameters::StateSaver savestate(_parameters);
    CollisionOptionsStateSaver optionstate(GetEnv()->GetCollisionChecker(),GetEnv()->GetCollisionChecker()->GetCollisionOptions()|CO_ActiveDOFs,false);

    SpatialTreeBase* TreeA = &_treeForward;
    SpatialTreeBase* TreeB = &_treeBackward;
    SimpleNodePtr iConnectedA = NULL;
    SimpleNodePtr iConnectedB = NULL;
    int iter = 0;

    bool bSampleGoal = true;
    PlannerProgress progress;
    PlannerAction callbackaction=PA_None;
    while(_vgoalpaths.size() < _parameters->_minimumgoalpaths && iter < 3*_parameters->_nMaxIterations) {
        RAVELOG_VERBOSE_FORMAT("env=%d, iter=%d, forward=%d, backward=%d", GetEnv()->GetId()%(iter/3)%_treeForward.GetNumNodes()%_treeBackward.GetNumNodes());
        ++iter;

        // have to check callbacks at the beginning since code can continue
        callbackaction = _CallCallbacks(progress);
        if( callbackaction ==  PA_Interrupt ) {
            return OPENRAVE_PLANNER_STATUS(str(boost::format("env=%d, Planning was interrupted")%GetEnv()->GetId()), PS_Interrupted);
        }
        else if( callbackaction == PA_ReturnWithAnySolution ) {
            if( !_vgoalpaths.empty() ) {
                break;
            }
        }

        if( _parameters->_nMaxPlanningTime > 0 ) {
            uint32_t elapsedtime = utils::GetMilliTime()-basetime;
            if( elapsedtime >= _parameters->_nMaxPlanningTime ) {
                RAVELOG_DEBUG_FORMAT("env=%d, time exceeded (%dms) so breaking. iter=%d < %d", GetEnv()->GetId()%elapsedtime%(iter/3)%_parameters->_nMaxIterations);
                break;
            }
        }


        if( !!_parameters->_samplegoalfn ) {
            vector<dReal> vgoal;
            if( _parameters->_samplegoalfn(vgoal) ) {
                RAVELOG_VERBOSE(str(boost::format("env=%d, inserting new goal index %d")%GetEnv()->GetId()%_vecGoalNodes.size()));
                _vecGoalNodes.push_back(_treeBackward.InsertNode(NULL, vgoal, _vecGoalNodes.size()));
                _nValidGoals++;
            }
        }
        if( !!_parameters->_sampleinitialfn ) {
            vector<dReal> vinitial;
            if( _parameters->_sampleinitialfn(vinitial) ) {
                RAVELOG_VERBOSE(str(boost::format("env=%d, inserting new initial %d")%GetEnv()->GetId()%_vecInitialNodes.size()));
                _vecInitialNodes.push_back(_treeForward.InsertNode(NULL,vinitial, _vecInitialNodes.size()));
            }
        }

        _sampleConfig.clear();
        if( (bSampleGoal || _uniformsampler->SampleSequenceOneReal() < _fGoalBiasProb) && _nValidGoals > 0 ) {
            bSampleGoal = false;
            // sample goal as early as possible
            uint32_t bestgoalindex = -1;
            for(size_t testiter = 0; testiter < _vecGoalNodes.size()*3; ++testiter) {
                const uint32_t sampleindex = _uniformsampler->SampleSequenceOneUInt32();
                const uint32_t goalindex = sampleindex % _vecGoalNodes.size();
                if( !_vecGoalNodes.at(goalindex) ) {
                    continue; // dummy
                }
                // make sure goal is not already found
                bool bfound = false;
                FOREACHC(itgoalpath,_vgoalpaths) {
                    if( goalindex == (uint32_t)itgoalpath->goalindex ) {
                        bfound = true;
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
                RAVELOG_WARN_FORMAT("env=%d, iterations exceeded", GetEnv()->GetId());
                break;
            }
            continue;
        }

        et = TreeB->Extend(TreeA->GetVectorConfig(iConnectedA), iConnectedB);     // extend B toward A

        if( et == ET_Connected ) {
            // connected, process goal
            _vgoalpaths.emplace_back();
            GOALPATH& lastpath = _vgoalpaths.back();
            if(TreeA == &_treeForward) {
                _ExtractPath(lastpath, iConnectedA, iConnectedB);
            }
            else {
                _ExtractPath(lastpath, iConnectedB, iConnectedA);
            }
            const int goalindex = lastpath.goalindex;
            const int startindex = lastpath.startindex;
            if( IS_DEBUGLEVEL(Level_Debug) ) {
                std::stringstream ss;
                ss << std::setprecision(std::numeric_limits<dReal>::digits10+1);
                ss << "env=" << GetEnv()->GetId() << ", found a goal, start index=" << startindex << " goal index=" << goalindex << ", path length=" << lastpath.length << ", startvalues=[";
                for(int i = 0; i < _parameters->GetDOF(); ++i) {
                    ss << lastpath.qall.at(i) << ", ";
                }
                ss << "]; goalvalues=[";
                for(int i = 0; i < _parameters->GetDOF(); ++i) {
                    ss << lastpath.qall.at(lastpath.qall.size()-_parameters->GetDOF()+i) << ", ";
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

        swap(TreeA, TreeB);
        iter += 3;
        if( iter > 3*_parameters->_nMaxIterations ) {
            RAVELOG_WARN_FORMAT("env=%d, iterations exceeded %d", GetEnv()->GetId()%_parameters->_nMaxIterations);
            break;
        }

        progress._iteration = iter/3;
    }

    if( _vgoalpaths.empty() ) {
        std::string description = str(boost::format(_("env=%d, plan failed in %fs, iter=%d, nMaxIterations=%d"))%GetEnv()->GetId()%(0.001f*(float)(utils::GetMilliTime()-basetime))%(iter/3)%_parameters->_nMaxIterations);
        RAVELOG_WARN(description);
        return OPENRAVE_PLANNER_STATUS(description, PS_Failed);
    }

    // std::vector<GOALPATH>::iterator itbest = _vgoalpaths.begin();
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
    const std::string description = str(boost::format(_("env=%d, plan success, iters=%d, path=%d points, computation time=%fs\n"))%GetEnv()->GetId()%progress._iteration%ptraj->GetNumWaypoints()%(0.001f*(float)(utils::GetMilliTime()-basetime)));
    RAVELOG_DEBUG(description);
    PlannerStatus status = _ProcessPostPlanners(_robot,ptraj);
    //TODO should use accessor to change description
    status.description = description;
    return status;
}

void BirrtPlanner::_ExtractPath(GOALPATH& goalpath, SimpleNodePtr iConnectedForward, SimpleNodePtr iConnectedBackward)
{
    const int dof = _parameters->GetDOF();
    _cachedpath.clear();

    // add nodes from the forward tree
    SimpleNodePtr pforward = iConnectedForward;
    goalpath.startindex = -1;
    while(true) {
        _cachedpath.insert(_cachedpath.begin(), pforward->q, pforward->q+dof);
        //vecnodes.push_front(pforward);
        if(!pforward->rrtparent) {
            goalpath.startindex = pforward->_userdata;
            break;
        }
        pforward = pforward->rrtparent;
    }

    // add nodes from the backward tree
    goalpath.goalindex = -1;
    SimpleNodePtr pbackward = iConnectedBackward;
    while(true) {
        //vecnodes.push_back(pbackward);
        _cachedpath.insert(_cachedpath.end(), pbackward->q, pbackward->q+dof);
        if(!pbackward->rrtparent) {
            goalpath.goalindex = pbackward->_userdata;
            break;
        }
        pbackward = pbackward->rrtparent;
    }

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
    goalpath.qall.resize(_cachedpath.size());
    std::copy(_cachedpath.begin(), _cachedpath.end(), goalpath.qall.begin());
    goalpath.length = 0;
    std::vector<dReal> vivel(dof,1.0);
    for(size_t i = 0; i < dof; ++i) {
        if( _parameters->_vConfigVelocityLimit.at(i) != 0 ) {
            vivel[i] = 1.0/_parameters->_vConfigVelocityLimit[i];
        }
    }

    // take distance scaled with respect to velocities with the first and last points only!
    // this is because rrt paths can initially be very complex but simplify down to something simpler.
    std::vector<dReal> vdiff(goalpath.qall.begin(), goalpath.qall.begin()+dof);
    _parameters->_diffstatefn(vdiff, std::vector<dReal>(goalpath.qall.end()-dof, goalpath.qall.end()));
    for(size_t i = 0; i < dof; ++i) {
        goalpath.length += RaveFabs(vdiff[i])*vivel[i];
    }
}

bool BirrtPlanner::_DumpTreeCommand(std::ostream& os, std::istream& is) {
    std::string filename = RaveGetHomeDirectory() + string("/birrtdump.txt");
    getline(is, filename);
    boost::trim(filename);
    RAVELOG_VERBOSE_FORMAT("dumping rrt tree to %s", filename);
    ofstream f(filename.c_str());
    f << std::setprecision(std::numeric_limits<dReal>::digits10+1);
    _treeForward.DumpTree(f);
    _treeBackward.DumpTree(f);
    return true;
}
