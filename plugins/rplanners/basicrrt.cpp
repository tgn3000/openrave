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

BasicRrtPlanner::BasicRrtPlanner(EnvironmentBasePtr penv) : RrtPlanner(penv)
{
    __description = "Rosen's Basic RRT planner";
    _fGoalBiasProb = dReal(0.05);
    _bOneStep = false;
    RegisterCommand("DumpTree", boost::bind(&BasicRrtPlanner::_DumpTreeCommand,this,_1,_2),
                    "dumps the source and goal trees to $OPENRAVE_HOME/basicrrtdump.txt. The first N values are the DOF values, the last value is the parent index.\n");
}

bool BasicRrtPlanner::InitPlan(RobotBasePtr pbase, PlannerParametersConstPtr pparams)
{
    EnvironmentMutex::scoped_lock lock(GetEnv()->GetMutex());
    _parameters.reset(new BasicRRTParameters());
    _parameters->copy(pparams);
    if( !RrtPlanner::_InitPlan(pbase,_parameters) ) {
        _parameters.reset();
        return false;
    }

    CollisionOptionsStateSaver optionstate(GetEnv()->GetCollisionChecker(),GetEnv()->GetCollisionChecker()->GetCollisionOptions()|CO_ActiveDOFs,false);

    //read in all goals
    int goal_index = 0;
    vector<dReal> vgoal(_parameters->GetDOF());
    _vecGoals.clear();
    while(!_parameters->vgoalconfig.empty()) {
        for(int i = 0; i < _parameters->GetDOF(); i++) {
            if(goal_index < (int)_parameters->vgoalconfig.size())
                vgoal[i] = _parameters->vgoalconfig[goal_index];
            else {
                RAVELOG_ERROR_FORMAT("env=%d, BasicRrtPlanner::InitPlan - Error: goals are improperly specified", GetEnv()->GetId());
                _parameters.reset();
                return false;
            }
            goal_index++;
        }

        if( GetParameters()->CheckPathAllConstraints(vgoal,vgoal, {}, {}, 0, IT_OpenStart) == 0 ) {
            _vecGoals.push_back(vgoal);
        }
        else {
            RAVELOG_WARN_FORMAT("env=%d, goal in collision", GetEnv()->GetId());
        }

        if(goal_index == (int)_parameters->vgoalconfig.size()) {
            break;
        }
    }

    if(( _vecGoals.empty()) && !_parameters->_goalfn ) {
        RAVELOG_WARN_FORMAT("env=%d, no goals or goal function specified", GetEnv()->GetId());
        _parameters.reset();
        return false;
    }

    _bOneStep = _parameters->_nRRTExtentType == 1;
    RAVELOG_DEBUG_FORMAT("env=%d, BasicRrtPlanner initialized _nRRTExtentType=%d", GetEnv()->GetId()%_parameters->_nRRTExtentType);
    return true;
}

PlannerStatus BasicRrtPlanner::PlanPath(TrajectoryBasePtr ptraj, int planningoptions)
{
    if(!_parameters) {
        std::string description = str(boost::format("env=%d, BasicRrtPlanner::PlanPath - Error, planner not initialized")%GetEnv()->GetId());
        RAVELOG_WARN(description);
        return OPENRAVE_PLANNER_STATUS(description, PS_Failed);
    }

    EnvironmentMutex::scoped_lock lock(GetEnv()->GetMutex());
    uint32_t basetime = utils::GetMilliTime();

    SimpleNodePtr lastnode; // the last node visited by the RRT
    SimpleNodePtr bestGoalNode = NULL; // the best goal node found already by the RRT. If this is not NULL, then RRT succeeded
    dReal fBestGoalNodeDist = 0; // configuration distance from initial position to the goal node

    // the main planning loop
    PlannerParameters::StateSaver savestate(_parameters);
    CollisionOptionsStateSaver optionstate(GetEnv()->GetCollisionChecker(),GetEnv()->GetCollisionChecker()->GetCollisionOptions()|CO_ActiveDOFs,false);

    std::vector<dReal> vtempinitialconfig;
    PlannerAction callbackaction = PA_None;
    PlannerProgress progress;
    int iter = 0;
    _goalindex = -1; // index into vgoalconfig if the goal is found
    _startindex = -1;

    int numfoundgoals = 0;

    while(iter < _parameters->_nMaxIterations) {
        iter++;
        if( !!bestGoalNode && iter >= _parameters->_nMinIterations ) {
            break;
        }
        if( !!_parameters->_samplegoalfn ) {
            vector<dReal> vgoal;
            if( _parameters->_samplegoalfn(vgoal) ) {
                RAVELOG_VERBOSE_FORMAT("env=%d, found goal", GetEnv()->GetId());
                _vecGoals.push_back(vgoal);
            }
        }
        if( !!_parameters->_sampleinitialfn ) {
            vector<dReal> vinitial;
            if( _parameters->_sampleinitialfn(vinitial) ) {
                RAVELOG_VERBOSE_FORMAT("env=%d, found initial", GetEnv()->GetId());
                _vecInitialNodes.push_back(_treeForward.InsertNode(NULL, vinitial, _vecInitialNodes.size()));
            }
        }

        if( (iter == 1 || RaveRandomFloat() < _fGoalBiasProb ) && !_vecGoals.empty() ) {
            _sampleConfig = _vecGoals[RaveRandomInt()%_vecGoals.size()];
        }
        else if( !_parameters->_samplefn(_sampleConfig) ) {
            continue;
        }

        // extend A
        ExtendType et = _treeForward.Extend(_sampleConfig, lastnode, _bOneStep);

        if( et == ET_Connected ) {
            FOREACH(itgoal, _vecGoals) {
                if( _parameters->_distmetricfn(*itgoal, _treeForward.GetVectorConfig(lastnode)) < 2*_parameters->_fStepLength ) {
                    SimpleNodePtr pforward = lastnode;
                    while(1) {
                        if(!pforward->rrtparent) {
                            break;
                        }
                        pforward = pforward->rrtparent;
                    }
                    vtempinitialconfig = _treeForward.GetVectorConfig(pforward); // GetVectorConfig returns the same reference, so need to make a copy
                    dReal fGoalNodeDist = _parameters->_distmetricfn(vtempinitialconfig, _treeForward.GetVectorConfig(lastnode));
                    if( !bestGoalNode || fBestGoalNodeDist > fGoalNodeDist ) {
                        bestGoalNode = lastnode;
                        fBestGoalNodeDist = fGoalNodeDist;
                        _goalindex = (int)(itgoal-_vecGoals.begin());
                    }
                    if( iter >= _parameters->_nMinIterations ) {
                        RAVELOG_DEBUG_FORMAT("env=%d, found goal index: %d", GetEnv()->GetId()%_goalindex);
                        break;
                    }
                }
            }
        }

        // check the goal heuristic more often
        if(( et != ET_Failed) && !!_parameters->_goalfn ) {
            // have to check all the newly created nodes since anyone could be already in the goal (do not have to do this with _vecGoals since that is being sampled)
            bool bfound = false;
            SimpleNodePtr ptestnode = lastnode;
            while(!!ptestnode && ptestnode->_userdata==0) { // when userdata is 0, then it hasn't been checked for goal yet
                if( _parameters->_goalfn(_treeForward.GetVectorConfig(ptestnode)) <= 1e-4f ) {
                    bfound = true;
                    numfoundgoals++;
                    ptestnode->_userdata = 1;
                    SimpleNodePtr pforward = ptestnode;
                    while(1) {
                        if(!pforward->rrtparent) {
                            break;
                        }
                        pforward = pforward->rrtparent;
                    }
                    vtempinitialconfig = _treeForward.GetVectorConfig(pforward); // GetVectorConfig returns the same reference, so need to make a copy
                    dReal fGoalNodeDist = _parameters->_distmetricfn(vtempinitialconfig, _treeForward.GetVectorConfig(ptestnode));
                    if( !bestGoalNode || fBestGoalNodeDist > fGoalNodeDist ) {
                        bestGoalNode = ptestnode;
                        fBestGoalNodeDist = fGoalNodeDist;
                        _goalindex = -1;
                        RAVELOG_DEBUG_FORMAT("env=%d, found node at goal at dist=%f at %d iterations, computation time=%fs", GetEnv()->GetId()%fBestGoalNodeDist%iter%(0.001f*(float)(utils::GetMilliTime()-basetime)));
                    }
                }

                ptestnode->_userdata = 1;
                ptestnode = ptestnode->rrtparent;
            }
            if( bfound ) {
                if( iter >= _parameters->_nMinIterations ) {
                    // check how many times we've got a goal?
                    if( numfoundgoals >= (int)_parameters->_minimumgoalpaths ) {
                        break;
                    }
                }
            }
        }

        // check if reached any goals
        if( iter > _parameters->_nMaxIterations ) {
            RAVELOG_WARN_FORMAT("env=%d, iterations exceeded %d\n", GetEnv()->GetId()%_parameters->_nMaxIterations);
            break;
        }

        if( !!bestGoalNode && _parameters->_nMaxPlanningTime > 0 ) {
            uint32_t elapsedtime = utils::GetMilliTime()-basetime;
            if( elapsedtime >= _parameters->_nMaxPlanningTime ) {
                RAVELOG_VERBOSE_FORMAT("env=%d, time exceeded (%dms) so breaking with bestdist=%f", GetEnv()->GetId()%elapsedtime%fBestGoalNodeDist);
                break;
            }
        }

        progress._iteration = iter;
        callbackaction = _CallCallbacks(progress);
        if( callbackaction ==  PA_Interrupt ) {
            return OPENRAVE_PLANNER_STATUS(str(boost::format("env=%d, Planning was interrupted")%GetEnv()->GetId()), PS_Interrupted);
        }
        else if( callbackaction == PA_ReturnWithAnySolution ) {
            if( !!bestGoalNode ) {
                break;
            }
        }
    }

    if( !bestGoalNode ) {
        std::string description = str(boost::format("env=%d, plan failed, %fs")%GetEnv()->GetId()%(0.001f*(float)(utils::GetMilliTime()-basetime)));
        RAVELOG_DEBUG(description);
        return OPENRAVE_PLANNER_STATUS(description, PS_Failed);
    }

    const int dof = _parameters->GetDOF();
    _cachedpath.clear();

    // add nodes from the forward tree
    SimpleNodePtr pforward = bestGoalNode;
    while(1) {
        _cachedpath.insert(_cachedpath.begin(), pforward->q, pforward->q+dof);
        if(!pforward->rrtparent) {
            break;
        }
        pforward = pforward->rrtparent;
    }

    _SimpleOptimizePath(_cachedpath,10);
    if( ptraj->GetConfigurationSpecification().GetDOF() == 0 ) {
        ptraj->Init(_parameters->_configurationspecification);
    }
    std::vector<dReal> vinsertvalues(_cachedpath.begin(), _cachedpath.end());
    ptraj->Insert(ptraj->GetNumWaypoints(), vinsertvalues, _parameters->_configurationspecification);

    PlannerStatus status = _ProcessPostPlanners(_robot,ptraj);
    RAVELOG_DEBUG_FORMAT("env=%d, plan success, path=%d points computation time=%fs, maxPlanningTime=%f", GetEnv()->GetId()%ptraj->GetNumWaypoints()%((0.001f*(float)(utils::GetMilliTime()-basetime)))%(0.001*_parameters->_nMaxPlanningTime));
    return status;
}

bool BasicRrtPlanner::_DumpTreeCommand(std::ostream& os, std::istream& is) {
    std::string filename = RaveGetHomeDirectory() + string("/basicrrtdump.txt");
    getline(is, filename);
    boost::trim(filename);
    RAVELOG_VERBOSE(str(boost::format("dumping rrt tree to %s")%filename));
    ofstream f(filename.c_str());
    f << std::setprecision(std::numeric_limits<dReal>::digits10+1);
    _treeForward.DumpTree(f);
    return true;
}

ExplorationPlanner::ExplorationPlanner(EnvironmentBasePtr penv) : RrtPlanner(penv) {
    __description = ":Interface Author: Rosen Diankov\n\nRRT-based exploration planner";
}

bool ExplorationPlanner::InitPlan(RobotBasePtr pbase, PlannerParametersConstPtr pparams)
{
    EnvironmentMutex::scoped_lock lock(GetEnv()->GetMutex());
    _parameters.reset(new ExplorationParameters());
    _parameters->copy(pparams);
    if( !RrtPlanner::_InitPlan(pbase,_parameters) ) {
        _parameters.reset();
        return false;
    }
    RAVELOG_DEBUG_FORMAT("env=%d, ExplorationPlanner::InitPlan - RRT Planner Initialized", GetEnv()->GetId());
    return true;
}

PlannerStatus ExplorationPlanner::PlanPath(TrajectoryBasePtr ptraj, int planningoptions)
{
    _goalindex = -1;
    _startindex = -1;
    if( !_parameters ) {
        return OPENRAVE_PLANNER_STATUS(PS_Failed);
    }
    EnvironmentMutex::scoped_lock lock(GetEnv()->GetMutex());
    std::vector<dReal> vSampleConfig;

    PlannerParameters::StateSaver savestate(_parameters);
    CollisionOptionsStateSaver optionstate(GetEnv()->GetCollisionChecker(),GetEnv()->GetCollisionChecker()->GetCollisionOptions()|CO_ActiveDOFs,false);

    int iter = 0;
    while(iter < _parameters->_nMaxIterations && _treeForward.GetNumNodes() < _parameters->_nExpectedDataSize ) {
        ++iter;

        if( RaveRandomFloat() < _parameters->_fExploreProb ) {
            // explore
            int inode = RaveRandomInt()%_treeForward.GetNumNodes();
            SimpleNodePtr pnode = _treeForward.GetNodeFromIndex(inode);

            if( !_parameters->_sampleneighfn(vSampleConfig, _treeForward.GetVectorConfig(pnode), _parameters->_fStepLength) ) {
                continue;
            }
            if( GetParameters()->CheckPathAllConstraints(_treeForward.GetVectorConfig(pnode), vSampleConfig, {}, {}, 0, IT_OpenStart) == 0 ) {
                _treeForward.InsertNode(pnode, vSampleConfig, 0);
                GetEnv()->UpdatePublishedBodies();
                RAVELOG_DEBUG_FORMAT("env=%d, size %d", GetEnv()->GetId()%_treeForward.GetNumNodes());
            }
        }
        else {     // rrt extend
            if( !_parameters->_samplefn(vSampleConfig) ) {
                continue;
            }
            SimpleNodePtr plastnode;
            if( _treeForward.Extend(vSampleConfig, plastnode, true) == ET_Connected ) {
                RAVELOG_DEBUG_FORMAT("env=%d, size %d", GetEnv()->GetId()%_treeForward.GetNumNodes());
            }
        }
    }

    if( ptraj->GetConfigurationSpecification().GetDOF() == 0 ) {
        ptraj->Init(_parameters->_configurationspecification);
    }
    // save nodes to trajectory
    std::vector<SimpleNodePtr> vnodes;
    _treeForward.GetNodesVector(vnodes);
    FOREACH(itnode, vnodes) {
        ptraj->Insert(ptraj->GetNumWaypoints(), _treeForward.GetVectorConfig(*itnode), _parameters->_configurationspecification);
    }
    return OPENRAVE_PLANNER_STATUS(PS_HasSolution);
}
