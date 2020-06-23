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
#ifndef  BIRRT_PLANNER_H
#define  BIRRT_PLANNER_H

#include <boost/algorithm/string.hpp>
#include "rplanners.h"

static const dReal g_fEpsilonDotProduct = RavePow(g_fEpsilon,0.8);

class RrtPlanner;
using RrtPlannerPtr = boost::shared_ptr<RrtPlanner>;
using RrtPlannerConstPtr = boost::shared_ptr<RrtPlanner const>;

class RrtPlanner : public PlannerBase
{
public:
    RrtPlanner(EnvironmentBasePtr penv);
    virtual ~RrtPlanner() = default;

    virtual bool _InitPlan(RobotBasePtr pbase, PlannerParametersPtr params);
    /// \brief simple path optimization given a path of dof values. Every _parameters->GetDOF() are one point in the path
    virtual void _SimpleOptimizePath(std::deque<dReal>& path, int numiterations);

    // get functions
    bool GetGoalIndexCommand(std::ostream& os, std::istream& is);
    bool GetInitGoalIndicesCommand(std::ostream& os, std::istream& is);

protected:
    RobotBasePtr _robot;
    std::vector<dReal> _sampleConfig;
    int _goalindex, _startindex;
    SpaceSamplerBasePtr _uniformsampler;
    ConstraintFilterReturnPtr _filterreturn;
    std::deque<dReal> _cachedpath;

    SpatialTree _treeForward;
    std::vector< SimpleNodePtr > _vecInitialNodes;

    RrtPlannerPtr shared_planner();
    RrtPlannerConstPtr shared_planner_const() const;
};

class BirrtPlanner : public RrtPlanner
{
public:
    BirrtPlanner(EnvironmentBasePtr penv);
    virtual ~BirrtPlanner() = default;

    struct GOALPATH
    {
        GOALPATH() {}
        std::vector<dReal> qall;
        int startindex = -1;
        int goalindex = -1;
        dReal length = 0;
    };

    virtual bool InitPlan(RobotBasePtr pbase, PlannerParametersConstPtr pparams);
    virtual PlannerStatus PlanPath(TrajectoryBasePtr ptraj, int planningoptions) override;
    virtual void _ExtractPath(GOALPATH& goalpath, SimpleNodePtr iConnectedForward, SimpleNodePtr iConnectedBackward);

    // get functions
    virtual PlannerParametersConstPtr GetParameters() const override { return _parameters; }
    virtual bool _DumpTreeCommand(std::ostream& os, std::istream& is);

protected:
    RRTParametersPtr _parameters;
    SpatialTree _treeBackward;
    dReal _fGoalBiasProb;
    std::vector< SimpleNodePtr > _vecGoalNodes;
    size_t _nValidGoals; ///< num valid goals
    std::vector<GOALPATH> _vgoalpaths;
};

class BasicRrtPlanner : public RrtPlanner
{
public:
    BasicRrtPlanner(EnvironmentBasePtr penv);
    virtual ~BasicRrtPlanner() = default;

    bool InitPlan(RobotBasePtr pbase, PlannerParametersConstPtr pparams) override;
    PlannerStatus PlanPath(TrajectoryBasePtr ptraj, int planningoptions) override;

    // get functions
    virtual PlannerParametersConstPtr GetParameters() const override { return _parameters; }
    virtual bool _DumpTreeCommand(std::ostream& os, std::istream& is);

protected:
    BasicRRTParametersPtr _parameters;
    dReal _fGoalBiasProb;
    bool _bOneStep;
    std::vector< std::vector<dReal> > _vecGoals;
    int _nValidGoals; ///< num valid goals
};

class ExplorationPlanner : public RrtPlanner
{
public:
    ExplorationPlanner(EnvironmentBasePtr penv);
    virtual ~ExplorationPlanner() = default;

    virtual bool InitPlan(RobotBasePtr pbase, PlannerParametersConstPtr pparams) override;
    virtual PlannerStatus PlanPath(TrajectoryBasePtr ptraj, int planningoptions) override;

    // get functions
    virtual PlannerParametersConstPtr GetParameters() const override { return _parameters; }

private:
    boost::shared_ptr<ExplorationParameters> _parameters;
};

#ifdef RAVE_REGISTER_BOOST
#include BOOST_TYPEOF_INCREMENT_REGISTRATION_GROUP()
BOOST_TYPEOF_REGISTER_TYPE(BirrtPlanner::GOALPATH)
#endif

#endif // BIRRT_PLANNER_H
