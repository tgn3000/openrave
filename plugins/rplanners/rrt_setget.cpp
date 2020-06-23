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
