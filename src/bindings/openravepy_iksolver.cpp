// -*- coding: utf-8 -*-
// Copyright (C) 2006-2011 Rosen Diankov <rosen.diankov@gmail.com>
//
// This file is part of OpenRAVE.
// OpenRAVE is free software: you can redistribute it and/or modify
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
#define NO_IMPORT_ARRAY
#include "openravepy_int.h"
#include <openrave/utils.h>
#include "openravepy_iksolverbase.h"

namespace openravepy {

bool ExtractIkReturn(object o, IkReturn& ikfr)
{
    extract<PyIkReturnPtr > pyikfr(o);
    if( pyikfr.check() ) {
        ikfr = ((PyIkReturnPtr)pyikfr)->_ret;
        return true;
    }
    return false;
}

object toPyIkReturn(const IkReturn& ret)
{
    return object(PyIkReturnPtr(new PyIkReturn(ret)));
}

IkSolverBasePtr GetIkSolver(object oiksolver)
{
    extract<PyIkSolverBasePtr> pyiksolver(oiksolver);
    if( pyiksolver.check() ) {
        return ((PyIkSolverBasePtr)pyiksolver)->GetIkSolver();
    }
    return IkSolverBasePtr();
}

IkSolverBasePtr GetIkSolver(PyIkSolverBasePtr pyIkSolver)
{
    return !pyIkSolver ? IkSolverBasePtr() : pyIkSolver->GetIkSolver();
}

PyInterfaceBasePtr toPyIkSolver(IkSolverBasePtr pIkSolver, PyEnvironmentBasePtr pyenv)
{
    return !pIkSolver ? PyInterfaceBasePtr() : PyInterfaceBasePtr(new PyIkSolverBase(pIkSolver,pyenv));
}

object toPyIkSolver(IkSolverBasePtr pIkSolver, object opyenv)
{
    extract<PyEnvironmentBasePtr> pyenv(opyenv);
    if( pyenv.check() ) {
        return object(toPyIkSolver(pIkSolver,(PyEnvironmentBasePtr)pyenv));
    }
    return object();
}

PyIkSolverBasePtr RaveCreateIkSolver(PyEnvironmentBasePtr pyenv, const std::string& name)
{
    IkSolverBasePtr p = OpenRAVE::RaveCreateIkSolver(GetEnvironment(pyenv), name);
    if( !p ) {
        return PyIkSolverBasePtr();
    }
    return PyIkSolverBasePtr(new PyIkSolverBase(p,pyenv));
}

void init_openravepy_iksolver()
{
    enum_<IkFilterOptions>("IkFilterOptions" DOXY_ENUM(IkFilterOptions))
    .value("CheckEnvCollisions",IKFO_CheckEnvCollisions)
    .value("IgnoreSelfCollisions",IKFO_IgnoreSelfCollisions)
    .value("IgnoreJointLimits",IKFO_IgnoreJointLimits)
    .value("IgnoreCustomFilters",IKFO_IgnoreCustomFilters)
    .value("IgnoreEndEffectorCollisions",IKFO_IgnoreEndEffectorCollisions)
    .value("IgnoreEndEffectorEnvCollisions",IKFO_IgnoreEndEffectorEnvCollisions)
    .value("IgnoreEndEffectorSelfCollisions",IKFO_IgnoreEndEffectorSelfCollisions)
    ;

    enum_<IkReturnAction>("IkReturnAction" DOXY_ENUM(IkReturnAction))
    .value("Success",IKRA_Success)
    .value("Reject",IKRA_Reject)
    .value("Quit",IKRA_Quit)
    .value("QuitEndEffectorCollision",IKRA_QuitEndEffectorCollision)
    .value("RejectKinematics",IKRA_RejectKinematics)
    .value("RejectSelfCollision",IKRA_RejectSelfCollision)
    .value("RejectEnvCollision",IKRA_RejectEnvCollision)
    .value("RejectJointLimits",IKRA_RejectJointLimits)
    .value("RejectKinematicsPrecision",IKRA_RejectKinematicsPrecision)
    .value("RejectCustomFilter",IKRA_RejectCustomFilter)
    ;

    {
        scope ikreturn = class_<PyIkReturn, PyIkReturnPtr>("IkReturn", DOXY_CLASS(IkReturn), no_init)
                         .def(init<IkReturnAction>(args("action")))
                         .def("GetAction",&PyIkReturn::GetAction, "Retuns IkReturn::_action")
                         .def("GetSolution",&PyIkReturn::GetSolution, "Retuns IkReturn::_vsolution")
                         .def("GetUserData",&PyIkReturn::GetUserData, "Retuns IkReturn::_userdata")
                         .def("GetMapData",&PyIkReturn::GetMapData, args("key"), "Indexes into the map and returns an array of numbers. If key doesn't exist, returns None")
                         .def("GetMapDataDict",&PyIkReturn::GetMapDataDict, "Returns a dictionary copy for IkReturn::_mapdata")
                         .def("SetUserData",&PyIkReturn::SetUserData,args("data"),"Set IKReturn::_userdata")
                         .def("SetSolution",&PyIkReturn::SetSolution,args("solution"),"Set IKReturn::_vsolution")
                         .def("SetMapKeyValue",&PyIkReturn::SetMapKeyValue,args("key,value"),"Adds key/value pair to IKReturn::_mapdata")
        ;
    }

    {
        PyIkReturnPtr (PyIkSolverBase::*Solve)(object, object, int) = &PyIkSolverBase::Solve;
        PyIkReturnPtr (PyIkSolverBase::*SolveFree)(object, object, object, int) = &PyIkSolverBase::Solve;
        object (PyIkSolverBase::*SolveAll)(object, int) = &PyIkSolverBase::SolveAll;
        object (PyIkSolverBase::*SolveAllFree)(object, object, int) = &PyIkSolverBase::SolveAll;
        class_<PyIkSolverBase, OPENRAVE_SHARED_PTR<PyIkSolverBase>, bases<PyInterfaceBase> >("IkSolver", DOXY_CLASS(IkSolverBase), no_init)
        .def("Solve",Solve,args("ikparam","q0","filteroptions"), DOXY_FN(IkSolverBase, Solve "const IkParameterization&; const std::vector; int; IkReturnPtr"))
        .def("Solve",SolveFree,args("ikparam","q0","freeparameters", "filteroptions"), DOXY_FN(IkSolverBase, Solve "const IkParameterization&; const std::vector; const std::vector; int; IkReturnPtr"))
        .def("SolveAll",SolveAll,args("ikparam","filteroptions"), DOXY_FN(IkSolverBase, SolveAll "const IkParameterization&; int; std::vector<IkReturnPtr>"))
        .def("SolveAll",SolveAllFree,args("ikparam","freeparameters","filteroptions"), DOXY_FN(IkSolverBase, SolveAll "const IkParameterization&; const std::vector; int; std::vector<IkReturnPtr>"))
        .def("GetNumFreeParameters",&PyIkSolverBase::GetNumFreeParameters, DOXY_FN(IkSolverBase,GetNumFreeParameters))
        .def("GetFreeParameters",&PyIkSolverBase::GetFreeParameters, DOXY_FN(IkSolverBase,GetFreeParameters))
        .def("Supports",&PyIkSolverBase::Supports, args("iktype"), DOXY_FN(IkSolverBase,Supports))
        .def("CallFilters",&PyIkSolverBase::CallFilters, args("ikparam"), DOXY_FN(IkSolverBase,CallFilters))
        .def("RegisterCustomFilter",&PyIkSolverBase::RegisterCustomFilter, args("priority","callback"), DOXY_FN(IkSolverBase,RegisterCustomFilter))
        ;
    }

    def("RaveCreateIkSolver",openravepy::RaveCreateIkSolver,args("env","name"),DOXY_FN1(RaveCreateIkSolver));
}

}
