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
#include "openravepy_controllerbase.h"
#include "openravepy_multicontrollerbase.h"
#include "openravepy_robot.h"
#include "openravepy_trajectorybase.h"
#include "openravepy_environment.h"
#include "openravepy_collisionreport.h"

namespace openravepy {

ControllerBasePtr GetController(PyControllerBasePtr pycontroller)
{
    return !pycontroller ? ControllerBasePtr() : pycontroller->GetOpenRAVEController();
}

PyInterfaceBasePtr toPyController(ControllerBasePtr pcontroller, PyEnvironmentBasePtr pyenv)
{
    if( !pcontroller )  {
        return PyInterfaceBasePtr();
    }
    // TODO this is a hack
    // unfortunately dynamic_pointer_cast will not work. The most ideal situation is to have MultiControllerBase registered as its own individual interface....
    else if( pcontroller->GetXMLId() == std::string("MultiController") ) {
        return PyInterfaceBasePtr(new PyMultiControllerBase(OPENRAVE_STATIC_POINTER_CAST<MultiControllerBase>(pcontroller), pyenv));
    }
    else {
        return PyInterfaceBasePtr(new PyControllerBase(pcontroller, pyenv));
    }
}

PyControllerBasePtr RaveCreateController(PyEnvironmentBasePtr pyenv, const std::string& name)
{
    ControllerBasePtr pcontroller = OpenRAVE::RaveCreateController(GetEnvironment(pyenv), name);
    if( !pcontroller ) {
        return PyControllerBasePtr();
    }
    return PyControllerBasePtr(new PyControllerBase(pcontroller, pyenv));
}

PyMultiControllerBasePtr RaveCreateMultiController(PyEnvironmentBasePtr pyenv, const std::string& name)
{
    MultiControllerBasePtr pcontroller = OpenRAVE::RaveCreateMultiController(GetEnvironment(pyenv), name);
    if( !pcontroller ) {
        return PyMultiControllerBasePtr();
    }
    return PyMultiControllerBasePtr(new PyMultiControllerBase(pcontroller, pyenv));
}

void init_openravepy_controller(py::module& m)
{
    {
        bool (PyControllerBase::*init1)(PyRobotBasePtr,const string &) = &PyControllerBase::Init;
        bool (PyControllerBase::*init2)(PyRobotBasePtr,object,int) = &PyControllerBase::Init;
        bool (PyControllerBase::*setdesired1)(object) = &PyControllerBase::SetDesired;
        bool (PyControllerBase::*setdesired2)(object,object) = &PyControllerBase::SetDesired;
        py::class_<PyControllerBase, OPENRAVE_SHARED_PTR<PyControllerBase>, PyInterfaceBase>(m, "Controller", DOXY_CLASS(ControllerBase))
        .def("Init",init1, DOXY_FN(ControllerBase,Init))
        .def("Init",init2, py::arg("robot"), py::arg("dofindices"), py::arg("controltransform"), DOXY_FN(ControllerBase,Init))
        .def("GetControlDOFIndices",&PyControllerBase::GetControlDOFIndices,DOXY_FN(ControllerBase,GetControlDOFIndices))
        .def("IsControlTransformation",&PyControllerBase::IsControlTransformation, DOXY_FN(ControllerBase,IsControlTransformation))
        .def("GetRobot",&PyControllerBase::GetRobot, DOXY_FN(ControllerBase,GetRobot))
        .def("Reset",&PyControllerBase::Reset, py::arg("options"), DOXY_FN(ControllerBase,Reset))
        .def("SetDesired",setdesired1, py::arg("values"), DOXY_FN(ControllerBase,SetDesired))
        .def("SetDesired",setdesired2, py::arg("values"), py::arg("transform"), DOXY_FN(ControllerBase,SetDesired))
        .def("SetPath",&PyControllerBase::SetPath, DOXY_FN(ControllerBase,SetPath))
        .def("SimulationStep",&PyControllerBase::SimulationStep, DOXY_FN(ControllerBase,SimulationStep "dReal"))
        .def("IsDone",&PyControllerBase::IsDone, DOXY_FN(ControllerBase,IsDone))
        .def("GetTime",&PyControllerBase::GetTime, DOXY_FN(ControllerBase,GetTime))
        .def("GetVelocity",&PyControllerBase::GetVelocity, DOXY_FN(ControllerBase,GetVelocity))
        .def("GetTorque",&PyControllerBase::GetTorque, DOXY_FN(ControllerBase,GetTorque))
        ;
    }

    {
        py::class_<PyMultiControllerBase, OPENRAVE_SHARED_PTR<PyMultiControllerBase>, PyControllerBase>(m, "MultiController", DOXY_CLASS(MultiControllerBase))
        .def("AttachController",&PyMultiControllerBase::AttachController, py::arg("controller"), py::arg("dofindices"), py::arg("controltransform"), DOXY_FN(MultiControllerBase,AttachController))
        .def("RemoveController",&PyMultiControllerBase::RemoveController, py::arg("controller"), DOXY_FN(MultiControllerBase,RemoveController))
        .def("GetController",&PyMultiControllerBase::GetController, py::arg("dof"), DOXY_FN(MultiControllerBase,GetController))
        ;
    }

    m.def("RaveCreateController",openravepy::RaveCreateController,py::arg("env"), py::arg("name"),DOXY_FN1(RaveCreateController));
    m.def("RaveCreateMultiController",openravepy::RaveCreateMultiController,py::arg("env"), py::arg("name"),DOXY_FN1(RaveCreateMultiController));
}

}
