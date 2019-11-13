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
#include "openravepy_physicsengine.hpp"

namespace openravepy {

PhysicsEngineBasePtr GetPhysicsEngine(PyPhysicsEngineBasePtr pyPhysicsEngine)
{
    return !pyPhysicsEngine ? PhysicsEngineBasePtr() : pyPhysicsEngine->GetPhysicsEngine();
}

PyInterfaceBasePtr toPyPhysicsEngine(PhysicsEngineBasePtr pPhysicsEngine, PyEnvironmentBasePtr pyenv)
{
    return !pPhysicsEngine ? PyInterfaceBasePtr() : PyInterfaceBasePtr(new PyPhysicsEngineBase(pPhysicsEngine,pyenv));
}

PyPhysicsEngineBasePtr RaveCreatePhysicsEngine(PyEnvironmentBasePtr pyenv, const std::string& name)
{
    PhysicsEngineBasePtr p = OpenRAVE::RaveCreatePhysicsEngine(GetEnvironment(pyenv), name);
    if( !p ) {
        return PyPhysicsEngineBasePtr();
    }
    return PyPhysicsEngineBasePtr(new PyPhysicsEngineBase(p,pyenv));
}

void init_openravepy_physicsengine()
{
    class_<PyPhysicsEngineBase, OPENRAVE_SHARED_PTR<PyPhysicsEngineBase>, bases<PyInterfaceBase> >("PhysicsEngine", DOXY_CLASS(PhysicsEngineBase), no_init)
    .def("GetPhysicsOptions",&PyPhysicsEngineBase::GetPhysicsOptions, DOXY_FN(PhysicsEngineBase,GetPhysicsOptions))
    .def("SetPhysicsOptions",&PyPhysicsEngineBase::SetPhysicsOptions, DOXY_FN(PhysicsEngineBase,SetPhysicsOptions "int"))
    .def("InitEnvironment",&PyPhysicsEngineBase::InitEnvironment, DOXY_FN(PhysicsEngineBase,InitEnvironment))
    .def("DestroyEnvironment",&PyPhysicsEngineBase::DestroyEnvironment, DOXY_FN(PhysicsEngineBase,DestroyEnvironment))
    .def("InitKinBody",&PyPhysicsEngineBase::InitKinBody, DOXY_FN(PhysicsEngineBase,InitKinBody))
    .def("SetLinkVelocity",&PyPhysicsEngineBase::SetLinkVelocity, args("link","velocity"), DOXY_FN(PhysicsEngineBase,SetLinkVelocity))
    .def("SetLinkVelocities",&PyPhysicsEngineBase::SetLinkVelocity, args("body","velocities"), DOXY_FN(PhysicsEngineBase,SetLinkVelocities))
    .def("GetLinkVelocity",&PyPhysicsEngineBase::GetLinkVelocity, DOXY_FN(PhysicsEngineBase,GetLinkVelocity))
    .def("GetLinkVelocities",&PyPhysicsEngineBase::GetLinkVelocity, DOXY_FN(PhysicsEngineBase,GetLinkVelocities))
    .def("SetBodyForce",&PyPhysicsEngineBase::SetBodyForce, DOXY_FN(PhysicsEngineBase,SetBodyForce))
    .def("SetBodyTorque",&PyPhysicsEngineBase::SetBodyTorque, DOXY_FN(PhysicsEngineBase,SetBodyTorque))
    .def("AddJointTorque",&PyPhysicsEngineBase::AddJointTorque, DOXY_FN(PhysicsEngineBase,AddJointTorque))
    .def("GetLinkForceTorque",&PyPhysicsEngineBase::GetLinkForceTorque, DOXY_FN(PhysicsEngineBase,GetLinkForceTorque))
    .def("GetJointForceTorque",&PyPhysicsEngineBase::GetJointForceTorque, DOXY_FN(PhysicsEngineBase,GetJointForceTorque))
    .def("SetGravity",&PyPhysicsEngineBase::SetGravity, DOXY_FN(PhysicsEngineBase,SetGravity))
    .def("GetGravity",&PyPhysicsEngineBase::GetGravity, DOXY_FN(PhysicsEngineBase,GetGravity))
    .def("SimulateStep",&PyPhysicsEngineBase::SimulateStep, DOXY_FN(PhysicsEngineBase,SimulateStep))
    ;

    def("RaveCreatePhysicsEngine",openravepy::RaveCreatePhysicsEngine,args("env","name"),DOXY_FN1(RaveCreatePhysicsEngine));
}

}
