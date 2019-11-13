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
#ifndef OPENRAVEPY_INTERNAL_PHYSICSENGINE_H
#define OPENRAVEPY_INTERNAL_PHYSICSENGINE_H

#define NO_IMPORT_ARRAY
#include "openravepy_int.h"

namespace openravepy {

using py::object;

class PyPhysicsEngineBase : public PyInterfaceBase
{
protected:
    PhysicsEngineBasePtr _pPhysicsEngine;
public:
    PyPhysicsEngineBase(PhysicsEngineBasePtr pPhysicsEngine, PyEnvironmentBasePtr pyenv) : PyInterfaceBase(pPhysicsEngine, pyenv),_pPhysicsEngine(pPhysicsEngine) {
    }
    virtual ~PyPhysicsEngineBase() {
    }

    PhysicsEngineBasePtr GetPhysicsEngine() {
        return _pPhysicsEngine;
    }

    bool SetPhysicsOptions(int physicsoptions) {
        return _pPhysicsEngine->SetPhysicsOptions(physicsoptions);
    }
    int GetPhysicsOptions() const {
        return _pPhysicsEngine->GetPhysicsOptions();
    }
    bool InitEnvironment() {
        return _pPhysicsEngine->InitEnvironment();
    }
    void DestroyEnvironment() {
        _pPhysicsEngine->DestroyEnvironment();
    }
    bool InitKinBody(PyKinBodyPtr pbody) {
        CHECK_POINTER(pbody); return _pPhysicsEngine->InitKinBody(openravepy::GetKinBody(pbody));
    }

    bool SetLinkVelocity(object pylink, object linearvel, object angularvel)
    {
        CHECK_POINTER(pylink);
        return _pPhysicsEngine->SetLinkVelocity(openravepy::GetKinBodyLink(pylink),ExtractVector3(linearvel),ExtractVector3(angularvel));
    }

    bool SetLinkVelocities(PyKinBodyPtr pykinbody, object ovelocities)
    {
        std::vector<std::pair<Vector,Vector> > velocities;
        velocities.resize(len(ovelocities));
        for(size_t i = 0; i < velocities.size(); ++i) {
            vector<dReal> v = ExtractArray<dReal>(ovelocities[i]);
            BOOST_ASSERT(v.size()==6);
            velocities[i].first.x = v[0];
            velocities[i].first.y = v[1];
            velocities[i].first.z = v[2];
            velocities[i].second.x = v[3];
            velocities[i].second.y = v[4];
            velocities[i].second.z = v[5];
        }
        return _pPhysicsEngine->SetLinkVelocities(openravepy::GetKinBody(pykinbody), velocities);
    }

    object GetLinkVelocity(object pylink)
    {
        CHECK_POINTER(pylink);
        Vector linearvel, angularvel;
        if( !_pPhysicsEngine->GetLinkVelocity(openravepy::GetKinBodyLink(pylink),linearvel,angularvel) ) {
            return object();
        }
        return py::make_tuple(toPyVector3(linearvel),toPyVector3(angularvel));
    }

    object GetLinkVelocities(PyKinBodyPtr pykinbody)
    {
        CHECK_POINTER(pykinbody);
        KinBodyPtr pbody = openravepy::GetKinBody(pykinbody);
        if( pbody->GetLinks().size() == 0 ) {
            return py::array_t<dReal>({1, 0}, nullptr);
        }
        std::vector<std::pair<Vector,Vector> > velocities;
        if( !_pPhysicsEngine->GetLinkVelocities(pbody,velocities) ) {
            return object();
        }
        npy_intp dims[] = { npy_intp(velocities.size()), 6};
        PyObject *pyvel = PyArray_SimpleNew(2,dims, sizeof(dReal)==8 ? PyArray_DOUBLE : PyArray_FLOAT);
        dReal* pfvel = (dReal*)PyArray_DATA(pyvel);
        for(size_t i = 0; i < velocities.size(); ++i) {
            pfvel[6*i+0] = velocities[i].first.x;
            pfvel[6*i+1] = velocities[i].first.y;
            pfvel[6*i+2] = velocities[i].first.z;
            pfvel[6*i+3] = velocities[i].second.x;
            pfvel[6*i+4] = velocities[i].second.y;
            pfvel[6*i+5] = velocities[i].second.z;
        }
        return toPyArrayN(pfvel, 6 * velocities.size());
    }

    bool SetBodyForce(object pylink, object force, object position, bool bAdd)
    {
        CHECK_POINTER(pylink);
        return _pPhysicsEngine->SetBodyForce(openravepy::GetKinBodyLink(pylink),ExtractVector3(force),ExtractVector3(position),bAdd);
    }

    bool SetBodyTorque(object pylink, object torque, bool bAdd)
    {
        CHECK_POINTER(pylink);
        return _pPhysicsEngine->SetBodyTorque(openravepy::GetKinBodyLink(pylink),ExtractVector3(torque),bAdd);
    }

    bool AddJointTorque(object pyjoint, object torques)
    {
        CHECK_POINTER(pyjoint);
        return _pPhysicsEngine->AddJointTorque(openravepy::GetKinBodyJoint(pyjoint),ExtractArray<dReal>(torques));
    }

    object GetLinkForceTorque(object pylink)
    {
        CHECK_POINTER(pylink);
        Vector force, torque;
        if( !_pPhysicsEngine->GetLinkForceTorque(openravepy::GetKinBodyLink(pylink),force,torque) ) {
            return object();
        }
        return py::make_tuple(toPyVector3(force),toPyVector3(torque));
    }

    object GetJointForceTorque(object pyjoint)
    {
        CHECK_POINTER(pyjoint);
        Vector force, torque;
        if( !_pPhysicsEngine->GetJointForceTorque(openravepy::GetKinBodyJoint(pyjoint),force,torque) ) {
            return object();
        }
        return py::make_tuple(toPyVector3(force),toPyVector3(torque));
    }

    void SetGravity(object gravity) {
        _pPhysicsEngine->SetGravity(ExtractVector3(gravity));
    }
    object GetGravity() {
        return toPyVector3(_pPhysicsEngine->GetGravity());
    }

    void SimulateStep(dReal fTimeElapsed) {
        _pPhysicsEngine->SimulateStep(fTimeElapsed);
    }
};

} // namespace openravepy

#endif // OPENRAVEPY_INTERNAL_PHYSICSENGINE_H
