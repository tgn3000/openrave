// -*- coding: utf-8 -*-
// Copyright (C) 2006-2013 Rosen Diankov <rosen.diankov@gmail.com>
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
#ifndef OPENRAVEPY_INTERNAL_CONTROLLERBASE_H
#define OPENRAVEPY_INTERNAL_CONTROLLERBASE_H

#include <openrave/utils.h>

#define NO_IMPORT_ARRAY
#include "openravepy_int.h"

using py::object;

namespace openravepy {

class PyControllerBase : public PyInterfaceBase
{
protected:
    ControllerBasePtr _pcontroller;
public:
    PyControllerBase(ControllerBasePtr pcontroller, PyEnvironmentBasePtr pyenv) : PyInterfaceBase(pcontroller, pyenv), _pcontroller(pcontroller) {
    }
    virtual ~PyControllerBase() {
    }

    ControllerBasePtr GetOpenRAVEController() {
        return _pcontroller;
    }

    bool Init(PyRobotBasePtr pyrobot, const string& args)
    {
        RAVELOG_WARN("PyControllerBase::Init(robot,args) deprecated!\n");
        CHECK_POINTER(pyrobot);
        RobotBasePtr probot = openravepy::GetRobot(pyrobot);
        std::vector<int> dofindices;
        for(int i = 0; i < probot->GetDOF(); ++i) {
            dofindices.push_back(i);
        }
        return _pcontroller->Init(probot, dofindices,1);
    }

    bool Init(PyRobotBasePtr pyrobot, object odofindices, int nControlTransformation)
    {
        CHECK_POINTER(pyrobot);
        vector<int> dofindices = ExtractArray<int>(odofindices);
        return _pcontroller->Init(openravepy::GetRobot(pyrobot),dofindices,nControlTransformation);
    }

    object GetControlDOFIndices() {
        return toPyArray(_pcontroller->GetControlDOFIndices());
    }
    int IsControlTransformation() {
        return _pcontroller->IsControlTransformation();
    }
    object GetRobot()
    {
        return py::cast(openravepy::toPyRobot(_pcontroller->GetRobot(),_pyenv));
    }

    void Reset(int options=0) {
        _pcontroller->Reset(options);
    }

    bool SetDesired(object o)
    {
        vector<dReal> values = ExtractArray<dReal>(o);
        if( values.size() == 0 ) {
            throw openrave_exception(_("no values specified"));
        }
        return _pcontroller->SetDesired(values);
    }

    bool SetDesired(object o, object otransform)
    {
        if( IS_PYTHONOBJECT_NONE(otransform) ) {
            return SetDesired(o);
        }
        return _pcontroller->SetDesired(ExtractArray<dReal>(o),TransformConstPtr(new Transform(ExtractTransform(otransform))));
    }

    bool SetPath(PyTrajectoryBasePtr pytraj)
    {
        CHECK_POINTER(pytraj);
        return _pcontroller->SetPath (openravepy::GetTrajectory(pytraj));
    }

    void SimulationStep(dReal fTimeElapsed) {
        _pcontroller->SimulationStep(fTimeElapsed);
    }

    bool IsDone() {
        return _pcontroller->IsDone();
    }
    dReal GetTime() {
        return _pcontroller->GetTime();
    }

    object GetVelocity()
    {
        vector<dReal> velocity;
        _pcontroller->GetVelocity(velocity);
        return toPyArray(velocity);
    }

    object GetTorque()
    {
        vector<dReal> torque;
        _pcontroller->GetTorque(torque);
        return toPyArray(torque);
    }
};

} // namespace openravepy

#endif // OPENRAVEPY_INTERNAL_CONTROLLERBASE_H
