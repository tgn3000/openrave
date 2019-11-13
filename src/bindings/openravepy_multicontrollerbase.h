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
#ifndef OPENRAVEPY_INTERNAL_MULTICONTROLLERBASE_H
#define OPENRAVEPY_INTERNAL_MULTICONTROLLERBASE_H

#include <openrave/utils.h>

#define NO_IMPORT_ARRAY
#include "openravepy_int.h"

using py::object;

namespace openravepy {

class PyMultiControllerBase : public PyControllerBase
{
private:
    MultiControllerBasePtr _pmulticontroller;

public:
    PyMultiControllerBase(MultiControllerBasePtr pmulticontroller, PyEnvironmentBasePtr pyenv) : PyControllerBase(pmulticontroller, pyenv), _pmulticontroller(pmulticontroller) {
    }
    virtual ~PyMultiControllerBase() {
    }

    bool AttachController(PyControllerBasePtr ocontroller, object odofindices, int nControlTransformation) {
        CHECK_POINTER(ocontroller);
        vector<int> dofindices = ExtractArray<int>(odofindices);
        return _pmulticontroller->AttachController(ocontroller->GetOpenRAVEController(), dofindices, nControlTransformation);
    }

    void RemoveController(PyControllerBasePtr ocontroller) {
        CHECK_POINTER(ocontroller);
        _pmulticontroller->RemoveController(ocontroller->GetOpenRAVEController());
    }

    object GetController(int dof) {
        CHECK_POINTER(_pmulticontroller);
        ControllerBasePtr pcontroller = _pmulticontroller->GetController(dof);
        return object(openravepy::toPyController(pcontroller, _pyenv));
    }
};

} // namespace openravepy
#endif // OPENRAVEPY_INTERNAL_MULTICONTROLLERBASE_H
