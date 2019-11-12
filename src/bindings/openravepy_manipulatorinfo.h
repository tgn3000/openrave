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
#ifndef OPENRAVEPY_INTERNAL_MANIPULATORINFO_H
#define OPENRAVEPY_INTERNAL_MANIPULATORINFO_H

#define NO_IMPORT_ARRAY
#include "openravepy_int.h"

namespace openravepy {

class PyManipulatorInfo
{
public:
    PyManipulatorInfo() {
        _tLocalTool = ReturnTransform(Transform());
        _vChuckingDirection = py::array_t<dReal>({1, 0}, nullptr);
        _vdirection = toPyVector3(Vector(0,0,1));
        _vGripperJointNames = py::list();
    }
    PyManipulatorInfo(const RobotBase::ManipulatorInfo& info) {
        _name = ConvertStringToUnicode(info._name);
        _sBaseLinkName = ConvertStringToUnicode(info._sBaseLinkName);
        _sEffectorLinkName = ConvertStringToUnicode(info._sEffectorLinkName);
        _tLocalTool = ReturnTransform(info._tLocalTool);
        _vChuckingDirection = toPyArray(info._vChuckingDirection);
        _vdirection = toPyVector3(info._vdirection);
        _sIkSolverXMLId = info._sIkSolverXMLId;
        py::list vGripperJointNames;
        FOREACHC(itname, info._vGripperJointNames) {
            vGripperJointNames.append(ConvertStringToUnicode(*itname));
        }
        _vGripperJointNames = vGripperJointNames;
    }

    RobotBase::ManipulatorInfoPtr GetManipulatorInfo() const
    {
        RobotBase::ManipulatorInfoPtr pinfo(new RobotBase::ManipulatorInfo());
        pinfo->_name = _name.cast<std::string>();
        pinfo->_sBaseLinkName = _sBaseLinkName.cast<std::string>();
        pinfo->_sEffectorLinkName = _sEffectorLinkName.cast<std::string>();
        pinfo->_tLocalTool = ExtractTransform(_tLocalTool);
        pinfo->_vChuckingDirection = ExtractArray<dReal>(_vChuckingDirection);
        pinfo->_vdirection = ExtractVector3(_vdirection);
        pinfo->_sIkSolverXMLId = _sIkSolverXMLId;
        pinfo->_vGripperJointNames = ExtractArray<std::string>(_vGripperJointNames);
        return pinfo;
    }

    py::object _name, _sBaseLinkName, _sEffectorLinkName;
    py::object _tLocalTool;
    py::object _vChuckingDirection;
    py::object _vdirection;
    std::string _sIkSolverXMLId;
    py::object _vGripperJointNames;
};

} // namespace openravepy

#endif // OPENRAVEPY_INTERNAL_MANIPULATORINFO_H
