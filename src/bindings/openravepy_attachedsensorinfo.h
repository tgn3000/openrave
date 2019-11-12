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
#ifndef OPENRAVEPY_INTERNAL_ATTACHEDSENSORINFO_H
#define OPENRAVEPY_INTERNAL_ATTACHEDSENSORINFO_H

#define NO_IMPORT_ARRAY
#include "openravepy_int.h"

namespace openravepy {

class PyAttachedSensorInfo
{
public:
    PyAttachedSensorInfo() {
    }
    PyAttachedSensorInfo(const RobotBase::AttachedSensorInfo& info) {
        _name = ConvertStringToUnicode(info._name);
        _linkname = ConvertStringToUnicode(info._linkname);
        _trelative = ReturnTransform(info._trelative);
        _sensorname = ConvertStringToUnicode(info._sensorname);
        _sensorgeometry = toPySensorGeometry(info._sensorgeometry);
    }

    RobotBase::AttachedSensorInfoPtr GetAttachedSensorInfo() const
    {
        RobotBase::AttachedSensorInfoPtr pinfo(new RobotBase::AttachedSensorInfo());
        pinfo->_name = _name.cast<std::string>();
        pinfo->_linkname = _linkname.cast<std::string>();
        pinfo->_trelative = ExtractTransform(_trelative);
        pinfo->_sensorname = _sensorname.cast<std::string>();
        pinfo->_sensorgeometry = _sensorgeometry->GetGeometry();
        return pinfo;
    }

    object _name, _linkname;
    object _trelative;
    object _sensorname;
    PySensorGeometryPtr _sensorgeometry;
};

} // namespace openravepy

#endif // OPENRAVEPY_INTERNAL_ATTACHEDSENSORINFO_H
