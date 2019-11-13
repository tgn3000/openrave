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
#ifndef OPENRAVEPY_INTERNAL_TACTILEGEOMDATA_H
#define OPENRAVEPY_INTERNAL_TACTILEGEOMDATA_H

#define NO_IMPORT_ARRAY
#include "openravepy_int.h"

namespace openravepy
{

using py::object;

// TODO fill rest of fields
class PyTactileGeomData : public PySensorGeometry
{
public:
    PyTactileGeomData() {
        thickness = 0.0;
    }
    PyTactileGeomData(OPENRAVE_SHARED_PTR<SensorBase::TactileGeomData const> pgeom)
    {
        thickness = pgeom->thickness;
    }
    virtual ~PyTactileGeomData() {
    }
    virtual SensorBase::SensorType GetType() {
        return SensorBase::ST_Tactile;
    }
    virtual SensorBase::SensorGeometryPtr GetGeometry() {
        OPENRAVE_SHARED_PTR<SensorBase::TactileGeomData> geom(new SensorBase::TactileGeomData());
        geom->thickness = thickness;
        return geom;
    }

    dReal thickness;
};

} // namespace openravepy
#endif // OPENRAVEPY_INTERNAL_TACTILEGEOMDATA_H
