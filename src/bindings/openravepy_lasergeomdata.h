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
#ifndef OPENRAVEPY_INTERNAL_LASERGEOMDATA_H
#define OPENRAVEPY_INTERNAL_LASERGEOMDATA_H

#define NO_IMPORT_ARRAY
#include "openravepy_int.h"

namespace openravepy
{

using py::object;

class PyLaserGeomData : public PySensorGeometry
{
public:
    PyLaserGeomData() {
        min_angle = py::make_tuple(0.0, 0.0);
        max_angle = py::make_tuple(0.0, 0.0);
        min_range = 0.0;
        max_range = 0.0;
        time_increment = 0.0;
        time_scan = 0.0;
    }
    PyLaserGeomData(OPENRAVE_SHARED_PTR<SensorBase::LaserGeomData const> pgeom)
    {
        min_angle = py::make_tuple(pgeom->min_angle[0], pgeom->min_angle[1]);
        max_angle = py::make_tuple(pgeom->max_angle[0], pgeom->max_angle[1]);
        min_range = pgeom->min_range;
        max_range = pgeom->max_range;
        time_increment = pgeom->time_increment;
        time_scan = pgeom->time_scan;
    }
    virtual ~PyLaserGeomData() {
    }
    virtual SensorBase::SensorType GetType() {
        return SensorBase::ST_Laser;
    }
    virtual SensorBase::SensorGeometryPtr GetGeometry() {
        OPENRAVE_SHARED_PTR<SensorBase::LaserGeomData> geom(new SensorBase::LaserGeomData());
        geom->min_angle[0] = min_angle[0].cast<dReal>();
        geom->min_angle[1] = min_angle[1].cast<dReal>();
        geom->max_angle[0] = max_angle[0].cast<dReal>();
        geom->max_angle[1] = max_angle[1].cast<dReal>();
        geom->min_range = min_range;
        geom->max_range = max_range;
        geom->time_increment = time_increment;
        geom->time_scan = time_scan;
        return geom;
    }

    py::tuple min_angle, max_angle, resolution;
    dReal min_range, max_range, time_increment, time_scan;
};
} // namespace openravepy
#endif // OPENRAVEPY_INTERNAL_LASERGEOMDATA_H