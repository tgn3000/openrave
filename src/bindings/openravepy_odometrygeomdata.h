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
#ifndef OPENRAVEPY_INTERNAL_ODOMETRYGEOMDATA_H
#define OPENRAVEPY_INTERNAL_ODOMETRYGEOMDATA_H

#define NO_IMPORT_ARRAY
#include "openravepy_int.h"

namespace openravepy
{

using py::object;

class PyOdometryGeomData : public PySensorGeometry
{
public:
    PyOdometryGeomData() {
    }
    PyOdometryGeomData(OPENRAVE_SHARED_PTR<SensorBase::OdometryGeomData const> pgeom)
    {
        targetid = pgeom->targetid;
    }
    virtual ~PyOdometryGeomData() {
    }
    virtual SensorBase::SensorType GetType() {
        return SensorBase::ST_Odometry;
    }
    virtual SensorBase::SensorGeometryPtr GetGeometry() {
        OPENRAVE_SHARED_PTR<SensorBase::OdometryGeomData> geom(new SensorBase::OdometryGeomData());
        geom->targetid = targetid;
        return geom;
    }

    std::string targetid;
};

} // namespace openravepy

#endif // OPENRAVEPY_INTERNAL_ODOMETRYGEOMDATA_H