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
#ifndef OPENRAVEPY_INTERNAL_CAMERAGEOMDATA_H
#define OPENRAVEPY_INTERNAL_CAMERAGEOMDATA_H

#define NO_IMPORT_ARRAY
#include "openravepy_int.h"

namespace openravepy
{

using py::object;

class PyCameraGeomData : public PySensorGeometry
{
public:
    PyCameraGeomData() {
        width = 0;
        height = 0;
        measurement_time = 1;
        gain = 1;
    }
    PyCameraGeomData(OPENRAVE_SHARED_PTR<SensorBase::CameraGeomData const> pgeom) : intrinsics(pgeom->intrinsics)
    {
        hardware_id = pgeom->hardware_id;
        width = pgeom->width;
        height = pgeom->height;
        sensor_reference = pgeom->sensor_reference;
        target_region = pgeom->target_region;
        measurement_time = pgeom->measurement_time;
        gain = pgeom->gain;
    }
    virtual ~PyCameraGeomData() {
    }
    virtual SensorBase::SensorType GetType() {
        return SensorBase::ST_Camera;
    }
    virtual SensorBase::SensorGeometryPtr GetGeometry() {
        OPENRAVE_SHARED_PTR<SensorBase::CameraGeomData> geom(new SensorBase::CameraGeomData());
        geom->hardware_id = hardware_id;
        geom->width = width;
        geom->height = height;
        geom->intrinsics = intrinsics.GetCameraIntrinsics();
        geom->sensor_reference = sensor_reference;
        geom->target_region = target_region;
        geom->measurement_time = measurement_time;
        geom->gain = gain;
        return geom;
    }

    std::string hardware_id;
    PyCameraIntrinsics intrinsics;
    int width, height;
    std::string sensor_reference;
    std::string target_region;
    dReal measurement_time;
    dReal gain;
};

} // namespace openravepy
#endif // OPENRAVEPY_INTERNAL_CAMERAGEOMDATA_H