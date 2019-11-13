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
#ifndef OPENRAVEPY_INTERNAL_ACTUATORGEOMDATA_H
#define OPENRAVEPY_INTERNAL_ACTUATORGEOMDATA_H

#define NO_IMPORT_ARRAY
#include "openravepy_int.h"

namespace openravepy
{

using py::object;

class PyActuatorGeomData : public PySensorGeometry
{
public:
    PyActuatorGeomData() {
        maxtorque = 0.0;
        maxcurrent = 0.0;
        nominalcurrent = 0.0;
        maxvelocity = 0.0;
        maxacceleration = 0.0;
        maxjerk = 0.0;
        staticfriction = 0.0;
        viscousfriction = 0.0;
    }
    PyActuatorGeomData(OPENRAVE_SHARED_PTR<SensorBase::ActuatorGeomData const> pgeom)
    {
        maxtorque = pgeom->maxtorque;
        maxcurrent = pgeom->maxcurrent;
        nominalcurrent = pgeom->nominalcurrent;
        maxvelocity = pgeom->maxvelocity;
        maxacceleration = pgeom->maxacceleration;
        maxjerk = pgeom->maxjerk;
        staticfriction = pgeom->staticfriction;
        viscousfriction = pgeom->viscousfriction;
    }
    virtual ~PyActuatorGeomData() {
    }
    virtual SensorBase::SensorType GetType() {
        return SensorBase::ST_Actuator;
    }
    virtual SensorBase::SensorGeometryPtr GetGeometry() {
        OPENRAVE_SHARED_PTR<SensorBase::ActuatorGeomData> geom(new SensorBase::ActuatorGeomData());
        geom->maxtorque = maxtorque;
        geom->maxcurrent = maxcurrent;
        geom->nominalcurrent = nominalcurrent;
        geom->maxvelocity = maxvelocity;
        geom->maxacceleration = maxacceleration;
        geom->maxjerk = maxjerk;
        geom->staticfriction = staticfriction;
        geom->viscousfriction = viscousfriction;
        return geom;
    }

    dReal maxtorque, maxcurrent, nominalcurrent, maxvelocity, maxacceleration, maxjerk, staticfriction, viscousfriction;
};

} // namespace openravepy
#endif // OPENRAVEPY_INTERNAL_ACTUATORGEOMDATA_H
