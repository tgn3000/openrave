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
#ifndef OPENRAVEPY_INTERNAL_ELECTRICMOTORACTUATORINFO_H
#define OPENRAVEPY_INTERNAL_ELECTRICMOTORACTUATORINFO_H

#define NO_IMPORT_ARRAY
#include "openravepy_int.h"
#include "openravepy_linkinfo.h"
#include "openravepy_geometryinfo.h"

namespace openravepy
{

using py::object;

class PyElectricMotorActuatorInfo
{
public:
    PyElectricMotorActuatorInfo() {
        gear_ratio = 0;
        assigned_power_rating = 0;
        max_speed = 0;
        no_load_speed = 0;
        stall_torque = 0;
        max_instantaneous_torque = 0;
        nominal_torque = 0;
        rotor_inertia = 0;
        torque_constant = 0;
        nominal_voltage = 0;
        speed_constant = 0;
        starting_current = 0;
        terminal_resistance = 0;
        coloumb_friction = 0;
        viscous_friction = 0;
    }
    PyElectricMotorActuatorInfo(const ElectricMotorActuatorInfo& info) {
        model_type = info.model_type;
        gear_ratio = info.gear_ratio;
        assigned_power_rating = info.assigned_power_rating;
        max_speed = info.max_speed;
        no_load_speed = info.no_load_speed;
        stall_torque = info.stall_torque;
        max_instantaneous_torque = info.max_instantaneous_torque;
        FOREACH(itpoint, info.nominal_speed_torque_points) {
            nominal_speed_torque_points.append(py::make_tuple(itpoint->first, itpoint->second));
        }
        FOREACH(itpoint, info.max_speed_torque_points) {
            max_speed_torque_points.append(py::make_tuple(itpoint->first, itpoint->second));
        }
        nominal_torque = info.nominal_torque;
        rotor_inertia = info.rotor_inertia;
        torque_constant = info.torque_constant;
        nominal_voltage = info.nominal_voltage;
        speed_constant = info.speed_constant;
        starting_current = info.starting_current;
        terminal_resistance = info.terminal_resistance;
        coloumb_friction = info.coloumb_friction;
        viscous_friction = info.viscous_friction;
    }

    ElectricMotorActuatorInfoPtr GetElectricMotorActuatorInfo() {
        ElectricMotorActuatorInfoPtr pinfo(new ElectricMotorActuatorInfo());
        ElectricMotorActuatorInfo& info = *pinfo;
        info.model_type = model_type;
        info.gear_ratio = gear_ratio;
        info.assigned_power_rating = assigned_power_rating;
        info.max_speed = max_speed;
        info.no_load_speed = no_load_speed;
        info.stall_torque = stall_torque;
        info.max_instantaneous_torque = max_instantaneous_torque;
        if( !IS_PYTHONOBJECT_NONE(nominal_speed_torque_points) ) {
            size_t num = len(nominal_speed_torque_points);
            for(size_t i = 0; i < num; ++i) {
                info.nominal_speed_torque_points.emplace_back(
                    nominal_speed_torque_points[i][0].cast<dReal>(),
                    nominal_speed_torque_points[i][1].cast<dReal>()
                );
            }
        }
        if( !IS_PYTHONOBJECT_NONE(max_speed_torque_points) ) {
            size_t num = len(max_speed_torque_points);
            for(size_t i = 0; i < num; ++i) {
                info.max_speed_torque_points.emplace_back(
                    max_speed_torque_points[i][0].cast<dReal>(),
                    max_speed_torque_points[i][1].cast<dReal>()
                );
            }
        }
        info.nominal_torque = nominal_torque;
        info.rotor_inertia = rotor_inertia;
        info.torque_constant = torque_constant;
        info.nominal_voltage = nominal_voltage;
        info.speed_constant = speed_constant;
        info.starting_current = starting_current;
        info.terminal_resistance = terminal_resistance;
        info.coloumb_friction = coloumb_friction;
        info.viscous_friction = viscous_friction;
        return pinfo;
    }

    std::string model_type;
    dReal gear_ratio;
    dReal assigned_power_rating;
    dReal max_speed;
    dReal no_load_speed;
    dReal stall_torque;
    dReal max_instantaneous_torque;
    py::list nominal_speed_torque_points, max_speed_torque_points;
    dReal nominal_torque;
    dReal rotor_inertia;
    dReal torque_constant;
    dReal nominal_voltage;
    dReal speed_constant;
    dReal starting_current;
    dReal terminal_resistance;
    dReal coloumb_friction;
    dReal viscous_friction;
};
typedef OPENRAVE_SHARED_PTR<PyElectricMotorActuatorInfo> PyElectricMotorActuatorInfoPtr;

} // namespace openravepy

#endif // OPENRAVEPY_INTERNAL_ELECTRICMOTORACTUATORINFO_H
