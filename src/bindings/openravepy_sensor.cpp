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
#define NO_IMPORT_ARRAY
#include "openravepy_int.h"

namespace openravepy {

SensorBasePtr GetSensor(PySensorBasePtr pysensor)
{
    return !pysensor ? SensorBasePtr() : pysensor->GetSensor();
}

PyInterfaceBasePtr toPySensor(SensorBasePtr psensor, PyEnvironmentBasePtr pyenv)
{
    return !psensor ? PyInterfaceBasePtr() : PyInterfaceBasePtr(new PySensorBase(psensor,pyenv));
}

object toPySensorData(SensorBasePtr psensor, PyEnvironmentBasePtr pyenv)
{
    if( !psensor ) {
        return object();
    }
    return object(PySensorBase(psensor,pyenv).GetSensorData());
}

PySensorBasePtr RaveCreateSensor(PyEnvironmentBasePtr pyenv, const std::string& name)
{
    SensorBasePtr p = OpenRAVE::RaveCreateSensor(GetEnvironment(pyenv), name);
    if( !p ) {
        return PySensorBasePtr();
    }
    return PySensorBasePtr(new PySensorBase(p,pyenv));
}

PySensorGeometryPtr toPySensorGeometry(SensorBase::SensorGeometryPtr pgeom)
{
    if( !!pgeom ) {
        if( pgeom->GetType() == SensorBase::ST_Camera ) {
            return PySensorGeometryPtr(new PyCameraGeomData(OPENRAVE_STATIC_POINTER_CAST<SensorBase::CameraGeomData const>(pgeom)));
        }
        else if( pgeom->GetType() == SensorBase::ST_Laser ) {
            return PySensorGeometryPtr(new PyLaserGeomData(OPENRAVE_STATIC_POINTER_CAST<SensorBase::LaserGeomData const>(pgeom)));
        }

    }
    return PySensorGeometryPtr();
}

PyCameraIntrinsicsPtr toPyCameraIntrinsics(const geometry::RaveCameraIntrinsics<float>& intrinsics)
{
    return PyCameraIntrinsicsPtr(new PyCameraIntrinsics(intrinsics));
}

PyCameraIntrinsicsPtr toPyCameraIntrinsics(const geometry::RaveCameraIntrinsics<double>& intrinsics)
{
    return PyCameraIntrinsicsPtr(new PyCameraIntrinsics(intrinsics));
}

BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(Configure_overloads, Configure, 1, 2)

void init_openravepy_sensor()
{
    {
        OPENRAVE_SHARED_PTR<PySensorBase::PySensorData> (PySensorBase::*GetSensorData1)() = &PySensorBase::GetSensorData;
        OPENRAVE_SHARED_PTR<PySensorBase::PySensorData> (PySensorBase::*GetSensorData2)(SensorBase::SensorType) = &PySensorBase::GetSensorData;
        scope sensor = class_<PySensorBase, OPENRAVE_SHARED_PTR<PySensorBase>, bases<PyInterfaceBase> >("Sensor", DOXY_CLASS(SensorBase), no_init)
                       .def("Configure",&PySensorBase::Configure, Configure_overloads(args("command","blocking"), DOXY_FN(SensorBase,Configure)))
                       .def("SimulationStep",&PySensorBase::SimulationStep, args("timeelapsed"), DOXY_FN(SensorBase,SimulationStep))
                       .def("GetSensorData",GetSensorData1, DOXY_FN(SensorBase,GetSensorData))
                       .def("GetSensorData",GetSensorData2, DOXY_FN(SensorBase,GetSensorData))
                       .def("CreateSensorData",&PySensorBase::CreateSensorData, DOXY_FN(SensorBase,CreateSensorData))
                       .def("GetSensorGeometry",&PySensorBase::GetSensorGeometry,DOXY_FN(SensorBase,GetSensorGeometry))
                       .def("SetSensorGeometry",&PySensorBase::SetSensorGeometry, DOXY_FN(SensorBase,SetSensorGeometry))
                       .def("SetTransform",&PySensorBase::SetTransform, DOXY_FN(SensorBase,SetTransform))
                       .def("GetTransform",&PySensorBase::GetTransform, DOXY_FN(SensorBase,GetTransform))
                       .def("GetTransformPose",&PySensorBase::GetTransformPose, DOXY_FN(SensorBase,GetTransform))
                       .def("GetName",&PySensorBase::GetName, DOXY_FN(SensorBase,GetName))
                       .def("SetName",&PySensorBase::SetName, DOXY_FN(SensorBase,SetName))
                       .def("Supports",&PySensorBase::Supports, DOXY_FN(SensorBase,Supports))
                       .def("__str__",&PySensorBase::__str__)
                       .def("__unicode__",&PySensorBase::__unicode__)
                       .def("__repr__",&PySensorBase::__repr__)
        ;

        class_<PyCameraIntrinsics, OPENRAVE_SHARED_PTR<PyCameraIntrinsics> >("CameraIntrinsics", DOXY_CLASS(geometry::RaveCameraIntrinsics))
        .def_readwrite("K",&PyCameraIntrinsics::K)
        .def_readwrite("distortion_model",&PyCameraIntrinsics::distortion_model)
        .def_readwrite("distortion_coeffs",&PyCameraIntrinsics::distortion_coeffs)
        .def_readwrite("focal_length",&PyCameraIntrinsics::focal_length)
        ;

        class_<PySensorBase::PySensorData, OPENRAVE_SHARED_PTR<PySensorBase::PySensorData> >("SensorData", DOXY_CLASS(SensorBase::SensorData),no_init)
        .def_readonly("type",&PySensorBase::PySensorData::type)
        .def_readonly("stamp",&PySensorBase::PySensorData::stamp)
        ;
        class_<PySensorBase::PyLaserSensorData, OPENRAVE_SHARED_PTR<PySensorBase::PyLaserSensorData>, bases<PySensorBase::PySensorData> >("LaserSensorData", DOXY_CLASS(SensorBase::LaserSensorData),no_init)
        .def_readonly("positions",&PySensorBase::PyLaserSensorData::positions)
        .def_readonly("ranges",&PySensorBase::PyLaserSensorData::ranges)
        .def_readonly("intensity",&PySensorBase::PyLaserSensorData::intensity)
        ;
        class_<PySensorBase::PyCameraSensorData, OPENRAVE_SHARED_PTR<PySensorBase::PyCameraSensorData>, bases<PySensorBase::PySensorData> >("CameraSensorData", DOXY_CLASS(SensorBase::CameraSensorData),no_init)
        .def_readonly("transform",&PySensorBase::PyCameraSensorData::transform)
        .def_readonly("imagedata",&PySensorBase::PyCameraSensorData::imagedata)
        .def_readonly("KK",&PySensorBase::PyCameraSensorData::KK)
        .def_readonly("intrinsics",&PySensorBase::PyCameraSensorData::intrinsics)
        ;
        class_<PySensorBase::PyJointEncoderSensorData, OPENRAVE_SHARED_PTR<PySensorBase::PyJointEncoderSensorData>, bases<PySensorBase::PySensorData> >("JointEncoderSensorData", DOXY_CLASS(SensorBase::JointEncoderSensorData),no_init)
        .def_readonly("encoderValues",&PySensorBase::PyJointEncoderSensorData::encoderValues)
        .def_readonly("encoderVelocity",&PySensorBase::PyJointEncoderSensorData::encoderVelocity)
        .def_readonly("resolution",&PySensorBase::PyJointEncoderSensorData::resolution)
        ;
        class_<PySensorBase::PyForce6DSensorData, OPENRAVE_SHARED_PTR<PySensorBase::PyForce6DSensorData>, bases<PySensorBase::PySensorData> >("Force6DSensorData", DOXY_CLASS(SensorBase::Force6DSensorData),no_init)
        .def_readonly("force",&PySensorBase::PyForce6DSensorData::force)
        .def_readonly("torque",&PySensorBase::PyForce6DSensorData::torque)
        ;
        class_<PySensorBase::PyIMUSensorData, OPENRAVE_SHARED_PTR<PySensorBase::PyIMUSensorData>, bases<PySensorBase::PySensorData> >("IMUSensorData", DOXY_CLASS(SensorBase::IMUSensorData),no_init)
        .def_readonly("rotation",&PySensorBase::PyIMUSensorData::rotation)
        .def_readonly("angular_velocity",&PySensorBase::PyIMUSensorData::angular_velocity)
        .def_readonly("linear_acceleration",&PySensorBase::PyIMUSensorData::linear_acceleration)
        .def_readonly("rotation_covariance",&PySensorBase::PyIMUSensorData::rotation_covariance)
        .def_readonly("angular_velocity_covariance",&PySensorBase::PyIMUSensorData::angular_velocity_covariance)
        .def_readonly("linear_acceleration_covariance",&PySensorBase::PyIMUSensorData::linear_acceleration_covariance)
        ;
        class_<PySensorBase::PyOdometrySensorData, OPENRAVE_SHARED_PTR<PySensorBase::PyOdometrySensorData>, bases<PySensorBase::PySensorData> >("OdometrySensorData", DOXY_CLASS(SensorBase::OdometrySensorData),no_init)
        .def_readonly("pose",&PySensorBase::PyOdometrySensorData::pose)
        .def_readonly("linear_velocity",&PySensorBase::PyOdometrySensorData::linear_velocity)
        .def_readonly("angular_velocity",&PySensorBase::PyOdometrySensorData::angular_velocity)
        .def_readonly("pose_covariance",&PySensorBase::PyOdometrySensorData::pose_covariance)
        .def_readonly("velocity_covariance",&PySensorBase::PyOdometrySensorData::velocity_covariance)
        .def_readonly("targetid",&PySensorBase::PyOdometrySensorData::targetid)
        ;
        class_<PySensorBase::PyTactileSensorData, OPENRAVE_SHARED_PTR<PySensorBase::PyTactileSensorData>, bases<PySensorBase::PySensorData> >("TactileSensorData", DOXY_CLASS(SensorBase::TactileSensorData),no_init)
        .def_readonly("forces",&PySensorBase::PyTactileSensorData::forces)
        .def_readonly("force_covariance",&PySensorBase::PyTactileSensorData::force_covariance)
        .def_readonly("positions",&PySensorBase::PyTactileSensorData::positions)
        .def_readonly("thickness",&PySensorBase::PyTactileSensorData::thickness)
        ;
        {
            class_<PySensorBase::PyActuatorSensorData, OPENRAVE_SHARED_PTR<PySensorBase::PyActuatorSensorData>, bases<PySensorBase::PySensorData> >("ActuatorSensorData", DOXY_CLASS(SensorBase::ActuatorSensorData),no_init)
            .def_readonly("state",&PySensorBase::PyActuatorSensorData::state)
            .def_readonly("measuredcurrent",&PySensorBase::PyActuatorSensorData::measuredcurrent)
            .def_readonly("measuredtemperature",&PySensorBase::PyActuatorSensorData::measuredtemperature)
            .def_readonly("appliedcurrent",&PySensorBase::PyActuatorSensorData::appliedcurrent)
            .def_readonly("maxtorque",&PySensorBase::PyActuatorSensorData::maxtorque)
            .def_readonly("maxcurrent",&PySensorBase::PyActuatorSensorData::maxcurrent)
            .def_readonly("nominalcurrent",&PySensorBase::PyActuatorSensorData::nominalcurrent)
            .def_readonly("maxvelocity",&PySensorBase::PyActuatorSensorData::maxvelocity)
            .def_readonly("maxacceleration",&PySensorBase::PyActuatorSensorData::maxacceleration)
            .def_readonly("maxjerk",&PySensorBase::PyActuatorSensorData::maxjerk)
            .def_readonly("staticfriction",&PySensorBase::PyActuatorSensorData::staticfriction)
            .def_readonly("viscousfriction",&PySensorBase::PyActuatorSensorData::viscousfriction)
            ;
            enum_<SensorBase::ActuatorSensorData::ActuatorState>("ActuatorState" DOXY_ENUM(ActuatorState))
            .value("Undefined",SensorBase::ActuatorSensorData::AS_Undefined)
            .value("Idle",SensorBase::ActuatorSensorData::AS_Idle)
            .value("Moving",SensorBase::ActuatorSensorData::AS_Moving)
            .value("Stalled",SensorBase::ActuatorSensorData::AS_Stalled)
            .value("Braked",SensorBase::ActuatorSensorData::AS_Braked)
            ;
        }

        enum_<SensorBase::SensorType>("Type" DOXY_ENUM(SensorType))
        .value("Invalid",SensorBase::ST_Invalid)
        .value("Laser",SensorBase::ST_Laser)
        .value("Camera",SensorBase::ST_Camera)
        .value("JointEncoder",SensorBase::ST_JointEncoder)
        .value("Force6D",SensorBase::ST_Force6D)
        .value("IMU",SensorBase::ST_IMU)
        .value("Odometry",SensorBase::ST_Odometry)
        .value("Tactile",SensorBase::ST_Tactile)
        .value("Actuator",SensorBase::ST_Actuator)
        ;
        enum_<SensorBase::ConfigureCommand>("ConfigureCommand" DOXY_ENUM(ConfigureCommand))
        .value("PowerOn",SensorBase::CC_PowerOn)
        .value("PowerOff",SensorBase::CC_PowerOff)
        .value("PowerCheck",SensorBase::CC_PowerCheck)
        .value("RenderDataOn",SensorBase::CC_RenderDataOn)
        .value("RenderDataOff",SensorBase::CC_RenderDataOff)
        .value("RenderDataCheck",SensorBase::CC_RenderDataCheck)
        .value("RenderGeometryOn",SensorBase::CC_RenderGeometryOn)
        .value("RenderGeometryOff",SensorBase::CC_RenderGeometryOff)
        .value("RenderGeometryCheck",SensorBase::CC_RenderGeometryCheck)
        ;
    }

    class_<PySensorGeometry, OPENRAVE_SHARED_PTR<PySensorGeometry>, boost::noncopyable >("SensorGeometry", DOXY_CLASS(PySensorGeometry),no_init)
    .def("GetType",boost::python::pure_virtual(&PySensorGeometry::GetType))
    ;
    class_<PyCameraGeomData, OPENRAVE_SHARED_PTR<PyCameraGeomData>, bases<PySensorGeometry> >("CameraGeomData", DOXY_CLASS(SensorBase::CameraGeomData))
    .def_readwrite("intrinsics",&PyCameraGeomData::intrinsics)
    .def_readwrite("hardware_id",&PyCameraGeomData::hardware_id)
    .def_readwrite("width",&PyCameraGeomData::width)
    .def_readwrite("height",&PyCameraGeomData::height)
    .def_readwrite("sensor_reference",&PyCameraGeomData::sensor_reference)
    .def_readwrite("target_region",&PyCameraGeomData::target_region)
    .def_readwrite("measurement_time",&PyCameraGeomData::measurement_time)
    .def_readwrite("gain",&PyCameraGeomData::gain)
    .def_readwrite("KK",&PyCameraGeomData::intrinsics) // deprecated
    ;

    class_<PyLaserGeomData, OPENRAVE_SHARED_PTR<PyLaserGeomData>, bases<PySensorGeometry> >("LaserGeomData", DOXY_CLASS(SensorBase::LaserGeomData))
    .def_readwrite("min_angle",&PyLaserGeomData::min_angle)
    .def_readwrite("max_angle",&PyLaserGeomData::max_angle)
    .def_readwrite("min_range",&PyLaserGeomData::min_range)
    .def_readwrite("max_range",&PyLaserGeomData::max_range)
    .def_readwrite("time_increment",&PyLaserGeomData::time_increment)
    .def_readwrite("time_scan",&PyLaserGeomData::time_scan)
    ;

    class_<PyJointEncoderGeomData, OPENRAVE_SHARED_PTR<PyJointEncoderGeomData>, bases<PySensorGeometry> >("JointEncoderGeomData", DOXY_CLASS(SensorBase::JointEncoderGeomData))
    .def_readwrite("resolution",&PyJointEncoderGeomData::resolution)
    ;

    class_<PyForce6DGeomData, OPENRAVE_SHARED_PTR<PyForce6DGeomData>, bases<PySensorGeometry> >("Force6DGeomData", DOXY_CLASS(SensorBase::Force6DGeomData))
    ;

    class_<PyIMUGeomData, OPENRAVE_SHARED_PTR<PyIMUGeomData>, bases<PySensorGeometry> >("IMUGeomData", DOXY_CLASS(SensorBase::IMUGeomData))
    .def_readwrite("time_measurement",&PyIMUGeomData::time_measurement)
    ;

    class_<PyOdometryGeomData, OPENRAVE_SHARED_PTR<PyOdometryGeomData>, bases<PySensorGeometry> >("OdometryGeomData", DOXY_CLASS(SensorBase::OdometryGeomData))
    .def_readwrite("targetid",&PyOdometryGeomData::targetid)
    ;

    class_<PyTactileGeomData, OPENRAVE_SHARED_PTR<PyTactileGeomData>, bases<PySensorGeometry> >("TactileGeomData", DOXY_CLASS(SensorBase::TactileGeomData))
    .def_readwrite("thickness",&PyTactileGeomData::thickness)
    ;

    class_<PyActuatorGeomData, OPENRAVE_SHARED_PTR<PyActuatorGeomData>, bases<PySensorGeometry> >("ActuatorGeomData", DOXY_CLASS(SensorBase::ActuatorGeomData))
    .def_readwrite("maxtorque",&PyActuatorGeomData::maxtorque)
    .def_readwrite("maxcurrent",&PyActuatorGeomData::maxcurrent)
    .def_readwrite("nominalcurrent",&PyActuatorGeomData::nominalcurrent)
    .def_readwrite("maxvelocity",&PyActuatorGeomData::maxvelocity)
    .def_readwrite("maxacceleration",&PyActuatorGeomData::maxacceleration)
    .def_readwrite("maxjerk",&PyActuatorGeomData::maxjerk)
    .def_readwrite("staticfriction",&PyActuatorGeomData::staticfriction)
    .def_readwrite("viscousfriction",&PyActuatorGeomData::viscousfriction)
    ;

    def("RaveCreateSensor",openravepy::RaveCreateSensor,args("env","name"),DOXY_FN1(RaveCreateSensor));
}

}
