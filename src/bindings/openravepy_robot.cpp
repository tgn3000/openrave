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
#define NO_IMPORT_ARRAY
#include "openravepy_robot.h"
#include "openravepy_kinbody.h"
#include "openravepy_manipulatorinfo.h"
#include "openravepy_attachedsensorinfo.h"
#include "openravepy_connectedbodyinfo.h"
#include "openravepy_controllerbase.h"
#include "openravepy_collisionreport.h"
#include "openravepy_iksolverbase.h"
#include "openravepy_environment.h"

namespace openravepy {

PyManipulatorInfoPtr toPyManipulatorInfo(const RobotBase::ManipulatorInfo& manipulatorinfo)
{
    return PyManipulatorInfoPtr(new PyManipulatorInfo(manipulatorinfo));
}

PyAttachedSensorInfoPtr toPyAttachedSensorInfo(const RobotBase::AttachedSensorInfo& attachedSensorinfo)
{
    return PyAttachedSensorInfoPtr(new PyAttachedSensorInfo(attachedSensorinfo));
}

PyConnectedBodyInfoPtr toPyConnectedBodyInfo(const RobotBase::ConnectedBodyInfo& connectedBodyInfo, PyEnvironmentBasePtr pyenv)
{
    return PyConnectedBodyInfoPtr(new PyConnectedBodyInfo(connectedBodyInfo, pyenv));
}

// class ManipulatorInfo_pickle_suite : public pickle_suite
// {
// public:
//     static boost::python::tuple getstate(const PyManipulatorInfo& r)
//     {
//         return py::make_tuple(r._name, r._sBaseLinkName, r._sEffectorLinkName, r._tLocalTool, r._vChuckingDirection, r._vdirection, r._sIkSolverXMLId, r._vGripperJointNames);
//     }
//     static void setstate(PyManipulatorInfo& r, boost::python::tuple state) {
//         r._name = state[0];
//         r._sBaseLinkName = state[1];
//         r._sEffectorLinkName = state[2];
//         r._tLocalTool = state[3];
//         r._vChuckingDirection = state[4];
//         r._vdirection = state[5];
//         r._sIkSolverXMLId = boost::python::extract<std::string>(state[6]);
//         r._vGripperJointNames = state[7];
//     }
// };

RobotBasePtr GetRobot(object o)
{
    try {
        PyRobotBasePtr pyrobot = o.cast<PyRobotBasePtr>();
        if(pyrobot != nullptr) {
            return GetRobot((PyRobotBasePtr)pyrobot);
        }
    }
    catch (...) {}
    return RobotBasePtr();
}

RobotBasePtr GetRobot(PyRobotBasePtr pyrobot)
{
    return !pyrobot ? RobotBasePtr() : pyrobot->GetRobot();
}

PyInterfaceBasePtr toPyRobot(RobotBasePtr probot, PyEnvironmentBasePtr pyenv)
{
    return !probot ? PyInterfaceBasePtr() : PyInterfaceBasePtr(new PyRobotBase(probot,pyenv));
}

RobotBase::ManipulatorPtr GetRobotManipulator(object o)
{
    try {
        PyRobotBase::PyManipulatorPtr pymanipulator = o.cast<PyRobotBase::PyManipulatorPtr>();
        if(pymanipulator != nullptr) {
            return ((PyRobotBase::PyManipulatorPtr)pymanipulator)->GetManipulator();
        }
    }
    catch(...) {}
    return RobotBase::ManipulatorPtr();
}

object toPyRobotManipulator(RobotBase::ManipulatorPtr pmanip, PyEnvironmentBasePtr pyenv)
{
    return !pmanip ? object() : py::cast(PyRobotBase::PyManipulatorPtr(new PyRobotBase::PyManipulator(pmanip,pyenv)));
}

PyRobotBasePtr RaveCreateRobot(PyEnvironmentBasePtr pyenv, const std::string& name)
{
    RobotBasePtr p = OpenRAVE::RaveCreateRobot(openravepy::GetEnvironment(pyenv), name);
    if( !p ) {
        return PyRobotBasePtr();
    }
    return PyRobotBasePtr(new PyRobotBase(p,pyenv));
}

// BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(GetIkParameterization_overloads, GetIkParameterization, 1, 2)
// BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(CheckEndEffectorCollision_overloads, CheckEndEffectorCollision, 1, 3)
// BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(CheckEndEffectorSelfCollision_overloads, CheckEndEffectorSelfCollision, 1, 4)
// BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(FindIKSolution_overloads, FindIKSolution, 2, 4)
// BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(FindIKSolutionFree_overloads, FindIKSolution, 3, 5)
// BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(FindIKSolutions_overloads, FindIKSolutions, 2, 4)
// BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(FindIKSolutionsFree_overloads, FindIKSolutions, 3, 5)
// BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(GetArmConfigurationSpecification_overloads, GetArmConfigurationSpecification, 0, 1)
// BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(GetIkConfigurationSpecification_overloads, GetIkConfigurationSpecification, 1, 2)
// BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(CreateRobotStateSaver_overloads, CreateRobotStateSaver, 0,1)
// BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(SetActiveDOFValues_overloads, SetActiveDOFValues, 1,2)
// BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(SetActiveDOFVelocities_overloads, SetActiveDOFVelocities, 1,2)
// BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(AddManipulator_overloads, AddManipulator, 1,2)
// BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(AddAttachedSensor_overloads, AddAttachedSensor, 1,2)
// BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(AddConnectedBody_overloads, AddConnectedBody, 1,2)
// BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(GetActiveConfigurationSpecification_overloads, GetActiveConfigurationSpecification, 0, 1)
// BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(Restore_overloads, Restore, 0,1)
// BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(Init_overloads, Init, 4,5)
// BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(UpdateInfo_overloads, UpdateInfo, 0,1)
// BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(UpdateAndGetInfo_overloads, UpdateAndGetInfo, 0,1)
// BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(CheckLinkSelfCollision_overloads, CheckLinkSelfCollision, 2, 3)

void init_openravepy_robot(py::module& m)
{
    object dofaffine = py::enum_<DOFAffine>(m, "DOFAffine" DOXY_ENUM(DOFAffine))
                       .value("NoTransform",DOF_NoTransform)
                       .value("X",DOF_X)
                       .value("Y",DOF_Y)
                       .value("Z",DOF_Z)
                       .value("RotationAxis",DOF_RotationAxis)
                       .value("Rotation3D",DOF_Rotation3D)
                       .value("RotationQuat",DOF_RotationQuat)
                       .value("RotationMask",DOF_RotationMask)
                       .value("Transform",DOF_Transform)
    ;


    object manipulatorinfo = py::class_<PyManipulatorInfo, OPENRAVE_SHARED_PTR<PyManipulatorInfo> >(m, "ManipulatorInfo", DOXY_CLASS(RobotBase::ManipulatorInfo))
                             .def_readwrite("_name",&PyManipulatorInfo::_name)
                             .def_readwrite("_sBaseLinkName",&PyManipulatorInfo::_sBaseLinkName)
                             .def_readwrite("_sEffectorLinkName",&PyManipulatorInfo::_sEffectorLinkName)
                             .def_readwrite("_tLocalTool",&PyManipulatorInfo::_tLocalTool)
                             .def_readwrite("_vChuckingDirection",&PyManipulatorInfo::_vChuckingDirection)
                             .def_readwrite("_vClosingDirection",&PyManipulatorInfo::_vChuckingDirection) // back compat
                             .def_readwrite("_vdirection",&PyManipulatorInfo::_vdirection)
                             .def_readwrite("_sIkSolverXMLId",&PyManipulatorInfo::_sIkSolverXMLId)
                             .def_readwrite("_vGripperJointNames",&PyManipulatorInfo::_vGripperJointNames)
                             // .def_pickle(ManipulatorInfo_pickle_suite())
    ;

    object attachedsensorinfo = py::class_<PyAttachedSensorInfo, OPENRAVE_SHARED_PTR<PyAttachedSensorInfo> >(m, "AttachedSensorInfo", DOXY_CLASS(RobotBase::AttachedSensorInfo))
                                .def_readwrite("_name", &PyAttachedSensorInfo::_name)
                                .def_readwrite("_linkname", &PyAttachedSensorInfo::_linkname)
                                .def_readwrite("_trelative", &PyAttachedSensorInfo::_trelative)
                                .def_readwrite("_sensorname", &PyAttachedSensorInfo::_sensorname)
                                .def_readwrite("_sensorgeometry", &PyAttachedSensorInfo::_sensorgeometry)
    ;

    object connectedbodyinfo = py::class_<PyConnectedBodyInfo, OPENRAVE_SHARED_PTR<PyConnectedBodyInfo> >(m, "ConnectedBodyInfo", DOXY_CLASS(RobotBase::ConnectedBodyInfo))
                               .def_readwrite("_name", &PyConnectedBodyInfo::_name)
                               .def_readwrite("_linkname", &PyConnectedBodyInfo::_linkname)
                               .def_readwrite("_trelative", &PyConnectedBodyInfo::_trelative)
                               .def_readwrite("_url", &PyConnectedBodyInfo::_url)
                               .def_readwrite("_linkInfos", &PyConnectedBodyInfo::_linkInfos)
                               .def_readwrite("_jointInfos", &PyConnectedBodyInfo::_jointInfos)
                               .def_readwrite("_manipulatorInfos", &PyConnectedBodyInfo::_manipulatorInfos)
                               .def_readwrite("_attachedSensorInfos", &PyConnectedBodyInfo::_attachedSensorInfos)
    ;

    {
        void (PyRobotBase::*psetactivedofs1)(const object&) = &PyRobotBase::SetActiveDOFs;
        void (PyRobotBase::*psetactivedofs2)(const object&, int) = &PyRobotBase::SetActiveDOFs;
        void (PyRobotBase::*psetactivedofs3)(const object&, int, object) = &PyRobotBase::SetActiveDOFs;

        bool (PyRobotBase::*pgrab1)(PyKinBodyPtr) = &PyRobotBase::Grab;
        bool (PyRobotBase::*pgrab2)(PyKinBodyPtr,object) = &PyRobotBase::Grab;

        PyRobotBase::PyManipulatorPtr (PyRobotBase::*setactivemanipulator2)(const std::string&) = &PyRobotBase::SetActiveManipulator;
        PyRobotBase::PyManipulatorPtr (PyRobotBase::*setactivemanipulator3)(PyRobotBase::PyManipulatorPtr) = &PyRobotBase::SetActiveManipulator;

        object (PyRobotBase::*GetManipulators1)() = &PyRobotBase::GetManipulators;
        object (PyRobotBase::*GetManipulators2)(const string &) = &PyRobotBase::GetManipulators;
        bool (PyRobotBase::*setcontroller1)(PyControllerBasePtr,const string &) = &PyRobotBase::SetController;
        bool (PyRobotBase::*setcontroller2)(PyControllerBasePtr,object,int) = &PyRobotBase::SetController;
        bool (PyRobotBase::*setcontroller3)(PyControllerBasePtr) = &PyRobotBase::SetController;
        bool (PyRobotBase::*initrobot)(object, object, object, object, const std::string&) = &PyRobotBase::Init;
        object robot = py::class_<PyRobotBase, OPENRAVE_SHARED_PTR<PyRobotBase>, PyKinBody >(m, "Robot", DOXY_CLASS(RobotBase))
                      .def(py::init<RobotBasePtr, PyEnvironmentBasePtr>())
                      .def(py::init<const PyRobotBase&>())
                      .def("Init", initrobot, py::arg("linkinfos"), py::arg("jointinfos"), py::arg( "manipinfos"), py::arg( "attachedsensorinfos"), py::arg( "uri"), DOXY_FN(RobotBase, Init))
                      .def("GetManipulators",GetManipulators1, DOXY_FN(RobotBase,GetManipulators))
                      .def("GetManipulators",GetManipulators2,py::arg("manipname"), DOXY_FN(RobotBase,GetManipulators))
                      .def("GetManipulator",&PyRobotBase::GetManipulator,py::arg("manipname"), "Return the manipulator whose name matches")
                      .def("SetActiveManipulator",setactivemanipulator2,py::arg("manipname"), DOXY_FN(RobotBase,SetActiveManipulator "const std::string"))
                      .def("SetActiveManipulator",setactivemanipulator3,py::arg("manip"), "Set the active manipulator given a pointer")
                      .def("GetActiveManipulator",&PyRobotBase::GetActiveManipulator, DOXY_FN(RobotBase,GetActiveManipulator))
                      .def("AddManipulator",&PyRobotBase::AddManipulator, py::arg("manipinfo"), py::arg( "removeduplicate"), DOXY_FN(RobotBase,AddManipulator))
                      .def("AddAttachedSensor",&PyRobotBase::AddAttachedSensor, py::arg("attachedsensorinfo"), py::arg( "removeduplicate"), DOXY_FN(RobotBase,AddAttachedSensor))
                      .def("RemoveAttachedSensor",&PyRobotBase::RemoveAttachedSensor, py::arg("attsensor"), DOXY_FN(RobotBase,RemoveAttachedSensor))
                      .def("RemoveManipulator",&PyRobotBase::RemoveManipulator, py::arg("manip"), DOXY_FN(RobotBase,RemoveManipulator))
                      .def("GetAttachedSensors",&PyRobotBase::GetAttachedSensors, DOXY_FN(RobotBase,GetAttachedSensors))
                      .def("GetAttachedSensor",&PyRobotBase::GetAttachedSensor,py::arg("sensorname"), "Return the attached sensor whose name matches")
                      .def("GetSensors",&PyRobotBase::GetSensors)
                      .def("GetSensor",&PyRobotBase::GetSensor,py::arg("sensorname"))
                      .def("AddConnectedBody",&PyRobotBase::AddConnectedBody, py::arg("connectedbodyinfo"), py::arg( "removeduplicate"), DOXY_FN(RobotBase,AddConnectedBody))
                      .def("RemoveConnectedBody",&PyRobotBase::RemoveConnectedBody, py::arg("connectedbody"), DOXY_FN(RobotBase,RemoveConnectedBody))
                      .def("GetConnectedBodies",&PyRobotBase::GetConnectedBodies, DOXY_FN(RobotBase,GetConnectedBodies))
                      .def("GetConnectedBody",&PyRobotBase::GetConnectedBody, py::arg("bodyname"), DOXY_FN(RobotBase,GetConnectedBody))
                      .def("GetConnectedBodyActiveStates",&PyRobotBase::GetConnectedBodyActiveStates, DOXY_FN(RobotBase,GetConnectedBodyActiveStates))
                      .def("SetConnectedBodyActiveStates",&PyRobotBase::SetConnectedBodyActiveStates, DOXY_FN(RobotBase,SetConnectedBodyActiveStates))
                      .def("GetController",&PyRobotBase::GetController, DOXY_FN(RobotBase,GetController))
                      .def("SetController",setcontroller1, py::arg("robot"), py::arg("args"), DOXY_FN(RobotBase,SetController))
                      .def("SetController",setcontroller2, py::arg("robot"), py::arg("dofindices"), py::arg("controltransform"), DOXY_FN(RobotBase,SetController))
                      .def("SetController",setcontroller3, py::arg("robot"), DOXY_FN(RobotBase,SetController))
                      .def("SetActiveDOFs",psetactivedofs1,py::arg("dofindices"), DOXY_FN(RobotBase,SetActiveDOFs "const std::vector; int"))
                      .def("SetActiveDOFs",psetactivedofs2,py::arg("dofindices"), py::arg("affine"), DOXY_FN(RobotBase,SetActiveDOFs "const std::vector; int"))
                      .def("SetActiveDOFs",psetactivedofs3,py::arg("dofindices"), py::arg("affine"), py::arg("rotationaxis"), DOXY_FN(RobotBase,SetActiveDOFs "const std::vector; int; const Vector"))
                      .def("GetActiveDOF",&PyRobotBase::GetActiveDOF, DOXY_FN(RobotBase,GetActiveDOF))
                      .def("GetAffineDOF",&PyRobotBase::GetAffineDOF, DOXY_FN(RobotBase,GetAffineDOF))
                      .def("GetAffineDOFIndex",&PyRobotBase::GetAffineDOFIndex,py::arg("index"), DOXY_FN(RobotBase,GetAffineDOFIndex))
                      .def("GetAffineRotationAxis",&PyRobotBase::GetAffineRotationAxis, DOXY_FN(RobotBase,GetAffineRotationAxis))
                      .def("SetAffineTranslationLimits",&PyRobotBase::SetAffineTranslationLimits,py::arg("lower"), py::arg("upper"), DOXY_FN(RobotBase,SetAffineTranslationLimits))
                      .def("SetAffineRotationAxisLimits",&PyRobotBase::SetAffineRotationAxisLimits,py::arg("lower"), py::arg("upper"), DOXY_FN(RobotBase,SetAffineRotationAxisLimits))
                      .def("SetAffineRotation3DLimits",&PyRobotBase::SetAffineRotation3DLimits,py::arg("lower"), py::arg("upper"), DOXY_FN(RobotBase,SetAffineRotation3DLimits))
                      .def("SetAffineRotationQuatLimits",&PyRobotBase::SetAffineRotationQuatLimits,py::arg("quatangle"), DOXY_FN(RobotBase,SetAffineRotationQuatLimits))
                      .def("SetAffineTranslationMaxVels",&PyRobotBase::SetAffineTranslationMaxVels,py::arg("vels"), DOXY_FN(RobotBase,SetAffineTranslationMaxVels))
                      .def("SetAffineRotationAxisMaxVels",&PyRobotBase::SetAffineRotationAxisMaxVels,py::arg("velocity"), DOXY_FN(RobotBase,SetAffineRotationAxisMaxVels))
                      .def("SetAffineRotation3DMaxVels",&PyRobotBase::SetAffineRotation3DMaxVels,py::arg("velocity"), DOXY_FN(RobotBase,SetAffineRotation3DMaxVels))
                      .def("SetAffineRotationQuatMaxVels",&PyRobotBase::SetAffineRotationQuatMaxVels,py::arg("velocity"), DOXY_FN(RobotBase,SetAffineRotationQuatMaxVels))
                      .def("SetAffineTranslationResolution",&PyRobotBase::SetAffineTranslationResolution,py::arg("resolution"), DOXY_FN(RobotBase,SetAffineTranslationResolution))
                      .def("SetAffineRotationAxisResolution",&PyRobotBase::SetAffineRotationAxisResolution,py::arg("resolution"), DOXY_FN(RobotBase,SetAffineRotationAxisResolution))
                      .def("SetAffineRotation3DResolution",&PyRobotBase::SetAffineRotation3DResolution,py::arg("resolution"), DOXY_FN(RobotBase,SetAffineRotation3DResolution))
                      .def("SetAffineRotationQuatResolution",&PyRobotBase::SetAffineRotationQuatResolution,py::arg("resolution"), DOXY_FN(RobotBase,SetAffineRotationQuatResolution))
                      .def("SetAffineTranslationWeights",&PyRobotBase::SetAffineTranslationWeights,py::arg("weights"), DOXY_FN(RobotBase,SetAffineTranslationWeights))
                      .def("SetAffineRotationAxisWeights",&PyRobotBase::SetAffineRotationAxisWeights,py::arg("weights"), DOXY_FN(RobotBase,SetAffineRotationAxisWeights))
                      .def("SetAffineRotation3DWeights",&PyRobotBase::SetAffineRotation3DWeights,py::arg("weights"), DOXY_FN(RobotBase,SetAffineRotation3DWeights))
                      .def("SetAffineRotationQuatWeights",&PyRobotBase::SetAffineRotationQuatWeights,py::arg("weights"), DOXY_FN(RobotBase,SetAffineRotationQuatWeights))
                      .def("GetAffineTranslationLimits",&PyRobotBase::GetAffineTranslationLimits, DOXY_FN(RobotBase,GetAffineTranslationLimits))
                      .def("GetAffineRotationAxisLimits",&PyRobotBase::GetAffineRotationAxisLimits, DOXY_FN(RobotBase,GetAffineRotationAxisLimits))
                      .def("GetAffineRotation3DLimits",&PyRobotBase::GetAffineRotation3DLimits, DOXY_FN(RobotBase,GetAffineRotation3DLimits))
                      .def("GetAffineRotationQuatLimits",&PyRobotBase::GetAffineRotationQuatLimits, DOXY_FN(RobotBase,GetAffineRotationQuatLimits))
                      .def("GetAffineTranslationMaxVels",&PyRobotBase::GetAffineTranslationMaxVels, DOXY_FN(RobotBase,GetAffineTranslationMaxVels))
                      .def("GetAffineRotationAxisMaxVels",&PyRobotBase::GetAffineRotationAxisMaxVels, DOXY_FN(RobotBase,GetAffineRotationAxisMaxVels))
                      .def("GetAffineRotation3DMaxVels",&PyRobotBase::GetAffineRotation3DMaxVels, DOXY_FN(RobotBase,GetAffineRotation3DMaxVels))
                      .def("GetAffineRotationQuatMaxVels",&PyRobotBase::GetAffineRotationQuatMaxVels, DOXY_FN(RobotBase,GetAffineRotationQuatMaxVels))
                      .def("GetAffineTranslationResolution",&PyRobotBase::GetAffineTranslationResolution, DOXY_FN(RobotBase,GetAffineTranslationResolution))
                      .def("GetAffineRotationAxisResolution",&PyRobotBase::GetAffineRotationAxisResolution, DOXY_FN(RobotBase,GetAffineRotationAxisResolution))
                      .def("GetAffineRotation3DResolution",&PyRobotBase::GetAffineRotation3DResolution, DOXY_FN(RobotBase,GetAffineRotation3DResolution))
                      .def("GetAffineRotationQuatResolution",&PyRobotBase::GetAffineRotationQuatResolution, DOXY_FN(RobotBase,GetAffineRotationQuatResolution))
                      .def("GetAffineTranslationWeights",&PyRobotBase::GetAffineTranslationWeights, DOXY_FN(RobotBase,GetAffineTranslationWeights))
                      .def("GetAffineRotationAxisWeights",&PyRobotBase::GetAffineRotationAxisWeights, DOXY_FN(RobotBase,GetAffineRotationAxisWeights))
                      .def("GetAffineRotation3DWeights",&PyRobotBase::GetAffineRotation3DWeights, DOXY_FN(RobotBase,GetAffineRotation3DWeights))
                      .def("GetAffineRotationQuatWeights",&PyRobotBase::GetAffineRotationQuatWeights, DOXY_FN(RobotBase,GetAffineRotationQuatWeights))
                      .def("SetActiveDOFValues",&PyRobotBase::SetActiveDOFValues, py::arg("values"), py::arg("checklimits"), DOXY_FN(RobotBase,SetActiveDOFValues))
                      .def("GetActiveDOFValues",&PyRobotBase::GetActiveDOFValues, DOXY_FN(RobotBase,GetActiveDOFValues))
                      .def("GetActiveDOFWeights",&PyRobotBase::GetActiveDOFWeights, DOXY_FN(RobotBase,GetActiveDOFWeights))
                      .def("SetActiveDOFVelocities",&PyRobotBase::SetActiveDOFVelocities, py::arg("velocities"), py::arg("checklimits"), DOXY_FN(RobotBase,SetActiveDOFVelocities))
                      .def("GetActiveDOFVelocities",&PyRobotBase::GetActiveDOFVelocities, DOXY_FN(RobotBase,GetActiveDOFVelocities))
                      .def("GetActiveDOFLimits",&PyRobotBase::GetActiveDOFLimits, DOXY_FN(RobotBase,GetActiveDOFLimits))
                      .def("GetActiveDOFMaxVel",&PyRobotBase::GetActiveDOFMaxVel, DOXY_FN(RobotBase,GetActiveDOFMaxVel))
                      .def("GetActiveDOFMaxAccel",&PyRobotBase::GetActiveDOFMaxAccel, DOXY_FN(RobotBase,GetActiveDOFMaxAccel))
                      .def("GetActiveDOFMaxJerk",&PyRobotBase::GetActiveDOFMaxJerk, DOXY_FN(RobotBase,GetActiveDOFMaxJerk))
                      .def("GetActiveDOFHardMaxVel",&PyRobotBase::GetActiveDOFHardMaxVel, DOXY_FN(RobotBase,GetActiveDOFHardMaxVel))
                      .def("GetActiveDOFHardMaxAccel",&PyRobotBase::GetActiveDOFHardMaxAccel, DOXY_FN(RobotBase,GetActiveDOFHardMaxAccel))
                      .def("GetActiveDOFHardMaxJerk",&PyRobotBase::GetActiveDOFHardMaxJerk, DOXY_FN(RobotBase,GetActiveDOFHardMaxJerk))
                      .def("GetActiveDOFResolutions",&PyRobotBase::GetActiveDOFResolutions, DOXY_FN(RobotBase,GetActiveDOFResolutions))
                      .def("GetActiveConfigurationSpecification",&PyRobotBase::GetActiveConfigurationSpecification, py::arg("interpolation"),DOXY_FN(RobotBase,GetActiveConfigurationSpecification))
                      .def("GetActiveJointIndices",&PyRobotBase::GetActiveJointIndices)
                      .def("GetActiveDOFIndices",&PyRobotBase::GetActiveDOFIndices, DOXY_FN(RobotBase,GetActiveDOFIndices))
                      .def("SubtractActiveDOFValues",&PyRobotBase::SubtractActiveDOFValues, py::arg("values0"), py::arg("values1"), DOXY_FN(RobotBase,SubtractActiveDOFValues))
                      .def("CalculateActiveJacobian",&PyRobotBase::CalculateActiveJacobian,py::arg("linkindex"), py::arg("offset"), DOXY_FN(RobotBase,CalculateActiveJacobian "int; const Vector; std::vector"))
                      .def("CalculateActiveRotationJacobian",&PyRobotBase::CalculateActiveRotationJacobian,py::arg("linkindex"), py::arg("quat"), DOXY_FN(RobotBase,CalculateActiveRotationJacobian "int; const Vector; std::vector"))
                      .def("CalculateActiveAngularVelocityJacobian",&PyRobotBase::CalculateActiveAngularVelocityJacobian,py::arg("linkindex"), DOXY_FN(RobotBase,CalculateActiveAngularVelocityJacobian "int; std::vector"))
                      .def("Grab",pgrab1,py::arg("body"), DOXY_FN(RobotBase,Grab "KinBodyPtr"))
                      .def("Grab",pgrab2,py::arg("body"), py::arg("grablink"), DOXY_FN(RobotBase,Grab "KinBodyPtr; LinkPtr"))
                      .def("CheckLinkSelfCollision", &PyRobotBase::CheckLinkSelfCollision, py::arg("linkindex"), py::arg("linktrans"), py::arg("report"), DOXY_FN(RobotBase,CheckLinkSelfCollision))
                      .def("WaitForController",&PyRobotBase::WaitForController,py::arg("timeout"), "Wait until the robot controller is done")
                      .def("GetRobotStructureHash",&PyRobotBase::GetRobotStructureHash, DOXY_FN(RobotBase,GetRobotStructureHash))
                      .def("CreateRobotStateSaver",&PyRobotBase::CreateRobotStateSaver, py::arg("options"), "Creates an object that can be entered using 'with' and returns a RobotStateSaver")
                      .def("__repr__", &PyRobotBase::__repr__)
                      .def("__str__", &PyRobotBase::__str__)
                      .def("__unicode__", &PyRobotBase::__unicode__)
        ;
        robot.attr("DOFAffine") = dofaffine; // deprecated (11/10/04)
        robot.attr("ManipulatorInfo") = manipulatorinfo;
        robot.attr("AttachedSensorInfo") = attachedsensorinfo;

        object (PyRobotBase::PyManipulator::*pmanipik)(object, int, bool, bool) const = &PyRobotBase::PyManipulator::FindIKSolution;
        object (PyRobotBase::PyManipulator::*pmanipikf)(object, object, int, bool, bool) const = &PyRobotBase::PyManipulator::FindIKSolution;
        object (PyRobotBase::PyManipulator::*pmanipiks)(object, int, bool, bool) const = &PyRobotBase::PyManipulator::FindIKSolutions;
        object (PyRobotBase::PyManipulator::*pmanipiksf)(object, object, int, bool, bool) const = &PyRobotBase::PyManipulator::FindIKSolutions;

        bool (PyRobotBase::PyManipulator::*pCheckEndEffectorCollision0)(PyCollisionReportPtr) const = &PyRobotBase::PyManipulator::CheckEndEffectorCollision;
        bool (PyRobotBase::PyManipulator::*pCheckEndEffectorCollision1)(object,PyCollisionReportPtr,int) const = &PyRobotBase::PyManipulator::CheckEndEffectorCollision;
        bool (PyRobotBase::PyManipulator::*pCheckEndEffectorSelfCollision0)(PyCollisionReportPtr) const = &PyRobotBase::PyManipulator::CheckEndEffectorSelfCollision;
        bool (PyRobotBase::PyManipulator::*pCheckEndEffectorSelfCollision1)(object,PyCollisionReportPtr,int,bool) const = &PyRobotBase::PyManipulator::CheckEndEffectorSelfCollision;
        bool (PyRobotBase::PyManipulator::*pCheckIndependentCollision1)() const = &PyRobotBase::PyManipulator::CheckIndependentCollision;
        bool (PyRobotBase::PyManipulator::*pCheckIndependentCollision2)(PyCollisionReportPtr) const = &PyRobotBase::PyManipulator::CheckIndependentCollision;

        std::string GetIkParameterization_doc = std::string(DOXY_FN(RobotBase::Manipulator,GetIkParameterization "const IkParameterization; bool")) + std::string(DOXY_FN(RobotBase::Manipulator,GetIkParameterization "IkParameterizationType; bool"));
        py::class_<PyRobotBase::PyManipulator, OPENRAVE_SHARED_PTR<PyRobotBase::PyManipulator> >(m, "Manipulator", DOXY_CLASS(RobotBase::Manipulator))
        .def("GetEndEffectorTransform", &PyRobotBase::PyManipulator::GetTransform, DOXY_FN(RobotBase::Manipulator,GetTransform))
        .def("GetTransform", &PyRobotBase::PyManipulator::GetTransform, DOXY_FN(RobotBase::Manipulator,GetTransform))
        .def("GetTransformPose", &PyRobotBase::PyManipulator::GetTransformPose, DOXY_FN(RobotBase::Manipulator,GetTransform))
        .def("GetVelocity", &PyRobotBase::PyManipulator::GetVelocity, DOXY_FN(RobotBase::Manipulator,GetVelocity))
        .def("GetName",&PyRobotBase::PyManipulator::GetName, DOXY_FN(RobotBase::Manipulator,GetName))
        .def("SetName",&PyRobotBase::PyManipulator::SetName, py::arg("name"), DOXY_FN(RobotBase::Manipulator,SetName))
        .def("GetRobot",&PyRobotBase::PyManipulator::GetRobot, DOXY_FN(RobotBase::Manipulator,GetRobot))
        .def("SetIkSolver",&PyRobotBase::PyManipulator::SetIkSolver, DOXY_FN(RobotBase::Manipulator,SetIkSolver))
        .def("GetIkSolver",&PyRobotBase::PyManipulator::GetIkSolver, DOXY_FN(RobotBase::Manipulator,GetIkSolver))
        .def("SetIKSolver",&PyRobotBase::PyManipulator::SetIkSolver, DOXY_FN(RobotBase::Manipulator,SetIkSolver))
        .def("GetNumFreeParameters",&PyRobotBase::PyManipulator::GetNumFreeParameters, DOXY_FN(RobotBase::Manipulator,GetNumFreeParameters))
        .def("GetFreeParameters",&PyRobotBase::PyManipulator::GetFreeParameters, DOXY_FN(RobotBase::Manipulator,GetFreeParameters))
        .def("FindIKSolution",pmanipik, py::arg("param"), py::arg("filteroptions"), py::arg("ikreturn"), py::arg("releasegil"), DOXY_FN(RobotBase::Manipulator,FindIKSolution "const IkParameterization; std::vector; int"))
        .def("FindIKSolution",pmanipikf, py::arg("param"), py::arg("freevalues"), py::arg("filteroptions"), py::arg("ikreturn"), py::arg("releasegil"), DOXY_FN(RobotBase::Manipulator,FindIKSolution "const IkParameterization; const std::vector; std::vector; int"))
        .def("FindIKSolutions",pmanipiks, py::arg("param"), py::arg("filteroptions"), py::arg("ikreturn"), py::arg("releasegil"), DOXY_FN(RobotBase::Manipulator,FindIKSolutions "const IkParameterization; std::vector; int"))
        .def("FindIKSolutions",pmanipiksf, py::arg("param"), py::arg("freevalues"), py::arg("filteroptions"), py::arg("ikreturn"), py::arg("releasegil"), DOXY_FN(RobotBase::Manipulator,FindIKSolutions "const IkParameterization; const std::vector; std::vector; int"))
        .def("GetIkParameterization",&PyRobotBase::PyManipulator::GetIkParameterization, py::arg("iktype"), py::arg("inworld"), GetIkParameterization_doc.c_str())
        .def("GetBase",&PyRobotBase::PyManipulator::GetBase, DOXY_FN(RobotBase::Manipulator,GetBase))
        .def("GetEndEffector",&PyRobotBase::PyManipulator::GetEndEffector, DOXY_FN(RobotBase::Manipulator,GetEndEffector))
        .def("ReleaseAllGrabbed",&PyRobotBase::PyManipulator::ReleaseAllGrabbed, DOXY_FN(RobotBase::Manipulator,ReleaseAllGrabbed))
        .def("GetGraspTransform",&PyRobotBase::PyManipulator::GetGraspTransform, DOXY_FN(RobotBase::Manipulator,GetLocalToolTransform))
        .def("GetLocalToolTransform",&PyRobotBase::PyManipulator::GetLocalToolTransform, DOXY_FN(RobotBase::Manipulator,GetLocalToolTransform))
        .def("GetLocalToolTransformPose",&PyRobotBase::PyManipulator::GetLocalToolTransformPose, DOXY_FN(RobotBase::Manipulator,GetLocalToolTransformPose))
        .def("SetLocalToolTransform",&PyRobotBase::PyManipulator::SetLocalToolTransform, py::arg("transform"), DOXY_FN(RobotBase::Manipulator,SetLocalToolTransform))
        .def("SetLocalToolDirection",&PyRobotBase::PyManipulator::SetLocalToolDirection, py::arg("direction"), DOXY_FN(RobotBase::Manipulator,SetLocalToolDirection))
        .def("SetClosingDirection",&PyRobotBase::PyManipulator::SetClosingDirection, py::arg("closingdirection"), DOXY_FN(RobotBase::Manipulator,SetClosingDirection))
        .def("SetChuckingDirection",&PyRobotBase::PyManipulator::SetChuckingDirection, py::arg("chuckingdirection"), DOXY_FN(RobotBase::Manipulator,SetChuckingDirection))
        .def("GetGripperJoints",&PyRobotBase::PyManipulator::GetGripperJoints, DOXY_FN(RobotBase::Manipulator,GetGripperIndices))
        .def("GetGripperIndices",&PyRobotBase::PyManipulator::GetGripperIndices, DOXY_FN(RobotBase::Manipulator,GetGripperIndices))
        .def("GetArmJoints",&PyRobotBase::PyManipulator::GetArmJoints, DOXY_FN(RobotBase::Manipulator,GetArmIndices))
        .def("GetArmIndices",&PyRobotBase::PyManipulator::GetArmIndices, DOXY_FN(RobotBase::Manipulator,GetArmIndices))
        .def("GetArmDOFValues",&PyRobotBase::PyManipulator::GetArmDOFValues, DOXY_FN(RobotBase::Manipulator,GetArmDOFValues))
        .def("GetGripperDOFValues",&PyRobotBase::PyManipulator::GetGripperDOFValues, DOXY_FN(RobotBase::Manipulator,GetGripperDOFValues))
        .def("GetArmDOF",&PyRobotBase::PyManipulator::GetArmDOF, DOXY_FN(RobotBase::Manipulator,GetArmDOF))
        .def("GetGripperDOF",&PyRobotBase::PyManipulator::GetGripperDOF, DOXY_FN(RobotBase::Manipulator,GetGripperDOF))
        .def("GetClosingDirection",&PyRobotBase::PyManipulator::GetClosingDirection, DOXY_FN(RobotBase::Manipulator,GetClosingDirection))
        .def("GetChuckingDirection",&PyRobotBase::PyManipulator::GetChuckingDirection, DOXY_FN(RobotBase::Manipulator,GetChuckingDirection))
        .def("GetDirection",&PyRobotBase::PyManipulator::GetDirection, DOXY_FN(RobotBase::Manipulator,GetLocalToolDirection))
        .def("GetLocalToolDirection",&PyRobotBase::PyManipulator::GetLocalToolDirection, DOXY_FN(RobotBase::Manipulator,GetLocalToolDirection))
        .def("IsGrabbing",&PyRobotBase::PyManipulator::IsGrabbing,py::arg("body"), DOXY_FN(RobotBase::Manipulator,IsGrabbing))
        .def("GetChildJoints",&PyRobotBase::PyManipulator::GetChildJoints, DOXY_FN(RobotBase::Manipulator,GetChildJoints))
        .def("GetChildDOFIndices",&PyRobotBase::PyManipulator::GetChildDOFIndices, DOXY_FN(RobotBase::Manipulator,GetChildDOFIndices))
        .def("GetChildLinks",&PyRobotBase::PyManipulator::GetChildLinks, DOXY_FN(RobotBase::Manipulator,GetChildLinks))
        .def("IsChildLink",&PyRobotBase::PyManipulator::IsChildLink, DOXY_FN(RobotBase::Manipulator,IsChildLink))
        .def("GetIndependentLinks",&PyRobotBase::PyManipulator::GetIndependentLinks, DOXY_FN(RobotBase::Manipulator,GetIndependentLinks))
        .def("GetArmConfigurationSpecification",&PyRobotBase::PyManipulator::GetArmConfigurationSpecification, py::arg("interpolation"),DOXY_FN(RobotBase::Manipulator,GetArmConfigurationSpecification))
        .def("GetIkConfigurationSpecification",&PyRobotBase::PyManipulator::GetIkConfigurationSpecification, py::arg("iktype"), py::arg("interpolation"),DOXY_FN(RobotBase::Manipulator,GetIkConfigurationSpecification))
        .def("CheckEndEffectorCollision",pCheckEndEffectorCollision1, py::arg("transform"), py::arg("report"), py::arg("numredundantsamples"), DOXY_FN(RobotBase::Manipulator,CheckEndEffectorCollision))
        .def("CheckEndEffectorCollision",pCheckEndEffectorCollision0,py::arg("report"), DOXY_FN(RobotBase::Manipulator,CheckEndEffectorCollision))
        .def("CheckEndEffectorSelfCollision",pCheckEndEffectorSelfCollision0,py::arg("report"), DOXY_FN(RobotBase::Manipulator,CheckEndEffectorSelfCollision))
        .def("CheckEndEffectorSelfCollision",pCheckEndEffectorSelfCollision1, py::arg("transform"), py::arg("report"), py::arg("numredundantsamples"), py::arg("ignoreManipulatorLinks"), DOXY_FN(RobotBase::Manipulator,CheckEndEffectorSelfCollision))
        .def("CheckIndependentCollision",pCheckIndependentCollision1, DOXY_FN(RobotBase::Manipulator,CheckIndependentCollision))
        .def("CheckIndependentCollision",pCheckIndependentCollision2,py::arg("report"), DOXY_FN(RobotBase::Manipulator,CheckIndependentCollision))
        .def("CalculateJacobian",&PyRobotBase::PyManipulator::CalculateJacobian,DOXY_FN(RobotBase::Manipulator,CalculateJacobian))
        .def("CalculateRotationJacobian",&PyRobotBase::PyManipulator::CalculateRotationJacobian,DOXY_FN(RobotBase::Manipulator,CalculateRotationJacobian))
        .def("CalculateAngularVelocityJacobian",&PyRobotBase::PyManipulator::CalculateAngularVelocityJacobian,DOXY_FN(RobotBase::Manipulator,CalculateAngularVelocityJacobian))
        .def("GetStructureHash",&PyRobotBase::PyManipulator::GetStructureHash, DOXY_FN(RobotBase::Manipulator,GetStructureHash))
        .def("GetKinematicsStructureHash",&PyRobotBase::PyManipulator::GetKinematicsStructureHash, DOXY_FN(RobotBase::Manipulator,GetKinematicsStructureHash))
        .def("GetInverseKinematicsStructureHash",&PyRobotBase::PyManipulator::GetInverseKinematicsStructureHash, py::arg("iktype"), DOXY_FN(RobotBase::Manipulator,GetInverseKinematicsStructureHash))
        .def("GetInfo",&PyRobotBase::PyManipulator::GetInfo, DOXY_FN(RobotBase::Manipulator,GetInfo))
        .def("__repr__",&PyRobotBase::PyManipulator::__repr__)
        .def("__str__",&PyRobotBase::PyManipulator::__str__)
        .def("__unicode__",&PyRobotBase::PyManipulator::__unicode__)
        .def("__eq__",&PyRobotBase::PyManipulator::__eq__)
        .def("__ne__",&PyRobotBase::PyManipulator::__ne__)
        .def("__hash__",&PyRobotBase::PyManipulator::__hash__)
        ;

        py::class_<PyRobotBase::PyAttachedSensor, OPENRAVE_SHARED_PTR<PyRobotBase::PyAttachedSensor> >(m, "AttachedSensor", DOXY_CLASS(RobotBase::AttachedSensor))
        .def("GetSensor",&PyRobotBase::PyAttachedSensor::GetSensor, DOXY_FN(RobotBase::AttachedSensor,GetSensor))
        .def("GetAttachingLink",&PyRobotBase::PyAttachedSensor::GetAttachingLink, DOXY_FN(RobotBase::AttachedSensor,GetAttachingLink))
        .def("GetRelativeTransform",&PyRobotBase::PyAttachedSensor::GetRelativeTransform, DOXY_FN(RobotBase::AttachedSensor,GetRelativeTransform))
        .def("GetTransform",&PyRobotBase::PyAttachedSensor::GetTransform, DOXY_FN(RobotBase::AttachedSensor,GetTransform))
        .def("GetTransformPose",&PyRobotBase::PyAttachedSensor::GetTransformPose, DOXY_FN(RobotBase::AttachedSensor,GetTransform))
        .def("GetRobot",&PyRobotBase::PyAttachedSensor::GetRobot, DOXY_FN(RobotBase::AttachedSensor,GetRobot))
        .def("GetName",&PyRobotBase::PyAttachedSensor::GetName, DOXY_FN(RobotBase::AttachedSensor,GetName))
        .def("GetData",&PyRobotBase::PyAttachedSensor::GetData, DOXY_FN(RobotBase::AttachedSensor,GetData))
        .def("SetRelativeTransform",&PyRobotBase::PyAttachedSensor::SetRelativeTransform,py::arg("transform"), DOXY_FN(RobotBase::AttachedSensor,SetRelativeTransform))
        .def("GetStructureHash",&PyRobotBase::PyAttachedSensor::GetStructureHash, DOXY_FN(RobotBase::AttachedSensor,GetStructureHash))
        .def("UpdateInfo",&PyRobotBase::PyAttachedSensor::UpdateInfo, py::arg("type"), DOXY_FN(RobotBase::AttachedSensor,UpdateInfo))
        .def("GetInfo",&PyRobotBase::PyAttachedSensor::GetInfo, DOXY_FN(RobotBase::AttachedSensor,GetInfo))
        .def("UpdateAndGetInfo",&PyRobotBase::PyAttachedSensor::UpdateAndGetInfo, DOXY_FN(RobotBase::AttachedSensor,UpdateAndGetInfo))
        .def("__str__",&PyRobotBase::PyAttachedSensor::__str__)
        .def("__repr__",&PyRobotBase::PyAttachedSensor::__repr__)
        .def("__unicode__",&PyRobotBase::PyAttachedSensor::__unicode__)
        .def("__eq__",&PyRobotBase::PyAttachedSensor::__eq__)
        .def("__ne__",&PyRobotBase::PyAttachedSensor::__ne__)
        .def("__hash__",&PyRobotBase::PyAttachedSensor::__hash__)
        ;

        py::class_<PyRobotBase::PyConnectedBody, OPENRAVE_SHARED_PTR<PyRobotBase::PyConnectedBody> >(m, "ConnectedBody", DOXY_CLASS(RobotBase::ConnectedBody))
        .def("GetName",&PyRobotBase::PyConnectedBody::GetName, DOXY_FN(RobotBase::ConnectedBody,GetName))
        .def("GetInfo",&PyRobotBase::PyConnectedBody::GetInfo, DOXY_FN(RobotBase::ConnectedBody,GetInfo))
        .def("SetActive", &PyRobotBase::PyConnectedBody::SetActive, DOXY_FN(RobotBase::ConnectedBody,SetActive))
        .def("IsActive", &PyRobotBase::PyConnectedBody::IsActive, DOXY_FN(RobotBase::ConnectedBody,IsActive))
        .def("SetLinkEnable", &PyRobotBase::PyConnectedBody::SetLinkEnable, DOXY_FN(RobotBase::ConnectedBody,SetLinkEnable))
        .def("SetLinkVisible", &PyRobotBase::PyConnectedBody::SetLinkVisible, DOXY_FN(RobotBase::ConnectedBody,SetLinkVisible))
        .def("GetTransform",&PyRobotBase::PyConnectedBody::GetTransform, DOXY_FN(RobotBase::ConnectedBody,GetTransform))
        .def("GetTransformPose",&PyRobotBase::PyConnectedBody::GetTransformPose, DOXY_FN(RobotBase::ConnectedBody,GetTransformPose))
        .def("GetRelativeTransform",&PyRobotBase::PyConnectedBody::GetRelativeTransform, DOXY_FN(RobotBase::ConnectedBody,GetRelativeTransform))
        .def("GetRelativeTransformPose",&PyRobotBase::PyConnectedBody::GetRelativeTransformPose, DOXY_FN(RobotBase::ConnectedBody,GetRelativeTransformPose))
        .def("GetResolvedLinks",&PyRobotBase::PyConnectedBody::GetResolvedLinks, DOXY_FN(RobotBase::ConnectedBody,GetResolvedLinks))
        .def("GetResolvedJoints",&PyRobotBase::PyConnectedBody::GetResolvedJoints, DOXY_FN(RobotBase::ConnectedBody,GetResolvedJoints))
        .def("GetResolvedManipulators",&PyRobotBase::PyConnectedBody::GetResolvedManipulators, DOXY_FN(RobotBase::ConnectedBody,GetResolvedManipulators))
        .def("__str__",&PyRobotBase::PyConnectedBody::__str__)
        .def("__repr__",&PyRobotBase::PyConnectedBody::__repr__)
        .def("__unicode__",&PyRobotBase::PyConnectedBody::__unicode__)
        .def("__eq__",&PyRobotBase::PyConnectedBody::__eq__)
        .def("__ne__",&PyRobotBase::PyConnectedBody::__ne__)
        .def("__hash__",&PyRobotBase::PyConnectedBody::__hash__);

        py::class_<PyRobotBase::PyRobotStateSaver, OPENRAVE_SHARED_PTR<PyRobotBase::PyRobotStateSaver> >(m, "RobotStateSaver", DOXY_CLASS(Robot::RobotStateSaver))
        .def(py::init<PyRobotBasePtr>())
        .def(py::init<PyRobotBasePtr, object>())
        .def("GetBody",&PyRobotBase::PyRobotStateSaver::GetBody,DOXY_FN(Robot::RobotStateSaver, GetBody))
        .def("Restore",&PyRobotBase::PyRobotStateSaver::Restore, py::arg("body"), DOXY_FN(Robot::RobotStateSaver, Restore))
        .def("Release",&PyRobotBase::PyRobotStateSaver::Release,DOXY_FN(Robot::RobotStateSaver, Release))
        .def("__str__",&PyRobotBase::PyRobotStateSaver::__str__)
        .def("__unicode__",&PyRobotBase::PyRobotStateSaver::__unicode__)
        ;
    }

    m.def("RaveCreateRobot",openravepy::RaveCreateRobot, py::arg("env"), py::arg("name"), DOXY_FN1(RaveCreateRobot));
}

}
