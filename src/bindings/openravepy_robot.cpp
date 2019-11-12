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

namespace openravepy {

class PyManipulatorInfo
{
public:
    PyManipulatorInfo() {
        _tLocalTool = ReturnTransform(Transform());
        _vChuckingDirection = np::array(boost::python::list());
        _vdirection = toPyVector3(Vector(0,0,1));
        _vGripperJointNames = boost::python::list();
    }
    PyManipulatorInfo(const RobotBase::ManipulatorInfo& info) {
        _name = ConvertStringToUnicode(info._name);
        _sBaseLinkName = ConvertStringToUnicode(info._sBaseLinkName);
        _sEffectorLinkName = ConvertStringToUnicode(info._sEffectorLinkName);
        _tLocalTool = ReturnTransform(info._tLocalTool);
        _vChuckingDirection = toPyArray(info._vChuckingDirection);
        _vdirection = toPyVector3(info._vdirection);
        _sIkSolverXMLId = info._sIkSolverXMLId;
        boost::python::list vGripperJointNames;
        FOREACHC(itname, info._vGripperJointNames) {
            vGripperJointNames.append(ConvertStringToUnicode(*itname));
        }
        _vGripperJointNames = vGripperJointNames;
    }

    RobotBase::ManipulatorInfoPtr GetManipulatorInfo() const
    {
        RobotBase::ManipulatorInfoPtr pinfo(new RobotBase::ManipulatorInfo());
        pinfo->_name = boost::python::extract<std::string>(_name);
        pinfo->_sBaseLinkName = boost::python::extract<std::string>(_sBaseLinkName);
        pinfo->_sEffectorLinkName = boost::python::extract<std::string>(_sEffectorLinkName);
        pinfo->_tLocalTool = ExtractTransform(_tLocalTool);
        pinfo->_vChuckingDirection = ExtractArray<dReal>(_vChuckingDirection);
        pinfo->_vdirection = ExtractVector3(_vdirection);
        pinfo->_sIkSolverXMLId = _sIkSolverXMLId;
        pinfo->_vGripperJointNames = ExtractArray<std::string>(_vGripperJointNames);
        return pinfo;
    }

    object _name, _sBaseLinkName, _sEffectorLinkName;
    object _tLocalTool;
    object _vChuckingDirection;
    object _vdirection;
    std::string _sIkSolverXMLId;
    object _vGripperJointNames;
};

PyManipulatorInfoPtr toPyManipulatorInfo(const RobotBase::ManipulatorInfo& manipulatorinfo)
{
    return PyManipulatorInfoPtr(new PyManipulatorInfo(manipulatorinfo));
}

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
        pinfo->_name = boost::python::extract<std::string>(_name);
        pinfo->_linkname = boost::python::extract<std::string>(_linkname);
        pinfo->_trelative = ExtractTransform(_trelative);
        pinfo->_sensorname = boost::python::extract<std::string>(_sensorname);
        pinfo->_sensorgeometry = _sensorgeometry->GetGeometry();
        return pinfo;
    }

    object _name, _linkname;
    object _trelative;
    object _sensorname;
    PySensorGeometryPtr _sensorgeometry;
};

PyAttachedSensorInfoPtr toPyAttachedSensorInfo(const RobotBase::AttachedSensorInfo& attachedSensorinfo)
{
    return PyAttachedSensorInfoPtr(new PyAttachedSensorInfo(attachedSensorinfo));
}

class PyConnectedBodyInfo
{
public:
    PyConnectedBodyInfo() {
    }
    PyConnectedBodyInfo(const RobotBase::ConnectedBodyInfo& info, PyEnvironmentBasePtr pyenv)
    {
        _name = ConvertStringToUnicode(info._name);
        _linkname = ConvertStringToUnicode(info._linkname);
        _trelative = ReturnTransform(info._trelative);
        _url = ConvertStringToUnicode(info._url);

        boost::python::list linkInfos;
        FOREACH(itlinkinfo, info._vLinkInfos) {
            linkInfos.append(toPyLinkInfo(**itlinkinfo));
        }
        _linkInfos = linkInfos;

        boost::python::list jointInfos;
        FOREACH(itjointinfo, info._vJointInfos) {
            jointInfos.append(toPyJointInfo(**itjointinfo, pyenv));
        }
        _jointInfos = jointInfos;

        boost::python::list manipulatorInfos;
        FOREACH(itmanipulatorinfo, info._vManipulatorInfos) {
            manipulatorInfos.append(toPyManipulatorInfo(**itmanipulatorinfo));
        }
        _manipulatorInfos = manipulatorInfos;

        boost::python::list attachedSensorInfos;
        FOREACH(itattachedSensorinfo, info._vAttachedSensorInfos) {
            attachedSensorInfos.append(toPyAttachedSensorInfo(**itattachedSensorinfo));
        }
        _attachedSensorInfos = attachedSensorInfos;
    }

    RobotBase::ConnectedBodyInfoPtr GetConnectedBodyInfo() const
    {
        RobotBase::ConnectedBodyInfoPtr pinfo(new RobotBase::ConnectedBodyInfo());
        pinfo->_name = boost::python::extract<std::string>(_name);
        pinfo->_linkname = boost::python::extract<std::string>(_linkname);
        pinfo->_trelative = ExtractTransform(_trelative);
        pinfo->_url = boost::python::extract<std::string>(_url);
        // extract all the infos
        return pinfo;
    }

    object _name;
    object _linkname;
    object _trelative;
    object _url;
    object _linkInfos;
    object _jointInfos;
    object _manipulatorInfos;
    object _attachedSensorInfos;

};

PyConnectedBodyInfoPtr toPyConnectedBodyInfo(const RobotBase::ConnectedBodyInfo& connectedBodyInfo, PyEnvironmentBasePtr pyenv)
{
    return PyConnectedBodyInfoPtr(new PyConnectedBodyInfo(connectedBodyInfo, pyenv));
}

class ManipulatorInfo_pickle_suite : public pickle_suite
{
public:
    static boost::python::tuple getstate(const PyManipulatorInfo& r)
    {
        return boost::python::make_tuple(r._name, r._sBaseLinkName, r._sEffectorLinkName, r._tLocalTool, r._vChuckingDirection, r._vdirection, r._sIkSolverXMLId, r._vGripperJointNames);
    }
    static void setstate(PyManipulatorInfo& r, boost::python::tuple state) {
        r._name = state[0];
        r._sBaseLinkName = state[1];
        r._sEffectorLinkName = state[2];
        r._tLocalTool = state[3];
        r._vChuckingDirection = state[4];
        r._vdirection = state[5];
        r._sIkSolverXMLId = boost::python::extract<std::string>(state[6]);
        r._vGripperJointNames = state[7];
    }
};

RobotBasePtr GetRobot(object o)
{
    extract<PyRobotBasePtr> pyrobot(o);
    if( pyrobot.check() ) {
        return GetRobot((PyRobotBasePtr)pyrobot);
    }
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
    extract<PyRobotBase::PyManipulatorPtr> pymanipulator(o);
    if( pymanipulator.check() ) {
        return ((PyRobotBase::PyManipulatorPtr)pymanipulator)->GetManipulator();
    }
    return RobotBase::ManipulatorPtr();
}

object toPyRobotManipulator(RobotBase::ManipulatorPtr pmanip, PyEnvironmentBasePtr pyenv)
{
    return !pmanip ? object() : object(PyRobotBase::PyManipulatorPtr(new PyRobotBase::PyManipulator(pmanip,pyenv)));
}

PyRobotBasePtr RaveCreateRobot(PyEnvironmentBasePtr pyenv, const std::string& name)
{
    RobotBasePtr p = OpenRAVE::RaveCreateRobot(openravepy::GetEnvironment(pyenv), name);
    if( !p ) {
        return PyRobotBasePtr();
    }
    return PyRobotBasePtr(new PyRobotBase(p,pyenv));
}

BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(GetIkParameterization_overloads, GetIkParameterization, 1, 2)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(CheckEndEffectorCollision_overloads, CheckEndEffectorCollision, 1, 3)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(CheckEndEffectorSelfCollision_overloads, CheckEndEffectorSelfCollision, 1, 4)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(FindIKSolution_overloads, FindIKSolution, 2, 4)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(FindIKSolutionFree_overloads, FindIKSolution, 3, 5)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(FindIKSolutions_overloads, FindIKSolutions, 2, 4)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(FindIKSolutionsFree_overloads, FindIKSolutions, 3, 5)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(GetArmConfigurationSpecification_overloads, GetArmConfigurationSpecification, 0, 1)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(GetIkConfigurationSpecification_overloads, GetIkConfigurationSpecification, 1, 2)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(CreateRobotStateSaver_overloads, CreateRobotStateSaver, 0,1)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(SetActiveDOFValues_overloads, SetActiveDOFValues, 1,2)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(SetActiveDOFVelocities_overloads, SetActiveDOFVelocities, 1,2)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(AddManipulator_overloads, AddManipulator, 1,2)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(AddAttachedSensor_overloads, AddAttachedSensor, 1,2)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(AddConnectedBody_overloads, AddConnectedBody, 1,2)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(GetActiveConfigurationSpecification_overloads, GetActiveConfigurationSpecification, 0, 1)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(Restore_overloads, Restore, 0,1)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(Init_overloads, Init, 4,5)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(UpdateInfo_overloads, UpdateInfo, 0,1)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(UpdateAndGetInfo_overloads, UpdateAndGetInfo, 0,1)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(CheckLinkSelfCollision_overloads, CheckLinkSelfCollision, 2, 3)

void init_openravepy_robot()
{
    object dofaffine = enum_<DOFAffine>("DOFAffine" DOXY_ENUM(DOFAffine))
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


    object manipulatorinfo = class_<PyManipulatorInfo, OPENRAVE_SHARED_PTR<PyManipulatorInfo> >("ManipulatorInfo", DOXY_CLASS(RobotBase::ManipulatorInfo))
                             .def_readwrite("_name",&PyManipulatorInfo::_name)
                             .def_readwrite("_sBaseLinkName",&PyManipulatorInfo::_sBaseLinkName)
                             .def_readwrite("_sEffectorLinkName",&PyManipulatorInfo::_sEffectorLinkName)
                             .def_readwrite("_tLocalTool",&PyManipulatorInfo::_tLocalTool)
                             .def_readwrite("_vChuckingDirection",&PyManipulatorInfo::_vChuckingDirection)
                             .def_readwrite("_vClosingDirection",&PyManipulatorInfo::_vChuckingDirection) // back compat
                             .def_readwrite("_vdirection",&PyManipulatorInfo::_vdirection)
                             .def_readwrite("_sIkSolverXMLId",&PyManipulatorInfo::_sIkSolverXMLId)
                             .def_readwrite("_vGripperJointNames",&PyManipulatorInfo::_vGripperJointNames)
                             .def_pickle(ManipulatorInfo_pickle_suite())
    ;

    object attachedsensorinfo = class_<PyAttachedSensorInfo, OPENRAVE_SHARED_PTR<PyAttachedSensorInfo> >("AttachedSensorInfo", DOXY_CLASS(RobotBase::AttachedSensorInfo))
                                .def_readwrite("_name", &PyAttachedSensorInfo::_name)
                                .def_readwrite("_linkname", &PyAttachedSensorInfo::_linkname)
                                .def_readwrite("_trelative", &PyAttachedSensorInfo::_trelative)
                                .def_readwrite("_sensorname", &PyAttachedSensorInfo::_sensorname)
                                .def_readwrite("_sensorgeometry", &PyAttachedSensorInfo::_sensorgeometry)
    ;

    object connectedbodyinfo = class_<PyConnectedBodyInfo, OPENRAVE_SHARED_PTR<PyConnectedBodyInfo> >("ConnectedBodyInfo", DOXY_CLASS(RobotBase::ConnectedBodyInfo))
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
        scope robot = class_<PyRobotBase, OPENRAVE_SHARED_PTR<PyRobotBase>, bases<PyKinBody, PyInterfaceBase> >("Robot", DOXY_CLASS(RobotBase), no_init)
                      .def("Init", initrobot, Init_overloads(args("linkinfos", "jointinfos", "manipinfos", "attachedsensorinfos", "uri"), DOXY_FN(RobotBase, Init)))
                      .def("GetManipulators",GetManipulators1, DOXY_FN(RobotBase,GetManipulators))
                      .def("GetManipulators",GetManipulators2,args("manipname"), DOXY_FN(RobotBase,GetManipulators))
                      .def("GetManipulator",&PyRobotBase::GetManipulator,args("manipname"), "Return the manipulator whose name matches")
                      .def("SetActiveManipulator",setactivemanipulator2,args("manipname"), DOXY_FN(RobotBase,SetActiveManipulator "const std::string"))
                      .def("SetActiveManipulator",setactivemanipulator3,args("manip"), "Set the active manipulator given a pointer")
                      .def("GetActiveManipulator",&PyRobotBase::GetActiveManipulator, DOXY_FN(RobotBase,GetActiveManipulator))
                      .def("AddManipulator",&PyRobotBase::AddManipulator, AddManipulator_overloads(args("manipinfo", "removeduplicate"), DOXY_FN(RobotBase,AddManipulator)))
                      .def("AddAttachedSensor",&PyRobotBase::AddAttachedSensor, AddAttachedSensor_overloads(args("attachedsensorinfo", "removeduplicate"), DOXY_FN(RobotBase,AddAttachedSensor)))
                      .def("RemoveAttachedSensor",&PyRobotBase::RemoveAttachedSensor, args("attsensor"), DOXY_FN(RobotBase,RemoveAttachedSensor))
                      .def("RemoveManipulator",&PyRobotBase::RemoveManipulator, args("manip"), DOXY_FN(RobotBase,RemoveManipulator))
                      .def("GetAttachedSensors",&PyRobotBase::GetAttachedSensors, DOXY_FN(RobotBase,GetAttachedSensors))
                      .def("GetAttachedSensor",&PyRobotBase::GetAttachedSensor,args("sensorname"), "Return the attached sensor whose name matches")
                      .def("GetSensors",&PyRobotBase::GetSensors)
                      .def("GetSensor",&PyRobotBase::GetSensor,args("sensorname"))
                      .def("AddConnectedBody",&PyRobotBase::AddConnectedBody, AddConnectedBody_overloads(args("connectedbodyinfo", "removeduplicate"), DOXY_FN(RobotBase,AddConnectedBody)))
                      .def("RemoveConnectedBody",&PyRobotBase::RemoveConnectedBody, args("connectedbody"), DOXY_FN(RobotBase,RemoveConnectedBody))
                      .def("GetConnectedBodies",&PyRobotBase::GetConnectedBodies, DOXY_FN(RobotBase,GetConnectedBodies))
                      .def("GetConnectedBody",&PyRobotBase::GetConnectedBody, args("bodyname"), DOXY_FN(RobotBase,GetConnectedBody))
                      .def("GetConnectedBodyActiveStates",&PyRobotBase::GetConnectedBodyActiveStates, DOXY_FN(RobotBase,GetConnectedBodyActiveStates))
                      .def("SetConnectedBodyActiveStates",&PyRobotBase::SetConnectedBodyActiveStates, DOXY_FN(RobotBase,SetConnectedBodyActiveStates))
                      .def("GetController",&PyRobotBase::GetController, DOXY_FN(RobotBase,GetController))
                      .def("SetController",setcontroller1,DOXY_FN(RobotBase,SetController))
                      .def("SetController",setcontroller2,args("robot","dofindices","controltransform"), DOXY_FN(RobotBase,SetController))
                      .def("SetController",setcontroller3,DOXY_FN(RobotBase,SetController))
                      .def("SetActiveDOFs",psetactivedofs1,args("dofindices"), DOXY_FN(RobotBase,SetActiveDOFs "const std::vector; int"))
                      .def("SetActiveDOFs",psetactivedofs2,args("dofindices","affine"), DOXY_FN(RobotBase,SetActiveDOFs "const std::vector; int"))
                      .def("SetActiveDOFs",psetactivedofs3,args("dofindices","affine","rotationaxis"), DOXY_FN(RobotBase,SetActiveDOFs "const std::vector; int; const Vector"))
                      .def("GetActiveDOF",&PyRobotBase::GetActiveDOF, DOXY_FN(RobotBase,GetActiveDOF))
                      .def("GetAffineDOF",&PyRobotBase::GetAffineDOF, DOXY_FN(RobotBase,GetAffineDOF))
                      .def("GetAffineDOFIndex",&PyRobotBase::GetAffineDOFIndex,args("index"), DOXY_FN(RobotBase,GetAffineDOFIndex))
                      .def("GetAffineRotationAxis",&PyRobotBase::GetAffineRotationAxis, DOXY_FN(RobotBase,GetAffineRotationAxis))
                      .def("SetAffineTranslationLimits",&PyRobotBase::SetAffineTranslationLimits,args("lower","upper"), DOXY_FN(RobotBase,SetAffineTranslationLimits))
                      .def("SetAffineRotationAxisLimits",&PyRobotBase::SetAffineRotationAxisLimits,args("lower","upper"), DOXY_FN(RobotBase,SetAffineRotationAxisLimits))
                      .def("SetAffineRotation3DLimits",&PyRobotBase::SetAffineRotation3DLimits,args("lower","upper"), DOXY_FN(RobotBase,SetAffineRotation3DLimits))
                      .def("SetAffineRotationQuatLimits",&PyRobotBase::SetAffineRotationQuatLimits,args("quatangle"), DOXY_FN(RobotBase,SetAffineRotationQuatLimits))
                      .def("SetAffineTranslationMaxVels",&PyRobotBase::SetAffineTranslationMaxVels,args("lower","upper"), DOXY_FN(RobotBase,SetAffineTranslationMaxVels))
                      .def("SetAffineRotationAxisMaxVels",&PyRobotBase::SetAffineRotationAxisMaxVels,args("velocity"), DOXY_FN(RobotBase,SetAffineRotationAxisMaxVels))
                      .def("SetAffineRotation3DMaxVels",&PyRobotBase::SetAffineRotation3DMaxVels,args("velocity"), DOXY_FN(RobotBase,SetAffineRotation3DMaxVels))
                      .def("SetAffineRotationQuatMaxVels",&PyRobotBase::SetAffineRotationQuatMaxVels,args("velocity"), DOXY_FN(RobotBase,SetAffineRotationQuatMaxVels))
                      .def("SetAffineTranslationResolution",&PyRobotBase::SetAffineTranslationResolution,args("resolution"), DOXY_FN(RobotBase,SetAffineTranslationResolution))
                      .def("SetAffineRotationAxisResolution",&PyRobotBase::SetAffineRotationAxisResolution,args("resolution"), DOXY_FN(RobotBase,SetAffineRotationAxisResolution))
                      .def("SetAffineRotation3DResolution",&PyRobotBase::SetAffineRotation3DResolution,args("resolution"), DOXY_FN(RobotBase,SetAffineRotation3DResolution))
                      .def("SetAffineRotationQuatResolution",&PyRobotBase::SetAffineRotationQuatResolution,args("resolution"), DOXY_FN(RobotBase,SetAffineRotationQuatResolution))
                      .def("SetAffineTranslationWeights",&PyRobotBase::SetAffineTranslationWeights,args("weights"), DOXY_FN(RobotBase,SetAffineTranslationWeights))
                      .def("SetAffineRotationAxisWeights",&PyRobotBase::SetAffineRotationAxisWeights,args("weights"), DOXY_FN(RobotBase,SetAffineRotationAxisWeights))
                      .def("SetAffineRotation3DWeights",&PyRobotBase::SetAffineRotation3DWeights,args("weights"), DOXY_FN(RobotBase,SetAffineRotation3DWeights))
                      .def("SetAffineRotationQuatWeights",&PyRobotBase::SetAffineRotationQuatWeights,args("weights"), DOXY_FN(RobotBase,SetAffineRotationQuatWeights))
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
                      .def("SetActiveDOFValues",&PyRobotBase::SetActiveDOFValues,SetActiveDOFValues_overloads(args("values","checklimits"), DOXY_FN(RobotBase,SetActiveDOFValues)))
                      .def("GetActiveDOFValues",&PyRobotBase::GetActiveDOFValues, DOXY_FN(RobotBase,GetActiveDOFValues))
                      .def("GetActiveDOFWeights",&PyRobotBase::GetActiveDOFWeights, DOXY_FN(RobotBase,GetActiveDOFWeights))
                      .def("SetActiveDOFVelocities",&PyRobotBase::SetActiveDOFVelocities, SetActiveDOFVelocities_overloads(args("velocities","checklimits"), DOXY_FN(RobotBase,SetActiveDOFVelocities)))
                      .def("GetActiveDOFVelocities",&PyRobotBase::GetActiveDOFVelocities, DOXY_FN(RobotBase,GetActiveDOFVelocities))
                      .def("GetActiveDOFLimits",&PyRobotBase::GetActiveDOFLimits, DOXY_FN(RobotBase,GetActiveDOFLimits))
                      .def("GetActiveDOFMaxVel",&PyRobotBase::GetActiveDOFMaxVel, DOXY_FN(RobotBase,GetActiveDOFMaxVel))
                      .def("GetActiveDOFMaxAccel",&PyRobotBase::GetActiveDOFMaxAccel, DOXY_FN(RobotBase,GetActiveDOFMaxAccel))
                      .def("GetActiveDOFMaxJerk",&PyRobotBase::GetActiveDOFMaxJerk, DOXY_FN(RobotBase,GetActiveDOFMaxJerk))
                      .def("GetActiveDOFHardMaxVel",&PyRobotBase::GetActiveDOFHardMaxVel, DOXY_FN(RobotBase,GetActiveDOFHardMaxVel))
                      .def("GetActiveDOFHardMaxAccel",&PyRobotBase::GetActiveDOFHardMaxAccel, DOXY_FN(RobotBase,GetActiveDOFHardMaxAccel))
                      .def("GetActiveDOFHardMaxJerk",&PyRobotBase::GetActiveDOFHardMaxJerk, DOXY_FN(RobotBase,GetActiveDOFHardMaxJerk))
                      .def("GetActiveDOFResolutions",&PyRobotBase::GetActiveDOFResolutions, DOXY_FN(RobotBase,GetActiveDOFResolutions))
                      .def("GetActiveConfigurationSpecification",&PyRobotBase::GetActiveConfigurationSpecification, GetActiveConfigurationSpecification_overloads(args("interpolation"),DOXY_FN(RobotBase,GetActiveConfigurationSpecification)))
                      .def("GetActiveJointIndices",&PyRobotBase::GetActiveJointIndices)
                      .def("GetActiveDOFIndices",&PyRobotBase::GetActiveDOFIndices, DOXY_FN(RobotBase,GetActiveDOFIndices))
                      .def("SubtractActiveDOFValues",&PyRobotBase::SubtractActiveDOFValues, args("values0","values1"), DOXY_FN(RobotBase,SubtractActiveDOFValues))
                      .def("CalculateActiveJacobian",&PyRobotBase::CalculateActiveJacobian,args("linkindex","offset"), DOXY_FN(RobotBase,CalculateActiveJacobian "int; const Vector; std::vector"))
                      .def("CalculateActiveRotationJacobian",&PyRobotBase::CalculateActiveRotationJacobian,args("linkindex","quat"), DOXY_FN(RobotBase,CalculateActiveRotationJacobian "int; const Vector; std::vector"))
                      .def("CalculateActiveAngularVelocityJacobian",&PyRobotBase::CalculateActiveAngularVelocityJacobian,args("linkindex"), DOXY_FN(RobotBase,CalculateActiveAngularVelocityJacobian "int; std::vector"))
                      .def("Grab",pgrab1,args("body"), DOXY_FN(RobotBase,Grab "KinBodyPtr"))
                      .def("Grab",pgrab2,args("body","grablink"), DOXY_FN(RobotBase,Grab "KinBodyPtr; LinkPtr"))
                      .def("CheckLinkSelfCollision", &PyRobotBase::CheckLinkSelfCollision, CheckLinkSelfCollision_overloads(args("linkindex", "linktrans", "report"), DOXY_FN(RobotBase,CheckLinkSelfCollision)))
                      .def("WaitForController",&PyRobotBase::WaitForController,args("timeout"), "Wait until the robot controller is done")
                      .def("GetRobotStructureHash",&PyRobotBase::GetRobotStructureHash, DOXY_FN(RobotBase,GetRobotStructureHash))
                      .def("CreateRobotStateSaver",&PyRobotBase::CreateRobotStateSaver, CreateRobotStateSaver_overloads(args("options"), "Creates an object that can be entered using 'with' and returns a RobotStateSaver")[return_value_policy<manage_new_object>()])
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
        class_<PyRobotBase::PyManipulator, OPENRAVE_SHARED_PTR<PyRobotBase::PyManipulator> >("Manipulator", DOXY_CLASS(RobotBase::Manipulator), no_init)
        .def("GetEndEffectorTransform", &PyRobotBase::PyManipulator::GetTransform, DOXY_FN(RobotBase::Manipulator,GetTransform))
        .def("GetTransform", &PyRobotBase::PyManipulator::GetTransform, DOXY_FN(RobotBase::Manipulator,GetTransform))
        .def("GetTransformPose", &PyRobotBase::PyManipulator::GetTransformPose, DOXY_FN(RobotBase::Manipulator,GetTransform))
        .def("GetVelocity", &PyRobotBase::PyManipulator::GetVelocity, DOXY_FN(RobotBase::Manipulator,GetVelocity))
        .def("GetName",&PyRobotBase::PyManipulator::GetName, DOXY_FN(RobotBase::Manipulator,GetName))
        .def("SetName",&PyRobotBase::PyManipulator::SetName, args("name"), DOXY_FN(RobotBase::Manipulator,SetName))
        .def("GetRobot",&PyRobotBase::PyManipulator::GetRobot, DOXY_FN(RobotBase::Manipulator,GetRobot))
        .def("SetIkSolver",&PyRobotBase::PyManipulator::SetIkSolver, DOXY_FN(RobotBase::Manipulator,SetIkSolver))
        .def("GetIkSolver",&PyRobotBase::PyManipulator::GetIkSolver, DOXY_FN(RobotBase::Manipulator,GetIkSolver))
        .def("SetIKSolver",&PyRobotBase::PyManipulator::SetIkSolver, DOXY_FN(RobotBase::Manipulator,SetIkSolver))
        .def("GetNumFreeParameters",&PyRobotBase::PyManipulator::GetNumFreeParameters, DOXY_FN(RobotBase::Manipulator,GetNumFreeParameters))
        .def("GetFreeParameters",&PyRobotBase::PyManipulator::GetFreeParameters, DOXY_FN(RobotBase::Manipulator,GetFreeParameters))
        .def("FindIKSolution",pmanipik,FindIKSolution_overloads(args("param","filteroptions","ikreturn","releasegil"), DOXY_FN(RobotBase::Manipulator,FindIKSolution "const IkParameterization; std::vector; int")))
        .def("FindIKSolution",pmanipikf,FindIKSolutionFree_overloads(args("param","freevalues","filteroptions","ikreturn","releasegil"), DOXY_FN(RobotBase::Manipulator,FindIKSolution "const IkParameterization; const std::vector; std::vector; int")))
        .def("FindIKSolutions",pmanipiks,FindIKSolutions_overloads(args("param","filteroptions","ikreturn","releasegil"), DOXY_FN(RobotBase::Manipulator,FindIKSolutions "const IkParameterization; std::vector; int")))
        .def("FindIKSolutions",pmanipiksf,FindIKSolutionsFree_overloads(args("param","freevalues","filteroptions","ikreturn","releasegil"), DOXY_FN(RobotBase::Manipulator,FindIKSolutions "const IkParameterization; const std::vector; std::vector; int")))
        .def("GetIkParameterization",&PyRobotBase::PyManipulator::GetIkParameterization, GetIkParameterization_overloads(args("iktype","inworld"), GetIkParameterization_doc.c_str()))
        .def("GetBase",&PyRobotBase::PyManipulator::GetBase, DOXY_FN(RobotBase::Manipulator,GetBase))
        .def("GetEndEffector",&PyRobotBase::PyManipulator::GetEndEffector, DOXY_FN(RobotBase::Manipulator,GetEndEffector))
        .def("ReleaseAllGrabbed",&PyRobotBase::PyManipulator::ReleaseAllGrabbed, DOXY_FN(RobotBase::Manipulator,ReleaseAllGrabbed))
        .def("GetGraspTransform",&PyRobotBase::PyManipulator::GetGraspTransform, DOXY_FN(RobotBase::Manipulator,GetLocalToolTransform))
        .def("GetLocalToolTransform",&PyRobotBase::PyManipulator::GetLocalToolTransform, DOXY_FN(RobotBase::Manipulator,GetLocalToolTransform))
        .def("GetLocalToolTransformPose",&PyRobotBase::PyManipulator::GetLocalToolTransformPose, DOXY_FN(RobotBase::Manipulator,GetLocalToolTransformPose))
        .def("SetLocalToolTransform",&PyRobotBase::PyManipulator::SetLocalToolTransform, args("transform"), DOXY_FN(RobotBase::Manipulator,SetLocalToolTransform))
        .def("SetLocalToolDirection",&PyRobotBase::PyManipulator::SetLocalToolDirection, args("direction"), DOXY_FN(RobotBase::Manipulator,SetLocalToolDirection))
        .def("SetClosingDirection",&PyRobotBase::PyManipulator::SetClosingDirection, args("closingdirection"), DOXY_FN(RobotBase::Manipulator,SetClosingDirection))
        .def("SetChuckingDirection",&PyRobotBase::PyManipulator::SetChuckingDirection, args("chuckingdirection"), DOXY_FN(RobotBase::Manipulator,SetChuckingDirection))
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
        .def("IsGrabbing",&PyRobotBase::PyManipulator::IsGrabbing,args("body"), DOXY_FN(RobotBase::Manipulator,IsGrabbing))
        .def("GetChildJoints",&PyRobotBase::PyManipulator::GetChildJoints, DOXY_FN(RobotBase::Manipulator,GetChildJoints))
        .def("GetChildDOFIndices",&PyRobotBase::PyManipulator::GetChildDOFIndices, DOXY_FN(RobotBase::Manipulator,GetChildDOFIndices))
        .def("GetChildLinks",&PyRobotBase::PyManipulator::GetChildLinks, DOXY_FN(RobotBase::Manipulator,GetChildLinks))
        .def("IsChildLink",&PyRobotBase::PyManipulator::IsChildLink, DOXY_FN(RobotBase::Manipulator,IsChildLink))
        .def("GetIndependentLinks",&PyRobotBase::PyManipulator::GetIndependentLinks, DOXY_FN(RobotBase::Manipulator,GetIndependentLinks))
        .def("GetArmConfigurationSpecification",&PyRobotBase::PyManipulator::GetArmConfigurationSpecification, GetArmConfigurationSpecification_overloads(args("interpolation"),DOXY_FN(RobotBase::Manipulator,GetArmConfigurationSpecification)))
        .def("GetIkConfigurationSpecification",&PyRobotBase::PyManipulator::GetIkConfigurationSpecification, GetIkConfigurationSpecification_overloads(args("iktype", "interpolation"),DOXY_FN(RobotBase::Manipulator,GetIkConfigurationSpecification)))
        .def("CheckEndEffectorCollision",pCheckEndEffectorCollision1,CheckEndEffectorCollision_overloads(args("transform", "report", "numredundantsamples"), DOXY_FN(RobotBase::Manipulator,CheckEndEffectorCollision)))
        .def("CheckEndEffectorCollision",pCheckEndEffectorCollision0,args("report"), DOXY_FN(RobotBase::Manipulator,CheckEndEffectorCollision))
        .def("CheckEndEffectorSelfCollision",pCheckEndEffectorSelfCollision0,args("report"), DOXY_FN(RobotBase::Manipulator,CheckEndEffectorSelfCollision))
        .def("CheckEndEffectorSelfCollision",pCheckEndEffectorSelfCollision1,CheckEndEffectorSelfCollision_overloads(args("transform", "report", "numredundantsamples","ignoreManipulatorLinks"), DOXY_FN(RobotBase::Manipulator,CheckEndEffectorSelfCollision)))
        .def("CheckIndependentCollision",pCheckIndependentCollision1, DOXY_FN(RobotBase::Manipulator,CheckIndependentCollision))
        .def("CheckIndependentCollision",pCheckIndependentCollision2,args("report"), DOXY_FN(RobotBase::Manipulator,CheckIndependentCollision))
        .def("CalculateJacobian",&PyRobotBase::PyManipulator::CalculateJacobian,DOXY_FN(RobotBase::Manipulator,CalculateJacobian))
        .def("CalculateRotationJacobian",&PyRobotBase::PyManipulator::CalculateRotationJacobian,DOXY_FN(RobotBase::Manipulator,CalculateRotationJacobian))
        .def("CalculateAngularVelocityJacobian",&PyRobotBase::PyManipulator::CalculateAngularVelocityJacobian,DOXY_FN(RobotBase::Manipulator,CalculateAngularVelocityJacobian))
        .def("GetStructureHash",&PyRobotBase::PyManipulator::GetStructureHash, DOXY_FN(RobotBase::Manipulator,GetStructureHash))
        .def("GetKinematicsStructureHash",&PyRobotBase::PyManipulator::GetKinematicsStructureHash, DOXY_FN(RobotBase::Manipulator,GetKinematicsStructureHash))
        .def("GetInverseKinematicsStructureHash",&PyRobotBase::PyManipulator::GetInverseKinematicsStructureHash, args("iktype"), DOXY_FN(RobotBase::Manipulator,GetInverseKinematicsStructureHash))
        .def("GetInfo",&PyRobotBase::PyManipulator::GetInfo, DOXY_FN(RobotBase::Manipulator,GetInfo))
        .def("__repr__",&PyRobotBase::PyManipulator::__repr__)
        .def("__str__",&PyRobotBase::PyManipulator::__str__)
        .def("__unicode__",&PyRobotBase::PyManipulator::__unicode__)
        .def("__eq__",&PyRobotBase::PyManipulator::__eq__)
        .def("__ne__",&PyRobotBase::PyManipulator::__ne__)
        .def("__hash__",&PyRobotBase::PyManipulator::__hash__)
        ;

        class_<PyRobotBase::PyAttachedSensor, OPENRAVE_SHARED_PTR<PyRobotBase::PyAttachedSensor> >("AttachedSensor", DOXY_CLASS(RobotBase::AttachedSensor), no_init)
        .def("GetSensor",&PyRobotBase::PyAttachedSensor::GetSensor, DOXY_FN(RobotBase::AttachedSensor,GetSensor))
        .def("GetAttachingLink",&PyRobotBase::PyAttachedSensor::GetAttachingLink, DOXY_FN(RobotBase::AttachedSensor,GetAttachingLink))
        .def("GetRelativeTransform",&PyRobotBase::PyAttachedSensor::GetRelativeTransform, DOXY_FN(RobotBase::AttachedSensor,GetRelativeTransform))
        .def("GetTransform",&PyRobotBase::PyAttachedSensor::GetTransform, DOXY_FN(RobotBase::AttachedSensor,GetTransform))
        .def("GetTransformPose",&PyRobotBase::PyAttachedSensor::GetTransformPose, DOXY_FN(RobotBase::AttachedSensor,GetTransform))
        .def("GetRobot",&PyRobotBase::PyAttachedSensor::GetRobot, DOXY_FN(RobotBase::AttachedSensor,GetRobot))
        .def("GetName",&PyRobotBase::PyAttachedSensor::GetName, DOXY_FN(RobotBase::AttachedSensor,GetName))
        .def("GetData",&PyRobotBase::PyAttachedSensor::GetData, DOXY_FN(RobotBase::AttachedSensor,GetData))
        .def("SetRelativeTransform",&PyRobotBase::PyAttachedSensor::SetRelativeTransform,args("transform"), DOXY_FN(RobotBase::AttachedSensor,SetRelativeTransform))
        .def("GetStructureHash",&PyRobotBase::PyAttachedSensor::GetStructureHash, DOXY_FN(RobotBase::AttachedSensor,GetStructureHash))
        .def("UpdateInfo",&PyRobotBase::PyAttachedSensor::UpdateInfo, UpdateInfo_overloads(args("type"), DOXY_FN(RobotBase::AttachedSensor,UpdateInfo)))
        .def("GetInfo",&PyRobotBase::PyAttachedSensor::GetInfo, DOXY_FN(RobotBase::AttachedSensor,GetInfo))
        .def("UpdateAndGetInfo",&PyRobotBase::PyAttachedSensor::UpdateAndGetInfo, UpdateAndGetInfo_overloads(DOXY_FN(RobotBase::AttachedSensor,UpdateAndGetInfo)))
        .def("__str__",&PyRobotBase::PyAttachedSensor::__str__)
        .def("__repr__",&PyRobotBase::PyAttachedSensor::__repr__)
        .def("__unicode__",&PyRobotBase::PyAttachedSensor::__unicode__)
        .def("__eq__",&PyRobotBase::PyAttachedSensor::__eq__)
        .def("__ne__",&PyRobotBase::PyAttachedSensor::__ne__)
        .def("__hash__",&PyRobotBase::PyAttachedSensor::__hash__)
        ;

        class_<PyRobotBase::PyConnectedBody, OPENRAVE_SHARED_PTR<PyRobotBase::PyConnectedBody> >("ConnectedBody", DOXY_CLASS(RobotBase::ConnectedBody), no_init)
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

        class_<PyRobotBase::PyRobotStateSaver, OPENRAVE_SHARED_PTR<PyRobotBase::PyRobotStateSaver> >("RobotStateSaver", DOXY_CLASS(Robot::RobotStateSaver), no_init)
        .def(init<PyRobotBasePtr>(args("robot")))
        .def(init<PyRobotBasePtr,object>(args("robot","options")))
        .def("GetBody",&PyRobotBase::PyRobotStateSaver::GetBody,DOXY_FN(Robot::RobotStateSaver, GetBody))
        .def("Restore",&PyRobotBase::PyRobotStateSaver::Restore,Restore_overloads(args("body"), DOXY_FN(Robot::RobotStateSaver, Restore)))
        .def("Release",&PyRobotBase::PyRobotStateSaver::Release,DOXY_FN(Robot::RobotStateSaver, Release))
        .def("__str__",&PyRobotBase::PyRobotStateSaver::__str__)
        .def("__unicode__",&PyRobotBase::PyRobotStateSaver::__unicode__)
        ;
    }

    def("RaveCreateRobot",openravepy::RaveCreateRobot,args("env","name"),DOXY_FN1(RaveCreateRobot));
}

}
