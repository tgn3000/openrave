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
#ifndef OPENRAVEPY_INTERNAL_JOINTINFO_H
#define OPENRAVEPY_INTERNAL_JOINTINFO_H

#define NO_IMPORT_ARRAY
#include "openravepy_int.h"
#include "openravepy_electricmotoractuatorinfo.h"

namespace openravepy
{

using py::object;

class PyJointInfo
{
public:
    PyJointInfo() {
        _type = KinBody::JointNone;
        _vanchor = toPyVector3(Vector());
        _vaxes = py::list();
        _vresolution = toPyVector3(Vector(0.02,0.02,0.02));
        // As for initial values for vel, accel, and jerk, please see kinbodyjoint.cpp's definition.
        _vmaxvel = toPyVector3(Vector(10,10,10));
        _vhardmaxvel = toPyVector3(Vector(0,0,0));
        _vmaxaccel = toPyVector3(Vector(50,50,50));
        _vhardmaxaccel = toPyVector3(Vector(0,0,0));
        _vmaxjerk = toPyVector3(Vector(2e6,2e6,2e6));
        _vhardmaxjerk = toPyVector3(Vector(0, 0, 0));
        _vmaxtorque = toPyVector3(Vector(1e5,1e5,1e5));
        _vmaxinertia = toPyVector3(Vector(1e5,1e5,1e5));
        _vweights = toPyVector3(Vector(1,1,1));
        _voffsets = toPyVector3(Vector(0,0,0));
        _vlowerlimit = toPyVector3(Vector(0,0,0));
        _vupperlimit = toPyVector3(Vector(0,0,0));
        _bIsCircular = py::list();
        _bIsActive = true;
    }

    PyJointInfo(const KinBody::JointInfo& info, PyEnvironmentBasePtr pyenv) {
        _type = info._type;
        _name = ConvertStringToUnicode(info._name);
        _linkname0 = ConvertStringToUnicode(info._linkname0);
        _linkname1 = ConvertStringToUnicode(info._linkname1);
        _vanchor = toPyVector3(info._vanchor);
        py::list vaxes;
        for(size_t i = 0; i < info._vaxes.size(); ++i) {
            vaxes.append(toPyVector3(info._vaxes[i]));
        }
        _vaxes = vaxes;
        _vcurrentvalues = toPyArray(info._vcurrentvalues);
        _vresolution = toPyArray<dReal,3>(info._vresolution);
        _vmaxvel = toPyArray<dReal,3>(info._vmaxvel);
        _vhardmaxvel = toPyArray<dReal,3>(info._vhardmaxvel);
        _vmaxaccel = toPyArray<dReal,3>(info._vmaxaccel);
        _vhardmaxaccel = toPyArray<dReal,3>(info._vhardmaxaccel);
        _vmaxjerk = toPyArray<dReal,3>(info._vmaxjerk);
        _vhardmaxjerk = toPyArray<dReal,3>(info._vhardmaxjerk);
        _vmaxtorque = toPyArray<dReal,3>(info._vmaxtorque);
        _vmaxinertia = toPyArray<dReal,3>(info._vmaxinertia);
        _vweights = toPyArray<dReal,3>(info._vweights);
        _voffsets = toPyArray<dReal,3>(info._voffsets);
        _vlowerlimit = toPyArray<dReal,3>(info._vlowerlimit);
        _vupperlimit = toPyArray<dReal,3>(info._vupperlimit);
        _trajfollow = py::cast(toPyTrajectory(info._trajfollow, pyenv));
        FOREACHC(itmimic, info._vmimic) {
            if( !*itmimic ) {
                _vmimic.append(py::object());
            }
            else {
                py::list oequations;
                FOREACHC(itequation, (*itmimic)->_equations) {
                    oequations.append(*itequation);
                }
                _vmimic.append(oequations);
            }
        }
        FOREACHC(it, info._mapFloatParameters) {
            _mapFloatParameters[it->first] = toPyArray(it->second);
        }
        FOREACHC(it, info._mapIntParameters) {
            _mapIntParameters[it->first] = toPyArray(it->second);
        }
        FOREACHC(it, info._mapStringParameters) {
            _mapStringParameters[it->first] = ConvertStringToUnicode(it->second);
        }
        py::list bIsCircular;
        FOREACHC(it, info._bIsCircular) {
            bIsCircular.append(*it);
        }
        _bIsCircular = bIsCircular;
        _bIsActive = info._bIsActive;
        if( !!info._infoElectricMotor ) {
            _infoElectricMotor = PyElectricMotorActuatorInfoPtr(new PyElectricMotorActuatorInfo(*info._infoElectricMotor));
        }
    }

    KinBody::JointInfoPtr GetJointInfo() {
        KinBody::JointInfoPtr pinfo(new KinBody::JointInfo());
        KinBody::JointInfo& info = *pinfo;
        info._type = _type;
        if( !IS_PYTHONOBJECT_NONE(_name) ) {
            info._name = _name.cast<std::string>();
        }
        if( !IS_PYTHONOBJECT_NONE(_linkname0) ) {
            info._linkname0 = _linkname0.cast<std::string>();
        }
        if( !IS_PYTHONOBJECT_NONE(_linkname1) ) {
            info._linkname1 = _linkname1.cast<std::string>();
        }
        info._vanchor = ExtractVector3(_vanchor);

        // We might be able to replace these exceptions with static_assert in C++11
        size_t num = len(_vaxes);
        OPENRAVE_EXCEPTION_FORMAT0(num == info._vaxes.size(), ORE_InvalidState);
        for(size_t i = 0; i < num; ++i) {
            info._vaxes[i] = ExtractVector3(_vaxes[i]);
        }

        if( !IS_PYTHONOBJECT_NONE(_vcurrentvalues) ) {
            info._vcurrentvalues = ExtractArray<dReal>(_vcurrentvalues);
        }

        num = len(_vresolution);
        OPENRAVE_EXCEPTION_FORMAT0(num == info._vresolution.size(), ORE_InvalidState);
        for(size_t i = 0; i < num; ++i) {
            info._vresolution[i] = _vresolution[i].cast<dReal>();
        }

        num = len(_vmaxvel);
        OPENRAVE_EXCEPTION_FORMAT0(num == info._vmaxvel.size(), ORE_InvalidState);
        for(size_t i = 0; i < num; ++i) {
            info._vmaxvel[i] = _vmaxvel[i].cast<dReal>();
        }

        num = len(_vhardmaxvel);
        OPENRAVE_EXCEPTION_FORMAT0(num == info._vhardmaxvel.size(), ORE_InvalidState);
        for(size_t i = 0; i < num; ++i) {
            info._vhardmaxvel[i] = _vhardmaxvel[i].cast<dReal>();
        }

        num = len(_vmaxaccel);
        OPENRAVE_EXCEPTION_FORMAT0(num == info._vmaxaccel.size(), ORE_InvalidState);
        for(size_t i = 0; i < num; ++i) {
            info._vmaxaccel[i] = _vmaxaccel[i].cast<dReal>();
        }

        num = len(_vhardmaxaccel);
        OPENRAVE_EXCEPTION_FORMAT0(num == info._vhardmaxaccel.size(), ORE_InvalidState);
        for(size_t i = 0; i < num; ++i) {
            info._vhardmaxaccel[i] = _vhardmaxaccel[i].cast<dReal>();
        }

        num = len(_vmaxjerk);
        OPENRAVE_EXCEPTION_FORMAT0(num == info._vmaxjerk.size(), ORE_InvalidState);
        for(size_t i = 0; i < num; ++i) {
            info._vmaxjerk[i] = _vmaxjerk[i].cast<dReal>();
        }

        num = len(_vhardmaxjerk);
        OPENRAVE_EXCEPTION_FORMAT0(num == info._vhardmaxjerk.size(), ORE_InvalidState);
        for(size_t i = 0; i < num; ++i) {
            info._vhardmaxjerk[i] = _vhardmaxjerk[i].cast<dReal>();
        }

        num = len(_vmaxtorque);
        OPENRAVE_EXCEPTION_FORMAT0(num == info._vmaxtorque.size(), ORE_InvalidState);
        for(size_t i = 0; i < num; ++i) {
            info._vmaxtorque[i] = _vmaxtorque[i].cast<dReal>();
        }

        num = len(_vmaxinertia);
        OPENRAVE_EXCEPTION_FORMAT0(num == info._vmaxinertia.size(), ORE_InvalidState);
        for(size_t i = 0; i < num; ++i) {
            info._vmaxinertia[i] = _vmaxinertia[i].cast<dReal>();
        }

        num = len(_vweights);
        OPENRAVE_EXCEPTION_FORMAT0(num == info._vweights.size(), ORE_InvalidState);
        for(size_t i = 0; i < num; ++i) {
            info._vweights[i] = _vweights[i].cast<dReal>();
        }

        num = len(_voffsets);
        OPENRAVE_EXCEPTION_FORMAT0(num == info._voffsets.size(), ORE_InvalidState);
        for(size_t i = 0; i < num; ++i) {
            info._voffsets[i] = _voffsets[i].cast<dReal>();
        }

        num = len(_vlowerlimit);
        OPENRAVE_EXCEPTION_FORMAT0(num == info._vlowerlimit.size(), ORE_InvalidState);
        for(size_t i = 0; i < num; ++i) {
            info._vlowerlimit[i] = _vlowerlimit[i].cast<dReal>();
        }

        num = len(_vupperlimit);
        OPENRAVE_EXCEPTION_FORMAT0(num == info._vupperlimit.size(), ORE_InvalidState);
        for(size_t i = 0; i < num; ++i) {
            info._vupperlimit[i] = _vupperlimit[i].cast<dReal>();
        }

        if( !IS_PYTHONOBJECT_NONE(_trajfollow) ) {
            info._trajfollow = GetTrajectory(_trajfollow);
        }
        if( !IS_PYTHONOBJECT_NONE(_vmimic) ) {
            num = len(_vmimic);
            for(size_t i = 0; i < num; ++i) {
                py::object omimic = _vmimic[i];
                if( !IS_PYTHONOBJECT_NONE(omimic) ) {
                    OPENRAVE_ASSERT_OP(len(omimic),==,3);
                    info._vmimic[i].reset(new KinBody::MimicInfo());
                    for(size_t j = 0; j < 3; ++j) {
                        info._vmimic[i]->_equations.at(j) = omimic[j].cast<std::string>();
                    }
                }
            }
        }
        // num = len(_mapFloatParameters);
        // py::object okeyvalueiter = _mapFloatParameters.iteritems();
        info._mapFloatParameters.clear();
        // for(size_t i = 0; i < num; ++i) {
        //     py::object okeyvalue = okeyvalueiter.attr("next") ();
        //     std::string name = okeyvalue[0].cast<std::string>();
        //     info._mapFloatParameters[name] = ExtractArray<dReal>(okeyvalue[1]);
        // }
        for(auto item : _mapFloatParameters) {
            std::string name = item.first.cast<std::string>();
            info._mapFloatParameters[name] = ExtractArray<dReal>(item.second.cast<py::object>());
        }
        // okeyvalueiter = _mapIntParameters.iteritems();
        // num = len(_mapIntParameters);
        info._mapIntParameters.clear();
        // for(size_t i = 0; i < num; ++i) {
        //     py::object okeyvalue = okeyvalueiter.attr("next") ();
        //     std::string name = okeyvalue[0].cast<std::string>();
        //     info._mapIntParameters[name] = ExtractArray<int>(okeyvalue[1]);
        // }
        for(auto item : _mapIntParameters) {
            std::string name = item.first.cast<std::string>();
            info._mapIntParameters[name] = ExtractArray<int>(item.second.cast<py::object>());
        }
        // okeyvalueiter = _mapStringParameters.iteritems();
        // num = len(_mapStringParameters);
        info._mapStringParameters.clear();
        // for(size_t i = 0; i < num; ++i) {
        //     py::object okeyvalue = okeyvalueiter.attr("next") ();
        //     std::string name = okeyvalue[0].cast<std::string>();
        //     info._mapStringParameters[name] = (std::string)okeyvalue[1].cast<std::string>();
        // }
        for(auto item : _mapStringParameters) {
            std::string name = item.first.cast<std::string>();
            info._mapStringParameters[name] = item.second.cast<std::string>();
        }
        num = len(_bIsCircular);
        for(size_t i = 0; i < num; ++i) {
            info._bIsCircular.at(i) = _bIsCircular[i].cast<int>()!=0;
        }
        info._bIsActive = _bIsActive;
        if( !!_infoElectricMotor ) {
            //PyElectricMotorActuatorInfoPtr pinfo = _infoElectricMotor.cast<PyElectricMotorActuatorInfoPtr>();
            //if( !!pinfo ) {
            info._infoElectricMotor = _infoElectricMotor->GetElectricMotorActuatorInfo();
            //}
        }
        return pinfo;
    }
    KinBody::JointType _type;
    py::object _name;
    py::object _linkname0, _linkname1;
    py::object _vanchor, _vaxes, _vcurrentvalues, _vresolution, _vmaxvel, _vhardmaxvel, _vmaxaccel, _vhardmaxaccel, _vmaxjerk, _vhardmaxjerk, _vmaxtorque, _vmaxinertia, _vweights, _voffsets, _vlowerlimit, _vupperlimit;
    py::object _trajfollow;
    PyElectricMotorActuatorInfoPtr _infoElectricMotor;
    py::list _vmimic;
    py::dict _mapFloatParameters, _mapIntParameters, _mapStringParameters;
    py::object _bIsCircular;
    bool _bIsActive;
};

} // namespace openravepy

#endif // OPENRAVEPY_INTERNAL_JOINTINFO_H