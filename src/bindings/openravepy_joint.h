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
#ifndef OPENRAVEPY_INTERNAL_JOINT_H
#define OPENRAVEPY_INTERNAL_JOINT_H

#define NO_IMPORT_ARRAY
#include "openravepy_int.h"
#include "openravepy_jointinfo.h"
#include "openravepy_link.h"

namespace openravepy
{

using py::object;

class PyJoint
{
    KinBody::JointPtr _pjoint;
    PyEnvironmentBasePtr _pyenv;
public:
    PyJoint(KinBody::JointPtr pjoint, PyEnvironmentBasePtr pyenv) : _pjoint(pjoint), _pyenv(pyenv) {
    }
    virtual ~PyJoint() {
    }

    KinBody::JointPtr GetJoint() {
        return _pjoint;
    }

    py::object GetName() {
        return ConvertStringToUnicode(_pjoint->GetName());
    }
    bool IsMimic(int iaxis=-1) {
        return _pjoint->IsMimic(iaxis);
    }
    string GetMimicEquation(int iaxis=0, int itype=0, const std::string& format="") {
        return _pjoint->GetMimicEquation(iaxis,itype,format);
    }
    py::object GetMimicDOFIndices(int iaxis=0) {
        std::vector<int> vmimicdofs;
        _pjoint->GetMimicDOFIndices(vmimicdofs,iaxis);
        return toPyArray(vmimicdofs);
    }
    void SetMimicEquations(int iaxis, const std::string& poseq, const std::string& veleq, const std::string& acceleq) {
        _pjoint->SetMimicEquations(iaxis,poseq,veleq,acceleq);
    }

    dReal GetMaxVel(int iaxis=0) const {
        return _pjoint->GetMaxVel(iaxis);
    }
    dReal GetMaxAccel(int iaxis=0) const {
        return _pjoint->GetMaxAccel(iaxis);
    }
    dReal GetMaxJerk(int iaxis=0) const {
        return _pjoint->GetMaxJerk(iaxis);
    }
    dReal GetMaxTorque(int iaxis=0) const {
        return _pjoint->GetMaxTorque(iaxis);
    }
    py::object GetInstantaneousTorqueLimits(int iaxis=0) const {
        std::pair<dReal, dReal> values = _pjoint->GetInstantaneousTorqueLimits(iaxis);
        return py::make_tuple(values.first, values.second);
    }
    py::object GetNominalTorqueLimits(int iaxis=0) const {
        std::pair<dReal, dReal> values = _pjoint->GetNominalTorqueLimits(iaxis);
        return py::make_tuple(values.first, values.second);
    }

    dReal GetMaxInertia(int iaxis=0) const {
        return _pjoint->GetMaxInertia(iaxis);
    }

    int GetDOFIndex() const {
        return _pjoint->GetDOFIndex();
    }
    int GetJointIndex() const {
        return _pjoint->GetJointIndex();
    }

    PyKinBodyPtr GetParent() const {
        return PyKinBodyPtr(new PyKinBody(_pjoint->GetParent(),_pyenv));
    }

    PyLinkPtr GetFirstAttached() const {
        return !_pjoint->GetFirstAttached() ? PyLinkPtr() : PyLinkPtr(new PyLink(_pjoint->GetFirstAttached(), _pyenv));
    }
    PyLinkPtr GetSecondAttached() const {
        return !_pjoint->GetSecondAttached() ? PyLinkPtr() : PyLinkPtr(new PyLink(_pjoint->GetSecondAttached(), _pyenv));
    }

    KinBody::JointType GetType() const {
        return _pjoint->GetType();
    }
    bool IsCircular(int iaxis) const {
        return _pjoint->IsCircular(iaxis);
    }
    bool IsRevolute(int iaxis) const {
        return _pjoint->IsRevolute(iaxis);
    }
    bool IsPrismatic(int iaxis) const {
        return _pjoint->IsPrismatic(iaxis);
    }
    bool IsStatic() const {
        return _pjoint->IsStatic();
    }

    int GetDOF() const {
        return _pjoint->GetDOF();
    }
    py::object GetValues() const {
        vector<dReal> values;
        _pjoint->GetValues(values);
        return toPyArray(values);
    }
    dReal GetValue(int iaxis) const {
        return _pjoint->GetValue(iaxis);
    }
    py::object GetVelocities() const {
        vector<dReal> values;
        _pjoint->GetVelocities(values);
        return toPyArray(values);
    }

    py::object GetAnchor() const {
        return toPyVector3(_pjoint->GetAnchor());
    }
    py::object GetAxis(int iaxis=0) {
        return toPyVector3(_pjoint->GetAxis(iaxis));
    }
    PyLinkPtr GetHierarchyParentLink() const {
        return !_pjoint->GetHierarchyParentLink() ? PyLinkPtr() : PyLinkPtr(new PyLink(_pjoint->GetHierarchyParentLink(),_pyenv));
    }
    PyLinkPtr GetHierarchyChildLink() const {
        return !_pjoint->GetHierarchyChildLink() ? PyLinkPtr() : PyLinkPtr(new PyLink(_pjoint->GetHierarchyChildLink(),_pyenv));
    }
    py::object GetInternalHierarchyAxis(int iaxis) {
        return toPyVector3(_pjoint->GetInternalHierarchyAxis(iaxis));
    }
    py::object GetInternalHierarchyLeftTransform() {
        return ReturnTransform(_pjoint->GetInternalHierarchyLeftTransform());
    }
    py::object GetInternalHierarchyLeftTransformPose() {
        return toPyArray(_pjoint->GetInternalHierarchyLeftTransform());
    }
    py::object GetInternalHierarchyRightTransform() {
        return ReturnTransform(_pjoint->GetInternalHierarchyRightTransform());
    }
    py::object GetInternalHierarchyRightTransformPose() {
        return toPyArray(_pjoint->GetInternalHierarchyRightTransform());
    }

    py::object GetLimits() const {
        vector<dReal> lower, upper;
        _pjoint->GetLimits(lower,upper);
        return py::make_tuple(toPyArray(lower),toPyArray(upper));
    }
    py::object GetVelocityLimits() const {
        vector<dReal> vlower,vupper;
        _pjoint->GetVelocityLimits(vlower,vupper);
        return py::make_tuple(toPyArray(vlower),toPyArray(vupper));
    }
    py::object GetAccelerationLimits() const {
        vector<dReal> v;
        _pjoint->GetAccelerationLimits(v);
        return toPyArray(v);
    }
    py::object GetJerkLimits() const {
        vector<dReal> v;
        _pjoint->GetJerkLimits(v);
        return toPyArray(v);
    }
    py::object GetHardVelocityLimits() const {
        vector<dReal> v;
        _pjoint->GetHardVelocityLimits(v);
        return toPyArray(v);
    }
    py::object GetHardAccelerationLimits() const {
        vector<dReal> v;
        _pjoint->GetHardAccelerationLimits(v);
        return toPyArray(v);
    }
    py::object GetHardJerkLimits() const {
        vector<dReal> v;
        _pjoint->GetHardJerkLimits(v);
        return toPyArray(v);
    }
    py::object GetTorqueLimits() const {
        vector<dReal> v;
        _pjoint->GetTorqueLimits(v);
        return toPyArray(v);
    }

    dReal GetWrapOffset(int iaxis=0) {
        return _pjoint->GetWrapOffset(iaxis);
    }
    void SetWrapOffset(dReal offset, int iaxis=0) {
        _pjoint->SetWrapOffset(offset,iaxis);
    }
    void SetLimits(py::object olower, py::object oupper) {
        vector<dReal> vlower = ExtractArray<dReal>(olower);
        vector<dReal> vupper = ExtractArray<dReal>(oupper);
        if(( vlower.size() != vupper.size()) ||( (int)vlower.size() != _pjoint->GetDOF()) ) {
            throw openrave_exception(_("limits are wrong dimensions"));
        }
        _pjoint->SetLimits(vlower,vupper);
    }
    void SetVelocityLimits(py::object omaxlimits) {
        vector<dReal> vmaxlimits = ExtractArray<dReal>(omaxlimits);
        if( (int)vmaxlimits.size() != _pjoint->GetDOF() ) {
            throw openrave_exception(_("limits are wrong dimensions"));
        }
        _pjoint->SetVelocityLimits(vmaxlimits);
    }
    void SetAccelerationLimits(py::object omaxlimits) {
        vector<dReal> vmaxlimits = ExtractArray<dReal>(omaxlimits);
        if( (int)vmaxlimits.size() != _pjoint->GetDOF() ) {
            throw openrave_exception(_("limits are wrong dimensions"));
        }
        _pjoint->SetAccelerationLimits(vmaxlimits);
    }
    void SetJerkLimits(py::object omaxlimits) {
        vector<dReal> vmaxlimits = ExtractArray<dReal>(omaxlimits);
        if( (int)vmaxlimits.size() != _pjoint->GetDOF() ) {
            throw openrave_exception(_("limits are wrong dimensions"));
        }
        _pjoint->SetJerkLimits(vmaxlimits);
    }
    void SetHardVelocityLimits(py::object omaxlimits) {
        vector<dReal> vmaxlimits = ExtractArray<dReal>(omaxlimits);
        if( (int)vmaxlimits.size() != _pjoint->GetDOF() ) {
            throw openrave_exception(_("limits are wrong dimensions"));
        }
        _pjoint->SetHardVelocityLimits(vmaxlimits);
    }
    void SetHardAccelerationLimits(py::object omaxlimits) {
        vector<dReal> vmaxlimits = ExtractArray<dReal>(omaxlimits);
        if( (int)vmaxlimits.size() != _pjoint->GetDOF() ) {
            throw openrave_exception(_("limits are wrong dimensions"));
        }
        _pjoint->SetHardAccelerationLimits(vmaxlimits);
    }
    void SetHardJerkLimits(py::object omaxlimits) {
        vector<dReal> vmaxlimits = ExtractArray<dReal>(omaxlimits);
        if( (int)vmaxlimits.size() != _pjoint->GetDOF() ) {
            throw openrave_exception(_("limits are wrong dimensions"));
        }
        _pjoint->SetHardJerkLimits(vmaxlimits);
    }
    void SetTorqueLimits(py::object omaxlimits) {
        vector<dReal> vmaxlimits = ExtractArray<dReal>(omaxlimits);
        if( (int)vmaxlimits.size() != _pjoint->GetDOF() ) {
            throw openrave_exception(_("limits are wrong dimensions"));
        }
        _pjoint->SetTorqueLimits(vmaxlimits);
    }

    py::object GetResolutions() const {
        vector<dReal> resolutions;
        _pjoint->GetResolutions(resolutions);
        return toPyArray(resolutions);
    }
    dReal GetResolution(int iaxis) {
        return _pjoint->GetResolution(iaxis);
    }
    void SetResolution(dReal resolution) {
        _pjoint->SetResolution(resolution);
    }

    py::object GetWeights() const {
        vector<dReal> weights;
        _pjoint->GetWeights(weights);
        return toPyArray(weights);
    }
    dReal GetWeight(int iaxis) {
        return _pjoint->GetWeight(iaxis);
    }
    void SetWeights(py::object o) {
        _pjoint->SetWeights(ExtractArray<dReal>(o));
    }

    py::object SubtractValues(py::object ovalues0, py::object ovalues1) {
        vector<dReal> values0 = ExtractArray<dReal>(ovalues0);
        vector<dReal> values1 = ExtractArray<dReal>(ovalues1);
        BOOST_ASSERT((int)values0.size() == GetDOF() );
        BOOST_ASSERT((int)values1.size() == GetDOF() );
        _pjoint->SubtractValues(values0,values1);
        return toPyArray(values0);
    }

    dReal SubtractValue(dReal value0, dReal value1, int iaxis) {
        return _pjoint->SubtractValue(value0,value1,iaxis);
    }

    void AddTorque(py::object otorques) {
        vector<dReal> vtorques = ExtractArray<dReal>(otorques);
        return _pjoint->AddTorque(vtorques);
    }

    py::object GetFloatParameters(py::object oname=py::object(), int index=-1) const {
        return GetCustomParameters(_pjoint->GetFloatParameters(), oname, index);
    }

    void SetFloatParameters(const std::string& key, py::object oparameters)
    {
        _pjoint->SetFloatParameters(key,ExtractArray<dReal>(oparameters));
    }

    py::object GetIntParameters(py::object oname=py::object(), int index=-1) const {
        return GetCustomParameters(_pjoint->GetIntParameters(), oname, index);
    }

    void SetIntParameters(const std::string& key, py::object oparameters)
    {
        _pjoint->SetIntParameters(key,ExtractArray<int>(oparameters));
    }

    py::object GetStringParameters(py::object oname=py::object()) const {
        if( IS_PYTHONOBJECT_NONE(oname) ) {
            py::dict oparameters;
            FOREACHC(it, _pjoint->GetStringParameters()) {
                oparameters[it->first] = ConvertStringToUnicode(it->second);
            }
            return std::move(oparameters);
        }
        std::string name = oname.cast<std::string>();
        std::map<std::string, std::string >::const_iterator it = _pjoint->GetStringParameters().find(name);
        if( it != _pjoint->GetStringParameters().end() ) {
            return ConvertStringToUnicode(it->second);
        }
        return py::object();
    }

    void SetStringParameters(const std::string& key, py::object ovalue)
    {
        _pjoint->SetStringParameters(key,ovalue.cast<std::string>());
    }

    void UpdateInfo() {
        _pjoint->UpdateInfo();
    }
    py::object GetInfo() {
        return py::cast(PyJointInfoPtr(new PyJointInfo(_pjoint->GetInfo(), _pyenv)));
    }
    py::object UpdateAndGetInfo() {
        return py::cast(PyJointInfoPtr(new PyJointInfo(_pjoint->UpdateAndGetInfo(), _pyenv)));
    }

    string __repr__() {
        return boost::str(boost::format("RaveGetEnvironment(%d).GetKinBody('%s').GetJoint('%s')")%RaveGetEnvironmentId(_pjoint->GetParent()->GetEnv())%_pjoint->GetParent()->GetName()%_pjoint->GetName());
    }
    string __str__() {
        return boost::str(boost::format("<joint:%s (%d), dof=%d, parent=%s>")%_pjoint->GetName()%_pjoint->GetJointIndex()%_pjoint->GetDOFIndex()%_pjoint->GetParent()->GetName());
    }
    py::object __unicode__() {
        return ConvertStringToUnicode(__str__());
    }
    bool __eq__(OPENRAVE_SHARED_PTR<PyJoint> p) {
        return !!p && _pjoint==p->_pjoint;
    }
    bool __ne__(OPENRAVE_SHARED_PTR<PyJoint> p) {
        return !p || _pjoint!=p->_pjoint;
    }
    int __hash__() {
        return static_cast<int>(uintptr_t(_pjoint.get()));
    }
};

} // namespace openravepy

#endif // OPENRAVEPY_INTERNAL_JOINT_H
