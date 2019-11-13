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
#ifndef OPENRAVEPY_INTERNAL_LINK_H
#define OPENRAVEPY_INTERNAL_LINK_H

#define NO_IMPORT_ARRAY
#include "openravepy_int.h"
#include "openravepy_linkinfo.h"
#include "openravepy_geometryinfo.h"

namespace openravepy
{

using py::object;

class PyLink
{
    KinBody::LinkPtr _plink;
    PyEnvironmentBasePtr _pyenv;
public:
    class PyGeometry
    {
        KinBody::Link::GeometryPtr _pgeometry;
public:
        PyGeometry(KinBody::Link::GeometryPtr pgeometry) : _pgeometry(pgeometry) {
        }

        virtual void SetCollisionMesh(py::object pytrimesh) {
            TriMesh mesh;
            if( ExtractTriMesh(pytrimesh,mesh) ) {
                _pgeometry->SetCollisionMesh(mesh);
            }
            else {
                throw openrave_exception(_("bad trimesh"));
            }
        }

        bool InitCollisionMesh(float fTessellation=1) {
            return _pgeometry->InitCollisionMesh(fTessellation);
        }
        uint8_t GetSideWallExists() const {
            return _pgeometry->GetSideWallExists();
        }

        py::object GetCollisionMesh() {
            return toPyTriMesh(_pgeometry->GetCollisionMesh());
        }
        py::object ComputeAABB(py::object otransform) const {
            return toPyAABB(_pgeometry->ComputeAABB(ExtractTransform(otransform)));
        }
        void SetDraw(bool bDraw) {
            _pgeometry->SetVisible(bDraw);
        }
        bool SetVisible(bool visible) {
            return _pgeometry->SetVisible(visible);
        }
        void SetTransparency(float f) {
            _pgeometry->SetTransparency(f);
        }
        void SetAmbientColor(py::object ocolor) {
            _pgeometry->SetAmbientColor(ExtractVector3(ocolor));
        }
        void SetDiffuseColor(py::object ocolor) {
            _pgeometry->SetDiffuseColor(ExtractVector3(ocolor));
        }
        void SetRenderFilename(const string& filename) {
            _pgeometry->SetRenderFilename(filename);
        }
        void SetName(const std::string& name) {
            _pgeometry->SetName(name);
        }
        bool IsDraw() {
            RAVELOG_WARN("IsDraw deprecated, use Geometry.IsVisible\n");
            return _pgeometry->IsVisible();
        }
        bool IsVisible() {
            return _pgeometry->IsVisible();
        }
        bool IsModifiable() {
            return _pgeometry->IsModifiable();
        }
        GeometryType GetType() {
            return _pgeometry->GetType();
        }
        py::object GetTransform() {
            return ReturnTransform(_pgeometry->GetTransform());
        }
        py::object GetTransformPose() {
            return toPyArray(_pgeometry->GetTransform());
        }
        dReal GetSphereRadius() const {
            return _pgeometry->GetSphereRadius();
        }
        dReal GetCylinderRadius() const {
            return _pgeometry->GetCylinderRadius();
        }
        dReal GetCylinderHeight() const {
            return _pgeometry->GetCylinderHeight();
        }
        py::object GetBoxExtents() const {
            return toPyVector3(_pgeometry->GetBoxExtents());
        }
        py::object GetContainerOuterExtents() const {
            return toPyVector3(_pgeometry->GetContainerOuterExtents());
        }
        py::object GetContainerInnerExtents() const {
            return toPyVector3(_pgeometry->GetContainerInnerExtents());
        }
        py::object GetContainerBottomCross() const {
            return toPyVector3(_pgeometry->GetContainerBottomCross());
        }
        py::object GetContainerBottom() const {
            return toPyVector3(_pgeometry->GetContainerBottom());
        }
        py::object GetRenderScale() const {
            return toPyVector3(_pgeometry->GetRenderScale());
        }
        py::object GetRenderFilename() const {
            return ConvertStringToUnicode(_pgeometry->GetRenderFilename());
        }
        py::object GetName() const {
            return ConvertStringToUnicode(_pgeometry->GetName());
        }
        float GetTransparency() const {
            return _pgeometry->GetTransparency();
        }
        py::object GetDiffuseColor() const {
            return toPyVector3(_pgeometry->GetDiffuseColor());
        }
        py::object GetAmbientColor() const {
            return toPyVector3(_pgeometry->GetAmbientColor());
        }
        py::object GetInfo() {
            return py::cast(PyGeometryInfoPtr(new PyGeometryInfo(_pgeometry->GetInfo())));
        }
        py::object ComputeInnerEmptyVolume() const
        {
            Transform tInnerEmptyVolume;
            Vector abInnerEmptyExtents;
            if( _pgeometry->ComputeInnerEmptyVolume(tInnerEmptyVolume, abInnerEmptyExtents) ) {
                return py::make_tuple(ReturnTransform(tInnerEmptyVolume), toPyVector3(abInnerEmptyExtents));
            }
            return py::make_tuple(py::object(), py::object());
        }
        bool __eq__(OPENRAVE_SHARED_PTR<PyGeometry> p) {
            return !!p && _pgeometry == p->_pgeometry;
        }
        bool __ne__(OPENRAVE_SHARED_PTR<PyGeometry> p) {
            return !p || _pgeometry != p->_pgeometry;
        }
        int __hash__() {
            return static_cast<int>(uintptr_t(_pgeometry.get()));
        }
    };

    PyLink(KinBody::LinkPtr plink, PyEnvironmentBasePtr pyenv) : _plink(plink), _pyenv(pyenv) {
    }
    virtual ~PyLink() {
    }

    KinBody::LinkPtr GetLink() {
        return _plink;
    }

    py::object GetName() {
        return ConvertStringToUnicode(_plink->GetName());
    }
    int GetIndex() {
        return _plink->GetIndex();
    }
    bool IsEnabled() const {
        return _plink->IsEnabled();
    }
    bool SetVisible(bool visible) {
        return _plink->SetVisible(visible);
    }
    bool IsVisible() const {
        return _plink->IsVisible();
    }
    bool IsStatic() const {
        return _plink->IsStatic();
    }
    void Enable(bool bEnable) {
        _plink->Enable(bEnable);
    }

    py::object GetParent() const
    {
        KinBodyPtr parent = _plink->GetParent();
        if( parent->IsRobot() ) {
            return py::cast(toPyRobot(RaveInterfaceCast<RobotBase>(_plink->GetParent()),_pyenv));
        }
        else {
            return py::cast(PyKinBodyPtr(new PyKinBody(_plink->GetParent(),_pyenv)));
        }
    }

    py::object GetParentLinks() const
    {
        std::vector<KinBody::LinkPtr> vParentLinks;
        _plink->GetParentLinks(vParentLinks);
        py::list links;
        FOREACHC(itlink, vParentLinks) {
            links.append(PyLinkPtr(new PyLink(*itlink, _pyenv)));
        }
        return std::move(links);
    }

    bool IsParentLink(OPENRAVE_SHARED_PTR<PyLink> pylink) const {
        return _plink->IsParentLink(*pylink->GetLink());
    }

    py::object GetCollisionData() {
        return toPyTriMesh(_plink->GetCollisionData());
    }
    py::object ComputeLocalAABB() const { // TODO py::object otransform=py::object()
        //if( IS_PYTHONOBJECT_NONE(otransform) ) {
        return toPyAABB(_plink->ComputeLocalAABB());
    }

    py::object ComputeAABB() const {
        return toPyAABB(_plink->ComputeAABB());
    }

    py::object ComputeAABBFromTransform(py::object otransform) const {
        return toPyAABB(_plink->ComputeAABBFromTransform(ExtractTransform(otransform)));
    }

    py::object GetTransform() const {
        return ReturnTransform(_plink->GetTransform());
    }
    py::object GetTransformPose() const {
        return toPyArray(_plink->GetTransform());
    }

    py::object GetCOMOffset() const {
        return toPyVector3(_plink->GetCOMOffset());
    }
    py::object GetLocalCOM() const {
        return toPyVector3(_plink->GetLocalCOM());
    }
    py::object GetGlobalCOM() const {
        return toPyVector3(_plink->GetGlobalCOM());
    }

    py::object GetLocalInertia() const {
        TransformMatrix t = _plink->GetLocalInertia();
        npy_intp dims[] = { 3, 3};
        PyObject *pyvalues = PyArray_SimpleNew(2,dims, sizeof(dReal)==8 ? PyArray_DOUBLE : PyArray_FLOAT);
        dReal* pdata = (dReal*)PyArray_DATA(pyvalues);
        pdata[0] = t.m[0]; pdata[1] = t.m[1]; pdata[2] = t.m[2];
        pdata[3] = t.m[4]; pdata[4] = t.m[5]; pdata[5] = t.m[6];
        pdata[6] = t.m[8]; pdata[7] = t.m[9]; pdata[8] = t.m[10];
        return toPyArrayN(pdata, 9);
        // return static_cast<np::array>(handle<>(pyvalues));
    }
    py::object GetGlobalInertia() const {
        TransformMatrix t = _plink->GetGlobalInertia();
        npy_intp dims[] = { 3, 3};
        PyObject *pyvalues = PyArray_SimpleNew(2,dims, sizeof(dReal)==8 ? PyArray_DOUBLE : PyArray_FLOAT);
        dReal* pdata = (dReal*)PyArray_DATA(pyvalues);
        pdata[0] = t.m[0]; pdata[1] = t.m[1]; pdata[2] = t.m[2];
        pdata[3] = t.m[4]; pdata[4] = t.m[5]; pdata[5] = t.m[6];
        pdata[6] = t.m[8]; pdata[7] = t.m[9]; pdata[8] = t.m[10];
        return toPyArrayN(pdata, 9);
        // return static_cast<np::array>(handle<>(pyvalues));
    }
    dReal GetMass() const {
        return _plink->GetMass();
    }
    py::object GetPrincipalMomentsOfInertia() const {
        return toPyVector3(_plink->GetPrincipalMomentsOfInertia());
    }
    py::object GetLocalMassFrame() const {
        return ReturnTransform(_plink->GetLocalMassFrame());
    }
    py::object GetGlobalMassFrame() const {
        return ReturnTransform(_plink->GetGlobalMassFrame());
    }
    void SetLocalMassFrame(py::object omassframe) {
        _plink->SetLocalMassFrame(ExtractTransform(omassframe));
    }
    void SetPrincipalMomentsOfInertia(py::object oinertiamoments) {
        _plink->SetPrincipalMomentsOfInertia(ExtractVector3(oinertiamoments));
    }
    void SetMass(dReal mass) {
        _plink->SetMass(mass);
    }

    void SetStatic(bool bStatic) {
        _plink->SetStatic(bStatic);
    }
    void SetTransform(py::object otrans) {
        _plink->SetTransform(ExtractTransform(otrans));
    }
    void SetForce(py::object oforce, py::object opos, bool bAdd) {
        return _plink->SetForce(ExtractVector3(oforce),ExtractVector3(opos),bAdd);
    }
    void SetTorque(py::object otorque, bool bAdd) {
        return _plink->SetTorque(ExtractVector3(otorque),bAdd);
    }

    py::object GetGeometries() {
        py::list geoms;
        size_t N = _plink->GetGeometries().size();
        for(size_t i = 0; i < N; ++i) {
            geoms.append(OPENRAVE_SHARED_PTR<PyGeometry>(new PyGeometry(_plink->GetGeometry(i))));
        }
        return std::move(geoms);
    }

    void InitGeometries(py::object ogeometryinfos)
    {
        std::vector<KinBody::GeometryInfoConstPtr> geometries(len(ogeometryinfos));
        for(size_t i = 0; i < geometries.size(); ++i) {
            PyGeometryInfoPtr pygeom = ogeometryinfos[i].cast<PyGeometryInfoPtr>();
            if( !pygeom ) {
                throw OPENRAVE_EXCEPTION_FORMAT0(_("cannot cast to KinBody.GeometryInfo"),ORE_InvalidArguments);
            }
            geometries[i] = pygeom->GetGeometryInfo();
        }
        return _plink->InitGeometries(geometries);
    }

    void AddGeometry(py::object ogeometryinfo, bool addToGroups)
    {
        PyGeometryInfoPtr pygeom = ogeometryinfo.cast<PyGeometryInfoPtr>();
        if( !pygeom ) {
            throw OPENRAVE_EXCEPTION_FORMAT0(_("cannot cast to KinBody.GeometryInfo"),ORE_InvalidArguments);
        }
        _plink->AddGeometry(pygeom->GetGeometryInfo(), addToGroups);
    }

    void RemoveGeometryByName(const std::string& geometryname, bool removeFromAllGroups)
    {
        _plink->RemoveGeometryByName(geometryname, removeFromAllGroups);
    }

    void SetGeometriesFromGroup(const std::string& name)
    {
        _plink->SetGeometriesFromGroup(name);
    }

    py::object GetGeometriesFromGroup(const std::string& name)
    {
        py::list ogeometryinfos;
        FOREACHC(itinfo, _plink->GetGeometriesFromGroup(name)) {
            ogeometryinfos.append(PyGeometryInfoPtr(new PyGeometryInfo(**itinfo)));
        }
        return std::move(ogeometryinfos);
    }

    void SetGroupGeometries(const std::string& name, py::object ogeometryinfos)
    {
        std::vector<KinBody::GeometryInfoPtr> geometries(len(ogeometryinfos));
        for(size_t i = 0; i < geometries.size(); ++i) {
            PyGeometryInfoPtr pygeom = ogeometryinfos[i].cast<PyGeometryInfoPtr>();
            if( !pygeom ) {
                throw OPENRAVE_EXCEPTION_FORMAT0(_("cannot cast to KinBody.GeometryInfo"),ORE_InvalidArguments);
            }
            geometries[i] = pygeom->GetGeometryInfo();
        }
        _plink->SetGroupGeometries(name, geometries);
    }

    int GetGroupNumGeometries(const std::string& geomname)
    {
        return _plink->GetGroupNumGeometries(geomname);
    }

    py::object GetRigidlyAttachedLinks() const {
        std::vector<KinBody::LinkPtr> vattachedlinks;
        _plink->GetRigidlyAttachedLinks(vattachedlinks);
        py::list links;
        FOREACHC(itlink, vattachedlinks) {
            links.append(PyLinkPtr(new PyLink(*itlink, _pyenv)));
        }
        return std::move(links);
    }

    bool IsRigidlyAttached(OPENRAVE_SHARED_PTR<PyLink> plink) {
        CHECK_POINTER(plink);
        return _plink->IsRigidlyAttached(*plink->GetLink());
    }

    void SetVelocity(py::object olinear, py::object oangular) {
        _plink->SetVelocity(ExtractVector3(olinear),ExtractVector3(oangular));
    }

    py::object GetVelocity() const {
        std::pair<Vector,Vector> velocity;
        velocity = _plink->GetVelocity();
        boost::array<dReal,6> v = {{ velocity.first.x, velocity.first.y, velocity.first.z, velocity.second.x, velocity.second.y, velocity.second.z}};
        return toPyArray<dReal,6>(v);
    }

    py::object GetFloatParameters(py::object oname=py::object(), int index=-1) const {
        return GetCustomParameters(_plink->GetFloatParameters(), oname, index);
    }

    void SetFloatParameters(const std::string& key, py::object oparameters)
    {
        _plink->SetFloatParameters(key,ExtractArray<dReal>(oparameters));
    }

    py::object GetIntParameters(py::object oname=py::object(), int index=-1) const {
        return GetCustomParameters(_plink->GetIntParameters(), oname, index);
    }

    void SetIntParameters(const std::string& key, py::object oparameters)
    {
        _plink->SetIntParameters(key,ExtractArray<int>(oparameters));
    }

    py::object GetStringParameters(py::object oname=py::object()) const
    {
        if( IS_PYTHONOBJECT_NONE(oname) ) {
            py::dict oparameters;
            FOREACHC(it, _plink->GetStringParameters()) {
                oparameters[it->first] = ConvertStringToUnicode(it->second);
            }
            return std::move(oparameters);
        }
        std::string name = oname.cast<std::string>();
        std::map<std::string, std::string >::const_iterator it = _plink->GetStringParameters().find(name);
        if( it != _plink->GetStringParameters().end() ) {
            return ConvertStringToUnicode(it->second);
        }
        return py::object();
    }

    void SetStringParameters(const std::string& key, py::object ovalue)
    {
        _plink->SetStringParameters(key,ovalue.cast<std::string>());
    }

    void UpdateInfo() {
        _plink->UpdateInfo();
    }
    py::object GetInfo() {
        return py::cast(PyLinkInfoPtr(new PyLinkInfo(_plink->GetInfo())));
    }
    py::object UpdateAndGetInfo() {
        return py::cast(PyLinkInfoPtr(new PyLinkInfo(_plink->UpdateAndGetInfo())));
    }


    std::string __repr__() {
        return boost::str(boost::format("RaveGetEnvironment(%d).GetKinBody('%s').GetLink('%s')")%RaveGetEnvironmentId(_plink->GetParent()->GetEnv())%_plink->GetParent()->GetName()%_plink->GetName());
    }
    std::string __str__() {
        return boost::str(boost::format("<link:%s (%d), parent=%s>")%_plink->GetName()%_plink->GetIndex()%_plink->GetParent()->GetName());
    }
    py::object __unicode__() {
        return ConvertStringToUnicode(__str__());
    }
    bool __eq__(OPENRAVE_SHARED_PTR<PyLink> p) {
        return !!p && _plink == p->_plink;
    }
    bool __ne__(OPENRAVE_SHARED_PTR<PyLink> p) {
        return !p || _plink != p->_plink;
    }
    int __hash__() {
        return static_cast<int>(uintptr_t(_plink.get()));
    }
};

} // namespace openravepy

#endif // OPENRAVEPY_INTERNAL_LINK_H
