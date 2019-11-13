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
#ifndef OPENRAVEPY_INTERNAL_GEOMETRYINFO_H
#define OPENRAVEPY_INTERNAL_GEOMETRYINFO_H

#define NO_IMPORT_ARRAY
#include "openravepy_int.h"
#include "openravepy_sidewall.h"

namespace openravepy
{

using py::object;

class PyGeometryInfo
{
public:
    PyGeometryInfo() {
        _t = ReturnTransform(Transform());
        _vGeomData = toPyVector4(Vector());
        _vGeomData2 = toPyVector4(Vector());
        _vGeomData3 = toPyVector4(Vector());
        _vGeomData4 = toPyVector4(Vector());
        _vDiffuseColor = toPyVector3(Vector(1,1,1));
        _vAmbientColor = toPyVector3(Vector(0,0,0));
        _type = GT_None;
        _fTransparency = 0;
        _vRenderScale = toPyVector3(Vector(1,1,1));
        _vCollisionScale = toPyVector3(Vector(1,1,1));
        _bVisible = true;
        _bModifiable = true;
        _vSideWalls = py::list();
    }
    PyGeometryInfo(const KinBody::GeometryInfo& info) {
        Init(info);
    }


    void Init(const KinBody::GeometryInfo& info) {
        _t = ReturnTransform(info._t);
        _vGeomData = toPyVector4(info._vGeomData);
        _vGeomData2 = toPyVector4(info._vGeomData2);
        _vGeomData3 = toPyVector4(info._vGeomData3);
        _vGeomData4 = toPyVector4(info._vGeomData4);

        _vSideWalls = py::list();
        for (size_t i = 0; i < info._vSideWalls.size(); ++i) {
            _vSideWalls.append(PySideWall(info._vSideWalls[i]));
        }

        _vDiffuseColor = toPyVector3(info._vDiffuseColor);
        _vAmbientColor = toPyVector3(info._vAmbientColor);
        _meshcollision = toPyTriMesh(info._meshcollision);
        _type = info._type;
        _name = ConvertStringToUnicode(info._name);
        _filenamerender = ConvertStringToUnicode(info._filenamerender);
        _filenamecollision = ConvertStringToUnicode(info._filenamecollision);
        _vRenderScale = toPyVector3(info._vRenderScale);
        _vCollisionScale = toPyVector3(info._vCollisionScale);
        _fTransparency = info._fTransparency;
        _bVisible = info._bVisible;
        _bModifiable = info._bModifiable;
        //TODO
        //_mapExtraGeometries = info. _mapExtraGeometries;
    }

    py::object ComputeInnerEmptyVolume()
    {
        Transform tInnerEmptyVolume;
        Vector abInnerEmptyExtents;
        KinBody::GeometryInfoPtr pgeominfo = GetGeometryInfo();
        if( pgeominfo->ComputeInnerEmptyVolume(tInnerEmptyVolume, abInnerEmptyExtents) ) {
            return py::make_tuple(ReturnTransform(tInnerEmptyVolume), toPyVector3(abInnerEmptyExtents));
        }
        return py::make_tuple(py::object(), py::object());
    }

    py::object ComputeAABB(py::object otransform) {
        KinBody::GeometryInfoPtr pgeominfo = GetGeometryInfo();
        return toPyAABB(pgeominfo->ComputeAABB(ExtractTransform(otransform)));
    }

#ifdef OPENRAVE_RAPIDJSON
    void DeserializeJSON(py::object obj, const dReal fUnitScale=1.0)
    {
        rapidjson::Document doc;
        toRapidJSONValue(obj, doc, doc.GetAllocator());
        KinBody::GeometryInfoPtr pgeominfo = GetGeometryInfo();
        pgeominfo->DeserializeJSON(doc, fUnitScale);
        Init(*pgeominfo);
    }

    py::object SerializeJSON(const dReal fUnitScale=1.0, py::object ooptions=py::object())
    {
        rapidjson::Document doc;
        KinBody::GeometryInfoPtr pgeominfo = GetGeometryInfo();
        pgeominfo->SerializeJSON(doc, doc.GetAllocator(), fUnitScale, pyGetIntFromPy(ooptions,0));
        return toPyObject(doc);
    }
#endif // OPENRAVE_RAPIDJSON

    KinBody::GeometryInfoPtr GetGeometryInfo() {
        KinBody::GeometryInfoPtr pinfo(new KinBody::GeometryInfo());
        KinBody::GeometryInfo& info = *pinfo;
        info._t = ExtractTransform(_t);
        info._vGeomData = ExtractVector<dReal>(_vGeomData);
        info._vGeomData2 = ExtractVector<dReal>(_vGeomData2);
        info._vGeomData3 = ExtractVector<dReal>(_vGeomData3);
        info._vGeomData4 = ExtractVector<dReal>(_vGeomData4);

        info._vSideWalls.clear();
        for (size_t i = 0; i < len(_vSideWalls); ++i) {
            info._vSideWalls.push_back({});
            OPENRAVE_SHARED_PTR<PySideWall> pysidewall = _vSideWalls[i].cast<OPENRAVE_SHARED_PTR<PySideWall> >();
            pysidewall->Get(info._vSideWalls[i]);
        }

        info._vDiffuseColor = ExtractVector34<dReal>(_vDiffuseColor,0);
        info._vAmbientColor = ExtractVector34<dReal>(_vAmbientColor,0);
        if( !IS_PYTHONOBJECT_NONE(_meshcollision) ) {
            ExtractTriMesh(_meshcollision,info._meshcollision);
        }
        info._type = _type;
        if( !IS_PYTHONOBJECT_NONE(_name) ) {
            info._name = _name.cast<std::string>();
        }
        if( !IS_PYTHONOBJECT_NONE(_filenamerender) ) {
            info._filenamerender = _filenamerender.cast<std::string>();
        }
        if( !IS_PYTHONOBJECT_NONE(_filenamecollision) ) {
            info._filenamecollision = _filenamecollision.cast<std::string>();
        }
        info._vRenderScale = ExtractVector3(_vRenderScale);
        info._vCollisionScale = ExtractVector3(_vCollisionScale);
        info._fTransparency = _fTransparency;
        info._bVisible = _bVisible;
        info._bModifiable = _bModifiable;
        //TODO
        //info._mapExtraGeometries =  _mapExtraGeometries;
        return pinfo;
    }

    py::object _t, _vGeomData, _vGeomData2, _vGeomData3, _vGeomData4, _vDiffuseColor, _vAmbientColor, _meshcollision;
    py::list _vSideWalls;
    float _containerBaseHeight;
    GeometryType _type;
    py::object _name;
    py::object _filenamerender, _filenamecollision;
    py::object _vRenderScale, _vCollisionScale;
    py::dict _mapExtraGeometries;
    float _fTransparency;
    bool _bVisible, _bModifiable;
};
typedef OPENRAVE_SHARED_PTR<PyGeometryInfo> PyGeometryInfoPtr;

} // namespace openravepy

#endif // OPENRAVEPY_INTERNAL_GEOMETRYINFO_H
