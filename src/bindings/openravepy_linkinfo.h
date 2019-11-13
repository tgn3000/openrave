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
#ifndef OPENRAVEPY_INTERNAL_LINKINFO_H
#define OPENRAVEPY_INTERNAL_LINKINFO_H

#define NO_IMPORT_ARRAY
#include "openravepy_int.h"
#include "openravepy_geometryinfo.h"

namespace openravepy
{

using py::object;

class PyLinkInfo
{
public:
    PyLinkInfo() {
        _t = ReturnTransform(Transform());
        _tMassFrame = ReturnTransform(Transform());
        _mass = 0;
        _vinertiamoments = toPyVector3(Vector(1,1,1));
        _bStatic = false;
        _bIsEnabled = true;
        _vForcedAdjacentLinks = py::list();
    }
    PyLinkInfo(const KinBody::LinkInfo& info) {
        FOREACHC(itgeominfo, info._vgeometryinfos) {
            _vgeometryinfos.append(PyGeometryInfoPtr(new PyGeometryInfo(**itgeominfo)));
        }
        _name = ConvertStringToUnicode(info._name);
        _t = ReturnTransform(info._t);
        _tMassFrame = ReturnTransform(info._tMassFrame);
        _mass = info._mass;
        _vinertiamoments = toPyVector3(info._vinertiamoments);
        FOREACHC(it, info._mapFloatParameters) {
            _mapFloatParameters[it->first] = toPyArray(it->second);
        }
        FOREACHC(it, info._mapIntParameters) {
            _mapIntParameters[it->first] = toPyArray(it->second);
        }
        FOREACHC(it, info._mapStringParameters) {
            _mapStringParameters[it->first] = ConvertStringToUnicode(it->second);
        }
        py::list vForcedAdjacentLinks;
        FOREACHC(it, info._vForcedAdjacentLinks) {
            vForcedAdjacentLinks.append(ConvertStringToUnicode(*it));
        }
        _vForcedAdjacentLinks = vForcedAdjacentLinks;
        _bStatic = info._bStatic;
        _bIsEnabled = info._bIsEnabled;
    }

    KinBody::LinkInfoPtr GetLinkInfo() {
        KinBody::LinkInfoPtr pinfo(new KinBody::LinkInfo());
        KinBody::LinkInfo& info = *pinfo;
        info._vgeometryinfos.resize(len(_vgeometryinfos));
        for(size_t i = 0; i < info._vgeometryinfos.size(); ++i) {
            PyGeometryInfoPtr pygeom = _vgeometryinfos[i].cast<PyGeometryInfoPtr>();
            info._vgeometryinfos[i] = pygeom->GetGeometryInfo();
        }
        if( !IS_PYTHONOBJECT_NONE(_name) ) {
            info._name = _name.cast<std::string>();
        }
        info._t = ExtractTransform(_t);
        info._tMassFrame = ExtractTransform(_tMassFrame);
        info._mass = _mass;
        info._vinertiamoments = ExtractVector3(_vinertiamoments);
        // size_t num = len(_mapFloatParameters);
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
        info._vForcedAdjacentLinks = ExtractArray<std::string>(_vForcedAdjacentLinks);
        info._bStatic = _bStatic;
        info._bIsEnabled = _bIsEnabled;
        return pinfo;
    }

    py::list _vgeometryinfos;
    py::object _name;
    py::object _t, _tMassFrame;
    dReal _mass;
    py::object _vinertiamoments;
    py::dict _mapFloatParameters, _mapIntParameters, _mapStringParameters;
    py::object _vForcedAdjacentLinks;
    bool _bStatic;
    bool _bIsEnabled;
};

} // namespace openravepy

#endif // OPENRAVEPY_INTERNAL_LINKINFO_H
