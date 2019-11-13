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
#ifndef OPENRAVEPY_INTERNAL_SIDEWALL_H
#define OPENRAVEPY_INTERNAL_SIDEWALL_H

#define NO_IMPORT_ARRAY
#include "openravepy_int.h"

namespace openravepy
{

using py::object;

class PySideWall
{
public:
    PySideWall() {
        transf = ReturnTransform(Transform());
        vExtents = toPyVector3(Vector());
        type = 0;
    }
    PySideWall(const KinBody::GeometryInfo::SideWall& sidewall) {
        transf = ReturnTransform(sidewall.transf);
        vExtents = toPyVector3(sidewall.vExtents);
        type = sidewall.type;
    }
    void Get(KinBody::GeometryInfo::SideWall& sidewall) {
        sidewall.transf = ExtractTransform(transf);
        sidewall.vExtents = ExtractVector<dReal>(vExtents);
        sidewall.type = static_cast<KinBody::GeometryInfo::SideWallType>(type);
    }

    py::object transf, vExtents;
    int type;
};

} // namespace openravepy

#endif // OPENRAVEPY_INTERNAL_SIDEWALL_H
