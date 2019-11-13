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
#ifndef OPENRAVEPY_INTERNAL_COLLISIONREPORT_H
#define OPENRAVEPY_INTERNAL_COLLISIONREPORT_H
#define NO_IMPORT_ARRAY
#include "openravepy_int.h"

namespace openravepy {

class PyCollisionReport
{
public:
    PyCollisionReport() : report(new CollisionReport()) {
    }
    PyCollisionReport(CollisionReportPtr report) : report(report) {
    }
    virtual ~PyCollisionReport() {
    }

    struct PYCONTACT
    {
        PYCONTACT() {
        }
        PYCONTACT(const CollisionReport::CONTACT& c)
        {
            pos = toPyVector3(c.pos);
            norm = toPyVector3(c.norm);
            depth = c.depth;
        }

        string __str__()
        {
            Vector vpos = ExtractVector3(pos), vnorm = ExtractVector3(norm);
            stringstream ss;
            ss << std::setprecision(std::numeric_limits<dReal>::digits10+1);     /// have to do this or otherwise precision gets lost
            ss << "pos=["<<vpos.x<<", "<<vpos.y<<", "<<vpos.z<<"], norm=["<<vnorm.x<<", "<<vnorm.y<<", "<<vnorm.z<<"]";
            return ss.str();
        }
        object __unicode__() {
            return ConvertStringToUnicode(__str__());
        }
        object pos, norm;
        dReal depth;
    };

    void init(PyEnvironmentBasePtr pyenv)
    {
        options = report->options;
        minDistance = report->minDistance;
        numWithinTol = report->numWithinTol;
        nKeepPrevious = report->nKeepPrevious;
        if( !!report->plink1 ) {
            plink1 = toPyKinBodyLink(OPENRAVE_CONST_POINTER_CAST<KinBody::Link>(report->plink1), pyenv);
        }
        else {
            plink1 = object();
        }
        if( !!report->plink2 ) {
            plink2 = toPyKinBodyLink(OPENRAVE_CONST_POINTER_CAST<KinBody::Link>(report->plink2), pyenv);
        }
        else {
            plink2 = object();
        }
        py::list newcontacts;
        FOREACH(itc, report->contacts) {
            newcontacts.append(PYCONTACT(*itc));
        }
        contacts = newcontacts;

        py::list newLinkColliding;
        FOREACHC(itlinks, report->vLinkColliding) {
            object pylink1, pylink2;
            if( !!itlinks->first ) {
                pylink1 = toPyKinBodyLink(OPENRAVE_CONST_POINTER_CAST<KinBody::Link>(itlinks->first), pyenv);
            }
            if( !!itlinks->second ) {
                pylink2 = toPyKinBodyLink(OPENRAVE_CONST_POINTER_CAST<KinBody::Link>(itlinks->second), pyenv);
            }
            newLinkColliding.append(py::make_tuple(pylink1, pylink2));
        }
        vLinkColliding = newLinkColliding;
    }

    string __str__()
    {
        return report->__str__();
    }
    object __unicode__() {
        return ConvertStringToUnicode(__str__());
    }

    int options;
    object plink1, plink2;

    py::list vLinkColliding;
    dReal minDistance;
    int numWithinTol;
    py::list contacts;
    uint32_t nKeepPrevious;
    CollisionReportPtr report;
};

} // namespace openravepy

#endif // OPENRAVEPY_INTERNAL_COLLISIONREPORT_H
