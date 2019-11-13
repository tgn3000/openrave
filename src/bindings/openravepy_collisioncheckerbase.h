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
#ifndef OPENRAVEPY_INTERNAL_COLLISIONCHECKERBASE_H
#define OPENRAVEPY_INTERNAL_COLLISIONCHECKERBASE_H

#define NO_IMPORT_ARRAY
#include "openravepy_int.h"

namespace openravepy {

class PyCollisionCheckerBase : public PyInterfaceBase
{
protected:
    CollisionCheckerBasePtr _pCollisionChecker;
public:
    PyCollisionCheckerBase(CollisionCheckerBasePtr pCollisionChecker, PyEnvironmentBasePtr pyenv) : PyInterfaceBase(pCollisionChecker, pyenv), _pCollisionChecker(pCollisionChecker) {
    }
    virtual ~PyCollisionCheckerBase() {
    }

    CollisionCheckerBasePtr GetCollisionChecker() {
        return _pCollisionChecker;
    }

    bool SetCollisionOptions(int options) {
        return _pCollisionChecker->SetCollisionOptions(options);
    }
    int GetCollisionOptions() const {
        return _pCollisionChecker->GetCollisionOptions();
    }

    bool InitEnvironment()
    {
        return _pCollisionChecker->InitEnvironment();
    }

    void DestroyEnvironment()
    {
        return _pCollisionChecker->DestroyEnvironment();
    }

    bool InitKinBody(PyKinBodyPtr pbody)
    {
        return _pCollisionChecker->InitKinBody(GetKinBody(pbody));
    }

    void SetGeometryGroup(const std::string& groupname)
    {
        _pCollisionChecker->SetGeometryGroup(groupname);
    }

    bool SetBodyGeometryGroup(PyKinBodyPtr pybody, const std::string& groupname)
    {
        return _pCollisionChecker->SetBodyGeometryGroup(GetKinBody(pybody), groupname);
    }

    object GetGeometryGroup()
    {
        return ConvertStringToUnicode(_pCollisionChecker->GetGeometryGroup());
    }

    void RemoveKinBody(PyKinBodyPtr pbody)
    {
        _pCollisionChecker->RemoveKinBody(GetKinBody(pbody));
    }

    bool CheckCollision(PyKinBodyPtr pbody1)
    {
        CHECK_POINTER(pbody1);
        return _pCollisionChecker->CheckCollision(KinBodyConstPtr(GetKinBody(pbody1)));
    }
    bool CheckCollision(PyKinBodyPtr pbody1, PyCollisionReportPtr pReport)
    {
        CHECK_POINTER(pbody1);
        bool bCollision = _pCollisionChecker->CheckCollision(KinBodyConstPtr(GetKinBody(pbody1)), GetCollisionReport(pReport));
        UpdateCollisionReport(pReport,_pyenv);
        return bCollision;
    }

    bool CheckCollision(PyKinBodyPtr pbody1, PyKinBodyPtr pbody2)
    {
        CHECK_POINTER(pbody1);
        CHECK_POINTER(pbody2);
        return _pCollisionChecker->CheckCollision(KinBodyConstPtr(GetKinBody(pbody1)), KinBodyConstPtr(GetKinBody(pbody2)));
    }

    bool CheckCollision(PyKinBodyPtr pbody1, PyKinBodyPtr pbody2, PyCollisionReportPtr pReport)
    {
        CHECK_POINTER(pbody1);
        CHECK_POINTER(pbody2);
        bool bCollision = _pCollisionChecker->CheckCollision(KinBodyConstPtr(GetKinBody(pbody1)), KinBodyConstPtr(GetKinBody(pbody2)), GetCollisionReport(pReport));
        UpdateCollisionReport(pReport,_pyenv);
        return bCollision;
    }

    bool CheckCollision(object o1)
    {
        CHECK_POINTER(o1);
        KinBody::LinkConstPtr plink = GetKinBodyLinkConst(o1);
        if( !!plink ) {
            return _pCollisionChecker->CheckCollision(plink);
        }
        KinBodyConstPtr pbody = GetKinBody(o1);
        if( !!pbody ) {
            return _pCollisionChecker->CheckCollision(pbody);
        }
        throw OPENRAVE_EXCEPTION_FORMAT0(_("CheckCollision(object) invalid argument"),ORE_InvalidArguments);
    }

    bool CheckCollision(object o1, PyCollisionReportPtr pReport)
    {
        CHECK_POINTER(o1);
        KinBody::LinkConstPtr plink = GetKinBodyLinkConst(o1);
        bool bCollision;
        if( !!plink ) {
            bCollision = _pCollisionChecker->CheckCollision(plink,GetCollisionReport(pReport));
        }
        else {
            KinBodyConstPtr pbody = GetKinBody(o1);
            if( !!pbody ) {
                bCollision = _pCollisionChecker->CheckCollision(pbody,GetCollisionReport(pReport));
            }
            else {
                throw OPENRAVE_EXCEPTION_FORMAT0(_("invalid argument"),ORE_InvalidArguments);
            }
        }
        UpdateCollisionReport(pReport,_pyenv);
        return bCollision;
    }

    bool CheckCollision(object o1, object o2)
    {
        CHECK_POINTER(o1);
        CHECK_POINTER(o2);
        KinBody::LinkConstPtr plink = GetKinBodyLinkConst(o1);
        if( !!plink ) {
            KinBody::LinkConstPtr plink2 = GetKinBodyLinkConst(o2);
            if( !!plink2 ) {
                return _pCollisionChecker->CheckCollision(plink,plink2);
            }
            KinBodyConstPtr pbody2 = GetKinBody(o2);
            if( !!pbody2 ) {
                return _pCollisionChecker->CheckCollision(plink,pbody2);
            }
            CollisionReportPtr preport2 = GetCollisionReport(o2);
            if( !!preport2 ) {
                bool bCollision = _pCollisionChecker->CheckCollision(plink,preport2);
                UpdateCollisionReport(o2,_pyenv);
                return bCollision;
            }
            throw OPENRAVE_EXCEPTION_FORMAT0(_("invalid argument 2"),ORE_InvalidArguments);
        }
        KinBodyConstPtr pbody = GetKinBody(o1);
        if( !!pbody ) {
            KinBody::LinkConstPtr plink2 = GetKinBodyLinkConst(o2);
            if( !!plink2 ) {
                return _pCollisionChecker->CheckCollision(plink2,pbody);
            }
            KinBodyConstPtr pbody2 = GetKinBody(o2);
            if( !!pbody2 ) {
                return _pCollisionChecker->CheckCollision(pbody,pbody2);
            }
            CollisionReportPtr preport2 = GetCollisionReport(o2);
            if( !!preport2 ) {
                bool bCollision = _pCollisionChecker->CheckCollision(pbody,preport2);
                UpdateCollisionReport(o2,_pyenv);
                return bCollision;
            }
            throw OPENRAVE_EXCEPTION_FORMAT0(_("invalid argument 2"),ORE_InvalidArguments);
        }
        throw OPENRAVE_EXCEPTION_FORMAT0(_("invalid argument 1"),ORE_InvalidArguments);
    }
    bool CheckCollision(object o1, object o2, PyCollisionReportPtr pReport)
    {
        CHECK_POINTER(o1);
        CHECK_POINTER(o2);
        bool bCollision = false;
        KinBody::LinkConstPtr plink = GetKinBodyLinkConst(o1);
        if( !!plink ) {
            KinBody::LinkConstPtr plink2 = GetKinBodyLinkConst(o2);
            if( !!plink2 ) {
                bCollision = _pCollisionChecker->CheckCollision(plink,plink2, GetCollisionReport(pReport));
            }
            else {
                KinBodyConstPtr pbody2 = GetKinBody(o2);
                if( !!pbody2 ) {
                    bCollision = _pCollisionChecker->CheckCollision(plink,pbody2, GetCollisionReport(pReport));
                }
                else {
                    throw OPENRAVE_EXCEPTION_FORMAT0(_("invalid argument 2"),ORE_InvalidArguments);
                }
            }
        }
        else {
            KinBodyConstPtr pbody = GetKinBody(o1);
            if( !!pbody ) {
                KinBody::LinkConstPtr plink2 = GetKinBodyLinkConst(o2);
                if( !!plink2 ) {
                    bCollision = _pCollisionChecker->CheckCollision(plink2,pbody, GetCollisionReport(pReport));
                }
                else {
                    KinBodyConstPtr pbody2 = GetKinBody(o2);
                    if( !!pbody2 ) {
                        bCollision = _pCollisionChecker->CheckCollision(pbody,pbody2, GetCollisionReport(pReport));
                    }
                    else {
                        throw OPENRAVE_EXCEPTION_FORMAT0(_("invalid argument 2"),ORE_InvalidArguments);
                    }
                }
            }
            else {
                throw OPENRAVE_EXCEPTION_FORMAT0(_("invalid argument 1"),ORE_InvalidArguments);
            }
        }
        UpdateCollisionReport(pReport,_pyenv);
        return bCollision;
    }

    bool CheckCollision(object o1, PyKinBodyPtr pybody2)
    {
        CHECK_POINTER(o1);
        CHECK_POINTER(pybody2);
        KinBodyConstPtr pbody2 = GetKinBody(pybody2);
        KinBody::LinkConstPtr plink = GetKinBodyLinkConst(o1);
        if( !!plink ) {
            return _pCollisionChecker->CheckCollision(plink,pbody2);
        }
        KinBodyConstPtr pbody1 = GetKinBody(o1);
        if( !!pbody1 ) {
            return _pCollisionChecker->CheckCollision(pbody1,pbody2);
        }
        throw OPENRAVE_EXCEPTION_FORMAT0(_("CheckCollision(object) invalid argument"),ORE_InvalidArguments);
    }

    bool CheckCollision(object o1, PyKinBodyPtr pybody2, PyCollisionReportPtr pReport)
    {
        CHECK_POINTER(o1);
        CHECK_POINTER(pybody2);
        KinBodyConstPtr pbody2 = GetKinBody(pybody2);
        KinBody::LinkConstPtr plink = GetKinBodyLinkConst(o1);
        bool bCollision = false;
        if( !!plink ) {
            bCollision = _pCollisionChecker->CheckCollision(plink,pbody2,GetCollisionReport(pReport));
        }
        else {
            KinBodyConstPtr pbody1 = GetKinBody(o1);
            if( !!pbody1 ) {
                bCollision = _pCollisionChecker->CheckCollision(pbody1,pbody2,GetCollisionReport(pReport));
            }
            else {
                throw OPENRAVE_EXCEPTION_FORMAT0(_("CheckCollision(object) invalid argument"),ORE_InvalidArguments);
            }
        }
        UpdateCollisionReport(pReport,_pyenv);
        return bCollision;
    }

    bool CheckCollision(object o1, object bodyexcluded, object linkexcluded)
    {
        CollisionReportPtr preport = GetCollisionReport(linkexcluded);
        if( !!preport ) {
            throw OPENRAVE_EXCEPTION_FORMAT0(_("3rd argument should be linkexcluded, rather than CollisionReport! Try report="),ORE_InvalidArguments);
        }

        KinBody::LinkConstPtr plink1 = GetKinBodyLinkConst(o1);
        KinBodyConstPtr pbody1 = GetKinBody(o1);

        std::vector<KinBodyConstPtr> vbodyexcluded;
        for(int i = 0; i < len(bodyexcluded); ++i) {
            PyKinBodyPtr pbody = bodyexcluded[i].cast<PyKinBodyPtr>();
            if( !!pbody ) {
                vbodyexcluded.push_back(GetKinBody(pbody));
            }
            else {
                RAVELOG_ERROR("failed to get excluded body\n");
            }
        }
        std::vector<KinBody::LinkConstPtr> vlinkexcluded;
        for(int i = 0; i < len(linkexcluded); ++i) {
            KinBody::LinkConstPtr plink2 = GetKinBodyLinkConst(linkexcluded[i]);
            if( !!plink2 ) {
                vlinkexcluded.push_back(plink2);
            }
            else {
                RAVELOG_ERROR("failed to get excluded link\n");
            }
        }
        if( !!plink1 ) {
            return _pCollisionChecker->CheckCollision(plink1,vbodyexcluded,vlinkexcluded);
        }
        else if( !!pbody1 ) {
            return _pCollisionChecker->CheckCollision(pbody1,vbodyexcluded,vlinkexcluded);
        }
        else {
            throw OPENRAVE_EXCEPTION_FORMAT0(_("invalid argument 1"),ORE_InvalidArguments);
        }
    }

    bool CheckCollision(object o1, object bodyexcluded, object linkexcluded, PyCollisionReportPtr pReport)
    {
        std::vector<KinBodyConstPtr> vbodyexcluded;
        KinBody::LinkConstPtr plink1 = GetKinBodyLinkConst(o1);
        KinBodyConstPtr pbody1 = GetKinBody(o1);

        for(int i = 0; i < len(bodyexcluded); ++i) {
            PyKinBodyPtr pbody = bodyexcluded[i].cast<PyKinBodyPtr>();
            if( !!pbody ) {
                vbodyexcluded.push_back(GetKinBody(pbody));
            }
            else {
                RAVELOG_ERROR("failed to get excluded body\n");
            }
        }
        std::vector<KinBody::LinkConstPtr> vlinkexcluded;
        for(int i = 0; i < len(linkexcluded); ++i) {
            KinBody::LinkConstPtr plink2 = GetKinBodyLinkConst(linkexcluded[i]);
            if( !!plink2 ) {
                vlinkexcluded.push_back(plink2);
            }
            else {
                RAVELOG_ERROR("failed to get excluded link\n");
            }
        }

        bool bCollision=false;
        if( !!plink1 ) {
            bCollision = _pCollisionChecker->CheckCollision(plink1, vbodyexcluded, vlinkexcluded, GetCollisionReport(pReport));
        }
        else if( !!pbody1 ) {
            bCollision = _pCollisionChecker->CheckCollision(pbody1, vbodyexcluded, vlinkexcluded, GetCollisionReport(pReport));
        }
        else {
            throw OPENRAVE_EXCEPTION_FORMAT0(_("invalid argument 1"),ORE_InvalidArguments);
        }

        UpdateCollisionReport(pReport,_pyenv);
        return bCollision;
    }

    bool CheckCollision(PyKinBodyPtr pbody, object bodyexcluded, object linkexcluded)
    {
        std::vector<KinBodyConstPtr> vbodyexcluded;
        for(int i = 0; i < len(bodyexcluded); ++i) {
            PyKinBodyPtr pbody = bodyexcluded[i].cast<PyKinBodyPtr>();
            if( !!pbody ) {
                vbodyexcluded.push_back(GetKinBody(pbody));
            }
            else {
                RAVELOG_ERROR("failed to get excluded body\n");
            }
        }
        std::vector<KinBody::LinkConstPtr> vlinkexcluded;
        for(int i = 0; i < len(linkexcluded); ++i) {
            KinBody::LinkConstPtr plink2 = GetKinBodyLinkConst(linkexcluded[i]);
            if( !!plink2 ) {
                vlinkexcluded.push_back(plink2);
            }
            else {
                RAVELOG_ERROR("failed to get excluded link\n");
            }
        }
        return _pCollisionChecker->CheckCollision(KinBodyConstPtr(GetKinBody(pbody)),vbodyexcluded,vlinkexcluded);
    }

    bool CheckCollision(PyKinBodyPtr pbody, object bodyexcluded, object linkexcluded, PyCollisionReportPtr pReport)
    {
        std::vector<KinBodyConstPtr> vbodyexcluded;
        for(int i = 0; i < len(bodyexcluded); ++i) {
            PyKinBodyPtr pbody = bodyexcluded[i].cast<PyKinBodyPtr>();
            if( !!pbody ) {
                vbodyexcluded.push_back(GetKinBody(pbody));
            }
            else {
                RAVELOG_ERROR("failed to get excluded body\n");
            }
        }
        std::vector<KinBody::LinkConstPtr> vlinkexcluded;
        for(int i = 0; i < len(linkexcluded); ++i) {
            KinBody::LinkConstPtr plink2 = GetKinBodyLinkConst(linkexcluded[i]);
            if( !!plink2 ) {
                vlinkexcluded.push_back(plink2);
            }
            else {
                RAVELOG_ERROR("failed to get excluded link\n");
            }
        }

        bool bCollision = _pCollisionChecker->CheckCollision(KinBodyConstPtr(GetKinBody(pbody)), vbodyexcluded, vlinkexcluded, GetCollisionReport(pReport));
        UpdateCollisionReport(pReport,_pyenv);
        return bCollision;
    }

    bool CheckCollision(OPENRAVE_SHARED_PTR<PyRay> pyray, PyKinBodyPtr pbody)
    {
        return _pCollisionChecker->CheckCollision(pyray->r,KinBodyConstPtr(GetKinBody(pbody)));
    }

    bool CheckCollision(OPENRAVE_SHARED_PTR<PyRay> pyray, PyKinBodyPtr pbody, PyCollisionReportPtr pReport)
    {
        bool bCollision = _pCollisionChecker->CheckCollision(pyray->r, KinBodyConstPtr(GetKinBody(pbody)), GetCollisionReport(pReport));
        UpdateCollisionReport(pReport,_pyenv);
        return bCollision;
    }

    object CheckCollisionRays(object rays, PyKinBodyPtr pbody,bool bFrontFacingOnly=false)
    {
        object shape = rays.attr("shape");
        int num = shape[0].cast<int>();
        if( num == 0 ) {
            return py::make_tuple(py::array_t<dReal>({1, 0}, nullptr), py::array_t<dReal>({1, 0}, nullptr));
        }
        if( shape[1].cast<int>() != 6 ) {
            throw openrave_exception(_("rays object needs to be a Nx6 vector\n"));
        }
        CollisionReport report;
        CollisionReportPtr preport(&report,null_deleter());

        RAY r;
        npy_intp dims[] = { num,6};
        PyObject *pypos = PyArray_SimpleNew(2,dims, sizeof(dReal)==8 ? PyArray_DOUBLE : PyArray_FLOAT);
        dReal* ppos = (dReal*)PyArray_DATA(pypos);
        PyObject* pycollision = PyArray_SimpleNew(1,&dims[0], PyArray_BOOL);
        bool* pcollision = (bool*)PyArray_DATA(pycollision);
        for(int i = 0; i < num; ++i, ppos += 6) {
            vector<dReal> ray = ExtractArray<dReal>(rays[i]);
            r.pos.x = ray[0];
            r.pos.y = ray[1];
            r.pos.z = ray[2];
            r.dir.x = ray[3];
            r.dir.y = ray[4];
            r.dir.z = ray[5];
            bool bCollision;
            if( !pbody ) {
                bCollision = _pCollisionChecker->CheckCollision(r, preport);
            }
            else {
                bCollision = _pCollisionChecker->CheckCollision(r, KinBodyConstPtr(GetKinBody(pbody)), preport);
            }
            pcollision[i] = false;
            ppos[0] = 0; ppos[1] = 0; ppos[2] = 0; ppos[3] = 0; ppos[4] = 0; ppos[5] = 0;
            if( bCollision &&( report.contacts.size() > 0) ) {
                if( !bFrontFacingOnly ||( report.contacts[0].norm.dot3(r.dir)<0) ) {
                    pcollision[i] = true;
                    ppos[0] = report.contacts[0].pos.x;
                    ppos[1] = report.contacts[0].pos.y;
                    ppos[2] = report.contacts[0].pos.z;
                    ppos[3] = report.contacts[0].norm.x;
                    ppos[4] = report.contacts[0].norm.y;
                    ppos[5] = report.contacts[0].norm.z;
                }
            }
        }
        return py::make_tuple(
            toPyArrayN(pcollision, num),
            toPyArrayN(ppos, 6 * num)
        );
    }

    bool CheckCollision(OPENRAVE_SHARED_PTR<PyRay> pyray)
    {
        return _pCollisionChecker->CheckCollision(pyray->r);
    }

    bool CheckCollision(OPENRAVE_SHARED_PTR<PyRay> pyray, PyCollisionReportPtr pReport)
    {
        bool bCollision = _pCollisionChecker->CheckCollision(pyray->r, GetCollisionReport(pReport));
        UpdateCollisionReport(pReport,_pyenv);
        return bCollision;
    }

    bool CheckCollisionTriMesh(object otrimesh, PyKinBodyPtr pybody, PyCollisionReportPtr pReport)
    {
        TriMesh trimesh;
        if( !ExtractTriMesh(otrimesh,trimesh) ) {
            throw openrave_exception(_("bad trimesh"));
        }
        KinBodyConstPtr pbody(GetKinBody(pybody));
        bool bCollision = _pCollisionChecker->CheckCollision(trimesh, pbody, GetCollisionReport(pReport));
        UpdateCollisionReport(pReport,_pyenv);
        return bCollision;
    }

    bool CheckCollisionTriMesh(object otrimesh, PyCollisionReportPtr pReport)
    {
        TriMesh trimesh;
        if( !ExtractTriMesh(otrimesh,trimesh) ) {
            throw openrave_exception(_("bad trimesh"));
        }
        bool bCollision = _pCollisionChecker->CheckCollision(trimesh, GetCollisionReport(pReport));
        UpdateCollisionReport(pReport,_pyenv);
        return bCollision;
    }

    bool CheckCollisionOBB(object oaabb, object otransform, PyCollisionReportPtr pReport)
    {
        AABB aabb = ExtractAABB(oaabb);
        Transform t = ExtractTransform(otransform);
        bool bCollision = _pCollisionChecker->CheckCollision(aabb, t, GetCollisionReport(pReport));
        UpdateCollisionReport(pReport,_pyenv);
        return bCollision;
    }

    virtual bool CheckSelfCollision(object o1, PyCollisionReportPtr pReport)
    {
        KinBody::LinkConstPtr plink1 = GetKinBodyLinkConst(o1);
        KinBodyConstPtr pbody1 = GetKinBody(o1);
        bool bCollision;
        if( !!plink1 ) {
            bCollision = _pCollisionChecker->CheckSelfCollision(plink1, GetCollisionReport(pReport));
        }
        else if( !!pbody1 ) {
            bCollision = _pCollisionChecker->CheckSelfCollision(pbody1, GetCollisionReport(pReport));
        }
        else {
            throw OPENRAVE_EXCEPTION_FORMAT0(_("invalid parameters to CheckSelfCollision"), ORE_InvalidArguments);
        }
        UpdateCollisionReport(pReport,_pyenv);
        return bCollision;
    }
};

} // namespace openravepy

#endif // OPENRAVEPY_INTERNAL_COLLISIONCHECKERBASE_H
