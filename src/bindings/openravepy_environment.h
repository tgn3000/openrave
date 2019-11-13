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
#ifndef OPENRAVEPY_INTERNAL_ENVIRONMENT_H
#define OPENRAVEPY_INTERNAL_ENVIRONMENT_H

#include <openrave/utils.h>
#include <boost/thread/once.hpp>
#include <boost/scoped_ptr.hpp>
#include <boost/multi_array.hpp>

#define NO_IMPORT_ARRAY
#include "openravepy_int.h"
#include "openravepy_kinbody.h"
#include "openravepy_viewermanager.h"
#include "openravepy_collisioncheckerbase.h"

namespace openravepy
{

using py::object;

class PyEnvironmentBase : public OPENRAVE_ENABLE_SHARED_FROM_THIS<PyEnvironmentBase>
{
#if BOOST_VERSION < 103500
    boost::mutex _envmutex;
    std::list<OPENRAVE_SHARED_PTR<EnvironmentMutex::scoped_lock> > _listenvlocks, _listfreelocks;
#endif
protected:
    EnvironmentBasePtr _penv;

    PyInterfaceBasePtr _toPyInterface(InterfaceBasePtr pinterface)
    {
        if( !pinterface ) {
            return PyInterfaceBasePtr();
        }
        switch(pinterface->GetInterfaceType()) {
        case PT_Planner: return openravepy::toPyPlanner(OPENRAVE_STATIC_POINTER_CAST<PlannerBase>(pinterface),shared_from_this());
        case PT_Robot: return openravepy::toPyRobot(OPENRAVE_STATIC_POINTER_CAST<RobotBase>(pinterface),shared_from_this());
        case PT_SensorSystem: return openravepy::toPySensorSystem(OPENRAVE_STATIC_POINTER_CAST<SensorSystemBase>(pinterface),shared_from_this());
        case PT_Controller: return openravepy::toPyController(OPENRAVE_STATIC_POINTER_CAST<ControllerBase>(pinterface),shared_from_this());
        case PT_Module: return openravepy::toPyModule(OPENRAVE_STATIC_POINTER_CAST<ModuleBase>(pinterface),shared_from_this());
        case PT_IkSolver: return openravepy::toPyIkSolver(OPENRAVE_STATIC_POINTER_CAST<IkSolverBase>(pinterface),shared_from_this());
        case PT_KinBody: return openravepy::toPyKinBody(OPENRAVE_STATIC_POINTER_CAST<KinBody>(pinterface),shared_from_this());
        case PT_PhysicsEngine: return openravepy::toPyPhysicsEngine(OPENRAVE_STATIC_POINTER_CAST<PhysicsEngineBase>(pinterface),shared_from_this());
        case PT_Sensor: return openravepy::toPySensor(OPENRAVE_STATIC_POINTER_CAST<SensorBase>(pinterface),shared_from_this());
        case PT_CollisionChecker: return openravepy::toPyCollisionChecker(OPENRAVE_STATIC_POINTER_CAST<CollisionCheckerBase>(pinterface),shared_from_this());
        case PT_Trajectory: return openravepy::toPyTrajectory(OPENRAVE_STATIC_POINTER_CAST<TrajectoryBase>(pinterface),shared_from_this());
        case PT_Viewer: return openravepy::toPyViewer(OPENRAVE_STATIC_POINTER_CAST<ViewerBase>(pinterface),shared_from_this());
        case PT_SpaceSampler: return openravepy::toPySpaceSampler(OPENRAVE_STATIC_POINTER_CAST<SpaceSamplerBase>(pinterface),shared_from_this());
        }
        return PyInterfaceBasePtr();
    }

    void _BodyCallback(object fncallback, KinBodyPtr pbody, int action)
    {
        object res;
        PyGILState_STATE gstate = PyGILState_Ensure();
        try {
            fncallback(openravepy::toPyKinBody(pbody, shared_from_this()), action);
        }
        catch(...) {
            RAVELOG_ERROR("exception occured in python body callback:\n");
            PyErr_Print();
        }
        PyGILState_Release(gstate);
    }

    CollisionAction _CollisionCallback(object fncallback, CollisionReportPtr preport, bool bFromPhysics)
    {
        object res;
        PyGILState_STATE gstate = PyGILState_Ensure();
        try {
            res = fncallback(openravepy::toPyCollisionReport(preport,shared_from_this()),bFromPhysics);
        }
        catch(...) {
            RAVELOG_ERROR("exception occured in python collision callback:\n");
            PyErr_Print();
        }
        CollisionAction ret = CA_DefaultAction;
        if( IS_PYTHONOBJECT_NONE(res) || !res ) {
            ret = CA_DefaultAction;
            RAVELOG_WARN("collision callback nothing returning, so executing default action\n");
        }
        else {
            try {
                ret = (CollisionAction) res.cast<int>();
            }
            catch(...) {
                RAVELOG_WARN("collision callback nothing returning, so executing default action\n");
            }
        }
        PyGILState_Release(gstate);
        return ret;
    }

public:
    PyEnvironmentBase(int options=ECO_StartSimulationThread)
    {
        if( !RaveGlobalState() ) {
            RaveInitialize(true);
        }
        _penv = RaveCreateEnvironment(options);
    }
    PyEnvironmentBase(EnvironmentBasePtr penv) : _penv(penv) {
    }

    PyEnvironmentBase(const PyEnvironmentBase &pyenv)
    {
        _penv = pyenv._penv;
    }

    virtual ~PyEnvironmentBase()
    {
    }

    void Reset() {
        _penv->Reset();
    }
    void Destroy() {
        ViewerManager::GetInstance().RemoveViewersOfEnvironment(_penv);
        _penv->Destroy();
    }

    PyEnvironmentBasePtr CloneSelf(int options)
    {
//        string strviewer;
//        if( options & Clone_Viewer ) {
//            boost::mutex::scoped_lock lockcreate(_mutexViewer);
//            if( !!_penv->GetViewer() ) {
//                strviewer = _penv->GetViewer()->GetXMLId();
//            }
//        }
        PyEnvironmentBasePtr pnewenv(new PyEnvironmentBase(_penv->CloneSelf(options)));
//        if( strviewer.size() > 0 ) {
//            pnewenv->SetViewer(strviewer);
//        }
        return pnewenv;
    }

    void Clone(PyEnvironmentBasePtr pyreference, int options)
    {
        if( options & Clone_Viewer ) {
            if( !!_penv->GetViewer() && !!pyreference->GetEnv()->GetViewer() ) {
                if( _penv->GetViewer()->GetXMLId() != pyreference->GetEnv()->GetViewer()->GetXMLId() ) {
                    RAVELOG_VERBOSE("reset the viewer since it has to be cloned\n");
                    //boost::mutex::scoped_lock lockcreate(pyreference->_mutexViewer);
                    SetViewer("");
                }
            }
        }
        _penv->Clone(pyreference->GetEnv(),options);
    }

    bool SetCollisionChecker(PyCollisionCheckerBasePtr pchecker)
    {
        return _penv->SetCollisionChecker(openravepy::GetCollisionChecker(pchecker));
    }
    object GetCollisionChecker()
    {
        return py::cast(openravepy::toPyCollisionChecker(_penv->GetCollisionChecker(), shared_from_this()));
    }
    bool CheckCollision(PyKinBodyPtr pbody1)
    {
        CHECK_POINTER(pbody1);
        return _penv->CheckCollision(KinBodyConstPtr(openravepy::GetKinBody(pbody1)));
    }
    bool CheckCollision(PyKinBodyPtr pbody1, PyCollisionReportPtr pReport)
    {
        CHECK_POINTER(pbody1);
        bool bCollision = _penv->CheckCollision(KinBodyConstPtr(openravepy::GetKinBody(pbody1)), openravepy::GetCollisionReport(pReport));
        openravepy::UpdateCollisionReport(pReport,shared_from_this());
        return bCollision;
    }

    bool CheckCollision(PyKinBodyPtr pbody1, PyKinBodyPtr pbody2)
    {
        CHECK_POINTER(pbody1);
        CHECK_POINTER(pbody2);
        return _penv->CheckCollision(KinBodyConstPtr(openravepy::GetKinBody(pbody1)), KinBodyConstPtr(openravepy::GetKinBody(pbody2)));
    }

    bool CheckCollision(PyKinBodyPtr pbody1, PyKinBodyPtr pbody2, PyCollisionReportPtr pReport)
    {
        CHECK_POINTER(pbody1);
        CHECK_POINTER(pbody2);
        bool bCollision = _penv->CheckCollision(KinBodyConstPtr(openravepy::GetKinBody(pbody1)), KinBodyConstPtr(openravepy::GetKinBody(pbody2)), openravepy::GetCollisionReport(pReport));
        openravepy::UpdateCollisionReport(pReport,shared_from_this());
        return bCollision;
    }

    bool CheckCollision(object o1)
    {
        CHECK_POINTER(o1);
        KinBody::LinkConstPtr plink = openravepy::GetKinBodyLinkConst(o1);
        if( !!plink ) {
            return _penv->CheckCollision(plink);
        }
        KinBodyConstPtr pbody = openravepy::GetKinBody(o1);
        if( !!pbody ) {
            return _penv->CheckCollision(pbody);
        }
        throw OPENRAVE_EXCEPTION_FORMAT0(_("CheckCollision(object) invalid argument"),ORE_InvalidArguments);
    }

    bool CheckCollision(object o1, PyCollisionReportPtr pReport)
    {
        CHECK_POINTER(o1);
        KinBody::LinkConstPtr plink = openravepy::GetKinBodyLinkConst(o1);
        bool bCollision;
        if( !!plink ) {
            bCollision = _penv->CheckCollision(plink,openravepy::GetCollisionReport(pReport));
        }
        else {
            KinBodyConstPtr pbody = openravepy::GetKinBody(o1);
            if( !!pbody ) {
                bCollision = _penv->CheckCollision(pbody,openravepy::GetCollisionReport(pReport));
            }
            else {
                throw OPENRAVE_EXCEPTION_FORMAT0(_("invalid argument"),ORE_InvalidArguments);
            }
        }
        openravepy::UpdateCollisionReport(pReport,shared_from_this());
        return bCollision;
    }

    bool CheckCollision(object o1, object o2)
    {
        CHECK_POINTER(o1);
        CHECK_POINTER(o2);
        KinBody::LinkConstPtr plink = openravepy::GetKinBodyLinkConst(o1);
        if( !!plink ) {
            KinBody::LinkConstPtr plink2 = openravepy::GetKinBodyLinkConst(o2);
            if( !!plink2 ) {
                return _penv->CheckCollision(plink,plink2);
            }
            KinBodyConstPtr pbody2 = openravepy::GetKinBody(o2);
            if( !!pbody2 ) {
                return _penv->CheckCollision(plink,pbody2);
            }
            CollisionReportPtr preport2 = openravepy::GetCollisionReport(o2);
            if( !!preport2 ) {
                bool bCollision = _penv->CheckCollision(plink,preport2);
                openravepy::UpdateCollisionReport(o2,shared_from_this());
                return bCollision;
            }
            throw OPENRAVE_EXCEPTION_FORMAT0(_("invalid argument 2"),ORE_InvalidArguments);
        }
        KinBodyConstPtr pbody = openravepy::GetKinBody(o1);
        if( !!pbody ) {
            KinBody::LinkConstPtr plink2 = openravepy::GetKinBodyLinkConst(o2);
            if( !!plink2 ) {
                return _penv->CheckCollision(plink2,pbody);
            }
            KinBodyConstPtr pbody2 = openravepy::GetKinBody(o2);
            if( !!pbody2 ) {
                return _penv->CheckCollision(pbody,pbody2);
            }
            CollisionReportPtr preport2 = openravepy::GetCollisionReport(o2);
            if( !!preport2 ) {
                bool bCollision = _penv->CheckCollision(pbody,preport2);
                openravepy::UpdateCollisionReport(o2,shared_from_this());
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
        KinBody::LinkConstPtr plink = openravepy::GetKinBodyLinkConst(o1);
        if( !!plink ) {
            KinBody::LinkConstPtr plink2 = openravepy::GetKinBodyLinkConst(o2);
            if( !!plink2 ) {
                bCollision = _penv->CheckCollision(plink,plink2, openravepy::GetCollisionReport(pReport));
            }
            else {
                KinBodyConstPtr pbody2 = openravepy::GetKinBody(o2);
                if( !!pbody2 ) {
                    bCollision = _penv->CheckCollision(plink,pbody2, openravepy::GetCollisionReport(pReport));
                }
                else {
                    throw OPENRAVE_EXCEPTION_FORMAT0(_("invalid argument 2"),ORE_InvalidArguments);
                }
            }
        }
        {
            KinBodyConstPtr pbody = openravepy::GetKinBody(o1);
            if( !!pbody ) {
                KinBody::LinkConstPtr plink2 = openravepy::GetKinBodyLinkConst(o2);
                if( !!plink2 ) {
                    bCollision = _penv->CheckCollision(plink2,pbody, openravepy::GetCollisionReport(pReport));
                }
                else {
                    KinBodyConstPtr pbody2 = openravepy::GetKinBody(o2);
                    if( !!pbody2 ) {
                        bCollision = _penv->CheckCollision(pbody,pbody2, openravepy::GetCollisionReport(pReport));
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
        openravepy::UpdateCollisionReport(pReport,shared_from_this());
        return bCollision;
    }

    bool CheckCollision(object o1, PyKinBodyPtr pybody2)
    {
        CHECK_POINTER(o1);
        CHECK_POINTER(pybody2);
        KinBodyConstPtr pbody2 = openravepy::GetKinBody(pybody2);
        KinBody::LinkConstPtr plink = openravepy::GetKinBodyLinkConst(o1);
        if( !!plink ) {
            return _penv->CheckCollision(plink,pbody2);
        }
        KinBodyConstPtr pbody1 = openravepy::GetKinBody(o1);
        if( !!pbody1 ) {
            return _penv->CheckCollision(pbody1,pbody2);
        }
        throw OPENRAVE_EXCEPTION_FORMAT0(_("CheckCollision(object) invalid argument"),ORE_InvalidArguments);
    }

    bool CheckCollision(object o1, PyKinBodyPtr pybody2, PyCollisionReportPtr pReport)
    {
        CHECK_POINTER(o1);
        CHECK_POINTER(pybody2);
        KinBodyConstPtr pbody2 = openravepy::GetKinBody(pybody2);
        KinBody::LinkConstPtr plink = openravepy::GetKinBodyLinkConst(o1);
        bool bCollision = false;
        if( !!plink ) {
            bCollision = _penv->CheckCollision(plink,pbody2,openravepy::GetCollisionReport(pReport));
        }
        else {
            KinBodyConstPtr pbody1 = openravepy::GetKinBody(o1);
            if( !!pbody1 ) {
                bCollision = _penv->CheckCollision(pbody1,pbody2,openravepy::GetCollisionReport(pReport));
            }
            else {
                throw OPENRAVE_EXCEPTION_FORMAT0(_("CheckCollision(object) invalid argument"),ORE_InvalidArguments);
            }
        }
        openravepy::UpdateCollisionReport(pReport,shared_from_this());
        return bCollision;
    }

    bool CheckCollision(object o1, object bodyexcluded, object linkexcluded)
    {
        CollisionReportPtr preport = openravepy::GetCollisionReport(linkexcluded);
        if( !!preport ) {
            throw OPENRAVE_EXCEPTION_FORMAT0(_("3rd argument should be linkexcluded, rather than CollisionReport! Try report="),ORE_InvalidArguments);
        }

        KinBody::LinkConstPtr plink1 = openravepy::GetKinBodyLinkConst(o1);
        KinBodyConstPtr pbody1 = openravepy::GetKinBody(o1);

        std::vector<KinBodyConstPtr> vbodyexcluded;
        for(int i = 0; i < len(bodyexcluded); ++i) {
            try {
                PyKinBodyPtr pbody = bodyexcluded[i].cast<PyKinBodyPtr>();
                if( pbody != nullptr ) {
                    vbodyexcluded.push_back(openravepy::GetKinBody(pbody));
                }               
            }
            catch(...) {
                RAVELOG_ERROR("failed to get excluded body\n");
            }
        }
        std::vector<KinBody::LinkConstPtr> vlinkexcluded;
        for(int i = 0; i < len(linkexcluded); ++i) {
            KinBody::LinkConstPtr plink2 = openravepy::GetKinBodyLinkConst(linkexcluded[i]);
            if( !!plink2 ) {
                vlinkexcluded.push_back(plink2);
            }
            else {
                RAVELOG_ERROR("failed to get excluded link\n");
            }
        }
        if( !!plink1 ) {
            return _penv->CheckCollision(plink1,vbodyexcluded,vlinkexcluded);
        }
        else if( !!pbody1 ) {
            return _penv->CheckCollision(pbody1,vbodyexcluded,vlinkexcluded);
        }
        else {
            throw OPENRAVE_EXCEPTION_FORMAT0(_("invalid argument 1"),ORE_InvalidArguments);
        }
    }

    bool CheckCollision(object o1, object bodyexcluded, object linkexcluded, PyCollisionReportPtr pReport)
    {
        std::vector<KinBodyConstPtr> vbodyexcluded;
        KinBody::LinkConstPtr plink1 = openravepy::GetKinBodyLinkConst(o1);
        KinBodyConstPtr pbody1 = openravepy::GetKinBody(o1);

        for(int i = 0; i < len(bodyexcluded); ++i) {
            try {
                PyKinBodyPtr pbody = bodyexcluded[i].cast<PyKinBodyPtr>();
                if( !!pbody ) {
                    vbodyexcluded.push_back(openravepy::GetKinBody(pbody));
                }                
            }
            catch(...) {
                RAVELOG_ERROR("failed to get excluded body\n");
            }
        }
        std::vector<KinBody::LinkConstPtr> vlinkexcluded;
        for(int i = 0; i < len(linkexcluded); ++i) {
            KinBody::LinkConstPtr plink2 = openravepy::GetKinBodyLinkConst(linkexcluded[i]);
            if( !!plink2 ) {
                vlinkexcluded.push_back(plink2);
            }
            else {
                RAVELOG_ERROR("failed to get excluded link\n");
            }
        }

        bool bCollision=false;
        if( !!plink1 ) {
            bCollision = _penv->CheckCollision(plink1, vbodyexcluded, vlinkexcluded, openravepy::GetCollisionReport(pReport));
        }
        else if( !!pbody1 ) {
            bCollision = _penv->CheckCollision(pbody1, vbodyexcluded, vlinkexcluded, openravepy::GetCollisionReport(pReport));
        }
        else {
            throw OPENRAVE_EXCEPTION_FORMAT0(_("invalid argument 1"),ORE_InvalidArguments);
        }

        openravepy::UpdateCollisionReport(pReport,shared_from_this());
        return bCollision;
    }

    bool CheckCollision(PyKinBodyPtr pbody, object bodyexcluded, object linkexcluded)
    {
        std::vector<KinBodyConstPtr> vbodyexcluded;
        for(int i = 0; i < len(bodyexcluded); ++i) {
            try {
                PyKinBodyPtr pbody = bodyexcluded[i].cast<PyKinBodyPtr>();
                if( pbody != nullptr ) {
                    vbodyexcluded.push_back(openravepy::GetKinBody(pbody));
                }                
            }
            catch(...) {
                RAVELOG_ERROR("failed to get excluded body\n");
            }
        }
        std::vector<KinBody::LinkConstPtr> vlinkexcluded;
        for(int i = 0; i < len(linkexcluded); ++i) {
            KinBody::LinkConstPtr plink2 = openravepy::GetKinBodyLinkConst(linkexcluded[i]);
            if( !!plink2 ) {
                vlinkexcluded.push_back(plink2);
            }
            else {
                RAVELOG_ERROR("failed to get excluded link\n");
            }
        }
        return _penv->CheckCollision(KinBodyConstPtr(openravepy::GetKinBody(pbody)),vbodyexcluded,vlinkexcluded);
    }

    bool CheckCollision(PyKinBodyPtr pbody, object bodyexcluded, object linkexcluded, PyCollisionReportPtr pReport)
    {
        std::vector<KinBodyConstPtr> vbodyexcluded;
        for(int i = 0; i < len(bodyexcluded); ++i) {
            try {
                PyKinBodyPtr pbody = bodyexcluded[i].cast<PyKinBodyPtr>();
                if( pbody != nullptr ) {
                    vbodyexcluded.push_back(openravepy::GetKinBody(pbody));
                }
            }
            catch(...) {
                RAVELOG_ERROR("failed to get excluded body\n");
            }
        }
        std::vector<KinBody::LinkConstPtr> vlinkexcluded;
        for(int i = 0; i < len(linkexcluded); ++i) {
            KinBody::LinkConstPtr plink2 = openravepy::GetKinBodyLinkConst(linkexcluded[i]);
            if( !!plink2 ) {
                vlinkexcluded.push_back(plink2);
            }
            else {
                RAVELOG_ERROR("failed to get excluded link\n");
            }
        }

        bool bCollision = _penv->CheckCollision(KinBodyConstPtr(openravepy::GetKinBody(pbody)), vbodyexcluded, vlinkexcluded, openravepy::GetCollisionReport(pReport));
        openravepy::UpdateCollisionReport(pReport,shared_from_this());
        return bCollision;
    }

    bool CheckCollision(OPENRAVE_SHARED_PTR<PyRay> pyray, PyKinBodyPtr pbody)
    {
        return _penv->CheckCollision(pyray->r,KinBodyConstPtr(openravepy::GetKinBody(pbody)));
    }

    bool CheckCollision(OPENRAVE_SHARED_PTR<PyRay> pyray, PyKinBodyPtr pbody, PyCollisionReportPtr pReport)
    {
        bool bCollision = _penv->CheckCollision(pyray->r, KinBodyConstPtr(openravepy::GetKinBody(pbody)), openravepy::GetCollisionReport(pReport));
        openravepy::UpdateCollisionReport(pReport,shared_from_this());
        return bCollision;
    }

    object CheckCollisionRays(py::array_t<dReal> rays, PyKinBodyPtr pbody, bool bFrontFacingOnly = false)
    {
        object shape = rays.attr("shape");
        int nRays = shape[0].cast<int>();
        if( nRays == 0 ) {
            return py::make_tuple(
                py::array_t<dReal>({1, 0}, nullptr),
                py::array_t<dReal>({1, 0}, nullptr)
            );
        }
        if( shape[1].cast<int>() != 6 ) {
            throw openrave_exception(_("rays object needs to be a Nx6 vector\n"));
        }
        CollisionReport report;
        CollisionReportPtr preport(&report,null_deleter());

        PyArrayObject *pPyRays = PyArray_GETCONTIGUOUS(reinterpret_cast<PyArrayObject*>(rays.ptr()));
        AutoPyArrayObjectDereferencer pyderef(pPyRays);

        if( !PyArray_ISFLOAT(pPyRays) ) {
            throw openrave_exception(_("rays has to be a float array\n"));
        }

        bool isFloat = PyArray_ITEMSIZE(pPyRays) == sizeof(float); // or double
        const float *pRaysFloat = isFloat ? reinterpret_cast<const float*>(PyArray_DATA(pPyRays)) : NULL;
        const double *pRaysDouble = isFloat ? NULL : reinterpret_cast<const double*>(PyArray_DATA(pPyRays));

        RAY r;
        npy_intp dims[] = { nRays,6};
        PyObject *pypos = PyArray_SimpleNew(2,dims, sizeof(dReal) == sizeof(double) ? PyArray_DOUBLE : PyArray_FLOAT);
        dReal* ppos = (dReal*)PyArray_DATA(pypos);
        std::memset(ppos, 0, nRays * sizeof(dReal));
        PyObject* pycollision = PyArray_SimpleNew(1,&dims[0], PyArray_BOOL);
        // numpy bool = uint8_t
        uint8_t* pcollision = (uint8_t*)PyArray_DATA(pycollision);
        std::memset(pcollision, 0, nRays * sizeof(uint8_t));
        {
            openravepy::PythonThreadSaver threadsaver;

            for(int i = 0; i < nRays; ++i, ppos += 6) {
                if (isFloat) {
                    r.pos.x = pRaysFloat[0];
                    r.pos.y = pRaysFloat[1];
                    r.pos.z = pRaysFloat[2];
                    r.dir.x = pRaysFloat[3];
                    r.dir.y = pRaysFloat[4];
                    r.dir.z = pRaysFloat[5];
                    pRaysFloat += 6;
                } else {
                    r.pos.x = pRaysDouble[0];
                    r.pos.y = pRaysDouble[1];
                    r.pos.z = pRaysDouble[2];
                    r.dir.x = pRaysDouble[3];
                    r.dir.y = pRaysDouble[4];
                    r.dir.z = pRaysDouble[5];
                    pRaysDouble += 6;
                }

                bool bCollision;
                if( !pbody ) {
                    bCollision = _penv->CheckCollision(r, preport);
                }
                else {
                    bCollision = _penv->CheckCollision(r, KinBodyConstPtr(openravepy::GetKinBody(pbody)), preport);
                }

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
        }

        return py::make_tuple(
            toPyArrayN(pcollision, nRays),
            toPyArrayN(ppos, 6 * nRays)
        );
    }

    bool CheckCollision(OPENRAVE_SHARED_PTR<PyRay> pyray)
    {
        return _penv->CheckCollision(pyray->r);
    }

    bool CheckCollision(OPENRAVE_SHARED_PTR<PyRay> pyray, PyCollisionReportPtr pReport)
    {
        bool bCollision = _penv->CheckCollision(pyray->r, openravepy::GetCollisionReport(pReport));
        openravepy::UpdateCollisionReport(pReport,shared_from_this());
        return bCollision;
    }

    bool Load(const string &filename) {
        openravepy::PythonThreadSaver threadsaver;
        return _penv->Load(filename);
    }
    bool Load(const string &filename, object odictatts) {
        AttributesList dictatts = toAttributesList(odictatts);
        openravepy::PythonThreadSaver threadsaver;
        return _penv->Load(filename, dictatts);
    }
    bool LoadURI(const string &filename, object odictatts=object()) {
        AttributesList dictatts = toAttributesList(odictatts);
        openravepy::PythonThreadSaver threadsaver;
        return _penv->LoadURI(filename, dictatts);
    }
    bool LoadData(const string &data) {
        openravepy::PythonThreadSaver threadsaver;
        return _penv->LoadData(data);
    }
    bool LoadData(const string &data, object odictatts) {
        AttributesList dictatts = toAttributesList(odictatts);
        openravepy::PythonThreadSaver threadsaver;
        return _penv->LoadData(data, dictatts);
    }

    void Save(const string &filename, EnvironmentBase::SelectionOptions options=EnvironmentBase::SO_Everything, object odictatts=object()) {
        try {
            std::string otarget = odictatts.cast<std::string>();
            // old version
            AttributesList atts;
            atts.emplace_back("target", (std::string)otarget);
            openravepy::PythonThreadSaver threadsaver;
            _penv->Save(filename,options,atts);
        }
        catch (...) {
            // new version
            AttributesList dictatts = toAttributesList(odictatts);
            openravepy::PythonThreadSaver threadsaver;
            _penv->Save(filename,options,dictatts);
        }
    }

    object WriteToMemory(const string &filetype, EnvironmentBase::SelectionOptions options=EnvironmentBase::SO_Everything, object odictatts=object()) {
        std::vector<char> output;
        try {
            std::string otarget = odictatts.cast<std::string>();
            // old version
            AttributesList atts;
            atts.emplace_back("target", (std::string)otarget);
            _penv->WriteToMemory(filetype,output,options,atts);
        }
        catch(...) {
            // new version
            _penv->WriteToMemory(filetype,output,options,toAttributesList(odictatts));
        }

        if( output.size() == 0 ) {
            return py::object();
        }
        else {
            return py::cast(PyBytes_FromStringAndSize(&output[0], output.size()));
        }
    }

    object ReadRobotURI(const string &filename)
    {
        return py::cast(openravepy::toPyRobot(_penv->ReadRobotURI(filename),shared_from_this()));
    }
    object ReadRobotURI(const string &filename, object odictatts)
    {
        return py::cast(openravepy::toPyRobot(_penv->ReadRobotURI(RobotBasePtr(), filename,toAttributesList(odictatts)),shared_from_this()));
    }
    object ReadRobotData(const string &data)
    {
        return py::cast(openravepy::toPyRobot(_penv->ReadRobotData(RobotBasePtr(), data, AttributesList()), shared_from_this()));
    }
    object ReadRobotData(const string &data, object odictatts)
    {
        return py::cast(openravepy::toPyRobot(_penv->ReadRobotData(RobotBasePtr(), data, toAttributesList(odictatts)),shared_from_this()));
    }
    object ReadKinBodyURI(const string &filename)
    {
        return py::cast(openravepy::toPyKinBody(_penv->ReadKinBodyURI(filename), shared_from_this()));
    }
    object ReadKinBodyURI(const string &filename, object odictatts)
    {
        return py::cast(openravepy::toPyKinBody(_penv->ReadKinBodyURI(KinBodyPtr(), filename, toAttributesList(odictatts)),shared_from_this()));
    }
    object ReadKinBodyData(const string &data)
    {
        return py::cast(openravepy::toPyKinBody(_penv->ReadKinBodyData(KinBodyPtr(), data, AttributesList()),shared_from_this()));
    }
    object ReadKinBodyData(const string &data, object odictatts)
    {
        return py::cast(openravepy::toPyKinBody(_penv->ReadKinBodyData(KinBodyPtr(), data, toAttributesList(odictatts)),shared_from_this()));
    }
    PyInterfaceBasePtr ReadInterfaceURI(const std::string& filename)
    {
        return _toPyInterface(_penv->ReadInterfaceURI(filename));
    }
    PyInterfaceBasePtr ReadInterfaceURI(const std::string& filename, object odictatts)
    {
        return _toPyInterface(_penv->ReadInterfaceURI(filename, toAttributesList(odictatts)));
    }
    object ReadTrimeshURI(const std::string& filename)
    {
        OPENRAVE_SHARED_PTR<TriMesh> ptrimesh = _penv->ReadTrimeshURI(OPENRAVE_SHARED_PTR<TriMesh>(),filename);
        if( !ptrimesh ) {
            return object();
        }
        return toPyTriMesh(*ptrimesh);
    }
    object ReadTrimeshURI(const std::string& filename, object odictatts)
    {
        OPENRAVE_SHARED_PTR<TriMesh> ptrimesh = _penv->ReadTrimeshURI(OPENRAVE_SHARED_PTR<TriMesh>(),filename,toAttributesList(odictatts));
        if( !ptrimesh ) {
            return object();
        }
        return toPyTriMesh(*ptrimesh);
    }

    object ReadTrimeshData(const std::string& data, const std::string& formathint)
    {
        OPENRAVE_SHARED_PTR<TriMesh> ptrimesh = _penv->ReadTrimeshData(OPENRAVE_SHARED_PTR<TriMesh>(),data,formathint);
        if( !ptrimesh ) {
            return object();
        }
        return toPyTriMesh(*ptrimesh);
    }
    object ReadTrimeshData(const std::string& data, const std::string& formathint, object odictatts)
    {
        OPENRAVE_SHARED_PTR<TriMesh> ptrimesh = _penv->ReadTrimeshData(OPENRAVE_SHARED_PTR<TriMesh>(),data,formathint,toAttributesList(odictatts));
        if( !ptrimesh ) {
            return object();
        }
        return toPyTriMesh(*ptrimesh);
    }

    void Add(PyInterfaceBasePtr pinterface, bool bAnonymous=false, const std::string& cmdargs="") {
        _penv->Add(pinterface->GetInterfaceBase(), bAnonymous, cmdargs);
    }

    void AddKinBody(PyKinBodyPtr pbody) {
        CHECK_POINTER(pbody); _penv->Add(openravepy::GetKinBody(pbody));
    }
    void AddKinBody(PyKinBodyPtr pbody, bool bAnonymous) {
        CHECK_POINTER(pbody); _penv->Add(openravepy::GetKinBody(pbody),bAnonymous);
    }
    void AddRobot(PyRobotBasePtr robot) {
        CHECK_POINTER(robot);
        _penv->Add(openravepy::GetRobot(robot));
    }
    void AddRobot(PyRobotBasePtr robot, bool bAnonymous) {
        CHECK_POINTER(robot);
        _penv->Add(openravepy::GetRobot(robot),bAnonymous);
    }
    void AddSensor(PySensorBasePtr sensor) {
        CHECK_POINTER(sensor);
        _penv->Add(openravepy::GetSensor(sensor));
    }
    void AddSensor(PySensorBasePtr sensor, bool bAnonymous) {
        CHECK_POINTER(sensor);
        _penv->Add(openravepy::GetSensor(sensor),bAnonymous);
    }
    void AddViewer(PyViewerBasePtr viewer) {
        CHECK_POINTER(viewer);
        _penv->Add(openravepy::GetViewer(viewer));
    }

    bool RemoveKinBody(PyKinBodyPtr pbody) {
        CHECK_POINTER(pbody);
        RAVELOG_WARN("openravepy RemoveKinBody deprecated, use Remove\n");
        return _penv->Remove(openravepy::GetKinBody(pbody));
    }

    bool RemoveKinBodyByName(const std::string& name) {
        return _penv->RemoveKinBodyByName(name);
    }

    object GetKinBody(const string &name)
    {
        KinBodyPtr pbody = _penv->GetKinBody(name);
        if( !pbody ) {
            return object();
        }
        if( pbody->IsRobot() ) {
            return py::cast(openravepy::toPyRobot(RaveInterfaceCast<RobotBase>(pbody),shared_from_this()));
        }
        else {
            return py::cast(openravepy::toPyKinBody(pbody,shared_from_this()));
        }
    }
    object GetRobot(const string &name)
    {
        return py::cast(openravepy::toPyRobot(_penv->GetRobot(name), shared_from_this()));
    }
    object GetSensor(const string &name)
    {
        return py::cast(openravepy::toPySensor(_penv->GetSensor(name),shared_from_this()));
    }

    object GetBodyFromEnvironmentId(int id)
    {
        return py::cast(openravepy::toPyKinBody(_penv->GetBodyFromEnvironmentId(id),shared_from_this()));
    }

    int AddModule(PyModuleBasePtr prob, const string &args) {
        CHECK_POINTER(prob);
        return _penv->AddModule(openravepy::GetModule(prob),args);
    }
    bool RemoveProblem(PyModuleBasePtr prob) {
        CHECK_POINTER(prob);
        RAVELOG_WARN("openravepy RemoveProblem deprecated, use Remove\n");
        return _penv->Remove(openravepy::GetModule(prob));
    }
    bool Remove(PyInterfaceBasePtr obj) {
        CHECK_POINTER(obj);

        // have to check if viewer in order to notify viewer manager
        ViewerBasePtr pviewer = RaveInterfaceCast<ViewerBase>(obj->GetInterfaceBase());
        if( !!pviewer ) {
            ViewerManager::GetInstance().RemoveViewer(pviewer);
        }
        return _penv->Remove(obj->GetInterfaceBase());
    }

    object GetModules()
    {
        std::list<ModuleBasePtr> listModules;
        _penv->GetModules(listModules);
        py::list modules;
        FOREACHC(itprob, listModules) {
            modules.append(openravepy::toPyModule(*itprob,shared_from_this()));
        }
        return std::move(modules);
    }

    bool SetPhysicsEngine(PyPhysicsEngineBasePtr pengine)
    {
        return _penv->SetPhysicsEngine(openravepy::GetPhysicsEngine(pengine));
    }
    object GetPhysicsEngine() {
        return py::cast(openravepy::toPyPhysicsEngine(_penv->GetPhysicsEngine(),shared_from_this()));
    }

    object RegisterBodyCallback(object fncallback)
    {
        if( !fncallback ) {
            throw openrave_exception(_("callback not specified"));
        }
        UserDataPtr p = _penv->RegisterBodyCallback(boost::bind(&PyEnvironmentBase::_BodyCallback,shared_from_this(),fncallback,_1,_2));
        if( !p ) {
            throw openrave_exception(_("registration handle is NULL"));
        }
        return openravepy::GetUserData(p);
    }

    object RegisterCollisionCallback(object fncallback)
    {
        if( !fncallback ) {
            throw openrave_exception(_("callback not specified"));
        }
        UserDataPtr p = _penv->RegisterCollisionCallback(boost::bind(&PyEnvironmentBase::_CollisionCallback,shared_from_this(),fncallback,_1,_2));
        if( !p ) {
            throw openrave_exception(_("registration handle is NULL"));
        }
        return openravepy::GetUserData(p);
    }

    bool HasRegisteredCollisionCallbacks()
    {
        return _penv->HasRegisteredCollisionCallbacks();
    }

    void StepSimulation(dReal timeStep) {
        _penv->StepSimulation(timeStep);
    }
    void StartSimulation(dReal fDeltaTime, bool bRealTime=true) {
        _penv->StartSimulation(fDeltaTime,bRealTime);
    }
    void StopSimulation(int shutdownthread=1) {
        _penv->StopSimulation(shutdownthread);
    }
    uint64_t GetSimulationTime() {
        return _penv->GetSimulationTime();
    }
    bool IsSimulationRunning() {
        return _penv->IsSimulationRunning();
    }

    void Lock()
    {
        // first try to lock without releasing the GIL since it is faster
        uint64_t nTimeoutMicroseconds = 2000; // 2ms
        uint64_t basetime = OpenRAVE::utils::GetMicroTime();
        while(OpenRAVE::utils::GetMicroTime()-basetime<nTimeoutMicroseconds ) {
            if( TryLock() ) {
                return;
            }
            boost::this_thread::sleep(boost::posix_time::microseconds(10));
        }

        // failed, so must be a python thread blocking it...
        LockReleaseGil();
    }

    /// \brief raw locking without any python overhead
    void LockRaw()
    {
#if BOOST_VERSION < 103500
        boost::mutex::scoped_lock envlock(_envmutex);
        if( _listfreelocks.size() > 0 ) {
            _listfreelocks.back()->lock();
            _listenvlocks.splice(_listenvlocks.end(),_listfreelocks,--_listfreelocks.end());
        }
        else {
            _listenvlocks.push_back(OPENRAVE_SHARED_PTR<EnvironmentMutex::scoped_lock>(new EnvironmentMutex::scoped_lock(_penv->GetMutex())));
        }
#else
        _penv->GetMutex().lock();
#endif
    }

    void LockReleaseGil()
    {
        PythonThreadSaver saver;
        LockRaw();
    }

    void Unlock()
    {
#if BOOST_VERSION < 103500
        boost::mutex::scoped_lock envlock(_envmutex);
        BOOST_ASSERT(_listenvlocks.size()>0);
        _listenvlocks.back()->unlock();
        _listfreelocks.splice(_listfreelocks.end(),_listenvlocks,--_listenvlocks.end());
#else
        _penv->GetMutex().unlock();
#endif
    }

    /// try locking the environment while releasing the GIL. This can get into a deadlock after env lock is acquired and before gil is re-acquired
    bool TryLockReleaseGil()
    {
        bool bSuccess = false;
        PythonThreadSaver saver;
#if BOOST_VERSION < 103500
        OPENRAVE_SHARED_PTR<EnvironmentMutex::scoped_try_lock> lockenv(new EnvironmentMutex::scoped_try_lock(GetEnv()->GetMutex(),false));
        if( !!lockenv->try_lock() ) {
            bSuccess = true;
            _listenvlocks.push_back(OPENRAVE_SHARED_PTR<EnvironmentMutex::scoped_lock>(new EnvironmentMutex::scoped_lock(_penv->GetMutex())));
        }
#else
        if( _penv->GetMutex().try_lock() ) {
            bSuccess = true;
        }
#endif
        return bSuccess;
    }

    bool TryLock()
    {
        bool bSuccess = false;
#if BOOST_VERSION < 103500
        OPENRAVE_SHARED_PTR<EnvironmentMutex::scoped_try_lock> lockenv(new EnvironmentMutex::scoped_try_lock(GetEnv()->GetMutex(),false));
        if( !!lockenv->try_lock() ) {
            bSuccess = true;
            _listenvlocks.push_back(OPENRAVE_SHARED_PTR<EnvironmentMutex::scoped_lock>(new EnvironmentMutex::scoped_lock(_penv->GetMutex())));
        }
#else
        if( _penv->GetMutex().try_lock() ) {
            bSuccess = true;
        }
#endif
        return bSuccess;
    }


    bool Lock(float timeout)
    {
        uint64_t nTimeoutMicroseconds = timeout*1000000;
        uint64_t basetime = OpenRAVE::utils::GetMicroTime();
        while(OpenRAVE::utils::GetMicroTime()-basetime<nTimeoutMicroseconds ) {
            if( TryLock() ) {
                return true;
            }
            boost::this_thread::sleep(boost::posix_time::milliseconds(1));
        }
        return false;
    }

    void __enter__()
    {
        Lock();
    }

    void __exit__(object type, object value, object traceback)
    {
        Unlock();
    }

    bool SetViewer(const string &viewername, bool showviewer=true)
    {
        ViewerBasePtr pviewer = ViewerManager::GetInstance().AddViewer(_penv, viewername, showviewer, true);
        return !(pviewer == NULL);
    }

    /// \brief sets the default viewer
    bool SetDefaultViewer(bool showviewer=true)
    {
        std::string viewername = RaveGetDefaultViewerType();
        if( viewername.size() > 0 ) {
            ViewerBasePtr pviewer = ViewerManager::GetInstance().AddViewer(_penv, viewername, showviewer, true);
            return !!pviewer;
        }

        return false;
    }

    object GetViewer()
    {
        return py::cast(openravepy::toPyViewer(_penv->GetViewer(),shared_from_this()));
    }

    /// returns the number of points
    static size_t _getGraphPoints(object opoints, vector<float>&vpoints)
    {
        if( PyObject_HasAttrString(opoints.ptr(),"shape") ) {
            object pointshape = opoints.attr("shape");
            switch(len(pointshape)) {
            case 1:
                vpoints = ExtractArray<float>(opoints);
                if( vpoints.size()%3 ) {
                    throw OPENRAVE_EXCEPTION_FORMAT(_("points have bad size %d"), vpoints.size(),ORE_InvalidArguments);
                }
                return vpoints.size()/3;
            case 2: {
                int num = pointshape[0].cast<int>();
                int dim = pointshape[1].cast<int>();
                vpoints = ExtractArray<float>(opoints.attr("flat"));
                if(dim % 3) {
                    throw OPENRAVE_EXCEPTION_FORMAT(_("points have bad size %dx%d"), num%dim,ORE_InvalidArguments);
                }
                return num*(dim/3);
            }
            default:
                throw openrave_exception(_("points have bad dimension"));
            }
        }
        // assume it is a regular 1D list
        vpoints = ExtractArray<float>(opoints);
        if( vpoints.size()% 3 ) {
            throw OPENRAVE_EXCEPTION_FORMAT(_("points have bad size %d"), vpoints.size(),ORE_InvalidArguments);
        }
        return vpoints.size()/3;
    }

    /// returns the number of colors
    static size_t _getGraphColors(object ocolors, vector<float>&vcolors)
    {
        if( !IS_PYTHONOBJECT_NONE(ocolors) ) {
            if( PyObject_HasAttrString(ocolors.ptr(),"shape") ) {
                object colorshape = ocolors.attr("shape");
                switch( len(colorshape) ) {
                case 1:
                    break;
                case 2: {
                    int numcolors = colorshape[0].cast<int>();
                    int colordim = colorshape[1].cast<int>();
                    if(( colordim != 3) &&( colordim != 4) ) {
                        throw OPENRAVE_EXCEPTION_FORMAT(_("colors dim %d needs to be 3 or 4"),colordim, ORE_InvalidArguments);
                    }
                    vcolors = ExtractArray<float>(ocolors.attr("flat"));
                    return numcolors;
                }
                default:
                    throw OPENRAVE_EXCEPTION_FORMAT(_("colors has %d dimensions"),len(colorshape), ORE_InvalidArguments);
                }
            }
            vcolors = ExtractArray<float>(ocolors);
            if( vcolors.size() == 3 ) {
                vcolors.push_back(1.0f);
            }
            else if( vcolors.size() != 4 ) {
                throw OPENRAVE_EXCEPTION_FORMAT(_("colors has incorrect number of values %d"),vcolors.size(), ORE_InvalidArguments);
            }
            return 1;
        }
        // default
        RaveVector<float> vcolor(1,0.5,0.5,1.0);
        vcolors.resize(4);
        vcolors[0] = 1; vcolors[1] = 0.5f; vcolors[2] = 0.5f; vcolors[3] = 1.0f;
        return 1;
    }

    static pair<size_t,size_t> _getGraphPointsColors(object opoints, object ocolors, vector<float>&vpoints, vector<float>&vcolors)
    {
        size_t numpoints = _getGraphPoints(opoints,vpoints);
        size_t numcolors = _getGraphColors(ocolors,vcolors);
        if( numpoints <= 0 ) {
            throw openrave_exception(_("points cannot be empty"),ORE_InvalidArguments);
        }
        if(( numcolors > 1) &&( numpoints != numcolors) ) {
            throw openrave_exception(boost::str(boost::format(_("number of points (%d) need to match number of colors (%d)"))%numpoints%numcolors));
        }
        return make_pair(numpoints,numcolors);
    }

    object plot3(object opoints,float pointsize,object ocolors=object(),int drawstyle=0)
    {
        vector<float> vpoints, vcolors;
        pair<size_t,size_t> sizes = _getGraphPointsColors(opoints,ocolors,vpoints,vcolors);
        bool bhasalpha = vcolors.size() == 4*sizes.second;
        if( sizes.first == sizes.second ) {
            return toPyGraphHandle(_penv->plot3(&vpoints[0],sizes.first,sizeof(float)*3,pointsize,&vcolors[0],drawstyle,bhasalpha));
        }
        BOOST_ASSERT(vcolors.size()<=4);
        RaveVector<float> vcolor;
        for(int i = 0; i < (int)vcolors.size(); ++i) {
            vcolor[i] = vcolors[i];
        }
        return toPyGraphHandle(_penv->plot3(&vpoints[0],sizes.first,sizeof(float)*3,pointsize,vcolor,drawstyle));
    }

    object drawlinestrip(object opoints,float linewidth,object ocolors=object(),int drawstyle=0)
    {
        vector<float> vpoints, vcolors;
        pair<size_t,size_t> sizes = _getGraphPointsColors(opoints,ocolors,vpoints,vcolors);
        //bool bhasalpha = vcolors.size() == 4*sizes.second;
        if( sizes.first == sizes.second ) {
            return toPyGraphHandle(_penv->drawlinestrip(&vpoints[0],sizes.first,sizeof(float)*3,linewidth,&vcolors[0]));
        }
        BOOST_ASSERT(vcolors.size()<=4);
        RaveVector<float> vcolor;
        for(int i = 0; i < (int)vcolors.size(); ++i) {
            vcolor[i] = vcolors[i];
        }
        return toPyGraphHandle(_penv->drawlinestrip(&vpoints[0],sizes.first,sizeof(float)*3,linewidth,vcolor));
    }

    object drawlinelist(object opoints,float linewidth,object ocolors=object(),int drawstyle=0)
    {
        vector<float> vpoints, vcolors;
        pair<size_t,size_t> sizes = _getGraphPointsColors(opoints,ocolors,vpoints,vcolors);
        //bool bhasalpha = vcolors.size() == 4*sizes.second;
        if( sizes.first == sizes.second ) {
            return toPyGraphHandle(_penv->drawlinelist(&vpoints[0],sizes.first,sizeof(float)*3,linewidth,&vcolors[0]));
        }
        BOOST_ASSERT(vcolors.size()<=4);
        RaveVector<float> vcolor;
        for(int i = 0; i < (int)vcolors.size(); ++i) {
            vcolor[i] = vcolors[i];
        }
        return toPyGraphHandle(_penv->drawlinelist(&vpoints[0],sizes.first,sizeof(float)*3,linewidth,vcolor));
    }

    object drawarrow(object op1, object op2, float linewidth=0.002, object ocolor=object())
    {
        RaveVector<float> vcolor(1,0.5,0.5,1);
        if( !IS_PYTHONOBJECT_NONE(ocolor) ) {
            vcolor = ExtractVector34(ocolor,1.0f);
        }
        return toPyGraphHandle(_penv->drawarrow(ExtractVector3(op1),ExtractVector3(op2),linewidth,vcolor));
    }

    object drawbox(object opos, object oextents, object ocolor=object())
    {
        RaveVector<float> vcolor(1,0.5,0.5,1);
        if( !IS_PYTHONOBJECT_NONE(ocolor) ) {
            vcolor = ExtractVector34(ocolor,1.0f);
        }
        return toPyGraphHandle(_penv->drawbox(ExtractVector3(opos),ExtractVector3(oextents)));
    }

    object drawplane(object otransform, object oextents, const boost::multi_array<float,2>&_vtexture)
    {
        boost::multi_array<float,3> vtexture(boost::extents[1][_vtexture.shape()[0]][_vtexture.shape()[1]]);
        vtexture[0] = _vtexture;
        boost::array<size_t,3> dims = { { _vtexture.shape()[0],_vtexture.shape()[1],1}};
        vtexture.reshape(dims);
        return toPyGraphHandle(_penv->drawplane(RaveTransform<float>(ExtractTransform(otransform)), 
            RaveVector<float>(oextents[0].cast<float>(), oextents[1].cast<float>(), 0), vtexture));
    }
    object drawplane(object otransform, object oextents, const boost::multi_array<float,3>&vtexture)
    {
        return toPyGraphHandle(_penv->drawplane(RaveTransform<float>(ExtractTransform(otransform)), 
            RaveVector<float>(oextents[0].cast<float>(), oextents[1].cast<float>(), 0), vtexture));
    }

    object drawtrimesh(object opoints, object oindices=object(), object ocolors=object())
    {
        vector<float> vpoints;
        _getGraphPoints(opoints,vpoints);
        vector<int> vindices;
        int* pindices = NULL;
        int numTriangles = vpoints.size()/9;
        if( !IS_PYTHONOBJECT_NONE(oindices) ) {
            vindices = ExtractArray<int>(oindices.attr("flat"));
            if( vindices.size() > 0 ) {
                numTriangles = vindices.size()/3;
                pindices = &vindices[0];
            }
        }
        RaveVector<float> vcolor(1,0.5,0.5,1);
        if( !IS_PYTHONOBJECT_NONE(ocolors) ) {
            object shape = ocolors.attr("shape");
            if( len(shape) == 1 ) {
                return toPyGraphHandle(_penv->drawtrimesh(&vpoints[0],sizeof(float)*3,pindices,numTriangles,ExtractVector34(ocolors,1.0f)));
            }
            else {
                BOOST_ASSERT(shape[0].cast<size_t>()==vpoints.size()/3);
                return toPyGraphHandle(_penv->drawtrimesh(&vpoints[0], sizeof(float)*3, pindices, numTriangles, /*extract<boost::multi_array<float,2> >*/ocolors.cast<Vector>()));
            }
        }
        return toPyGraphHandle(_penv->drawtrimesh(&vpoints[0],sizeof(float)*3,pindices,numTriangles,RaveVector<float>(1,0.5,0.5,1)));
    }

    object GetBodies()
    {
        std::vector<KinBodyPtr> vbodies;
        _penv->GetBodies(vbodies);
        py::list bodies;
        FOREACHC(itbody, vbodies) {
            if( (*itbody)->IsRobot() ) {
                bodies.append(openravepy::toPyRobot(RaveInterfaceCast<RobotBase>(*itbody),shared_from_this()));
            }
            else {
                bodies.append(openravepy::toPyKinBody(*itbody,shared_from_this()));
            }
        }
        return std::move(bodies);
    }

    object GetRobots()
    {
        std::vector<RobotBasePtr> vrobots;
        _penv->GetRobots(vrobots);
        py::list robots;
        FOREACHC(itrobot, vrobots) {
            robots.append(openravepy::toPyRobot(*itrobot,shared_from_this()));
        }
        return std::move(robots);
    }

    object GetSensors()
    {
        std::vector<SensorBasePtr> vsensors;
        _penv->GetSensors(vsensors);
        py::list sensors;
        FOREACHC(itsensor, vsensors) {
            sensors.append(openravepy::toPySensor(*itsensor,shared_from_this()));
        }
        return std::move(sensors);
    }

    void UpdatePublishedBodies()
    {
        _penv->UpdatePublishedBodies();
    }

    object GetPublishedBodies(uint64_t timeout=0)
    {
        std::vector<KinBody::BodyState> vbodystates;
        _penv->GetPublishedBodies(vbodystates, timeout);
        py::list ostates;
        FOREACH(itstate, vbodystates) {
            py::dict ostate;
            ostate["body"] = toPyKinBody(itstate->pbody, shared_from_this());
            py::list olinktransforms;
            FOREACH(ittransform, itstate->vectrans) {
                olinktransforms.append(ReturnTransform(*ittransform));
            }
            ostate["linktransforms"] = olinktransforms;
            ostate["jointvalues"] = toPyArray(itstate->jointvalues);
            ostate["linkEnableStates"] = toPyArray(itstate->vLinkEnableStates);
            ostate["connectedBodyActiveStates"] = toPyArray(itstate->vConnectedBodyActiveStates);
            ostate["name"] = ConvertStringToUnicode(itstate->strname);
            ostate["uri"] = ConvertStringToUnicode(itstate->uri);
            ostate["updatestamp"] = itstate->updatestamp;
            ostate["environmentid"] = itstate->environmentid;
            ostate["activeManipulatorName"] = itstate->activeManipulatorName;
            ostate["activeManipulatorTransform"] = ReturnTransform(itstate->activeManipulatorTransform);
            ostates.append(ostate);
        }
        return std::move(ostates);
    }

    object GetPublishedBody(const string &name, uint64_t timeout = 0)
    {
        KinBody::BodyState bodystate;
        if( !_penv->GetPublishedBody(name, bodystate, timeout) ) {
            return object();
        }

        py::dict ostate;
        ostate["body"] = toPyKinBody(bodystate.pbody, shared_from_this());
        py::list olinktransforms;
        FOREACH(ittransform, bodystate.vectrans) {
            olinktransforms.append(ReturnTransform(*ittransform));
        }
        ostate["linktransforms"] = olinktransforms;
        ostate["jointvalues"] = toPyArray(bodystate.jointvalues);
        ostate["linkEnableStates"] = toPyArray(bodystate.vLinkEnableStates);
        ostate["connectedBodyActiveStates"] = toPyArray(bodystate.vConnectedBodyActiveStates);
        ostate["name"] = ConvertStringToUnicode(bodystate.strname);
        ostate["uri"] = ConvertStringToUnicode(bodystate.uri);
        ostate["updatestamp"] = bodystate.updatestamp;
        ostate["environmentid"] = bodystate.environmentid;
        ostate["activeManipulatorName"] = bodystate.activeManipulatorName;
        ostate["activeManipulatorTransform"] = ReturnTransform(bodystate.activeManipulatorTransform);
        return std::move(ostate);
    }

    object GetPublishedBodyJointValues(const string &name, uint64_t timeout=0)
    {
        std::vector<dReal> jointValues;
        if( !_penv->GetPublishedBodyJointValues(name, jointValues, timeout) ) {
            return object();
        }
        return toPyArray(jointValues);
    }

    object GetPublishedBodyTransformsMatchingPrefix(const string &prefix, uint64_t timeout=0) {
        std::vector< std::pair<std::string, Transform> > nameTransfPairs;
        _penv->GetPublishedBodyTransformsMatchingPrefix(prefix, nameTransfPairs, timeout);

        py::dict otransforms;
        FOREACH(itpair, nameTransfPairs) {
            otransforms[itpair->first] = ReturnTransform(itpair->second);
        }

        return std::move(otransforms);
    }

    object Triangulate(PyKinBodyPtr pbody)
    {
        CHECK_POINTER(pbody);
        TriMesh mesh;
        _penv->Triangulate(mesh, *openravepy::GetKinBody(pbody));
        return toPyTriMesh(mesh);
    }

    object TriangulateScene(EnvironmentBase::SelectionOptions options, const string &name)
    {
        TriMesh mesh;
        _penv->TriangulateScene(mesh,options,name);
        return toPyTriMesh(mesh);
    }

    void SetDebugLevel(object olevel) {
        _penv->SetDebugLevel(pyGetIntFromPy(olevel,Level_Info));
    }
    int GetDebugLevel() const {
        return _penv->GetDebugLevel();
    }

    string GetHomeDirectory() {
        RAVELOG_WARN("Environment.GetHomeDirectory is deprecated, use RaveGetHomeDirectory\n"); return RaveGetHomeDirectory();
    }

    void SetUserData(PyUserData pdata) {
        _penv->SetUserData(pdata._handle);
    }
    void SetUserData(object o) {
        _penv->SetUserData(OPENRAVE_SHARED_PTR<UserData>(new PyUserObject(o)));
    }
    object GetUserData() const {
        return openravepy::GetUserData(_penv->GetUserData());
    }

    void SetUnit(std::string unitname, dReal unitmult){
        _penv->SetUnit(std::make_pair(unitname, unitmult));
    }

    object GetUnit() const {
        std::pair<std::string, dReal> unit = _penv->GetUnit();
        return py::make_tuple(unit.first, unit.second);

    }

    bool __eq__(PyEnvironmentBasePtr p) {
        return !!p && _penv==p->_penv;
    }
    bool __ne__(PyEnvironmentBasePtr p) {
        return !p || _penv!=p->_penv;
    }
    string __repr__() {
        return boost::str(boost::format("RaveGetEnvironment(%d)")%RaveGetEnvironmentId(_penv));
    }
    string __str__() {
        return boost::str(boost::format("<env %d>")%RaveGetEnvironmentId(_penv));
    }
    object __unicode__() {
        return ConvertStringToUnicode(__str__());
    }

    EnvironmentBasePtr GetEnv() const {
        return _penv;
    }
};

} // namespace openravepy

#endif // OPENRAVEPY_INTERNAL_ENVIRONMENT_H