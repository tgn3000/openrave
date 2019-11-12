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
#ifndef OPENRAVEPY_INTERNAL_H
#define OPENRAVEPY_INTERNAL_H

#include <Python.h>

#include <openrave-core/openrave-core.h>

#ifndef _WIN32
#include <sys/time.h>
#define Sleep(milli) usleep(1000*milli)
#else
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#endif

#include <sstream>
#include <exception>

#include <boost/array.hpp>
#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/condition.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/format.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <boost/version.hpp>

#define PY_ARRAY_UNIQUE_SYMBOL PyArrayHandle
// #include <boost/python.hpp>
// #include <boost/python/exception_translator.hpp>
// #include <boost/python/docstring_options.hpp>
#include <pyconfig.h>
#include <numpy/arrayobject.h>

#define OPENRAVE_BININGS_PYARRAY
#include "bindings.h"
#include "docstrings.h"

#define _(msgid) OpenRAVE::RaveGetLocalizedTextForDomain("openrave", msgid)

#define CHECK_POINTER(p) { \
        if( !(p) ) { throw openrave_exception(boost::str(boost::format(_("[%s:%d]: invalid pointer"))%__PRETTY_FUNCTION__%__LINE__)); } \
}

// using namespace boost::python;
using namespace std;
using namespace OpenRAVE;

namespace openravepy {

struct DummyStruct {};

class PyInterfaceBase;
class PyKinBody;
class PyRobotBase;
class PyEnvironmentBase;
class PyCollisionReport;
class PyPhysicsEngineBase;
class PyCollisionCheckerBase;
class PyIkSolverBase;
class PyPlannerBase;
class PySensorBase;
class PySensorSystemBase;
class PyControllerBase;
class PyMultiControllerBase;
class PyTrajectoryBase;
class PyModuleBase;
class PyViewerBase;
class PySpaceSamplerBase;
class PyConfigurationSpecification;
class PyIkParameterization;
class PyXMLReadable;
class PyCameraIntrinsics;
class PyLinkInfo;
class PyJointInfo;
class PyManipulatorInfo;
class PyAttachedSensorInfo;
class PyConnectedBodyInfo;
class PyLink;
class PyJoint;

typedef OPENRAVE_SHARED_PTR<PyInterfaceBase> PyInterfaceBasePtr;
typedef OPENRAVE_SHARED_PTR<PyInterfaceBase const> PyInterfaceBaseConstPtr;
typedef OPENRAVE_SHARED_PTR<PyKinBody> PyKinBodyPtr;
typedef OPENRAVE_SHARED_PTR<PyKinBody const> PyKinBodyConstPtr;
typedef OPENRAVE_SHARED_PTR<PyRobotBase> PyRobotBasePtr;
typedef OPENRAVE_SHARED_PTR<PyRobotBase const> PyRobotBaseConstPtr;
typedef OPENRAVE_SHARED_PTR<PyEnvironmentBase> PyEnvironmentBasePtr;
typedef OPENRAVE_SHARED_PTR<PyEnvironmentBase const> PyEnvironmentBaseConstPtr;
typedef OPENRAVE_SHARED_PTR<PyIkSolverBase> PyIkSolverBasePtr;
typedef OPENRAVE_SHARED_PTR<PyIkSolverBase const> PyIkSolverBaseConstPtr;
typedef OPENRAVE_SHARED_PTR<PyTrajectoryBase> PyTrajectoryBasePtr;
typedef OPENRAVE_SHARED_PTR<PyTrajectoryBase const> PyTrajectoryBaseConstPtr;
typedef OPENRAVE_SHARED_PTR<PyPhysicsEngineBase> PyPhysicsEngineBasePtr;
typedef OPENRAVE_SHARED_PTR<PyPhysicsEngineBase const> PyPhysicsEngineBaseConstPtr;
typedef OPENRAVE_SHARED_PTR<PyCollisionCheckerBase> PyCollisionCheckerBasePtr;
typedef OPENRAVE_SHARED_PTR<PyCollisionCheckerBase const> PyCollisionCheckerBaseConstPtr;
typedef OPENRAVE_SHARED_PTR<PyCollisionReport> PyCollisionReportPtr;
typedef OPENRAVE_SHARED_PTR<PyCollisionReport const> PyCollisionReportConstPtr;
typedef OPENRAVE_SHARED_PTR<PyPlannerBase> PyPlannerBasePtr;
typedef OPENRAVE_SHARED_PTR<PyPlannerBase const> PyPlannerBaseConstPtr;
typedef OPENRAVE_SHARED_PTR<PySensorBase> PySensorBasePtr;
typedef OPENRAVE_SHARED_PTR<PySensorBase const> PySensorBaseConstPtr;
typedef OPENRAVE_SHARED_PTR<PySensorSystemBase> PySensorSystemBasePtr;
typedef OPENRAVE_SHARED_PTR<PySensorSystemBase const> PySensorSystemBaseConstPtr;
typedef OPENRAVE_SHARED_PTR<PyControllerBase> PyControllerBasePtr;
typedef OPENRAVE_SHARED_PTR<PyControllerBase const> PyControllerBaseConstPtr;
typedef OPENRAVE_SHARED_PTR<PyMultiControllerBase> PyMultiControllerBasePtr;
typedef OPENRAVE_SHARED_PTR<PyMultiControllerBase const> PyMultiControllerBaseConstPtr;
typedef OPENRAVE_SHARED_PTR<PyModuleBase> PyModuleBasePtr;
typedef OPENRAVE_SHARED_PTR<PyModuleBase const> PyModuleBaseConstPtr;
typedef OPENRAVE_SHARED_PTR<PyViewerBase> PyViewerBasePtr;
typedef OPENRAVE_SHARED_PTR<PyViewerBase const> PyViewerBaseConstPtr;
typedef OPENRAVE_SHARED_PTR<PySpaceSamplerBase> PySpaceSamplerBasePtr;
typedef OPENRAVE_SHARED_PTR<PySpaceSamplerBase const> PySpaceSamplerBaseConstPtr;
typedef OPENRAVE_SHARED_PTR<PyConfigurationSpecification> PyConfigurationSpecificationPtr;
typedef OPENRAVE_SHARED_PTR<PyConfigurationSpecification const> PyConfigurationSpecificationConstPtr;
typedef OPENRAVE_SHARED_PTR<PyIkParameterization> PyIkParameterizationPtr;
typedef OPENRAVE_SHARED_PTR<PyXMLReadable> PyXMLReadablePtr;
typedef OPENRAVE_SHARED_PTR<PyCameraIntrinsics> PyCameraIntrinsicsPtr;
typedef OPENRAVE_SHARED_PTR<PyLinkInfo> PyLinkInfoPtr;
typedef OPENRAVE_SHARED_PTR<PyJointInfo> PyJointInfoPtr;
typedef OPENRAVE_SHARED_PTR<PyManipulatorInfo> PyManipulatorInfoPtr;
typedef OPENRAVE_SHARED_PTR<PyAttachedSensorInfo> PyAttachedSensorInfoPtr;
typedef OPENRAVE_SHARED_PTR<PyConnectedBodyInfo> PyConnectedBodyInfoPtr;
typedef OPENRAVE_SHARED_PTR<PyLink> PyLinkPtr;
typedef OPENRAVE_SHARED_PTR<PyLink const> PyLinkConstPtr;
typedef OPENRAVE_SHARED_PTR<PyJoint> PyJointPtr;
typedef OPENRAVE_SHARED_PTR<PyJoint const> PyJointConstPtr;

inline uint64_t GetMicroTime()
{
#ifdef _WIN32
    LARGE_INTEGER count, freq;
    QueryPerformanceCounter(&count);
    QueryPerformanceFrequency(&freq);
    return (count.QuadPart * 1000000) / freq.QuadPart;
#else
    struct timeval t;
    gettimeofday(&t, NULL);
    return (uint64_t)t.tv_sec*1000000+t.tv_usec;
#endif
}

#ifdef OPENRAVE_RAPIDJSON
/// conversion between rapidjson value and py::object
py::object toPyObject(const rapidjson::Value& value);
void toRapidJSONValue(py::object &obj, rapidjson::Value &value, rapidjson::Document::AllocatorType& allocator);
#endif // OPENRAVE_RAPIDJSON

/// used externally, don't change definitions
//@{
Transform ExtractTransform(const py::object& oraw);
TransformMatrix ExtractTransformMatrix(const py::object& oraw);
py::object toPyArray(const TransformMatrix& t);
py::object toPyArray(const Transform& t);

XMLReadablePtr ExtractXMLReadable(py::object o);
py::object toPyXMLReadable(XMLReadablePtr p);
bool ExtractIkParameterization(py::object o, IkParameterization& ikparam);
py::object toPyIkParameterization(const IkParameterization& ikparam);
py::object toPyIkParameterization(const std::string& serializeddata);
//@}

struct null_deleter {
    void operator()(void const *) const {
    }
};

class PythonThreadSaver
{
public:
    PythonThreadSaver() {
        _save = PyEval_SaveThread();
    }
    virtual ~PythonThreadSaver() {
        PyEval_RestoreThread(_save);
    }
protected:
    PyThreadState *_save;
};

/// \brief release and restore the python GIL... no thread state saved..?
class PythonGILSaver
{
public:
    PythonGILSaver() {
        PyEval_ReleaseLock();
    }
    virtual ~PythonGILSaver() {
        PyEval_AcquireLock();
    }
};

class AutoPyArrayObjectDereferencer
{
public:
    AutoPyArrayObjectDereferencer(PyArrayObject* pyarrobj) : _pyarrobj(pyarrobj) {
    }
    ~AutoPyArrayObjectDereferencer() {
        Py_DECREF(_pyarrobj);
    }

private:
    PyArrayObject* _pyarrobj;
};

typedef OPENRAVE_SHARED_PTR<PythonThreadSaver> PythonThreadSaverPtr;

inline RaveVector<float> ExtractFloat3(const py::object& o)
{
    return RaveVector<float>(o[0].cast<float>(), o[1].cast<float>(), o[2].cast<float>());
}

template <typename T>
inline RaveVector<T> ExtractVector2Type(const py::object& o)
{
    return RaveVector<T>(o[0].cast<T>(), o[1].cast<T>(),0);
}

template <typename T>
inline RaveVector<T> ExtractVector3Type(const py::object& o)
{
    return RaveVector<T>(o[0].cast<T>(), o[1].cast<T>(), o[2].cast<T>());
}

template <typename T>
inline RaveVector<T> ExtractVector4Type(const py::object& o)
{
    return RaveVector<T>(o[0].cast<T>(), o[1].cast<T>(), o[2].cast<T>(), o[3].cast<T>());
}

inline Vector ExtractVector2(const py::object& oraw)
{
    return ExtractVector2Type<dReal>(oraw);
}

inline Vector ExtractVector3(const py::object& oraw)
{
    return ExtractVector3Type<dReal>(oraw);
}

inline Vector ExtractVector4(const py::object& oraw)
{
    return ExtractVector4Type<dReal>(oraw);
}

template <typename T>
inline RaveVector<T> ExtractVector34(const py::object& oraw,T fdefaultw)
{
    int n = len(oraw);
    if( n == 3 ) {
        RaveVector<T> v = ExtractVector3Type<T>(oraw);
        v.w = fdefaultw;
        return v;
    }
    else if( n == 4 ) {
        return ExtractVector4Type<T>(oraw);
    }
    throw openrave_exception(_("unexpected vector size"));
}

template <typename T>
inline RaveVector<T> ExtractVector(const py::object& oraw)
{
    int n = len(oraw);
    if( n > 4 ) {
        throw OPENRAVE_EXCEPTION_FORMAT(_("unexpected vector size %d"),n,ORE_InvalidArguments);
    }
    Vector v;
    for(int i = 0; i < n; ++i) {
        v[i] = (T)oraw[i].cast<T>();
    }
    return v;
}

template <typename T>
inline RaveTransform<T> ExtractTransformType(const py::object& o)
{
    if( len(o) == 7 ) {
        return RaveTransform<T>(RaveVector<T>(o[0].cast<T>(), o[1].cast<T>(), o[2].cast<T>(), o[3].cast<T>()), RaveVector<T>(o[4].cast<T>(), o[5].cast<T>(), o[6].cast<T>()));
    }
    RaveTransformMatrix<T> t;
    for(int i = 0; i < 3; ++i) {
        py::object orow = o[i];
        t.m[4*i+0] = orow[0].cast<T>();
        t.m[4*i+1] = orow[1].cast<T>();
        t.m[4*i+2] = orow[2].cast<T>();
        t.trans[i] = orow[3].cast<T>();
    }
    return t;
}

template <typename T>
inline RaveTransformMatrix<T> ExtractTransformMatrixType(const py::object& o)
{
    if( len(o) == 7 ) {
        return RaveTransform<T>(RaveVector<T>(o[0].cast<T>(), o[1].cast<T>(), o[2].cast<T>(), o[3].cast<T>()), RaveVector<T>(o[4].cast<T>(), o[5].cast<T>(), o[6].cast<T>()));
    }
    RaveTransformMatrix<T> t;
    for(int i = 0; i < 3; ++i) {
        py::object orow = o[i];
        t.m[4*i+0] = orow[0].cast<T>();
        t.m[4*i+1] = orow[1].cast<T>();
        t.m[4*i+2] = orow[2].cast<T>();
        t.trans[i] = orow[3].cast<T>();
    }
    return t;
}

inline py::object toPyArrayRotation(const TransformMatrix& t)
{
    py::array_t<dReal> A;
    A.resize({3, 3});
    for(int i = 0; i < 3; ++i) {
        for(int j = 0; j < 3; ++j) {
            A[i][j] = t.m[4*i+j];
        }
    }
    return std::move(A);
}

inline py::object toPyArray3(const std::vector<RaveVector<float> >& v)
{
    npy_intp dims[] = { npy_intp(v.size()), npy_intp(3) };
    PyObject *pyvalues = PyArray_SimpleNew(2,dims, PyArray_FLOAT);
    float* pf = nullptr;
    if( v.empty() ) {
        return toPyArrayN(pf, 0);
    }
    pf = (float*) PyArray_DATA(pyvalues);
    FOREACHC(it, v) {
        *pf++ = it->x;
        *pf++ = it->y;
        *pf++ = it->z;
    }
    return toPyArrayN(pf, 3 * v.size());
    // return static_cast<numeric::array>(handle<>(pyvalues));
}

inline py::object toPyArray3(const std::vector<RaveVector<double> >& v)
{
    npy_intp dims[] = { npy_intp(v.size()), npy_intp(3) };
    PyObject *pyvalues = PyArray_SimpleNew(2,dims, PyArray_DOUBLE);
    double* pf = nullptr;
    if( v.empty() ) {
        return toPyArrayN(pf, 0);
    }
    pf = (double*) PyArray_DATA(pyvalues);
    FOREACHC(it, v) {
        *pf++ = it->x;
        *pf++ = it->y;
        *pf++ = it->z;
    }
    return toPyArrayN(pf, 3 * v.size());
    // return static_cast<numeric::array>(handle<>(pyvalues));
}

inline py::object toPyVector2(Vector v)
{
    std::array<dReal, 2> a {v.x, v.y};
    return py::array_t<dReal>({1, 2}, a.data()); 
}

inline py::object toPyVector3(Vector v)
{
    std::array<dReal, 3> a {v.x, v.y, v.z};
    return py::array_t<dReal>({1, 3}, a.data()); 
}

inline py::object toPyVector4(Vector v)
{
    std::array<dReal, 4> a {v.x, v.y, v.z, v.w};
    return py::array_t<dReal>({1, 4}, a.data()); 
}

/// \brief converts dictionary of keyvalue pairs
AttributesList toAttributesList(py::dict odict);
/// \brief converts list of tuples [(key,value),(key,value)], it is possible for keys to repeat
AttributesList toAttributesList(py::list olist);
AttributesList toAttributesList(py::object oattributes);

bool GetReturnTransformQuaternions();

template <typename T>
inline py::object ReturnTransform(T t)
{
    if( GetReturnTransformQuaternions() ) {
        return toPyArray(Transform(t));
    }
    else {
        return toPyArray(TransformMatrix(t));
    }
}

class PyPluginInfo
{
public:
    PyPluginInfo(const PLUGININFO& info)
    {
        FOREACHC(it, info.interfacenames) {
            py::list names;
            FOREACHC(itname,it->second)
            names.append(*itname);
            interfacenames.append(py::make_tuple(it->first,names));
        }
        version = OPENRAVE_VERSION_STRING_FORMAT(info.version);
    }

    py::list interfacenames;
    string version;
};

class PyGraphHandle
{
public:
    PyGraphHandle() {
    }
    PyGraphHandle(GraphHandlePtr handle) : _handle(handle) {
    }
    virtual ~PyGraphHandle() {
    }

    void SetTransform(py::object otrans) {
        _handle->SetTransform(RaveTransform<float>(ExtractTransformMatrixType<float>(otrans)));
    }
    void SetShow(bool bshow) {
        _handle->SetShow(bshow);
    }
    void Close()
    {
        _handle.reset();
    }

private:
    GraphHandlePtr _handle;
};

class PyEnvironmentLockSaver
{
public:
    PyEnvironmentLockSaver(PyEnvironmentBasePtr pyenv, bool braw);
    ~PyEnvironmentLockSaver();
protected:
    PyEnvironmentBasePtr _pyenv;
};

typedef OPENRAVE_SHARED_PTR<PyEnvironmentLockSaver> PyEnvironmentLockSaverPtr;

class PyUserData
{
public:
    PyUserData() {
    }
    PyUserData(UserDataPtr handle) : _handle(handle) {
    }
    virtual ~PyUserData() {
    }
    virtual void Close() {
        _handle.reset();
    }
    UserDataPtr _handle;
};

class PySerializableData : public PyUserData
{
public:
    class StringSerializableData : public SerializableData
    {
public:
        StringSerializableData(const std::string& data) : _data(data) {
        }

        virtual void Serialize(std::ostream& O, int options=0) const {
            O << _data;
        }

        virtual void Deserialize(std::istream& I) {
            // need to read the entire input
            stringbuf buf;
            I.get(buf, 0);
            _data = buf.str();
        }

        std::string _data;
    };

    PySerializableData() {
    }
    PySerializableData(const std::string& data) {
        _handle.reset(new StringSerializableData(data));
    }
    PySerializableData(SerializableDataPtr handle) : _handle(handle) {
    }
    void Close() {
        _handle.reset();
    }
    py::object Serialize(int options) {
        std::stringstream ss;
        ss << std::setprecision(std::numeric_limits<dReal>::digits10+1);
        _handle->Serialize(ss,options);
        return ConvertStringToUnicode(ss.str());
    }
    void Deserialize(const std::string& s) {
        std::stringstream ss(s);
        _handle->Deserialize(ss);
    }
    SerializableDataPtr _handle;
};

class PyUserObject : public UserData
{
public:
    PyUserObject(py::object o) : _o(o) {
    }
    py::object _o;
};

class PyRay
{
public:
    PyRay() {
    }
    PyRay(py::object newpos, py::object newdir);
    PyRay(const RAY& newr) : r(newr) {
    }
    py::object dir();
    py::object pos();
    virtual string __repr__();
    virtual string __str__();
    virtual py::object __unicode__();
    RAY r;
};

py::object toPyGraphHandle(const GraphHandlePtr p);
py::object toPyUserData(UserDataPtr p);
void init_openravepy_ikparameterization();
py::object toPyAABB(const AABB& ab);
py::object toPyRay(const RAY& r);
RAY ExtractRay(py::object o);

/// \brief PyAABB -> AABB
AABB ExtractAABB(py::object o);
bool ExtractRay(py::object o, RAY& r);
py::object toPyTriMesh(const TriMesh& mesh);
bool ExtractTriMesh(py::object o, TriMesh& mesh);

class PyInterfaceBase
{
protected:
    InterfaceBasePtr _pbase;
    PyEnvironmentBasePtr _pyenv;
public:
    PyInterfaceBase(InterfaceBasePtr pbase, PyEnvironmentBasePtr pyenv);
    virtual ~PyInterfaceBase() {
    }

    InterfaceType GetInterfaceType() const {
        return _pbase->GetInterfaceType();
    }
    string GetXMLId() const {
        return _pbase->GetXMLId();
    }
    string GetPluginName() const {
        return _pbase->GetPluginName();
    }
    py::object GetDescription() const {
        return ConvertStringToUnicode(_pbase->GetDescription());
    }
    void SetDescription(const std::string& s) {
        _pbase->SetDescription(s);
    }
    PyEnvironmentBasePtr GetEnv() const;

    void Clone(PyInterfaceBasePtr preference, int cloningoptions) {
        CHECK_POINTER(preference);
        _pbase->Clone(preference->GetInterfaceBase(),cloningoptions);
    }

    void SetUserData(PyUserData pdata) {
        _pbase->SetUserData(std::string(), pdata._handle);
    }
    void SetUserData(const std::string& key, PyUserData pdata) {
        _pbase->SetUserData(key, pdata._handle);
    }
    void SetUserData(py::object o) {
        _pbase->SetUserData(std::string(), OPENRAVE_SHARED_PTR<UserData>(new PyUserObject(o)));
    }
    void SetUserData(const std::string& key, py::object o) {
        _pbase->SetUserData(key, OPENRAVE_SHARED_PTR<UserData>(new PyUserObject(o)));
    }
    bool RemoveUserData(const std::string& key) {
        return _pbase->RemoveUserData(key);
    }
    py::object GetUserData(const std::string& key=std::string()) const;

    bool SupportsCommand(const string& cmd);
    py::object SendCommand(const string& in, bool releasegil=false, bool lockenv=false);

#ifdef OPENRAVE_RAPIDJSON
    bool SupportsJSONCommand(const string& cmd);
    py::object SendJSONCommand(const string& cmd, py::object input, bool releasegil=false, bool lockenv=false);
#endif // OPENRAVE_RAPIDJSON

    virtual py::object GetReadableInterfaces();
    virtual py::object GetReadableInterface(const std::string& xmltag);

    virtual void SetReadableInterface(const std::string& xmltag, py::object oreadable);

    virtual string __repr__() {
        return boost::str(boost::format("RaveCreateInterface(RaveGetEnvironment(%d),InterfaceType.%s,'%s')")%RaveGetEnvironmentId(_pbase->GetEnv())%RaveGetInterfaceName(_pbase->GetInterfaceType())%_pbase->GetXMLId());
    }
    virtual string __str__() {
        return boost::str(boost::format("<%s:%s>")%RaveGetInterfaceName(_pbase->GetInterfaceType())%_pbase->GetXMLId());
    }
    virtual py::object __unicode__() {
        return ConvertStringToUnicode(__str__());
    }
    virtual int __hash__() {
        return static_cast<int>(uintptr_t(_pbase.get()));
    }
    virtual bool __eq__(PyInterfaceBasePtr p) {
        return !!p && _pbase == p->GetInterfaceBase();
    }
    virtual bool __ne__(PyInterfaceBasePtr p) {
        return !p || _pbase != p->GetInterfaceBase();
    }
    virtual InterfaceBasePtr GetInterfaceBase() {
        return _pbase;
    }
};

class PySensorGeometry
{
public:
    virtual ~PySensorGeometry() {
    }
    virtual SensorBase::SensorType GetType()=0;
    virtual SensorBase::SensorGeometryPtr GetGeometry()=0;
};

typedef OPENRAVE_SHARED_PTR<PySensorGeometry> PySensorGeometryPtr;

PySensorGeometryPtr toPySensorGeometry(SensorBase::SensorGeometryPtr);

bool ExtractIkReturn(py::object o, IkReturn& ikfr);
py::object toPyIkReturn(const IkReturn& ret);

py::object GetUserData(UserDataPtr pdata);

EnvironmentBasePtr GetEnvironment(PyEnvironmentBasePtr);
EnvironmentBasePtr GetEnvironment(py::object);
void LockEnvironment(PyEnvironmentBasePtr);
void UnlockEnvironment(PyEnvironmentBasePtr);
int RaveGetEnvironmentId(PyEnvironmentBasePtr pyenv);
PyEnvironmentBasePtr RaveGetEnvironment(int id);

void init_openravepy_collisionchecker();
CollisionCheckerBasePtr GetCollisionChecker(PyCollisionCheckerBasePtr);
PyInterfaceBasePtr toPyCollisionChecker(CollisionCheckerBasePtr, PyEnvironmentBasePtr);
CollisionReportPtr GetCollisionReport(py::object);
CollisionReportPtr GetCollisionReport(PyCollisionReportPtr);
PyCollisionReportPtr toPyCollisionReport(CollisionReportPtr, PyEnvironmentBasePtr);
void UpdateCollisionReport(PyCollisionReportPtr, PyEnvironmentBasePtr);
void UpdateCollisionReport(py::object, PyEnvironmentBasePtr);
void init_openravepy_controller();
ControllerBasePtr GetController(PyControllerBasePtr);
PyInterfaceBasePtr toPyController(ControllerBasePtr, PyEnvironmentBasePtr);
void init_openravepy_iksolver();
IkSolverBasePtr GetIkSolver(py::object);
IkSolverBasePtr GetIkSolver(PyIkSolverBasePtr);
PyInterfaceBasePtr toPyIkSolver(IkSolverBasePtr, PyEnvironmentBasePtr);
py::object toPyIkSolver(IkSolverBasePtr, py::object);
void init_openravepy_kinbody();
KinBodyPtr GetKinBody(py::object);
KinBodyPtr GetKinBody(PyKinBodyPtr);
PyEnvironmentBasePtr GetPyEnvFromPyKinBody(py::object okinbody);
PyInterfaceBasePtr toPyKinBody(KinBodyPtr, PyEnvironmentBasePtr);
py::object toPyKinBody(KinBodyPtr, py::object opyenv);
py::object toPyKinBodyLink(KinBody::LinkPtr plink, PyEnvironmentBasePtr);
py::object toPyKinBodyLink(KinBody::LinkPtr plink, py::object opyenv);
KinBody::LinkPtr GetKinBodyLink(py::object);
KinBody::LinkConstPtr GetKinBodyLinkConst(py::object);
py::object toPyKinBodyJoint(KinBody::JointPtr pjoint, PyEnvironmentBasePtr);
KinBody::JointPtr GetKinBodyJoint(py::object);
std::string reprPyKinBodyJoint(py::object);
std::string strPyKinBodyJoint(py::object);
void init_openravepy_module();
ModuleBasePtr GetModule(PyModuleBasePtr);
PyInterfaceBasePtr toPyModule(ModuleBasePtr, PyEnvironmentBasePtr);
void init_openravepy_physicsengine();
PhysicsEngineBasePtr GetPhysicsEngine(PyPhysicsEngineBasePtr);
PyInterfaceBasePtr toPyPhysicsEngine(PhysicsEngineBasePtr, PyEnvironmentBasePtr);
void init_openravepy_planner();
PlannerBasePtr GetPlanner(PyPlannerBasePtr);
PyInterfaceBasePtr toPyPlanner(PlannerBasePtr, PyEnvironmentBasePtr);
PlannerBase::PlannerParametersPtr GetPlannerParameters(py::object);
PlannerBase::PlannerParametersConstPtr GetPlannerParametersConst(py::object);

py::object toPyPlannerParameters(PlannerBase::PlannerParametersPtr params);
void init_openravepy_robot();
RobotBasePtr GetRobot(py::object);
RobotBasePtr GetRobot(PyRobotBasePtr);
PyInterfaceBasePtr toPyRobot(RobotBasePtr, PyEnvironmentBasePtr);
RobotBase::ManipulatorPtr GetRobotManipulator(py::object);
py::object toPyRobotManipulator(RobotBase::ManipulatorPtr, PyEnvironmentBasePtr);
void init_openravepy_sensor();
SensorBasePtr GetSensor(PySensorBasePtr);
PyInterfaceBasePtr toPySensor(SensorBasePtr, PyEnvironmentBasePtr);
py::object toPySensorData(SensorBasePtr, PyEnvironmentBasePtr);
void init_openravepy_sensorsystem();
SensorSystemBasePtr GetSensorSystem(PySensorSystemBasePtr);
PyInterfaceBasePtr toPySensorSystem(SensorSystemBasePtr, PyEnvironmentBasePtr);
void init_openravepy_spacesampler();
SpaceSamplerBasePtr GetSpaceSampler(PySpaceSamplerBasePtr);
PyInterfaceBasePtr toPySpaceSampler(SpaceSamplerBasePtr, PyEnvironmentBasePtr);
void init_openravepy_trajectory();
TrajectoryBasePtr GetTrajectory(py::object);
TrajectoryBasePtr GetTrajectory(PyTrajectoryBasePtr);
PyInterfaceBasePtr toPyTrajectory(TrajectoryBasePtr, PyEnvironmentBasePtr);
py::object toPyTrajectory(TrajectoryBasePtr, py::object opyenv);
PyEnvironmentBasePtr toPyEnvironment(PyTrajectoryBasePtr);
// input can be class derived from PyInterfaceBase
py::object toPyEnvironment(py::object opyinterface);
PyEnvironmentBasePtr toPyEnvironment(PyKinBodyPtr);
void init_openravepy_viewer();
ViewerBasePtr GetViewer(PyViewerBasePtr);
PyInterfaceBasePtr toPyViewer(ViewerBasePtr, PyEnvironmentBasePtr);

int pyGetIntFromPy(py::object olevel, int defaultvalue);
py::object toPyPlannerStatus(const PlannerStatus&);
    
PyConfigurationSpecificationPtr toPyConfigurationSpecification(const ConfigurationSpecification&);
const ConfigurationSpecification& GetConfigurationSpecification(PyConfigurationSpecificationPtr);

PyCameraIntrinsicsPtr toPyCameraIntrinsics(const geometry::RaveCameraIntrinsics<float>&);
PyCameraIntrinsicsPtr toPyCameraIntrinsics(const geometry::RaveCameraIntrinsics<double>&);

PyLinkPtr toPyLink(KinBody::LinkPtr plink, PyEnvironmentBasePtr pyenv);
PyJointPtr toPyJoint(KinBody::JointPtr pjoint, PyEnvironmentBasePtr pyenv);
PyLinkInfoPtr toPyLinkInfo(const KinBody::LinkInfo& linkinfo);
PyJointInfoPtr toPyJointInfo(const KinBody::JointInfo& jointinfo, PyEnvironmentBasePtr pyenv);
PyManipulatorInfoPtr toPyManipulatorInfo(const RobotBase::ManipulatorInfo& manipulatorinfo);
PyAttachedSensorInfoPtr toPyAttachedSensorInfo(const RobotBase::AttachedSensorInfo& attachedSensorinfo);
PyConnectedBodyInfoPtr toPyConnectedBodyInfo(const RobotBase::ConnectedBodyInfo& connectedBodyInfo, PyEnvironmentBasePtr pyenv);

PyInterfaceBasePtr RaveCreateInterface(PyEnvironmentBasePtr pyenv, InterfaceType type, const std::string& name);
void init_openravepy_global();
void InitPlanningUtils();

}

#endif
