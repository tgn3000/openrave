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
#define NO_IMPORT_ARRAY
#include "openravepy_int.h"

#include <openrave/xmlreaders.h>
#include <openrave/utils.h>
#include "openravepy_configurationspecification.h"
#include "openravepy_kinbody.h"
#include "openravepy_environment.h"
#include "openravepy_collisionreport.h"

namespace openravepy {

using py::object;

PyRay::PyRay(object newpos, object newdir)
{
    r.pos = ExtractVector3(newpos);
    r.dir = ExtractVector3(newdir);
}

object PyRay::dir() {
    return toPyVector3(r.dir);
}
object PyRay::pos() {
    return toPyVector3(r.pos);
}

string PyRay::__repr__() {
    return boost::str(boost::format("<Ray([%.15e,%.15e,%.15e],[%.15e,%.15e,%.15e])>")%r.pos.x%r.pos.y%r.pos.z%r.dir.x%r.dir.y%r.dir.z);
}
string PyRay::__str__() {
    return boost::str(boost::format("<%.15e %.15e %.15e %.15e %.15e %.15e>")%r.pos.x%r.pos.y%r.pos.z%r.dir.x%r.dir.y%r.dir.z);
}
object PyRay::__unicode__() {
    return ConvertStringToUnicode(__str__());
}

class PyXMLReadable
{
public:
    PyXMLReadable(XMLReadablePtr xmlreadable) : _xmlreadable(xmlreadable) {
    }
    virtual ~PyXMLReadable() {
    }
    std::string GetXMLId() const {
        return _xmlreadable->GetXMLId();
    }
    object Serialize(int options=0)
    {
        std::string xmlid;
        OpenRAVE::xmlreaders::StreamXMLWriter writer(xmlid);
        _xmlreadable->Serialize(OpenRAVE::xmlreaders::StreamXMLWriterPtr(&writer,utils::null_deleter()),options);
        std::stringstream ss;
        writer.Serialize(ss);
        return ConvertStringToUnicode(ss.str());
    }

    XMLReadablePtr GetXMLReadable() {
        return _xmlreadable;
    }
protected:
    XMLReadablePtr _xmlreadable;
};

XMLReadablePtr ExtractXMLReadable(object o) {
    if( !IS_PYTHONOBJECT_NONE(o) ) {
        try {
            PyXMLReadablePtr pyreadable = o.cast<PyXMLReadablePtr>();
            if(pyreadable != nullptr) {
                return ((PyXMLReadablePtr)pyreadable)->GetXMLReadable();
            }
        }
        catch(...) {}
    }
    return XMLReadablePtr();
}

object toPyXMLReadable(XMLReadablePtr p) {
    return p ? py::cast(PyXMLReadablePtr(new PyXMLReadable(p))) : object();
}

namespace xmlreaders
{

class PyStaticClass
{
public:
};

PyXMLReadablePtr pyCreateStringXMLReadable(const std::string& xmlid, const std::string& data)
{
    return PyXMLReadablePtr(new PyXMLReadable(XMLReadablePtr(new OpenRAVE::xmlreaders::StringXMLReadable(xmlid, data))));
}

} // end namespace xmlreaders

object toPyGraphHandle(const GraphHandlePtr p)
{
    if( !p ) {
        return object();
    }
    return py::cast(PyGraphHandle(p));
}

object toPyUserData(UserDataPtr p)
{
    if( !p ) {
        return object();
    }
    return py::cast(PyUserData(p));
}

object toPyRay(const RAY& r)
{
    return py::cast(OPENRAVE_SHARED_PTR<PyRay>(new PyRay(r)));
}

RAY ExtractRay(object o)
{
    try {
        OPENRAVE_SHARED_PTR<PyRay> pyray = o.cast<OPENRAVE_SHARED_PTR<PyRay>>();
        if (pyray != nullptr) {
            return pyray->r;
        }
    }
    catch(...) {}
    return RAY();
}

bool ExtractRay(object o, RAY& ray)
{
    try {
        OPENRAVE_SHARED_PTR<PyRay> pyray = o.cast<OPENRAVE_SHARED_PTR<PyRay>>();
        ray = pyray->r;
        return true;
    }
    catch (...) {}
    return false;
}

// class Ray_pickle_suite : public pickle_suite
// {
// public:
//     static boost::python::tuple getinitargs(const PyRay& r)
//     {
//         return py::make_tuple(toPyVector3(r.r.pos),toPyVector3(r.r.dir));
//     }
// };

class PyAABB
{
public:
    PyAABB() {
    }
    PyAABB(object newpos, object newextents) {
        ab.pos = ExtractVector3(newpos);
        ab.extents = ExtractVector3(newextents);
    }
    PyAABB(const AABB& newab) : ab(newab) {
    }

    object extents() {
        return toPyVector3(ab.extents);
    }
    object pos() {
        return toPyVector3(ab.pos);
    }

    py::dict toDict() {
        py::dict d;
        d["pos"] = pos();
        d["extents"] = extents();
        return d;
    }

    virtual string __repr__() {
        return boost::str(boost::format("AABB([%.15e,%.15e,%.15e],[%.15e,%.15e,%.15e])")%ab.pos.x%ab.pos.y%ab.pos.z%ab.extents.x%ab.extents.y%ab.extents.z);
    }
    virtual string __str__() {
        return boost::str(boost::format("<%.15e %.15e %.15e %.15e %.15e %.15e>")%ab.pos.x%ab.pos.y%ab.pos.z%ab.extents.x%ab.extents.y%ab.extents.z);
    }
    virtual object __unicode__() {
        return ConvertStringToUnicode(__str__());
    }

    AABB ab;
};

AABB ExtractAABB(object o)
{
    try {
        OPENRAVE_SHARED_PTR<PyAABB> pyaabb = o.cast<OPENRAVE_SHARED_PTR<PyAABB>>();
        if(pyaabb != nullptr){
            return pyaabb->ab;
        }
    }
    catch(...) {}
    return AABB();
}

object toPyAABB(const AABB& ab)
{
    return py::cast(OPENRAVE_SHARED_PTR<PyAABB>(new PyAABB(ab)));
}

// class AABB_pickle_suite : public pickle_suite
// {
// public:
//     static boost::python::tuple getinitargs(const PyAABB& ab)
//     {
//         return py::make_tuple(toPyVector3(ab.ab.pos),toPyVector3(ab.ab.extents));
//     }
// };

class PyTriMesh
{
public:
    PyTriMesh() {
    }
    PyTriMesh(object vertices, object indices) : vertices(vertices), indices(indices) {
    }
    PyTriMesh(const TriMesh& mesh) {
        npy_intp dims[] = { npy_intp(mesh.vertices.size()), npy_intp(3)};
        PyObject *pyvertices = PyArray_SimpleNew(2,dims, sizeof(dReal)==8 ? PyArray_DOUBLE : PyArray_FLOAT);
        dReal* pvdata = (dReal*)PyArray_DATA(pyvertices);
        FOREACHC(itv, mesh.vertices) {
            *pvdata++ = itv->x;
            *pvdata++ = itv->y;
            *pvdata++ = itv->z;
        }
        vertices = toPyArrayN((dReal*)PyArray_DATA(pyvertices), 3 * mesh.vertices.size());

        dims[0] = mesh.indices.size()/3;
        dims[1] = 3;
        PyObject *pyindices = PyArray_SimpleNew(2,dims, PyArray_INT32);
        int32_t* pidata = reinterpret_cast<int32_t*>PyArray_DATA(pyindices);
        std::memcpy(pidata, mesh.indices.data(), mesh.indices.size() * sizeof(int32_t));
        indices = toPyArrayN(pidata, mesh.indices.size());
    }

    void GetTriMesh(TriMesh& mesh) {
        int numverts = len(vertices);
        mesh.vertices.resize(numverts);

        PyObject *pPyVertices = vertices.ptr();
        if (PyArray_Check(pPyVertices)) {
            if (PyArray_NDIM(pPyVertices) != 2) {
                throw openrave_exception(_("vertices must be a 2D array"), ORE_InvalidArguments);
            }
            if (!PyArray_ISFLOAT(pPyVertices)) {
                throw openrave_exception(_("vertices must be in float"), ORE_InvalidArguments);
            }
            PyArrayObject* pPyVerticesContiguous = PyArray_GETCONTIGUOUS(reinterpret_cast<PyArrayObject*>(pPyVertices));
            AutoPyArrayObjectDereferencer pydecref(pPyVerticesContiguous);

            const size_t typeSize = PyArray_ITEMSIZE(pPyVerticesContiguous);
            const size_t n = PyArray_DIM(pPyVerticesContiguous, 0);
            const size_t nElems = PyArray_DIM(pPyVerticesContiguous, 1);

            if (typeSize == sizeof(float)) {
                const float *vdata = reinterpret_cast<float*>(PyArray_DATA(pPyVerticesContiguous));

                for (size_t i = 0, j = 0; i < n; ++i, j += nElems) {
                    mesh.vertices[i].x = static_cast<dReal>(vdata[j + 0]);
                    mesh.vertices[i].y = static_cast<dReal>(vdata[j + 1]);
                    mesh.vertices[i].z = static_cast<dReal>(vdata[j + 2]);
                }
            } else if (typeSize == sizeof(double)) {
                const double *vdata = reinterpret_cast<double*>(PyArray_DATA(pPyVerticesContiguous));

                for (size_t i = 0, j = 0; i < n; ++i, j += nElems) {
                    mesh.vertices[i].x = static_cast<dReal>(vdata[j + 0]);
                    mesh.vertices[i].y = static_cast<dReal>(vdata[j + 1]);
                    mesh.vertices[i].z = static_cast<dReal>(vdata[j + 2]);
                }
            } else {
                throw openrave_exception(_("Unsupported vertices type"), ORE_InvalidArguments);
            }

        } else {
            for(int i = 0; i < numverts; ++i) {
                object ov = vertices[i];
                mesh.vertices[i].x = ov[0].cast<dReal>();
                mesh.vertices[i].y = ov[1].cast<dReal>();
                mesh.vertices[i].z = ov[2].cast<dReal>();
            }
        }

        const size_t numtris = len(indices);
        mesh.indices.resize(3*numtris);
        PyObject *pPyIndices = indices.ptr();
        if (PyArray_Check(pPyIndices)) {
            if (PyArray_NDIM(pPyIndices) != 2 || PyArray_DIM(pPyIndices, 1) != 3 || !PyArray_ISINTEGER(pPyIndices)) {
                throw openrave_exception(_("indices must be a Nx3 int array"), ORE_InvalidArguments);
            }
            PyArrayObject* pPyIndiciesContiguous = PyArray_GETCONTIGUOUS(reinterpret_cast<PyArrayObject*>(pPyIndices));
            AutoPyArrayObjectDereferencer pydecref(pPyIndiciesContiguous);

            const size_t typeSize = PyArray_ITEMSIZE(pPyIndiciesContiguous);
            const bool signedInt = PyArray_ISSIGNED(pPyIndiciesContiguous);

            if (typeSize == sizeof(int32_t)) {
                if (signedInt) {
                    const int32_t *idata = reinterpret_cast<int32_t*>(PyArray_DATA(pPyIndiciesContiguous));
                    std::memcpy(mesh.indices.data(), idata, numtris * 3 * sizeof(int32_t));
                } else {
                    const uint32_t *idata = reinterpret_cast<uint32_t*>(PyArray_DATA(pPyIndiciesContiguous));
                    for (size_t i = 0; i < 3 * numtris; ++i) {
                        mesh.indices[i] = static_cast<int32_t>(idata[i]);
                    }
                }
            } else if (typeSize == sizeof(int64_t)) {
                if (signedInt) {
                    const int64_t *idata = reinterpret_cast<int64_t*>(PyArray_DATA(pPyIndiciesContiguous));
                    for (size_t i = 0; i < 3 * numtris; ++i) {
                        mesh.indices[i] = static_cast<int32_t>(idata[i]);
                    }
                } else {
                    const uint64_t *idata = reinterpret_cast<uint64_t*>(PyArray_DATA(pPyIndiciesContiguous));
                    for (size_t i = 0; i < 3 * numtris; ++i) {
                        mesh.indices[i] = static_cast<int32_t>(idata[i]);
                    }
                }
            } else if (typeSize == sizeof(uint16_t) && !signedInt) {
                const uint16_t *idata = reinterpret_cast<uint16_t*>(PyArray_DATA(pPyIndiciesContiguous));
                for (size_t i = 0; i < 3 * numtris; ++i) {
                    mesh.indices[i] = static_cast<int32_t>(idata[i]);
                }
            } else {
                throw openrave_exception(_("Unsupported indices type"), ORE_InvalidArguments);
            }

        } else {
            for(size_t i = 0; i < numtris; ++i) {
                object oi = indices[i];
                mesh.indices[3*i+0] = oi[0].cast<int32_t>();
                mesh.indices[3*i+1] = oi[1].cast<int32_t>();
                mesh.indices[3*i+2] = oi[2].cast<int32_t>();
            }
        }
    }

    string __str__() {
        return boost::str(boost::format("<trimesh: verts %d, tris=%d>")%len(vertices)%len(indices));
    }
    object __unicode__() {
        return ConvertStringToUnicode(__str__());
    }

    object vertices,indices;
};

bool ExtractTriMesh(object o, TriMesh& mesh)
{
    try {
        OPENRAVE_SHARED_PTR<PyTriMesh> pytrimesh = o.cast<OPENRAVE_SHARED_PTR<PyTriMesh>>();
        if(pytrimesh != nullptr){
            pytrimesh->GetTriMesh(mesh);
            return true;
        }
    }
    catch(...) {}
    return false;
}

object toPyTriMesh(const TriMesh& mesh)
{
    return py::cast(OPENRAVE_SHARED_PTR<PyTriMesh>(new PyTriMesh(mesh)));
}

// class TriMesh_pickle_suite : public pickle_suite
// {
// public:
//     static boost::python::tuple getinitargs(const PyTriMesh& r)
//     {
//         return py::make_tuple(r.vertices,r.indices);
//     }
// };

// class ConfigurationSpecification_pickle_suite : public pickle_suite
// {
// public:
//     static boost::python::tuple getinitargs(const PyConfigurationSpecification& pyspec)
//     {
//         std::stringstream ss;
//         ss << pyspec._spec;
//         return py::make_tuple(ss.str());
//     }
// };

PyConfigurationSpecificationPtr toPyConfigurationSpecification(const ConfigurationSpecification &spec)
{
    return PyConfigurationSpecificationPtr(new PyConfigurationSpecification(spec));
}

const ConfigurationSpecification& GetConfigurationSpecification(PyConfigurationSpecificationPtr p)
{
    return p->_spec;
}

// struct spec_from_group
// {
//     spec_from_group()
//     {
//         boost::python::converter::registry::push_back(&convertible, &construct, boost::python::type_id<PyConfigurationSpecificationPtr>());
//     }

//     static void* convertible(PyObject* obj)
//     {
//         return obj == Py_None ||  boost::python::extract<ConfigurationSpecification::Group>(obj).check() ? obj : NULL;
//     }

//     static void construct(PyObject* obj, boost::python::converter::rvalue_from_python_stage1_data* data)
//     {
//         ConfigurationSpecification::Group g = (ConfigurationSpecification::Group)boost::python::extract<ConfigurationSpecification::Group>(obj);
//         void* storage = ((boost::python::converter::rvalue_from_python_storage<PyConfigurationSpecificationPtr>*)data)->storage.bytes;
//         new (storage) PyConfigurationSpecificationPtr(new PyConfigurationSpecification(g));
//         data->convertible = storage;
//     }
// };

PyConfigurationSpecificationPtr pyRaveGetAffineConfigurationSpecification(int affinedofs,PyKinBodyPtr pybody=PyKinBodyPtr(), const std::string& interpolation="")
{
    return openravepy::toPyConfigurationSpecification(RaveGetAffineConfigurationSpecification(affinedofs,openravepy::GetKinBody(pybody), interpolation));
}

object pyRaveGetAffineDOFValuesFromTransform(object otransform, int affinedofs, object oActvAffineRotationAxis=object())
{
    Vector vActvAffineRotationAxis(0,0,1);
    if( !IS_PYTHONOBJECT_NONE(oActvAffineRotationAxis) ) {
        vActvAffineRotationAxis = ExtractVector3(oActvAffineRotationAxis);
    }
    std::vector<dReal> values(RaveGetAffineDOF(affinedofs));
    RaveGetAffineDOFValuesFromTransform(values.begin(),ExtractTransform(otransform), affinedofs, vActvAffineRotationAxis);
    return toPyArray(values);
}

std::string openravepyCompilerVersion()
{
    stringstream ss;
#if defined(_MSC_VER)
    ss << "msvc " << _MSC_VER;
#elif defined(__GNUC__)
    ss << "gcc " << __GNUC__ << "." << __GNUC_MINOR__ << "." << __GNUC_PATCHLEVEL__;
#elif defined(__MINGW32_VERSION)
    ss << "mingw " << __MINGW32_VERSION;
#endif
    return ss.str();
}

void raveLog(const string &s, int level)
{
    if( s.size() > 0 ) {
        RavePrintfA(s,level);
    }
}

void raveLogFatal(const string &s)
{
    raveLog(s,Level_Fatal);
}
void raveLogError(const string &s)
{
    raveLog(s,Level_Error);
}
void raveLogWarn(const string &s)
{
    raveLog(s,Level_Warn);
}
void raveLogInfo(const string &s)
{
    raveLog(s,Level_Info);
}
void raveLogDebug(const string &s)
{
    raveLog(s,Level_Debug);
}
void raveLogVerbose(const string &s)
{
    raveLog(s,Level_Verbose);
}

int pyGetIntFromPy(object olevel, int defaultvalue)
{
    int level = defaultvalue;
    if( !IS_PYTHONOBJECT_NONE(olevel) ) {
        // some version of boost python return true for extract::check, even through the actual conversion will throw an OverflowError
        // therefore check for conversion compatibility starting at the longest signed integer
        try {
            int64_t levelint64 = olevel.cast<int64_t>();
            level = static_cast<int>((int64_t)levelint64);
        }
        catch(...) {
            try {
                uint64_t leveluint64 = olevel.cast<uint64_t>();
                level = static_cast<int>((uint64_t)leveluint64);
            }
            catch(...) {
                try {
                    uint32_t leveluint32 = olevel.cast<uint32_t>();
                    level = static_cast<int>((uint32_t)leveluint32);
                }
                catch(...) {
                    try {
                        int levelint32 = olevel.cast<int>();
                        level = levelint32;
                    }
                    catch(...) {
                        RAVELOG_WARN("failed to extract int\n");
                    }
                }
            }
        }
    }
    return level;
}

void pyRaveSetDebugLevel(object olevel)
{
    OpenRAVE::RaveSetDebugLevel(pyGetIntFromPy(olevel, Level_Info));
}

int pyRaveInitialize(bool bLoadAllPlugins=true, object olevel=object())
{

    return OpenRAVE::RaveInitialize(bLoadAllPlugins,pyGetIntFromPy(olevel, Level_Info));
}

void pyRaveSetDataAccess(object oaccess)
{
    OpenRAVE::RaveSetDataAccess(pyGetIntFromPy(oaccess, Level_Info));
}

// return None if nothing found
object pyRaveInvertFileLookup(const std::string& filename)
{
    std::string newfilename;
    if( OpenRAVE::RaveInvertFileLookup(newfilename, filename) ) {
        return ConvertStringToUnicode(newfilename);
    }
    return object();
}

object RaveGetPluginInfo()
{
    py::list plugins;
    std::list< std::pair<std::string, PLUGININFO> > listplugins;
    OpenRAVE::RaveGetPluginInfo(listplugins);
    FOREACH(itplugin, listplugins) {
        plugins.append(py::make_tuple(itplugin->first, py::cast(OPENRAVE_SHARED_PTR<PyPluginInfo>(new PyPluginInfo(itplugin->second)))));
    }
    return std::move(plugins);
}

object RaveGetLoadedInterfaces()
{
    std::map<InterfaceType, std::vector<std::string> > interfacenames;
    OpenRAVE::RaveGetLoadedInterfaces(interfacenames);
    py::dict ointerfacenames;
    FOREACHC(it, interfacenames) {
        py::list names;
        FOREACHC(itname,it->second) {
            names.append(*itname);
        }
        ointerfacenames[it->first] = names;
    }
    return std::move(ointerfacenames);
}

PyInterfaceBasePtr pyRaveClone(PyInterfaceBasePtr pyreference, int cloningoptions, PyEnvironmentBasePtr pyenv=PyEnvironmentBasePtr())
{
    if( !pyenv ) {
        pyenv = pyreference->GetEnv();
    }
    EnvironmentBasePtr penv = openravepy::GetEnvironment(pyenv);
    InterfaceBasePtr pclone = OpenRAVE::RaveClone<InterfaceBase>(pyreference->GetInterfaceBase(), cloningoptions, penv);
    switch(pclone->GetInterfaceType()) {
    case PT_Planner: return toPyPlanner(RaveInterfaceCast<PlannerBase>(pclone), pyenv);
    case PT_Robot: return toPyRobot(RaveInterfaceCast<RobotBase>(pclone), pyenv);
    case PT_SensorSystem: return toPySensorSystem(RaveInterfaceCast<SensorSystemBase>(pclone), pyenv);
    case PT_Controller: return toPyController(RaveInterfaceCast<ControllerBase>(pclone), pyenv);
    case PT_Module: return toPyModule(RaveInterfaceCast<ModuleBase>(pclone), pyenv);
    case PT_InverseKinematicsSolver: return toPyIkSolver(RaveInterfaceCast<IkSolverBase>(pclone), pyenv);
    case PT_KinBody: return toPyKinBody(RaveInterfaceCast<KinBody>(pclone), pyenv);
    case PT_PhysicsEngine: return toPyPhysicsEngine(RaveInterfaceCast<PhysicsEngineBase>(pclone), pyenv);
    case PT_Sensor: return toPySensor(RaveInterfaceCast<SensorBase>(pclone), pyenv);
    case PT_CollisionChecker: return toPyCollisionChecker(RaveInterfaceCast<CollisionCheckerBase>(pclone), pyenv);
    case PT_Trajectory: return toPyTrajectory(RaveInterfaceCast<TrajectoryBase>(pclone), pyenv);
    case PT_Viewer: return toPyViewer(RaveInterfaceCast<ViewerBase>(pclone), pyenv);
    case PT_SpaceSampler: return toPySpaceSampler(RaveInterfaceCast<SpaceSamplerBase>(pclone), pyenv);
    }
    throw openrave_exception(_("invalid interface type"),ORE_InvalidArguments);
}

object quatFromAxisAngle1(object oaxis)
{
    return toPyVector4(quatFromAxisAngle(ExtractVector3(oaxis)));
}

object quatFromAxisAngle2(object oaxis, dReal angle)
{
    return toPyVector4(quatFromAxisAngle(ExtractVector3(oaxis),angle));
}

object quatFromRotationMatrix(object R)
{
    TransformMatrix t;
    t.rotfrommat(R[0][0].cast<dReal>(), R[0][1].cast<dReal>(), R[0][2].cast<dReal>(),
                 R[1][0].cast<dReal>(), R[1][1].cast<dReal>(), R[1][2].cast<dReal>(),
                 R[2][0].cast<dReal>(), R[2][1].cast<dReal>(), R[2][2].cast<dReal>());
    return toPyVector4(quatFromMatrix(t));
}

object InterpolateQuatSlerp(object q1, object q2, dReal t, bool forceshortarc=true)
{
    return toPyVector4(InterpolateQuatSlerp(ExtractVector4(q1),ExtractVector4(q2),t, forceshortarc));
}

object InterpolateQuatSquad(object q0, object q1, object q2, object q3, dReal t, bool forceshortarc=true)
{
    return toPyVector4(InterpolateQuatSquad(ExtractVector4(q0),ExtractVector4(q1),ExtractVector4(q2),ExtractVector4(q3), t, forceshortarc));
}

object axisAngleFromRotationMatrix(object R)
{
    TransformMatrix t;
    t.rotfrommat(R[0][0].cast<dReal>(), R[0][1].cast<dReal>(), R[0][2].cast<dReal>(),
                 R[1][0].cast<dReal>(), R[1][1].cast<dReal>(), R[1][2].cast<dReal>(),
                 R[2][0].cast<dReal>(), R[2][1].cast<dReal>(), R[2][2].cast<dReal>());
    return toPyVector3(axisAngleFromMatrix(t));
}

object axisAngleFromQuat(object oquat)
{
    return toPyVector3(axisAngleFromQuat(ExtractVector4(oquat)));
}

object rotationMatrixFromQuat(object oquat)
{
    return toPyArrayRotation(matrixFromQuat(ExtractVector4(oquat)));
}

object rotationMatrixFromQArray(object qarray)
{
    py::list orots;
    int N = len(qarray);
    for(int i = 0; i < N; ++i) {
        orots.append(rotationMatrixFromQuat(qarray[i]));
    }
    return std::move(orots);
}

object matrixFromQuat(object oquat)
{
    return toPyArray(matrixFromQuat(ExtractVector4(oquat)));
}

object rotationMatrixFromAxisAngle1(object oaxis)
{
    return toPyArrayRotation(matrixFromAxisAngle(ExtractVector3(oaxis)));
}

object rotationMatrixFromAxisAngle2(object oaxis, dReal angle)
{
    return toPyArrayRotation(matrixFromAxisAngle(ExtractVector3(oaxis),angle));
}

object matrixFromAxisAngle1(object oaxis)
{
    return toPyArray(matrixFromAxisAngle(ExtractVector3(oaxis)));
}

object matrixFromAxisAngle2(object oaxis, dReal angle)
{
    return toPyArray(matrixFromAxisAngle(ExtractVector3(oaxis),angle));
}

object matrixFromPose(object opose)
{
    return toPyArray(TransformMatrix(ExtractTransformType<dReal>(opose)));
}

object matrixFromPoses(object oposes)
{
    py::list omatrices;
    int N = len(oposes);
    for(int i = 0; i < N; ++i) {
        omatrices.append(matrixFromPose(oposes[i]));
    }
    return std::move(omatrices);
}

object poseFromMatrix(object o)
{
    TransformMatrix t;
    for(int i = 0; i < 3; ++i) {
        t.m[4*i+0] = o[i][0].cast<dReal>();
        t.m[4*i+1] = o[i][1].cast<dReal>();
        t.m[4*i+2] = o[i][2].cast<dReal>();
        t.trans[i] = o[i][3].cast<dReal>();
    }
    return toPyArray(Transform(t));
}

object poseFromMatrices(object otransforms)
{
    int N = len(otransforms);
    if( N == 0 ) {
        return py::array_t<dReal>({1, 0}, nullptr);
    }
    npy_intp dims[] = { N,7};
    PyObject *pyvalues = PyArray_SimpleNew(2,dims, sizeof(dReal)==8 ? PyArray_DOUBLE : PyArray_FLOAT);
    dReal* pvalues = (dReal*)PyArray_DATA(pyvalues);
    TransformMatrix tm;
    for(int j = 0; j < N; ++j) {
        object o = otransforms[j];
        for(int i = 0; i < 3; ++i) {
            tm.m[4*i+0] = o[i][0].cast<dReal>();
            tm.m[4*i+1] = o[i][1].cast<dReal>();
            tm.m[4*i+2] = o[i][2].cast<dReal>();
            tm.trans[i] = o[i][3].cast<dReal>();
        }
        Transform tpose(tm);
        pvalues[0] = tpose.rot.x; pvalues[1] = tpose.rot.y; pvalues[2] = tpose.rot.z; pvalues[3] = tpose.rot.w;
        pvalues[4] = tpose.trans.x; pvalues[5] = tpose.trans.y; pvalues[6] = tpose.trans.z;
        pvalues += 7;
    }
    return toPyArrayN((dReal*)PyArray_DATA(pyvalues), 7 * N);
}

object InvertPoses(object o)
{
    int N = len(o);
    if( N == 0 ) {
        return py::array_t<dReal>({1, 0}, nullptr);
    }
    npy_intp dims[] = { N,7};
    PyObject *pytrans = PyArray_SimpleNew(2,dims, sizeof(dReal)==8 ? PyArray_DOUBLE : PyArray_FLOAT);
    dReal* ptrans = (dReal*)PyArray_DATA(pytrans);
    for(int i = 0; i < N; ++i, ptrans += 7) {
        object oinputtrans = o[i];
        Transform t = Transform(Vector(oinputtrans[0].cast<dReal>(),oinputtrans[1].cast<dReal>(),oinputtrans[2].cast<dReal>(),oinputtrans[3].cast<dReal>()),
                                Vector(oinputtrans[4].cast<dReal>(),oinputtrans[5].cast<dReal>(),oinputtrans[6].cast<dReal>())).inverse();
        ptrans[0] = t.rot.x; ptrans[1] = t.rot.y; ptrans[2] = t.rot.z; ptrans[3] = t.rot.w;
        ptrans[4] = t.trans.x; ptrans[5] = t.trans.y; ptrans[6] = t.trans.z;
    }
    return toPyArrayN((dReal*)PyArray_DATA(pytrans), 7 * N);
}

object InvertPose(object opose)
{
    Transform t = ExtractTransformType<dReal>(opose);
    return toPyArray(t.inverse());
}

object quatRotateDirection(object source, object target)
{
    return toPyVector4(quatRotateDirection(ExtractVector3(source), ExtractVector3(target)));
}

object ExtractAxisFromQuat(object oquat, int iaxis)
{
    return toPyVector3(ExtractAxisFromQuat(ExtractVector4(oquat), iaxis));
}

object normalizeAxisRotation(object axis, object quat)
{
    std::pair<dReal, Vector > res = normalizeAxisRotation(ExtractVector3(axis), ExtractVector4(quat));
    return py::make_tuple(res.first,toPyVector4(res.second));
}

object MultiplyQuat(object oquat1, object oquat2)
{
    return toPyVector4(OpenRAVE::geometry::quatMultiply(ExtractVector4(oquat1),ExtractVector4(oquat2)));
}

object InvertQuat(object oquat)
{
    return toPyVector4(OpenRAVE::geometry::quatInverse(ExtractVector4(oquat)));
}

object MultiplyPose(object opose1, object opose2)
{
    return toPyArray(ExtractTransformType<dReal>(opose1)*ExtractTransformType<dReal>(opose2));
}

object poseTransformPoint(object opose, object opoint)
{
    Transform t = ExtractTransformType<dReal>(opose);
    Vector newpoint = t*ExtractVector3(opoint);
    return toPyVector3(newpoint);
}

object poseTransformPoints(object opose, object opoints)
{
    Transform t = ExtractTransformType<dReal>(opose);
    int N = len(opoints);
    npy_intp dims[] = { N,3};
    PyObject *pytrans = PyArray_SimpleNew(2,dims, sizeof(dReal)==8 ? PyArray_DOUBLE : PyArray_FLOAT);
    dReal* ptrans = (dReal*)PyArray_DATA(pytrans);
    for(int i = 0; i < N; ++i, ptrans += 3) {
        Vector newpoint = t*ExtractVector3(opoints[i]);
        ptrans[0] = newpoint.x; ptrans[1] = newpoint.y; ptrans[2] = newpoint.z;
    }
    return toPyArrayN((dReal*)PyArray_DATA(pytrans), 3 * N);
}

object TransformLookat(object olookat, object ocamerapos, object ocameraup)
{
    return toPyArray(transformLookat(ExtractVector3(olookat),ExtractVector3(ocamerapos),ExtractVector3(ocameraup)));
}

dReal ComputePoseDistSqr(object opose0, object opose1, dReal quatweight=1.0)
{
    Transform t0 = ExtractTransformType<dReal>(opose0);
    Transform t1 = ExtractTransformType<dReal>(opose1);
    dReal e1 = (t0.rot-t1.rot).lengthsqr4();
    dReal e2 = (t0.rot+t1.rot).lengthsqr4();
    dReal e = e1 < e2 ? e1 : e2;
    return (t0.trans-t1.trans).lengthsqr3() + quatweight*e;
}

string matrixSerialization(object o)
{
    stringstream ss;
    ss << std::setprecision(std::numeric_limits<dReal>::digits10+1);     /// have to do this or otherwise precision gets lost
    ss << ExtractTransformMatrix(o);
    return ss.str();
}

string poseSerialization(object o)
{
    stringstream ss;
    ss << std::setprecision(std::numeric_limits<dReal>::digits10+1);     /// have to do this or otherwise precision gets lost
    ss << ExtractTransform(o);
    return ss.str();
}

void init_openravepy_global(py::module& m)
{
    py::enum_<OpenRAVEErrorCode>(m, "ErrorCode" DOXY_ENUM(OpenRAVEErrorCode))
    .value("Failed",ORE_Failed)
    .value("InvalidArguments",ORE_InvalidArguments)
    .value("EnvironmentNotLocked",ORE_EnvironmentNotLocked)
    .value("CommandNotSupported",ORE_CommandNotSupported)
    .value("Assert",ORE_Assert)
    .value("InvalidPlugin",ORE_InvalidPlugin)
    .value("InvalidInterfaceHash",ORE_InvalidInterfaceHash)
    .value("NotImplemented",ORE_NotImplemented)
    .value("InconsistentConstraints",ORE_InconsistentConstraints)
    .value("NotInitialized",ORE_NotInitialized)
    .value("InvalidState",ORE_InvalidState)
    .value("Timeout",ORE_Timeout)
    ;
    py::enum_<DebugLevel>(m, "DebugLevel" DOXY_ENUM(DebugLevel))
    .value("Fatal",Level_Fatal)
    .value("Error",Level_Error)
    .value("Warn",Level_Warn)
    .value("Info",Level_Info)
    .value("Debug",Level_Debug)
    .value("Verbose",Level_Verbose)
    .value("VerifyPlans",Level_VerifyPlans)
    ;
    py::enum_<SerializationOptions>(m, "SerializationOptions" DOXY_ENUM(SerializationOptions))
    .value("Kinematics",SO_Kinematics)
    .value("Dynamics",SO_Dynamics)
    .value("BodyState",SO_BodyState)
    .value("NamesAndFiles",SO_NamesAndFiles)
    .value("RobotManipulators",SO_RobotManipulators)
    .value("RobotSensors",SO_RobotSensors)
    .value("Geometry",SO_Geometry)
    .value("InverseKinematics",SO_InverseKinematics)
    .value("JointLimits",SO_JointLimits)
    ;
    py::enum_<InterfaceType>(m, "InterfaceType" DOXY_ENUM(InterfaceType))
    .value(RaveGetInterfaceName(PT_Planner).c_str(),PT_Planner)
    .value(RaveGetInterfaceName(PT_Robot).c_str(),PT_Robot)
    .value(RaveGetInterfaceName(PT_SensorSystem).c_str(),PT_SensorSystem)
    .value(RaveGetInterfaceName(PT_Controller).c_str(),PT_Controller)
    .value("probleminstance",PT_Module)
    .value(RaveGetInterfaceName(PT_Module).c_str(),PT_Module)
    .value(RaveGetInterfaceName(PT_IkSolver).c_str(),PT_IkSolver)
    .value(RaveGetInterfaceName(PT_KinBody).c_str(),PT_KinBody)
    .value(RaveGetInterfaceName(PT_PhysicsEngine).c_str(),PT_PhysicsEngine)
    .value(RaveGetInterfaceName(PT_Sensor).c_str(),PT_Sensor)
    .value(RaveGetInterfaceName(PT_CollisionChecker).c_str(),PT_CollisionChecker)
    .value(RaveGetInterfaceName(PT_Trajectory).c_str(),PT_Trajectory)
    .value(RaveGetInterfaceName(PT_Viewer).c_str(),PT_Viewer)
    .value(RaveGetInterfaceName(PT_SpaceSampler).c_str(),PT_SpaceSampler)
    ;
    py::enum_<CloningOptions>(m, "CloningOptions" DOXY_ENUM(CloningOptions))
    .value("Bodies",Clone_Bodies)
    .value("Viewer",Clone_Viewer)
    .value("Simulation",Clone_Simulation)
    .value("RealControllers",Clone_RealControllers)
    .value("Sensors",Clone_Sensors)
    .value("Modules",Clone_Modules)
    ;
    py::enum_<PhysicsEngineOptions>(m, "PhysicsEngineOptions" DOXY_ENUM(PhysicsEngineOptions))
    .value("SelfCollisions",PEO_SelfCollisions)
    ;

    py::enum_<IntervalType>(m, "Interval" DOXY_ENUM(IntervalType))
    .value("Open",IT_Open)
    .value("OpenStart",IT_OpenStart)
    .value("OpenEnd",IT_OpenEnd)
    .value("Closed",IT_Closed)
    ;
    py::enum_<SampleDataType>(m, "SampleDataType" DOXY_ENUM(SampleDataType))
    .value("Real",SDT_Real)
    .value("Uint32",SDT_Uint32)
    ;

    py::class_<UserData, UserDataPtr >(m, "UserData", DOXY_CLASS(UserData))
    ;

    py::class_< OPENRAVE_SHARED_PTR< void > >(m, "VoidPointer", "Holds auto-managed resources, deleting it releases its shared data.");

    py::class_<PyGraphHandle, OPENRAVE_SHARED_PTR<PyGraphHandle> >(m, "GraphHandle", DOXY_CLASS(GraphHandle))
    .def("SetTransform",&PyGraphHandle::SetTransform,DOXY_FN(GraphHandle,SetTransform))
    .def("SetShow",&PyGraphHandle::SetShow,DOXY_FN(GraphHandle,SetShow))
    .def("Close",&PyGraphHandle::Close,DOXY_FN(GraphHandle,Close))
    ;

    py::class_<PyUserData, OPENRAVE_SHARED_PTR<PyUserData> >(m, "UserData", DOXY_CLASS(UserData))
    .def("close",&PyUserData::Close,"deprecated")
    .def("Close",&PyUserData::Close,"force releasing the user handle point.")
    ;
    py::class_<PySerializableData, OPENRAVE_SHARED_PTR<PySerializableData>, PyUserData>(m, "SerializableData", DOXY_CLASS(SerializableData))
    .def(py::init<std::string>())
    .def("Close",&PySerializableData::Close,DOXY_FN(SerializableData,Close))
    .def("Serialize",&PySerializableData::Serialize, py::arg("options"), DOXY_FN(SerializableData, Serialize))
    .def("Deserialize",&PySerializableData::Deserialize,py::arg("data"), DOXY_FN(SerializableData, Deserialize))
    ;

    py::class_<PyRay>(m, "Ray", DOXY_CLASS(geometry::ray))
    .def(py::init<object, object>())
    .def("dir",&PyRay::dir)
    .def("pos",&PyRay::pos)
    .def("__str__",&PyRay::__str__)
    .def("__unicode__",&PyRay::__unicode__)
    .def("__repr__",&PyRay::__repr__)
    // .def_pickle(Ray_pickle_suite())
    ;
    py::class_<PyAABB>(m, "AABB", DOXY_CLASS(geometry::aabb))
    .def(py::init<object, object>())
    .def("extents",&PyAABB::extents)
    .def("pos",&PyAABB::pos)
    .def("__str__",&PyAABB::__str__)
    .def("__unicode__",&PyAABB::__unicode__)
    .def("__repr__",&PyAABB::__repr__)
    .def("toDict", &PyAABB::toDict)
    // .def_pickle(AABB_pickle_suite())
    ;
    py::class_<PyTriMesh>(m, "TriMesh", DOXY_CLASS(TriMesh))
    .def(py::init<object, object>())
    .def_readwrite("vertices",&PyTriMesh::vertices)
    .def_readwrite("indices",&PyTriMesh::indices)
    .def("__str__",&PyTriMesh::__str__)
    .def("__unicode__",&PyTriMesh::__unicode__)
    // .def_pickle(TriMesh_pickle_suite())
    ;
    // py::class_<InterfaceBase>(m, "InterfaceBase", DOXY_CLASS(InterfaceBase))
    // ;

    py::class_<PyXMLReadable>(m, "XMLReadable", DOXY_CLASS(XMLReadable))
    .def(py::init<XMLReadablePtr>())
    .def("GetXMLId", &PyXMLReadable::GetXMLId, DOXY_FN(XMLReadable, GetXMLId))
    .def("Serialize", &PyXMLReadable::Serialize, py::arg("options"), DOXY_FN(XMLReadable, Serialize))
    ;

    py::class_<PyPluginInfo, OPENRAVE_SHARED_PTR<PyPluginInfo> >(m, "PluginInfo", DOXY_CLASS(PLUGININFO))
    .def_readonly("interfacenames",&PyPluginInfo::interfacenames)
    .def_readonly("version",&PyPluginInfo::version)
    ;

    {
        int (PyConfigurationSpecification::*addgroup1)(const std::string&, int, const std::string&) = &PyConfigurationSpecification::AddGroup;
        int (PyConfigurationSpecification::*addgroup2)(const ConfigurationSpecification::Group&) = &PyConfigurationSpecification::AddGroup;

        object configurationspecification = py::class_<PyConfigurationSpecification, OPENRAVE_SHARED_PTR<PyConfigurationSpecification>>(m, "ConfigurationSpecification",DOXY_CLASS(ConfigurationSpecification))
                                           .def(py::init<PyConfigurationSpecificationPtr>())
                                           .def(py::init<const ConfigurationSpecification::Group&>())
                                           .def(py::init<const std::string&>())
                                           .def("GetGroupFromName",&PyConfigurationSpecification::GetGroupFromName, DOXY_FN(ConfigurationSpecification,GetGroupFromName))
                                           .def("FindCompatibleGroup",&PyConfigurationSpecification::FindCompatibleGroup, DOXY_FN(ConfigurationSpecification,FindCompatibleGroup))
                                           .def("FindTimeDerivativeGroup",&PyConfigurationSpecification::FindTimeDerivativeGroup, DOXY_FN(ConfigurationSpecification,FindTimeDerivativeGroup))
                                           .def("GetDOF",&PyConfigurationSpecification::GetDOF,DOXY_FN(ConfigurationSpecification,GetDOF))
                                           .def("IsValid",&PyConfigurationSpecification::IsValid,DOXY_FN(ConfigurationSpecification,IsValid))
                                           .def("ResetGroupOffsets",&PyConfigurationSpecification::ResetGroupOffsets,DOXY_FN(ConfigurationSpecification,ResetGroupOffsets))
                                           .def("AddVelocityGroups",&PyConfigurationSpecification::AddVelocityGroups,py::arg("adddeltatime"), DOXY_FN(ConfigurationSpecification,AddVelocityGroups))
                                           .def("AddDerivativeGroups",&PyConfigurationSpecification::AddDerivativeGroups,py::arg("deriv"), py::arg("adddeltatime"), DOXY_FN(ConfigurationSpecification,AddDerivativeGroups))
                                           .def("AddDeltaTimeGroup",&PyConfigurationSpecification::AddDeltaTimeGroup,DOXY_FN(ConfigurationSpecification,AddDeltaTimeGroup))
                                           .def("RemoveGroups", &PyConfigurationSpecification::RemoveGroups, py::arg("groupname"), py::arg("exactmatch"), DOXY_FN(ConfigurationSpecification, RemoveGroups))
                                           .def("AddGroup",addgroup1,py::arg("name"), py::arg("dof"), py::arg("interpolation"), DOXY_FN(ConfigurationSpecification,AddGroup "const std::string; int; const std::string"))
                                           .def("AddGroup",addgroup2,py::arg("group"), DOXY_FN(ConfigurationSpecification,AddGroup "const"))
                                           .def("ConvertToVelocitySpecification",&PyConfigurationSpecification::ConvertToVelocitySpecification,DOXY_FN(ConfigurationSpecification,ConvertToVelocitySpecification))
                                           .def("ConvertToDerivativeSpecification",&PyConfigurationSpecification::ConvertToDerivativeSpecification, py::arg("timederivative"), DOXY_FN(ConfigurationSpecification, ConvertToDerivativeSpecification))
                                           .def("GetTimeDerivativeSpecification",&PyConfigurationSpecification::GetTimeDerivativeSpecification,DOXY_FN(ConfigurationSpecification,GetTimeDerivativeSpecification))

                                           .def("ExtractTransform",&PyConfigurationSpecification::ExtractTransform, py::arg("transform"), py::arg("data"), py::arg("body"), py::arg("timederivative"),DOXY_FN(ConfigurationSpecification,ExtractTransform))
                                           .def("ExtractAffineValues",&PyConfigurationSpecification::ExtractAffineValues, py::arg("data"), py::arg("body"), py::arg("affinedofs"), py::arg("timederivative"),DOXY_FN(ConfigurationSpecification,ExtractAffineValues))
                                           .def("ExtractIkParameterization",&PyConfigurationSpecification::ExtractIkParameterization, py::arg("data"), py::arg("timederivative"), py::arg("robotname"), py::arg("manipulatorname"),DOXY_FN(ConfigurationSpecification,ExtractIkParameterization))
                                           .def("ExtractJointValues",&PyConfigurationSpecification::ExtractJointValues, py::arg("data"), py::arg("body"), py::arg("indices"), py::arg("timederivative"),DOXY_FN(ConfigurationSpecification,ExtractJointValues))
                                           .def("ExtractDeltaTime",&PyConfigurationSpecification::ExtractDeltaTime,py::arg("data"),DOXY_FN(ConfigurationSpecification,ExtractDeltaTime))
                                           .def("InsertDeltaTime",&PyConfigurationSpecification::InsertDeltaTime,py::arg("data"), py::arg("deltatime"),DOXY_FN(ConfigurationSpecification,InsertDeltaTime))
                                           .def("InsertJointValues",&PyConfigurationSpecification::InsertJointValues,py::arg("data"), py::arg("values"), py::arg("body"), py::arg("indices"), py::arg("timederivative"),DOXY_FN(ConfigurationSpecification,InsertJointValues))
                                           .def("ExtractUsedBodies", &PyConfigurationSpecification::ExtractUsedBodies, py::arg("env"), DOXY_FN(ConfigurationSpecification, ExtractUsedBodies))
                                           .def("ExtractUsedIndices", &PyConfigurationSpecification::ExtractUsedIndices, py::arg("env"), DOXY_FN(ConfigurationSpecification, ExtractUsedIndices))
                                           .def("ConvertData", &PyConfigurationSpecification::ConvertData, py::arg("targetspec"), py::arg("sourcedata"), py::arg("numpoints"), py::arg("env"), py::arg("filluninitialized"), DOXY_FN(ConfigurationSpecification, ConvertData))
                                           .def("ConvertDataFromPrevious", &PyConfigurationSpecification::ConvertDataFromPrevious, py::arg("targetdata"), py::arg("targetspec"), py::arg("sourcedata"), py::arg("numpoints"), py::arg("env"), DOXY_FN(ConfigurationSpecification, ConvertData))
                                           .def("GetGroups", &PyConfigurationSpecification::GetGroups, /*py::arg("env"), */"returns a list of the groups")
                                           .def("__eq__",&PyConfigurationSpecification::__eq__)
                                           .def("__ne__",&PyConfigurationSpecification::__ne__)
                                           .def("__add__",&PyConfigurationSpecification::__add__)
                                           .def("__iadd__",&PyConfigurationSpecification::__iadd__)
                                           // .def_pickle(ConfigurationSpecification_pickle_suite())
                                           .def("__str__",&PyConfigurationSpecification::__str__)
                                           .def("__unicode__",&PyConfigurationSpecification::__unicode__)
                                           .def("__repr__",&PyConfigurationSpecification::__repr__)
        ;

        {
            object group = py::class_<ConfigurationSpecification::Group>(m, "Group",DOXY_CLASS(ConfigurationSpecification::Group))
                          .def_readwrite("name",&ConfigurationSpecification::Group::name)
                          .def_readwrite("interpolation",&ConfigurationSpecification::Group::interpolation)
                          .def_readwrite("offset",&ConfigurationSpecification::Group::offset)
                          .def_readwrite("dof",&ConfigurationSpecification::Group::dof)
            ;
        }
    }

    // openravepy::spec_from_group();

    {
        object scope_xmlreaders = py::class_<xmlreaders::PyStaticClass>(m, "xmlreaders")
                                 .def("CreateStringXMLReadable", xmlreaders::pyCreateStringXMLReadable/*, py::arg("xmlid"), py::arg("data")*/)
        ;
    }

    m.def("RaveSetDebugLevel",openravepy::pyRaveSetDebugLevel,py::arg("level"), DOXY_FN1(RaveSetDebugLevel));
    m.def("RaveGetDebugLevel",OpenRAVE::RaveGetDebugLevel,DOXY_FN1(RaveGetDebugLevel));
    m.def("RaveSetDataAccess",openravepy::pyRaveSetDataAccess,py::arg("accessoptions"), DOXY_FN1(RaveSetDataAccess));
    m.def("RaveGetDataAccess",OpenRAVE::RaveGetDataAccess, DOXY_FN1(RaveGetDataAccess));
    m.def("RaveGetDefaultViewerType", OpenRAVE::RaveGetDefaultViewerType, DOXY_FN1(RaveGetDefaultViewerType));
    m.def("RaveFindLocalFile",OpenRAVE::RaveFindLocalFile, py::arg("filename"), py::arg("curdir"), DOXY_FN1(RaveFindLocalFile));
    m.def("RaveInvertFileLookup",openravepy::pyRaveInvertFileLookup,py::arg("filename"), DOXY_FN1(RaveInvertFileLookup));
    m.def("RaveGetHomeDirectory",OpenRAVE::RaveGetHomeDirectory,DOXY_FN1(RaveGetHomeDirectory));
    m.def("RaveFindDatabaseFile",OpenRAVE::RaveFindDatabaseFile,DOXY_FN1(RaveFindDatabaseFile));
    m.def("RaveLogFatal",openravepy::raveLogFatal,py::arg("log"),"Send a fatal log to the openrave system");
    m.def("RaveLogError",openravepy::raveLogError,py::arg("log"),"Send an error log to the openrave system");
    m.def("RaveLogWarn",openravepy::raveLogWarn,py::arg("log"),"Send a warn log to the openrave system");
    m.def("RaveLogInfo",openravepy::raveLogInfo,py::arg("log"),"Send an info log to the openrave system");
    m.def("RaveLogDebug",openravepy::raveLogDebug,py::arg("log"),"Send a debug log to the openrave system");
    m.def("RaveLogVerbose",openravepy::raveLogVerbose,py::arg("log"),"Send a verbose log to the openrave system");
    m.def("RaveLog",openravepy::raveLog,py::arg("log"), py::arg("level"),"Send a log to the openrave system with excplicit level");
    m.def("RaveInitialize",openravepy::pyRaveInitialize, py::arg("load_all_plugins"), py::arg("level"),DOXY_FN1(RaveInitialize));
    m.def("RaveDestroy",RaveDestroy,DOXY_FN1(RaveDestroy));
    m.def("RaveGetPluginInfo",openravepy::RaveGetPluginInfo,DOXY_FN1(RaveGetPluginInfo));
    m.def("RaveGetLoadedInterfaces",openravepy::RaveGetLoadedInterfaces,DOXY_FN1(RaveGetLoadedInterfaces));
    m.def("RaveReloadPlugins",OpenRAVE::RaveReloadPlugins,DOXY_FN1(RaveReloadPlugins));
    m.def("RaveLoadPlugin",OpenRAVE::RaveLoadPlugin,py::arg("filename"),DOXY_FN1(RaveLoadPlugins));
    m.def("RaveHasInterface",OpenRAVE::RaveHasInterface,py::arg("type"), py::arg("name"),DOXY_FN1(RaveHasInterface));
    m.def("RaveGlobalState",OpenRAVE::RaveGlobalState,DOXY_FN1(RaveGlobalState));
    m.def("RaveClone",openravepy::pyRaveClone, py::arg("ref"), py::arg("cloningoptions"), py::arg("cloneenv"), DOXY_FN1(RaveClone));
    m.def("RaveGetIkTypeFromUniqueId",OpenRAVE::RaveGetIkTypeFromUniqueId,py::arg("uniqueid"), DOXY_FN1(RaveGetIkTypeFromUniqueId));
    m.def("RaveGetIndexFromAffineDOF",OpenRAVE::RaveGetIndexFromAffineDOF, py::arg("affinedofs"), py::arg("dof"), DOXY_FN1(RaveGetIndexFromAffineDOF));
    m.def("RaveGetAffineDOFFromIndex",OpenRAVE::RaveGetAffineDOFFromIndex, py::arg("affinedofs"), py::arg("index"), DOXY_FN1(RaveGetAffineDOFFromIndex));
    m.def("RaveGetAffineDOF",OpenRAVE::RaveGetAffineDOF, py::arg("affinedofs"), DOXY_FN1(RaveGetAffineDOF));
    m.def("RaveGetAffineDOFValuesFromTransform",openravepy::pyRaveGetAffineDOFValuesFromTransform, py::arg("transform"), py::arg("affinedofs"), py::arg("rotationaxis"), DOXY_FN1(RaveGetAffineDOFValuesFromTransform));
    m.def("RaveGetAffineConfigurationSpecification",openravepy::pyRaveGetAffineConfigurationSpecification, py::arg("affinedofs"), py::arg("body"), py::arg("interpolation"), DOXY_FN1(RaveGetAffineConfigurationSpecification));

    m.def("raveSetDebugLevel",openravepy::pyRaveSetDebugLevel,py::arg("level"), DOXY_FN1(RaveSetDebugLevel));
    m.def("raveGetDebugLevel",OpenRAVE::RaveGetDebugLevel,DOXY_FN1(RaveGetDebugLevel));
    m.def("raveLogFatal",openravepy::raveLogFatal,py::arg("log"),"Send a fatal log to the openrave system");
    m.def("raveLogError",openravepy::raveLogError,py::arg("log"),"Send an error log to the openrave system");
    m.def("raveLogWarn",openravepy::raveLogWarn,py::arg("log"),"Send a warn log to the openrave system");
    m.def("raveLogInfo",openravepy::raveLogInfo,py::arg("log"),"Send an info log to the openrave system");
    m.def("raveLogDebug",openravepy::raveLogDebug,py::arg("log"),"Send a debug log to the openrave system");
    m.def("raveLogVerbose",openravepy::raveLogVerbose,py::arg("log"),"Send a verbose log to the openrave system");
    m.def("raveLog",openravepy::raveLog,py::arg("log"), py::arg("level"),"Send a log to the openrave system with excplicit level");

    m.def("quatFromAxisAngle",openravepy::quatFromAxisAngle1, py::arg("axisangle"), DOXY_FN1(quatFromAxisAngle "const RaveVector"));
    m.def("quatFromAxisAngle",openravepy::quatFromAxisAngle2, py::arg("axis"), py::arg("angle"), DOXY_FN1(quatFromAxisAngle "const RaveVector; T"));
    m.def("quatFromRotationMatrix",openravepy::quatFromRotationMatrix, py::arg("rotation"), DOXY_FN1(quatFromMatrix "const RaveTransform"));
    m.def("InterpolateQuatSlerp",openravepy::InterpolateQuatSlerp, py::arg("quat0"), py::arg("quat1"), py::arg("t"), py::arg("forceshortarc"), DOXY_FN1(InterpolateQuatSlerp "const RaveVector; const RaveVector; T"));
    m.def("InterpolateQuatSquad",openravepy::InterpolateQuatSquad, py::arg("quat0"), py::arg("quat1"), py::arg("quat2"), py::arg("quat3"), py::arg("t"), py::arg("forceshortarc"), DOXY_FN1(InterpolateQuatSquad));
    m.def("quatSlerp",openravepy::InterpolateQuatSlerp, py::arg("quat0"), py::arg("quat1"), py::arg("t"), py::arg("forceshortarc"), DOXY_FN1(quatSlerp "const RaveVector; const RaveVector; T")); // deprecated
    m.def("axisAngleFromRotationMatrix",openravepy::axisAngleFromRotationMatrix, py::arg("rotation"), DOXY_FN1(axisAngleFromMatrix "const RaveTransformMatrix"));
    m.def("axisAngleFromQuat",openravepy::axisAngleFromQuat, py::arg("quat"), DOXY_FN1(axisAngleFromQuat "const RaveVector"));
    m.def("rotationMatrixFromQuat",openravepy::rotationMatrixFromQuat, py::arg("quat"), DOXY_FN1(matrixFromQuat "const RaveVector"));
    m.def("rotationMatrixFromQArray",openravepy::rotationMatrixFromQArray,py::arg("quatarray"),"Converts an array of quaternions to a list of 3x3 rotation matrices.\n\n:param quatarray: nx4 array\n");
    m.def("matrixFromQuat",openravepy::matrixFromQuat, py::arg("quat"), "Converts a quaternion to a 4x4 affine matrix.\n\n:param quat: 4 values\n");
    m.def("rotationMatrixFromAxisAngle",openravepy::rotationMatrixFromAxisAngle1, py::arg("axisangle"), DOXY_FN1(matrixFromAxisAngle "const RaveVector"));
    m.def("rotationMatrixFromAxisAngle",openravepy::rotationMatrixFromAxisAngle2, py::arg("axis"), py::arg("angle"), DOXY_FN1(matrixFromAxisAngle "const RaveVector, T"));
    m.def("matrixFromAxisAngle",openravepy::matrixFromAxisAngle1, py::arg("axisangle"), DOXY_FN1(matrixFromAxisAngle "const RaveVector"));
    m.def("matrixFromAxisAngle",openravepy::matrixFromAxisAngle2, py::arg("axis"), py::arg("angle"), DOXY_FN1(matrixFromAxisAngle "const RaveVector, T"));
    m.def("matrixFromPose",openravepy::matrixFromPose, py::arg("pose"), "Converts a 7 element quaterion+translation transform to a 4x4 matrix.\n\n:param pose: 7 values\n");
    m.def("matrixFromPoses",openravepy::matrixFromPoses, py::arg("poses"), "Converts a Nx7 element quaterion+translation array to a 4x4 matrices.\n\n:param poses: nx7 array\n");
    m.def("poseFromMatrix",openravepy::poseFromMatrix, py::arg("transform"), "Converts a 4x4 matrix to a 7 element quaternion+translation representation.\n\n:param transform: 3x4 or 4x4 affine matrix\n");
    m.def("poseFromMatrices",openravepy::poseFromMatrices, py::arg("transforms"), "Converts an array/list of 4x4 matrices to a Nx7 array where each row is quaternion+translation representation.\n\n:param transforms: list of 3x4 or 4x4 affine matrices\n");
    m.def("InvertPoses",openravepy::InvertPoses,py::arg("poses"), "Inverts a Nx7 array of poses where first 4 columns are the quaternion and last 3 are the translation components.\n\n:param poses: nx7 array");
    m.def("InvertPose",openravepy::InvertPose,py::arg("pose"), "Inverts a 7-element pose where first 4 columns are the quaternion and last 3 are the translation components.\n\n:param pose: 7-element array");
    m.def("quatRotateDirection",openravepy::quatRotateDirection,py::arg("sourcedir"), py::arg("targetdir"), DOXY_FN1(quatRotateDirection));
    m.def("ExtractAxisFromQuat",openravepy::ExtractAxisFromQuat,py::arg("quat"), py::arg("iaxis"),DOXY_FN1(ExtractAxisFromQuat));
    m.def("MultiplyQuat",openravepy::MultiplyQuat,py::arg("quat0"), py::arg("quat1"),DOXY_FN1(quatMultiply));
    m.def("quatMult",openravepy::MultiplyQuat,py::arg("quat0"), py::arg("quat1"),DOXY_FN1(quatMultiply));
    m.def("quatMultiply",openravepy::MultiplyQuat,py::arg("quat0"), py::arg("quat1"),DOXY_FN1(quatMultiply));
    m.def("InvertQuat",openravepy::InvertQuat,py::arg("quat"),DOXY_FN1(quatInverse));
    m.def("quatInverse",openravepy::InvertQuat,py::arg("quat"),DOXY_FN1(quatInverse));
    m.def("MultiplyPose",openravepy::MultiplyPose,py::arg("pose1"), py::arg("pose2"),"multiplies two poses.\n\n:param pose1: 7 values\n\n:param pose2: 7 values\n");
    m.def("poseTransformPoint",openravepy::poseTransformPoint,py::arg("pose"), py::arg("point"),"left-transforms a 3D point by a pose transformation.\n\n:param pose: 7 values\n\n:param points: 3 values");
    m.def("poseTransformPoints",openravepy::poseTransformPoints,py::arg("pose"), py::arg("points"),"left-transforms a set of points by a pose transformation.\n\n:param pose: 7 values\n\n:param points: Nx3 values");
    m.def("TransformLookat",openravepy::TransformLookat,py::arg("lookat"), py::arg("camerapos"), py::arg("cameraup"),"Returns a camera matrix that looks along a ray with a desired up vector.\n\n:param lookat: unit axis, 3 values\n\n:param camerapos: 3 values\n\n:param cameraup: unit axis, 3 values\n");
    m.def("transformLookat",openravepy::TransformLookat,py::arg("lookat"), py::arg("camerapos"), py::arg("cameraup"),"Returns a camera matrix that looks along a ray with a desired up vector.\n\n:param lookat: unit axis, 3 values\n\n:param camerapos: 3 values\n\n:param cameraup: unit axis, 3 values\n");
    m.def("matrixSerialization",openravepy::matrixSerialization,py::arg("transform"),"Serializes a transformation into a string representing a 3x4 matrix.\n\n:param transform: 3x4 or 4x4 array\n");
    m.def("poseSerialization",openravepy::poseSerialization, py::arg("pose"), "Serializes a transformation into a string representing a quaternion with translation.\n\n:param pose: 7 values\n");
    m.def("openravepyCompilerVersion",openravepy::openravepyCompilerVersion,"Returns the compiler version that openravepy_int was compiled with");
    m.def("normalizeAxisRotation",openravepy::normalizeAxisRotation,py::arg("axis"), py::arg("quat"),DOXY_FN1(normalizeAxisRotation));
    m.def("ComputePoseDistSqr", openravepy::ComputePoseDistSqr, py::arg("pose0"), py::arg("pose1"), py::arg("quatweight"));

    // deprecated
    m.def("invertPoses",openravepy::InvertPoses,py::arg("poses"), "Inverts a Nx7 array of poses where first 4 columns are the quaternion and last 3 are the translation components.\n\n:param poses: nx7 array");
    m.def("poseMult",openravepy::MultiplyPose,py::arg("pose1"), py::arg("pose2"),"multiplies two poses.\n\n:param pose1: 7 values\n\n:param pose2: 7 values\n");
}

} // end namespace openravepy
