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
#include "openravepy_int.h"

#include <openrave/utils.h>
#include <boost/thread/once.hpp>
#include <boost/scoped_ptr.hpp>
#include <boost/multi_array.hpp>

namespace openravepy
{

#ifdef OPENRAVE_RAPIDJSON

// convert from rapidjson to python object
object toPyObject(const rapidjson::Value& value)
{
    switch (value.GetType()) {
    case rapidjson::kObjectType:
        {
            boost::python::dict d;
            for (rapidjson::Value::ConstMemberIterator it = value.MemberBegin(); it != value.MemberEnd(); ++it) {
                d[it->name.GetString()] = toPyObject(it->value);
            }
            return d;
        }
    case rapidjson::kArrayType:
        {
            boost::python::list l;
            for (rapidjson::Value::ConstValueIterator it = value.Begin(); it != value.End(); ++it) {
                l.append(toPyObject(*it));
            }
            return l;
        }
    case rapidjson::kTrueType:
        return boost::python::object(boost::python::handle<>(PyBool_FromLong(1)));
    case rapidjson::kFalseType:
        return boost::python::object(boost::python::handle<>(PyBool_FromLong(0)));
    case rapidjson::kStringType:
        return ConvertStringToUnicode(value.GetString());
    case rapidjson::kNumberType:
        if (value.IsDouble()) {
            return boost::python::object(boost::python::handle<>(PyFloat_FromDouble(value.GetDouble())));
        }
        else {
            return boost::python::object(boost::python::handle<>(PyLong_FromLong(value.GetInt64())));
        }
    case rapidjson::kNullType:
        return object();
    default:
        PyErr_SetString(PyExc_RuntimeError, "unsupported type");
        return object();
    }
}

#define FILL_RAPIDJSON_FROMARRAY_1D(pyarrayvalues, T, rapidjsonsetfn) {  \
    const T *vdata = reinterpret_cast<T*>(PyArray_DATA(pyarrayvalues)); \
    for (int i = 0; i < dims[0]; i++) { \
        rapidjson::Value elementValue; \
        elementValue.rapidjsonsetfn(vdata[i]); \
        value.PushBack(elementValue, allocator); \
    } \
} \

#define FILL_RAPIDJSON_FROMARRAY_2D(pyarrayvalues, T, rapidjsonsetfn) {                            \
    const T *vdata = reinterpret_cast<T*>(PyArray_DATA(pyarrayvalues)); \
    for (int i = 0; i < dims[0]; i++) { \
        rapidjson::Value colvalues(rapidjson::kArrayType); \
        for (int j = 0; j < dims[1]; j++) { \
            rapidjson::Value elementValue; \
            elementValue.rapidjsonsetfn(vdata[i*dims[0]+j]); \
            colvalues.PushBack(elementValue, allocator); \
        } \
        value.PushBack(colvalues, allocator); \
    } \
} \

#define FILL_RAPIDJSON_FROMARRAY(pyarrayvalues, T, rapidjsonsetfn, ndims) { \
    if( ndims == 1 ) { \
        FILL_RAPIDJSON_FROMARRAY_1D(pyarrayvalues, T, rapidjsonsetfn); \
    } \
    else if( ndims == 2 ) { \
        FILL_RAPIDJSON_FROMARRAY_2D(pyarrayvalues, T, rapidjsonsetfn); \
    } \
    else { \
        throw OPENRAVE_EXCEPTION_FORMAT(_("do not support array object with %d dims"), ndims, ORE_InvalidArguments); \
    } \
}

// convert from python object to rapidjson
void toRapidJSONValue(object &obj, rapidjson::Value &value, rapidjson::Document::AllocatorType& allocator)
{
    if (obj.ptr() == Py_None)
    {
        value.SetNull();
    }
    else if (PyBool_Check(obj.ptr()))
    {
        value.SetBool(obj.ptr() == Py_True);
    }
    else if (PyFloat_Check(obj.ptr()))
    {
        value.SetDouble(PyFloat_AsDouble(obj.ptr()));
    }
    else if (PyInt_Check(obj.ptr()))
    {
        value.SetInt64(PyLong_AsLong(obj.ptr()));
    }
    else if (PyUnicode_Check(obj.ptr()))
    {
        value.SetString(PyString_AsString(obj.ptr()), PyString_GET_SIZE(obj.ptr()));
    }
    else if (PyUnicode_Check(obj.ptr()))
    {
        value.SetString(PyBytes_AsString(obj.ptr()), PyBytes_GET_SIZE(obj.ptr()));
    }
    else if (PyTuple_Check(obj.ptr()))
    {
        boost::python::tuple t = boost::python::extract<boost::python::tuple>(obj);
        value.SetArray();
        for (int i = 0; i < len(t); i++)
        {
            boost::python::object o = boost::python::extract<boost::python::object>(t[i]);
            rapidjson::Value elementValue;
            toRapidJSONValue(o, elementValue, allocator);
            value.PushBack(elementValue, allocator);
        }
    }
    else if (PyList_Check(obj.ptr()))
    {
        boost::python::list l = boost::python::extract<boost::python::list>(obj);
        value.SetArray();
        int numitems = len(l);
        for (int i = 0; i < numitems; i++) {
            boost::python::object o = boost::python::extract<boost::python::object>(l[i]);
            rapidjson::Value elementValue;
            toRapidJSONValue(o, elementValue, allocator);
            value.PushBack(elementValue, allocator);
        }
    }
    else if (PyDict_Check(obj.ptr()))
    {
        boost::python::dict d = boost::python::extract<boost::python::dict>(obj);
        boost::python::object iterator = d.iteritems();
        value.SetObject();
        int numitems = len(d);
        for (int i = 0; i < numitems; i++)
        {
            rapidjson::Value keyValue, valueValue;
            boost::python::tuple kv = boost::python::extract<boost::python::tuple>(iterator.attr("next")());
            {
                boost::python::object k = boost::python::extract<object>(kv[0]);
                toRapidJSONValue(k, keyValue, allocator);
            }
            {
                boost::python::object v = boost::python::extract<object>(kv[1]);
                toRapidJSONValue(v, valueValue, allocator);
            }
            value.AddMember(keyValue, valueValue, allocator);
        }
    }
    else if (PyArray_Check(obj.ptr()) ) {
        PyArrayObject* pyarrayvalues = PyArray_GETCONTIGUOUS(reinterpret_cast<PyArrayObject*>(obj.ptr()));
        AutoPyArrayObjectDereferencer pydecref(pyarrayvalues);

        int ndims = PyArray_NDIM(pyarrayvalues);
        npy_intp* dims = PyArray_DIMS(pyarrayvalues);
        const size_t typeSize = PyArray_ITEMSIZE(pyarrayvalues);
        value.SetArray();
        if( ndims > 0 ) {
            if (PyArray_ISFLOAT(pyarrayvalues) ) {
                if( typeSize == sizeof(float)) {
                    FILL_RAPIDJSON_FROMARRAY(pyarrayvalues, float, SetFloat, ndims);
                }
                else if( typeSize == sizeof(double)) {
                    FILL_RAPIDJSON_FROMARRAY(pyarrayvalues, double, SetDouble, ndims);
                }
                else {
                    throw OPENRAVE_EXCEPTION_FORMAT(_("do not support array object float with %d type size"), typeSize, ORE_InvalidArguments);
                }
            }
            else if (PyArray_ISINTEGER(pyarrayvalues) ) {
                if( typeSize == sizeof(int) ) {
                    if( PyArray_ISSIGNED(pyarrayvalues) ) {
                        FILL_RAPIDJSON_FROMARRAY(pyarrayvalues, int, SetInt, ndims);
                    }
                    else {
                        FILL_RAPIDJSON_FROMARRAY(pyarrayvalues, uint32_t, SetUint, ndims);
                    }
                }
                else if( typeSize == sizeof(int64_t) ) {
                    if( PyArray_ISSIGNED(pyarrayvalues) ) {
                        FILL_RAPIDJSON_FROMARRAY(pyarrayvalues, uint64_t, SetInt64, ndims);
                    }
                    else {
                        FILL_RAPIDJSON_FROMARRAY(pyarrayvalues, uint64_t, SetUint64, ndims);
                    }
                }
                else {
                    throw OPENRAVE_EXCEPTION_FORMAT(_("do not support array object integer with %d type size"), typeSize, ORE_InvalidArguments);
                }
            }
            else {
                throw OPENRAVE_EXCEPTION_FORMAT(_("do not support array object with %d type size"), typeSize, ORE_InvalidArguments);
            }
        }
    }           
    else
    {
        throw OPENRAVE_EXCEPTION_FORMAT0(_("unsupported python type"), ORE_InvalidArguments);
    }
}

#endif // OPENRAVE_RAPIDJSON
    
/// if set, will return all transforms are 1x7 vectors where first 4 compoonents are quaternion
static bool s_bReturnTransformQuaternions = false;
bool GetReturnTransformQuaternions() {
    return s_bReturnTransformQuaternions;
}
void SetReturnTransformQuaternions(bool bset) {
    s_bReturnTransformQuaternions = bset;
}

Transform ExtractTransform(const object& oraw)
{
    return ExtractTransformType<dReal>(oraw);
}

TransformMatrix ExtractTransformMatrix(const object& oraw)
{
    return ExtractTransformMatrixType<dReal>(oraw);
}

object toPyArray(const TransformMatrix& t)
{
    boost::python::tuple shapeA = boost::python::make_tuple(4, 4);
    np::dtype dt = sizeof(dReal)==8 ? np::dtype::get_builtin<double>() : np::dtype::get_builtin<float>();
    np::ndarray A = np::zeros(shapeA, dt);
    for(int i = 0; i < 3; ++i) {
        for(int j = 0; j < 3; ++j) {
            A[i][j] = t.m[4*i+j];
        }
        A[i][3] = t.trans[i];
    }
    A[3][0] = A[3][1] = A[3][2] = 0;
    A[3][3] = 1;
    return std::move(A);
}


object toPyArray(const Transform& t)
{
    npy_intp dims[] = { 7};
    PyObject *pyvalues = PyArray_SimpleNew(1,dims, sizeof(dReal)==8 ? PyArray_DOUBLE : PyArray_FLOAT);
    dReal* pdata = (dReal*)PyArray_DATA(pyvalues);
    pdata[0] = t.rot.x; pdata[1] = t.rot.y; pdata[2] = t.rot.z; pdata[3] = t.rot.w;
    pdata[4] = t.trans.x; pdata[5] = t.trans.y; pdata[6] = t.trans.z;
    return toPyArrayN(pdata, 7);
}

AttributesList toAttributesList(boost::python::dict odict)
{
    AttributesList atts;
    if( !IS_PYTHONOBJECT_NONE(odict) ) {
        boost::python::list iterkeys = (boost::python::list)odict.iterkeys();
        size_t num = boost::python::len(iterkeys);
        for (size_t i = 0; i < num; i++) {
            // Because we know they're strings, we can do this
            std::string key = boost::python::extract<std::string>(iterkeys[i]);
            std::string value = boost::python::extract<std::string>(odict[iterkeys[i]]);
            atts.emplace_back(key, value);
        }
    }
    return atts;
}

AttributesList toAttributesList(boost::python::list oattributes)
{
    AttributesList atts;
    if( !IS_PYTHONOBJECT_NONE(oattributes) ) {
        size_t num=len(oattributes);
        for (size_t i = 0; i < num; i++) {
            // Because we know they're strings, we can do this
            std::string key = boost::python::extract<std::string>(oattributes[i][0]);
            std::string value = boost::python::extract<std::string>(oattributes[i][1]);
            atts.emplace_back(key, value);
        }
    }
    return atts;
}

AttributesList toAttributesList(boost::python::object oattributes)
{
    if( !IS_PYTHONOBJECT_NONE(oattributes) ) {
        boost::python::extract<boost::python::dict> odictextractor(oattributes);
        if( odictextractor.check() ) {
            return toAttributesList((boost::python::dict)odictextractor());
        }
        // assume list
        boost::python::extract<boost::python::list> olistextractor(oattributes);
        return toAttributesList((boost::python::list)olistextractor());
    }
    return AttributesList();
}

/// \brief manages all the viewers created through SetViewer into a single thread
class ViewerManager
{
    /// \brief info about the viewer to create or that is created
    struct ViewerInfo
    {
        EnvironmentBasePtr _penv;
        std::string _viewername;
        ViewerBasePtr _pviewer; /// the created viewer
        boost::condition _cond;  ///< notify when viewer thread is done processing and has initialized _pviewer
        bool _bShowViewer; ///< true if should show the viewer when initially created
    };
    typedef OPENRAVE_SHARED_PTR<ViewerInfo> ViewerInfoPtr;
public:
    ViewerManager() {
        _bShutdown = false;
        _bInMain = false;
        _threadviewer.reset(new boost::thread(boost::bind(&ViewerManager::_RunViewerThread, this)));
    }

    virtual ~ViewerManager() {
        Destroy();
    }

    static ViewerManager& GetInstance() {
        boost::call_once(_InitializeSingleton, _onceInitialize);
        // Return reference to object.
        return *_singleton;
    }

    /// \brief adds a viewer to the environment whose GUI thread will be managed by _RunViewerThread
    ///
    /// \param bDoNotAddIfExists if true, will not add a viewer if one already exists and is added to the manager
    ViewerBasePtr AddViewer(EnvironmentBasePtr penv, const string &strviewer, bool bShowViewer, bool bDoNotAddIfExists=true)
    {
        ViewerBasePtr pviewer;
        if( strviewer.size() > 0 ) {

            if( bDoNotAddIfExists ) {
                // check all existing viewers
                boost::mutex::scoped_lock lock(_mutexViewer);
                std::list<ViewerInfoPtr>::iterator itviewer = _listviewerinfos.begin();
                while(itviewer != _listviewerinfos.end() ) {
                    if( (*itviewer)->_penv == penv ) {
                        if( (*itviewer)->_viewername == strviewer ) {
                            if( !!(*itviewer)->_pviewer ) {
                                (*itviewer)->_pviewer->Show(bShowViewer);
                            }
                            return (*itviewer)->_pviewer;
                        }

                        // should remove the viewer so can re-add a new one
                        if( !!(*itviewer)->_pviewer ) {
                            (*itviewer)->_penv->Remove((*itviewer)->_pviewer);
                        }
                        itviewer = _listviewerinfos.erase(itviewer);
                    }
                    else {
                        ++itviewer;
                    }
                }
            }

            ViewerInfoPtr pinfo(new ViewerInfo());
            pinfo->_penv = penv;
            pinfo->_viewername = strviewer;
            pinfo->_bShowViewer = bShowViewer;
            if( _bInMain ) {
                // create in this thread since viewer thread is already waiting on another viewer
                pviewer = RaveCreateViewer(penv, strviewer);
                if( !!pviewer ) {
                    penv->AddViewer(pviewer);
                    // TODO uncomment once Show posts to queue
                    if( bShowViewer ) {
                        pviewer->Show(1);
                    }
                    pinfo->_pviewer = pviewer;
                    boost::mutex::scoped_lock lock(_mutexViewer);
                    _listviewerinfos.push_back(pinfo);
                    _conditionViewer.notify_all();
                }
            }
            else {
                // no viewer has been created yet, so let the viewer thread create it (if using Qt, this initializes the QApplication in the right thread
                boost::mutex::scoped_lock lock(_mutexViewer);
                _listviewerinfos.push_back(pinfo);
                _conditionViewer.notify_all();

                /// wait until viewer thread process it
                pinfo->_cond.wait(_mutexViewer);
                pviewer = pinfo->_pviewer;
            }
        }
        return pviewer;
    }

    /// \brief if removed, returns true
    bool RemoveViewer(ViewerBasePtr pviewer)
    {
        if( !pviewer ) {
            return false;
        }
        {
            boost::mutex::scoped_lock lock(_mutexViewer);
            FOREACH(itviewer, _listviewerinfos) {
                ViewerBasePtr ptestviewer = (*itviewer)->_pviewer;
                if(ptestviewer == pviewer ) {
                    pviewer->quitmainloop();
                    _listviewerinfos.erase(itviewer);
                    return true;
                }
            }
        }
        return false;
    }

    /// \brief if anything removed, returns true
    bool RemoveViewersOfEnvironment(EnvironmentBasePtr penv)
    {
        if( !penv ) {
            return false;
        }
        bool bremoved = false;
        {
            boost::mutex::scoped_lock lock(_mutexViewer);
            std::list<ViewerInfoPtr>::iterator itinfo = _listviewerinfos.begin();
            while(itinfo != _listviewerinfos.end() ) {
                if( (*itinfo)->_penv == penv ) {
                    itinfo = _listviewerinfos.erase(itinfo);
                    bremoved = true;
                }
                else {
                    ++itinfo;
                }
            }
        }
        return bremoved;
    }

    void Destroy() {
        _bShutdown = true;
        {
            boost::mutex::scoped_lock lock(_mutexViewer);
            // have to notify everyone
            FOREACH(itinfo, _listviewerinfos) {
                (*itinfo)->_cond.notify_all();
            }
            _listviewerinfos.clear();
            _conditionViewer.notify_all();
        }
        if( !!_threadviewer ) {
            _threadviewer->join();
        }
        _threadviewer.reset();
    }

protected:
    void _RunViewerThread()
    {
        while(!_bShutdown) {
            std::list<ViewerBasePtr> listviewers, listtempviewers;
            bool bShowViewer = true;
            {
                boost::mutex::scoped_lock lock(_mutexViewer);
                if( _listviewerinfos.size() == 0 ) {
                    _conditionViewer.wait(lock);
                    if( _listviewerinfos.size() == 0 ) {
                        continue;
                    }
                }

                listtempviewers.clear(); // viewers to add to env once lock is released
                listviewers.clear();
                std::list<ViewerInfoPtr>::iterator itinfo = _listviewerinfos.begin();
                while(itinfo != _listviewerinfos.end() ) {
                    ViewerInfoPtr pinfo = *itinfo;
                    if( !pinfo->_pviewer ) {
                        pinfo->_pviewer = RaveCreateViewer(pinfo->_penv, pinfo->_viewername);
                        // have to notify other thread that viewer is present before the environment lock happens! otherwise we can get into deadlock between c++ and python
                        pinfo->_cond.notify_all();
                        if( !!pinfo->_pviewer ) {
                            listtempviewers.push_back(pinfo->_pviewer);
                            ++itinfo;
                        }
                        else {
                            // erase from _listviewerinfos
                            itinfo = _listviewerinfos.erase(itinfo);
                        }
                    }
                    else {
                        ++itinfo;
                    }

                    if( !!pinfo->_pviewer ) {
                        if( listviewers.size() == 0 ) {
                            bShowViewer = pinfo->_bShowViewer;
                        }
                        listviewers.push_back(pinfo->_pviewer);
                    }
                }
            }

            FOREACH(itaddviewer, listtempviewers) {
                (*itaddviewer)->GetEnv()->AddViewer(*itaddviewer);
            }

            ViewerBasePtr puseviewer;
            FOREACH(itviewer, listviewers) {
                // double check if viewer is added to env
                bool bfound = false;
                listtempviewers.clear();
                (*itviewer)->GetEnv()->GetViewers(listtempviewers);
                FOREACH(itviewer2, listtempviewers) {
                    if( *itviewer == *itviewer2 ) {
                        bfound = true;
                        break;
                    }
                }
                if( bfound ) {
                    puseviewer = *itviewer;
                    break;
                }
                else {
                    // viewer is not in environment any more, so erase from list
                    listviewers.erase(itviewer);
                    break; // break since modifying list
                }
            }

            listtempviewers.clear();

            if( !!puseviewer ) {
                _bInMain = true;
                try {
                    puseviewer->main(bShowViewer);
                }
                catch(const std::exception& ex) {
                    RAVELOG_ERROR_FORMAT("got exception in viewer main thread %s", ex.what());
                }
                catch(...) {
                    RAVELOG_ERROR("got unknown exception in viewer main thread\n");
                }

                _bInMain = false;
                // remove from _listviewerinfos in order to avoid running the main loop again
                {
                    boost::mutex::scoped_lock lock(_mutexViewer);
                    FOREACH(itinfo, _listviewerinfos) {
                        if( (*itinfo)->_pviewer == puseviewer ) {
                            _listviewerinfos.erase(itinfo);
                            break;
                        }
                    }
                }
                puseviewer.reset();
            }
            // just go and run the next viewer's loop, don't exit here!
        }
        RAVELOG_DEBUG("shutting down viewer manager thread\n");
    }

    static void _InitializeSingleton()
    {
        _singleton.reset(new ViewerManager());

    }

    OPENRAVE_SHARED_PTR<boost::thread> _threadviewer;
    boost::mutex _mutexViewer;
    boost::condition _conditionViewer;
    std::list<ViewerInfoPtr> _listviewerinfos;

    bool _bShutdown; ///< if true, shutdown everything
    bool _bInMain; ///< if true, viewer thread is running a main function

    static boost::scoped_ptr<ViewerManager> _singleton; ///< singleton
    static boost::once_flag _onceInitialize; ///< makes sure initialization is atomic

}; // class ViewerManager

boost::scoped_ptr<ViewerManager> ViewerManager::_singleton(0);
boost::once_flag ViewerManager::_onceInitialize = BOOST_ONCE_INIT;

PyInterfaceBase::PyInterfaceBase(InterfaceBasePtr pbase, PyEnvironmentBasePtr pyenv) : _pbase(pbase), _pyenv(pyenv)
{
    CHECK_POINTER(_pbase);
    CHECK_POINTER(_pyenv);
}

object PyInterfaceBase::GetUserData(const std::string& key) const {
    return openravepy::GetUserData(_pbase->GetUserData(key));
}

bool PyInterfaceBase::SupportsCommand(const string& cmd)
{
    return _pbase->SupportsCommand(cmd);
}

#ifdef OPENRAVE_RAPIDJSON
bool PyInterfaceBase::SupportsJSONCommand(const string& cmd)
{
    return _pbase->SupportsJSONCommand(cmd);
}
#endif // OPENRAVE_RAPIDJSON

object PyInterfaceBase::SendCommand(const string& in, bool releasegil, bool lockenv)
{
    stringstream sin(in), sout;
    {
        openravepy::PythonThreadSaverPtr statesaver;
        openravepy::PyEnvironmentLockSaverPtr envsaver;
        if( releasegil ) {
            statesaver.reset(new openravepy::PythonThreadSaver());
            if( lockenv ) {
                // GIL is already released, so use a regular environment lock
                envsaver.reset(new openravepy::PyEnvironmentLockSaver(_pyenv, true));
            }
        }
        else {
            if( lockenv ) {
                // try to safely lock the environment first
                envsaver.reset(new openravepy::PyEnvironmentLockSaver(_pyenv, false));
            }
        }
        sout << std::setprecision(std::numeric_limits<dReal>::digits10+1);     /// have to do this or otherwise precision gets lost
        if( !_pbase->SendCommand(sout,sin) ) {
            return object();
        }
    }
    return object(sout.str());
}

#ifdef OPENRAVE_RAPIDJSON

object PyInterfaceBase::SendJSONCommand(const string& cmd, object input, bool releasegil, bool lockenv)
{
    rapidjson::Document in, out;
    toRapidJSONValue(input, in, in.GetAllocator());

    {
        openravepy::PythonThreadSaverPtr statesaver;
        openravepy::PyEnvironmentLockSaverPtr envsaver;
        if( releasegil ) {
            statesaver.reset(new openravepy::PythonThreadSaver());
            if( lockenv ) {
                // GIL is already released, so use a regular environment lock
                envsaver.reset(new openravepy::PyEnvironmentLockSaver(_pyenv, true));
            }
        }
        else {
            if( lockenv ) {
                // try to safely lock the environment first
                envsaver.reset(new openravepy::PyEnvironmentLockSaver(_pyenv, false));
            }
        }

        _pbase->SendJSONCommand(cmd, in, out);
    }

    return toPyObject(out);
}

#endif // OPENRAVE_RAPIDJSON

object PyInterfaceBase::GetReadableInterfaces()
{
    boost::python::dict ointerfaces;
    FOREACHC(it,_pbase->GetReadableInterfaces()) {
        ointerfaces[it->first] = toPyXMLReadable(it->second);
    }
    return std::move(ointerfaces);
}

object PyInterfaceBase::GetReadableInterface(const std::string& xmltag)
{
    return toPyXMLReadable(_pbase->GetReadableInterface(xmltag));
}

void PyInterfaceBase::SetReadableInterface(const std::string& xmltag, object oreadable)
{
    _pbase->SetReadableInterface(xmltag,ExtractXMLReadable(oreadable));
}

PyEnvironmentBasePtr PyInterfaceBase::GetEnv() const
{
#if BOOST_VERSION >= 103500
    return _pyenv;
#else
    // if raw shared_ptr is returned, then python will throw RuntimeError: tr1::bad_weak_ptr when env is used
    return PyEnvironmentBasePtr(new PyEnvironmentBase(_pyenv->GetEnv()));
#endif
}

object GetUserData(UserDataPtr pdata)
{
    OPENRAVE_SHARED_PTR<PyUserObject> po = OPENRAVE_DYNAMIC_POINTER_CAST<PyUserObject>(pdata);
    if( !!po ) {
        return po->_o;
    }
    else {
        SerializableDataPtr pserializable = OPENRAVE_DYNAMIC_POINTER_CAST<SerializableData>(pdata);
        if( !!pserializable ) {
            return object(PySerializableData(pserializable));
        }
        else if( !!pdata ) {
            return object(PyUserData(pdata));
        }
        else {
            return object();
        }
    }
}

EnvironmentBasePtr GetEnvironment(PyEnvironmentBasePtr pyenv)
{
    return !pyenv ? EnvironmentBasePtr() : pyenv->GetEnv();
}

EnvironmentBasePtr GetEnvironment(object o)
{
    if( !IS_PYTHONOBJECT_NONE(o)) {
        extract<PyEnvironmentBasePtr> pyenv(o);
        if( pyenv.check() ) {
            return ((PyEnvironmentBasePtr)pyenv)->GetEnv();
        }
    }
    return EnvironmentBasePtr();
}

object toPyEnvironment(object o)
{
    extract<PyInterfaceBasePtr> pyinterface(o);
    if( pyinterface.check() ) {
        return object(((PyInterfaceBasePtr)pyinterface)->GetEnv());
    }
    return object();
}

void LockEnvironment(PyEnvironmentBasePtr pyenv)
{
    pyenv->Lock();
}

void UnlockEnvironment(PyEnvironmentBasePtr pyenv)
{
    pyenv->Unlock();
}

PyEnvironmentLockSaver::PyEnvironmentLockSaver(PyEnvironmentBasePtr pyenv, bool braw) : _pyenv(pyenv)
{
    if( braw ) {
        _pyenv->LockRaw();
    }
    else {
        _pyenv->Lock();
    }
}
PyEnvironmentLockSaver::~PyEnvironmentLockSaver()
{
    _pyenv->Unlock();
}

object RaveGetEnvironments()
{
    std::list<EnvironmentBasePtr> listenvironments;
    OpenRAVE::RaveGetEnvironments(listenvironments);
    boost::python::list oenvironments;
    FOREACH(it,listenvironments) {
        oenvironments.append(PyEnvironmentBasePtr(new PyEnvironmentBase(*it)));
    }
    return std::move(oenvironments);
}
int RaveGetEnvironmentId(PyEnvironmentBasePtr pyenv)
{
    return OpenRAVE::RaveGetEnvironmentId(pyenv->GetEnv());
}

PyEnvironmentBasePtr RaveGetEnvironment(int id)
{
    EnvironmentBasePtr penv = OpenRAVE::RaveGetEnvironment(id);
    if( !penv ) {
        return PyEnvironmentBasePtr();
    }
    return PyEnvironmentBasePtr(new PyEnvironmentBase(penv));
}

PyInterfaceBasePtr RaveCreateInterface(PyEnvironmentBasePtr pyenv, InterfaceType type, const std::string& name)
{
    InterfaceBasePtr p = OpenRAVE::RaveCreateInterface(pyenv->GetEnv(), type, name);
    if( !p ) {
        return PyInterfaceBasePtr();
    }
    return PyInterfaceBasePtr(new PyInterfaceBase(p,pyenv));
}

BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(LoadURI_overloads, LoadURI, 1, 2)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(SetCamera_overloads, SetCamera, 2, 4)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(StartSimulation_overloads, StartSimulation, 1, 2)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(StopSimulation_overloads, StopSimulation, 0, 1)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(SetViewer_overloads, SetViewer, 1, 2)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(SetDefaultViewer_overloads, SetDefaultViewer, 0, 1)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(CheckCollisionRays_overloads, CheckCollisionRays, 2, 3)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(plot3_overloads, plot3, 2, 4)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(drawlinestrip_overloads, drawlinestrip, 2, 4)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(drawlinelist_overloads, drawlinelist, 2, 4)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(drawarrow_overloads, drawarrow, 2, 4)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(drawbox_overloads, drawbox, 2, 3)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(drawtrimesh_overloads, drawtrimesh, 1, 3)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(SendCommand_overloads, SendCommand, 1, 3)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(SendJSONCommand_overloads, SendJSONCommand, 2, 4)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(Add_overloads, Add, 1, 3)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(Save_overloads, Save, 1, 3)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(WriteToMemory_overloads, WriteToMemory, 1, 3)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(GetUserData_overloads, GetUserData, 0, 1)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(GetPublishedBody_overloads, GetPublishedBody, 1, 2)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(GetPublishedBodies_overloads, GetPublishedBodies, 0, 1)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(GetPublishedBodyJointValues_overloads, GetPublishedBodyJointValues, 1, 2)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(GetPublishedBodyTransformsMatchingPrefix_overloads, GetPublishedBodyTransformsMatchingPrefix, 1, 2)

object get_openrave_exception_unicode(openrave_exception* p)
{
    std::string s = p->message();
    return ConvertStringToUnicode(s);
}

std::string get_openrave_exception_repr(openrave_exception* p)
{
    return boost::str(boost::format("<openrave_exception('%s',ErrorCode.%s)>")%p->message()%GetErrorCodeString(p->GetCode()));
}

object get_std_runtime_error_unicode(std::runtime_error* p)
{
    std::string s(p->what());
    return ConvertStringToUnicode(s);
}

std::string get_std_runtime_error_repr(std::runtime_error* p)
{
    return boost::str(boost::format("<std_exception('%s')>")%p->what());
}

} // namespace openravepy

PYBIND11_MODULE(openravepy_int, m)
{
    using namespace openravepy;
#if BOOST_VERSION >= 103500
    docstring_options doc_options;
    doc_options.disable_cpp_signatures();
    doc_options.enable_py_signatures();
    doc_options.enable_user_defined();
#endif
    // new
    Py_Initialize();
    np::initialize();

    // expansion of the macro `import_array()` in 
    // python3/lib/python3.7/site-packages/numpy/core/include/numpy/__multiarray_api.h
    if (_import_array() < 0) {
        PyErr_Print();
        PyErr_SetString(PyExc_ImportError, "numpy.core.multiarray failed to import");
        return;
    }

    // boost::python::numeric::array::set_module_and_type("numpy", "ndarray");
    // int_from_number<int>();
    // int_from_number<uint8_t>();
    // float_from_number<float>();
    // float_from_number<double>();
    init_python_bindings();

    typedef return_value_policy< copy_const_reference > return_copy_const_ref;
    class_< openrave_exception >( "_openrave_exception_", DOXY_CLASS(openrave_exception) )
    .def( init<const std::string&>() )
    .def( init<const openrave_exception&>() )
    .def( "message", &openrave_exception::message, return_copy_const_ref() )
    .def("GetCode", &openrave_exception::GetCode )
    .def( "__str__", &openrave_exception::message, return_copy_const_ref() )
    .def( "__unicode__", get_openrave_exception_unicode)
    .def( "__repr__", get_openrave_exception_repr)
    ;
    // exception_translator<openrave_exception>();
    class_< std::runtime_error >( "_std_runtime_error_", no_init)
    .def( init<const std::string&>() )
    .def( init<const std::runtime_error&>() )
    .def( "message", &std::runtime_error::what)
    .def( "__str__", &std::runtime_error::what)
    .def( "__unicode__", get_std_runtime_error_unicode)
    .def( "__repr__", get_std_runtime_error_repr)
    ;
    // exception_translator<std::runtime_error>();
    //exception_translator<std::exception>();
    class_< boost::bad_function_call, bases<std::runtime_error> >( "_boost_bad_function_call_");
    // exception_translator<boost::bad_function_call>();

    class_<PyEnvironmentBase, PyEnvironmentBasePtr > classenv("Environment", DOXY_CLASS(EnvironmentBase));
    {
        void (PyInterfaceBase::*setuserdata1)(PyUserData) = &PyInterfaceBase::SetUserData;
        void (PyInterfaceBase::*setuserdata2)(object) = &PyInterfaceBase::SetUserData;
        void (PyInterfaceBase::*setuserdata3)(const std::string&, PyUserData) = &PyInterfaceBase::SetUserData;
        void (PyInterfaceBase::*setuserdata4)(const std::string&, object) = &PyInterfaceBase::SetUserData;
        std::string sSendCommandDoc = std::string(DOXY_FN(InterfaceBase,SendCommand)) + std::string("The calling conventions between C++ and Python differ a little.\n\n\
In C++ the syntax is::\n\n  success = SendCommand(OUT, IN)\n\n\
In python, the syntax is::\n\n\
  OUT = SendCommand(IN,releasegil)\n\
  success = OUT is not None\n\n\n\
The **releasegil** parameter controls whether the python Global Interpreter Lock should be released when executing this code. For calls that take a long time and if there are many threads running called from different python threads, releasing the GIL could speed up things a lot. Please keep in mind that releasing and re-acquiring the GIL also takes computation time.\n\
Because race conditions can pop up when trying to lock the openrave environment without releasing the GIL, if lockenv=True is specified, the system can try to safely lock the openrave environment without causing a deadlock with the python GIL and other threads.\n");
        class_<PyInterfaceBase, OPENRAVE_SHARED_PTR<PyInterfaceBase> >("Interface", DOXY_CLASS(InterfaceBase), no_init)
        .def("GetInterfaceType",&PyInterfaceBase::GetInterfaceType, DOXY_FN(InterfaceBase,GetInterfaceType))
        .def("GetXMLId",&PyInterfaceBase::GetXMLId, DOXY_FN(InterfaceBase,GetXMLId))
        .def("GetPluginName",&PyInterfaceBase::GetPluginName, DOXY_FN(InterfaceBase,GetPluginName))
        .def("GetDescription",&PyInterfaceBase::GetDescription, DOXY_FN(InterfaceBase,GetDescription))
        .def("SetDescription",&PyInterfaceBase::SetDescription, DOXY_FN(InterfaceBase,SetDescription))
        .def("GetEnv",&PyInterfaceBase::GetEnv, DOXY_FN(InterfaceBase,GetEnv))
        .def("Clone",&PyInterfaceBase::Clone,args("ref","cloningoptions"), DOXY_FN(InterfaceBase,Clone))
        .def("SetUserData",setuserdata1,args("data"), DOXY_FN(InterfaceBase,SetUserData))
        .def("SetUserData",setuserdata2,args("data"), DOXY_FN(InterfaceBase,SetUserData))
        .def("SetUserData",setuserdata3,args("key","data"), DOXY_FN(InterfaceBase,SetUserData))
        .def("SetUserData",setuserdata4,args("key", "data"), DOXY_FN(InterfaceBase,SetUserData))
        .def("RemoveUserData", &PyInterfaceBase::RemoveUserData, DOXY_FN(InterfaceBase, RemoveUserData))
        .def("GetUserData",&PyInterfaceBase::GetUserData, GetUserData_overloads(args("key"), DOXY_FN(InterfaceBase,GetUserData)))
        .def("SupportsCommand",&PyInterfaceBase::SupportsCommand, args("cmd"), DOXY_FN(InterfaceBase,SupportsCommand))
        .def("SendCommand",&PyInterfaceBase::SendCommand, SendCommand_overloads(args("cmd","releasegil","lockenv"), sSendCommandDoc.c_str()))
#ifdef OPENRAVE_RAPIDJSON
        .def("SupportsJSONCommand",&PyInterfaceBase::SupportsJSONCommand, args("cmd"), DOXY_FN(InterfaceBase,SupportsJSONCommand))
        .def("SendJSONCommand",&PyInterfaceBase::SendJSONCommand, SendJSONCommand_overloads(args("cmd","input","releasegil","lockenv"), DOXY_FN(InterfaceBase,SendJSONCommand)))
#endif // OPENRAVE_RAPIDJSON
        .def("GetReadableInterfaces",&PyInterfaceBase::GetReadableInterfaces,DOXY_FN(InterfaceBase,GetReadableInterfaces))
        .def("GetReadableInterface",&PyInterfaceBase::GetReadableInterface,DOXY_FN(InterfaceBase,GetReadableInterface))
        .def("SetReadableInterface",&PyInterfaceBase::SetReadableInterface,args("xmltag","xmlreadable"), DOXY_FN(InterfaceBase,SetReadableInterface))
        .def("__repr__", &PyInterfaceBase::__repr__)
        .def("__str__", &PyInterfaceBase::__str__)
        .def("__unicode__", &PyInterfaceBase::__unicode__)
        .def("__hash__",&PyInterfaceBase::__hash__)
        .def("__eq__",&PyInterfaceBase::__eq__)
        .def("__ne__",&PyInterfaceBase::__ne__)
        ;
    }

    {
        bool (PyEnvironmentBase::*pcolb)(PyKinBodyPtr) = &PyEnvironmentBase::CheckCollision;
        bool (PyEnvironmentBase::*pcolbr)(PyKinBodyPtr, PyCollisionReportPtr) = &PyEnvironmentBase::CheckCollision;
        bool (PyEnvironmentBase::*pcolbb)(PyKinBodyPtr,PyKinBodyPtr) = &PyEnvironmentBase::CheckCollision;
        bool (PyEnvironmentBase::*pcolbbr)(PyKinBodyPtr, PyKinBodyPtr,PyCollisionReportPtr) = &PyEnvironmentBase::CheckCollision;
        bool (PyEnvironmentBase::*pcoll)(object) = &PyEnvironmentBase::CheckCollision;
        bool (PyEnvironmentBase::*pcollr)(object, PyCollisionReportPtr) = &PyEnvironmentBase::CheckCollision;
        bool (PyEnvironmentBase::*pcolll)(object,object) = &PyEnvironmentBase::CheckCollision;
        bool (PyEnvironmentBase::*pcolllr)(object,object, PyCollisionReportPtr) = &PyEnvironmentBase::CheckCollision;
        bool (PyEnvironmentBase::*pcollb)(object, PyKinBodyPtr) = &PyEnvironmentBase::CheckCollision;
        bool (PyEnvironmentBase::*pcollbr)(object, PyKinBodyPtr, PyCollisionReportPtr) = &PyEnvironmentBase::CheckCollision;
        bool (PyEnvironmentBase::*pcolle)(object,object,object) = &PyEnvironmentBase::CheckCollision;
        bool (PyEnvironmentBase::*pcoller)(object, object,object,PyCollisionReportPtr) = &PyEnvironmentBase::CheckCollision;
        bool (PyEnvironmentBase::*pcolbe)(PyKinBodyPtr,object,object) = &PyEnvironmentBase::CheckCollision;
        bool (PyEnvironmentBase::*pcolber)(PyKinBodyPtr, object,object,PyCollisionReportPtr) = &PyEnvironmentBase::CheckCollision;
        bool (PyEnvironmentBase::*pcolyb)(OPENRAVE_SHARED_PTR<PyRay>,PyKinBodyPtr) = &PyEnvironmentBase::CheckCollision;
        bool (PyEnvironmentBase::*pcolybr)(OPENRAVE_SHARED_PTR<PyRay>, PyKinBodyPtr, PyCollisionReportPtr) = &PyEnvironmentBase::CheckCollision;
        bool (PyEnvironmentBase::*pcoly)(OPENRAVE_SHARED_PTR<PyRay>) = &PyEnvironmentBase::CheckCollision;
        bool (PyEnvironmentBase::*pcolyr)(OPENRAVE_SHARED_PTR<PyRay>, PyCollisionReportPtr) = &PyEnvironmentBase::CheckCollision;

        void (PyEnvironmentBase::*Lock1)() = &PyEnvironmentBase::Lock;
        bool (PyEnvironmentBase::*Lock2)(float) = &PyEnvironmentBase::Lock;

        object (PyEnvironmentBase::*drawplane1)(object, object, const boost::multi_array<float,2>&) = &PyEnvironmentBase::drawplane;
        object (PyEnvironmentBase::*drawplane2)(object, object, const boost::multi_array<float,3>&) = &PyEnvironmentBase::drawplane;

        void (PyEnvironmentBase::*addkinbody1)(PyKinBodyPtr) = &PyEnvironmentBase::AddKinBody;
        void (PyEnvironmentBase::*addkinbody2)(PyKinBodyPtr,bool) = &PyEnvironmentBase::AddKinBody;
        void (PyEnvironmentBase::*addrobot1)(PyRobotBasePtr) = &PyEnvironmentBase::AddRobot;
        void (PyEnvironmentBase::*addrobot2)(PyRobotBasePtr,bool) = &PyEnvironmentBase::AddRobot;
        void (PyEnvironmentBase::*addsensor1)(PySensorBasePtr) = &PyEnvironmentBase::AddSensor;
        void (PyEnvironmentBase::*addsensor2)(PySensorBasePtr,bool) = &PyEnvironmentBase::AddSensor;
        void (PyEnvironmentBase::*setuserdata1)(PyUserData) = &PyEnvironmentBase::SetUserData;
        void (PyEnvironmentBase::*setuserdata2)(object) = &PyEnvironmentBase::SetUserData;
        bool (PyEnvironmentBase::*load1)(const string &) = &PyEnvironmentBase::Load;
        bool (PyEnvironmentBase::*load2)(const string &, object) = &PyEnvironmentBase::Load;
        bool (PyEnvironmentBase::*loaddata1)(const string &) = &PyEnvironmentBase::LoadData;
        bool (PyEnvironmentBase::*loaddata2)(const string &, object) = &PyEnvironmentBase::LoadData;
        object (PyEnvironmentBase::*readrobotxmlfile1)(const string &) = &PyEnvironmentBase::ReadRobotURI;
        object (PyEnvironmentBase::*readrobotxmlfile2)(const string &,object) = &PyEnvironmentBase::ReadRobotURI;
        object (PyEnvironmentBase::*readrobotxmldata1)(const string &) = &PyEnvironmentBase::ReadRobotData;
        object (PyEnvironmentBase::*readrobotxmldata2)(const string &,object) = &PyEnvironmentBase::ReadRobotData;
        object (PyEnvironmentBase::*readkinbodyxmlfile1)(const string &) = &PyEnvironmentBase::ReadKinBodyURI;
        object (PyEnvironmentBase::*readkinbodyxmlfile2)(const string &,object) = &PyEnvironmentBase::ReadKinBodyURI;
        object (PyEnvironmentBase::*readkinbodyxmldata1)(const string &) = &PyEnvironmentBase::ReadKinBodyData;
        object (PyEnvironmentBase::*readkinbodyxmldata2)(const string &,object) = &PyEnvironmentBase::ReadKinBodyData;
        PyInterfaceBasePtr (PyEnvironmentBase::*readinterfacexmlfile1)(const string &) = &PyEnvironmentBase::ReadInterfaceURI;
        PyInterfaceBasePtr (PyEnvironmentBase::*readinterfacexmlfile2)(const string &,object) = &PyEnvironmentBase::ReadInterfaceURI;
        object (PyEnvironmentBase::*readtrimeshfile1)(const std::string&) = &PyEnvironmentBase::ReadTrimeshURI;
        object (PyEnvironmentBase::*readtrimeshfile2)(const std::string&,object) = &PyEnvironmentBase::ReadTrimeshURI;
        object (PyEnvironmentBase::*readtrimeshdata1)(const std::string&,const std::string&) = &PyEnvironmentBase::ReadTrimeshData;
        object (PyEnvironmentBase::*readtrimeshdata2)(const std::string&,const std::string&,object) = &PyEnvironmentBase::ReadTrimeshData;
        scope env = classenv
                    .def(init<optional<int> >(args("options")))
                    .def("Reset",&PyEnvironmentBase::Reset, DOXY_FN(EnvironmentBase,Reset))
                    .def("Destroy",&PyEnvironmentBase::Destroy, DOXY_FN(EnvironmentBase,Destroy))
                    .def("CloneSelf",&PyEnvironmentBase::CloneSelf,args("options"), DOXY_FN(EnvironmentBase,CloneSelf))
                    .def("Clone",&PyEnvironmentBase::Clone,args("reference","options"), DOXY_FN(EnvironmentBase,Clone))
                    .def("SetCollisionChecker",&PyEnvironmentBase::SetCollisionChecker,args("collisionchecker"), DOXY_FN(EnvironmentBase,SetCollisionChecker))
                    .def("GetCollisionChecker",&PyEnvironmentBase::GetCollisionChecker, DOXY_FN(EnvironmentBase,GetCollisionChecker))

                    .def("CheckCollision",pcolb,args("body"), DOXY_FN(EnvironmentBase,CheckCollision "KinBodyConstPtr; CollisionReportPtr"))
                    .def("CheckCollision",pcolbr,args("body","report"), DOXY_FN(EnvironmentBase,CheckCollision "KinBodyConstPtr; CollisionReportPtr"))
                    .def("CheckCollision",pcolbb,args("body1","body2"), DOXY_FN(EnvironmentBase,CheckCollision "KinBodyConstPtr; KinBodyConstPtr; CollisionReportPtr"))
                    .def("CheckCollision",pcolbbr,args("body1","body2","report"), DOXY_FN(EnvironmentBase,CheckCollision "KinBodyConstPtr; KinBodyConstPtr; CollisionReportPtr"))
                    .def("CheckCollision",pcoll,args("link"), DOXY_FN(EnvironmentBase,CheckCollision "KinBody::LinkConstPtr; CollisionReportPtr"))
                    .def("CheckCollision",pcollr,args("link","report"), DOXY_FN(EnvironmentBase,CheckCollision "KinBody::LinkConstPtr; CollisionReportPtr"))
                    .def("CheckCollision",pcolll,args("link1","link2"), DOXY_FN(EnvironmentBase,CheckCollision "KinBody::LinkConstPtr; KinBody::LinkConstPtr; CollisionReportPtr"))
                    .def("CheckCollision",pcolllr,args("link1","link2","report"), DOXY_FN(EnvironmentBase,CheckCollision "KinBody::LinkConstPtr; KinBody::LinkConstPtr; CollisionReportPtr"))
                    .def("CheckCollision",pcollb,args("link","body"), DOXY_FN(EnvironmentBase,CheckCollision "KinBody::LinkConstPtr; KinBodyConstPtr; CollisionReportPtr"))
                    .def("CheckCollision",pcollbr,args("link","body","report"), DOXY_FN(EnvironmentBase,CheckCollision "KinBody::LinkConstPtr; KinBodyConstPtr; CollisionReportPtr"))
                    .def("CheckCollision",pcolle,args("link","bodyexcluded","linkexcluded"), DOXY_FN(EnvironmentBase,CheckCollision "KinBody::LinkConstPtr; const std::vector; const std::vector; CollisionReportPtr"))
                    .def("CheckCollision",pcoller,args("link","bodyexcluded","linkexcluded","report"), DOXY_FN(EnvironmentBase,CheckCollision "KinBody::LinkConstPtr; const std::vector; const std::vector; CollisionReportPtr"))
                    .def("CheckCollision",pcolbe,args("body","bodyexcluded","linkexcluded"), DOXY_FN(EnvironmentBase,CheckCollision "KinBodyConstPtr; const std::vector; const std::vector; CollisionReportPtr"))
                    .def("CheckCollision",pcolber,args("body","bodyexcluded","linkexcluded","report"), DOXY_FN(EnvironmentBase,CheckCollision "KinBodyConstPtr; const std::vector; const std::vector; CollisionReportPtr"))
                    .def("CheckCollision",pcolyb,args("ray","body"), DOXY_FN(EnvironmentBase,CheckCollision "const RAY; KinBodyConstPtr; CollisionReportPtr"))
                    .def("CheckCollision",pcolybr,args("ray","body","report"), DOXY_FN(EnvironmentBase,CheckCollision "const RAY; KinBodyConstPtr; CollisionReportPtr"))
                    .def("CheckCollision",pcoly,args("ray"), DOXY_FN(EnvironmentBase,CheckCollision "const RAY; CollisionReportPtr"))
                    .def("CheckCollision",pcolyr,args("ray"), DOXY_FN(EnvironmentBase,CheckCollision "const RAY; CollisionReportPtr"))
                    .def("CheckCollisionRays",&PyEnvironmentBase::CheckCollisionRays,
                         CheckCollisionRays_overloads(args("rays","body","front_facing_only"),
                                                      "Check if any rays hit the body and returns their contact points along with a vector specifying if a collision occured or not. Rays is a Nx6 array, first 3 columsn are position, last 3 are direction*range."))
                    .def("LoadURI",&PyEnvironmentBase::LoadURI,LoadURI_overloads(args("filename","atts"), DOXY_FN(EnvironmentBase,LoadURI)))
                    .def("Load",load1,args("filename"), DOXY_FN(EnvironmentBase,Load))
                    .def("Load",load2,args("filename","atts"), DOXY_FN(EnvironmentBase,Load))
                    .def("LoadData",loaddata1,args("data"), DOXY_FN(EnvironmentBase,LoadData))
                    .def("LoadData",loaddata2,args("data","atts"), DOXY_FN(EnvironmentBase,LoadData))
                    .def("Save",&PyEnvironmentBase::Save,Save_overloads(args("filename","options","atts"), DOXY_FN(EnvironmentBase,Save)))
                    .def("WriteToMemory",&PyEnvironmentBase::WriteToMemory,WriteToMemory_overloads(args("filetype","options","atts"), DOXY_FN(EnvironmentBase,WriteToMemory)))
                    .def("ReadRobotURI",readrobotxmlfile1,args("filename"), DOXY_FN(EnvironmentBase,ReadRobotURI "const std::string"))
                    .def("ReadRobotXMLFile",readrobotxmlfile1,args("filename"), DOXY_FN(EnvironmentBase,ReadRobotURI "const std::string"))
                    .def("ReadRobotURI",readrobotxmlfile2,args("filename","atts"), DOXY_FN(EnvironmentBase,ReadRobotURI "RobotBasePtr; const std::string; const AttributesList"))
                    .def("ReadRobotXMLFile",readrobotxmlfile2,args("filename","atts"), DOXY_FN(EnvironmentBase,ReadRobotURI "RobotBasePtr; const std::string; const AttributesList"))
                    .def("ReadRobotData",readrobotxmldata1,args("data"), DOXY_FN(EnvironmentBase,ReadRobotData "RobotBasePtr; const std::string; const AttributesList"))
                    .def("ReadRobotXMLData",readrobotxmldata1,args("data"), DOXY_FN(EnvironmentBase,ReadRobotData "RobotBasePtr; const std::string; const AttributesList"))
                    .def("ReadRobotData",readrobotxmldata2,args("data","atts"), DOXY_FN(EnvironmentBase,ReadRobotData "RobotBasePtr; const std::string; const AttributesList"))
                    .def("ReadRobotXMLData",readrobotxmldata2,args("data","atts"), DOXY_FN(EnvironmentBase,ReadRobotData "RobotBasePtr; const std::string; const AttributesList"))
                    .def("ReadKinBodyURI",readkinbodyxmlfile1,args("filename"), DOXY_FN(EnvironmentBase,ReadKinBodyURI "const std::string"))
                    .def("ReadKinBodyXMLFile",readkinbodyxmlfile1,args("filename"), DOXY_FN(EnvironmentBase,ReadKinBodyURI "const std::string"))
                    .def("ReadKinBodyURI",readkinbodyxmlfile2,args("filename","atts"), DOXY_FN(EnvironmentBase,ReadKinBodyURI "KinBody; const std::string; const AttributesList"))
                    .def("ReadKinBodyXMLFile",readkinbodyxmlfile2,args("filename","atts"), DOXY_FN(EnvironmentBase,ReadKinBodyURI "KinBody; const std::string; const AttributesList"))
                    .def("ReadKinBodyData",readkinbodyxmldata1,args("data"), DOXY_FN(EnvironmentBase,ReadKinBodyData "KinBodyPtr; const std::string; const AttributesList"))
                    .def("ReadKinBodyXMLData",readkinbodyxmldata1,args("data"), DOXY_FN(EnvironmentBase,ReadKinBodyData "KinBodyPtr; const std::string; const AttributesList"))
                    .def("ReadKinBodyData",readkinbodyxmldata2,args("data","atts"), DOXY_FN(EnvironmentBase,ReadKinBodyData "KinBodyPtr; const std::string; const AttributesList"))
                    .def("ReadKinBodyXMLData",readkinbodyxmldata2,args("data","atts"), DOXY_FN(EnvironmentBase,ReadKinBodyData "KinBodyPtr; const std::string; const AttributesList"))
                    .def("ReadInterfaceURI",readinterfacexmlfile1,args("filename"), DOXY_FN(EnvironmentBase,ReadInterfaceURI "InterfaceBasePtr; InterfaceType; const std::string; const AttributesList"))
                    .def("ReadInterfaceXMLFile",readinterfacexmlfile1,args("filename"), DOXY_FN(EnvironmentBase,ReadInterfaceURI "InterfaceBasePtr; InterfaceType; const std::string; const AttributesList"))
                    .def("ReadInterfaceURI",readinterfacexmlfile2,args("filename","atts"), DOXY_FN(EnvironmentBase,ReadInterfaceURI "InterfaceBasePtr; InterfaceType; const std::string; const AttributesList"))
                    .def("ReadInterfaceXMLFile",readinterfacexmlfile2,args("filename","atts"), DOXY_FN(EnvironmentBase,ReadInterfaceURI "InterfaceBasePtr; InterfaceType; const std::string; const AttributesList"))
                    .def("ReadTrimeshURI",readtrimeshfile1,args("filename"), DOXY_FN(EnvironmentBase,ReadTrimeshURI))
                    .def("ReadTrimeshURI",readtrimeshfile2,args("filename","atts"), DOXY_FN(EnvironmentBase,ReadTrimeshURI))
                    .def("ReadTrimeshFile",readtrimeshfile1,args("filename"), DOXY_FN(EnvironmentBase,ReadTrimeshURI))
                    .def("ReadTrimeshFile",readtrimeshfile2,args("filename","atts"), DOXY_FN(EnvironmentBase,ReadTrimeshURI))
                    .def("ReadTrimeshData",readtrimeshdata1,args("data", "formathint"), DOXY_FN(EnvironmentBase,ReadTrimeshData))
                    .def("ReadTrimeshData",readtrimeshdata2,args("data","formathint","atts"), DOXY_FN(EnvironmentBase,ReadTrimeshData))
                    .def("Add", &PyEnvironmentBase::Add, Add_overloads(args("interface","anonymous","cmdargs"), DOXY_FN(EnvironmentBase,Add)))
                    .def("AddKinBody",addkinbody1,args("body"), DOXY_FN(EnvironmentBase,AddKinBody))
                    .def("AddKinBody",addkinbody2,args("body","anonymous"), DOXY_FN(EnvironmentBase,AddKinBody))
                    .def("AddRobot",addrobot1,args("robot"), DOXY_FN(EnvironmentBase,AddRobot))
                    .def("AddRobot",addrobot2,args("robot","anonymous"), DOXY_FN(EnvironmentBase,AddRobot))
                    .def("AddSensor",addsensor1,args("sensor"), DOXY_FN(EnvironmentBase,AddSensor))
                    .def("AddSensor",addsensor2,args("sensor","anonymous"), DOXY_FN(EnvironmentBase,AddSensor))
                    .def("AddViewer",addsensor2,args("sensor","anonymous"), DOXY_FN(EnvironmentBase,AddViewer))
                    .def("RemoveKinBody",&PyEnvironmentBase::RemoveKinBody,args("body"), DOXY_FN(EnvironmentBase,RemoveKinBody))
                    .def("RemoveKinBodyByName",&PyEnvironmentBase::RemoveKinBodyByName,args("name"), DOXY_FN(EnvironmentBase,RemoveKinBodyByName))
                    .def("Remove",&PyEnvironmentBase::Remove,args("interface"), DOXY_FN(EnvironmentBase,Remove))
                    .def("GetKinBody",&PyEnvironmentBase::GetKinBody,args("name"), DOXY_FN(EnvironmentBase,GetKinBody))
                    .def("GetRobot",&PyEnvironmentBase::GetRobot,args("name"), DOXY_FN(EnvironmentBase,GetRobot))
                    .def("GetSensor",&PyEnvironmentBase::GetSensor,args("name"), DOXY_FN(EnvironmentBase,GetSensor))
                    .def("GetBodyFromEnvironmentId",&PyEnvironmentBase::GetBodyFromEnvironmentId, DOXY_FN(EnvironmentBase,GetBodyFromEnvironmentId))
                    .def("AddModule",&PyEnvironmentBase::AddModule,args("module","args"), DOXY_FN(EnvironmentBase,AddModule))
                    .def("LoadProblem",&PyEnvironmentBase::AddModule,args("module","args"), DOXY_FN(EnvironmentBase,AddModule))
                    .def("RemoveProblem",&PyEnvironmentBase::RemoveProblem,args("prob"), DOXY_FN(EnvironmentBase,RemoveProblem))
                    .def("GetModules",&PyEnvironmentBase::GetModules, DOXY_FN(EnvironmentBase,GetModules))
                    .def("GetLoadedProblems",&PyEnvironmentBase::GetModules, DOXY_FN(EnvironmentBase,GetModules))
                    .def("SetPhysicsEngine",&PyEnvironmentBase::SetPhysicsEngine,args("physics"), DOXY_FN(EnvironmentBase,SetPhysicsEngine))
                    .def("GetPhysicsEngine",&PyEnvironmentBase::GetPhysicsEngine, DOXY_FN(EnvironmentBase,GetPhysicsEngine))
                    .def("RegisterBodyCallback",&PyEnvironmentBase::RegisterBodyCallback,args("callback"), DOXY_FN(EnvironmentBase,RegisterBodyCallback))
                    .def("RegisterCollisionCallback",&PyEnvironmentBase::RegisterCollisionCallback,args("callback"), DOXY_FN(EnvironmentBase,RegisterCollisionCallback))
                    .def("HasRegisteredCollisionCallbacks",&PyEnvironmentBase::HasRegisteredCollisionCallbacks,DOXY_FN(EnvironmentBase,HasRegisteredCollisionCallbacks))
                    .def("StepSimulation",&PyEnvironmentBase::StepSimulation,args("timestep"), DOXY_FN(EnvironmentBase,StepSimulation))
                    .def("StartSimulation",&PyEnvironmentBase::StartSimulation,StartSimulation_overloads(args("timestep","realtime"), DOXY_FN(EnvironmentBase,StartSimulation)))
                    .def("StopSimulation",&PyEnvironmentBase::StopSimulation, StopSimulation_overloads(args("shutdownthread"), DOXY_FN(EnvironmentBase,StopSimulation)))
                    .def("GetSimulationTime",&PyEnvironmentBase::GetSimulationTime, DOXY_FN(EnvironmentBase,GetSimulationTime))
                    .def("IsSimulationRunning",&PyEnvironmentBase::IsSimulationRunning, DOXY_FN(EnvironmentBase,IsSimulationRunning))
                    .def("Lock",Lock1,"Locks the environment mutex.")
                    .def("Lock",Lock2,args("timeout"), "Locks the environment mutex with a timeout.")
                    .def("Unlock",&PyEnvironmentBase::Unlock,"Unlocks the environment mutex.")
                    .def("TryLock",&PyEnvironmentBase::TryLock,"Tries to locks the environment mutex, returns false if it failed.")
                    .def("LockPhysics",Lock1,args("lock"), "Locks the environment mutex.")
                    .def("LockPhysics",Lock2,args("lock","timeout"), "Locks the environment mutex with a timeout.")
                    .def("SetViewer",&PyEnvironmentBase::SetViewer,SetViewer_overloads(args("viewername","showviewer"), "Attaches the viewer and starts its thread"))
                    .def("SetDefaultViewer",&PyEnvironmentBase::SetDefaultViewer,SetDefaultViewer_overloads(args("showviewer"), "Attaches the default viewer (controlled by environment variables and internal settings) and starts its thread"))
                    .def("GetViewer",&PyEnvironmentBase::GetViewer, DOXY_FN(EnvironmentBase,GetViewer))
                    .def("plot3",&PyEnvironmentBase::plot3,plot3_overloads(args("points","pointsize","colors","drawstyle"), DOXY_FN(EnvironmentBase,plot3 "const float; int; int; float; const float; int, bool")))
                    .def("drawlinestrip",&PyEnvironmentBase::drawlinestrip,drawlinestrip_overloads(args("points","linewidth","colors","drawstyle"), DOXY_FN(EnvironmentBase,drawlinestrip "const float; int; int; float; const float")))
                    .def("drawlinelist",&PyEnvironmentBase::drawlinelist,drawlinelist_overloads(args("points","linewidth","colors","drawstyle"), DOXY_FN(EnvironmentBase,drawlinelist "const float; int; int; float; const float")))
                    .def("drawarrow",&PyEnvironmentBase::drawarrow,drawarrow_overloads(args("p1","p2","linewidth","color"), DOXY_FN(EnvironmentBase,drawarrow)))
                    .def("drawbox",&PyEnvironmentBase::drawbox,drawbox_overloads(args("pos","extents","color"), DOXY_FN(EnvironmentBase,drawbox)))
                    .def("drawplane",drawplane1,args("transform","extents","texture"), DOXY_FN(EnvironmentBase,drawplane))
                    .def("drawplane",drawplane2,args("transform","extents","texture"), DOXY_FN(EnvironmentBase,drawplane))
                    .def("drawtrimesh",&PyEnvironmentBase::drawtrimesh,drawtrimesh_overloads(args("points","indices","colors"), DOXY_FN(EnvironmentBase,drawtrimesh "const float; int; const int; int; const boost::multi_array")))
                    .def("GetRobots",&PyEnvironmentBase::GetRobots, DOXY_FN(EnvironmentBase,GetRobots))
                    .def("GetBodies",&PyEnvironmentBase::GetBodies, DOXY_FN(EnvironmentBase,GetBodies))
                    .def("GetSensors",&PyEnvironmentBase::GetSensors, DOXY_FN(EnvironmentBase,GetSensors))
                    .def("UpdatePublishedBodies",&PyEnvironmentBase::UpdatePublishedBodies, DOXY_FN(EnvironmentBase,UpdatePublishedBodies))
                    .def("GetPublishedBody",&PyEnvironmentBase::GetPublishedBody, GetPublishedBody_overloads(args("name", "timeout"), DOXY_FN(EnvironmentBase,GetPublishedBody)))
                    .def("GetPublishedBodies",&PyEnvironmentBase::GetPublishedBodies, GetPublishedBodies_overloads(args("timeout"), DOXY_FN(EnvironmentBase,GetPublishedBodies)))
                    .def("GetPublishedBodyJointValues",&PyEnvironmentBase::GetPublishedBodyJointValues, GetPublishedBodyJointValues_overloads(args("name", "timeout"), DOXY_FN(EnvironmentBase,GetPublishedBodyJointValues)))
                    .def("GetPublishedBodyTransformsMatchingPrefix",&PyEnvironmentBase::GetPublishedBodyTransformsMatchingPrefix, GetPublishedBodyTransformsMatchingPrefix_overloads(args("prefix", "timeout"), DOXY_FN(EnvironmentBase,GetPublishedBodyTransformsMatchingPrefix)))
                    .def("Triangulate",&PyEnvironmentBase::Triangulate,args("body"), DOXY_FN(EnvironmentBase,Triangulate))
                    .def("TriangulateScene",&PyEnvironmentBase::TriangulateScene,args("options","name"), DOXY_FN(EnvironmentBase,TriangulateScene))
                    .def("SetDebugLevel",&PyEnvironmentBase::SetDebugLevel,args("level"), DOXY_FN(EnvironmentBase,SetDebugLevel))
                    .def("GetDebugLevel",&PyEnvironmentBase::GetDebugLevel, DOXY_FN(EnvironmentBase,GetDebugLevel))
                    .def("GetHomeDirectory",&PyEnvironmentBase::GetHomeDirectory, DOXY_FN(EnvironmentBase,GetHomeDirectory))
                    .def("SetUserData",setuserdata1,args("data"), DOXY_FN(InterfaceBase,SetUserData))
                    .def("SetUserData",setuserdata2,args("data"), DOXY_FN(InterfaceBase,SetUserData))
                    .def("GetUserData",&PyEnvironmentBase::GetUserData, DOXY_FN(InterfaceBase,GetUserData))
                    .def("GetUnit",&PyEnvironmentBase::GetUnit, DOXY_FN(EnvironmentBase,GetUnit))
                    .def("SetUnit",&PyEnvironmentBase::SetUnit, args("unitname","unitmult"),  DOXY_FN(EnvironmentBase,SetUnit))
                    .def("__enter__",&PyEnvironmentBase::__enter__)
                    .def("__exit__",&PyEnvironmentBase::__exit__)
                    .def("__eq__",&PyEnvironmentBase::__eq__)
                    .def("__ne__",&PyEnvironmentBase::__ne__)
                    .def("__repr__",&PyEnvironmentBase::__repr__)
                    .def("__str__",&PyEnvironmentBase::__str__)
                    .def("__unicode__",&PyEnvironmentBase::__unicode__)
        ;

        object selectionoptions = enum_<EnvironmentBase::SelectionOptions>("SelectionOptions" DOXY_ENUM(SelectionOptions))
                                  .value("NoRobots",EnvironmentBase::SO_NoRobots)
                                  .value("Robots",EnvironmentBase::SO_Robots)
                                  .value("Everything",EnvironmentBase::SO_Everything)
                                  .value("Body",EnvironmentBase::SO_Body)
                                  .value("AllExceptBody",EnvironmentBase::SO_AllExceptBody)
        ;
        env.attr("TriangulateOptions") = selectionoptions;
    }

    {
        scope options = class_<DummyStruct>("options")
                        .add_static_property("returnTransformQuaternion",GetReturnTransformQuaternions,SetReturnTransformQuaternions);
    }

    scope().attr("__version__") = OPENRAVE_VERSION_STRING;
    scope().attr("__author__") = "Rosen Diankov";
    scope().attr("__copyright__") = "2009-2012 Rosen Diankov (rosen.diankov@gmail.com)";
    scope().attr("__license__") = "Lesser GPL";
    scope().attr("__docformat__") = "restructuredtext";

    openravepy::init_openravepy_global();
    openravepy::InitPlanningUtils();

    openravepy::init_openravepy_collisionchecker();
    openravepy::init_openravepy_controller();
    openravepy::init_openravepy_ikparameterization();
    openravepy::init_openravepy_iksolver();
    openravepy::init_openravepy_kinbody();
    openravepy::init_openravepy_robot();
    openravepy::init_openravepy_module();
    openravepy::init_openravepy_physicsengine();
    openravepy::init_openravepy_planner();
    openravepy::init_openravepy_trajectory();
    openravepy::init_openravepy_sensor();
    openravepy::init_openravepy_sensorsystem();
    openravepy::init_openravepy_spacesampler();
    openravepy::init_openravepy_viewer();

    def("RaveGetEnvironmentId",openravepy::RaveGetEnvironmentId,DOXY_FN1(RaveGetEnvironmentId));
    def("RaveGetEnvironment",openravepy::RaveGetEnvironment,DOXY_FN1(RaveGetEnvironment));
    def("RaveGetEnvironments",openravepy::RaveGetEnvironments,DOXY_FN1(RaveGetEnvironments));
    def("RaveCreateInterface",openravepy::RaveCreateInterface,args("env","type","name"),DOXY_FN1(RaveCreateInterface));
}
