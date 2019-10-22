// Copyright (C) 2006-2012 Rosen Diankov <rosen.diankov@gmail.com>
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

// shouldn't include openrave.h!
#ifndef OPENRAVE_BOOST_PYTHON_BINDINGS
#define OPENRAVE_BOOST_PYTHON_BINDINGS

#include "../python3/lib/python3.7/site-packages/numpy/core/include/numpy/arrayobject.h"
#include "../python3/lib/python3.7/site-packages/numpy/core/include/numpy/arrayscalars.h"
#include <Python.h>
#include "include/boost/python.hpp"
#include "include/boost/python/numpy.hpp"
// #include <boost/array.hpp>
// #include <boost/multi_array.hpp>
// #include <boost/shared_ptr.hpp>
// #include <boost/format.hpp>
// #include <boost/python.hpp>
// #include <boost/assert.hpp>
// #include <boost/cstdint.hpp>
// #include <boost/version.hpp>
#include <stdint.h>

// #ifdef _MSC_VER
// #include <boost/typeof/std/string.hpp>
// #include <boost/typeof/std/vector.hpp>
// #include <boost/typeof/std/list.hpp>
// #include <boost/typeof/std/map.hpp>
// #include <boost/typeof/std/set.hpp>
// #include <boost/typeof/std/string.hpp>

// #define FOREACH(it, v) for(BOOST_TYPEOF(v) ::iterator it = (v).begin(); it != (v).end(); (it)++)
// #define FOREACH_NOINC(it, v) for(BOOST_TYPEOF(v) ::iterator it = (v).begin(); it != (v).end(); )

// #define FOREACHC(it, v) for(BOOST_TYPEOF(v) ::const_iterator it = (v).begin(); it != (v).end(); (it)++)
// #define FOREACHC_NOINC(it, v) for(BOOST_TYPEOF(v) ::const_iterator it = (v).begin(); it != (v).end(); )
// #define RAVE_REGISTER_BOOST

// #else
#include <string>
#include <vector>
#include <list>
#include <map>
#include <set>
#include <string>
#include <stdexcept>

// apparently there's a problem with higher versions of C++
#if __cplusplus > 199711L || defined(__GXX_EXPERIMENTAL_CXX0X__)
#include <typeinfo>
#define FOREACH(it, v) for(decltype((v).begin()) it = (v).begin(); it != (v).end(); (it)++)
#define FOREACH_NOINC(it, v) for(decltype((v).begin()) it = (v).begin(); it != (v).end(); )
#else // __cplusplus > 199711L || defined(__GXX_EXPERIMENTAL_CXX0X__)
#define FOREACH(it, v) for(typeof((v).begin())it = (v).begin(); it != (v).end(); (it)++)
#define FOREACH_NOINC(it, v) for(typeof((v).begin())it = (v).begin(); it != (v).end(); )
#endif // __cplusplus > 199711L || defined(__GXX_EXPERIMENTAL_CXX0X__)

#define FOREACHC FOREACH
#define FOREACHC_NOINC FOREACH_NOINC

// #endif

#include <complex>
#include <algorithm>

// is_none is not supported by older versions of python
#if BOOST_VERSION >= 104300
#define IS_PYTHONOBJECT_NONE(o) (o).is_none()
#else // 
#define IS_PYTHONOBJECT_NONE(o) (!!(o))
#endif // BOOST_VERSION >= 104300

namespace openravepy {

using namespace boost::python;
namespace p = boost::python;
using p::object;
using p::extract;
namespace np = p::numpy;
using bpndarray = np::ndarray; // boost::python::numeric::array;

// char* PyString_AsString(PyObject* pystring)
// {
//     char *pchar = nullptr;
//     if (PyUnicode_Check(pystring)) {
//         PyObject * temp_bytes = PyUnicode_AsEncodedString(pystring, "UTF-8", "strict"); // Owned reference
//         if (temp_bytes != NULL) {
//             pchar = PyBytes_AS_STRING(temp_bytes); // Borrowed pointer
//             pchar = strdup(pchar);
//             Py_DECREF(temp_bytes);
//         } else {
//             BOOST_ASSERT(0); // TODO: Handle encoding error.
//         }
//     }
//     else if (PyBytes_Check(pystring)) {
//         pchar = PyBytes_AS_STRING(pystring); // Borrowed pointer
//         pchar = strdup(pchar);
//     }
//     return pchar;
// }

// https://py3c.readthedocs.io/en/latest/reference.html
// https://docs.python.org/3/c-api/unicode.html#c.PyUnicode_AsUTF8
const char* PyString_AsString(PyObject* pystring) 
{
    return PyUnicode_AsUTF8(pystring);
}

inline boost::python::object ConvertStringToUnicode(const std::string& s)
{
    return boost::python::object(boost::python::handle<>(PyUnicode_Decode(s.c_str(),s.size(), "utf-8", NULL)));
}

class PyVoidHandle
{
public:
    PyVoidHandle() {
    }
    PyVoidHandle(boost::shared_ptr<void> handle) : _handle(handle) {
    }
    void Close() {
        _handle.reset();
    }
    boost::shared_ptr<void> _handle;
};

class PyVoidHandleConst
{
public:
    PyVoidHandleConst() {
    }
    PyVoidHandleConst(boost::shared_ptr<void const> handle) : _handle(handle) {
    }
    void Close() {
        _handle.reset();
    }
    boost::shared_ptr<void const> _handle;
};

template <typename T>
inline std::vector<T> ExtractArray(const object& o)
{
    if( IS_PYTHONOBJECT_NONE(o) ) {
        return std::vector<T>();
    }
    std::vector<T> v(len(o));
    for(size_t i = 0; i < v.size(); ++i) {
        v[i] = boost::python::extract<T>(o[i]);
    }
    return v;
}

template <typename T>
inline std::set<T> ExtractSet(const object& o)
{
    std::set<T> v;
    size_t nlen = len(o);
    for(size_t i = 0; i < nlen; ++i) {
        v.insert(extract<T>(o[i]));
    }
    return v;
}

template <typename T>
struct exception_translator
{
    exception_translator() {
        register_exception_translator<T>(&exception_translator::translate);

        //Register custom r-value converter
        //There are situations, where we have to pass the exception back to
        //C++ library. This will do the trick
        converter::registry::push_back( &exception_translator::convertible, &exception_translator::construct, type_id<T>() );
    }

    static void translate( const T& err )
    {
        object pimpl_err( err );
        object pyerr_class = pimpl_err.attr( "py_err_class" );
        object pyerr = pyerr_class( pimpl_err );
        PyErr_SetObject( pyerr_class.ptr(), incref( pyerr.ptr() ) );
    }

    //Sometimes, exceptions should be passed back to the library.
    static void* convertible(PyObject* py_obj){
        if( 1 != PyObject_IsInstance( py_obj, PyExc_Exception ) ) {
            return 0;
        }

        if( !PyObject_HasAttrString( py_obj, "_pimpl" ) ) {
            return 0;
        }

        object pyerr( handle<>( borrowed( py_obj ) ) );
        object pimpl = getattr( pyerr, "_pimpl" );
        extract<T> type_checker( pimpl );
        if( !type_checker.check() ) {
            return 0;
        }
        return py_obj;
    }

    static void construct( PyObject* py_obj, converter::rvalue_from_python_stage1_data* data)
    {
        typedef converter::rvalue_from_python_storage<T> storage_t;

        object pyerr( handle<>( borrowed( py_obj ) ) );
        object pimpl = getattr( pyerr, "_pimpl" );

        storage_t* the_storage = reinterpret_cast<storage_t*>( data );
        void* memory_chunk = the_storage->storage.bytes;
        new (memory_chunk) T( extract<T>(pimpl) );
        data->convertible = memory_chunk;
    }
};

template<typename T>
struct float_from_number
{
    float_from_number()
    {
        converter::registry::push_back(&convertible, &construct, type_id<T>());
    }

    static void* convertible( PyObject* obj)
    {
        return PyNumber_Check(obj) ? obj : NULL;
    }

    static void construct(PyObject* _obj, converter::rvalue_from_python_stage1_data* data)
    {
        PyObject* tmp = PyNumber_Float(_obj);
        T* storage = (T*)((converter::rvalue_from_python_storage<T>*)data)->storage.bytes;
        *storage = boost::python::extract<T>(tmp);
        Py_DECREF(tmp);
        data->convertible = storage;
    }
};

template<typename T>
struct int_from_number
{
    int_from_number()
    {
        converter::registry::push_back(&convertible, &construct, type_id<T>());
    }

    static void* convertible( PyObject* obj)
    {
        return PyNumber_Check(obj) ? obj : NULL;
    }

    static void construct(PyObject* _obj, converter::rvalue_from_python_stage1_data* data)
    {
        PyObject* tmp = PyNumber_Long(_obj);
        T* storage = (T*)((converter::rvalue_from_python_storage<T>*)data)->storage.bytes;
        *storage = boost::python::extract<T>(tmp);
        Py_DECREF(tmp);
        data->convertible = storage;
    }
};

inline std::string GetPyErrorString()
{
    PyObject *error, *value, *traceback, *string;
    PyErr_Fetch(&error, &value, &traceback);
    PyErr_NormalizeException(&error, &value, &traceback);
    std::string s;
    if(error != NULL) {
        PyObject* pystring = PyObject_Str(value);
        if(pystring != NULL) {
            s.assign(PyString_AsString(pystring));
            Py_DECREF(pystring);
        }
    }
    // Does nothing when the ptr is nullptr
    Py_DECREF(error);
    Py_DECREF(value);
    Py_DECREF(traceback);

    return s;
}

/// should call in the beginning of all BOOST_PYTHON_MODULE
void init_python_bindings();

#ifdef OPENRAVE_BININGS_PYARRAY

template <typename T>
inline bpndarray toPyArrayN(const T* data, size_t N)
{
    np::dtype dt = np::dtype::get_builtin<T>();
    if( N == 0 ) {
        return np::array(boost::python::list(), dt);
    }

    // https://www.boost.org/doc/libs/1_71_0/libs/python/doc/html/numpy/reference/ndarray.html
    p::tuple shape = p::make_tuple(N);
    p::tuple stride = p::make_tuple(N) ;
    p::object own;
    np::ndarray data_ex = np::from_data(data, dt, shape, stride, own);
    return data_ex;
    // npy_intp dims[] = {npy_intp(N)};
    // PyObject *pyvalues = PyArray_SimpleNew(1,dims, PyArray_FLOAT);
    // if( pvalues != NULL ) {
    //     memcpy(PyArray_DATA(pyvalues),pvalues,N*sizeof(float));
    // }
    // return static_cast<bpndarray>(handle<>(pyvalues));
}

// inline bpndarray toPyArrayN(const float* pvalues, std::vector<npy_intp>& dims)
// {
//     np::dtype dt = np::dtype::get_builtin<float>();
//     if( dims.size() == 0 ) {
//         return np::array(boost::python::list(), dt);
//     }
    
//     size_t totalsize = 1;
//     FOREACH(it,dims) {
//         totalsize *= *it;
//     }
//     if( totalsize == 0 ) {
//         return np::array(boost::python::list(), dt);
//     }
//     PyObject *pyvalues = PyArray_SimpleNew(dims.size(),&dims[0], PyArray_FLOAT);
//     if( pvalues != NULL ) {
//         memcpy(PyArray_DATA(pyvalues),pvalues,totalsize*sizeof(float));
//     }
//     return static_cast<bpndarray>(handle<>(pyvalues));
// }

// inline bpndarray toPyArrayN(const double* pvalues, size_t N)
// {
//     if( N == 0 ) {
//         return static_cast<bpndarray>(bpndarray(boost::python::list()).astype("f8"));
//     }
//     npy_intp dims[] = {npy_intp(N)};
//     PyObject *pyvalues = PyArray_SimpleNew(1,dims, PyArray_DOUBLE);
//     if( pvalues != NULL ) {
//         memcpy(PyArray_DATA(pyvalues),pvalues,N*sizeof(double));
//     }
//     return static_cast<bpndarray>(handle<>(pyvalues));
// }

// inline bpndarray toPyArrayN(const double* pvalues, std::vector<npy_intp>& dims)
// {
//     if( dims.size() == 0 ) {
//         return static_cast<bpndarray>(bpndarray(boost::python::list()).astype("f8"));
//     }
//     size_t totalsize = 1;
//     FOREACH(it,dims) {
//         totalsize *= *it;
//     }
//     if( totalsize == 0 ) {
//         return static_cast<bpndarray>(bpndarray(boost::python::list()).astype("f8"));
//     }
//     PyObject *pyvalues = PyArray_SimpleNew(dims.size(),&dims[0], PyArray_DOUBLE);
//     if( pvalues != NULL ) {
//         memcpy(PyArray_DATA(pyvalues),pvalues,totalsize*sizeof(double));
//     }
//     return static_cast<bpndarray>(handle<>(pyvalues));
// }

// inline bpndarray toPyArrayN(const uint8_t* pvalues, size_t N)
// {
//     if( N == 0 ) {
//         return static_cast<bpndarray>(bpndarray(boost::python::list()).astype("u1"));
//     }
//     npy_intp dims[] = {npy_intp(N)};
//     PyObject *pyvalues = PyArray_SimpleNew(1,&dims[0], PyArray_UINT8);
//     if( pvalues != NULL ) {
//         memcpy(PyArray_DATA(pyvalues),pvalues,N*sizeof(uint8_t));
//     }
//     return static_cast<bpndarray>(handle<>(pyvalues));
// }

// inline bpndarray toPyArrayN(const uint8_t* pvalues, std::vector<npy_intp>& dims)
// {
//     if( dims.size() == 0 ) {
//         return static_cast<bpndarray>(bpndarray(boost::python::list()).astype("u1"));
//     }
//     size_t totalsize = 1;
//     for(size_t i = 0; i < dims.size(); ++i) {
//         totalsize *= dims[i];
//     }
//     if( totalsize == 0 ) {
//         return static_cast<bpndarray>(bpndarray(boost::python::list()).astype("u1"));
//     }
//     PyObject *pyvalues = PyArray_SimpleNew(dims.size(),&dims[0], PyArray_UINT8);
//     if( pvalues != NULL ) {
//         memcpy(PyArray_DATA(pyvalues),pvalues,totalsize*sizeof(uint8_t));
//     }
//     return static_cast<bpndarray>(handle<>(pyvalues));
// }

// inline bpndarray toPyArrayN(const int* pvalues, size_t N)
// {
//     if( N == 0 ) {
//         return static_cast<bpndarray>(bpndarray(boost::python::list()).astype("i4"));
//     }
//     npy_intp dims[] = {npy_intp(N)};
//     PyObject *pyvalues = PyArray_SimpleNew(1,&dims[0], PyArray_INT32);
//     if( pvalues != NULL ) {
//         memcpy(PyArray_DATA(pyvalues),pvalues,N*sizeof(int));
//     }
//     return static_cast<bpndarray>(handle<>(pyvalues));
// }

// inline bpndarray toPyArrayN(const uint32_t* pvalues, size_t N)
// {
//     if( N == 0 ) {
//         return static_cast<bpndarray>(bpndarray(boost::python::list()).astype("u4"));
//     }
//     npy_intp dims[] = {npy_intp(N)};
//     PyObject *pyvalues = PyArray_SimpleNew(1,&dims[0], PyArray_UINT32);
//     if( pvalues != NULL ) {
//         memcpy(PyArray_DATA(pyvalues),pvalues,N*sizeof(uint32_t));
//     }
//     return static_cast<bpndarray>(handle<>(pyvalues));
// }

template <typename T>
inline object toPyList(const std::vector<T>& v)
{
    boost::python::list lvalues;
    FOREACHC(it,v) {
        lvalues.append(object(*it));
    }
    return lvalues;
}

template <typename T>
inline bpndarray toPyArray(const std::vector<T>& v)
{
    if( v.size() == 0 ) {
        return toPyArrayN((T*)NULL,0);
    }
    return toPyArrayN(&v[0],v.size());
}

template <typename T>
inline bpndarray toPyArray(const std::vector<T>& v, std::vector<npy_intp>& dims)
{
    if( v.size() == 0 ) {
        return toPyArrayN((T*)NULL,dims);
    }
    size_t totalsize = 1;
    FOREACH(it,dims)
    totalsize *= *it;
    BOOST_ASSERT(totalsize == v.size());
    return toPyArrayN(&v[0],dims);
}

template <typename T, int N>
inline bpndarray toPyArray(const boost::array<T,N>& v)
{
    if( v.size() == 0 ) {
        return toPyArrayN((T*)NULL,0);
    }
    return toPyArrayN(&v[0],v.size());
}

#endif // OPENRAVE_BININGS_PYARRAY

} // openravepy

#endif // OPENRAVE_BOOST_PYTHON_BINDINGS
