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
// along wtmp.cast<T>();ith this program.  If not, see <http://www.gnu.org/licenses/>.

// shouldn't include openrave.h!
#ifndef OPENRAVE_BOOST_PYTHON_BINDINGS
#define OPENRAVE_BOOST_PYTHON_BINDINGS

#include <numpy/arrayobject.h>
#include <numpy/arrayscalars.h>
#include <Python.h>
#include <pybind11/numpy.h>
// #include <boost/python.hpp>
// #include <boost/python/numpy.hpp>
// #include <boost/array.hpp>
// #include <boost/multi_array.hpp>
#include <boost/shared_ptr.hpp>

#define OPENRAVE_UNIQUE_PTR std::unique_ptr
#define OPENRAVE_SHARED_PTR std::shared_ptr
#define OPENRAVE_WEAK_PTR std::weak_ptr
#define OPENRAVE_STATIC_POINTER_CAST std::static_pointer_cast
#define OPENRAVE_ENABLE_SHARED_FROM_THIS std::enable_shared_from_this
#define OPENRAVE_DYNAMIC_POINTER_CAST std::dynamic_pointer_cast
#define OPENRAVE_CONST_POINTER_CAST std::const_pointer_cast
#define OPENRAVE_MAKE_SHARED std::make_shared
#define OPENRAVE_FUNCTION boost::function

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

#include <iostream>
#include <iomanip>

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
#else // BOOST_VERSION >= 104300
#define IS_PYTHONOBJECT_NONE(o) (!!(o))
#endif // BOOST_VERSION >= 104300

namespace py = pybind11;

namespace openravepy {
// char* PyString_AsString(PyObject* pystring)
// {
//     char *pchar = nullptr;
//     if (PyUnicode_Check(pystring)) {
//         PyObject * temp_bytes = PyUnicode_AsEncodedString(pystring, "UTF-8", "strict"); // Owned reference
//         if (temp_bytes != nullptr) {
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

// https://stackoverflow.com/questions/35041268/how-to-convert-a-vector-to-numpy-array-with-templates-and-boost
template <typename T>
struct select_npy_type
{};

template <>
struct select_npy_type<double>
{
    const static NPY_TYPES type = NPY_DOUBLE;
};

template <>
struct select_npy_type<float>
{
    const static NPY_TYPES type = NPY_FLOAT;
};

template <>
struct select_npy_type<int>
{
    const static NPY_TYPES type = NPY_INT;
};

template <>
struct select_npy_type<uint8_t>
{
    const static NPY_TYPES type = NPY_UINT8;
};

template <>
struct select_npy_type<uint16_t>
{
    const static NPY_TYPES type = NPY_UINT16;
};


template <>
struct select_npy_type<uint32_t>
{
    const static NPY_TYPES type = NPY_UINT32;
};


// https://py3c.readthedocs.io/en/latest/reference.html
// https://docs.python.org/3/c-api/unicode.html#c.PyUnicode_AsUTF8
inline const char* PyString_AsString(PyObject* pystring) 
{
    return PyUnicode_AsUTF8(pystring);
}

inline py::object ConvertStringToUnicode(const std::string& s)
{
    // https://py3c.readthedocs.io/en/latest/reference.html
    PyObject *x = PyUnicode_Decode(s.c_str(),s.size(), "utf-8", nullptr);
    // https://github.com/pybind/pybind11/issues/1201
    py::handle h = x; // implicit conversion OK
    return h.cast<py::object>();
}

class PyVoidHandle
{
public:
    PyVoidHandle() {
    }
    PyVoidHandle(OPENRAVE_SHARED_PTR<void> handle) : _handle(handle) {
    }
    void Close() {
        _handle.reset();
    }
    OPENRAVE_SHARED_PTR<void> _handle;
};

class PyVoidHandleConst
{
public:
    PyVoidHandleConst() {
    }
    PyVoidHandleConst(OPENRAVE_SHARED_PTR<void const> handle) : _handle(handle) {
    }
    void Close() {
        _handle.reset();
    }
    OPENRAVE_SHARED_PTR<void const> _handle;
};

template <typename T>
inline std::vector<T> ExtractArray(const py::object& o)
{
    if( IS_PYTHONOBJECT_NONE(o) ) {
        return std::vector<T>();
    }
    std::vector<T> v(len(o));
    for(size_t i = 0; i < v.size(); ++i) {
        v[i] = o[i].cast<T>();
    }
    return v;
}

template <typename T>
inline std::set<T> ExtractSet(const py::object& o)
{
    std::set<T> v;
    size_t nlen = len(o);
    for(size_t i = 0; i < nlen; ++i) {
        v.insert(o[i].cast<T>());
    }
    return v;
}

// template <typename T>
// struct exception_translator
// {
//     exception_translator() {
//         py::register_exception_translator<T>(&exception_translator::translate);

//         //Register custom r-value converter
//         //There are situations, where we have to pass the exception back to
//         //C++ library. This will do the trick
//         py::converter::registry::push_back( &exception_translator::convertible, &exception_translator::construct, py::type_id<T>() );
//     }

//     static void translate( const T& err )
//     {
//         py::object pimpl_err( err );
//         py::object pyerr_class = pimpl_err.attr( "py_err_class" );
//         py::object pyerr = pyerr_class( pimpl_err );
//         PyErr_SetObject( pyerr_class.ptr(), py::incref( pyerr.ptr() ) );
//     }

//     //Sometimes, exceptions should be passed back to the library.
//     static void* convertible(PyObject* py_obj){
//         if( 1 != PyObject_IsInstance( py_obj, PyExc_Exception ) ) {
//             return 0;
//         }

//         if( !PyObject_HasAttrString( py_obj, "_pimpl" ) ) {
//             return 0;
//         }

//         py::object pyerr( py::handle<>( py::borrowed( py_obj ) ) );
//         py::object pimpl = getattr( pyerr, "_pimpl" );
//         py::object type_checker = pimpl.cast<T>();
//         if( !type_checker.check() ) {
//             return 0;
//         }
//         return py_obj;
//     }

//     static void construct( PyObject* py_obj, py::converter::rvalue_from_python_stage1_data* data)
//     {
//         typedef py::converter::rvalue_from_python_storage<T> storage_t;

//         py::object pyerr( py::handle<>( py::borrowed( py_obj ) ) );
//         py::object pimpl = getattr( pyerr, "_pimpl" );

//         storage_t* the_storage = reinterpret_cast<storage_t*>( data );
//         void* memory_chunk = the_storage->storage.bytes;
//         // new (memory_chunk) T( extract<T>(pimpl) );
//         // data->convertible = memory_chunk;
//         data->convertible = new T(pimp.cast<T>());
//     }
// };

// template<typename T>
// struct float_from_number
// {
//     float_from_number()
//     {
//         py::converter::registry::push_back(&convertible, &construct, py::type_id<T>());
//     }

//     static void* convertible( PyObject* obj)
//     {
//         return PyNumber_Check(obj) ? obj : nullptr;
//     }

//     static void construct(PyObject* _obj, py::converter::rvalue_from_python_stage1_data* data)
//     {
//         PyObject* tmp = PyNumber_Float(_obj);
//         T* storage = (T*)((py::converter::rvalue_from_python_storage<T>*)data)->storage.bytes;
//         *storage = tmp.cast<T>();
//         Py_DECREF(tmp);
//         data->convertible = storage;
//     }
// };

// template<typename T>
// struct int_from_number
// {
//     int_from_number()
//     {
//         py::converter::registry::push_back(&convertible, &construct, py::type_id<T>());
//     }

//     static void* convertible( PyObject* obj)
//     {
//         return PyNumber_Check(obj) ? obj : nullptr;
//     }

//     static void construct(PyObject* _obj, py::converter::rvalue_from_python_stage1_data* data)
//     {
//         PyObject* tmp = PyNumber_Long(_obj);
//         T* storage = (T*)((py::converter::rvalue_from_python_storage<T>*)data)->storage.bytes;
//         *storage = tmp.cast<T>();
//         Py_DECREF(tmp);
//         data->convertible = storage;
//     }
// };

inline std::string GetPyErrorString()
{
    PyObject *error, *value, *traceback;
    PyErr_Fetch(&error, &value, &traceback);
    PyErr_NormalizeException(&error, &value, &traceback);
    std::string s;
    if(error != nullptr) {
        PyObject* pystring = PyObject_Str(value);
        if(pystring != nullptr) {
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
inline py::array_t<T> toPyArrayN(const T* pvalues, const size_t N)
{
    std::vector<npy_intp> dims {(long int)1, (long int)N};
    return py::array_t<T>(dims, pvalues); 
}

template <typename T>
inline py::array_t<T> toPyArrayN(const T* pvalues, std::vector<npy_intp>& dims)
{
    return py::array_t<T>(dims, pvalues); 
}

template <typename T>
inline py::object toPyList(const std::vector<T>& v)
{
    py::list lvalues;
    FOREACHC(it,v) {
        lvalues.append(py::object(*it));
    }
    return std::move(lvalues);
}

template <typename T>
inline py::array_t<T> toPyArray(const std::vector<T>& v)
{
    if( v.empty() ) {
        return toPyArrayN((T*)nullptr, 0);
    }
    return toPyArrayN(v.data(), v.size());
}

template <typename T>
inline py::array_t<T> toPyArray(const std::vector<T>& v, std::vector<npy_intp>& dims)
{
    if( v.empty() ) {
        return toPyArrayN((T*)nullptr, dims);
    }
    size_t totalsize = 1;
    FOREACH(it, dims) {
        totalsize *= *it;
    }
    BOOST_ASSERT(totalsize == v.size());
    return toPyArrayN(v.data(), dims);
}

template <typename T, int N>
inline py::array_t<T> toPyArray(const boost::array<T,N>& v)
{
    if( v.empty() ) {
        return toPyArrayN((T*)nullptr, 0);
    }
    return toPyArrayN(v.data(),v.size());
}

#endif // OPENRAVE_BININGS_PYARRAY

} // openravepy

#endif // OPENRAVE_BOOST_PYTHON_BINDINGS
