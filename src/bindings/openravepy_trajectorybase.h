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
#ifndef OPENRAVEPY_INTERNAL_TRAJECTORYBASE_H
#define OPENRAVEPY_INTERNAL_TRAJECTORYBASE_H

#define NO_IMPORT_ARRAY
#include "openravepy_int.h"

namespace openravepy {

using py::object;

class PyTrajectoryBase : public PyInterfaceBase
{
protected:
    TrajectoryBasePtr _ptrajectory;
public:
    PyTrajectoryBase(TrajectoryBasePtr pTrajectory, PyEnvironmentBasePtr pyenv) : PyInterfaceBase(pTrajectory, pyenv),_ptrajectory(pTrajectory) {
    }
    virtual ~PyTrajectoryBase() {
    }

    void Init(PyConfigurationSpecificationPtr pyspec) {
        _ptrajectory->Init(openravepy::GetConfigurationSpecification(pyspec));
    }

    void Insert(size_t index, object odata)
    {
        std::vector<dReal> vdata = ExtractArray<dReal>(odata);
        _ptrajectory->Insert(index,vdata);
    }

    void Insert(size_t index, object odata, bool bOverwrite)
    {
        std::vector<dReal> vdata = ExtractArray<dReal>(odata);
        _ptrajectory->Insert(index,vdata,bOverwrite);
    }

    void Insert(size_t index, object odata, PyConfigurationSpecificationPtr pyspec)
    {
        std::vector<dReal> vdata = ExtractArray<dReal>(odata);
        _ptrajectory->Insert(index,vdata,openravepy::GetConfigurationSpecification(pyspec));
    }

    void Insert(size_t index, object odata, PyConfigurationSpecificationPtr pyspec, bool bOverwrite)
    {
        std::vector<dReal> vdata = ExtractArray<dReal>(odata);
        _ptrajectory->Insert(index,vdata,openravepy::GetConfigurationSpecification(pyspec),bOverwrite);
    }

    void Remove(size_t startindex, size_t endindex)
    {
        _ptrajectory->Remove(startindex,endindex);
    }

    object Sample(dReal time) const
    {
        vector<dReal> values;
        _ptrajectory->Sample(values,time);
        return toPyArray(values);
    }

    object Sample(dReal time, PyConfigurationSpecificationPtr pyspec) const
    {
        vector<dReal> values;
        _ptrajectory->Sample(values,time,openravepy::GetConfigurationSpecification(pyspec), true);
        return toPyArray(values);
    }

    object SampleFromPrevious(object odata, dReal time, PyConfigurationSpecificationPtr pyspec) const
    {
        vector<dReal> vdata = ExtractArray<dReal>(odata);
        _ptrajectory->Sample(vdata,time,openravepy::GetConfigurationSpecification(pyspec), false);
        return toPyArray(vdata);
    }

    object SamplePoints2D(object otimes) const
    {
        vector<dReal> values;
        std::vector<dReal> vtimes = ExtractArray<dReal>(otimes);
        _ptrajectory->SamplePoints(values,vtimes);

        int numdof = _ptrajectory->GetConfigurationSpecification().GetDOF();
        npy_intp dims[] = { npy_intp(values.size()/numdof), npy_intp(numdof) };
        PyObject *pypos = PyArray_SimpleNew(2,dims, sizeof(dReal)==8 ? PyArray_DOUBLE : PyArray_FLOAT);
        if( values.size() > 0 ) {
            memcpy(PyArray_DATA(pypos), &values[0], values.size()*sizeof(values[0]));
        }
        return py::cast(pypos);
        // np::dtype dt = sizeof(dReal)==8 ? np::dtype::get_builtin<double>() : np::dtype::get_builtin<float>();
        // boost::python::object o((boost::python::handle<>(pypos)));
        // return np::array(o, dt);
        // return static_cast<numeric::array>(handle<>(pypos));
    }

    object SamplePoints2D(object otimes, PyConfigurationSpecificationPtr pyspec) const
    {
        vector<dReal> values;
        ConfigurationSpecification spec = openravepy::GetConfigurationSpecification(pyspec);
        std::vector<dReal> vtimes = ExtractArray<dReal>(otimes);
        _ptrajectory->SamplePoints(values, vtimes, spec);

        npy_intp dims[] = { npy_intp(values.size()/spec.GetDOF()), npy_intp(spec.GetDOF()) };
        PyObject *pypos = PyArray_SimpleNew(2,dims, sizeof(dReal)==8 ? PyArray_DOUBLE : PyArray_FLOAT);
        if( values.size() > 0 ) {
            memcpy(PyArray_DATA(pypos), &values[0], values.size()*sizeof(values[0]));
        }
        return py::cast(pypos);
        // np::dtype dt = sizeof(dReal)==8 ? np::dtype::get_builtin<double>() : np::dtype::get_builtin<float>();
        // boost::python::object o((boost::python::handle<>(pypos)));
        // return np::array(o, dt);
        // return static_cast<numeric::array>(handle<>(pypos));
    }

    object GetConfigurationSpecification() const {
        return py::cast(openravepy::toPyConfigurationSpecification(_ptrajectory->GetConfigurationSpecification()));
    }

    size_t GetNumWaypoints() const {
        return _ptrajectory->GetNumWaypoints();
    }

    object GetWaypoints(size_t startindex, size_t endindex) const
    {
        std::vector<dReal> values;
        _ptrajectory->GetWaypoints(startindex,endindex,values);
        return toPyArray(values);
    }

    object GetWaypoints(size_t startindex, size_t endindex, PyConfigurationSpecificationPtr pyspec) const
    {
        std::vector<dReal> values;
        _ptrajectory->GetWaypoints(startindex,endindex,values,openravepy::GetConfigurationSpecification(pyspec));
        return toPyArray(values);
    }

    // similar to GetWaypoints except returns a 2D array, one row for every waypoint
    object GetWaypoints2D(size_t startindex, size_t endindex) const
    {
        std::vector<dReal> values;
        _ptrajectory->GetWaypoints(startindex,endindex,values);
        int numdof = _ptrajectory->GetConfigurationSpecification().GetDOF();
        npy_intp dims[] = { npy_intp(values.size()/numdof), npy_intp(numdof) };
        PyObject *pypos = PyArray_SimpleNew(2,dims, sizeof(dReal)==8 ? PyArray_DOUBLE : PyArray_FLOAT);
        if( values.size() > 0 ) {
            memcpy(PyArray_DATA(pypos), &values[0], values.size()*sizeof(values[0]));
        }
        return py::cast(pypos);

        // np::dtype dt = sizeof(dReal)==8 ? np::dtype::get_builtin<double>() : np::dtype::get_builtin<float>();
        // boost::python::object o((boost::python::handle<>(pypos)));
        // return np::array(o, dt);
    }

    object __getitem__(int index) const
    {
        return GetWaypoint(index);
    }

    object __getitem__(py::slice indices) const
    {
        std::vector<int> vindices;
        int len = _ptrajectory->GetNumWaypoints();
        // https://github.com/pybind/pybind11/issues/1095
        size_t len_ = len, start_, stop_, step_, slicelength_;
        int start, stop, step, slicelength;
        if (indices.compute(len_, &start_, &stop_, &step_, &slicelength_)) {
            step = step_;
            start = start_;
            stop = stop_;
        }
        else {
            step = 1;
            start = step > 0 ? 0 : len-1;
            stop = step > 0 ? len : -1;
        }
        if(step == 0) {
            throw OPENRAVE_EXCEPTION_FORMAT0(_("step cannot be 0"),ORE_InvalidArguments);
        }
        for(int i = start; step > 0 ? i < stop : i > stop; i += step) {
            vindices.push_back(i);
        }

        std::vector<dReal> values;
        _ptrajectory->GetWaypoint(0,values);
        int numdof = _ptrajectory->GetConfigurationSpecification().GetDOF();
        npy_intp dims[] = { npy_intp(vindices.size()), npy_intp(numdof) };
        PyObject *pypos = PyArray_SimpleNew(2,dims, sizeof(dReal)==8 ? PyArray_DOUBLE : PyArray_FLOAT);
        int waypointSize = values.size()*sizeof(values[0]);
        for(int i=0;i<vindices.size();i++) {
            _ptrajectory->GetWaypoint(vindices[i],values);
            memcpy(PyArray_BYTES(pypos)+(i*waypointSize), &values[0], waypointSize);
        }
        return py::cast(pypos);
        // np::dtype dt = sizeof(dReal)==8 ? np::dtype::get_builtin<double>() : np::dtype::get_builtin<float>();
        // boost::python::object o((boost::python::handle<>(pypos)));
        // return np::array(o, dt);
    }

    object GetAllWaypoints2D() const
    {
        return GetWaypoints2D(0, _ptrajectory->GetNumWaypoints());
    }

    object GetWaypoints2D(size_t startindex, size_t endindex, PyConfigurationSpecificationPtr pyspec) const
    {
        vector<dReal> values;
        ConfigurationSpecification spec = openravepy::GetConfigurationSpecification(pyspec);
        _ptrajectory->GetWaypoints(startindex,endindex,values,spec);
        npy_intp dims[] = { npy_intp(values.size()/spec.GetDOF()), npy_intp(spec.GetDOF()) };
        PyObject *pypos = PyArray_SimpleNew(2,dims, sizeof(dReal)==8 ? PyArray_DOUBLE : PyArray_FLOAT);
        if( values.size() > 0 ) {
            memcpy(PyArray_DATA(pypos), &values[0], values.size()*sizeof(values[0]));
        }
        return py::cast(pypos);
        // np::dtype dt = sizeof(dReal)==8 ? np::dtype::get_builtin<double>() : np::dtype::get_builtin<float>();
        // boost::python::object o((boost::python::handle<>(pypos)));
        // return np::array(o, dt);
    }

    object GetAllWaypoints2D(PyConfigurationSpecificationPtr pyspec) const
    {
        return GetWaypoints2D(0, _ptrajectory->GetNumWaypoints(), pyspec);
    }

    object GetWaypoint(int index) const
    {
        std::vector<dReal> values;
        _ptrajectory->GetWaypoint(index,values);
        return toPyArray(values);
    }

    object GetWaypoint(int index, PyConfigurationSpecificationPtr pyspec) const
    {
        std::vector<dReal> values;
        _ptrajectory->GetWaypoint(index,values,openravepy::GetConfigurationSpecification(pyspec));
        return toPyArray(values);
    }

    size_t GetFirstWaypointIndexAfterTime(dReal time) const
    {
        return _ptrajectory->GetFirstWaypointIndexAfterTime(time);
    }

    dReal GetDuration() const {
        return _ptrajectory->GetDuration();
    }

    PyTrajectoryBasePtr deserialize(const string& s)
    {
        std::stringstream ss(s);
        InterfaceBasePtr p = _ptrajectory->deserialize(ss);
        return PyTrajectoryBasePtr(new PyTrajectoryBase(RaveInterfaceCast<TrajectoryBase>(p),_pyenv));
    }

    object serialize(object ooptions=object())
    {
        std::stringstream ss;
        ss << std::setprecision(std::numeric_limits<dReal>::digits10+1);
        _ptrajectory->serialize(ss,pyGetIntFromPy(ooptions,0));
        return py::cast(ss.str());
    }

    bool Read(const string& s, object probot) {
        RAVELOG_WARN("Trajectory.Read deprecated please use Trajerctory.deserialize\n");
        deserialize(s);
        return true;
    }

    object Write(object options) {
        RAVELOG_WARN("Trajectory.Write deprecated please use Trajerctory.serialize\n");
        return serialize(options);
    }

    TrajectoryBasePtr GetTrajectory() {
        return _ptrajectory;
    }
};

} // namespace openravepy
#endif // OPENRAVEPY_INTERNAL_TRAJECTORYBASE_H
