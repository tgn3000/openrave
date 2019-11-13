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
#define NO_IMPORT_ARRAY
#include "openravepy_int.h"

#include <boost/python/slice.hpp> // slice objects
#include "openravepy_trajectorybase.h"

namespace openravepy {

TrajectoryBasePtr GetTrajectory(object o)
{
    extract<PyTrajectoryBasePtr> pytrajectory(o);
    if( pytrajectory.check() ) {
        return GetTrajectory((PyTrajectoryBasePtr)pytrajectory);
    }
    return TrajectoryBasePtr();
}

TrajectoryBasePtr GetTrajectory(PyTrajectoryBasePtr pytrajectory)
{
    return !pytrajectory ? TrajectoryBasePtr() : pytrajectory->GetTrajectory();
}

PyInterfaceBasePtr toPyTrajectory(TrajectoryBasePtr ptrajectory, PyEnvironmentBasePtr pyenv)
{
    return !ptrajectory ? PyInterfaceBasePtr() : PyInterfaceBasePtr(new PyTrajectoryBase(ptrajectory,pyenv));
}

object toPyTrajectory(TrajectoryBasePtr ptraj, object opyenv)
{
    PyEnvironmentBasePtr pyenv = opyenv.cast<PyEnvironmentBasePtr>();
    if( pyenv != nullptr ) {
        return py::cast(toPyTrajectory(ptraj,(PyEnvironmentBasePtr)pyenv));
    }
    return py::object();
}

PyEnvironmentBasePtr toPyEnvironment(PyTrajectoryBasePtr pytraj)
{
    return pytraj->GetEnv();
}

PyTrajectoryBasePtr RaveCreateTrajectory(PyEnvironmentBasePtr pyenv, const std::string& name)
{
    TrajectoryBasePtr p = OpenRAVE::RaveCreateTrajectory(GetEnvironment(pyenv), name);
    if( !p ) {
        return PyTrajectoryBasePtr();
    }
    return PyTrajectoryBasePtr(new PyTrajectoryBase(p,pyenv));
}

BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(serialize_overloads, serialize, 0, 1)

void init_openravepy_trajectory()
{
    void (PyTrajectoryBase::*Insert1)(size_t,object) = &PyTrajectoryBase::Insert;
    void (PyTrajectoryBase::*Insert2)(size_t,object,bool) = &PyTrajectoryBase::Insert;
    void (PyTrajectoryBase::*Insert3)(size_t,object,PyConfigurationSpecificationPtr) = &PyTrajectoryBase::Insert;
    void (PyTrajectoryBase::*Insert4)(size_t,object,PyConfigurationSpecificationPtr,bool) = &PyTrajectoryBase::Insert;
    object (PyTrajectoryBase::*Sample1)(dReal) const = &PyTrajectoryBase::Sample;
    object (PyTrajectoryBase::*Sample2)(dReal, PyConfigurationSpecificationPtr) const = &PyTrajectoryBase::Sample;
    object (PyTrajectoryBase::*SamplePoints2D1)(object) const = &PyTrajectoryBase::SamplePoints2D;
    object (PyTrajectoryBase::*SamplePoints2D2)(object, PyConfigurationSpecificationPtr) const = &PyTrajectoryBase::SamplePoints2D;
    object (PyTrajectoryBase::*GetWaypoints1)(size_t,size_t) const = &PyTrajectoryBase::GetWaypoints;
    object (PyTrajectoryBase::*GetWaypoints2)(size_t,size_t,PyConfigurationSpecificationPtr) const = &PyTrajectoryBase::GetWaypoints;
    object (PyTrajectoryBase::*GetWaypoints2D1)(size_t,size_t) const = &PyTrajectoryBase::GetWaypoints2D;
    object (PyTrajectoryBase::*GetWaypoints2D2)(size_t,size_t,PyConfigurationSpecificationPtr) const = &PyTrajectoryBase::GetWaypoints2D;
    object (PyTrajectoryBase::*GetAllWaypoints2D1)() const = &PyTrajectoryBase::GetAllWaypoints2D;
    object (PyTrajectoryBase::*GetAllWaypoints2D2)(PyConfigurationSpecificationPtr) const = &PyTrajectoryBase::GetAllWaypoints2D;
    object (PyTrajectoryBase::*GetWaypoint1)(int) const = &PyTrajectoryBase::GetWaypoint;
    object (PyTrajectoryBase::*GetWaypoint2)(int,PyConfigurationSpecificationPtr) const = &PyTrajectoryBase::GetWaypoint;
    object (PyTrajectoryBase::*__getitem__1)(int) const = &PyTrajectoryBase::__getitem__;
    object (PyTrajectoryBase::*__getitem__2)(slice) const = &PyTrajectoryBase::__getitem__;
    class_<PyTrajectoryBase, OPENRAVE_SHARED_PTR<PyTrajectoryBase>, bases<PyInterfaceBase> >("Trajectory", DOXY_CLASS(TrajectoryBase), no_init)
    .def("Init",&PyTrajectoryBase::Init,args("spec"),DOXY_FN(TrajectoryBase,Init))
    .def("Insert",Insert1,args("index","data"),DOXY_FN(TrajectoryBase,Insert "size_t; const std::vector; bool"))
    .def("Insert",Insert2,args("index","data","overwrite"),DOXY_FN(TrajectoryBase,Insert "size_t; const std::vector; bool"))
    .def("Insert",Insert3,args("index","data","spec"),DOXY_FN(TrajectoryBase,Insert "size_t; const std::vector; const ConfigurationSpecification; bool"))
    .def("Insert",Insert4,args("index","data","spec","overwrite"),DOXY_FN(TrajectoryBase,Insert "size_t; const std::vector; const ConfigurationSpecification; bool"))
    .def("Remove",&PyTrajectoryBase::Remove,args("startindex","endindex"),DOXY_FN(TrajectoryBase,Remove))
    .def("Sample",Sample1,args("time"),DOXY_FN(TrajectoryBase,Sample "std::vector; dReal"))
    .def("Sample",Sample2,args("time","spec"),DOXY_FN(TrajectoryBase,Sample "std::vector; dReal; const ConfigurationSpecification"))
    .def("SampleFromPrevious",&PyTrajectoryBase::SampleFromPrevious,args("data","time","spec"),DOXY_FN(TrajectoryBase,Sample "std::vector; dReal; const ConfigurationSpecification"))
    .def("SamplePoints2D",SamplePoints2D1,args("times"),DOXY_FN(TrajectoryBase,SamplePoints2D "std::vector; std::vector"))
    .def("SamplePoints2D",SamplePoints2D2,args("times","spec"),DOXY_FN(TrajectoryBase,SamplePoints2D "std::vector; std::vector; const ConfigurationSpecification"))
    .def("GetConfigurationSpecification",&PyTrajectoryBase::GetConfigurationSpecification,DOXY_FN(TrajectoryBase,GetConfigurationSpecification))
    .def("GetNumWaypoints",&PyTrajectoryBase::GetNumWaypoints,DOXY_FN(TrajectoryBase,GetNumWaypoints))
    .def("GetWaypoints",GetWaypoints1,args("startindex","endindex"),DOXY_FN(TrajectoryBase, GetWaypoints "size_t; size_t; std::vector"))
    .def("GetWaypoints",GetWaypoints2,args("startindex","endindex","spec"),DOXY_FN(TrajectoryBase, GetWaypoints "size_t; size_t; std::vector, const ConfigurationSpecification&"))
    .def("GetWaypoints2D",GetWaypoints2D1,args("startindex","endindex"),DOXY_FN(TrajectoryBase, GetWaypoints "size_t; size_t; std::vector"))
    .def("GetWaypoints2D",GetWaypoints2D2,args("startindex","endindex","spec"),DOXY_FN(TrajectoryBase, GetWaypoints "size_t; size_t; std::vector, const ConfigurationSpecification&"))
    .def("GetAllWaypoints2D",GetAllWaypoints2D1,DOXY_FN(TrajectoryBase, GetWaypoints "size_t; size_t; std::vector"))
    .def("GetAllWaypoints2D",GetAllWaypoints2D2,args("spec"),DOXY_FN(TrajectoryBase, GetWaypoints "size_t; size_t; std::vector, const ConfigurationSpecification&"))
    .def("GetWaypoint",GetWaypoint1,args("index"),DOXY_FN(TrajectoryBase, GetWaypoint "int; std::vector"))
    .def("GetWaypoint",GetWaypoint2,args("index","spec"),DOXY_FN(TrajectoryBase, GetWaypoint "int; std::vector; const ConfigurationSpecification"))
    .def("GetFirstWaypointIndexAfterTime",&PyTrajectoryBase::GetFirstWaypointIndexAfterTime, DOXY_FN(TrajectoryBase, GetFirstWaypointIndexAfterTime))
    .def("GetDuration",&PyTrajectoryBase::GetDuration,DOXY_FN(TrajectoryBase, GetDuration))
    .def("serialize",&PyTrajectoryBase::serialize,serialize_overloads(args("options"),DOXY_FN(TrajectoryBase,serialize)))
    .def("deserialize",&PyTrajectoryBase::deserialize,args("data"),DOXY_FN(TrajectoryBase,deserialize))
    .def("Write",&PyTrajectoryBase::Write,args("options"),DOXY_FN(TrajectoryBase,Write))
    .def("Read",&PyTrajectoryBase::Read,args("data","robot"),DOXY_FN(TrajectoryBase,Read))
    .def("__len__",&PyTrajectoryBase::GetNumWaypoints,DOXY_FN(TrajectoryBase,__len__))
    .def("__getitem__",__getitem__1,args("index"),DOXY_FN(TrajectoryBase, __getitem__ "int"))
    .def("__getitem__",__getitem__2,args("indices"),DOXY_FN(TrajectoryBase, __getitem__ "slice"))
    ;

    def("RaveCreateTrajectory",openravepy::RaveCreateTrajectory,args("env","name"),DOXY_FN1(RaveCreateTrajectory));
}

}
