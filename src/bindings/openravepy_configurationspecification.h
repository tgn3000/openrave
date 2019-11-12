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
#ifndef OPENRAVEPY_INTERNAL_CONFIGURATIONSPECIFICATION_H
#define OPENRAVEPY_INTERNAL_CONFIGURATIONSPECIFICATION_H
#define NO_IMPORT_ARRAY
#include "openravepy_int.h"

#include <openrave/xmlreaders.h>
#include <openrave/utils.h>

namespace openravepy {

class PyConfigurationSpecification : public OPENRAVE_ENABLE_SHARED_FROM_THIS<PyConfigurationSpecification>
{
public:
    PyConfigurationSpecification() {
    }
    PyConfigurationSpecification(const std::string &s) {
        std::stringstream ss(s);
        ss >> _spec;
    }
    PyConfigurationSpecification(const ConfigurationSpecification& spec) {
        _spec = spec;
    }
    PyConfigurationSpecification(const ConfigurationSpecification::Group& g) {
        _spec = ConfigurationSpecification(g);
    }
    PyConfigurationSpecification(PyConfigurationSpecificationPtr pyspec) {
        _spec = pyspec->_spec;
    }
    virtual ~PyConfigurationSpecification() {
    }

    int GetDOF() const {
        return _spec.GetDOF();
    }

    bool IsValid() const {
        return _spec.IsValid();
    }

    const ConfigurationSpecification::Group& GetGroupFromName(const std::string& name) {
        return _spec.GetGroupFromName(name);
    }

    object FindCompatibleGroup(const std::string& name, bool exactmatch) const
    {
        std::vector<ConfigurationSpecification::Group>::const_iterator it  = _spec.FindCompatibleGroup(name,exactmatch);
        if( it == _spec._vgroups.end() ) {
            return object();
        }
        return py::cast(OPENRAVE_SHARED_PTR<ConfigurationSpecification::Group>(new ConfigurationSpecification::Group(*it)));
    }

    object FindTimeDerivativeGroup(const std::string& name, bool exactmatch) const
    {
        std::vector<ConfigurationSpecification::Group>::const_iterator it  = _spec.FindTimeDerivativeGroup(name,exactmatch);
        if( it == _spec._vgroups.end() ) {
            return object();
        }
        return py::cast(OPENRAVE_SHARED_PTR<ConfigurationSpecification::Group>(new ConfigurationSpecification::Group(*it)));
    }

//    ConfigurationSpecification GetTimeDerivativeSpecification(int timederivative) const;

    void ResetGroupOffsets()
    {
        _spec.ResetGroupOffsets();
    }

    void AddVelocityGroups(bool adddeltatime)
    {
        RAVELOG_WARN("openravepy AddVelocityGroups is deprecated, use AddDerivativeGroups\n");
        _spec.AddDerivativeGroups(1,adddeltatime);
    }

    void AddDerivativeGroups(int deriv, bool adddeltatime)
    {
        _spec.AddDerivativeGroups(deriv,adddeltatime);
    }

    int AddDeltaTimeGroup() {
        return _spec.AddDeltaTimeGroup();
    }

    int AddGroup(const std::string& name, int dof, const std::string& interpolation)
    {
        return _spec.AddGroup(name,dof,interpolation);
    }

    int AddGroup(const ConfigurationSpecification::Group& g)
    {
        return _spec.AddGroup(g);
    }

    int RemoveGroups(const std::string& groupname, bool exactmatch=true)
    {
        return _spec.RemoveGroups(groupname, exactmatch);
    }

    PyConfigurationSpecificationPtr ConvertToVelocitySpecification() const
    {
        return toPyConfigurationSpecification(_spec.ConvertToVelocitySpecification());
    }

    PyConfigurationSpecificationPtr ConvertToDerivativeSpecification(uint32_t timederivative) const
    {
        return toPyConfigurationSpecification(_spec.ConvertToDerivativeSpecification(timederivative));
    }

    PyConfigurationSpecificationPtr GetTimeDerivativeSpecification(int timederivative) const
    {
        return toPyConfigurationSpecification(_spec.GetTimeDerivativeSpecification(timederivative));
    }

    object ExtractTransform(object otransform, object odata, PyKinBodyPtr pybody, int timederivative=0) const
    {
        std::vector<dReal> vdata = ExtractArray<dReal>(odata);
        Transform t;
        if( !IS_PYTHONOBJECT_NONE(otransform) ) {
            t = openravepy::ExtractTransform(otransform);
        }
        if( _spec.ExtractTransform(t,vdata.begin(), GetKinBody(pybody)) ) {
            return ReturnTransform(t);
        }
        return object();
    }

    object ExtractIkParameterization(object odata, int timederivative=0, const std::string& robotname="", const std::string& manipulatorname="") const
    {
        IkParameterization ikparam;
        std::vector<dReal> vdata = ExtractArray<dReal>(odata);
        bool bfound = _spec.ExtractIkParameterization(ikparam, vdata.begin(), timederivative, robotname, manipulatorname);
        if( bfound ) {
            return toPyIkParameterization(ikparam);
        }
        else {
            return object();
        }
    }

    object ExtractAffineValues(object odata, PyKinBodyPtr pybody, int affinedofs, int timederivative=0) const
    {
        std::vector<dReal> vdata = ExtractArray<dReal>(odata);
        std::vector<dReal> values(RaveGetAffineDOF(affinedofs),0);
        bool bfound = _spec.ExtractAffineValues(values.begin(),vdata.begin(), GetKinBody(pybody),affinedofs, timederivative);
        if( bfound ) {
            return toPyArray(values);
        }
        else {
            return object();
        }
    }

    object ExtractJointValues(object odata, PyKinBodyPtr pybody, object oindices, int timederivative=0) const
    {
        std::vector<int> vindices = ExtractArray<int>(oindices);
        std::vector<dReal> vdata = ExtractArray<dReal>(odata);
        std::vector<dReal> values(vindices.size(),0);
        bool bfound = _spec.ExtractJointValues(values.begin(),vdata.begin(),openravepy::GetKinBody(pybody),vindices,timederivative);
        if( bfound ) {
            return toPyArray(values);
        }
        else {
            return object();
        }
    }

    object ExtractDeltaTime(object odata) const
    {
        std::vector<dReal> vdata = ExtractArray<dReal>(odata);
        dReal deltatime=0;
        bool bfound = _spec.ExtractDeltaTime(deltatime,vdata.begin());
        if( bfound ) {
            return py::cast(deltatime);
        }
        else {
            return object();
        }
    }

    bool InsertJointValues(object odata, object ovalues, PyKinBodyPtr pybody, object oindices, int timederivative=0) const
    {
        vector<int> vindices = ExtractArray<int>(oindices);
        std::vector<dReal> vdata = ExtractArray<dReal>(odata);
        std::vector<dReal> vvalues = ExtractArray<dReal>(ovalues);
        OPENRAVE_ASSERT_OP(vvalues.size(),==,vindices.size());
        OPENRAVE_ASSERT_OP(vdata.size(),>=,vvalues.size());
        OPENRAVE_ASSERT_OP((int)vdata.size(),==,_spec.GetDOF());
        if( !_spec.InsertJointValues(vdata.begin(), vvalues.begin(), openravepy::GetKinBody(pybody), vindices, timederivative) ) {
            return false;
        }
        // copy the value back, this is wasteful, but no other way to do it unless vdata pointer pointed directly to odata
        for(size_t i = 0; i < vdata.size(); ++i) {
            odata[i] = vdata[i];
        }
        return true;
    }

    bool InsertDeltaTime(object odata, dReal deltatime)
    {
        // it is easier to get the time index
        FOREACHC(itgroup,_spec._vgroups) {
            if( itgroup->name == "deltatime" ) {
                odata[itgroup->offset] = py::cast(deltatime);
                return true;
            }
        }
        return false;
    }

    py::list ExtractUsedBodies(PyEnvironmentBasePtr pyenv)
    {
        std::vector<KinBodyPtr> vusedbodies;
        _spec.ExtractUsedBodies(openravepy::GetEnvironment(pyenv), vusedbodies);
        py::list obodies;
        FOREACHC(itbody, vusedbodies) {
            if( (*itbody)->IsRobot() ) {
                obodies.append(openravepy::toPyRobot(RaveInterfaceCast<RobotBase>(*itbody), pyenv));
            }
            else {
                obodies.append(openravepy::toPyKinBody(*itbody, pyenv));
            }
        }
        return obodies;
    }

    object ExtractUsedIndices(PyKinBodyPtr pybody)
    {
        std::vector<int> useddofindices, usedconfigindices;
        _spec.ExtractUsedIndices(openravepy::GetKinBody(pybody), useddofindices, usedconfigindices);
        return py::make_tuple(toPyArray(useddofindices), toPyArray(usedconfigindices));
    }

//
//    static void ConvertGroupData(std::vector<dReal>::iterator ittargetdata, size_t targetstride, const Group& gtarget, std::vector<dReal>::const_iterator itsourcedata, size_t sourcestride, const Group& gsource, size_t numpoints, EnvironmentBaseConstPtr penv);
//
    // source spec is the current configurationspecification spec
    object ConvertData(PyConfigurationSpecificationPtr pytargetspec, object osourcedata, size_t numpoints, PyEnvironmentBasePtr pyenv, bool filluninitialized = true)
    {
        std::vector<dReal> vtargetdata(pytargetspec->_spec.GetDOF()*numpoints,0);
        std::vector<dReal> vsourcedata = ExtractArray<dReal>(osourcedata);
        ConfigurationSpecification::ConvertData(vtargetdata.begin(), pytargetspec->_spec, vsourcedata.begin(), _spec, numpoints, openravepy::GetEnvironment(pyenv), filluninitialized);
        return toPyArray(vtargetdata);
    }

    object ConvertDataFromPrevious(object otargetdata, PyConfigurationSpecificationPtr pytargetspec, object osourcedata, size_t numpoints, PyEnvironmentBasePtr pyenv)
    {
        std::vector<dReal> vtargetdata = ExtractArray<dReal>(otargetdata);
        std::vector<dReal> vsourcedata = ExtractArray<dReal>(osourcedata);
        ConfigurationSpecification::ConvertData(vtargetdata.begin(), pytargetspec->_spec, vsourcedata.begin(), _spec, numpoints, openravepy::GetEnvironment(pyenv), false);
        return toPyArray(vtargetdata);
    }

    py::list GetGroups()
    {
        py::list ogroups;
        FOREACHC(itgroup, _spec._vgroups) {
            ogroups.append(*itgroup);
        }
        return ogroups;
    }

    bool __eq__(PyConfigurationSpecificationPtr p) {
        return !!p && _spec==p->_spec;
    }
    bool __ne__(PyConfigurationSpecificationPtr p) {
        return !p || _spec!=p->_spec;
    }

    PyConfigurationSpecificationPtr __add__(PyConfigurationSpecificationPtr r)
    {
        return PyConfigurationSpecificationPtr(new PyConfigurationSpecification(_spec + r->_spec));
    }

    PyConfigurationSpecificationPtr __iadd__(PyConfigurationSpecificationPtr r)
    {
        _spec += r->_spec;
        return shared_from_this();
    }

    string __repr__() {
        std::stringstream ss;
        ss << "ConfigurationSpecification(\"\"\"" << _spec << "\"\"\")";
        return ss.str();
    }
    string __str__() {
        std::stringstream ss;
        ss << "<configuration dof=\"" << _spec.GetDOF() << "\">";
        return ss.str();
    }
    object __unicode__() {
        return ConvertStringToUnicode(__str__());
    }

    ConfigurationSpecification _spec;
};

} // namespace openravepy
#endif // OPENRAVEPY_INTERNAL_CONFIGURATIONSPECIFICATION_H
