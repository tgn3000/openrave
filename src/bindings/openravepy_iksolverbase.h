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
#ifndef OPENRAVEPY_INTERNAL_IKSOLVERBASE_H
#define OPENRAVEPY_INTERNAL_IKSOLVERBASE_H

#define NO_IMPORT_ARRAY
#include "openravepy_int.h"

namespace openravepy
{

using py::object;

class PyIkReturn
{
public:
    PyIkReturn(const IkReturn& ret) : _ret(ret) {
    }
    PyIkReturn(IkReturnPtr pret) : _ret(*pret) {
    }
    PyIkReturn(IkReturnAction action) : _ret(action) {
    }
    IkReturnAction GetAction() {
        return _ret._action;
    }
    object GetSolution() {
        return toPyArray(_ret._vsolution);
    }
    object GetUserData() {
        return openravepy::GetUserData(_ret._userdata);
    }
    object GetMapData(const std::string& key) {
        IkReturn::CustomData::const_iterator it = _ret._mapdata.find(key);
        if( it == _ret._mapdata.end() ) {
            return object();
        }
        return toPyArray(it->second);
    }
    object GetMapDataDict() {
        py::dict odata;
        FOREACHC(it,_ret._mapdata) {
            odata[it->first] = toPyArray(it->second);
        }
        return std::move(odata);
    }

    void SetUserData(PyUserData pdata) {
        _ret._userdata = pdata._handle;
    }
    void SetSolution(object osolution) {
        _ret._vsolution = ExtractArray<dReal>(osolution);
    }
    void SetMapKeyValue(const std::string& key, object ovalues) {
        _ret._mapdata[key] = ExtractArray<dReal>(ovalues);
    }

    IkReturn _ret;
};

typedef OPENRAVE_SHARED_PTR<PyIkReturn> PyIkReturnPtr;

class PyIkSolverBase : public PyInterfaceBase
{
protected:
    IkSolverBasePtr _pIkSolver;

    static IkReturn _CallCustomFilter(object fncallback, PyEnvironmentBasePtr pyenv, IkSolverBasePtr pIkSolver, std::vector<dReal>& values, RobotBase::ManipulatorConstPtr pmanip, const IkParameterization& ikparam)
    {
        object res;
        PyGILState_STATE gstate = PyGILState_Ensure();
        std::string errmsg;
        try {
            RobotBase::ManipulatorPtr pmanip2 = OPENRAVE_CONST_POINTER_CAST<RobotBase::Manipulator>(pmanip);
            res = fncallback(toPyArray(values), openravepy::toPyRobotManipulator(pmanip2,pyenv),toPyIkParameterization(ikparam));
        }
        catch(...) {
            errmsg = boost::str(boost::format("exception occured in python custom filter callback of iksolver %s: %s")%pIkSolver->GetXMLId()%GetPyErrorString());
        }
        IkReturn ikfr(IKRA_Success);
        if( IS_PYTHONOBJECT_NONE(res) ) {
            ikfr._action = IKRA_Reject;
        }
        else {
            if( !openravepy::ExtractIkReturn(res,ikfr) ) {
                try {
                    IkReturnAction ikfra = res.cast<IkReturnAction>();
                    ikfr._action = (IkReturnAction) ikfra;
                }
                catch(...) {
                    errmsg = "failed to convert return type of filter to IkReturn";
                }
            }
        }

        PyGILState_Release(gstate);
        if( errmsg.size() > 0 ) {
            throw openrave_exception(errmsg,ORE_Assert);
        }
        return ikfr;
    }

public:
    PyIkSolverBase(IkSolverBasePtr pIkSolver, PyEnvironmentBasePtr pyenv) : PyInterfaceBase(pIkSolver, pyenv), _pIkSolver(pIkSolver) {
    }
    virtual ~PyIkSolverBase() {
    }

    IkSolverBasePtr GetIkSolver() {
        return _pIkSolver;
    }

    int GetNumFreeParameters() const {
        return _pIkSolver->GetNumFreeParameters();
    }
    object GetFreeParameters() const {
        if( _pIkSolver->GetNumFreeParameters() == 0 ) {
            return py::array_t<dReal>({1, 0}, nullptr);
        }
        vector<dReal> values;
        _pIkSolver->GetFreeParameters(values);
        return toPyArray(values);
    }

    PyIkReturnPtr Solve(object oparam, object oq0, int filteroptions)
    {
        PyIkReturnPtr pyreturn(new PyIkReturn(IKRA_Reject));
        IkReturnPtr preturn(&pyreturn->_ret, utils::null_deleter());
        vector<dReal> q0;
        if( !IS_PYTHONOBJECT_NONE(oq0) ) {
            q0 = ExtractArray<dReal>(oq0);
        }
        IkParameterization ikparam;
        if( !ExtractIkParameterization(oparam,ikparam) ) {
            throw openrave_exception(_("first argument to IkSolver.Solve needs to be IkParameterization"),ORE_InvalidArguments);
        }
        _pIkSolver->Solve(ikparam, q0, filteroptions, preturn);
        return pyreturn;
    }

    object SolveAll(object oparam, int filteroptions)
    {
        py::list pyreturns;
        std::vector<IkReturnPtr> vikreturns;
        IkParameterization ikparam;
        if( !ExtractIkParameterization(oparam,ikparam) ) {
            throw openrave_exception(_("first argument to IkSolver.Solve needs to be IkParameterization"),ORE_InvalidArguments);
        }
        _pIkSolver->SolveAll(ikparam, filteroptions, vikreturns);
        FOREACH(itikreturn,vikreturns) {
            pyreturns.append(py::cast(PyIkReturnPtr(new PyIkReturn(*itikreturn))));
        }
        return std::move(pyreturns);
    }

    PyIkReturnPtr Solve(object oparam, object oq0, object oFreeParameters, int filteroptions)
    {
        PyIkReturnPtr pyreturn(new PyIkReturn(IKRA_Reject));
        IkReturnPtr preturn(&pyreturn->_ret, utils::null_deleter());
        vector<dReal> q0, vFreeParameters;
        if( !IS_PYTHONOBJECT_NONE(oq0) ) {
            q0 = ExtractArray<dReal>(oq0);
        }
        if( !IS_PYTHONOBJECT_NONE(oFreeParameters) ) {
            vFreeParameters = ExtractArray<dReal>(oFreeParameters);
        }
        IkParameterization ikparam;
        if( !ExtractIkParameterization(oparam,ikparam) ) {
            throw openrave_exception(_("first argument to IkSolver.Solve needs to be IkParameterization"),ORE_InvalidArguments);
        }
        _pIkSolver->Solve(ikparam, q0, vFreeParameters,filteroptions, preturn);
        return pyreturn;
    }

    object SolveAll(object oparam, object oFreeParameters, int filteroptions)
    {
        py::list pyreturns;
        std::vector<IkReturnPtr> vikreturns;
        IkParameterization ikparam;
        if( !ExtractIkParameterization(oparam,ikparam) ) {
            throw openrave_exception(_("first argument to IkSolver.Solve needs to be IkParameterization"),ORE_InvalidArguments);
        }
        vector<dReal> vFreeParameters;
        if( !IS_PYTHONOBJECT_NONE(oFreeParameters) ) {
            vFreeParameters = ExtractArray<dReal>(oFreeParameters);
        }
        _pIkSolver->SolveAll(ikparam, vFreeParameters, filteroptions, vikreturns);
        FOREACH(itikreturn,vikreturns) {
            pyreturns.append(py::cast(PyIkReturnPtr(new PyIkReturn(*itikreturn))));
        }
        return std::move(pyreturns);
    }

    PyIkReturnPtr CallFilters(object oparam)
    {
        PyIkReturnPtr pyreturn(new PyIkReturn(IKRA_Reject));
        IkReturnPtr preturn(&pyreturn->_ret, utils::null_deleter());
        IkParameterization ikparam;
        if( !ExtractIkParameterization(oparam,ikparam) ) {
            throw openrave_exception(_("first argument to IkSolver.Solve needs to be IkParameterization"),ORE_InvalidArguments);
        }
        _pIkSolver->CallFilters(ikparam, preturn);
        return pyreturn;
    }

    bool Supports(IkParameterizationType type) {
        return _pIkSolver->Supports(type);
    }

    object RegisterCustomFilter(int priority, object fncallback)
    {
        if( !fncallback ) {
            throw OPENRAVE_EXCEPTION_FORMAT0(_("callback not specified"),ORE_InvalidArguments);
        }
        return toPyUserData(_pIkSolver->RegisterCustomFilter(priority,boost::bind(&PyIkSolverBase::_CallCustomFilter,fncallback,_pyenv,_pIkSolver,_1,_2,_3)));
    }
};

} // namespace openravepy
#endif // OPENRAVEPY_INTERNAL_IKSOLVERBASE_H
