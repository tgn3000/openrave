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
#ifndef OPENRAVEPY_INTERNAL_CONNECTEDBODYINFO_H
#define OPENRAVEPY_INTERNAL_CONNECTEDBODYINFO_H

#define NO_IMPORT_ARRAY
#include "openravepy_int.h"

namespace openravepy {

class PyConnectedBodyInfo
{
public:
    PyConnectedBodyInfo() {
    }
    PyConnectedBodyInfo(const RobotBase::ConnectedBodyInfo& info, PyEnvironmentBasePtr pyenv)
    {
        _name = ConvertStringToUnicode(info._name);
        _linkname = ConvertStringToUnicode(info._linkname);
        _trelative = ReturnTransform(info._trelative);
        _url = ConvertStringToUnicode(info._url);

        py::list linkInfos;
        FOREACH(itlinkinfo, info._vLinkInfos) {
            linkInfos.append(toPyLinkInfo(**itlinkinfo));
        }
        _linkInfos = linkInfos;

        py::list jointInfos;
        FOREACH(itjointinfo, info._vJointInfos) {
            jointInfos.append(toPyJointInfo(**itjointinfo, pyenv));
        }
        _jointInfos = jointInfos;

        py::list manipulatorInfos;
        FOREACH(itmanipulatorinfo, info._vManipulatorInfos) {
            manipulatorInfos.append(toPyManipulatorInfo(**itmanipulatorinfo));
        }
        _manipulatorInfos = manipulatorInfos;

        py::list attachedSensorInfos;
        FOREACH(itattachedSensorinfo, info._vAttachedSensorInfos) {
            attachedSensorInfos.append(toPyAttachedSensorInfo(**itattachedSensorinfo));
        }
        _attachedSensorInfos = attachedSensorInfos;
    }

    RobotBase::ConnectedBodyInfoPtr GetConnectedBodyInfo() const
    {
        RobotBase::ConnectedBodyInfoPtr pinfo(new RobotBase::ConnectedBodyInfo());
        pinfo->_name = _name.cast<std::string>();
        pinfo->_linkname = _linkname.cast<std::string>();
        pinfo->_trelative = ExtractTransform(_trelative);
        pinfo->_url = _url.cast<std::string>();
        // extract all the infos
        return pinfo;
    }

    object _name;
    object _linkname;
    object _trelative;
    object _url;
    object _linkInfos;
    object _jointInfos;
    object _manipulatorInfos;
    object _attachedSensorInfos;

};

} // namespace openravepy
#endif // OPENRAVEPY_INTERNAL_CONNECTEDBODYINFO_H
