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
#ifndef OPENRAVEPY_INTERNAL_CAMERAINTRINSICS_H
#define OPENRAVEPY_INTERNAL_CAMERAINTRINSICS_H

#define NO_IMPORT_ARRAY
#include "openravepy_int.h"

namespace openravepy
{

using py::object;

class PyCameraIntrinsics
{
public:
    PyCameraIntrinsics() {}

    template <typename T>
    PyCameraIntrinsics(const geometry::RaveCameraIntrinsics<T>& intrinsics = geometry::RaveCameraIntrinsics<T>())
    {
        const std::vector<T> v {intrinsics.fx,0,intrinsics.cx,0,intrinsics.fy,intrinsics.cy,0,0,1};
        py::array_t<T> arr = toPyArrayN(v.data(), 9);
        arr.resize({3, 3});
        K = arr;
        distortion_model = intrinsics.distortion_model;
        distortion_coeffs = toPyArray(intrinsics.distortion_coeffs);
        focal_length = intrinsics.focal_length;
    }

    virtual ~PyCameraIntrinsics() {
    }

    virtual SensorBase::CameraIntrinsics GetCameraIntrinsics()
    {
        SensorBase::CameraIntrinsics intrinsics;
        if( IS_PYTHONOBJECT_NONE(K) ) {
            intrinsics.fx = 0;
            intrinsics.fy = 0;
            intrinsics.cx = 0;
            intrinsics.cy = 0;
        }
        else {
            intrinsics.fx = K[0][0].cast<dReal>();
            intrinsics.fy = K[1][1].cast<dReal>();
            intrinsics.cx = K[0][2].cast<dReal>();
            intrinsics.cy = K[1][2].cast<dReal>();
        }
        intrinsics.distortion_model = distortion_model;
        intrinsics.distortion_coeffs = ExtractArray<dReal>(distortion_coeffs);
        intrinsics.focal_length = focal_length;
        return intrinsics;
    }
    object K;
    string distortion_model;
    object distortion_coeffs;
    dReal focal_length;
};

} // namespace openravepy
#endif // OPENRAVEPY_INTERNAL_CAMERAINTRINSICS_H
