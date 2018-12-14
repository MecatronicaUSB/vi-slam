/**
* This file is part of UW-SLAM.
* 
* Copyright 2018.
* Developed by Fabio Morales,
* Email: fabmoraleshidalgo@gmail.com; GitHub: @fmoralesh
*
* UW-SLAM is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* UW-SLAM is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with UW-SLAM. If not, see <http://www.gnu.org/licenses/>.
*/
#pragma once
#include "Eigen/Core"
#include "ceres/ceres.h"
#include "sophus/sim3.hpp"
#include "sophus/se3.hpp"
#include "sophus/so3.hpp"

// This workaround creates a template specilization for Eigen's cast_impl,
// when casting from a ceres::Jet type. It relies on Eigen's internal API and
// might break with future versions of Eigen.
namespace Eigen {
namespace internal {

template <class T, int N, typename NewType>
struct cast_impl<ceres::Jet<T, N>, NewType> {
  EIGEN_DEVICE_FUNC
  static inline NewType run(ceres::Jet<T, N> const& x) {
    return static_cast<NewType>(x.a);
  }
};

}  // namespace internal
}  // namespace Eigen



namespace vi
{

#define SSEE(val,idx) (*(((float*)&val)+idx))
#define ALIGN __attribute__((__aligned__(16)))

typedef Sophus::SE3f SE3;
typedef Sophus::Sim3f Sim3;
typedef Sophus::SO3f SO3;

typedef Eigen::Vector4d QuaternionVector;
typedef Eigen::Vector3d TranslationVector;
typedef Eigen::Quaternion<SE3::Scalar> Quaternion;
typedef Eigen::Matrix<double,3,1> Mat31d;
typedef Eigen::Matrix<double,3,3> Mat33d;
typedef Eigen::Matrix<double,4,1> Mat41d;
typedef Eigen::Matrix<double,4,4> Mat44d;
typedef Eigen::Matrix<double,6,1> Mat61d;
typedef Eigen::Matrix<double,6,6> Mat66d;
typedef Eigen::Matrix<double,6,7> Mat67d;

typedef Eigen::Matrix<float,3,1> Mat31f;
typedef Eigen::Matrix<float,3,3> Mat33f;
typedef Eigen::Matrix<float,3,4> Mat34f;
typedef Eigen::Matrix<float,4,1> Mat41f;
typedef Eigen::Matrix<float,4,4> Mat44f;
typedef Eigen::Matrix<float,6,1> Mat61f;
typedef Eigen::Matrix<float,6,6> Mat66f;
typedef Eigen::Matrix<float,6,7> Mat67f;

// Global constants
extern const int PYRAMID_LEVELS;
extern int BLOCK_SIZE;
extern double GRADIENT_THRESHOLD;

}