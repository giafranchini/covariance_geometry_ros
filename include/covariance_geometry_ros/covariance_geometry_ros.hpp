// Copyright 2023 Andrea Ostuni, Giacomo Franchini - PIC4SeR
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#ifndef COVARIANCE_GEOMETRY_ROS__COVARIANCE_GEOMETRY_ROS_HPP_
#define COVARIANCE_GEOMETRY_ROS__COVARIANCE_GEOMETRY_ROS_HPP_

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_with_covariance.hpp>
// #include <tf2/LinearMath/Transform.h>

using Pose = geometry_msgs::msg::Pose;
using PoseWithCovariance = geometry_msgs::msg::PoseWithCovariance;

namespace covariance_geometry
{
/** @name  Pose composition: out = a (+) b
  @{ */
void compose(const Pose & a, const Pose & b, Pose & out);
void compose(
  const PoseWithCovariance & a, const PoseWithCovariance & b,
  PoseWithCovariance & out);
void compose(
  const PoseWithCovariance & a, const Pose & b,
  PoseWithCovariance & out);
void compose(
  const Pose & a, const PoseWithCovariance & b,
  PoseWithCovariance & out);

// Return-by-value versions:
static inline Pose compose(const Pose & a, const Pose & b)
{
  Pose out;
  compose(a, b, out);
  return out;
}
static inline PoseWithCovariance compose(
  const PoseWithCovariance & a,
  const PoseWithCovariance & b)
{
  PoseWithCovariance out;
  compose(a, b, out);
  return out;
}
static inline PoseWithCovariance compose(
  const PoseWithCovariance & a,
  const Pose & b)
{
  PoseWithCovariance out;
  compose(a, b, out);
  return out;
}
static inline PoseWithCovariance compose(
  const Pose & a,
  const PoseWithCovariance & b)
{
  PoseWithCovariance out;
  compose(a, b, out);
  return out;
}

// Return-by-value versions using TF2 transforms:
// PoseWithCovariance compose(const PoseWithCovariance &a,
//                            const tf2::Transform &b);
/** @} */
/** @name  Pose inverse composition (a "as seen from" b): out = a (-) b
      @{ */
void inverseCompose(const Pose & a, const Pose & b, Pose & out);
void inverseCompose(
  const PoseWithCovariance & a, const PoseWithCovariance & b,
  PoseWithCovariance & out);
void inverseCompose(
  const PoseWithCovariance & a, const Pose & b,
  PoseWithCovariance & out);
void inverseCompose(
  const Pose & a, const PoseWithCovariance & b,
  PoseWithCovariance & out);
// Return-by-value versions:
static inline Pose inverseCompose(const Pose & a, const Pose & b)
{
  Pose out;
  inverseCompose(a, b, out);
  return out;
}
static inline PoseWithCovariance inverseCompose(
  const PoseWithCovariance & a,
  const PoseWithCovariance & b)
{
  PoseWithCovariance out;
  inverseCompose(a, b, out);
  return out;
}
static inline PoseWithCovariance inverseCompose(
  const PoseWithCovariance & a,
  const Pose & b)
{
  PoseWithCovariance out;
  inverseCompose(a, b, out);
  return out;
}
static inline PoseWithCovariance inverseCompose(
  const Pose & a,
  const PoseWithCovariance & b)
{
  PoseWithCovariance out;
  inverseCompose(a, b, out);
  return out;
}
// Return-by-value versions using TF2 transforms:
// PoseWithCovariance inverseCompose(const PoseWithCovariance &a,
//                                   const tf2::Transform &b);

/** @} */
}  // namespace covariance_geometry

#endif  // COVARIANCE_GEOMETRY_ROS__COVARIANCE_GEOMETRY_ROS_HPP_
