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
#include "covariance_geometry/pose_composition.hpp"
#include "covariance_geometry/pose_covariance_composition.hpp"
#include "covariance_geometry/pose_covariance_representation.hpp"

#include "covariance_geometry_ros/covariance_geometry_ros.hpp"
#include "covariance_geometry_ros/utils.hpp"

namespace covariance_geometry
{
void compose(const Pose & a, const Pose & b, Pose & out)
{
  PoseQuaternion pose_a, pose_b, pose_out;
  fromROS(a, pose_a);
  fromROS(b, pose_b);
  ComposePose3DQuaternion(pose_a, pose_b, pose_out);
  toROS(pose_out, out);
}

void compose(
  const PoseWithCovariance & a, const PoseWithCovariance & b,
  PoseWithCovariance & out)
{
  PoseQuaternionCovarianceRPY pose_a, pose_b, pose_out;
  fromROS(a, pose_a);
  fromROS(b, pose_b);
  ComposePoseQuaternionCovarianceRPY(pose_a, pose_b, pose_out);
  toROS(pose_out, out);
}

void compose(
  const PoseWithCovariance & a, const Pose & b,
  PoseWithCovariance & out)
{
  PoseQuaternionCovarianceRPY pose_a, pose_out;
  PoseQuaternion pose_b;
  fromROS(a, pose_a);
  fromROS(b, pose_b);
  ComposePose3DQuaternion(pose_a.first, pose_b, pose_out.first);
  pose_out.second = pose_a.second;
  toROS(pose_out, out);
}

void compose(
  const Pose & a, const PoseWithCovariance & b,
  PoseWithCovariance & out)
{
  PoseQuaternion pose_a;
  PoseQuaternionCovarianceRPY pose_b, pose_out;
  fromROS(a, pose_a);
  fromROS(b, pose_b);
  ComposePose3DQuaternion(pose_a, pose_b.first, pose_out.first);
  pose_out.second = pose_b.second;
  toROS(pose_out, out);
}

// Compose a with b^-1
void inverseCompose(const Pose & a, const Pose & b, Pose & out)
{
  PoseQuaternion pose_a, pose_b, pose_out;
  fromROS(a, pose_a);
  fromROS(b, pose_b);
  ComposePose3DQuaternion(pose_a, InversePose(pose_b), pose_out);
  toROS(pose_out, out);
}

void inverseCompose(
  const PoseWithCovariance & a, const PoseWithCovariance & b,
  PoseWithCovariance & out)
{
  PoseQuaternionCovarianceRPY pose_a, pose_b, pose_out;
  fromROS(a, pose_a);
  fromROS(b, pose_b);
  ComposePoseQuaternionCovarianceRPY(
    pose_a, inversePose3DQuaternionCovarianceRPY(pose_b), pose_out);
  toROS(pose_out, out);
}

void inverseCompose(
  const PoseWithCovariance & a, const Pose & b,
  PoseWithCovariance & out)
{
  PoseQuaternionCovarianceRPY pose_a, pose_out;
  PoseQuaternion pose_b;
  fromROS(a, pose_a);
  fromROS(b, pose_b);
  ComposePose3DQuaternion(pose_a.first, InversePose(pose_b), pose_out.first);
  pose_out.second = pose_a.second;
  toROS(pose_out, out);
}
void inverseCompose(
  const Pose & a, const PoseWithCovariance & b,
  PoseWithCovariance & out)
{
  PoseQuaternion pose_a;
  PoseQuaternionCovarianceRPY pose_b, pose_out;
  fromROS(a, pose_a);
  fromROS(b, pose_b);
  pose_b = inversePose3DQuaternionCovarianceRPY(pose_b);
  ComposePose3DQuaternion(
    pose_a, pose_b.first,
    pose_out.first);
  pose_out.second = pose_b.second;
  toROS(pose_out, out);
}
}  // namespace covariance_geometry
