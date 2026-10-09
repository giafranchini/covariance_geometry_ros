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

#include <tf2/LinearMath/Transform.hpp>

#include <geometry_msgs/msg/pose_with_covariance.hpp>
#include "covariance_geometry/pose_representation.hpp"
#include "covariance_geometry/pose_covariance_representation.hpp"
#include "covariance_geometry_ros/covariance_geometry_ros.hpp"

#include "gtest/gtest.h"

namespace covariance_geometry
{

const Eigen::Vector3d coord1 = {-0.333330, 0, 0.100000};                // x, y, z
const Eigen::Quaterniond quat1 = {0.707107, 0.0, 0.707107, -0.000005};  // w, x, y, z
const Eigen::Vector3d coord2 = {1.435644, 0, 0.0};                      // x, y, z
const Eigen::Quaterniond quat2 = {1.0, 0.0, 0.0, 0.0};                  // w, x, y, z

TEST(Composition, ROSPoses)
{
  geometry_msgs::msg::Pose pose1, pose2, pose_out;

  pose1.position.x = coord1.x();
  pose1.position.y = coord1.y();
  pose1.position.z = coord1.z();
  pose1.orientation.x = quat1.x();
  pose1.orientation.y = quat1.y();
  pose1.orientation.z = quat1.z();
  pose1.orientation.w = quat1.w();

  pose2.position.x = coord2.x();
  pose2.position.y = coord2.y();
  pose2.position.z = coord2.z();
  pose2.orientation.x = quat2.x();
  pose2.orientation.y = quat2.y();
  pose2.orientation.z = quat2.z();
  pose2.orientation.w = quat2.w();

  pose_out = compose(pose1, pose2);
  EXPECT_NEAR(pose_out.position.x, coord1.x(), 1e-6);
}

TEST(Composition, ROSPoseAndTf)
{
  geometry_msgs::msg::Pose pose, pose_out;
  tf2::Transform tf;

  pose.position.x = coord1.x();
  pose.position.y = coord1.y();
  pose.position.z = coord1.z();
  pose.orientation.x = quat1.x();
  pose.orientation.y = quat1.y();
  pose.orientation.z = quat1.z();
  pose.orientation.w = quat1.w();

  tf.setOrigin(tf2::Vector3(coord2.x(), coord2.y(), coord2.z()));
  tf.setRotation(tf2::Quaternion(quat2.x(), quat2.y(), quat2.z(), quat2.w()));

  pose_out = compose(pose, tf);
  EXPECT_NEAR(pose_out.position.x, coord1.x(), 1e-6);
}

TEST(Composition, ROSPoseAndTfWithCovariance)
{
  geometry_msgs::msg::PoseWithCovariance pose, pose_out;
  tf2::Transform tf;

  pose.pose.position.x = coord1.x();
  pose.pose.position.y = coord1.y();
  pose.pose.position.z = coord1.z();
  pose.pose.orientation.x = quat1.x();
  pose.pose.orientation.y = quat1.y();
  pose.pose.orientation.z = quat1.z();
  pose.pose.orientation.w = quat1.w();

  tf.setOrigin(tf2::Vector3(coord2.x(), coord2.y(), coord2.z()));
  tf.setRotation(tf2::Quaternion(quat2.x(), quat2.y(), quat2.z(), quat2.w()));

  pose_out = compose(pose, tf);
  EXPECT_NEAR(pose_out.pose.position.x, coord1.x(), 1e-6);
}

TEST(Composition, ROSPosesWithCovariance)
{
  geometry_msgs::msg::PoseWithCovariance pose1, pose2, pose_out;

  pose1.pose.position.x = coord1.x();
  pose1.pose.position.y = coord1.y();
  pose1.pose.position.z = coord1.z();
  pose1.pose.orientation.x = quat1.x();
  pose1.pose.orientation.y = quat1.y();
  pose1.pose.orientation.z = quat1.z();
  pose1.pose.orientation.w = quat1.w();

  pose2.pose.position.x = coord2.x();
  pose2.pose.position.y = coord2.y();
  pose2.pose.position.z = coord2.z();
  pose2.pose.orientation.x = quat2.x();
  pose2.pose.orientation.y = quat2.y();
  pose2.pose.orientation.z = quat2.z();
  pose2.pose.orientation.w = quat2.w();

  pose_out = compose(pose1, pose2);
  EXPECT_NEAR(pose_out.pose.position.x, coord1.x(), 1e-6);
}
}  // namespace covariance_geometry
