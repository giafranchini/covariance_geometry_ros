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
#include "covariance_geometry/pose_covariance_representation.hpp"
#include "covariance_geometry_ros/utils.hpp"

#include "gtest/gtest.h"

using Pose = geometry_msgs::msg::Pose;
using PoseWithCovariance = geometry_msgs::msg::PoseWithCovariance;

namespace covariance_geometry
{

const Eigen::Vector3d coord = {0.111, -1.24, 0.35};                               // x, y, z
const Eigen::Vector3d rpy = {0.9067432, 0.4055079, 0.1055943};                    // roll, pitch, yaw
const Eigen::Quaterniond quat = {0.8746791, 0.4379822, 0.1581314, 0.1345454};     // w, x, y, z

const Eigen::Vector3d rpy_gl = {0.12, M_PI_2, 0.34};                              // roll, pitch, yaw
const Eigen::Quaterniond quat_gl = {0.6884861, 0.1612045, 0.6884861, 0.1612045};  // w, x, y, z

TEST(Conversion, PoseFromROS)
{
  Pose pose_in;
  PoseQuaternion pose_out;
  pose_in.position.x = coord.x();
  pose_in.position.y = coord.y();
  pose_in.position.z = coord.z();
  pose_in.orientation.x = quat.x();
  pose_in.orientation.y = quat.y();
  pose_in.orientation.z = quat.z();
  pose_in.orientation.w = quat.w();
  fromROS(pose_in, pose_out);
  EXPECT_TRUE(pose_out.first.isApprox(coord));
  EXPECT_TRUE(pose_out.second.isApprox(quat));
}

TEST(Conversion, PoseWithCovarianceFromROS)
{
  PoseWithCovariance pose_in;
  PoseQuaternionCovarianceRPY pose_out;
  pose_in.pose.position.x = coord.x();
  pose_in.pose.position.y = coord.y();
  pose_in.pose.position.z = coord.z();
  pose_in.pose.orientation.x = quat.x();
  pose_in.pose.orientation.y = quat.y();
  pose_in.pose.orientation.z = quat.z();
  pose_in.pose.orientation.w = quat.w();
  Eigen::Map<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>> cov(pose_in.covariance.data());
  cov = Eigen::Matrix<double, 6, 6, Eigen::RowMajor>::Random();
  cov.selfadjointView<Eigen::Upper>();
  Eigen::Matrix<double, 6, 6, Eigen::RowMajor> cov_out;
  cov_out = cov;
  fromROS(pose_in, pose_out);
  EXPECT_TRUE(pose_out.first.first.isApprox(coord));
  EXPECT_TRUE(pose_out.first.second.isApprox(quat));
  EXPECT_TRUE(pose_out.second.isApprox(cov_out));
}

TEST(Conversion, PoseToROS)
{
  PoseQuaternion pose_in;
  Pose pose_out;
  pose_in.first = coord;
  pose_in.second = quat;
  toROS(pose_in, pose_out);
  EXPECT_DOUBLE_EQ(pose_out.position.x, coord.x());
  EXPECT_DOUBLE_EQ(pose_out.position.y, coord.y());
  EXPECT_DOUBLE_EQ(pose_out.position.z, coord.z());
  EXPECT_DOUBLE_EQ(pose_out.orientation.x, quat.x());
  EXPECT_DOUBLE_EQ(pose_out.orientation.y, quat.y());
  EXPECT_DOUBLE_EQ(pose_out.orientation.z, quat.z());
  EXPECT_DOUBLE_EQ(pose_out.orientation.w, quat.w());
}

TEST(Conversion, PoseWithCovarianceToROS)
{
  PoseQuaternionCovarianceRPY pose_in;
  PoseWithCovariance pose_out;
  pose_in.first.first = coord;
  pose_in.first.second = quat;
  pose_in.second = Eigen::Matrix<double, 6, 6>::Random();
  pose_in.second.selfadjointView<Eigen::Upper>();

  toROS(pose_in, pose_out);
  EXPECT_DOUBLE_EQ(pose_out.pose.position.x, coord.x());
  EXPECT_DOUBLE_EQ(pose_out.pose.position.y, coord.y());
  EXPECT_DOUBLE_EQ(pose_out.pose.position.z, coord.z());
  EXPECT_DOUBLE_EQ(pose_out.pose.orientation.x, quat.x());
  EXPECT_DOUBLE_EQ(pose_out.pose.orientation.y, quat.y());
  EXPECT_DOUBLE_EQ(pose_out.pose.orientation.z, quat.z());
  EXPECT_DOUBLE_EQ(pose_out.pose.orientation.w, quat.w());
  Eigen::Map<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>> cov(pose_out.covariance.data());
  EXPECT_TRUE(cov.isApprox(pose_in.second));
}
}  // namespace covariance_geometry
