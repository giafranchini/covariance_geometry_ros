#ifndef COVARIANCE_GEOMETRY_ROS_UTILS_HPP_
#define COVARIANCE_GEOMETRY_ROS_UTILS_HPP_

#include <eigen3/Eigen/Core>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_with_covariance.hpp>
// #include <tf2/LinearMath/Transform.h>

using Pose = geometry_msgs::msg::Pose;
using PoseWithCovariance = geometry_msgs::msg::PoseWithCovariance;

namespace covariance_geometry
{
  /** @name  Convert from ROS msgs to covariance geometry ROS type
  @{ */
  inline void fromROS(const Pose &in, PoseQuaternion &out)
  {
    out.first.x() = in.position.x;
    out.first.y() = in.position.y;
    out.first.z() = in.position.z;
    out.second.x() = in.orientation.x;
    out.second.y() = in.orientation.y;
    out.second.z() = in.orientation.z;
    out.second.w() = in.orientation.w;
  }

  inline void fromROS(const PoseWithCovariance &in, PoseQuaternionCovarianceRPY &out)
  {
    fromROS(in.pose, out.first);
    Eigen::Map<const Eigen::Matrix<double, 6, 6, Eigen::RowMajor>> cov(in.covariance.data());
    out.second = cov;
  }

  // inline void fromROS(const tf2::Transform &in, PoseQuaternion &out)
  // {
  //   out.first(in.getOrigin());
  //   out.second(in.getRotation());
  // }

  /** @name  Convert from covariance geometry ROS type to ROS msgs
  @{ */
  inline void toROS(const PoseQuaternion &in, Pose &out)
  {
    out.position.x = in.first.x();
    out.position.y = in.first.y();
    out.position.z = in.first.z();
    out.orientation.x = in.second.x();
    out.orientation.y = in.second.y();
    out.orientation.z = in.second.z();
    out.orientation.w = in.second.w();
  }

  inline void toROS(const PoseQuaternionCovarianceRPY &in, PoseWithCovariance &out)
  {
    toROS(in.first, out.pose);
    Eigen::Map<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>> cov(out.covariance.data());
    cov = in.second;
  }

  // inline void toROS(const PoseQuaternion &in, tf2::Transform &out)
  // {
  //   out.setOrigin(in.first);
  //   out.setRotation(in.second);
  // }
}

#endif // COVARIANCE_GEOMETRY_ROS_UTILS_HPP_