#pragma once

#include "boost/thread.hpp"
#include "util/MinimalImage.h"
#include "IOWrapper/Output3DWrapper.h"

#include "stdio.h"
#include "FullSystem/HessianBlocks.h"
#include "util/FrameShell.h"
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

namespace dso
{

class FrameHessian;
class CalibHessian;
class FrameShell;

namespace IOWrap
{

class ROSOutputWrapper : public Output3DWrapper
{
private:
  ros::NodeHandle nh_;
  ros::Publisher pose_pub_;

public:
  inline ROSOutputWrapper(ros::NodeHandle& nh) : nh_(nh)
  {
    // Construct publisher
    pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("pose", 1);
  }

  virtual ~ROSOutputWrapper()
  {
  }

  virtual void publishGraph(const std::map<uint64_t, Eigen::Vector2i, std::less<uint64_t>, Eigen::aligned_allocator<std::pair<const uint64_t, Eigen::Vector2i>>> &connectivity) override
  {
  }

  virtual void publishKeyframes( std::vector<FrameHessian*> &frames, bool final, CalibHessian* HCalib) override
  {
  }

  virtual void publishCamPose(FrameShell* frame, CalibHessian* HCalib) override
  {
    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header.stamp.fromSec(frame->timestamp);
    pose_msg.pose.orientation.w = frame->camToWorld.so3().unit_quaternion().w();
    pose_msg.pose.orientation.x = frame->camToWorld.so3().unit_quaternion().x();
    pose_msg.pose.orientation.y = frame->camToWorld.so3().unit_quaternion().y();
    pose_msg.pose.orientation.z = frame->camToWorld.so3().unit_quaternion().z();
    pose_msg.pose.position.x = frame->camToWorld.translation().transpose()[0];
    pose_msg.pose.position.y = frame->camToWorld.translation().transpose()[1];
    pose_msg.pose.position.z = frame->camToWorld.translation().transpose()[2];

    pose_pub_.publish(pose_msg);
  }


  virtual void pushLiveFrame(FrameHessian* image) override
  {
      // can be used to get the raw image / intensity pyramid.
  }

  virtual void pushDepthImage(MinimalImageB3* image) override
  {
      // can be used to get the raw image with depth overlay.
  }
  virtual bool needPushDepthImage() override
  {
      return false;
  }

  virtual void pushDepthImageFloat(MinimalImageF* image, FrameHessian* KF ) override
  {
  }
};
}
}
