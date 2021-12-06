///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2018, Robotnik Automation, SLL
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//   * Redistributions of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//   * Neither the name of hiDOF Inc nor the names of its
//     contributors may be used to endorse or promote products derived from
//     this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//////////////////////////////////////////////////////////////////////////////

/*
 * Author: Marc Bosch-Jorge
 */

#include <algorithm>
#include <cstddef>

#include <cassert>
#include <string>
#include <hardware_interface/internal/hardware_resource_manager.h>
#include <hardware_interface/joint_command_interface.h>

#include <joint_read_command_controller/joint_read_command_controller.h>

namespace joint_read_command_controller
{
bool JointReadCommandController::init(hardware_interface::RobotHW* const robot_hw, ros::NodeHandle& root_nh,
                                      ros::NodeHandle& controller_nh)
{
  hardware_interface::JointStateInterface* js_iface = robot_hw->get<hardware_interface::JointStateInterface>();

  hardware_interface::PositionReadInterface* pos_iface = robot_hw->get<hardware_interface::PositionReadInterface>();
  hardware_interface::VelocityReadInterface* vel_iface = robot_hw->get<hardware_interface::VelocityReadInterface>();
  hardware_interface::EffortReadInterface* eff_iface = robot_hw->get<hardware_interface::EffortReadInterface>();

  std::vector<std::string> joint_names;
  if (pos_iface != 0)
  {
    const std::vector<std::string>& pos_names = pos_iface->getNames();
    for (size_t i = 0; i < pos_names.size(); i++)
    {
      hardware_interface::JointReadHandle h = pos_iface->getHandle(pos_names[i]);
      joint_reads_.push_back(JointRead(h, "pos", pos_names[i]));
      continue;
    }
    joint_names.insert(joint_names.end(), pos_names.begin(), pos_names.end());
  }
  if (vel_iface != 0)
  {
    const std::vector<std::string>& vel_names = vel_iface->getNames();
    for (size_t i = 0; i < vel_names.size(); i++)
    {
      hardware_interface::JointReadHandle h = vel_iface->getHandle(vel_names[i]);
      joint_reads_.push_back(JointRead(h, "vel", vel_names[i]));
      continue;
    }
    joint_names.insert(joint_names.end(), vel_names.begin(), vel_names.end());
  }
  if (eff_iface != 0)
  {
    const std::vector<std::string>& eff_names = eff_iface->getNames();
    for (size_t i = 0; i < eff_names.size(); i++)
    {
      hardware_interface::JointReadHandle h = eff_iface->getHandle(eff_names[i]);
      joint_reads_.push_back(JointRead(h, "eff", eff_names[i]));
      continue;
    }
    joint_names.insert(joint_names.end(), eff_names.begin(), eff_names.end());
  }

  // get publishing period
  if (!controller_nh.getParam("publish_rate", publish_rate_))
  {
    publish_rate_ = 0;
    ROS_WARN("Parameter 'publish_rate' not set. It will publish at control frequency");
  }

  // realtime publisher
  realtime_pub_.reset(new realtime_tools::RealtimePublisher<sensor_msgs::JointState>(root_nh, "joint_commands", 4));

  // get joints and allocate message
  for (unsigned i = 0; i < joint_reads_.size(); i++)
  {
    realtime_pub_->msg_.name.push_back(joint_reads_[i].joint_name);
    realtime_pub_->msg_.position.push_back(0.0);
    realtime_pub_->msg_.velocity.push_back(0.0);
    realtime_pub_->msg_.effort.push_back(0.0);
  }

  return true;
}

void JointReadCommandController::starting(const ros::Time& time)
{
  // initialize time
  last_publish_time_ = time;
}

void JointReadCommandController::update(const ros::Time& time, const ros::Duration& /*period*/)
{
  // limit rate of publishing
  if (publish_rate_ > 0.0 && last_publish_time_ + ros::Duration(1.0 / publish_rate_) > time)
    return;

  // try to publish
  if (realtime_pub_->trylock())
  {
    // we're actually publishing, so increment time
    last_publish_time_ = time;  // last_publish_time_ + ros::Duration(1.0/publish_rate_);

    // populate joint state message:
    // - fill only joints that are present in the EffortReadInterface, i.e. indices [0, num_hw_joints_)
    // - leave unchanged extra joints, which have static values, i.e. indices from num_hw_joints_ onwards
    realtime_pub_->msg_.header.stamp = time;
    for (unsigned i = 0; i < joint_reads_.size(); i++)
    {
      if (joint_reads_[i].type == "pos")
        realtime_pub_->msg_.position[i] = joint_reads_[i].handle.getCommand();
      if (joint_reads_[i].type == "vel")
        realtime_pub_->msg_.velocity[i] = joint_reads_[i].handle.getCommand();
      if (joint_reads_[i].type == "eff")
        realtime_pub_->msg_.effort[i] = joint_reads_[i].handle.getCommand();
    }
    realtime_pub_->unlockAndPublish();
  }
}

void JointReadCommandController::stopping(const ros::Time& /*time*/)
{
}
}

PLUGINLIB_EXPORT_CLASS(joint_read_command_controller::JointReadCommandController, controller_interface::ControllerBase)
