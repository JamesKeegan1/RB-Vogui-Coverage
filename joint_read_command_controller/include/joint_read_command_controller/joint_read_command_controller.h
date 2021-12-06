///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2012, hiDOF INC.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//   * Redistributions of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//   * Neither the name of hiDOF, Inc. nor the names of its
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
 * Author: Wim Meeussen
 */

#ifndef JOINT_READ_COMMAND_CONTROLLER_H
#define JOINT_READ_COMMAND_CONTROLLER_H

#include <joint_read_command_controller/joint_read_interface.h>
#include <controller_interface/controller.h>
#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <sensor_msgs/JointState.h>
#include <realtime_tools/realtime_publisher.h>
#include <boost/shared_ptr.hpp>

namespace joint_read_command_controller
{
struct JointRead
{
  JointRead(hardware_interface::JointReadHandle& handle, const std::string& type, const std::string& joint_name)
  {
    this->handle = handle;
    this->type = type;
    this->joint_name = joint_name;
  }

  hardware_interface::JointReadHandle handle;
  std::string type;
  std::string joint_name;
};

class JointReadCommandController
    : public controller_interface::MultiInterfaceController<hardware_interface::PositionReadInterface,
                                                            hardware_interface::VelocityReadInterface,
                                                            hardware_interface::EffortReadInterface>
{
public:
  JointReadCommandController() : publish_rate_(0.0), controller_interface::MultiInterfaceController<hardware_interface::PositionReadInterface,
                                                            hardware_interface::VelocityReadInterface,
                                                            hardware_interface::EffortReadInterface> (true) // allows optional interfaces
  {
  }

  virtual bool init(hardware_interface::RobotHW* const robot_hw, ros::NodeHandle& root_nh,
                    ros::NodeHandle& controller_nh);

  virtual void starting(const ros::Time& time);
  virtual void update(const ros::Time& time, const ros::Duration& /*period*/);
  virtual void stopping(const ros::Time& /*time*/);

private:
  std::vector<hardware_interface::JointStateHandle> joint_state_;
  std::vector<JointRead> joint_reads_;

  boost::shared_ptr<realtime_tools::RealtimePublisher<sensor_msgs::JointState> > realtime_pub_;
  ros::Time last_publish_time_;
  double publish_rate_;
};
}

#endif  // JOINT_READ_COMMAND_CONTROLLER_H
