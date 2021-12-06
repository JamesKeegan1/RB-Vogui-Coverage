# joint_read_command_controller

A ROS Control controller to publish commands issued to each joint controlled by other controllers through a topic, as a sensor_msgs/JointState.msg.

It provides a non exclusive access to the joints (as does JointStateController), but command writting is disabled.

## Usage

As a ROS Control controller, specify it on a yaml and then load it.

## Example Configuration

joint_read_command_controller:

&nbsp;&nbsp;&nbsp;&nbsp;type: joint_read_command_controller/JointReadCommandController
  
&nbsp;&nbsp;&nbsp;&nbsp;publish_rate: 65.0


If publish_rate parameter is not set or is 0, publish rate will be that of the control rate.

## How it works...

A new hardware_interface has been defined that does not claim resources. However, to be loadable it overrides internal demangling methods to mimic
the standard joint types (position, velocity, effort).

Then, the controller is a multi_interface controller that "controls" this "new" type of interface.
