# RB-Vogui-Coverage

Follow intructions below to download current RB Vogui simulation files

https://github.com/RobotnikAutomation/rbvogui_sim

# Simulation
To start Gazebo and Rviz simulation of RB Vogui and UR5, use the following code 

roslaunch rbvogui_sim_bringup rbvogui_complete.launch robot_model:=rbvogui robot_xacro:=rbvogui_std_ur5.urdf.xacro launch_arm:=true arm_manufacturer:=ur arm_model:=ur5 run_localization:=true run_navigation:=true

# Manipulator Movement
To allow movement of the UR5 Manipulator, use the following code

ROS_NAMESPACE=robot roslaunch rbvogui_ur5_moveit move_group.launch

# Python Algorithm
Once simulation and manipulator movement codes are finished initialising, run this code to start the python algorithm



# Matlab Algorithm
Once the python algorithm has started, run the main file shown below
