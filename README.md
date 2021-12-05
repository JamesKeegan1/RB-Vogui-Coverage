# RB-Vogui-Coverage

Follow intructions below to download current RB Vogui simulation files

https://github.com/RobotnikAutomation/rbvogui_sim

# Simulation
To start Gazebo and Rviz simulation of RB Vogui and UR5, use the following code 

roslaunch rbvogui_sim_bringup rbvogui_complete.launch robot_model:=rbvogui robot_xacro:=rbvogui_std_ur5.urdf.xacro launch_arm:=true arm_manufacturer:=ur arm_model:=ur5 run_localization:=true run_navigation:=true

![123](https://user-images.githubusercontent.com/68841450/144745678-0ff4a351-a85f-4c21-bd91-fa59314aa724.png)

# Manipulator Movement
To allow movement of the UR5 Manipulator, use the following code

ROS_NAMESPACE=robot roslaunch rbvogui_ur5_moveit move_group.launch

![1234](https://user-images.githubusercontent.com/68841450/144745684-c13a5515-69f5-48c2-bb88-217121e155c2.png)

# Python Algorithm
Once simulation and manipulator movement codes are finished initialising, run this code to start the python algorithm

ROS_NAMESPACE=robot rosrun ur5_mover ur5_mover.py

# Matlab Algorithm
Open the folder shown in the image below
![Pic 1](https://user-images.githubusercontent.com/68841450/144745525-cac5b59d-2302-46d9-9df0-10a21fd12ab1.png)

Choose the main.m file as shown in the image below!
![Screenshot from 2021-12-05 22-47-14](https://user-images.githubusercontent.com/68841450/144745721-75d6bf91-c1e1-4047-b536-3d560f3e1d1f.png)

Ensure that the main.m file is open in the MATLAB editor and run alongside the python algorithm above by clicking the run button in the image below
![Pic 2](https://user-images.githubusercontent.com/68841450/144745583-a23d31e7-1f99-43a5-a79d-584151145ed1.png)

# Worlds
There is a world file attached which has the sphere used in this simulation. To use this world file change the path in the RB Vogui launch file so it leads to this file

