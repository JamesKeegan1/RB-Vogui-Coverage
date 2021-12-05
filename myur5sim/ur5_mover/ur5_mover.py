#!/usr/bin/env python
# -*- coding: utf-8 -*-
#moveit_fk_demo.py


import rospy, sys
import math
from tf.transformations import quaternion_from_euler
import moveit_commander
import actionlib
from actionlib_msgs.msg import *
from copy import deepcopy
from geometry_msgs.msg import Twist, Pose, Point, Quaternion, PoseStamped
import moveit_msgs.msg
from std_msgs.msg import Header
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import String
from std_msgs.msg import Float64, Float64MultiArray, Int64
from time import sleep


# Basic Publisher that communicates with MATLAB
class PositionUpdater:
    def __init__(self):
        self.robotLoc = rospy.Publisher("/robot/locationCurrent", Float64)

# Basic Subscriber that communicates with MATLAB
class Updater:
    def __init__(self):
        self.update = Int64
        rospy.Subscriber("/robot/progressNow", Int64, self.callback)

    def callback(self, data):
        self.update = data.data

# Basic Subscriber that receives joint states from MATLAB
class PoseListener:
    def __init__(self):
        self.jointPositions = Float64MultiArray()
        self.isTrue = Float64
        self.isTrue = 0
        rospy.Subscriber("/robot/newPose", Float64MultiArray, self.callback)

    def callback(self, data):
        self.jointPositions = data.data
        self.isTrue = 1

# Class created to move UR5 in Gazebo
class MoveItFkDemo:
    def __init__(self):

        moveit_commander.roscpp_initialize(sys.argv)



        self.arm = moveit_commander.MoveGroupCommander('arm')

        self.end_effector_link = self.arm.get_end_effector_link()


        reference_frame = "robot_base_link"


        self.arm.set_pose_reference_frame(reference_frame)


        self.arm.allow_replanning(True)



        self.arm.set_goal_joint_tolerance(0.001)

        self.arm.set_goal_position_tolerance(0.01)
        self.arm.set_goal_orientation_tolerance(0.1)
        self.arm.set_planning_time(0.1)



        self.arm.set_max_acceleration_scaling_factor(0.5)
        self.arm.set_max_velocity_scaling_factor(0.5)

    def mover(self, jointState):

        self.arm.set_joint_value_target(jointState)
        self.arm.go()
        rospy.sleep(0.1)
        self.arm.stop()

# Class created to move RB Vogui's base position
class GoToPose():
    def __init__(self):

        self.goal_sent = False

        # What to do if shut down (e.g. Ctrl-C or failure)
        rospy.on_shutdown(self.shutdown)

        # Tell the action client that we want to spin a thread by default
        self.move_base = actionlib.SimpleActionClient("/robot/move_base", MoveBaseAction)
        rospy.loginfo("Wait for the action server to come up")

        # Allow up to 5 seconds for the action server to come up
        self.move_base.wait_for_server()

    def goto(self, pos, x, y, z, w):

        # Send a goal
        self.goal_sent = True
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'robot_map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = Pose(Point(pos['x'], pos['y'], 0.000), Quaternion(x, y, z, w))

        # Start moving
        self.move_base.send_goal(goal)

        # Allow TurtleBot up to 60 seconds to complete task
        success = self.move_base.wait_for_result(rospy.Duration(60))

        state = self.move_base.get_state()
        result = False

        if success and state == GoalStatus.SUCCEEDED:
            # We made it!
            rospy.loginfo("We got there!")
            result = True
        else:
            rospy.loginfo("Not quite!")
            self.move_base.cancel_goal()

        self.goal_sent = False
        return result

    def shutdown(self):
        if self.goal_sent:
            self.move_base.cancel_goal()
        rospy.loginfo("Stop")
        rospy.sleep(1)

if __name__ == "__main__":
    try:
        # Initialise Node, publishers and subscribers
        rospy.init_node('ur5_mover', anonymous=False)
        robotPos = PositionUpdater()
        listener = PoseListener()
        moverInit = MoveItFkDemo()
        voguiMovement = GoToPose()
        updaterSub = Updater()
        # Raise UR5 so it doesn't collide with object
        jointStates = [0, -3.141/2, 0, 0, 0, 0]
        moverInit.mover(jointStates)
        # Ensure RB Vogui Base is at the origin
        position = {'x': 0, 'y': 0}
        # quaternion_from_euler allows us to transfer roll, pitch and yaw values into a quaternion
        q = quaternion_from_euler(0, 0, 0)
        move = voguiMovement.goto(position, q[0], q[1], q[2], q[3])
        # Move to first inspection point
        position = {'x': 0.5, 'y': 0}
        q = quaternion_from_euler(0, 0, 0)
        move = voguiMovement.goto(position, q[0], q[1], q[2], q[3])
        zeroes = Float64MultiArray
        zeroes = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        # Set flag and valueRn to 0
        flag = 0
        valueRn = 0
        # Enter while loop while flag = 0
        while flag == 0:
            # Wait for new joint state message
            rospy.wait_for_message("/robot/newPose", Float64MultiArray)
            # Set variable jointStates to the current position published by MATLAB
            jointStates = listener.jointPositions
            # Publish jointStates variable to Gazebo
            moverInit.mover(jointStates)
            # valueRn is equal to the updaterSub
            valueRn = updaterSub.update
            # If valueRn is equal to 1 break the loop, otherwise continue
            if valueRn == 1:
                rospy.loginfo("We outta here")
                flag = 1
                break
            # Ball variable is published so MATLAB knows it can create a new joint state
            ball = 2.0
            robotPos.robotLoc.publish(ball)
            # Wait for new updaterSub value before repeating loop
            rospy.wait_for_message("/robot/progressNow", Int64)

        # Once out of loop publish ball value so MATLAB can recalculate joint states for new position
        ball = 2.0
        robotPos.robotLoc.publish(ball)
        position = {'x': 0.5, 'y': 0}                                           # The next few steps involve moving the RB Vogui to interim positions
        q = quaternion_from_euler(0, 0, 3.141/2)                                # These interim positions allow the mobile manipulator to reach the goal position
        move = voguiMovement.goto(position, q[0], q[1], q[2], q[3])             # without colliding with the object
        position = {'x': 0.5, 'y': 1}
        q = quaternion_from_euler(0, 0, 3.141/2)
        move = voguiMovement.goto(position, q[0], q[1], q[2], q[3])
        position = {'x': 0.5, 'y': 1}
        q = quaternion_from_euler(0, 0, 0)
        move = voguiMovement.goto(position, q[0], q[1], q[2], q[3])
        jointStates = [0, -3.141/2, 0, 0, 0, 0]
        moverInit.mover(jointStates)
        position = {'x': 1.5, 'y': 1}
        q = quaternion_from_euler(0, 0, 0)
        move = voguiMovement.goto(position, q[0], q[1], q[2], q[3])

        valueRn = 0                                                             # From here onwards the previous steps are repeated for the new manipulator paths
        flag = 0
        while flag == 0:
            rospy.wait_for_message("/robot/newPose", Float64MultiArray)
            jointStates = listener.jointPositions
            moverInit.mover(jointStates)
            valueRn = updaterSub.update
            if valueRn == 1:
                rospy.loginfo("We outta here")
                flag = 1
                break
            ball = 2.0
            robotPos.robotLoc.publish(ball)
            rospy.wait_for_message("/robot/progressNow", Int64)

        ball = 3.0
        robotPos.robotLoc.publish(ball)
        position = {'x': 2.5, 'y': 1}
        q = quaternion_from_euler(0, 0, 0)
        move = voguiMovement.goto(position, q[0], q[1], q[2], q[3])
        position = {'x': 2.5, 'y': 1}
        q = quaternion_from_euler(0, 0, -3.141/2)
        move = voguiMovement.goto(position, q[0], q[1], q[2], q[3])
        jointStates = [0, -3.141/2, 0, 0, 0, 0]
        moverInit.mover(jointStates)
        position = {'x': 2.5, 'y': 0}
        q = quaternion_from_euler(0, 0, -3.141/2)
        move = voguiMovement.goto(position, q[0], q[1], q[2], q[3])
        position = {'x': 2.5, 'y': 0}
        q = quaternion_from_euler(0, 0, 0)
        move = voguiMovement.goto(position, q[0], q[1], q[2], q[3])

        valueRn = 0
        flag = 0
        while flag == 0:
            rospy.wait_for_message("/robot/newPose", Float64MultiArray)
            jointStates = listener.jointPositions
            moverInit.mover(jointStates)
            valueRn = updaterSub.update
            if valueRn == 1:
                rospy.loginfo("We outta here")
                flag = 1
                break
            ball = 2.0
            robotPos.robotLoc.publish(ball)
            rospy.wait_for_message("/robot/progressNow", Int64)

        ball = 4.0
        robotPos.robotLoc.publish(ball)
        position = {'x': 2.5, 'y': 0}
        q = quaternion_from_euler(0, 0, -3.141/2)
        move = voguiMovement.goto(position, q[0], q[1], q[2], q[3])
        position = {'x': 2.5, 'y': -1}
        q = quaternion_from_euler(0, 0, -3.141/2)
        move = voguiMovement.goto(position, q[0], q[1], q[2], q[3])
        position = {'x': 2.5, 'y': -1}
        q = quaternion_from_euler(0, 0, -3.141)
        jointStates = [0, -3.141/2, 0, 0, 0, 0]
        moverInit.mover(jointStates)
        move = voguiMovement.goto(position, q[0], q[1], q[2], q[3])
        position = {'x': 1.5, 'y': -1}
        q = quaternion_from_euler(0, 0, -3.141)
        move = voguiMovement.goto(position, q[0], q[1], q[2], q[3])
        position = {'x': 1.5, 'y': -1}
        q = quaternion_from_euler(0, 0, 0)
        move = voguiMovement.goto(position, q[0], q[1], q[2], q[3])

        valueRn = 0
        flag = 0
        while flag == 0:
            rospy.wait_for_message("/robot/newPose", Float64MultiArray)
            jointStates = listener.jointPositions
            moverInit.mover(jointStates)
            valueRn = updaterSub.update
            if valueRn == 1:
                rospy.loginfo("We outta here")
                flag = 1
                break
            ball = 2.0
            robotPos.robotLoc.publish(ball)
            rospy.wait_for_message("/robot/progressNow", Int64)

        ball = 4.0
        robotPos.robotLoc.publish(ball)
        position = {'x': 1.5, 'y': -1}
        q = quaternion_from_euler(0, 0, -3.141)
        move = voguiMovement.goto(position, q[0], q[1], q[2], q[3])
        position = {'x': 0, 'y': -1}
        q = quaternion_from_euler(0, 0, -3.141)
        move = voguiMovement.goto(position, q[0], q[1], q[2], q[3])
        position = {'x': 0, 'y': 0}                                             # Returns to origin after last inspection
        q = quaternion_from_euler(0, 0, 3.141/2)
        move = voguiMovement.goto(position, q[0], q[1], q[2], q[3])
        position = {'x': 0, 'y': 0}
        q = quaternion_from_euler(0, 0, 0)
        move = voguiMovement.goto(position, q[0], q[1], q[2], q[3])
        ball = 6.0
        robotPos.robotLoc.publish(ball)
        jointStates = [0, 0, 0, 0, 0, 0]                                        # Sets UR5 to default position
        moverInit.mover(jointStates)

        rospy.sleep(1)

    except rospy.ROSInterruptException:
        pass
