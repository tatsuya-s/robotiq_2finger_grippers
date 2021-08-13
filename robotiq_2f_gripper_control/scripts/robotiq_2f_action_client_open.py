#! /usr/bin/env python
"""--------------------------------------------------------------------
This Node/Script shows an example on how to use a `SimpleActionClient` instance
to control a Robotiq 2 Finger gripper.

Parameters:
    action_name: Name of the action advertised by the `ActionServer` controlling the gripper.

@author: Daniel Felipe Ordonez Apraez
@email: daniels.ordonez@gmail.com
--------------------------------------------------------------------"""

import rospy

# Brings in the SimpleActionClient
import actionlib

from robotiq_2f_gripper_msgs.msg import CommandRobotiqGripperFeedback, CommandRobotiqGripperResult, CommandRobotiqGripperAction, CommandRobotiqGripperGoal
from robotiq_2f_gripper_control.robotiq_2f_gripper_driver import Robotiq2FingerGripperDriver as Robotiq

def operate_gripper():

    action_name = rospy.get_param('~action_name', 'command_robotiq_action')
    robotiq_client = actionlib.SimpleActionClient(action_name, CommandRobotiqGripperAction)
    # Wait until grippers are ready to take command
    robotiq_client.wait_for_server()

    rospy.logwarn("Client test: Starting sending goals")
    ## Manually set all the parameters of the gripper goal state.
    ######################################################################################

    goal = CommandRobotiqGripperGoal()
    goal.emergency_release = False
    goal.stop = False
    goal.position = 0.085
    goal.speed = 0.01
    goal.force = 100.0

    # Sends the goal to the gripper.
    robotiq_client.send_goal(goal)
    # Block processing thread until gripper movement is finished, comment if waiting is not necesary.
    robotiq_client.wait_for_result()

    # Prints out the result of executing the action
    return robotiq_client.get_result()

if __name__ == '__main__':
    # Initializes a rospy node so that the SimpleActionClient can
    # publish and subscribe over ROS.
    rospy.init_node('robotiq_2f_client_open')
    result = operate_gripper()