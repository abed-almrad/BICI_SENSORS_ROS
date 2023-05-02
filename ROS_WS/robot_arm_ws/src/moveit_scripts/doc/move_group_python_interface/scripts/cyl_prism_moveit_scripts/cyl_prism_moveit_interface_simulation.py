#!/usr/bin/env python3

# Software License Agreement (BSD License)
#
# Copyright (c) 2013, SRI International
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of SRI International nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Acorn Pooley, Mike Lautman

## BEGIN_SUB_TUTORIAL imports
##
## To use the Python MoveIt interfaces, we will import the `moveit_commander`_ namespace.
## This namespace provides us with a `MoveGroupCommander`_ class, a `PlanningSceneInterface`_ class,
## and a `RobotCommander`_ class. More on these below. We also import `rospy`_ and some messages that we will use:
##

# Python 2/3 compatibility imports
from __future__ import print_function
from six.moves import input

import sys
import copy
import rospy
import roslaunch
import moveit_commander
from gazebo_msgs.srv import SpawnModel, DeleteModel 
from std_srvs.srv import Empty
import geometry_msgs.msg
from geometry_msgs.msg import Pose, Point, Quaternion
import rosnode

from math import pi, tau, dist, fabs, cos
    
from std_msgs.msg import Float64
from moveit_commander.conversions import pose_to_list

def all_close(goal, actual, tolerance):
    """
    Convenience method for testing if the values in two lists are within a tolerance of each other.
    For Pose and PoseStamped inputs, the angle between the two quaternions is compared (the angle
    between the identical orientations q and -q is calculated correctly).
    @param: goal       A list of floats, a Pose or a PoseStamped
    @param: actual     A list of floats, a Pose or a PoseStamped
    @param: tolerance  A float
    @returns: bool
    """
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
        x0, y0, z0, qx0, qy0, qz0, qw0 = pose_to_list(actual)
        x1, y1, z1, qx1, qy1, qz1, qw1 = pose_to_list(goal)
        # Euclidean distance
        d = dist((x1, y1, z1), (x0, y0, z0))
        # phi = angle between orientations
        cos_phi_half = fabs(qx0 * qx1 + qy0 * qy1 + qz0 * qz1 + qw0 * qw1)
        return d <= tolerance and cos_phi_half >= cos(tolerance / 2.0)

    return True


class MoveGroupPythonInterfaceTutorial(object):
    """MoveGroupPythonInterfaceTutorial"""

    def __init__(self):
        super(MoveGroupPythonInterfaceTutorial, self).__init__()

        ## BEGIN_SUB_TUTORIAL setup
        ##
        ## First initialize `moveit_commander`_ and a `rospy`_ node:
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("move_group_python_interface_simulation", anonymous=True)

        ## Instantiate a `RobotCommander`_ object. Provides information such as the robot's
        ## kinematic model and the robot's current joint states
        robot = moveit_commander.RobotCommander()

        ## Instantiate a `PlanningSceneInterface`_ object.  This provides a remote interface
        ## for getting, setting, and updating the robot's internal understanding of the
        ## surrounding world:
        scene = moveit_commander.PlanningSceneInterface()

        ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
        ## to a planning group (group of joints).  In this tutorial the group is the primary
        ## arm joints in the Panda robot, so we set the group's name to "panda_arm".
        ## If you are using a different robot, change this value to the name of your robot
        ## arm planning group.
        ## This interface can be used to plan and execute motions:
        group_name = "arm"
        move_group = moveit_commander.MoveGroupCommander(group_name)

        ## END_SUB_TUTORIAL

        ## BEGIN_SUB_TUTORIAL basic_info
        ##
        ## Getting Basic Information
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^
        # We can get the name of the reference frame for this robot:
        planning_frame = move_group.get_planning_frame()
        print("============ Planning frame: %s" % planning_frame)

        # We can also print the name of the end-effector link for this group:
        eef_link = move_group.get_end_effector_link()
        print("============ End effector link: %s" % eef_link)

        # We can get a list of all the groups in the robot:
        group_names = robot.get_group_names()
        print("============ Available Planning Groups:", robot.get_group_names())
        current_pose = move_group.get_current_pose().pose

        print("============ End effector current state")
        
        print("x: ", current_pose.position.x)
        print("y: ", current_pose.position.y)
        print("z: ", current_pose.position.z)
        print("q0: ", current_pose.orientation.x)
        print("q1: ", current_pose.orientation.y)
        print("q2: ", current_pose.orientation.z)
        print("q3: ", current_pose.orientation.w)


        # Misc variables
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names

    def go_to_joint_state(self, j_0, j_1, j_2, j_3, j_4, j_5):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        move_group = self.move_group

        ## BEGIN_SUB_TUTORIAL plan_to_joint_state
        ##
        ## Planning to a Joint Goal
        ## ^^^^^^^^^^^^^^^^^^^^^^^^
        ## The Panda's zero configuration is at a `singularity <https://www.quora.com/Robotics-What-is-meant-by-kinematic-singularity>`_, so the first
        ## thing we want to do is move it to a slightly better configuration.
        ## We use the constant `tau = 2*pi <https://en.wikipedia.org/wiki/Turn_(angle)#Tau_proposals>`_ for convenience:
        # We get the joint values from the group and change some of the values:
        joint_goal = move_group.get_current_joint_values()
        joint_goal[0] = j_0*pi/180
        joint_goal[1] = j_1*pi/180
        joint_goal[2] = j_2*pi/180
        joint_goal[3] = j_3*pi/180
        joint_goal[4] = j_4*pi/180
        joint_goal[5] = j_5*pi/180

        # The go command can be called with joint values, poses, or without any
        # parameters if you have already set the pose or joint target for the group
        move_group.go(joint_goal, wait=True)

        # Calling ``stop()`` ensures that there is no residual movement
        move_group.stop()

        ## END_SUB_TUTORIAL

        # For testing:
        current_joints = move_group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.01)

    def go_to_pose_goal(self,x,y,z,q0,q1,q2,q3):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        move_group = self.move_group

        ## BEGIN_SUB_TUTORIAL plan_to_pose
        ##
        ## Planning to a Pose Goal
        ## ^^^^^^^^^^^^^^^^^^^^^^^
        ## We can plan a motion for this group to a desired pose for the
        ## end-effector:
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.position.x = x
        pose_goal.position.y = y
        pose_goal.position.z = z
        pose_goal.orientation.x = q0
        pose_goal.orientation.y = q1
        pose_goal.orientation.z = q2
        pose_goal.orientation.w = q3

        print("============ End effector goal pose")
        print("x: ", pose_goal.position.x)
        print("y: ", pose_goal.position.y)
        print("z: ", pose_goal.position.z)
        print("q0: ", pose_goal.orientation.x)
        print("q1: ", pose_goal.orientation.y)
        print("q2: ", pose_goal.orientation.z)
        print("q3: ", pose_goal.orientation.w)

        move_group.set_pose_target(pose_goal)
        ## Now, we call the planner to compute the plan and execute it.
        # `go()` returns a boolean indicating whether the planning and execution was successful.
        success = move_group.go(wait=True)
        # Calling `stop()` ensures that there is no residual movement
        move_group.stop()
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets().
        move_group.clear_pose_targets()
        current_pose = move_group.get_current_pose().pose
        print("Tolerance check passed: ", all_close(pose_goal, current_pose, 0.01))

            ## END_SUB_TUTORIAL

        print("============ End effector state (After abiding the tolerance value)")
        
        print("x: ", current_pose.position.x)
        print("y: ", current_pose.position.y)
        print("z: ", current_pose.position.z)
        print("q0: ", current_pose.orientation.x)
        print("q1: ", current_pose.orientation.y)
        print("q2: ", current_pose.orientation.z)
        print("q3: ", current_pose.orientation.w)




def main(i, desired_grasp_attempt, x, y, z, q0, q1, q2, q3, j_0, j_1, j_2, j_3, j_4, j_5, j_0_0_pos, j_4_0_pos, j_8_0_pos):
    
    try:

        rospy.sleep(1.0)

        if desired_grasp_attempt == "grasp_4":

            j_12_0_publisher.publish(data=0)
            j_13_0_publisher.publish(data=-0.5)
            j_14_0_publisher.publish(data=0.52)
            j_15_0_publisher.publish(data=0.52)

        else:

            j_12_0_publisher.publish(data=0)
            j_13_0_publisher.publish(data=-0.5)
            j_14_0_publisher.publish(data=0.52)
            j_15_0_publisher.publish(data=0.52)

            j_3_0_publisher.publish(data=-0.52)
            j_7_0_publisher.publish(data=-0.52)
            j_11_0_publisher.publish(data=-0.52)

    #        rospy.sleep(1.0)

            j_2_0_publisher.publish(data=-0.52)
            j_6_0_publisher.publish(data=-0.52)
            j_10_0_publisher.publish(data=-0.52)

    #        rospy.sleep(1.0)

            j_1_0_publisher.publish(data=-0.52)
            j_5_0_publisher.publish(data=-0.52)
            j_9_0_publisher.publish(data=-0.52)

    #        rospy.sleep(1.0)

    #        rospy.sleep(1.0)
        if desired_grasp_attempt == "grasp_3":
            tutorial.go_to_joint_state(64.84, -39.99, 61.42, -17.67, -26.27, -93.37)

        if desired_grasp_attempt == "grasp_4":
            tutorial.go_to_joint_state(62.16, -39.57, 66.44, -24.85, -28.24, -89.46)

            tutorial.go_to_joint_state(62.16, -62.9, 68.44, -24.85, -28.24, -89.46)            

        tutorial.go_to_joint_state(j_0, j_1, j_2, j_3, j_4, j_5)
        
#        print("Press enter to continue")
#        input()

#        input(
#            "============ Press `Enter` to approach the object from the first angle ..."
#        )
        rospy.sleep(1.0)

        tutorial.go_to_pose_goal(x, y, z, q0, q1, q2, q3)
        
#        print("Press enter to continue")
#        input()


#        print("============ Approach "+str(i)+" is complete!")

        rospy.sleep(1.0)
#        input(
#            "============ Press `Enter` to close the allegro hand fingers against the object..."
#        )

        if desired_grasp_attempt == "grasp_4":

            j_0_0_publisher.publish(data=j_0_0_pos)
            j_4_0_publisher.publish(data=j_4_0_pos)
            j_8_0_publisher.publish(data=j_8_0_pos)

            rospy.sleep(0.5)
        else:

            j_0_0_publisher.publish(data=j_0_0_pos)
            j_4_0_publisher.publish(data=j_4_0_pos)
            j_8_0_publisher.publish(data=j_8_0_pos)

            rospy.sleep(1.0)

            j_1_0_publisher.publish(data=0.52)
            j_5_0_publisher.publish(data=0.52)
            j_9_0_publisher.publish(data=0.52)

            rospy.sleep(1.0)

            j_2_0_publisher.publish(data=0.52)
            j_6_0_publisher.publish(data=0.52)
            j_10_0_publisher.publish(data=0.52)

            rospy.sleep(1.0)

            j_3_0_publisher.publish(data=0.52)
            j_7_0_publisher.publish(data=0.52)
            j_11_0_publisher.publish(data=0.52)

            rospy.sleep(0.5)

#        input(
#            "============ Press `Enter` to register the contact point cloud..."
#        )
        launch.start()
        process = launch.launch(saving_node)
        print("The saving node is activated: ", process.is_alive())

        rospy.sleep(0.5)

#        input(
#            "============ Press `Enter` to delete the ur5 workstation..."
#        )
        process = launch.launch(deletion_node)
        print("The deletion node is activated: ", process.is_alive())
        rospy.sleep(1.0)

#        input(
#            "============ Press `Enter` to spawn a new workstation at initial position..."
#        )
        spawner("ur5e_workstation_0", open("/home/abed/Documents/BICI_Project/BICI_SENSORS_ROS/ROS_WS/robot_arm_ws/src/coro_workstations/coro_descriptions/sdf/workstations/ur5e_workstation.sdf",'r').read(),"/gazebo/", Pose(position= Point(0,0,0),orientation=Quaternion(0,0,0,1)),"world")


        if(i==0):
            for j in range(1):
#                input(
#                    "============ Press `Enter` to kill the controller manager..."
#                )  
                rosnode.kill_nodes(["ros_control_controller_spawner"])
#                input(
#                    "============ Press `Enter` to load the controllers..."
#                )        
                launch.launch(control_mngr_node)
        else:
            for j in range(2):
#                input(
#                    "============ Press `Enter` to kill the controller manager..."
#                )  
                rosnode.kill_nodes(["ros_control_controller_spawner"])
#                input(
#                    "============ Press `Enter` to load the controllers..."
#                )        
                launch.launch(control_mngr_node)
                rospy.sleep(1.0)


    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


if __name__ == "__main__":

    print("")
    print("----------------------------------------------------------")
    print("Welcome to the interface for our grasping experiments")
    print("----------------------------------------------------------")
    print("Press Ctrl-D to exit at any time")
    print("")

    #Creating a dictionary to store the desired end effector and some joints poses for each grasp
    pos_dict =	{}

    pos_dict['grasp_0'] = {'x': -0.03750214465702802, 'y': -0.10263251236903577, 'z': 1.1511266381958,   # x was increased by 6.3 mm to reach contact and y was decreased by 2 mm
    'q0': 0.5091846113100845, 'q1': 0.5168832012167359, 'q2': -0.4824900672250095, 'q3': 0.4906792464881148,
    'j_0': 50.57, 'j_1': -76, 'j_2': 127.08, 'j_3': -48.04, 'j_4': 49.7, 'j_5': -90.99,
    'joints_pos_0':{'j_0_0_pos': -0.00761379, 'j_4_0_pos': -0.028295, 'j_8_0_pos': -0.0994968},
    'joints_pos_1':{'j_0_0_pos': -0.0214588, 'j_4_0_pos': -0.030919, 'j_8_0_pos': -0.106071},
    'joints_pos_2':{'j_0_0_pos': -0.0231649, 'j_4_0_pos': -0.0264939, 'j_8_0_pos': -0.103385},
    'joints_pos_3':{'j_0_0_pos': -0.00470707, 'j_4_0_pos': -0.0112859, 'j_8_0_pos': -0.0987053},
    'joints_pos_4':{'j_0_0_pos': 0.00784284, 'j_4_0_pos': -0.022653, 'j_8_0_pos': -0.100257},
    'joints_pos_5':{'j_0_0_pos': 0.0110093, 'j_4_0_pos': -0.0294931, 'j_8_0_pos': -0.103696},
    'joints_pos_6':{'j_0_0_pos': -0.0103135, 'j_4_0_pos': -0.0102128, 'j_8_0_pos': -0.110373},
    'joints_pos_7':{'j_0_0_pos': -0.0173443, 'j_4_0_pos': -0.0368234, 'j_8_0_pos': -0.0956433},
    'joints_pos_8':{'j_0_0_pos': 0.0034963, 'j_4_0_pos': -0.0126846, 'j_8_0_pos': -0.0967949},
    'joints_pos_9':{'j_0_0_pos': -0.00396311, 'j_4_0_pos': -0.012391, 'j_8_0_pos': -0.108488}}

    pos_dict['grasp_1'] = {'x': -0.11032590780479959, 'y': -0.23447572859494245, 'z': 1.1511266381958,     # x was increased by 3.5 mm to reach contact and y was increased by 3.5 mm
    'q0': 0.008640351269418915, 'q1': 0.7015205662192607, 'q2': -0.011668699676670028, 'q3': 0.7125012848754766,
    'j_0': 62.16, 'j_1': -39.57, 'j_2': 66.44, 'j_3': -24.85, 'j_4': -28.24, 'j_5': -89.46,
    'joints_pos_0':{'j_0_0_pos': -0.000504862, 'j_4_0_pos': -0.0264862, 'j_8_0_pos': -0.108088},
    'joints_pos_1':{'j_0_0_pos': 0.00979938, 'j_4_0_pos': -0.0101599, 'j_8_0_pos': -0.0979406},
    'joints_pos_2':{'j_0_0_pos': -0.00577911, 'j_4_0_pos': -0.030791, 'j_8_0_pos': -0.124544},
    'joints_pos_3':{'j_0_0_pos': -0.0100536, 'j_4_0_pos': -0.0129102, 'j_8_0_pos': -0.105622},
    'joints_pos_4':{'j_0_0_pos': -0.00820909, 'j_4_0_pos': -0.0122342, 'j_8_0_pos': -0.122107},
    'joints_pos_5':{'j_0_0_pos': -0.0057058, 'j_4_0_pos': -0.0247302, 'j_8_0_pos': -0.101658},
    'joints_pos_6':{'j_0_0_pos': -0.00602423, 'j_4_0_pos': -0.0124588, 'j_8_0_pos': -0.101188},
    'joints_pos_7':{'j_0_0_pos': -0.0134096, 'j_4_0_pos': -0.0282146, 'j_8_0_pos': -0.0965339},
    'joints_pos_8':{'j_0_0_pos': -0.00598818, 'j_4_0_pos': -0.01061, 'j_8_0_pos': -0.100572},
    'joints_pos_9':{'j_0_0_pos': -0.0111075, 'j_4_0_pos': -0.0273371, 'j_8_0_pos': -0.12039}}

    pos_dict['grasp_2'] = {'x': -0.10277238013017, 'y': -0.156779958555367, 'z': 1.1511266381958, # x was increased by 5.425265071939 mm, y was increased by 2.575139664674 mm
    'q0': 0.2605429, 'q1': 0.6556486, 'q2': -0.261117, 'q3': 0.6588325,
    'j_0': 53.45, 'j_1': -63.54, 'j_2': 109.82, 'j_3': -45.93, 'j_4': 13.36, 'j_5': -90.55,
    'joints_pos_0':{'j_0_0_pos': -0.00560981, 'j_4_0_pos': -0.0106044, 'j_8_0_pos': -0.088032},
    'joints_pos_1':{'j_0_0_pos': -0.00635791, 'j_4_0_pos': -0.0232379, 'j_8_0_pos': -0.102059},
    'joints_pos_2':{'j_0_0_pos': -0.00633008, 'j_4_0_pos': -0.0120065, 'j_8_0_pos': -0.118659},
    'joints_pos_3':{'j_0_0_pos': -0.00398488, 'j_4_0_pos': -0.0153128, 'j_8_0_pos': -0.0904356},
    'joints_pos_4':{'j_0_0_pos': -0.00518941, 'j_4_0_pos': -0.0128398, 'j_8_0_pos': -0.0892918},
    'joints_pos_5':{'j_0_0_pos': -0.00567146, 'j_4_0_pos': -0.0156297, 'j_8_0_pos': -0.116648},
    'joints_pos_6':{'j_0_0_pos': -0.00538539, 'j_4_0_pos': -0.019551, 'j_8_0_pos': -0.0870327},
    'joints_pos_7':{'j_0_0_pos': 0.0131945, 'j_4_0_pos': -0.0125819, 'j_8_0_pos': -0.0798454},
    'joints_pos_8':{'j_0_0_pos': 0.00945155, 'j_4_0_pos': -0.011036, 'j_8_0_pos': -0.0867611},
    'joints_pos_9':{'j_0_0_pos': -0.00117931, 'j_4_0_pos': -0.0114178, 'j_8_0_pos': -0.100712}}

    pos_dict['grasp_3'] = {'x': 0.022514723506118, 'y': -0.313087341866535, 'z': 1.1511266381958,       # x was decreased by 1.2 mm to reach contact with the palm, 
    'q0': -0.487874405059984, 'q1': 0.509695462981059, 'q2': 0.495372407035081, 'q3': 0.506749719538315,
    'j_0': 82.99, 'j_1': -30.46, 'j_2': 48.05, 'j_3': -17.36, 'j_4': -95.06, 'j_5': -90.47,
    'joints_pos_0':{'j_0_0_pos': -0.00843889, 'j_4_0_pos': -0.0184853, 'j_8_0_pos': -0.0924897},
    'joints_pos_1':{'j_0_0_pos': -0.005498, 'j_4_0_pos': -0.0116496, 'j_8_0_pos': -0.0915539},
    'joints_pos_2':{'j_0_0_pos': -0.0127131, 'j_4_0_pos': -0.0207798, 'j_8_0_pos': -0.0947811},
    'joints_pos_3':{'j_0_0_pos': -0.0162671, 'j_4_0_pos': -0.0123627, 'j_8_0_pos': -0.118811},
    'joints_pos_4':{'j_0_0_pos': -0.00647722, 'j_4_0_pos': -0.0261907, 'j_8_0_pos': -0.0867308},
    'joints_pos_5':{'j_0_0_pos': -0.00631889, 'j_4_0_pos': -0.0109754, 'j_8_0_pos': -0.0908911},
    'joints_pos_6':{'j_0_0_pos': -0.00310963, 'j_4_0_pos': -0.0183237, 'j_8_0_pos': -0.0904654},
    'joints_pos_7':{'j_0_0_pos': -0.0102885, 'j_4_0_pos': -0.0122753, 'j_8_0_pos': -0.0918848},
    'joints_pos_8':{'j_0_0_pos': -0.00945006, 'j_4_0_pos': -0.0118586, 'j_8_0_pos': -0.0911444},
    'joints_pos_9':{'j_0_0_pos': -0.00861539, 'j_4_0_pos': -0.0111022, 'j_8_0_pos': -0.118447}}

    pos_dict['grasp_4'] = {'x': -0.028074190223375257, 'y': -0.2903223679917239, 'z': 1.2435346668295724,  # z was decreased by 1.5 mm to avoid interference bet. the palm and the object, x was increased by 3 mm and y was increased by 4 mm
    'q0': -0.709134222436369, 'q1': 0.000903184888922603, 'q2': 0.01474255404848284, 'q3': 0.7049187867596882,
    'j_0': 78.21, 'j_1': -39.15, 'j_2': 46.85, 'j_3': -8.14, 'j_4': -104.81, 'j_5': -179.86,
    'joints_pos_0':{'j_0_0_pos': 0.00467271, 'j_4_0_pos': -0.0120985, 'j_8_0_pos': -0.0509781},
    'joints_pos_1':{'j_0_0_pos': 0.0045961, 'j_4_0_pos': -0.0121421, 'j_8_0_pos': -0.050826},
    'joints_pos_2':{'j_0_0_pos': 0.00463211, 'j_4_0_pos': -0.0123147, 'j_8_0_pos': -0.0507529},
    'joints_pos_3':{'j_0_0_pos': 0.00466091, 'j_4_0_pos': -0.012252, 'j_8_0_pos': -0.0507051},
    'joints_pos_4':{'j_0_0_pos': 0.00472339, 'j_4_0_pos': -0.0119996, 'j_8_0_pos': -0.0509498},
    'joints_pos_5':{'j_0_0_pos': 0.00472673, 'j_4_0_pos': -0.0123675, 'j_8_0_pos': -0.0507284},
    'joints_pos_6':{'j_0_0_pos': 0.00487648, 'j_4_0_pos': -0.00465743, 'j_8_0_pos': -0.0374336},
    'joints_pos_7':{'j_0_0_pos': 0.00484308, 'j_4_0_pos': -0.00412551, 'j_8_0_pos': -0.0386975},
    'joints_pos_8':{'j_0_0_pos': 0.00460293, 'j_4_0_pos': -0.012112, 'j_8_0_pos': -0.0510022},
    'joints_pos_9':{'j_0_0_pos': 0.00470364, 'j_4_0_pos': -0.0121958, 'j_8_0_pos': -0.0506643}}
    # Creating a set of callable services
    rospy.wait_for_service("/gazebo/pause_physics")
    sim_pauser = rospy.ServiceProxy("/gazebo/pause_physics", Empty) 

    rospy.wait_for_service("/gazebo/unpause_physics")
    sim_starter = rospy.ServiceProxy("/gazebo/unpause_physics", Empty)  

    rospy.wait_for_service("/gazebo/delete_model")
    model_deleter = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)       

    rospy.wait_for_service("/gazebo/spawn_sdf_model")
    spawner = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)

    # Introducing the ros saving node that will be needed to register the contact point cloud
    package = 'allegro_hand_taxels'
    executable = 'saving_node'
    saving_node = roslaunch.core.Node(package, executable)
    deletion_node = roslaunch.core.Node(package, 'deletion_node')
    control_mngr_node = roslaunch.core.Node('controller_manager','spawner',args="joint_state_controller pos_joint_traj_controller allegro_hand_right/joint_0_0_position_controller allegro_hand_right/joint_1_0_torque_controller allegro_hand_right/joint_2_0_torque_controller allegro_hand_right/joint_3_0_torque_controller allegro_hand_right/joint_4_0_position_controller allegro_hand_right/joint_5_0_torque_controller allegro_hand_right/joint_6_0_torque_controller allegro_hand_right/joint_7_0_torque_controller allegro_hand_right/joint_8_0_position_controller allegro_hand_right/joint_9_0_torque_controller allegro_hand_right/joint_10_0_torque_controller allegro_hand_right/joint_11_0_torque_controller allegro_hand_right/joint_12_0_position_controller allegro_hand_right/joint_13_0_position_controller allegro_hand_right/joint_14_0_torque_controller allegro_hand_right/joint_15_0_torque_controller")
    launch = roslaunch.scriptapi.ROSLaunch()

    # Create a publisher for each of the Allegro hand joints
    j_0_0_publisher = rospy.Publisher(
        "/allegro_hand_right/joint_0_0_position_controller/command",
        Float64,
        queue_size=20)
    j_1_0_publisher = rospy.Publisher(
        "/allegro_hand_right/joint_1_0_torque_controller/command",
        Float64,
        queue_size=20)  
    j_2_0_publisher = rospy.Publisher(
        "/allegro_hand_right/joint_2_0_torque_controller/command",
        Float64,
        queue_size=20) 
    j_3_0_publisher = rospy.Publisher(
        "/allegro_hand_right/joint_3_0_torque_controller/command",
        Float64,
        queue_size=20)  
    j_4_0_publisher = rospy.Publisher(
        "/allegro_hand_right/joint_4_0_position_controller/command",
        Float64,
        queue_size=20)  
    j_5_0_publisher = rospy.Publisher(
        "/allegro_hand_right/joint_5_0_torque_controller/command",
        Float64,
        queue_size=20) 
    j_6_0_publisher = rospy.Publisher(
        "/allegro_hand_right/joint_6_0_torque_controller/command",
        Float64,
        queue_size=20) 
    j_7_0_publisher = rospy.Publisher(
        "/allegro_hand_right/joint_7_0_torque_controller/command",
        Float64,
        queue_size=20) 
    j_8_0_publisher = rospy.Publisher(
        "/allegro_hand_right/joint_8_0_position_controller/command",
        Float64,
        queue_size=20) 
    j_9_0_publisher = rospy.Publisher(
        "/allegro_hand_right/joint_9_0_torque_controller/command",
        Float64,
        queue_size=20) 
    j_10_0_publisher = rospy.Publisher(
        "/allegro_hand_right/joint_10_0_torque_controller/command",
        Float64,
        queue_size=20) 
    j_11_0_publisher = rospy.Publisher(
        "/allegro_hand_right/joint_11_0_torque_controller/command",
        Float64,
        queue_size=20) 
    j_12_0_publisher = rospy.Publisher(
        "/allegro_hand_right/joint_12_0_position_controller/command",
        Float64,
        queue_size=20) 
    j_13_0_publisher = rospy.Publisher(
        "/allegro_hand_right/joint_13_0_position_controller/command",
        Float64,
        queue_size=20) 
    j_14_0_publisher = rospy.Publisher(
        "/allegro_hand_right/joint_14_0_torque_controller/command",
        Float64,
        queue_size=20) 
    j_15_0_publisher = rospy.Publisher(
        "/allegro_hand_right/joint_15_0_torque_controller/command",
        Float64,
        queue_size=20) 


#        input(
#            "============ Press `Enter` to set up the moveit_commander ..."
#        )
    tutorial = MoveGroupPythonInterfaceTutorial()

    print("Please enter the desired grasp attempt:(for example: grasp_0)")
    desired_grasp_attempt = input()

    #Setting the grasping attempt parameter
    rospy.set_param(param_name="grasp_attempt", param_value=desired_grasp_attempt)
    '''
    for v in range(100):
        print(
        "============ Please enter the desired iteration for "+desired_grasp_attempt+" (for example: 0)"
        )
        i = int(input())
    '''
    for i in range(10):
        #Setting the grasping iteration parameter
        rospy.set_param(param_name="iteration_nb", param_value=i)
        main(i, desired_grasp_attempt, pos_dict[desired_grasp_attempt]['x'], pos_dict[desired_grasp_attempt]['y'], pos_dict[desired_grasp_attempt]['z'],
                pos_dict[desired_grasp_attempt]['q0'], pos_dict[desired_grasp_attempt]['q1'], pos_dict[desired_grasp_attempt]['q2'],
                pos_dict[desired_grasp_attempt]['q3'], pos_dict[desired_grasp_attempt]['j_0'],
                pos_dict[desired_grasp_attempt]['j_1'], pos_dict[desired_grasp_attempt]['j_2'],
                pos_dict[desired_grasp_attempt]['j_3'], pos_dict[desired_grasp_attempt]['j_4'],
                pos_dict[desired_grasp_attempt]['j_5'], pos_dict[desired_grasp_attempt]['joints_pos_'+str(i)]['j_0_0_pos'],
            pos_dict[desired_grasp_attempt]['joints_pos_'+str(i)]['j_4_0_pos'], pos_dict[desired_grasp_attempt]['joints_pos_'+str(i)]['j_8_0_pos'])
