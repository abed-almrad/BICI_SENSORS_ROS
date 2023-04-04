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




def main(i, x, y, z, q0, q1, q2, q3, j_0, j_1, j_2, j_3, j_4, j_5, j_0_0_pos, j_4_0_pos, j_8_0_pos):

    try:

#        rospy.sleep(1.0)
        input(
            "============ Press `Enter` to set the robot at a favorable initial position ..."
        )
        j_12_0_publisher.publish(data=0)

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

        tutorial.go_to_joint_state(j_0, j_1, j_2, j_3, j_4, j_5)


        input(
            "============ Press `Enter` to approach the object from the first angle ..."
        )

        tutorial.go_to_pose_goal(x, y, z, q0, q1, q2, q3)


#        print("============ Approach "+str(i)+" is complete!")

        input(
            "============ Press `Enter` to close the allegro hand fingers against the object..."
        )
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

        input(
            "============ Press `Enter` to register the contact point cloud..."
        )
        launch.start()
        process = launch.launch(saving_node)
        print("The saving node is activated: ", process.is_alive())

        rospy.sleep(1.0)

        input(
            "============ Press `Enter` to delete the ur5 workstation..."
        )
        process = launch.launch(deletion_node)
        print("The deletion node is activated: ", process.is_alive())
        rospy.sleep(1.0)

        input(
            "============ Press `Enter` to spawn a new workstation at initial position..."
        )
        spawner("ur5e_workstation_0", open("/home/abed/Documents/BICI_Project/BICI_SENSORS_ROS/ROS_WS/robot_arm_ws/src/coro_workstations/coro_descriptions/sdf/workstations/ur5e_workstation.sdf",'r').read(),"/gazebo/", Pose(position= Point(0,0,0),orientation=Quaternion(0,0,0,1)),"world")

#        input(
#            "============ Press `Enter` to set the robot_description parameter..."
#        )   
#        rospy.set_param(param_name="robot_description", param_value= open("/home/abed/Documents/BICI_Project/BICI_SENSORS_ROS/ROS_WS/robot_arm_ws/src/coro_workstations/coro_descriptions/urdf/workstations/ur5e_workstation.urdf",'r').read())

#        rospy.sleep(1.0)

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



#        input(
#            "============ Press `Enter` to start another iteration..."
#        )
#        input(
#            "============ Press `Enter` to pause the simulation..."
#        )

#        sim_pauser()

#        input(
#            "============ Press `Enter` to unpause the simulation..."
#        )
#        sim_starter()
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
    pos_dict['grasp_0'] = {'x': -0.038420421352635, 'y': -0.10824651294124, 'z': 1.15484120989399,
    'q0': 0.494972210129084, 'q1': 0.517663199844739, 'q2': -0.498560916781039, 'q3': 0.488328101776754,
    'j_0': 40.47, 'j_1': -71.98, 'j_2': 122.41, 'j_3': -48.04, 'j_4': 39.76, 'j_5': -93.75,
    'joints_pos_0':{'j_0_0_pos': -0.110635, 'j_4_0_pos': -0.109172, 'j_8_0_pos': -0.221889},
    'joints_pos_1':{'j_0_0_pos': -0.114924, 'j_4_0_pos': -0.158254, 'j_8_0_pos': -0.189427},
    'joints_pos_2':{'j_0_0_pos': -0.0531916, 'j_4_0_pos': -0.0884681, 'j_8_0_pos': -0.0932488},
    'joints_pos_3':{'j_0_0_pos': -0.00633685, 'j_4_0_pos': 0.00501784, 'j_8_0_pos': -0.0947491},
    'joints_pos_4':{'j_0_0_pos': 0.00783013, 'j_4_0_pos': -0.0807499, 'j_8_0_pos': -0.138711},
    'joints_pos_5':{'j_0_0_pos': 0.00882769, 'j_4_0_pos': -0.0677357, 'j_8_0_pos': -0.0904451},
    'joints_pos_6':{'j_0_0_pos': -0.0550931, 'j_4_0_pos': -0.0158625, 'j_8_0_pos': -0.0959136},
    'joints_pos_7':{'j_0_0_pos': -0.00790226, 'j_4_0_pos': -0.0492159, 'j_8_0_pos': -0.0837464},
    'joints_pos_8':{'j_0_0_pos': 0.0379571, 'j_4_0_pos': 0.00768438, 'j_8_0_pos': -0.0920475},
    'joints_pos_9':{'j_0_0_pos': 0.0352053, 'j_4_0_pos': 0.000728325, 'j_8_0_pos': -0.0702119},}
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
    input(
    "============ Press `Enter` to start grasping iterations for "+desired_grasp_attempt+"..."
    )
    #Setting the grasping attempt parameter
    rospy.set_param(param_name="grasp_attempt", param_value=desired_grasp_attempt)
    for i in range(10):
        #Setting the grasping iteration parameter
        rospy.set_param(param_name="iteration_nb", param_value=i)
        main(i, pos_dict[desired_grasp_attempt]['x'], pos_dict[desired_grasp_attempt]['y'], pos_dict[desired_grasp_attempt]['z'],
             pos_dict[desired_grasp_attempt]['q0'], pos_dict[desired_grasp_attempt]['q1'], pos_dict[desired_grasp_attempt]['q2'],
             pos_dict[desired_grasp_attempt]['q3'], pos_dict[desired_grasp_attempt]['j_0'],
             pos_dict[desired_grasp_attempt]['j_1'], pos_dict[desired_grasp_attempt]['j_2'],
             pos_dict[desired_grasp_attempt]['j_3'], pos_dict[desired_grasp_attempt]['j_4'],
             pos_dict[desired_grasp_attempt]['j_5'], pos_dict[desired_grasp_attempt]['joints_pos_'+str(i)]['j_0_0_pos'],
            pos_dict[desired_grasp_attempt]['joints_pos_'+str(i)]['j_4_0_pos'], pos_dict[desired_grasp_attempt]['joints_pos_'+str(i)]['j_8_0_pos'])
