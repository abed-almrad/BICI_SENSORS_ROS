#!/usr/bin/env python3

# Python 2/3 compatibility imports
from __future__ import print_function
from six.moves import input

import sys
import copy
import rospy
import roslaunch
import moveit_commander
import geometry_msgs.msg
import rosnode

from math import pi, tau, dist, fabs, cos
    
from sensor_msgs.msg import JointState
from moveit_commander.conversions import pose_to_list

grasps_list = [ "grasp_"+str(count) for count in range(10)]

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
        rospy.init_node("move_group_python_interface_hardware", anonymous=True)

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

    def go_to_joint_state(self, j_0, j_1, j_2, j_3, j_4, j_5): #Joints positions are specified in degrees
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




def main(desired_grasp_attempt, i, x, y, z, q0, q1, q2, q3, j_0, j_1, j_2, j_3, j_4, j_5):
#    joint_effort_publisher.publish(position=[0.05, 0, 0, 0, 0.05, 0, 0, 0, -0.05, 0, 0, 0, 0 ,0 ,0 ,0], 
#                                   effort=[0, 0, 0, -0, 0, 0, 0, -0, 0, 0, 0, -0, 0 ,0 ,0 ,0])


    try:
        print("This is grasp: grasp_", desired_grasp_attempt)
        print("Iteration: ", i)


        input(
            "============ Press `Enter` to cancel any applied torque ..."
        )

        joint_effort_publisher.publish(effort=[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 ,0 ,0 ,0])

        input(
            "============ Press `Enter` to set the pd controlled joints to initial position ..."
        )
        joint_position_publisher.publish(position=[0.05, 0, 0, 0, 0.05, 0, 0, 0, -0.05, 0, 0, 0, 0 ,0 ,0 ,0])

#        rospy.sleep(1.0)
        '''
        input(
            "============ Press `Enter` to set the torque controlled joints base to initial position ..."
        )

        joint_effort_publisher.publish(position=[0.05, 0, 0, 0, 0.05, 0, 0, 0, -0.05, 0, 0, 0, 0 ,0 ,0 ,0],
#                                       effort=[0, 0, 0, -0.15, 0, 0, 0, 0, 0, 0, 0, 0, 0 ,0 ,0 ,0]) 
                                       effort=[0, 0, 0, -0.15, 0, 0, 0, -0.15, 0, 0, 0, -0.15, 0 ,0 ,0 ,0])

        rospy.sleep(0.5)

        joint_effort_publisher.publish(position=[0.05, 0, 0, 0, 0.05, 0, 0, 0, -0.05, 0, 0, 0, 0 ,0 ,0 ,0], 
#                                       effort=[0, 0, 0, -0.2, 0, 0, 0, 0, 0, 0, 0, 0, 0 ,0 ,0 ,0])
                                       effort=[0, 0, 0, -0.2, 0, 0, 0, -0.2, 0, 0, 0, -0.2, 0 ,0 ,0 ,0])

#        rospy.sleep(1.0)

        input(
            "============ Press `Enter` to set the torque controlled joints middle to initial position ..."
        )

        joint_effort_publisher.publish(position=[0.05, 0, 0, 0, 0.05, 0, 0, 0, -0.05, 0, 0, 0, 0 ,0 ,0 ,0], 
#                                       effort=[0, 0, -0.15, -0.15, 0, 0, 0, 0, 0, 0, 0, 0, 0 ,0 ,0 ,0])
                                       effort=[0, 0, -0.15, -0.15, 0, 0, -0.15, -0.15, 0, 0, -0.15, -0.15, 0 ,0 ,0 ,0])

        rospy.sleep(0.5)

        joint_effort_publisher.publish(position=[0.05, 0, 0, 0, 0.05, 0, 0, 0, -0.05, 0, 0, 0, 0 ,0 ,0 ,0], 
#                                       effort=[0, 0, -0.2, -0.2, 0, 0, 0, 0, 0, 0, 0, 0, 0 ,0 ,0 ,0])
                                       effort=[0, 0, -0.2, -0.2, 0, 0, -0.2, -0.2, 0, 0, -0.2, -0.2, 0 ,0 ,0 ,0])

#        rospy.sleep(1.0)

        input(
            "============ Press `Enter` to set the torque controlled joints periferies to initial position ..."
        )

        joint_effort_publisher.publish(position=[0.05, 0, 0, 0, 0.05, 0, 0, 0, -0.05, 0, 0, 0, 0 ,0 ,0 ,0],
#                                       effort=[0, -0.15, -0.15, -0.15, 0, 0, 0, 0, 0, 0, 0, 0, 0 ,0 ,0 ,0])
                                       effort=[0, -0.15, -0.15, -0.15, 0, -0.15, -0.15, -0.15, 0, -0.15, -0.15, -0.15, 0 ,0 ,0 ,0])

        rospy.sleep(0.5)

        joint_effort_publisher.publish(position=[0.05, 0, 0, 0, 0.05, 0, 0, 0, -0.05, 0, 0, 0, 0 ,0 ,0 ,0],
#                                       effort=[0, -0.2, -0.2, -0.2, 0, 0, 0, 0, 0, 0, 0, 0, 0 ,0 ,0 ,0])
                                       effort=[0, -0.2, -0.2, -0.2, 0, -0.2, -0.2, -0.2, 0, -0.2, -0.2, -0.2, 0 ,0 ,0 ,0])
        '''
        '''
# Setting the robotic arm to a favorable initial position
        input(
        "============ Press `Enter` to set the arm to a favorable initial position ..."
        )

        tutorial.go_to_joint_state(j_0, j_1, j_2, j_3, j_4, j_5)
        '''
# Activation of the taxels data averaging node
        input(
            "============ Press `Enter` to average the taxels data..."
        )
        launch.start()
        process = launch.launch(avg_node)
        print("The averaging node is activated: ", process.is_alive())
        '''
# Setting the end effector to the desired position for grasp_0
        input(
        "============ Press `Enter` to set the arm to the desired end effector pose for grasp 0 ..."
        )
        tutorial.go_to_pose_goal(x, y, z, q0, q1, q2, q3)
        '''

#CLOSE THE ALLEGRO HAND MANUALLY AGAINST THE OBJECT!!!!!!!!!!!!!!!!!!!!
        '''
        input(
            "============ PLEASE CLOSE THE ALLEGRO HAND MANUALLY BEFORE PROCEEDING!!!!!!!!!!!!!!! ..."
        )
# Allegro hand closure

        input(
            "============ Press `Enter` to close the torque controlled joints base ..."
        )

        joint_effort_publisher.publish(position=[0.05, 0, 0, 0, 0.05, 0, 0, 0, -0.05, 0, 0, 0, 0 ,0 ,0 ,0],
#                                       effort=[0, 0.1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 ,0 ,0 ,0])
                                        effort=[0, 0.1, 0, 0, 0, 0.1, 0, 0, 0, 0.1, 0, 0, 0 ,0 ,0 ,0])

        rospy.sleep(0.5)

        joint_effort_publisher.publish(position=[0.05, 0, 0, 0, 0.05, 0, 0, 0, -0.05, 0, 0, 0, 0 ,0 ,0 ,0],
#                                       effort=[0, 0.15, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 ,0 ,0 ,0])
                                       effort=[0, 0.15, 0, 0, 0, 0.15, 0, 0, 0, 0.15, 0, 0, 0 ,0 ,0 ,0])

        rospy.sleep(1.0)
        input(
            "============ Press `Enter` to close the torque controlled joints middle ..."
        )

        joint_effort_publisher.publish(position=[0.05, 0, 0, 0, 0.05, 0, 0, 0, -0.05, 0, 0, 0, 0 ,0 ,0 ,0],
#                                       effort=[0, 0.52, 0.52, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 ,0 ,0 ,0])
                                       effort=[0, 0.52, 0.52, 0, 0, 0.52, 0.52, 0, 0, 0.52, 0.52, 0, 0 ,0 ,0 ,0])

        rospy.sleep(1.0)

        input(
            "============ Press `Enter` to close the torque controlled joints periferies ..."
        )

        joint_effort_publisher.publish(position=[0.05, 0, 0, 0, 0.05, 0, 0, 0, -0.05, 0, 0, 0, 0 ,0 ,0 ,0],
#                                       effort=[0, 0.52, 0.52, 0.52, 0, 0, 0, 0, 0, 0, 0, 0, 0 ,0 ,0 ,0])
                                       effort=[0, 0.52, 0.52, 0.52, 0, 0.52, 0.52, 0.52, 0, 0.52, 0.52, 0.52, 0 ,0 ,0 ,0])

        rospy.sleep(0.5)

        input(
            "============ Press `Enter` to register the contact point cloud..."
        )
        launch.start()
        process = launch.launch(saving_node)
        print("The saving node is activated: ", process.is_alive())
        
        rospy.sleep(0.5)
        '''
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

    #Creating a dictionary to store the desired end effector poses for each grasp
    pos_dict =	{}
    pos_dict['grasp_0'] = {'x': -0.038420421352635, 'y': -0.10824651294124, 'z': 1.15484120989399,
    'q0': 0.494972210129084, 'q1': 0.517663199844739, 'q2': -0.498560916781039, 'q3': 0.488328101776754,
    'j_0': 40.47, 'j_1': -71.98, 'j_2': 122.41, 'j_3': -48.04, 'j_4': 39.76, 'j_5': -93.75}

    pos_dict['grasp_1'] = {'x': -0.106274277885961, 'y': -0.252325374003947, 'z': 1.15041613382691,
    'q0': -0.006514150104458, 'q1': 0.696756965530051, 'q2': 0.006723710381732, 'q3': 0.717246184062679,
    'j_0': 64.84, 'j_1': -39.99, 'j_2': 61.42, 'j_3': -17.67, 'j_4': -26.27, 'j_5': -93.37}


    # Introducing the ros saving and averaging nodes that will be needed to register the contact point cloud and average the taxels data respectively
    package = 'allegro_hand_taxels'
    saving_executable = 'saving_node'
    avg_executable = 'avg_node'
    saving_node = roslaunch.core.Node(package, saving_executable)
    avg_node = roslaunch.core.Node(package, avg_executable)
    launch = roslaunch.scriptapi.ROSLaunch()

    # Create a position publisher for the pd controlled Allegro hand joints
    joint_position_publisher = rospy.Publisher(
        "/allegroHand_0/joint_cmd",
        JointState,
        queue_size=20)
    # Create an effort publisher for the torque controlled Allegro hand joints
    joint_effort_publisher = rospy.Publisher(
        "/allegroHand_0/joint_cmd",
        JointState,
        queue_size=20)

#        input(
#            "============ Press `Enter` to set up the moveit_commander ..."
#        )
    tutorial = MoveGroupPythonInterfaceTutorial()

    print("Please enter the desired grasp attempt:(for example: 0)")
    desired_grasp_attempt = input()
    desired_grasp_attempt = int(desired_grasp_attempt)
    input(
    "============ Press `Enter` to start grasping iterations for grasp_"+str(desired_grasp_attempt)+"..."
    )
    #Setting the grasping attempt parameter
    rospy.set_param(param_name="grasp_attempt", param_value=desired_grasp_attempt)
    for i in range(10):
        #Setting the grasping iteration parameter
        rospy.set_param(param_name="iteration_nb", param_value=i)
        main(desired_grasp_attempt,i, pos_dict['grasp_'+str(desired_grasp_attempt)]['x'], pos_dict['grasp_'+str(desired_grasp_attempt)]['y'], pos_dict['grasp_'+str(desired_grasp_attempt)]['z'],
             pos_dict['grasp_'+str(desired_grasp_attempt)]['q0'], pos_dict['grasp_'+str(desired_grasp_attempt)]['q1'], pos_dict['grasp_'+str(desired_grasp_attempt)]['q2'],
             pos_dict['grasp_'+str(desired_grasp_attempt)]['q3'], pos_dict['grasp_'+str(desired_grasp_attempt)]['j_0'],
             pos_dict['grasp_'+str(desired_grasp_attempt)]['j_1'], pos_dict['grasp_'+str(desired_grasp_attempt)]['j_2'],
             pos_dict['grasp_'+str(desired_grasp_attempt)]['j_3'], pos_dict['grasp_'+str(desired_grasp_attempt)]['j_4'],
             pos_dict['grasp_'+str(desired_grasp_attempt)]['j_5'])