# BICI_SENSORS_ROS
This repository is for: 

1- the code needed to read the BICI sensors data and communicate them in ROS and other related software

2- the code needed to control the allegro hand

**P.S.** The proximal back and proximal front sensors of the thumb must be removed from the I2C network and the codes that use them (i.e. coordinates_listener.cpp) before programming and working with the pcbs to be soldered in the future.

**Allegro ROS Communication Package**

In order to download the setup the packages needed to communicate with the Allegro Hand, please execute the following shell scripts in order and follow the instructions as prompted:

1- Pcan_Downloads

2- Pcan_Bus_Setup

**P.S.** In order to read data from the tactile sensors, the USB serial communication must be setup by executing the following shell script (from its directory): ./serial_USB_Setup

**P.S.** After building everything in ROS_COMM_WS using catkin, the following command will initiate the torque control of the Allegro hand, as well as contact visualization/recording: roslaunch allegro_hand_controllers allegro_hand.launch HAND:=right CONTROLLER:=torque VISUALIZE:=true

**P.S.** In order to torque control the joints of the Allegro hand, use the following command: rostopic pub -1 /allegroHand_0/torque_cmd sensor_msgs/JointState "{effort: [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 ,0 ,0 ,0]}"

where each number correspond to a joint as per the instructions provided in the "Allegro_Torque_Control_Guide.pdf" file in the ROS_WS directory of the repository.

**P.S.** All the models used in Gazebo simulation (along with their meshes) are stored in the "models" folder under the "Gazebo_Simulation" directory. Before running the simulation make sure to move this folder into "~/.gazebo".

**P.S.** When building the ROS package (for the first time), "bici_ros_sensor_reader" package must be built first using the command: catkin_make --only-pkg-with-deps <target_package> , then switching back to building all the packages is needed using this command: catkin_make -DCATKIN_WHITELIST_PACKAGES=""

**P.S.** The coordinates_listener node should be run separately from the allegro_hand.launch file (should be working toward including it in the launch file) and from the ROW_WS directory.

**Robot Arm Workspace Building Notes**

    **P.S.** Before building any package, run the following two commands:

       - rosdep update

       - rosdep install --rosdistro noetic --ignore-src --from-paths src --os=ubuntu:focal

    **P.S.** To clone a branch, you have to use the following command:
        git clone -b "branch-name" --single-branch "repo-link"

    1- In case of a PC format, before building robotiq package, run the following command:
        sudo apt install ros-noetic-soem

    2- When building coro_workstations package, clone the robotiq_85_gripper separately inside the coro_workstations package (this is because  robotiq_85_gripper is used as a submodule inside the coro_workstations repo) 

**UR5-e Teaching Pendant Setup + Extra Notes (for hardware control via Moveit)**

    1- Upper right corner three-dashes button --> settings --> System --> URCaps --> Press the "+" button to add: "External Control", "rs485", "Robotiq_Grippers" and "Robotiq_Wrist_Camera" from usbdisk

    2- Go to the Run tab (in the upper toolbar) and load ROS.urp (by pressing the "Load Program" button, you can find the .urp file directly in the home directory)

    3- In case of a PC format, before running the hardware-related launch file, run the following command: sudo apt install ros-noetic-moveit-resources-prbt-moveit-config

**P.S.** In case of a format, how to setup your PC for ROS development:

1- In VScode: Use ctrl + shift + p to open the command palette and search there for "c_cpp_properties.json" then open the file and add the highlighted line of code as it appears in the "VScode_ROS_Setup" image under ROS_WS directory.
2- Install all the needed gazebo_ros packages (this is particularly needed for building the contact sensor plugin in QtCreator)
