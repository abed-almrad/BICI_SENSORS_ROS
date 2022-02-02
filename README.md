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

**P.S.** When building the ROS package (for the first time), "bici_ros_sensor_reader" package must be built first using the command: catkin_make --only-pkg-with-deps <target_package> , then switching back to building all the packages is needed using this command: catkin_make -DCATKIN_WHITELIST_PACKAGES=""

**P.S.** The coordinates_listener node should be run separately from the allegro_hand.launch file (should be working toward including it in the launch file)
