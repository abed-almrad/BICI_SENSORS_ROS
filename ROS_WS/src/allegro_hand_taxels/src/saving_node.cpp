#include "ros/ros.h"
#include "std_msgs/String.h"

int main(int argc, char **argv)
{

  ros::init(argc, argv, "saving_commander");
  ros::NodeHandle nh;
  std_msgs::String msg;
  ros::Publisher pub = nh.advertise<std_msgs::String>("saving_command", 1000);
  ros::Rate rate(10);
  while (ros::ok())
  {
    msg.data = "true";
    ROS_INFO("%s", msg.data.c_str());
    pub.publish(msg);
    ros::spinOnce();
    rate.sleep();
  }


  return 0;
}
// %EndTag(FULLTEXT)%