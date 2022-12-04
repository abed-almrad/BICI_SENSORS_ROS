#include "ros/ros.h"
#include "allegro_hand_taxels/model_deletion.h"

int main(int argc, char **argv)
{

  ros::init(argc, argv, "deletion_client");
  ros::NodeHandle nh;
  ros::ServiceClient deletion_client = nh.serviceClient<allegro_hand_taxels::model_deletion>("deletion_command");
  allegro_hand_taxels::model_deletion srv;
  srv.request.deletion_cmd = "true";

    if (deletion_client.call(srv))
    {
      std::cout << "Model deletion Response: " << srv.response.deletion_cmd_response << std::endl;
      srv.request.deletion_cmd.clear();
      srv.response.deletion_cmd_response.clear();
    }
    else
    {
      std::cout << "Failed to call service deletion_command" << std::endl;
      return 1;
    }

    return 0;
}


// %EndTag(FULLTEXT)%