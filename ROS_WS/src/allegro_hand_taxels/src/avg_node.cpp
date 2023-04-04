#include "ros/ros.h"
#include "allegro_hand_taxels/taxels_data_avg.h"

int main(int argc, char **argv)
{

  ros::init(argc, argv, "avg_client");
  ros::NodeHandle nh;
  ros::ServiceClient avg_client = nh.serviceClient<allegro_hand_taxels::taxels_data_avg>("avg_command");
  allegro_hand_taxels::taxels_data_avg srv;
  srv.request.avg_activation = "true";

  if (avg_client.call(srv))
  {
    std::cout << "Taxels Averaging Response: " << srv.response.avg_activation_response << std::endl;
    srv.request.avg_activation.clear();
    srv.response.avg_activation_response.clear();
  }
  else
  {
    std::cout << "Failed to call service avg_command" << std::endl;
    return 1;
  }

  return 0;
}
