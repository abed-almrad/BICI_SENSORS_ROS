#include "ros/ros.h"
#include "allegro_hand_taxels/coordinates_saving.h"

int main(int argc, char **argv)
{

  int i;
  ros::init(argc, argv, "saving_client");
  ros::NodeHandle nh;
  ros::ServiceClient saving_client = nh.serviceClient<allegro_hand_taxels::coordinates_saving>("saving_command");
//  ros::ServiceClient deletion_client = nh.serviceClient<allegro_hand_taxels::coordinates_saving>("deletion_command");
  allegro_hand_taxels::coordinates_saving srv;
  srv.request.saving_activation = "true";
  if(nh.getParam("iteration_nb",i))
  {
    srv.request.iteration_nb = i;

    if (saving_client.call(srv))
    {
      std::cout << "Coordinates Saving Response: " << srv.response.saving_activation_response << std::endl;
      srv.request.saving_activation.clear();
      srv.response.saving_activation_response.clear();
    }
    else
    {
      std::cout << "Failed to call service saving_command" << std::endl;
      return 1;
    }
/*
    if (deletion_client.call(srv))
    {
      std::cout << "Model deletion Response: " << srv.response.saving_activation_response << std::endl;
      srv.request.saving_activation.clear();
      srv.response.saving_activation_response.clear();
    }
    else
    {
      std::cout << "Failed to call service deletion_command" << std::endl;
      return 1;
    }
*/
    return 0;
  }

}
// %EndTag(FULLTEXT)%