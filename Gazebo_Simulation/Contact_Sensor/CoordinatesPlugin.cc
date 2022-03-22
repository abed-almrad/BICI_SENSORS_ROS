#include "CoordinatesPlugin.hh"


using namespace gazebo;
// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(CoordinatesPlugin)

CoordinatesPlugin::CoordinatesPlugin() : ModelPlugin ()
{
}
CoordinatesPlugin::~CoordinatesPlugin()
{
}

//Custom method to split a string
/*void CoordinatesPlugin::tokenize(std::string const &str, std::string const delim,
            std::vector<std::string> &out)
{
    size_t start;
    size_t end = 0;
    size_t delim_len = delim.length();
    size_t counter = 0;
    while (((start = str.find_first_not_of(delim, end)) != std::string::npos) && counter!=3)
    {
        end = str.find(delim, start);
        //std::cout<<str.substr(start, end - start)<<std::endl;
        out.push_back(str.substr(start, end - start));

        end += delim_len;
        counter++;
    }
}
*/

// Called by the world update start event
void CoordinatesPlugin::OnUpdate()
{

}

/// \brief ROS helper function that processes messages
void CoordinatesPlugin::QueueThread()
{
  static const double timeout = 0.01;
  while (this->rosNode->ok())
  {
    this->rosQueue.callAvailable(ros::WallDuration(timeout));
  }
}
// ROS saving subscriber callback function
void CoordinatesPlugin::saving_callback(const std_msgs::StringConstPtr &_msg) //
{
    this->act_cmd = _msg->data;
}

// Gazebo contact detection subscriber callback
void CoordinatesPlugin::contact_callback(ContactPtr &_msg) //
{

    if (_msg->contact_size() > 0)
    {



        std::string collision_entity = _msg->contact(0).collision1();
        std::string delim = "::";
        size_t location = collision_entity.find_last_of(delim);
        size_t delim_len = delim.length();
        //Elements of interest
        collision_entity = collision_entity.substr(location+delim_len-1);
        this->contact_pose = this->model->GetChildCollision(collision_entity)->WorldPose();

        this->contacts_map[collision_entity] = this->contact_pose;
        std::cout<<"Contact Element: "<<collision_entity<<" , Contact Position: "<<this->contact_pose<<std::endl;
    }
    if(this->act_cmd == "true")
    {
        //Flushing the contact locations into a .csv file
        for (auto const& [key, val] : this->contacts_map)
        {

            if(fp == NULL) {
                printf("file can't be opened\n");
                exit(1);
            }
        fflush(stdin);
        fprintf(fp, "%f,%f,%f \n", val.X(),val.Y(),val.Z());
        }

    fclose(fp);
    this->act_cmd = "false";
    ros::shutdown();
    }
            /*
            std::cout<< val.X()<< std::endl;
            this->myfile <<  val.X() << ',';
            std::cout<< val.Y()<< std::endl;
            this->myfile << val.Y() << ',';
            std::cout<< val.Z()<< std::endl;
            this->myfile << val.Z();
            this->myfile << '\n';
         }
         this->myfile.close();
         this->act_cmd = "false";
            */

}

void CoordinatesPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
      // Store the pointer to the model
      this->model = _parent;
      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&CoordinatesPlugin::OnUpdate, this));
      //Gazebo transport node creation and initialization
      this->node = transport::NodePtr(new transport::Node());
      this->node->Init(_parent->GetName());
      this->sensorManager = sensors::SensorManager::Instance();
      // The total number of attached contact sensors
      std::cout << "The number of attached sensors is: " + std::to_string(_parent->GetSensorCount()) << std::endl;
      //Create a subscriber for each contact sensor that was placed on each taxel and register a common callback
      for (int index = 0; index < _parent->GetSensorCount(); index++) {
          //Vectors sizing to avoid memory violation
          this->sensor_v.resize(_parent->GetSensorCount());
          this->subscribers_v.resize(_parent->GetSensorCount());

          this->sensor_v = this->sensorManager->GetSensors();
          std::string topic_name = sensor_v[index]->ScopedName();
          boost::replace_all(topic_name, "::", "/");
          std::cout << "/gazebo/"+topic_name << std::endl;
          this->subscribers_v[index] = this->node->Subscribe("/gazebo/"+topic_name,&CoordinatesPlugin::contact_callback,this);
          //this->saving_subscriber = this->node->Subscribe("saving_order",&CoordinatesPlugin::saving_callback,this);
          //Opening the .csv file for contact points registration
          std::string full_path = std::filesystem::current_path();
          size_t pos = full_path.find("Gazebo_Simulation");
          std::string path = full_path.substr(0,pos+17)+"/Pt_Cloud_Scripts/points_coordinates.csv";
          this->fp = fopen(path.c_str(), "w");

          //ROS subscriber
          // Initialize ros, if it has not already bee initialized.
          if (!ros::isInitialized())
          {
            int argc = 0;
            char **argv = NULL;
            ros::init(argc, argv, "gazebo_client",
                ros::init_options::NoSigintHandler);
          }

          // Create our ROS node. This acts in a similar manner to
          // the Gazebo node
          this->rosNode.reset(new ros::NodeHandle("gazebo_client"));

          // Create a named topic, and subscribe to it.
          ros::SubscribeOptions so =
            ros::SubscribeOptions::create<std_msgs::String>(
                      "/saving_order",
                      5,
                      boost::bind(&CoordinatesPlugin::saving_callback, this, _1),
                      ros::VoidPtr(),
                      &this->rosQueue);
          this->saving_subscriber = this->rosNode->subscribe(so);

          // Spin up the queue helper thread.
          this->rosQueueThread =
            std::thread(std::bind(&CoordinatesPlugin::QueueThread, this));
          this->rosQueueThread.detach();


      }


    }




//*****************************************************Test******************************************




