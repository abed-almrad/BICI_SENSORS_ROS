#ifndef _GAZEBO_COORDINATES_PLUGIN_HH_
#define _GAZEBO_COORDINATES_PLUGIN_HH_


#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/sensors/sensors.hh>
#include <boost/algorithm/string/replace.hpp>
#include <string>
#include <iostream>
#include <vector>
#include <map>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdio.h>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/subscribe_options.h>
#include <std_msgs/String.h>
#define contact_pts_nb 1300
typedef const boost::shared_ptr< const gazebo::msgs::Contacts> ContactPtr;
namespace gazebo
{
  /// \brief An example plugin for a contact sensor.
  class CoordinatesPlugin : public ModelPlugin
  {
    /// \brief Constructor.
    public: CoordinatesPlugin();

    /// \brief Destructor.
    public: ~CoordinatesPlugin();

    /// \brief Load the sensor plugin.
    /// \param[in] _sensor Pointer to the sensor that loaded this plugin.
    /// \param[in] _sdf SDF element that describes the plugin.
    public: virtual void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/);

    /// \brief Callback that receives the contact sensor's update signal.
    public: virtual void OnUpdate();

    //Custom method to split a string
    //public: virtual void tokenize(std::string const &str, std::string const delim,
    //              std::vector<std::string> &out);

    public: virtual void contact_callback(ContactPtr &_msg);
    public: virtual void saving_callback(const std_msgs::StringConstPtr &_msg);

    private: void QueueThread();
    //Pointer to the model
    private: physics::ModelPtr model;
    // Variable for the contact location
    private: ignition::math::Pose3d contact_pose;
    /// \brief Connection that maintains a link between the contact sensor's
    /// updated signal and the OnUpdate callback.
    private: event::ConnectionPtr updateConnection;
    /// \brief A node used for transport
    private: transport::NodePtr node;

    /// \brief A subscriber to a named topic.
    private: std::vector<transport::SubscriberPtr> subscribers_v;
    private: std::vector<std::shared_ptr<sensors::Sensor>> sensor_v;
    private: gazebo::sensors::SensorManager *sensorManager;
    private: std::map <std::string, ignition::math::Pose3d> contacts_map;
    private: FILE *fp;
    private: std::string act_cmd = "false";

    //ROS subscriber

    /// \brief A node use for ROS transport
    private: std::unique_ptr<ros::NodeHandle> rosNode;

    /// \brief A ROS subscriber
    private: ros::Subscriber saving_subscriber;

    /// \brief A ROS callbackqueue that helps process messages
    private: ros::CallbackQueue rosQueue;

    /// \brief A thread the keeps running the rosQueue
    private: std::thread rosQueueThread;
  };

}
#endif
