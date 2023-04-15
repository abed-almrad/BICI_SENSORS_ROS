// P.S. All lines code where mft and pft were mentioned, were commented out to purposefully
//ignore the middle and pinky fingertips
// sensors since they are currently giving unstable data streams

#ifndef _GAZEBO_COORDINATES_PLUGIN_HH_
#define _GAZEBO_COORDINATES_PLUGIN_HH_


#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Pose3.hh>
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
#include <stdio.h>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/subscribe_options.h>
#include <std_msgs/String.h>
#include <allegro_hand_taxels/coordinates_saving.h>
#include <utility>
#include <vector>
#include <algorithm>
#include <math.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <eigen3/Eigen/Dense>

#define JOINTS_COUNT 22
#define contact_pts_nb 1300
//#define dist_threshold 100
//#define dist_threshold 0.00212 // threshold distance between the contact point and the taxel in m
#define dist_threshold 0.00234 // threshold distance between the contact point and the taxel in m
typedef const boost::shared_ptr< const gazebo::msgs::Contacts> ContactPtr;
tf2_ros::Buffer tfBuffer;
tf2_ros::TransformListener *tfListener;

namespace gazebo
{
  /// \brief An example plugin for a contact sensor.
  class CoordinatesPlugin : public ModelPlugin
  {
    /// \brief Constructor.
    public: CoordinatesPlugin();

    /// \brief Destructor.
    public: ~CoordinatesPlugin();
    //Method for extracting the homogeneous matrix from the tf2 transformation
    public: virtual std::vector<std::vector<float>> homo_matrix(geometry_msgs::TransformStamped transformStamped);
    //Method for converting a 3D vector into its homogeneous representation
    public: virtual std::vector<float> homo_v(geometry_msgs::Vector3Stamped v3stamped);
    //Method for matrix multiplication
    public: virtual std::vector<float> homotrans(geometry_msgs::TransformStamped transformStamped,geometry_msgs::Vector3Stamped v3stamped);
    //Method to get the name of the sensor from the correspondent collission name
    public: virtual std::string getName(std::string str);
    //Method to calculate the Eucledian distance between two points in the 3D space
    public: virtual float dist(std::vector<float> P1, std::vector<float> P2);
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
    public: virtual bool saving_activation_function(allegro_hand_taxels::coordinates_saving::Request &req,
                                                    allegro_hand_taxels::coordinates_saving::Response &res);

    // A world variable
    private: physics::World world;
    //Contact info
    private: std::map <std::string, std::vector<std::vector<float>>> raw_contacts_map;
    private: std::map <std::string, std::vector<std::vector<float>>> filtered_contacts_map;
    private: std::map <std::string, std::vector<float>> raw_force_map;
    private: std::map <std::string, std::vector<float>> filtered_force_map;
    private: std::vector<std::vector<float>> contact_positions_v;
    private: std::vector<float> contact_forces_v;
    private: std::vector<std::vector<float>> filtered_contact_positions_v;
    private: std::vector<int> active_taxels_indices;
    private: std::string target_sensor;
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
    // brief a publisher for deleting a model in gazebo simulator
    private: transport::PublisherPtr model_del_pub;

    /// \brief A subscriber to a named topic.
    private: std::vector<transport::SubscriberPtr> subscribers_v;
    private: std::vector<std::shared_ptr<sensors::Sensor>> sensor_v;
    private: gazebo::sensors::SensorManager *sensorManager;
    private: std::map <std::string, ignition::math::Pose3d> contacts_map;
    private: FILE *fp;
    private: FILE *fp_filtered;
    private: FILE *fp_inactive_taxels;
    private: FILE *fp_joints_pos;
    private: std::string act_cmd = "false";
    private: bool thread_running = true;

    //ROS subscriber

    /// \brief A node use for ROS transport
    private: std::unique_ptr<ros::NodeHandle> rosNode;

    /// \brief A ROS server
    private: ros::ServiceServer service;

    /// \brief A ROS callbackqueue that helps process messages
    private: ros::CallbackQueue rosQueue;

    /// \brief A thread the keeps running the rosQueue
    private: std::thread rosQueueThread;

    // This part defines the datastructure for the sensors taxels. This will be useful to define
    // the taxels locations w.r.t the correspondent sensor's centroid

    //Method to build the tree of taxels locations
    public: virtual void taxelsTreeBld();
    // Map to store the taxels locations wrt the brackets to which they are attached
    private: std::map <std::string, std::vector<geometry_msgs::Vector3Stamped>> taxels_map;
    // Map to store the taxels locations wrt gazebo world frame
    private: std::map <std::string, std::vector<std::vector<float>>> transformed_taxels_map;
    // Taxels positions, grouped per sensor
//    private: std::vector<geometry_msgs::Vector3Stamped> boh_taxels;
    private: std::vector<geometry_msgs::Vector3Stamped> palm_taxels;
    private: std::vector<geometry_msgs::Vector3Stamped> ipb_taxels;
    private: std::vector<geometry_msgs::Vector3Stamped> mpb_taxels;
    private: std::vector<geometry_msgs::Vector3Stamped> ppb_taxels;
    private: std::vector<geometry_msgs::Vector3Stamped> ipf_taxels;
    private: std::vector<geometry_msgs::Vector3Stamped> mpf_taxels;
    private: std::vector<geometry_msgs::Vector3Stamped> ppf_taxels;
    private: std::vector<geometry_msgs::Vector3Stamped> tmb_taxels;
    private: std::vector<geometry_msgs::Vector3Stamped> imb_taxels;
    private: std::vector<geometry_msgs::Vector3Stamped> mmb_taxels;
    private: std::vector<geometry_msgs::Vector3Stamped> pmb_taxels;
    private: std::vector<geometry_msgs::Vector3Stamped> tmf_taxels;
    private: std::vector<geometry_msgs::Vector3Stamped> imf_taxels;
    private: std::vector<geometry_msgs::Vector3Stamped> mmf_taxels;
    private: std::vector<geometry_msgs::Vector3Stamped> pmf_taxels;
//    private: std::vector<geometry_msgs::Vector3Stamped> tft_taxels;
    private: std::vector<geometry_msgs::Vector3Stamped> ift_taxels;
//    private: std::vector<geometry_msgs::Vector3Stamped> mft_taxels;
//    private: std::vector<geometry_msgs::Vector3Stamped> pft_taxels;
    // Transformed taxels positions, grouped per sensor
//    private: std::vector<std::vector<float>> trans_boh_taxels;
    private: std::vector<std::vector<float>> trans_palm_taxels;
    private: std::vector<std::vector<float>> trans_ipb_taxels;
    private: std::vector<std::vector<float>> trans_mpb_taxels;
    private: std::vector<std::vector<float>> trans_ppb_taxels;
    private: std::vector<std::vector<float>> trans_ipf_taxels;
    private: std::vector<std::vector<float>> trans_mpf_taxels;
    private: std::vector<std::vector<float>> trans_ppf_taxels;
    private: std::vector<std::vector<float>> trans_tmb_taxels;
    private: std::vector<std::vector<float>> trans_imb_taxels;
    private: std::vector<std::vector<float>> trans_mmb_taxels;
    private: std::vector<std::vector<float>> trans_pmb_taxels;
    private: std::vector<std::vector<float>> trans_tmf_taxels;
    private: std::vector<std::vector<float>> trans_imf_taxels;
    private: std::vector<std::vector<float>> trans_mmf_taxels;
    private: std::vector<std::vector<float>> trans_pmf_taxels;
//    private: std::vector<std::vector<float>> trans_tft_taxels;
    private: std::vector<std::vector<float>> trans_ift_taxels;
//    private: std::vector<std::vector<float>> trans_mft_taxels;
//    private: std::vector<std::vector<float>> trans_pft_taxels;
    // A variable to store the tf transformation messages for each bracket on the allegro hand
    private: geometry_msgs::TransformStamped transformStamped;
    private: int sensor_count;
    // PlaceHolders
    private: geometry_msgs::Vector3Stamped v3s;
    // An integer value to keep track of the iteration of the contact points registration
    private: int reg_iter_count;
    // A string to keep track of the grasp attempt
    private: std::string reg_grasp_attempt;
    // A variable to store the pose of the wrist_3_link (which is equivalent to tool0's pose)
    private: ignition::math::Pose3d wrist_pose;
    // Vectors to store the model joints entities and their attributes
    private: std::vector<boost::shared_ptr<physics::Joint>> joints_v;
    private: std::vector<std::string> joints_names;
    private: std::vector<double> joints_pos;

  };

}
#endif
