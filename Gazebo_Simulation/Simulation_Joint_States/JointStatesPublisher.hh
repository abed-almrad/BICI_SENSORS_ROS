#include <iostream>
#include <string>
#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>

namespace gazebo
{
    class JointStatesPublisher : public ModelPlugin
    {

    public:
        /// \brief Constructor.
        JointStatesPublisher();

        /// \brief Destructor.
        ~JointStatesPublisher();

        void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
        void OnUpdate();

    private:
        physics::ModelPtr model;
        physics::Joint_V joints_vector;
        sdf::ElementPtr sdf;
        event::ConnectionPtr updateConnection;

        int num_joints = 0;
        std::vector <std::string> j_names;
        std::vector<double> j_pos, j_vel, j_effort;
        double alpha = 0.01;

        // ROS members
        std::unique_ptr<ros::NodeHandle> rosNode;
        ros::Publisher rosPubJointStates, rosPubJointStates2;
        ros::Timer timer;
        sensor_msgs::JointState joint_states_msg;

        void InitializeROSMembers();
        void timerCallback(const ros::TimerEvent &event);
        void ReadSimJointsData();
        double GetJointPosition(const physics::JointPtr joint, const double axis_index = 0);

    };
}
