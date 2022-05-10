#include "JointStatesPublisher.hh"

using namespace gazebo;
// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(JointStatesPublisher)

JointStatesPublisher::JointStatesPublisher() : ModelPlugin ()
{
}
JointStatesPublisher::~JointStatesPublisher()
{
}

void JointStatesPublisher::Load(gazebo::physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
    this->model = _parent;
    this->sdf = _sdf;
    physics::Joint_V all_joints_vector = model->GetJoints();

    for (auto const &j : all_joints_vector)
    {
        if (j->GetType() != (j->FIXED_JOINT | j->JOINT))
        {
            this->joints_vector.push_back(j);
        }


     }

    InitializeROSMembers();

    this->updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&JointStatesPublisher::OnUpdate, this));

}

void JointStatesPublisher::OnUpdate()
{
    ReadSimJointsData();
}

void JointStatesPublisher::InitializeROSMembers()
{
    if (!ros::isInitialized())
    {
        int argc = 0;
        char **argv = nullptr;
        ros::init(argc, argv, "Joint_States_Publisher", ros::init_options::NoSigintHandler);
    }
    this->rosNode = std::make_unique<ros::NodeHandle>(this->model->GetName());
    this->rosPubJointStates = this->rosNode->advertise<sensor_msgs::JointState>("/joint_states", 10, false);
    this->timer = this->rosNode->createTimer(ros::Duration(1.0 / 50.), &JointStatesPublisher::timerCallback,
                                                        this);    // TODO: Change rate later


    this->joint_states_msg = sensor_msgs::JointState();

    this->num_joints = this->joints_vector.size();

    this->joint_states_msg.position.resize(this->num_joints);
    this->joint_states_msg.velocity.resize(this->num_joints);
    this->joint_states_msg.effort.resize(this->num_joints);

    this->j_pos.resize(num_joints);
    this->j_vel.resize(num_joints);
    this->j_effort.resize(num_joints);

    for (auto const &j : joints_vector)
    {
        this->joint_states_msg.name.push_back(j->GetName());
    }
}

void JointStatesPublisher::timerCallback(const ros::TimerEvent &event)
{
    for (int i = 0; i < num_joints; i++)
    {
        this->joint_states_msg.position[i] = this->j_pos.at(i);
        this->joint_states_msg.velocity[i] = this->j_vel.at(i);
        this->joint_states_msg.effort[i] = this->j_effort.at(i);
    }
    joint_states_msg.header.stamp = ros::Time::now();
    this->rosPubJointStates.publish(this->joint_states_msg);

}

void JointStatesPublisher::ReadSimJointsData()
{
    for (int i = 0; i < num_joints; i++)
    {
        this->j_pos.at(i) = GetJointPosition(joints_vector.at(i)); //Joint position in radians
        this->j_effort.at(i) = joints_vector.at(i)->GetForce(0);
        this->j_vel.at(i) = alpha * joints_vector.at(i)->GetVelocity(0) + (1 - alpha) * this->j_vel.at(i);
    }
}

double JointStatesPublisher::GetJointPosition(const physics::JointPtr joint, const double axis_index)
{
#if GAZEBO_MAJOR_VERSION >= 8
    return joint->Position(axis_index); //the axis_index was set to 0 in the header file
                                        // which refers to the joint's only dof axis (P.S. for more
                                        // dof, more axes will be available for call)
#else
    return joint->GetAngle(axis_index).Radian();
#endif
}
