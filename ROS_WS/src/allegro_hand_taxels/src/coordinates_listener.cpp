#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/JointState.h>
#include <bici_ros_sensor_reader/TactileData.h>
#include <vector>
#include<algorithm>
#include <map>
#include <iostream>
#include <fstream>
#include <cstring>
#include "allegro_hand_taxels/coordinates_saving.h"
#include "allegro_hand_taxels/taxels_data_avg.h"
#include <string>
#include <filesystem>
#include <unistd.h>

#define NUM_SENSORS 22
#define NUM_SENSOR_MIN 8
#define NUM_ACTIVE_SENSORS 15 //The number of sensors that are currently installed and operational on the allegro hand 
// Sensors sizes
#define BOH_SIZE 118
#define PALM_SIZE 121
#define PB_SIZE 78
#define PF_SIZE 65
#define MB_SIZE 30
#define MF_SIZE 27
#define FT_SIZE 66
#define TMB_SIZE 47
#define TMF_SIZE 31

using namespace std;

ofstream active_taxels_file;
ofstream inactive_taxels_file;
ofstream joints_pos_file;
string act_cmd = "false";
string avg_cmd = "false";
int averaged_sensors_count = 0;
int reg_iter_count;
string reg_grasp_attempt;
int counter; //counter for adequately filling the markers_array before pushing to the rviz topic
tf2_ros::Buffer tfBuffer;
tf2_ros::TransformListener *tfListener;
visualization_msgs::MarkerArray markers_array; //Array of markers (e.g. a marker is created for each taxel)
int taxel_values_nb_max = 100; // Number of values used to initially to calculate the average for each taxel
//Variables for joints data
vector<string> ur_joint_names;
vector<double> ur_joint_positions;
vector<string> allegro_joint_names;
vector<double> allegro_joint_positions;

class Tactile_Sensor {
  public:
    uint8_t sensor_num;        // ID of the sensor
    int size;                  // The number of taxels
    string sensor_name;          // The link on which the sensor is installed
    uint16_t threshold = 150;  // Capacitance raw count threshold to detect contact.
                               // This might be later on initialized individually for each
                               //sensor or even taxel, after tuning
    uint32_t timestamp = 0; // This can be assigned to any random number
    std::vector<float> data;
    std::vector<float> temp_avg; // Temporary array of taxels average values for each sensor
    std::vector<float> avg; // Array of taxels average values for each sensor
    size_t taxel_values_nb = 0; // A variable to keep track of the number of values used to initially to calculate the average for each taxel
    bool sensor_averaged = false;
    geometry_msgs::TransformStamped transformStamped;
    // Two vectors for each sensor, storing the positions of respectively its active and inactive taxels relative to the world frame
    vector<vector<float>> trans_active_taxels;
    vector<vector<float>> trans_inactive_taxels;
    // Vector of for registering the contact forces (force = raw count)
    vector<float> contact_forces;
    // A vector for each sensor, storing the positions of its taxels relative to the frame attached to its origin
    vector<geometry_msgs::Vector3Stamped> sensor_taxels;
    // A vector of markers for each sensor to visualize the activation of its taxels
    vector<visualization_msgs::Marker> sensor_markers; 
    Tactile_Sensor(uint8_t sensor_num, int size, string sensor_name)
    {
        //Any other number can be used for initialization
        this->sensor_num = sensor_num;
        this->size = size;
        this->sensor_name = sensor_name;
        this->data.resize(size);
        this->avg.resize(size);
        this->temp_avg.resize(size);
        this->trans_active_taxels.resize(size, std::vector<float>(3, 0));//Vector monitoring the sensor's activated taxels positions
        this->trans_inactive_taxels.resize(size, std::vector<float>(3, 0));//Vector monitoring the sensor's inactive taxels positions
        this->contact_forces.resize(size);//Vector monitoring the contact forces at each taxel (force = raw count)
        this->sensor_taxels.resize(size); //Vector to store the positions of the taxels for each sensor relative to their correspondant bracket    
        this->sensor_markers.resize(size);    
    }
};

//The set of integrated sensors:

Tactile_Sensor sensor1(8, PF_SIZE, "ipf");
Tactile_Sensor sensor2(9, MB_SIZE, "pmb");
Tactile_Sensor sensor3(10, PB_SIZE, "ipb");
Tactile_Sensor sensor4(11, FT_SIZE, "mft");
Tactile_Sensor sensor5(12, MF_SIZE, "mmf");
Tactile_Sensor sensor6(13, PF_SIZE, "mpf");
Tactile_Sensor sensor7(14, MB_SIZE, "mmb");
Tactile_Sensor sensor8(15, PB_SIZE, "mpb");
Tactile_Sensor sensor9(16, FT_SIZE, "tft");
Tactile_Sensor sensor10(17, TMF_SIZE, "tmf");
Tactile_Sensor sensor11(18, PF_SIZE, "tpf");
Tactile_Sensor sensor12(19, TMB_SIZE, "tmb");
Tactile_Sensor sensor13(20, PB_SIZE, "tpb");
Tactile_Sensor sensor14(21, PALM_SIZE, "palm");
Tactile_Sensor sensor15(22, BOH_SIZE, "boh");
Tactile_Sensor sensor16(23, FT_SIZE, "pft");
Tactile_Sensor sensor17(24, MF_SIZE, "pmf");
Tactile_Sensor sensor18(25, PF_SIZE, "ppf");
Tactile_Sensor sensor19(26, MB_SIZE, "imb");
Tactile_Sensor sensor20(27, PB_SIZE, "ppb");
Tactile_Sensor sensor21(28, FT_SIZE, "ift");
Tactile_Sensor sensor22(29, MF_SIZE, "imf");



Tactile_Sensor Sensors_Array[NUM_SENSORS]{sensor1, sensor2, sensor3, sensor4, sensor5, sensor6, sensor7, sensor8,
sensor9, sensor10, sensor11, sensor12, sensor13, sensor14, sensor15, sensor16, sensor17, sensor18, sensor19,
sensor20, sensor21, sensor22};

//Method for converting a 3D vector into its homogeneous representation
std::vector<float> homo_v(geometry_msgs::Vector3Stamped v3stamped)
{
  std::vector<float> result;
    result.push_back(v3stamped.vector.x);
    result.push_back(v3stamped.vector.y);
    result.push_back(v3stamped.vector.z);
    result.push_back(1.0);

    return result;
}

//Method for extracting the homogeneous matrix from the tf2 transformation
std::vector<std::vector<float>> homo_matrix(geometry_msgs::TransformStamped transformStamped)
{
    double q0,q1,q2,q3,dx,dy,dz,norm;
    std::vector<std::vector<float>> homo_matrix;
    //Matrix resizing
    homo_matrix.resize(4,std::vector<float>(4,0));
    q0 = transformStamped.transform.rotation.x;
    q1 = transformStamped.transform.rotation.y;
    q2 = transformStamped.transform.rotation.z;
    q3 = transformStamped.transform.rotation.w;
    dx = transformStamped.transform.translation.x;
    dy = transformStamped.transform.translation.y;
    dz = transformStamped.transform.translation.z;
    norm = pow(q0,2)+pow(q1,2)+pow(q2,2)+pow(q3,2);
//Wikipedia Method
    homo_matrix[0][0] = 1-2*norm*(pow(q1,2)+pow(q2,2));
    homo_matrix[0][1] = 2*norm*(q0*q1-q2*q3);
    homo_matrix[0][2] = 2*norm*(q0*q2+q1*q3);
    homo_matrix[0][3] = dx;
    homo_matrix[1][0] = 2*norm*(q0*q1+q2*q3);
    homo_matrix[1][1] = 1-2*norm*(pow(q0,2)+pow(q2,2));
    homo_matrix[1][2] = 2*norm*(q1*q2-q0*q3);
    homo_matrix[1][3] = dy;
    homo_matrix[2][0] = 2*norm*(q0*q2-q1*q3);
    homo_matrix[2][1] = 2*norm*(q1*q2+q0*q3);
    homo_matrix[2][2] = 1-2*norm*(pow(q0,2)+pow(q1,2));
    homo_matrix[2][3] = dz;
    homo_matrix[3][0] = 0;
    homo_matrix[3][1] = 0;
    homo_matrix[3][2] = 0;
    homo_matrix[3][3] = 1;

    return homo_matrix;
}

//Method for matrix multiplication
std::vector<float> homotrans(geometry_msgs::TransformStamped transformStamped,geometry_msgs::Vector3Stamped v3stamped)
{
    std::vector<float> homogeneous_vector = homo_v(v3stamped);
    /*
    std::cout << "Homogeneous vector: " << homogeneous_vector[0] << " " << homogeneous_vector[1]
              << " " << homogeneous_vector[2] <<" " << homogeneous_vector[3] << std::endl;
    */
    std::vector<std::vector<float>> homogeneous_matrix = homo_matrix(transformStamped);
    /*
    std::cout << "Homogeneous matrix:\n"
              << homogeneous_matrix[0][0] << " " << homogeneous_matrix[0][1]
              << " " << homogeneous_matrix[0][2] << " " << homogeneous_matrix[0][3] << "\n"
              << homogeneous_matrix[1][0] << " " << homogeneous_matrix[1][1]<< " "
              <<homogeneous_matrix[1][2] << " " << homogeneous_matrix[1][3] << "\n"
              << homogeneous_matrix[2][0] << " " << homogeneous_matrix[2][1]<< " "
              <<homogeneous_matrix[2][2] << " " << homogeneous_matrix[2][3] << "\n"
              << homogeneous_matrix[3][0] << " " << homogeneous_matrix[3][1]<< " "
              <<homogeneous_matrix[3][2] << " " << homogeneous_matrix[3][3] << "\n"<<std::endl;
    */
    std::vector<float> result;
    result.resize(3);
    for(int row=0; row<homogeneous_matrix.size()-1;row++)
    {
        for(int column=0; column<homogeneous_matrix[0].size();column++)
        {
            result[row] = result[row] + homogeneous_matrix[row][column]*homogeneous_vector[column];
        }
    }

    //std::cout<<"Result: "<< result[0] << result[1] << result[2] << std::endl;
    return result;
}

//Method for the coordinates saving activation

bool saving_activation_function(allegro_hand_taxels::coordinates_saving::Request &req,
                                       allegro_hand_taxels::coordinates_saving::Response &res)
{
    res.saving_activation_response = req.saving_activation;
    std::cout << "Requested saving activation boolean string: " << req.saving_activation << std::endl;
    act_cmd = res.saving_activation_response;
    reg_iter_count = req.iteration_nb;
    reg_grasp_attempt = req.grasp_attempt;
    std::cout<<"sending back response: "<< act_cmd << std::endl;
    return true;


}

//Method for the taxels data averaging activation

bool avg_activation_function(allegro_hand_taxels::taxels_data_avg::Request &req,
                                       allegro_hand_taxels::taxels_data_avg::Response &res)
{
    res.avg_activation_response = req.avg_activation;
    std::cout << "Requested averaging activation boolean string: " << req.avg_activation << std::endl;
    avg_cmd = res.avg_activation_response;
    std::cout<<"sending back response: "<< avg_cmd << std::endl;
    return true;


}

//Joint states callback
void joint_callback(const sensor_msgs::JointState msg)
{
    if(msg.name[0].find("elbow_joint") != string::npos) // If elbow_joint is found in the first name, this means
    {                                                   // that we are dealing with the ur5 joints message
        ur_joint_names = msg.name;
        ur_joint_positions = msg.position;
    }
    if(msg.name[0].find("joint_0_0") != string::npos) // If joint_0_0 is found in the first name, this means
    {                                             // that we are dealing with the allegro hand joints message
        allegro_joint_names = msg.name;
        allegro_joint_positions = msg.position;
    }

    if (act_cmd == "true") // Activation of a save command through a ros node
    {
        string joints_pos_path = "/home/abed/Documents/BICI_Project/BICI_SENSORS_ROS/ROS_WS/Pt_Cloud_Scripts/"+reg_grasp_attempt+"_joints_pos_"+to_string(reg_iter_count)+".csv";
        joints_pos_file.open(joints_pos_path.c_str());

    }

}
//One common callback function
void callback(const bici_ros_sensor_reader::TactileData msg)
{
    Sensors_Array[msg.sensor_num-8].timestamp = msg.timestamp;
    Sensors_Array[msg.sensor_num-8].data = msg.data;

// Taxels data averaging following an avg_cmd command
    if (Sensors_Array[msg.sensor_num-8].taxel_values_nb < taxel_values_nb_max && avg_cmd == "true")
    {
        std::transform(Sensors_Array[msg.sensor_num-8].temp_avg.begin(), 
                       Sensors_Array[msg.sensor_num-8].temp_avg.end(),
                       msg.data.begin(), Sensors_Array[msg.sensor_num-8].temp_avg.begin(),
                       std::plus<float>()); // Running sum for the taxels values
        Sensors_Array[msg.sensor_num-8].taxel_values_nb += 1;
        return;
    }
    else if ((Sensors_Array[msg.sensor_num-8].taxel_values_nb == taxel_values_nb_max) && !Sensors_Array[msg.sensor_num-8].sensor_averaged && avg_cmd == "true")
    {
        int k = taxel_values_nb_max;
        std::transform(Sensors_Array[msg.sensor_num-8].temp_avg.begin(),
                       Sensors_Array[msg.sensor_num-8].temp_avg.end(),
                       Sensors_Array[msg.sensor_num-8].temp_avg.begin(), [k](float &c){ return c/k; });
        std::cout << "\n" << "For sensor number " << msg.sensor_num-0 << ", the baseline values are: "; 
        for (size_t i = 0; i < Sensors_Array[msg.sensor_num-8].temp_avg.size(); i++)
        {
            std::cout << Sensors_Array[msg.sensor_num-8].temp_avg[i] << ", "; 
        }
        std::cout << "." << std::endl;
        Sensors_Array[msg.sensor_num-8].sensor_averaged = true;    
    }
    if(avg_cmd == "true")
    {
        for (size_t sensor = 0; sensor < NUM_SENSORS; sensor++)
        {
            if (!(sensor >= 8 && sensor <=12)) // This line should be removed after adding the thumb's sensors to the allegro hand
            {
                if(Sensors_Array[sensor].sensor_averaged)
                {
                    averaged_sensors_count += 1;
                }
            }
        }
        
        if (averaged_sensors_count == NUM_ACTIVE_SENSORS) // i.e. if all tha sensors installed on the
                                                    // allegro hand were averaged following the avg_cmd command
        {
            avg_cmd = "false";
            for (size_t sensor = 0; sensor < NUM_SENSORS; sensor++)
            {
                Sensors_Array[sensor].taxel_values_nb = 0;
                Sensors_Array[sensor].sensor_averaged = false;
                averaged_sensors_count = 0;
                Sensors_Array[sensor].avg = Sensors_Array[sensor].temp_avg;
                Sensors_Array[sensor].temp_avg.clear();
                Sensors_Array[sensor].temp_avg.resize(Sensors_Array[sensor].size);
            }
            
        }
        else if (averaged_sensors_count <NUM_ACTIVE_SENSORS)
        {
            averaged_sensors_count = 0;
        }
    }
    
    
    

    if (act_cmd == "true") // Activation of a save command through a ros node
    {
        string path = "/home/abed/Documents/BICI_Project/BICI_SENSORS_ROS/ROS_WS/Pt_Cloud_Scripts/"+reg_grasp_attempt+"_active_points_coordinates_"+to_string(reg_iter_count)+".csv";
        string path_inactive_taxels = "/home/abed/Documents/BICI_Project/BICI_SENSORS_ROS/ROS_WS/Pt_Cloud_Scripts/"+reg_grasp_attempt+"_inactive_points_coordinates_"+to_string(reg_iter_count)+".csv";
        string joints_pos_path = "/home/abed/Documents/BICI_Project/BICI_SENSORS_ROS/ROS_WS/Pt_Cloud_Scripts/"+reg_grasp_attempt+"_joints_pos_"+to_string(reg_iter_count)+".csv";
        cout<<"THE SAVING PATH FOR THE ACTIVE PT CLOUD IS: " + path << endl;
        active_taxels_file.open(path.c_str());
        inactive_taxels_file.open(path_inactive_taxels.c_str());
        joints_pos_file.open(joints_pos_path.c_str());

        //Writing active taxels data to a csv file
        for(int j=0;j<NUM_SENSORS;j++)
        {
            for(int taxel=0;taxel<Sensors_Array[j].size;taxel++)
            {
                active_taxels_file << Sensors_Array[j].trans_active_taxels[taxel][0] << ','
                                << Sensors_Array[j].trans_active_taxels[taxel][1] << ','
                                << Sensors_Array[j].trans_active_taxels[taxel][2] << ','
                                << Sensors_Array[j].contact_forces[taxel] << ','
                                << '\n';
            }
            
            
        }

        active_taxels_file.close();

        //Writing inactive taxels data to a csv file
        for(int j=0;j<NUM_SENSORS;j++)
        {
            for(int taxel=0;taxel<Sensors_Array[j].size;taxel++)
            {
                inactive_taxels_file << Sensors_Array[j].trans_inactive_taxels[taxel][0] << ','
                                     << Sensors_Array[j].trans_inactive_taxels[taxel][1] << ','
                                     << Sensors_Array[j].trans_inactive_taxels[taxel][2] << ','
                                     << Sensors_Array[j].contact_forces[taxel] << ','
                                     << '\n';
            }            
        }

        inactive_taxels_file.close();

        //Writing the joints data to a .csv file
        for(int i=0;i<ur_joint_names.size();i++)
        {

                joints_pos_file << ur_joint_names[i] << ',' << ur_joint_positions[i] << ','
                                << '\n';  
            
        }
        for(int j=0;j<allegro_joint_names.size();j++)
        {

                joints_pos_file << allegro_joint_names[j] << ',' << allegro_joint_positions[j] << ','
                                << '\n';  
            
        }
        joints_pos_file.close();
        
        act_cmd = "false";     

    }
//    std::cout << "\n" << "Normalized readings for sensor number (" << msg.sensor_num-0 << "): " << std::endl;
    for(int taxel=0;taxel<Sensors_Array[msg.sensor_num-8].size;taxel++)
    {
        if (msg.sensor_num == 24 && taxel>20)
        {
            continue;
        }
//        std::cout << abs(Sensors_Array[msg.sensor_num-8].data[taxel]-Sensors_Array[msg.sensor_num-8].avg[taxel]) << " ";
       if (abs(Sensors_Array[msg.sensor_num-8].data[taxel]-Sensors_Array[msg.sensor_num-8].avg[taxel])>= Sensors_Array[msg.sensor_num-8].threshold)
       {
            std::cout << "\n" << "Activation difference for sensor number (" << msg.sensor_num-0 << "), Taxel number (" << taxel <<"): " << abs(Sensors_Array[msg.sensor_num-8].data[taxel]-Sensors_Array[msg.sensor_num-8].avg[taxel]) << std::endl;
            //cout << "marker color should change" << endl;
            Sensors_Array[msg.sensor_num-8].sensor_markers[taxel].ns = Sensors_Array[msg.sensor_num-8].sensor_name+"_sub_taxel";
            Sensors_Array[msg.sensor_num-8].sensor_markers[taxel].id= taxel;
            Sensors_Array[msg.sensor_num-8].sensor_markers[taxel].header.frame_id = Sensors_Array[msg.sensor_num-8].sensor_name+"_bracket";
            Sensors_Array[msg.sensor_num-8].sensor_markers[taxel].header.stamp = ros::Time(0);
            Sensors_Array[msg.sensor_num-8].sensor_markers[taxel].type = visualization_msgs::Marker::SPHERE;
            Sensors_Array[msg.sensor_num-8].sensor_markers[taxel].action = visualization_msgs::Marker::ADD;
            Sensors_Array[msg.sensor_num-8].sensor_markers[taxel].pose.position.x = Sensors_Array[msg.sensor_num-8].sensor_taxels[taxel].vector.x;
            Sensors_Array[msg.sensor_num-8].sensor_markers[taxel].pose.position.y = Sensors_Array[msg.sensor_num-8].sensor_taxels[taxel].vector.y;
            Sensors_Array[msg.sensor_num-8].sensor_markers[taxel].pose.position.z = Sensors_Array[msg.sensor_num-8].sensor_taxels[taxel].vector.z;
            Sensors_Array[msg.sensor_num-8].sensor_markers[taxel].pose.orientation.x = 0.0;
            Sensors_Array[msg.sensor_num-8].sensor_markers[taxel].pose.orientation.y = 0.0;
            Sensors_Array[msg.sensor_num-8].sensor_markers[taxel].pose.orientation.z = 0.0;
            Sensors_Array[msg.sensor_num-8].sensor_markers[taxel].pose.orientation.w = 1.0;
            Sensors_Array[msg.sensor_num-8].sensor_markers[taxel].scale.x = 0.003;
            Sensors_Array[msg.sensor_num-8].sensor_markers[taxel].scale.y = 0.003;
            Sensors_Array[msg.sensor_num-8].sensor_markers[taxel].scale.z = 0.003;
            Sensors_Array[msg.sensor_num-8].sensor_markers[taxel].color.r = 1.0f;
            Sensors_Array[msg.sensor_num-8].sensor_markers[taxel].color.g = 0.2f;
            Sensors_Array[msg.sensor_num-8].sensor_markers[taxel].color.b = 0.2f;
            Sensors_Array[msg.sensor_num-8].sensor_markers[taxel].lifetime = ros::Duration();
            
            Sensors_Array[msg.sensor_num-8].sensor_markers[taxel].color.a = 1.0;
        }
        else
        {
            //cout << "marker color should come back to normal" << endl;
            Sensors_Array[msg.sensor_num-8].sensor_markers[taxel].ns = Sensors_Array[msg.sensor_num-8].sensor_name+"_sub_taxel";
            Sensors_Array[msg.sensor_num-8].sensor_markers[taxel].id= taxel;
            Sensors_Array[msg.sensor_num-8].sensor_markers[taxel].header.frame_id = Sensors_Array[msg.sensor_num-8].sensor_name+"_bracket";
            Sensors_Array[msg.sensor_num-8].sensor_markers[taxel].header.stamp = ros::Time(0);
            Sensors_Array[msg.sensor_num-8].sensor_markers[taxel].type = visualization_msgs::Marker::SPHERE;
            Sensors_Array[msg.sensor_num-8].sensor_markers[taxel].action = visualization_msgs::Marker::ADD;
            Sensors_Array[msg.sensor_num-8].sensor_markers[taxel].pose.position.x = Sensors_Array[msg.sensor_num-8].sensor_taxels[taxel].vector.x;
            Sensors_Array[msg.sensor_num-8].sensor_markers[taxel].pose.position.y = Sensors_Array[msg.sensor_num-8].sensor_taxels[taxel].vector.y;
            Sensors_Array[msg.sensor_num-8].sensor_markers[taxel].pose.position.z = Sensors_Array[msg.sensor_num-8].sensor_taxels[taxel].vector.z;
            Sensors_Array[msg.sensor_num-8].sensor_markers[taxel].pose.orientation.x = 0.0;
            Sensors_Array[msg.sensor_num-8].sensor_markers[taxel].pose.orientation.y = 0.0;
            Sensors_Array[msg.sensor_num-8].sensor_markers[taxel].pose.orientation.z = 0.0;
            Sensors_Array[msg.sensor_num-8].sensor_markers[taxel].pose.orientation.w = 1.0;
            Sensors_Array[msg.sensor_num-8].sensor_markers[taxel].scale.x = 0.003;
            Sensors_Array[msg.sensor_num-8].sensor_markers[taxel].scale.y = 0.003;
            Sensors_Array[msg.sensor_num-8].sensor_markers[taxel].scale.z = 0.003;
            Sensors_Array[msg.sensor_num-8].sensor_markers[taxel].color.r = 1.0f;
            Sensors_Array[msg.sensor_num-8].sensor_markers[taxel].color.g = 0.2f;
            Sensors_Array[msg.sensor_num-8].sensor_markers[taxel].color.b = 0.2f;
            Sensors_Array[msg.sensor_num-8].sensor_markers[taxel].lifetime = ros::Duration();
            
            Sensors_Array[msg.sensor_num-8].sensor_markers[taxel].color.a = 0.0;            
        }
    
       if (tfBuffer.canTransform("world", Sensors_Array[msg.sensor_num-8].sensor_name+"_bracket", ros::Time(0))) 
       {
           try
           {
               Sensors_Array[msg.sensor_num-8].transformStamped = tfBuffer.lookupTransform("world", Sensors_Array[msg.sensor_num-8].sensor_name+"_bracket", ros::Time(0));                                                                                                                                                    
           }
           catch (ros::Exception &ex) 
           {
            ROS_WARN("%s",ex.what());
            ros::Duration(1.0).sleep();
            continue;
            }
            // **********************************
            // **********************************
            // The below if statement is only TEMPORARY (i.e. should be removed later) to suppress 
            // some taxels that are giving me
            // headache (possibly because of a soldering or sensor/dielectric installation issue on
            // the allegro hand)

            if (msg.sensor_num == 24 && taxel>20)
            {
                continue;
            }
            // The following if statement ignores the activation of the taxels that are located on the sides of the allegro hand
            // since it is highly unlikely for them to get activated due to a contact with the grasped object 
            if(abs(Sensors_Array[msg.sensor_num-8].data[taxel]-Sensors_Array[msg.sensor_num-8].avg[taxel])>= Sensors_Array[msg.sensor_num-8].threshold
               && !(msg.sensor_num-8==0 && taxel>44)&&!(msg.sensor_num-8==5 && taxel>44)&&!(msg.sensor_num-8==17 && taxel>44)// side taxels of the proximal front sensors     
               && !(msg.sensor_num-8==21 && taxel>17) && !(msg.sensor_num-8==4 && taxel>17) && !(msg.sensor_num-8==16 && taxel>17) // side taxels of the medial front sensors
               && !(msg.sensor_num-8==20 && taxel<30) && !(msg.sensor_num-8==3 && taxel<30) && !(msg.sensor_num-8==15 && taxel<30) // side taxels of the fingertip sensors
              )

            {
                Sensors_Array[msg.sensor_num-8].trans_active_taxels[taxel][0] = homotrans(Sensors_Array[msg.sensor_num-8].transformStamped,Sensors_Array[msg.sensor_num-8].sensor_taxels[taxel])[0];
                //cout<< "X Coordinate " << Sensors_Array[msg.sensor_num-8].trans_active_taxels[taxel][0] << endl; 
                Sensors_Array[msg.sensor_num-8].trans_active_taxels[taxel][1] = homotrans(Sensors_Array[msg.sensor_num-8].transformStamped,Sensors_Array[msg.sensor_num-8].sensor_taxels[taxel])[1];
                //cout<< "Y Coordinate " << Sensors_Array[msg.sensor_num-8].trans_active_taxels[taxel][1] << endl;
                Sensors_Array[msg.sensor_num-8].trans_active_taxels[taxel][2] = homotrans(Sensors_Array[msg.sensor_num-8].transformStamped,Sensors_Array[msg.sensor_num-8].sensor_taxels[taxel])[2];
                //cout<< "Z Coordinate " << Sensors_Array[msg.sensor_num-8].trans_active_taxels[taxel][2] << endl;
                //Contact force registration
                Sensors_Array[msg.sensor_num-8].contact_forces[taxel] = abs(Sensors_Array[msg.sensor_num-8].data[taxel]-Sensors_Array[msg.sensor_num-8].avg[taxel]);
                
                Sensors_Array[msg.sensor_num-8].trans_inactive_taxels[taxel][0] = 0;
                Sensors_Array[msg.sensor_num-8].trans_inactive_taxels[taxel][1] = 0;
                Sensors_Array[msg.sensor_num-8].trans_inactive_taxels[taxel][2] = 0;
            }
            else
            {
                Sensors_Array[msg.sensor_num-8].trans_inactive_taxels[taxel][0] = homotrans(Sensors_Array[msg.sensor_num-8].transformStamped,Sensors_Array[msg.sensor_num-8].sensor_taxels[taxel])[0];
                //cout<< "X Coordinate " << Sensors_Array[msg.sensor_num-8].trans_inactive_taxels[taxel][0] << endl; 
                Sensors_Array[msg.sensor_num-8].trans_inactive_taxels[taxel][1] = homotrans(Sensors_Array[msg.sensor_num-8].transformStamped,Sensors_Array[msg.sensor_num-8].sensor_taxels[taxel])[1];
                //cout<< "Y Coordinate " << Sensors_Array[msg.sensor_num-8].trans_inactive_taxels[taxel][1] << endl;
                Sensors_Array[msg.sensor_num-8].trans_inactive_taxels[taxel][2] = homotrans(Sensors_Array[msg.sensor_num-8].transformStamped,Sensors_Array[msg.sensor_num-8].sensor_taxels[taxel])[2];
                //cout<< "Z Coordinate " << Sensors_Array[msg.sensor_num-8].trans_inactive_taxels[taxel][2] << endl;
                //Contact force registration
                Sensors_Array[msg.sensor_num-8].contact_forces[taxel] = 0;  

                Sensors_Array[msg.sensor_num-8].trans_active_taxels[taxel][0] = 0;
                Sensors_Array[msg.sensor_num-8].trans_active_taxels[taxel][1] = 0;
                Sensors_Array[msg.sensor_num-8].trans_active_taxels[taxel][2] = 0;
            }
            
        }
        else
        {
            std::cout << "No transformation is available" << std::endl;
        }
        
    }
    counter = 0;
    for (size_t i = 0; i < (msg.sensor_num-8); i++)
    {
        counter += Sensors_Array[i].size;
    }
    

    for (size_t taxel = 0; taxel < Sensors_Array[msg.sensor_num-8].sensor_taxels.size(); taxel++)
    {
 //     cout << "Marker's frame: " << Sensors_Array[msg.sensor_num-8].sensor_markers[taxel].header.frame_id << endl;
 //     cout << "Marker's transparency: " << Sensors_Array[msg.sensor_num-8].sensor_markers[taxel].color.a << endl; 
      markers_array.markers[counter+taxel] = Sensors_Array[msg.sensor_num-8].sensor_markers[taxel];
    }


}



int main(int argc, char** argv){
    ros::init(argc, argv, "coordinates_listener");
    ros::NodeHandle node;
    ros::Rate rate(1.0);
    tfListener = new tf2_ros::TransformListener(tfBuffer);

    // Specifying the taxels positions w.r.t. to the correspondent brackets frames (i.e. the frames associated with " link_"bracket_name" " in onshape (P.S. Reconsider using push_back instead)
    //*******************************************************************
    //***************************************************************
/*
    //BOH
    //Row1
    Sensors_Array[14].sensor_taxels[0].vector.x = -0.026741;
    Sensors_Array[14].sensor_taxels[0].vector.y = -0.000447;
    Sensors_Array[14].sensor_taxels[0].vector.z = -0.0098;
    Sensors_Array[14].sensor_taxels[1].vector.x = -0.018232;
    Sensors_Array[14].sensor_taxels[1].vector.y = -0.000447;
    Sensors_Array[14].sensor_taxels[1].vector.z = -0.0098;
    Sensors_Array[14].sensor_taxels[2].vector.x = -0.009723;
    Sensors_Array[14].sensor_taxels[2].vector.y = -0.000447;
    Sensors_Array[14].sensor_taxels[2].vector.z = -0.0098;
    Sensors_Array[14].sensor_taxels[3].vector.x = -0.001214;
    Sensors_Array[14].sensor_taxels[3].vector.y = -0.000447;
    Sensors_Array[14].sensor_taxels[3].vector.z = -0.0098;
    Sensors_Array[14].sensor_taxels[4].vector.x = 0.007295;
    Sensors_Array[14].sensor_taxels[4].vector.y = -0.000447;
    Sensors_Array[14].sensor_taxels[4].vector.z = -0.0098;
    Sensors_Array[14].sensor_taxels[5].vector.x = 0.015804;
    Sensors_Array[14].sensor_taxels[5].vector.y = -0.000447;
    Sensors_Array[14].sensor_taxels[5].vector.z = -0.0098;
    Sensors_Array[14].sensor_taxels[6].vector.x = 0.024313;
    Sensors_Array[14].sensor_taxels[6].vector.y = -0.000447;
    Sensors_Array[14].sensor_taxels[6].vector.z = -0.0098;
    Sensors_Array[14].sensor_taxels[7].vector.x = 0.032822;
    Sensors_Array[14].sensor_taxels[7].vector.y = -0.000447;
    Sensors_Array[14].sensor_taxels[7].vector.z = -0.0098;
    Sensors_Array[14].sensor_taxels[8].vector.x = 0.041331;
    Sensors_Array[14].sensor_taxels[8].vector.y = -0.000447;
    Sensors_Array[14].sensor_taxels[8].vector.z = -0.0098;
    Sensors_Array[14].sensor_taxels[9].vector.x = 0.049840;
    Sensors_Array[14].sensor_taxels[9].vector.y = -0.000447;
    Sensors_Array[14].sensor_taxels[9].vector.z = -0.0098;
    Sensors_Array[14].sensor_taxels[10].vector.x = 0.058349;
    Sensors_Array[14].sensor_taxels[10].vector.y = -0.000447;
    Sensors_Array[14].sensor_taxels[10].vector.z = -0.0098;
    Sensors_Array[14].sensor_taxels[11].vector.x = 0.066858;
    Sensors_Array[14].sensor_taxels[11].vector.y = -0.000447;
    Sensors_Array[14].sensor_taxels[11].vector.z = -0.0098;
    //Row2
    Sensors_Array[14].sensor_taxels[12].vector.x = -0.026741;
    Sensors_Array[14].sensor_taxels[12].vector.y = -0.007149;
    Sensors_Array[14].sensor_taxels[12].vector.z = -0.0098;
    Sensors_Array[14].sensor_taxels[13].vector.x = -0.018232;
    Sensors_Array[14].sensor_taxels[13].vector.y = -0.007149;
    Sensors_Array[14].sensor_taxels[13].vector.z = -0.0098;
    Sensors_Array[14].sensor_taxels[14].vector.x = -0.009723;
    Sensors_Array[14].sensor_taxels[14].vector.y = -0.007149;
    Sensors_Array[14].sensor_taxels[14].vector.z = -0.0098;
    Sensors_Array[14].sensor_taxels[15].vector.x = -0.001214;
    Sensors_Array[14].sensor_taxels[15].vector.y = -0.007149;
    Sensors_Array[14].sensor_taxels[15].vector.z = -0.0098;
    Sensors_Array[14].sensor_taxels[16].vector.x = 0.007295;
    Sensors_Array[14].sensor_taxels[16].vector.y = -0.007149;
    Sensors_Array[14].sensor_taxels[16].vector.z = -0.0098;
    Sensors_Array[14].sensor_taxels[17].vector.x = 0.015804;
    Sensors_Array[14].sensor_taxels[17].vector.y = -0.007149;
    Sensors_Array[14].sensor_taxels[17].vector.z = -0.0098;
    Sensors_Array[14].sensor_taxels[18].vector.x = 0.024313;
    Sensors_Array[14].sensor_taxels[18].vector.y = -0.007149;
    Sensors_Array[14].sensor_taxels[18].vector.z = -0.0098;
    Sensors_Array[14].sensor_taxels[19].vector.x = 0.032822;
    Sensors_Array[14].sensor_taxels[19].vector.y = -0.007149;
    Sensors_Array[14].sensor_taxels[19].vector.z = -0.0098;
    Sensors_Array[14].sensor_taxels[20].vector.x = 0.041331;
    Sensors_Array[14].sensor_taxels[20].vector.y = -0.007149;
    Sensors_Array[14].sensor_taxels[20].vector.z = -0.0098;
    Sensors_Array[14].sensor_taxels[21].vector.x = 0.04984;
    Sensors_Array[14].sensor_taxels[21].vector.y = -0.007149;
    Sensors_Array[14].sensor_taxels[21].vector.z = -0.0098;
    Sensors_Array[14].sensor_taxels[22].vector.x = 0.058349;
    Sensors_Array[14].sensor_taxels[22].vector.y = -0.007149;
    Sensors_Array[14].sensor_taxels[22].vector.z = -0.0098;
    Sensors_Array[14].sensor_taxels[23].vector.x = 0.066858;
    Sensors_Array[14].sensor_taxels[23].vector.y = -0.007149;
    Sensors_Array[14].sensor_taxels[23].vector.z = -0.0098;
    //Row3
    Sensors_Array[14].sensor_taxels[24].vector.x = -0.026741;
    Sensors_Array[14].sensor_taxels[24].vector.y = -0.01385;
    Sensors_Array[14].sensor_taxels[24].vector.z = -0.0098;
    Sensors_Array[14].sensor_taxels[25].vector.x = -0.018232;
    Sensors_Array[14].sensor_taxels[25].vector.y = -0.01385;
    Sensors_Array[14].sensor_taxels[25].vector.z = -0.0098;
    Sensors_Array[14].sensor_taxels[26].vector.x = -0.009723;
    Sensors_Array[14].sensor_taxels[26].vector.y = -0.01385;
    Sensors_Array[14].sensor_taxels[26].vector.z = -0.0098;
    Sensors_Array[14].sensor_taxels[27].vector.x = -0.001214;
    Sensors_Array[14].sensor_taxels[27].vector.y = -0.01385;
    Sensors_Array[14].sensor_taxels[27].vector.z = -0.0098;
    Sensors_Array[14].sensor_taxels[28].vector.x = 0.007295;
    Sensors_Array[14].sensor_taxels[28].vector.y = -0.01385;
    Sensors_Array[14].sensor_taxels[28].vector.z = -0.0098;
    Sensors_Array[14].sensor_taxels[29].vector.x = 0.015804;
    Sensors_Array[14].sensor_taxels[29].vector.y = -0.01385;
    Sensors_Array[14].sensor_taxels[29].vector.z = -0.0098;
    Sensors_Array[14].sensor_taxels[30].vector.x = 0.024313;
    Sensors_Array[14].sensor_taxels[30].vector.y = -0.01385;
    Sensors_Array[14].sensor_taxels[30].vector.z = -0.0098;
    Sensors_Array[14].sensor_taxels[31].vector.x = 0.032822;
    Sensors_Array[14].sensor_taxels[31].vector.y = -0.01385;
    Sensors_Array[14].sensor_taxels[31].vector.z = -0.0098;
    Sensors_Array[14].sensor_taxels[32].vector.x = 0.041331;
    Sensors_Array[14].sensor_taxels[32].vector.y = -0.01385;
    Sensors_Array[14].sensor_taxels[32].vector.z = -0.0098;
    Sensors_Array[14].sensor_taxels[33].vector.x = 0.04984;
    Sensors_Array[14].sensor_taxels[33].vector.y = -0.01385;
    Sensors_Array[14].sensor_taxels[33].vector.z = -0.0098;
    Sensors_Array[14].sensor_taxels[34].vector.x = 0.058349;
    Sensors_Array[14].sensor_taxels[34].vector.y = -0.01385;
    Sensors_Array[14].sensor_taxels[34].vector.z = -0.0098;
    Sensors_Array[14].sensor_taxels[35].vector.x = 0.066858;
    Sensors_Array[14].sensor_taxels[35].vector.y = -0.01385;
    Sensors_Array[14].sensor_taxels[35].vector.z = -0.0098;
    //Row4
    Sensors_Array[14].sensor_taxels[36].vector.x = -0.026741;
    Sensors_Array[14].sensor_taxels[36].vector.y = -0.020551;
    Sensors_Array[14].sensor_taxels[36].vector.z = -0.0098;
    Sensors_Array[14].sensor_taxels[37].vector.x = -0.018232;
    Sensors_Array[14].sensor_taxels[37].vector.y = -0.020551;
    Sensors_Array[14].sensor_taxels[37].vector.z = -0.0098;
    Sensors_Array[14].sensor_taxels[38].vector.x = -0.009723;
    Sensors_Array[14].sensor_taxels[38].vector.y = -0.020551;
    Sensors_Array[14].sensor_taxels[38].vector.z = -0.0098;
    Sensors_Array[14].sensor_taxels[39].vector.x = -0.001214;
    Sensors_Array[14].sensor_taxels[39].vector.y = -0.020551;
    Sensors_Array[14].sensor_taxels[39].vector.z = -0.0098;
    Sensors_Array[14].sensor_taxels[40].vector.x = 0.007295;
    Sensors_Array[14].sensor_taxels[40].vector.y = -0.020551;
    Sensors_Array[14].sensor_taxels[40].vector.z = -0.0098;
    Sensors_Array[14].sensor_taxels[41].vector.x = 0.015804;
    Sensors_Array[14].sensor_taxels[41].vector.y = -0.020551;
    Sensors_Array[14].sensor_taxels[41].vector.z = -0.0098;
    Sensors_Array[14].sensor_taxels[42].vector.x = 0.024313;
    Sensors_Array[14].sensor_taxels[42].vector.y = -0.020551;
    Sensors_Array[14].sensor_taxels[42].vector.z = -0.0098;
    Sensors_Array[14].sensor_taxels[43].vector.x = 0.032822;
    Sensors_Array[14].sensor_taxels[43].vector.y = -0.020551;
    Sensors_Array[14].sensor_taxels[43].vector.z = -0.0098;
    Sensors_Array[14].sensor_taxels[44].vector.x = 0.041331;
    Sensors_Array[14].sensor_taxels[44].vector.y = -0.020551;
    Sensors_Array[14].sensor_taxels[44].vector.z = -0.0098;
    Sensors_Array[14].sensor_taxels[45].vector.x = 0.04984;
    Sensors_Array[14].sensor_taxels[45].vector.y = -0.020551;
    Sensors_Array[14].sensor_taxels[45].vector.z = -0.0098;
    Sensors_Array[14].sensor_taxels[46].vector.x = 0.058349;
    Sensors_Array[14].sensor_taxels[46].vector.y = -0.020551;
    Sensors_Array[14].sensor_taxels[46].vector.z = -0.0098;
    Sensors_Array[14].sensor_taxels[47].vector.x = 0.066858;
    Sensors_Array[14].sensor_taxels[47].vector.y = -0.020551;
    Sensors_Array[14].sensor_taxels[47].vector.z = -0.0098;
    //Row5
    Sensors_Array[14].sensor_taxels[48].vector.x = -0.026741;
    Sensors_Array[14].sensor_taxels[48].vector.y = -0.027252;
    Sensors_Array[14].sensor_taxels[48].vector.z = -0.0098;
    Sensors_Array[14].sensor_taxels[49].vector.x = -0.018232;
    Sensors_Array[14].sensor_taxels[49].vector.y = -0.027252;
    Sensors_Array[14].sensor_taxels[49].vector.z = -0.0098;
    Sensors_Array[14].sensor_taxels[50].vector.x = -0.009723;
    Sensors_Array[14].sensor_taxels[50].vector.y = -0.027252;
    Sensors_Array[14].sensor_taxels[50].vector.z = -0.0098;
    Sensors_Array[14].sensor_taxels[51].vector.x = -0.001214;
    Sensors_Array[14].sensor_taxels[51].vector.y = -0.027252;
    Sensors_Array[14].sensor_taxels[51].vector.z = -0.0098;
    Sensors_Array[14].sensor_taxels[52].vector.x = 0.007295;
    Sensors_Array[14].sensor_taxels[52].vector.y = -0.027252;
    Sensors_Array[14].sensor_taxels[52].vector.z = -0.0098;
    Sensors_Array[14].sensor_taxels[53].vector.x = 0.015804;
    Sensors_Array[14].sensor_taxels[53].vector.y = -0.027252;
    Sensors_Array[14].sensor_taxels[53].vector.z = -0.0098;
    Sensors_Array[14].sensor_taxels[54].vector.x = 0.024313;
    Sensors_Array[14].sensor_taxels[54].vector.y = -0.027252;
    Sensors_Array[14].sensor_taxels[54].vector.z = -0.0098;
    Sensors_Array[14].sensor_taxels[55].vector.x = 0.032822;
    Sensors_Array[14].sensor_taxels[55].vector.y = -0.027252;
    Sensors_Array[14].sensor_taxels[55].vector.z = -0.0098;
    Sensors_Array[14].sensor_taxels[56].vector.x = 0.041331;
    Sensors_Array[14].sensor_taxels[56].vector.y = -0.027252;
    Sensors_Array[14].sensor_taxels[56].vector.z = -0.0098;
    Sensors_Array[14].sensor_taxels[57].vector.x = 0.04984;
    Sensors_Array[14].sensor_taxels[57].vector.y = -0.027252;
    Sensors_Array[14].sensor_taxels[57].vector.z = -0.0098;
    Sensors_Array[14].sensor_taxels[58].vector.x = 0.058349;
    Sensors_Array[14].sensor_taxels[58].vector.y = -0.027252;
    Sensors_Array[14].sensor_taxels[58].vector.z = -0.0098;
    Sensors_Array[14].sensor_taxels[59].vector.x = 0.066858;
    Sensors_Array[14].sensor_taxels[59].vector.y = -0.027252;
    Sensors_Array[14].sensor_taxels[59].vector.z = -0.0098;
    //Row6
    Sensors_Array[14].sensor_taxels[60].vector.x = -0.026741;
    Sensors_Array[14].sensor_taxels[60].vector.y = -0.033954;
    Sensors_Array[14].sensor_taxels[60].vector.z = -0.0098;
    Sensors_Array[14].sensor_taxels[61].vector.x = -0.018232;
    Sensors_Array[14].sensor_taxels[61].vector.y = -0.033954;
    Sensors_Array[14].sensor_taxels[61].vector.z = -0.0098;
    Sensors_Array[14].sensor_taxels[62].vector.x = -0.009723;
    Sensors_Array[14].sensor_taxels[62].vector.y = -0.033954;
    Sensors_Array[14].sensor_taxels[62].vector.z = -0.0098;
    Sensors_Array[14].sensor_taxels[63].vector.x = -0.001214;
    Sensors_Array[14].sensor_taxels[63].vector.y = -0.033954;
    Sensors_Array[14].sensor_taxels[63].vector.z = -0.0098;
    Sensors_Array[14].sensor_taxels[64].vector.x = 0.007295;
    Sensors_Array[14].sensor_taxels[64].vector.y = -0.033954;
    Sensors_Array[14].sensor_taxels[64].vector.z = -0.0098;
    Sensors_Array[14].sensor_taxels[65].vector.x = 0.015804;
    Sensors_Array[14].sensor_taxels[65].vector.y = -0.033954;
    Sensors_Array[14].sensor_taxels[65].vector.z = -0.0098;
    Sensors_Array[14].sensor_taxels[66].vector.x = 0.024313;
    Sensors_Array[14].sensor_taxels[66].vector.y = -0.033954;
    Sensors_Array[14].sensor_taxels[66].vector.z = -0.0098;
    Sensors_Array[14].sensor_taxels[67].vector.x = 0.032822;
    Sensors_Array[14].sensor_taxels[67].vector.y = -0.033954;
    Sensors_Array[14].sensor_taxels[67].vector.z = -0.0098;
    Sensors_Array[14].sensor_taxels[68].vector.x = 0.041331;
    Sensors_Array[14].sensor_taxels[68].vector.y = -0.033954;
    Sensors_Array[14].sensor_taxels[68].vector.z = -0.0098;
    Sensors_Array[14].sensor_taxels[69].vector.x = 0.04984;
    Sensors_Array[14].sensor_taxels[69].vector.y = -0.033954;
    Sensors_Array[14].sensor_taxels[69].vector.z = -0.0098;
    Sensors_Array[14].sensor_taxels[70].vector.x = 0.058349;
    Sensors_Array[14].sensor_taxels[70].vector.y = -0.033954;
    Sensors_Array[14].sensor_taxels[70].vector.z = -0.0098;
    Sensors_Array[14].sensor_taxels[71].vector.x = 0.066858;
    Sensors_Array[14].sensor_taxels[71].vector.y = -0.033954;
    Sensors_Array[14].sensor_taxels[71].vector.z = -0.0098;
    //Row7
    Sensors_Array[14].sensor_taxels[72].vector.x = -0.026741;
    Sensors_Array[14].sensor_taxels[72].vector.y = -0.040611;
    Sensors_Array[14].sensor_taxels[72].vector.z = -0.0098;
    Sensors_Array[14].sensor_taxels[73].vector.x = -0.018232;
    Sensors_Array[14].sensor_taxels[73].vector.y = -0.040611;
    Sensors_Array[14].sensor_taxels[73].vector.z = -0.0098;
    Sensors_Array[14].sensor_taxels[74].vector.x = -0.009723;
    Sensors_Array[14].sensor_taxels[74].vector.y = -0.040611;
    Sensors_Array[14].sensor_taxels[74].vector.z = -0.0098;
    Sensors_Array[14].sensor_taxels[75].vector.x = -0.001214;
    Sensors_Array[14].sensor_taxels[75].vector.y = -0.040611;
    Sensors_Array[14].sensor_taxels[75].vector.z = -0.0098;
    Sensors_Array[14].sensor_taxels[76].vector.x = 0.007295;
    Sensors_Array[14].sensor_taxels[76].vector.y = -0.040611;
    Sensors_Array[14].sensor_taxels[76].vector.z = -0.0098;
    Sensors_Array[14].sensor_taxels[77].vector.x = 0.015804;
    Sensors_Array[14].sensor_taxels[77].vector.y = -0.040611;
    Sensors_Array[14].sensor_taxels[77].vector.z = -0.0098;
    Sensors_Array[14].sensor_taxels[78].vector.x = 0.024313;
    Sensors_Array[14].sensor_taxels[78].vector.y = -0.040611;
    Sensors_Array[14].sensor_taxels[78].vector.z = -0.0098;
    Sensors_Array[14].sensor_taxels[79].vector.x = 0.032822;
    Sensors_Array[14].sensor_taxels[79].vector.y = -0.040611;
    Sensors_Array[14].sensor_taxels[79].vector.z = -0.0098;
    Sensors_Array[14].sensor_taxels[80].vector.x = 0.041331;
    Sensors_Array[14].sensor_taxels[80].vector.y = -0.040611;
    Sensors_Array[14].sensor_taxels[80].vector.z = -0.0098;
    Sensors_Array[14].sensor_taxels[81].vector.x = 0.04984;
    Sensors_Array[14].sensor_taxels[81].vector.y = -0.040611;
    Sensors_Array[14].sensor_taxels[81].vector.z = -0.0098;
    Sensors_Array[14].sensor_taxels[82].vector.x = 0.058349;
    Sensors_Array[14].sensor_taxels[82].vector.y = -0.040611;
    Sensors_Array[14].sensor_taxels[82].vector.z = -0.0098;
    Sensors_Array[14].sensor_taxels[83].vector.x = 0.066858;
    Sensors_Array[14].sensor_taxels[83].vector.y = -0.040611;
    Sensors_Array[14].sensor_taxels[83].vector.z = -0.0098;
    //Row8
    Sensors_Array[14].sensor_taxels[84].vector.x = -0.026741;
    Sensors_Array[14].sensor_taxels[84].vector.y = -0.047356;
    Sensors_Array[14].sensor_taxels[84].vector.z = -0.0098;
    Sensors_Array[14].sensor_taxels[85].vector.x = -0.018232;
    Sensors_Array[14].sensor_taxels[85].vector.y = -0.047356;
    Sensors_Array[14].sensor_taxels[85].vector.z = -0.0098;
    Sensors_Array[14].sensor_taxels[86].vector.x = -0.009723;
    Sensors_Array[14].sensor_taxels[86].vector.y = -0.047356;
    Sensors_Array[14].sensor_taxels[86].vector.z = -0.0098;
    Sensors_Array[14].sensor_taxels[87].vector.x = -0.001214;
    Sensors_Array[14].sensor_taxels[87].vector.y = -0.047356;
    Sensors_Array[14].sensor_taxels[87].vector.z = -0.0098;
    Sensors_Array[14].sensor_taxels[88].vector.x = 0.007295;
    Sensors_Array[14].sensor_taxels[88].vector.y = -0.047356;
    Sensors_Array[14].sensor_taxels[88].vector.z = -0.0098;
    Sensors_Array[14].sensor_taxels[89].vector.x = 0.015804;
    Sensors_Array[14].sensor_taxels[89].vector.y = -0.047356;
    Sensors_Array[14].sensor_taxels[89].vector.z = -0.0098;
    Sensors_Array[14].sensor_taxels[90].vector.x = 0.024313;
    Sensors_Array[14].sensor_taxels[90].vector.y = -0.047356;
    Sensors_Array[14].sensor_taxels[90].vector.z = -0.0098;
    Sensors_Array[14].sensor_taxels[91].vector.x = 0.032822;
    Sensors_Array[14].sensor_taxels[91].vector.y = -0.047356;
    Sensors_Array[14].sensor_taxels[91].vector.z = -0.0098;
    Sensors_Array[14].sensor_taxels[92].vector.x = 0.041331;
    Sensors_Array[14].sensor_taxels[92].vector.y = -0.047356;
    Sensors_Array[14].sensor_taxels[92].vector.z = -0.0098;
    Sensors_Array[14].sensor_taxels[93].vector.x = 0.04984;
    Sensors_Array[14].sensor_taxels[93].vector.y = -0.047356;
    Sensors_Array[14].sensor_taxels[93].vector.z = -0.0098;
    Sensors_Array[14].sensor_taxels[94].vector.x = 0.058349;
    Sensors_Array[14].sensor_taxels[94].vector.y = -0.047356;
    Sensors_Array[14].sensor_taxels[94].vector.z = -0.0098;
    Sensors_Array[14].sensor_taxels[95].vector.x = 0.066858;
    Sensors_Array[14].sensor_taxels[95].vector.y = -0.047356;
    Sensors_Array[14].sensor_taxels[95].vector.z = -0.0098;
    //Row9
    Sensors_Array[14].sensor_taxels[96].vector.x = -0.026923;
    Sensors_Array[14].sensor_taxels[96].vector.y = -0.054057;
    Sensors_Array[14].sensor_taxels[96].vector.z = -0.0098;
    Sensors_Array[14].sensor_taxels[97].vector.x = -0.018414;
    Sensors_Array[14].sensor_taxels[97].vector.y = -0.054057;
    Sensors_Array[14].sensor_taxels[97].vector.z = -0.0098;
    Sensors_Array[14].sensor_taxels[98].vector.x = -0.009905;
    Sensors_Array[14].sensor_taxels[98].vector.y = -0.054057;
    Sensors_Array[14].sensor_taxels[98].vector.z = -0.0098;
    Sensors_Array[14].sensor_taxels[99].vector.x = -0.001396;
    Sensors_Array[14].sensor_taxels[99].vector.y = -0.054057;
    Sensors_Array[14].sensor_taxels[99].vector.z = -0.0098;
    Sensors_Array[14].sensor_taxels[100].vector.x = 0.007113;
    Sensors_Array[14].sensor_taxels[100].vector.y = -0.054057;
    Sensors_Array[14].sensor_taxels[100].vector.z = -0.0098;
    Sensors_Array[14].sensor_taxels[101].vector.x = 0.015622;
    Sensors_Array[14].sensor_taxels[101].vector.y = -0.054057;
    Sensors_Array[14].sensor_taxels[101].vector.z = -0.0098;
    Sensors_Array[14].sensor_taxels[102].vector.x = 0.024131;
    Sensors_Array[14].sensor_taxels[102].vector.y = -0.054057;
    Sensors_Array[14].sensor_taxels[102].vector.z = -0.0098;
    Sensors_Array[14].sensor_taxels[103].vector.x = 0.03264;
    Sensors_Array[14].sensor_taxels[103].vector.y = -0.054057;
    Sensors_Array[14].sensor_taxels[103].vector.z = -0.0098;
    Sensors_Array[14].sensor_taxels[104].vector.x = 0.041149;
    Sensors_Array[14].sensor_taxels[104].vector.y = -0.054057;
    Sensors_Array[14].sensor_taxels[104].vector.z = -0.0098;
    Sensors_Array[14].sensor_taxels[105].vector.x = 0.049658;
    Sensors_Array[14].sensor_taxels[105].vector.y = -0.054057;
    Sensors_Array[14].sensor_taxels[105].vector.z = -0.0098;
    Sensors_Array[14].sensor_taxels[106].vector.x = 0.058167;
    Sensors_Array[14].sensor_taxels[106].vector.y = -0.054057;
    Sensors_Array[14].sensor_taxels[106].vector.z = -0.0098;
    Sensors_Array[14].sensor_taxels[107].vector.x = 0.066676;
    Sensors_Array[14].sensor_taxels[107].vector.y = -0.054057;
    Sensors_Array[14].sensor_taxels[107].vector.z = -0.0098;
    //Row10
    Sensors_Array[14].sensor_taxels[108].vector.x = -0.026928;
    Sensors_Array[14].sensor_taxels[108].vector.y = -0.060963;
    Sensors_Array[14].sensor_taxels[108].vector.z = -0.0098;
    Sensors_Array[14].sensor_taxels[109].vector.x = -0.018419;
    Sensors_Array[14].sensor_taxels[109].vector.y = -0.060963;
    Sensors_Array[14].sensor_taxels[109].vector.z = -0.0098;
    Sensors_Array[14].sensor_taxels[110].vector.x = -0.00991;
    Sensors_Array[14].sensor_taxels[110].vector.y = -0.060963;
    Sensors_Array[14].sensor_taxels[110].vector.z = -0.0098;
    Sensors_Array[14].sensor_taxels[111].vector.x = 0.049658;
    Sensors_Array[14].sensor_taxels[111].vector.y = -0.060899;
    Sensors_Array[14].sensor_taxels[111].vector.z = -0.0098;
    Sensors_Array[14].sensor_taxels[112].vector.x = 0.058167;
    Sensors_Array[14].sensor_taxels[112].vector.y = -0.060899;
    Sensors_Array[14].sensor_taxels[112].vector.z = -0.0098;
    Sensors_Array[14].sensor_taxels[113].vector.x = 0.066731;
    Sensors_Array[14].sensor_taxels[113].vector.y = -0.060899;
    Sensors_Array[14].sensor_taxels[113].vector.z = -0.0098;
    //Row11
    Sensors_Array[14].sensor_taxels[114].vector.x = -0.018419;
    Sensors_Array[14].sensor_taxels[114].vector.y = -0.068096;
    Sensors_Array[14].sensor_taxels[114].vector.z = -0.0098;
    Sensors_Array[14].sensor_taxels[115].vector.x = -0.00991;
    Sensors_Array[14].sensor_taxels[115].vector.y = -0.068096;
    Sensors_Array[14].sensor_taxels[115].vector.z = -0.0098;
    Sensors_Array[14].sensor_taxels[116].vector.x = 0.049658;
    Sensors_Array[14].sensor_taxels[116].vector.y = -0.067905;
    Sensors_Array[14].sensor_taxels[116].vector.z = -0.0098;
    Sensors_Array[14].sensor_taxels[117].vector.x = 0.058167;
    Sensors_Array[14].sensor_taxels[117].vector.y = -0.067905;
    Sensors_Array[14].sensor_taxels[117].vector.z = -0.0098;
*/
    //PALM

    //UPPER MATRIX
    //Row1
    Sensors_Array[13].sensor_taxels[0].vector.x = -0.04382;
    Sensors_Array[13].sensor_taxels[0].vector.y = -0.083961;
    Sensors_Array[13].sensor_taxels[0].vector.z = -0.0061;
    Sensors_Array[13].sensor_taxels[1].vector.x = -0.04382;
    Sensors_Array[13].sensor_taxels[1].vector.y = -0.079524;
    Sensors_Array[13].sensor_taxels[1].vector.z = -0.0061;
    Sensors_Array[13].sensor_taxels[2].vector.x = -0.04382;
    Sensors_Array[13].sensor_taxels[2].vector.y = -0.075087;
    Sensors_Array[13].sensor_taxels[2].vector.z = -0.0061;
    Sensors_Array[13].sensor_taxels[3].vector.x = -0.04382;
    Sensors_Array[13].sensor_taxels[3].vector.y =-0.07065;
    Sensors_Array[13].sensor_taxels[3].vector.z = -0.0061;
    Sensors_Array[13].sensor_taxels[4].vector.x = -0.04382;
    Sensors_Array[13].sensor_taxels[4].vector.y = -0.066213;
    Sensors_Array[13].sensor_taxels[4].vector.z = -0.0061;
    Sensors_Array[13].sensor_taxels[5].vector.x = -0.04382;
    Sensors_Array[13].sensor_taxels[5].vector.y = -0.061776;
    Sensors_Array[13].sensor_taxels[5].vector.z = -0.0061;
    //Row2
    Sensors_Array[13].sensor_taxels[6].vector.x = -0.036508;
    Sensors_Array[13].sensor_taxels[6].vector.y = -0.083961;
    Sensors_Array[13].sensor_taxels[6].vector.z = -0.0061;
    Sensors_Array[13].sensor_taxels[7].vector.x = -0.036508;
    Sensors_Array[13].sensor_taxels[7].vector.y = -0.079524;
    Sensors_Array[13].sensor_taxels[7].vector.z = -0.0061;
    Sensors_Array[13].sensor_taxels[8].vector.x = -0.036508;
    Sensors_Array[13].sensor_taxels[8].vector.y = -0.075087;
    Sensors_Array[13].sensor_taxels[8].vector.z = -0.0061;
    Sensors_Array[13].sensor_taxels[9].vector.x = -0.036508;
    Sensors_Array[13].sensor_taxels[9].vector.y = -0.07065;
    Sensors_Array[13].sensor_taxels[9].vector.z = -0.0061;
    Sensors_Array[13].sensor_taxels[10].vector.x = -0.036508;
    Sensors_Array[13].sensor_taxels[10].vector.y = -0.066213;
    Sensors_Array[13].sensor_taxels[10].vector.z = -0.0061;
    Sensors_Array[13].sensor_taxels[11].vector.x = -0.036508;
    Sensors_Array[13].sensor_taxels[11].vector.y = -0.061776;
    Sensors_Array[13].sensor_taxels[11].vector.z = -0.0061;
    //Row3
    Sensors_Array[13].sensor_taxels[12].vector.x = -0.029196;
    Sensors_Array[13].sensor_taxels[12].vector.y = -0.083961;
    Sensors_Array[13].sensor_taxels[12].vector.z = -0.0061;
    Sensors_Array[13].sensor_taxels[13].vector.x = -0.029196;
    Sensors_Array[13].sensor_taxels[13].vector.y = -0.079524;
    Sensors_Array[13].sensor_taxels[13].vector.z = -0.0061;
    Sensors_Array[13].sensor_taxels[14].vector.x = -0.029196;
    Sensors_Array[13].sensor_taxels[14].vector.y = -0.075087;
    Sensors_Array[13].sensor_taxels[14].vector.z = -0.0061;
    Sensors_Array[13].sensor_taxels[15].vector.x = -0.029196;
    Sensors_Array[13].sensor_taxels[15].vector.y = -0.07065;
    Sensors_Array[13].sensor_taxels[15].vector.z = -0.0061;
    Sensors_Array[13].sensor_taxels[16].vector.x = -0.029196;
    Sensors_Array[13].sensor_taxels[16].vector.y = -0.066213;
    Sensors_Array[13].sensor_taxels[16].vector.z = -0.0061;
    Sensors_Array[13].sensor_taxels[17].vector.x = -0.029196;
    Sensors_Array[13].sensor_taxels[17].vector.y = -0.061776;
    Sensors_Array[13].sensor_taxels[17].vector.z = -0.0061;
    //Row4
    Sensors_Array[13].sensor_taxels[18].vector.x = -0.021884;
    Sensors_Array[13].sensor_taxels[18].vector.y = -0.083961;
    Sensors_Array[13].sensor_taxels[18].vector.z = -0.0061;
    Sensors_Array[13].sensor_taxels[19].vector.x = -0.021884;
    Sensors_Array[13].sensor_taxels[19].vector.y = -0.079524;
    Sensors_Array[13].sensor_taxels[19].vector.z = -0.0061;
    Sensors_Array[13].sensor_taxels[20].vector.x = -0.021884;
    Sensors_Array[13].sensor_taxels[20].vector.y = -0.075087;
    Sensors_Array[13].sensor_taxels[20].vector.z = -0.0061;
    Sensors_Array[13].sensor_taxels[21].vector.x = -0.021884;
    Sensors_Array[13].sensor_taxels[21].vector.y = -0.07065;
    Sensors_Array[13].sensor_taxels[21].vector.z = -0.0061;
    Sensors_Array[13].sensor_taxels[22].vector.x = -0.021884;
    Sensors_Array[13].sensor_taxels[22].vector.y = -0.066213;
    Sensors_Array[13].sensor_taxels[22].vector.z = -0.0061;
    Sensors_Array[13].sensor_taxels[23].vector.x = -0.021884;
    Sensors_Array[13].sensor_taxels[23].vector.y = -0.061776;
    Sensors_Array[13].sensor_taxels[23].vector.z = -0.0061;
    //Row5
    //Row4
    Sensors_Array[13].sensor_taxels[24].vector.x = -0.014572;
    Sensors_Array[13].sensor_taxels[24].vector.y = -0.083961;
    Sensors_Array[13].sensor_taxels[24].vector.z = -0.0061;
    Sensors_Array[13].sensor_taxels[25].vector.x = -0.014572;
    Sensors_Array[13].sensor_taxels[25].vector.y = -0.079524;
    Sensors_Array[13].sensor_taxels[25].vector.z = -0.0061;
    Sensors_Array[13].sensor_taxels[26].vector.x = -0.014572;
    Sensors_Array[13].sensor_taxels[26].vector.y = -0.075087;
    Sensors_Array[13].sensor_taxels[26].vector.z = -0.0061;
    Sensors_Array[13].sensor_taxels[27].vector.x = -0.014572;
    Sensors_Array[13].sensor_taxels[27].vector.y = -0.07065;
    Sensors_Array[13].sensor_taxels[27].vector.z = -0.0061;
    Sensors_Array[13].sensor_taxels[28].vector.x = -0.014572;
    Sensors_Array[13].sensor_taxels[28].vector.y = -0.066213;
    Sensors_Array[13].sensor_taxels[28].vector.z = -0.0061;
    Sensors_Array[13].sensor_taxels[29].vector.x = -0.014572;
    Sensors_Array[13].sensor_taxels[29].vector.y = -0.061776;
    Sensors_Array[13].sensor_taxels[29].vector.z = -0.0061;
    //Row6
    Sensors_Array[13].sensor_taxels[30].vector.x = -0.007259;
    Sensors_Array[13].sensor_taxels[30].vector.y = -0.083961;
    Sensors_Array[13].sensor_taxels[30].vector.z = -0.0061;
    Sensors_Array[13].sensor_taxels[31].vector.x = -0.007259;
    Sensors_Array[13].sensor_taxels[31].vector.y = -0.079524;
    Sensors_Array[13].sensor_taxels[31].vector.z = -0.0061;
    Sensors_Array[13].sensor_taxels[32].vector.x = -0.007259;
    Sensors_Array[13].sensor_taxels[32].vector.y = -0.075087;
    Sensors_Array[13].sensor_taxels[32].vector.z = -0.0061;
    Sensors_Array[13].sensor_taxels[33].vector.x = -0.007259;
    Sensors_Array[13].sensor_taxels[33].vector.y = -0.07065;
    Sensors_Array[13].sensor_taxels[33].vector.z = -0.0061;
    Sensors_Array[13].sensor_taxels[34].vector.x = -0.007259;
    Sensors_Array[13].sensor_taxels[34].vector.y = -0.066213;
    Sensors_Array[13].sensor_taxels[34].vector.z = -0.0061;
    Sensors_Array[13].sensor_taxels[35].vector.x = -0.007259;
    Sensors_Array[13].sensor_taxels[35].vector.y = -0.061776;
    Sensors_Array[13].sensor_taxels[35].vector.z = -0.0061;
    //Row7
    Sensors_Array[13].sensor_taxels[36].vector.x = 0.000153;
    Sensors_Array[13].sensor_taxels[36].vector.y = -0.083961;
    Sensors_Array[13].sensor_taxels[36].vector.z = -0.0061;
    Sensors_Array[13].sensor_taxels[37].vector.x = 0.000153;
    Sensors_Array[13].sensor_taxels[37].vector.y = -0.079524;
    Sensors_Array[13].sensor_taxels[37].vector.z = -0.0061;
    Sensors_Array[13].sensor_taxels[38].vector.x = 0.000153;
    Sensors_Array[13].sensor_taxels[38].vector.y = -0.075087;
    Sensors_Array[13].sensor_taxels[38].vector.z = -0.0061;
    Sensors_Array[13].sensor_taxels[39].vector.x = 0.000153;
    Sensors_Array[13].sensor_taxels[39].vector.y = -0.07065;
    Sensors_Array[13].sensor_taxels[39].vector.z = -0.0061;
    Sensors_Array[13].sensor_taxels[40].vector.x = 0.000153;
    Sensors_Array[13].sensor_taxels[40].vector.y = -0.066213;
    Sensors_Array[13].sensor_taxels[40].vector.z = -0.0061;
    Sensors_Array[13].sensor_taxels[41].vector.x = 0.000153;
    Sensors_Array[13].sensor_taxels[41].vector.y = -0.061776;
    Sensors_Array[13].sensor_taxels[41].vector.z = -0.0061;
    //Row8
    Sensors_Array[13].sensor_taxels[42].vector.x = 0.007465;
    Sensors_Array[13].sensor_taxels[42].vector.y = -0.083961;
    Sensors_Array[13].sensor_taxels[42].vector.z = -0.0061;
    Sensors_Array[13].sensor_taxels[43].vector.x = 0.007465;
    Sensors_Array[13].sensor_taxels[43].vector.y = -0.079524;
    Sensors_Array[13].sensor_taxels[43].vector.z = -0.0061;
    Sensors_Array[13].sensor_taxels[44].vector.x = 0.007465;
    Sensors_Array[13].sensor_taxels[44].vector.y = -0.075087;
    Sensors_Array[13].sensor_taxels[44].vector.z = -0.0061;
    Sensors_Array[13].sensor_taxels[45].vector.x = 0.007465;
    Sensors_Array[13].sensor_taxels[45].vector.y = -0.07065;
    Sensors_Array[13].sensor_taxels[45].vector.z = -0.0061;
    Sensors_Array[13].sensor_taxels[46].vector.x = 0.007465;
    Sensors_Array[13].sensor_taxels[46].vector.y = -0.066213;
    Sensors_Array[13].sensor_taxels[46].vector.z = -0.0061;
    Sensors_Array[13].sensor_taxels[47].vector.x = 0.007465;
    Sensors_Array[13].sensor_taxels[47].vector.y = -0.061776;
    Sensors_Array[13].sensor_taxels[47].vector.z = -0.0061;
    //Row9
    Sensors_Array[13].sensor_taxels[48].vector.x = 0.014777;
    Sensors_Array[13].sensor_taxels[48].vector.y = -0.083961;
    Sensors_Array[13].sensor_taxels[48].vector.z = -0.0061;
    Sensors_Array[13].sensor_taxels[49].vector.x = 0.014777;
    Sensors_Array[13].sensor_taxels[49].vector.y = -0.079524;
    Sensors_Array[13].sensor_taxels[49].vector.z = -0.0061;
    Sensors_Array[13].sensor_taxels[50].vector.x = 0.014777;
    Sensors_Array[13].sensor_taxels[50].vector.y = -0.075087;
    Sensors_Array[13].sensor_taxels[50].vector.z = -0.0061;
    Sensors_Array[13].sensor_taxels[51].vector.x = 0.014777;
    Sensors_Array[13].sensor_taxels[51].vector.y = -0.07065;
    Sensors_Array[13].sensor_taxels[51].vector.z = -0.0061;
    Sensors_Array[13].sensor_taxels[52].vector.x = 0.014777;
    Sensors_Array[13].sensor_taxels[52].vector.y = -0.066213;
    Sensors_Array[13].sensor_taxels[52].vector.z = -0.0061;
    Sensors_Array[13].sensor_taxels[53].vector.x = 0.014777;
    Sensors_Array[13].sensor_taxels[53].vector.y = -0.061776;
    Sensors_Array[13].sensor_taxels[53].vector.z = -0.0061;
    //Row10
    Sensors_Array[13].sensor_taxels[54].vector.x = 0.022089;
    Sensors_Array[13].sensor_taxels[54].vector.y = -0.083961;
    Sensors_Array[13].sensor_taxels[54].vector.z = -0.0061;
    Sensors_Array[13].sensor_taxels[55].vector.x = 0.022089;
    Sensors_Array[13].sensor_taxels[55].vector.y = -0.079524;
    Sensors_Array[13].sensor_taxels[55].vector.z = -0.0061;
    Sensors_Array[13].sensor_taxels[56].vector.x = 0.022089;
    Sensors_Array[13].sensor_taxels[56].vector.y = -0.075087;
    Sensors_Array[13].sensor_taxels[56].vector.z = -0.0061;
    Sensors_Array[13].sensor_taxels[57].vector.x = 0.022089;
    Sensors_Array[13].sensor_taxels[57].vector.y = -0.07065;
    Sensors_Array[13].sensor_taxels[57].vector.z = -0.0061;
    Sensors_Array[13].sensor_taxels[58].vector.x = 0.022089;
    Sensors_Array[13].sensor_taxels[58].vector.y = -0.066213;
    Sensors_Array[13].sensor_taxels[58].vector.z = -0.0061;
    Sensors_Array[13].sensor_taxels[59].vector.x = 0.022089;
    Sensors_Array[13].sensor_taxels[59].vector.y = -0.061776;
    Sensors_Array[13].sensor_taxels[59].vector.z = -0.0061;
    //Row11
    Sensors_Array[13].sensor_taxels[60].vector.x = 0.029401;
    Sensors_Array[13].sensor_taxels[60].vector.y = -0.083961;
    Sensors_Array[13].sensor_taxels[60].vector.z = -0.0061;
    Sensors_Array[13].sensor_taxels[61].vector.x = 0.029401;
    Sensors_Array[13].sensor_taxels[61].vector.y = -0.079524;
    Sensors_Array[13].sensor_taxels[61].vector.z = -0.0061;
    Sensors_Array[13].sensor_taxels[62].vector.x = 0.029401;
    Sensors_Array[13].sensor_taxels[62].vector.y = -0.075087;
    Sensors_Array[13].sensor_taxels[62].vector.z = -0.0061;
    Sensors_Array[13].sensor_taxels[63].vector.x = 0.029401;
    Sensors_Array[13].sensor_taxels[63].vector.y = -0.07065;
    Sensors_Array[13].sensor_taxels[63].vector.z = -0.0061;
    Sensors_Array[13].sensor_taxels[64].vector.x = 0.029401;
    Sensors_Array[13].sensor_taxels[64].vector.y = -0.066213;
    Sensors_Array[13].sensor_taxels[64].vector.z = -0.0061;
    Sensors_Array[13].sensor_taxels[65].vector.x = 0.029401;
    Sensors_Array[13].sensor_taxels[65].vector.y = -0.061776;
    Sensors_Array[13].sensor_taxels[65].vector.z = -0.0061;
    //LOWER MATRIX
    //Row1
    Sensors_Array[13].sensor_taxels[66].vector.x = -0.044614;
    Sensors_Array[13].sensor_taxels[66].vector.y = -0.053013;
    Sensors_Array[13].sensor_taxels[66].vector.z = -0.0061;
    Sensors_Array[13].sensor_taxels[67].vector.x = -0.044233;
    Sensors_Array[13].sensor_taxels[67].vector.y = -0.04774;
    Sensors_Array[13].sensor_taxels[67].vector.z = -0.0061;
    Sensors_Array[13].sensor_taxels[68].vector.x = -0.043878;
    Sensors_Array[13].sensor_taxels[68].vector.y = -0.042467;
    Sensors_Array[13].sensor_taxels[68].vector.z = -0.0061;
    Sensors_Array[13].sensor_taxels[69].vector.x = -0.04339;
    Sensors_Array[13].sensor_taxels[69].vector.y = -0.037194;
    Sensors_Array[13].sensor_taxels[69].vector.z = -0.0061;
    Sensors_Array[13].sensor_taxels[70].vector.x = -0.042969;
    Sensors_Array[13].sensor_taxels[70].vector.y = -0.031921;
    Sensors_Array[13].sensor_taxels[70].vector.z = -0.0061;
    Sensors_Array[13].sensor_taxels[71].vector.x = -0.042455;
    Sensors_Array[13].sensor_taxels[71].vector.y = -0.026648;
    Sensors_Array[13].sensor_taxels[71].vector.z = -0.0061;
    //Row2
    Sensors_Array[13].sensor_taxels[72].vector.x = -0.038391;
    Sensors_Array[13].sensor_taxels[72].vector.y = -0.053013;
    Sensors_Array[13].sensor_taxels[72].vector.z = -0.0061;
    Sensors_Array[13].sensor_taxels[73].vector.x = -0.038137;
    Sensors_Array[13].sensor_taxels[73].vector.y = -0.04774;
    Sensors_Array[13].sensor_taxels[73].vector.z = -0.0061;
    Sensors_Array[13].sensor_taxels[74].vector.x = -0.037909;
    Sensors_Array[13].sensor_taxels[74].vector.y = -0.042467;
    Sensors_Array[13].sensor_taxels[74].vector.z = -0.0061;
    Sensors_Array[13].sensor_taxels[75].vector.x = -0.037548;
    Sensors_Array[13].sensor_taxels[75].vector.y = -0.037194;
    Sensors_Array[13].sensor_taxels[75].vector.z = -0.0061;
    Sensors_Array[13].sensor_taxels[76].vector.x = -0.037254;
    Sensors_Array[13].sensor_taxels[76].vector.y = -0.031921;
    Sensors_Array[13].sensor_taxels[76].vector.z = -0.0061;
    Sensors_Array[13].sensor_taxels[77].vector.x = -0.036867;
    Sensors_Array[13].sensor_taxels[77].vector.y = -0.026648;
    Sensors_Array[13].sensor_taxels[77].vector.z = -0.0061;
    //Row3
    Sensors_Array[13].sensor_taxels[78].vector.x = -0.032168;
    Sensors_Array[13].sensor_taxels[78].vector.y = -0.053013;
    Sensors_Array[13].sensor_taxels[78].vector.z = -0.0061;
    Sensors_Array[13].sensor_taxels[79].vector.x = -0.032041;
    Sensors_Array[13].sensor_taxels[79].vector.y = -0.04774;
    Sensors_Array[13].sensor_taxels[79].vector.z = -0.0061;
    Sensors_Array[13].sensor_taxels[80].vector.x = -0.03194;
    Sensors_Array[13].sensor_taxels[80].vector.y = -0.042467;
    Sensors_Array[13].sensor_taxels[80].vector.z = -0.0061;
    Sensors_Array[13].sensor_taxels[81].vector.x = -0.031706;
    Sensors_Array[13].sensor_taxels[81].vector.y = -0.037194;
    Sensors_Array[13].sensor_taxels[81].vector.z = -0.0061;
    Sensors_Array[13].sensor_taxels[82].vector.x = -0.031539;
    Sensors_Array[13].sensor_taxels[82].vector.y = -0.031921;
    Sensors_Array[13].sensor_taxels[82].vector.z = -0.0061;
    Sensors_Array[13].sensor_taxels[83].vector.x = -0.031279;
    Sensors_Array[13].sensor_taxels[83].vector.y = -0.026648;
    Sensors_Array[13].sensor_taxels[83].vector.z = -0.0061;
    //Row4
    Sensors_Array[13].sensor_taxels[84].vector.x = -0.025945;
    Sensors_Array[13].sensor_taxels[84].vector.y = -0.053013;
    Sensors_Array[13].sensor_taxels[84].vector.z = -0.0061;
    Sensors_Array[13].sensor_taxels[85].vector.x = -0.025945;
    Sensors_Array[13].sensor_taxels[85].vector.y = -0.04774;
    Sensors_Array[13].sensor_taxels[85].vector.z = -0.0061;
    Sensors_Array[13].sensor_taxels[86].vector.x = -0.025971;
    Sensors_Array[13].sensor_taxels[86].vector.y = -0.042467;
    Sensors_Array[13].sensor_taxels[86].vector.z = -0.0061;
    Sensors_Array[13].sensor_taxels[87].vector.x = -0.025864;
    Sensors_Array[13].sensor_taxels[87].vector.y = -0.037194;
    Sensors_Array[13].sensor_taxels[87].vector.z = -0.0061;
    Sensors_Array[13].sensor_taxels[88].vector.x = -0.025824;
    Sensors_Array[13].sensor_taxels[88].vector.y = -0.031921;
    Sensors_Array[13].sensor_taxels[88].vector.z = -0.0061;
    Sensors_Array[13].sensor_taxels[89].vector.x = -0.025691;
    Sensors_Array[13].sensor_taxels[89].vector.y = -0.026648;
    Sensors_Array[13].sensor_taxels[89].vector.z = -0.0061;
    //Row5
    Sensors_Array[13].sensor_taxels[90].vector.x = -0.019722;
    Sensors_Array[13].sensor_taxels[90].vector.y = -0.053013;
    Sensors_Array[13].sensor_taxels[90].vector.z = -0.0061;
    Sensors_Array[13].sensor_taxels[91].vector.x = -0.019849;
    Sensors_Array[13].sensor_taxels[91].vector.y = -0.04774;
    Sensors_Array[13].sensor_taxels[91].vector.z = -0.0061;
    Sensors_Array[13].sensor_taxels[92].vector.x = -0.020002;
    Sensors_Array[13].sensor_taxels[92].vector.y = -0.042467;
    Sensors_Array[13].sensor_taxels[92].vector.z = -0.0061;
    Sensors_Array[13].sensor_taxels[93].vector.x = -0.020002;
    Sensors_Array[13].sensor_taxels[93].vector.y = -0.037194;
    Sensors_Array[13].sensor_taxels[93].vector.z = -0.0061;
    Sensors_Array[13].sensor_taxels[94].vector.x = -0.020109;
    Sensors_Array[13].sensor_taxels[94].vector.y = -0.031921;
    Sensors_Array[13].sensor_taxels[94].vector.z = -0.0061;
    Sensors_Array[13].sensor_taxels[95].vector.x = -0.020103;
    Sensors_Array[13].sensor_taxels[95].vector.y = -0.026648;
    Sensors_Array[13].sensor_taxels[95].vector.z = -0.0061;
    //Row6
    Sensors_Array[13].sensor_taxels[96].vector.x = -0.013499;
    Sensors_Array[13].sensor_taxels[96].vector.y = -0.053013;
    Sensors_Array[13].sensor_taxels[96].vector.z = -0.0061;
    Sensors_Array[13].sensor_taxels[97].vector.x = -0.013753;
    Sensors_Array[13].sensor_taxels[97].vector.y = -0.04774;
    Sensors_Array[13].sensor_taxels[97].vector.z = -0.0061;
    Sensors_Array[13].sensor_taxels[98].vector.x = -0.014033;
    Sensors_Array[13].sensor_taxels[98].vector.y = -0.042467;
    Sensors_Array[13].sensor_taxels[98].vector.z = -0.0061;
    Sensors_Array[13].sensor_taxels[99].vector.x = -0.014180;
    Sensors_Array[13].sensor_taxels[99].vector.y = -0.037194;
    Sensors_Array[13].sensor_taxels[99].vector.z = -0.0061;
    Sensors_Array[13].sensor_taxels[100].vector.x = -0.020109;
    Sensors_Array[13].sensor_taxels[100].vector.y = -0.031921;
    Sensors_Array[13].sensor_taxels[100].vector.z = -0.0061;
    Sensors_Array[13].sensor_taxels[101].vector.x = -0.014394;
    Sensors_Array[13].sensor_taxels[101].vector.y = -0.026648;
    Sensors_Array[13].sensor_taxels[101].vector.z = -0.0061;
    //Row7
    Sensors_Array[13].sensor_taxels[102].vector.x = -0.007221;
    Sensors_Array[13].sensor_taxels[102].vector.y = -0.053013;
    Sensors_Array[13].sensor_taxels[102].vector.z = -0.0061;
    Sensors_Array[13].sensor_taxels[103].vector.x = -0.007657;
    Sensors_Array[13].sensor_taxels[103].vector.y = -0.04774;
    Sensors_Array[13].sensor_taxels[103].vector.z = -0.0061;
    Sensors_Array[13].sensor_taxels[104].vector.x = -0.008064;
    Sensors_Array[13].sensor_taxels[104].vector.y = -0.042467;
    Sensors_Array[13].sensor_taxels[104].vector.z = -0.0061;
    Sensors_Array[13].sensor_taxels[105].vector.x = -0.008338;
    Sensors_Array[13].sensor_taxels[105].vector.y = -0.037194;
    Sensors_Array[13].sensor_taxels[105].vector.z = -0.0061;
    Sensors_Array[13].sensor_taxels[106].vector.x = -0.008679;
    Sensors_Array[13].sensor_taxels[106].vector.y = -0.031921;
    Sensors_Array[13].sensor_taxels[106].vector.z = -0.0061;
    Sensors_Array[13].sensor_taxels[107].vector.x = -0.008927;
    Sensors_Array[13].sensor_taxels[107].vector.y = -0.026648;
    Sensors_Array[13].sensor_taxels[107].vector.z = -0.0061;
    //UPPER SMALL MATRIX
    Sensors_Array[13].sensor_taxels[108].vector.x = 0.04043;
    Sensors_Array[13].sensor_taxels[108].vector.y = -0.085037;
    Sensors_Array[13].sensor_taxels[108].vector.z = -0.0061;
    Sensors_Array[13].sensor_taxels[109].vector.x = 0.04043;
    Sensors_Array[13].sensor_taxels[109].vector.y = -0.080719;
    Sensors_Array[13].sensor_taxels[109].vector.z = -0.0061;
    Sensors_Array[13].sensor_taxels[110].vector.x = 0.045611;
    Sensors_Array[13].sensor_taxels[110].vector.y = -0.085037;
    Sensors_Array[13].sensor_taxels[110].vector.z = -0.0061;
    Sensors_Array[13].sensor_taxels[111].vector.x = 0.045611;
    Sensors_Array[13].sensor_taxels[111].vector.y = -0.080719;
    Sensors_Array[13].sensor_taxels[111].vector.z = -0.0061;
    //LOWER SMALL MATRIX
    Sensors_Array[13].sensor_taxels[112].vector.x = 0.04043;
    Sensors_Array[13].sensor_taxels[112].vector.y = -0.06359;
    Sensors_Array[13].sensor_taxels[112].vector.z = -0.0061;
    Sensors_Array[13].sensor_taxels[113].vector.x = 0.04043;
    Sensors_Array[13].sensor_taxels[113].vector.y = -0.059201;
    Sensors_Array[13].sensor_taxels[113].vector.z = -0.0061;
    Sensors_Array[13].sensor_taxels[114].vector.x = 0.045611;
    Sensors_Array[13].sensor_taxels[114].vector.y = -0.06359;
    Sensors_Array[13].sensor_taxels[114].vector.z = -0.0061;
    Sensors_Array[13].sensor_taxels[115].vector.x = 0.045611;
    Sensors_Array[13].sensor_taxels[115].vector.y = -0.059201;
    Sensors_Array[13].sensor_taxels[115].vector.z = -0.0061;
    //LOWER MATRIX BOTTOM LINE
    Sensors_Array[13].sensor_taxels[116].vector.x = -0.037374;
    Sensors_Array[13].sensor_taxels[116].vector.y = -0.021375;
    Sensors_Array[13].sensor_taxels[116].vector.z = -0.0061;
    Sensors_Array[13].sensor_taxels[117].vector.x = -0.031459;
    Sensors_Array[13].sensor_taxels[117].vector.y = -0.021375;
    Sensors_Array[13].sensor_taxels[117].vector.z = -0.0061;
    Sensors_Array[13].sensor_taxels[118].vector.x = -0.025544;
    Sensors_Array[13].sensor_taxels[118].vector.y = -0.021375;
    Sensors_Array[13].sensor_taxels[118].vector.z = -0.0061;
    Sensors_Array[13].sensor_taxels[119].vector.x = -0.019629;
    Sensors_Array[13].sensor_taxels[119].vector.y = -0.021375;
    Sensors_Array[13].sensor_taxels[119].vector.z = -0.0061;
    Sensors_Array[13].sensor_taxels[120].vector.x = -0.013713;
    Sensors_Array[13].sensor_taxels[120].vector.y = -0.021375;
    Sensors_Array[13].sensor_taxels[120].vector.z = -0.0061;






    //Index Proximal Front

    //Row1
    Sensors_Array[0].sensor_taxels[0].vector.x = -0.024275;
    Sensors_Array[0].sensor_taxels[0].vector.y = 0.008283;
    Sensors_Array[0].sensor_taxels[0].vector.z = 0.012385;
    Sensors_Array[0].sensor_taxels[1].vector.x = -0.01934;
    Sensors_Array[0].sensor_taxels[1].vector.y = 0.008283;
    Sensors_Array[0].sensor_taxels[1].vector.z = 0.011953;
    Sensors_Array[0].sensor_taxels[2].vector.x = -0.014452;
    Sensors_Array[0].sensor_taxels[2].vector.y = 0.008283;
    Sensors_Array[0].sensor_taxels[2].vector.z = 0.011525;
    Sensors_Array[0].sensor_taxels[3].vector.x = -0.00773;
    Sensors_Array[0].sensor_taxels[3].vector.y = 0.008283;
    Sensors_Array[0].sensor_taxels[3].vector.z = 0.010937;
    Sensors_Array[0].sensor_taxels[4].vector.x = -0.002418;
    Sensors_Array[0].sensor_taxels[4].vector.y = 0.008283;
    Sensors_Array[0].sensor_taxels[4].vector.z = 0.010521;
    Sensors_Array[0].sensor_taxels[5].vector.x = 0.002881;
    Sensors_Array[0].sensor_taxels[5].vector.y = 0.008283;
    Sensors_Array[0].sensor_taxels[5].vector.z = 0.010008;
    Sensors_Array[0].sensor_taxels[6].vector.x = 0.008014;
    Sensors_Array[0].sensor_taxels[6].vector.y = 0.008283;
    Sensors_Array[0].sensor_taxels[6].vector.z = 0.009564;
    Sensors_Array[0].sensor_taxels[7].vector.x = 0.013002;
    Sensors_Array[0].sensor_taxels[7].vector.y = 0.008283;
    Sensors_Array[0].sensor_taxels[7].vector.z = 0.009122;
    Sensors_Array[0].sensor_taxels[8].vector.x = 0.019755;
    Sensors_Array[0].sensor_taxels[8].vector.y = 0.008283;
    Sensors_Array[0].sensor_taxels[8].vector.z = 0.008553;
    Sensors_Array[0].sensor_taxels[9].vector.x = 0.024652;
    Sensors_Array[0].sensor_taxels[9].vector.y = 0.008283;
    Sensors_Array[0].sensor_taxels[9].vector.z = 0.008093;
    Sensors_Array[0].sensor_taxels[10].vector.x = 0.029593;
    Sensors_Array[0].sensor_taxels[10].vector.y = 0.008283;
    Sensors_Array[0].sensor_taxels[10].vector.z = 0.007671;
    //Row2
    Sensors_Array[0].sensor_taxels[11].vector.x = -0.02393;
    Sensors_Array[0].sensor_taxels[11].vector.y = 0.008418;
    Sensors_Array[0].sensor_taxels[11].vector.z = 0.016317;
    Sensors_Array[0].sensor_taxels[12].vector.x = -0.018996;
    Sensors_Array[0].sensor_taxels[12].vector.y = 0.008418;
    Sensors_Array[0].sensor_taxels[12].vector.z = 0.015885;
    Sensors_Array[0].sensor_taxels[13].vector.x = -0.014108;
    Sensors_Array[0].sensor_taxels[13].vector.y = 0.008418;
    Sensors_Array[0].sensor_taxels[13].vector.z = 0.015457;
    Sensors_Array[0].sensor_taxels[14].vector.x = -0.007386;
    Sensors_Array[0].sensor_taxels[14].vector.y = 0.008418;
    Sensors_Array[0].sensor_taxels[14].vector.z = 0.014869;
    Sensors_Array[0].sensor_taxels[15].vector.x = -0.002074;
    Sensors_Array[0].sensor_taxels[15].vector.y = 0.008418;
    Sensors_Array[0].sensor_taxels[15].vector.z = 0.014454;
    Sensors_Array[0].sensor_taxels[16].vector.x = 0.003225;
    Sensors_Array[0].sensor_taxels[16].vector.y = 0.008418;
    Sensors_Array[0].sensor_taxels[16].vector.z = 0.013941;
    Sensors_Array[0].sensor_taxels[17].vector.x = 0.008358;
    Sensors_Array[0].sensor_taxels[17].vector.y = 0.008418;
    Sensors_Array[0].sensor_taxels[17].vector.z = 0.013497;
    Sensors_Array[0].sensor_taxels[18].vector.x = 0.013346;
    Sensors_Array[0].sensor_taxels[18].vector.y = 0.008418;
    Sensors_Array[0].sensor_taxels[18].vector.z = 0.013055;
    Sensors_Array[0].sensor_taxels[19].vector.x = 0.020099;
    Sensors_Array[0].sensor_taxels[19].vector.y = 0.008418;
    Sensors_Array[0].sensor_taxels[19].vector.z = 0.012486;
    Sensors_Array[0].sensor_taxels[20].vector.x = 0.025033;
    Sensors_Array[0].sensor_taxels[20].vector.y = 0.008418;
    Sensors_Array[0].sensor_taxels[20].vector.z = 0.012054;
    Sensors_Array[0].sensor_taxels[21].vector.x = 0.029937;
    Sensors_Array[0].sensor_taxels[21].vector.y = 0.008418;
    Sensors_Array[0].sensor_taxels[21].vector.z = 0.011604;
    //Row3
    Sensors_Array[0].sensor_taxels[22].vector.x = -0.023585;
    Sensors_Array[0].sensor_taxels[22].vector.y = 0.008329;
    Sensors_Array[0].sensor_taxels[22].vector.z = 0.02027;
    Sensors_Array[0].sensor_taxels[23].vector.x = -0.01865;
    Sensors_Array[0].sensor_taxels[23].vector.y = 0.008329;
    Sensors_Array[0].sensor_taxels[23].vector.z = 0.019838;
    Sensors_Array[0].sensor_taxels[24].vector.x = -0.013762;
    Sensors_Array[0].sensor_taxels[24].vector.y = 0.008329;
    Sensors_Array[0].sensor_taxels[24].vector.z = 0.019411;
    Sensors_Array[0].sensor_taxels[25].vector.x = -0.00704;
    Sensors_Array[0].sensor_taxels[25].vector.y = 0.008329;
    Sensors_Array[0].sensor_taxels[25].vector.z = 0.018822;
    Sensors_Array[0].sensor_taxels[26].vector.x = -0.001728;
    Sensors_Array[0].sensor_taxels[26].vector.y = 0.008329;
    Sensors_Array[0].sensor_taxels[26].vector.z = 0.018407;
    Sensors_Array[0].sensor_taxels[27].vector.x = 0.003571;
    Sensors_Array[0].sensor_taxels[27].vector.y = 0.008329;
    Sensors_Array[0].sensor_taxels[27].vector.z = 0.017894;
    Sensors_Array[0].sensor_taxels[28].vector.x = 0.008703;
    Sensors_Array[0].sensor_taxels[28].vector.y = 0.008329;
    Sensors_Array[0].sensor_taxels[28].vector.z = 0.01745;
    Sensors_Array[0].sensor_taxels[29].vector.x = 0.013692;
    Sensors_Array[0].sensor_taxels[29].vector.y = 0.008329;
    Sensors_Array[0].sensor_taxels[29].vector.z = 0.017009;
    Sensors_Array[0].sensor_taxels[30].vector.x = 0.020443;
    Sensors_Array[0].sensor_taxels[30].vector.y = 0.008329;
    Sensors_Array[0].sensor_taxels[30].vector.z = 0.016418;
    Sensors_Array[0].sensor_taxels[31].vector.x = 0.02534;
    Sensors_Array[0].sensor_taxels[31].vector.y = 0.008329;
    Sensors_Array[0].sensor_taxels[31].vector.z = 0.015957;
    Sensors_Array[0].sensor_taxels[32].vector.x = 0.030283;
    Sensors_Array[0].sensor_taxels[32].vector.y = 0.008329;
    Sensors_Array[0].sensor_taxels[32].vector.z = 0.015557;
    //Row4
    Sensors_Array[0].sensor_taxels[33].vector.x = -0.009048;
    Sensors_Array[0].sensor_taxels[33].vector.y = 0.007277;
    Sensors_Array[0].sensor_taxels[33].vector.z = 0.024677;
    Sensors_Array[0].sensor_taxels[34].vector.x = -0.00409;
    Sensors_Array[0].sensor_taxels[34].vector.y = 0.007277;
    Sensors_Array[0].sensor_taxels[34].vector.z = 0.024221;
    Sensors_Array[0].sensor_taxels[35].vector.x = 0.001613;
    Sensors_Array[0].sensor_taxels[35].vector.y = 0.007277;
    Sensors_Array[0].sensor_taxels[35].vector.z = 0.02383;
    Sensors_Array[0].sensor_taxels[36].vector.x = 0.0068;
    Sensors_Array[0].sensor_taxels[36].vector.y = 0.007277;
    Sensors_Array[0].sensor_taxels[36].vector.z = 0.023376;
    Sensors_Array[0].sensor_taxels[37].vector.x = 0.012334;
    Sensors_Array[0].sensor_taxels[37].vector.y = 0.007277;
    Sensors_Array[0].sensor_taxels[37].vector.z = 0.022812;
    Sensors_Array[0].sensor_taxels[38].vector.x = 0.017016;
    Sensors_Array[0].sensor_taxels[38].vector.y = 0.007277;
    Sensors_Array[0].sensor_taxels[38].vector.z = 0.022411;
    //Row5
    Sensors_Array[0].sensor_taxels[39].vector.x = -0.008665;
    Sensors_Array[0].sensor_taxels[39].vector.y = 0.005852;
    Sensors_Array[0].sensor_taxels[39].vector.z = 0.029059;
    Sensors_Array[0].sensor_taxels[40].vector.x = -0.003707;
    Sensors_Array[0].sensor_taxels[40].vector.y = 0.005852;
    Sensors_Array[0].sensor_taxels[40].vector.z = 0.028605;
    Sensors_Array[0].sensor_taxels[41].vector.x = 0.001996;
    Sensors_Array[0].sensor_taxels[41].vector.y = 0.005852;
    Sensors_Array[0].sensor_taxels[41].vector.z = 0.028207;
    Sensors_Array[0].sensor_taxels[42].vector.x = 0.007183;
    Sensors_Array[0].sensor_taxels[42].vector.y = 0.005852;
    Sensors_Array[0].sensor_taxels[42].vector.z = 0.027754;
    Sensors_Array[0].sensor_taxels[43].vector.x = 0.012715;
    Sensors_Array[0].sensor_taxels[43].vector.y = 0.005852;
    Sensors_Array[0].sensor_taxels[43].vector.z = 0.027174;
    Sensors_Array[0].sensor_taxels[44].vector.x = 0.017398;
    Sensors_Array[0].sensor_taxels[44].vector.y = 0.005852;
    Sensors_Array[0].sensor_taxels[44].vector.z = 0.026779;
    //Row6
    Sensors_Array[0].sensor_taxels[45].vector.x = -0.008457;
    Sensors_Array[0].sensor_taxels[45].vector.y = 0.001986;
    Sensors_Array[0].sensor_taxels[45].vector.z = 0.031433;
    Sensors_Array[0].sensor_taxels[46].vector.x = -0.003497;
    Sensors_Array[0].sensor_taxels[46].vector.y = 0.001986;
    Sensors_Array[0].sensor_taxels[46].vector.z = 0.030997;
    Sensors_Array[0].sensor_taxels[47].vector.x = 0.012924;
    Sensors_Array[0].sensor_taxels[47].vector.y = 0.001986;
    Sensors_Array[0].sensor_taxels[47].vector.z = 0.029562;
    Sensors_Array[0].sensor_taxels[48].vector.x = 0.017605;
    Sensors_Array[0].sensor_taxels[48].vector.y = 0.001986;
    Sensors_Array[0].sensor_taxels[48].vector.z = 0.029153;
    //Row7
    Sensors_Array[0].sensor_taxels[49].vector.x = -0.008405;
    Sensors_Array[0].sensor_taxels[49].vector.y = -0.002328;
    Sensors_Array[0].sensor_taxels[49].vector.z = 0.03203;
    Sensors_Array[0].sensor_taxels[50].vector.x = -0.003445;
    Sensors_Array[0].sensor_taxels[50].vector.y = -0.002328;
    Sensors_Array[0].sensor_taxels[50].vector.z = 0.031595;
    Sensors_Array[0].sensor_taxels[51].vector.x = 0.012976;
    Sensors_Array[0].sensor_taxels[51].vector.y = -0.002328;
    Sensors_Array[0].sensor_taxels[51].vector.z = 0.030159;
    Sensors_Array[0].sensor_taxels[52].vector.x = 0.017658;
    Sensors_Array[0].sensor_taxels[52].vector.y = -0.002328;
    Sensors_Array[0].sensor_taxels[52].vector.z = 0.029749;
    //Row8
    Sensors_Array[0].sensor_taxels[53].vector.x = -0.008396;
    Sensors_Array[0].sensor_taxels[53].vector.y = -0.006292;
    Sensors_Array[0].sensor_taxels[53].vector.z = 0.032132;
    Sensors_Array[0].sensor_taxels[54].vector.x = -0.003436;
    Sensors_Array[0].sensor_taxels[54].vector.y = -0.006292;
    Sensors_Array[0].sensor_taxels[54].vector.z = 0.031698;
    Sensors_Array[0].sensor_taxels[55].vector.x = 0.012987;
    Sensors_Array[0].sensor_taxels[55].vector.y = -0.006292;
    Sensors_Array[0].sensor_taxels[55].vector.z = 0.030278;
    Sensors_Array[0].sensor_taxels[56].vector.x = 0.017668;
    Sensors_Array[0].sensor_taxels[56].vector.y = -0.006292;
    Sensors_Array[0].sensor_taxels[56].vector.z = 0.029869;
    //Row9
    Sensors_Array[0].sensor_taxels[57].vector.x = -0.008394;
    Sensors_Array[0].sensor_taxels[57].vector.y = -0.010743;
    Sensors_Array[0].sensor_taxels[57].vector.z = 0.032153;
    Sensors_Array[0].sensor_taxels[58].vector.x = -0.003434;
    Sensors_Array[0].sensor_taxels[58].vector.y = -0.010743;
    Sensors_Array[0].sensor_taxels[58].vector.z = 0.031719;
    Sensors_Array[0].sensor_taxels[59].vector.x = 0.012987;
    Sensors_Array[0].sensor_taxels[59].vector.y = -0.010743;
    Sensors_Array[0].sensor_taxels[59].vector.z = 0.030282;
    Sensors_Array[0].sensor_taxels[60].vector.x = 0.017668;
    Sensors_Array[0].sensor_taxels[60].vector.y = -0.010743;
    Sensors_Array[0].sensor_taxels[60].vector.z = 0.029873;
    //Row10
    Sensors_Array[0].sensor_taxels[61].vector.x = -0.008394;
    Sensors_Array[0].sensor_taxels[61].vector.y = -0.015192;
    Sensors_Array[0].sensor_taxels[61].vector.z = 0.032153;
    Sensors_Array[0].sensor_taxels[62].vector.x = -0.003434;
    Sensors_Array[0].sensor_taxels[62].vector.y = -0.015192;
    Sensors_Array[0].sensor_taxels[62].vector.z = 0.031719;
    Sensors_Array[0].sensor_taxels[63].vector.x = 0.012987;
    Sensors_Array[0].sensor_taxels[63].vector.y = -0.015192;
    Sensors_Array[0].sensor_taxels[63].vector.z = 0.030282;
    Sensors_Array[0].sensor_taxels[64].vector.x = 0.017668;
    Sensors_Array[0].sensor_taxels[64].vector.y = -0.015192;
    Sensors_Array[0].sensor_taxels[64].vector.z = 0.029873;

    //Middle Proximal Front

    //Row1
    Sensors_Array[5].sensor_taxels[0].vector.x = -0.024275;
    Sensors_Array[5].sensor_taxels[0].vector.y = 0.008283;
    Sensors_Array[5].sensor_taxels[0].vector.z = 0.012385;
    Sensors_Array[5].sensor_taxels[1].vector.x = -0.01934;
    Sensors_Array[5].sensor_taxels[1].vector.y = 0.008283;
    Sensors_Array[5].sensor_taxels[1].vector.z = 0.011953;
    Sensors_Array[5].sensor_taxels[2].vector.x = -0.014452;
    Sensors_Array[5].sensor_taxels[2].vector.y = 0.008283;
    Sensors_Array[5].sensor_taxels[2].vector.z = 0.011525;
    Sensors_Array[5].sensor_taxels[3].vector.x = -0.00773;
    Sensors_Array[5].sensor_taxels[3].vector.y = 0.008283;
    Sensors_Array[5].sensor_taxels[3].vector.z = 0.010937;
    Sensors_Array[5].sensor_taxels[4].vector.x = -0.002418;
    Sensors_Array[5].sensor_taxels[4].vector.y = 0.008283;
    Sensors_Array[5].sensor_taxels[4].vector.z = 0.010521;
    Sensors_Array[5].sensor_taxels[5].vector.x = 0.002881;
    Sensors_Array[5].sensor_taxels[5].vector.y = 0.008283;
    Sensors_Array[5].sensor_taxels[5].vector.z = 0.010008;
    Sensors_Array[5].sensor_taxels[6].vector.x = 0.008014;
    Sensors_Array[5].sensor_taxels[6].vector.y = 0.008283;
    Sensors_Array[5].sensor_taxels[6].vector.z = 0.009564;
    Sensors_Array[5].sensor_taxels[7].vector.x = 0.013002;
    Sensors_Array[5].sensor_taxels[7].vector.y = 0.008283;
    Sensors_Array[5].sensor_taxels[7].vector.z = 0.009122;
    Sensors_Array[5].sensor_taxels[8].vector.x = 0.019755;
    Sensors_Array[5].sensor_taxels[8].vector.y = 0.008283;
    Sensors_Array[5].sensor_taxels[8].vector.z = 0.008553;
    Sensors_Array[5].sensor_taxels[9].vector.x = 0.024652;
    Sensors_Array[5].sensor_taxels[9].vector.y = 0.008283;
    Sensors_Array[5].sensor_taxels[9].vector.z = 0.008093;
    Sensors_Array[5].sensor_taxels[10].vector.x = 0.029593;
    Sensors_Array[5].sensor_taxels[10].vector.y = 0.008283;
    Sensors_Array[5].sensor_taxels[10].vector.z = 0.007671;
    //Row2
    Sensors_Array[5].sensor_taxels[11].vector.x = -0.02393;
    Sensors_Array[5].sensor_taxels[11].vector.y = 0.008418;
    Sensors_Array[5].sensor_taxels[11].vector.z = 0.016317;
    Sensors_Array[5].sensor_taxels[12].vector.x = -0.018996;
    Sensors_Array[5].sensor_taxels[12].vector.y = 0.008418;
    Sensors_Array[5].sensor_taxels[12].vector.z = 0.015885;
    Sensors_Array[5].sensor_taxels[13].vector.x = -0.014108;
    Sensors_Array[5].sensor_taxels[13].vector.y = 0.008418;
    Sensors_Array[5].sensor_taxels[13].vector.z = 0.015457;
    Sensors_Array[5].sensor_taxels[14].vector.x = -0.007386;
    Sensors_Array[5].sensor_taxels[14].vector.y = 0.008418;
    Sensors_Array[5].sensor_taxels[14].vector.z = 0.014869;
    Sensors_Array[5].sensor_taxels[15].vector.x = -0.002074;
    Sensors_Array[5].sensor_taxels[15].vector.y = 0.008418;
    Sensors_Array[5].sensor_taxels[15].vector.z = 0.014454;
    Sensors_Array[5].sensor_taxels[16].vector.x = 0.003225;
    Sensors_Array[5].sensor_taxels[16].vector.y = 0.008418;
    Sensors_Array[5].sensor_taxels[16].vector.z = 0.013941;
    Sensors_Array[5].sensor_taxels[17].vector.x = 0.008358;
    Sensors_Array[5].sensor_taxels[17].vector.y = 0.008418;
    Sensors_Array[5].sensor_taxels[17].vector.z = 0.013497;
    Sensors_Array[5].sensor_taxels[18].vector.x = 0.013346;
    Sensors_Array[5].sensor_taxels[18].vector.y = 0.008418;
    Sensors_Array[5].sensor_taxels[18].vector.z = 0.013055;
    Sensors_Array[5].sensor_taxels[19].vector.x = 0.020099;
    Sensors_Array[5].sensor_taxels[19].vector.y = 0.008418;
    Sensors_Array[5].sensor_taxels[19].vector.z = 0.012486;
    Sensors_Array[5].sensor_taxels[20].vector.x = 0.025033;
    Sensors_Array[5].sensor_taxels[20].vector.y = 0.008418;
    Sensors_Array[5].sensor_taxels[20].vector.z = 0.012054;
    Sensors_Array[5].sensor_taxels[21].vector.x = 0.029937;
    Sensors_Array[5].sensor_taxels[21].vector.y = 0.008418;
    Sensors_Array[5].sensor_taxels[21].vector.z = 0.011604;
    //Row3
    Sensors_Array[5].sensor_taxels[22].vector.x = -0.023585;
    Sensors_Array[5].sensor_taxels[22].vector.y = 0.008329;
    Sensors_Array[5].sensor_taxels[22].vector.z = 0.02027;
    Sensors_Array[5].sensor_taxels[23].vector.x = -0.01865;
    Sensors_Array[5].sensor_taxels[23].vector.y = 0.008329;
    Sensors_Array[5].sensor_taxels[23].vector.z = 0.019838;
    Sensors_Array[5].sensor_taxels[24].vector.x = -0.013762;
    Sensors_Array[5].sensor_taxels[24].vector.y = 0.008329;
    Sensors_Array[5].sensor_taxels[24].vector.z = 0.019411;
    Sensors_Array[5].sensor_taxels[25].vector.x = -0.00704;
    Sensors_Array[5].sensor_taxels[25].vector.y = 0.008329;
    Sensors_Array[5].sensor_taxels[25].vector.z = 0.018822;
    Sensors_Array[5].sensor_taxels[26].vector.x = -0.001728;
    Sensors_Array[5].sensor_taxels[26].vector.y = 0.008329;
    Sensors_Array[5].sensor_taxels[26].vector.z = 0.018407;
    Sensors_Array[5].sensor_taxels[27].vector.x = 0.003571;
    Sensors_Array[5].sensor_taxels[27].vector.y = 0.008329;
    Sensors_Array[5].sensor_taxels[27].vector.z = 0.017894;
    Sensors_Array[5].sensor_taxels[28].vector.x = 0.008703;
    Sensors_Array[5].sensor_taxels[28].vector.y = 0.008329;
    Sensors_Array[5].sensor_taxels[28].vector.z = 0.01745;
    Sensors_Array[5].sensor_taxels[29].vector.x = 0.013692;
    Sensors_Array[5].sensor_taxels[29].vector.y = 0.008329;
    Sensors_Array[5].sensor_taxels[29].vector.z = 0.017009;
    Sensors_Array[5].sensor_taxels[30].vector.x = 0.020443;
    Sensors_Array[5].sensor_taxels[30].vector.y = 0.008329;
    Sensors_Array[5].sensor_taxels[30].vector.z = 0.016418;
    Sensors_Array[5].sensor_taxels[31].vector.x = 0.02534;
    Sensors_Array[5].sensor_taxels[31].vector.y = 0.008329;
    Sensors_Array[5].sensor_taxels[31].vector.z = 0.015957;
    Sensors_Array[5].sensor_taxels[32].vector.x = 0.030283;
    Sensors_Array[5].sensor_taxels[32].vector.y = 0.008329;
    Sensors_Array[5].sensor_taxels[32].vector.z = 0.015557;
    //Row4
    Sensors_Array[5].sensor_taxels[33].vector.x = -0.009048;
    Sensors_Array[5].sensor_taxels[33].vector.y = 0.007277;
    Sensors_Array[5].sensor_taxels[33].vector.z = 0.024677;
    Sensors_Array[5].sensor_taxels[34].vector.x = -0.00409;
    Sensors_Array[5].sensor_taxels[34].vector.y = 0.007277;
    Sensors_Array[5].sensor_taxels[34].vector.z = 0.024221;
    Sensors_Array[5].sensor_taxels[35].vector.x = 0.001613;
    Sensors_Array[5].sensor_taxels[35].vector.y = 0.007277;
    Sensors_Array[5].sensor_taxels[35].vector.z = 0.02383;
    Sensors_Array[5].sensor_taxels[36].vector.x = 0.0068;
    Sensors_Array[5].sensor_taxels[36].vector.y = 0.007277;
    Sensors_Array[5].sensor_taxels[36].vector.z = 0.023376;
    Sensors_Array[5].sensor_taxels[37].vector.x = 0.012334;
    Sensors_Array[5].sensor_taxels[37].vector.y = 0.007277;
    Sensors_Array[5].sensor_taxels[37].vector.z = 0.022812;
    Sensors_Array[5].sensor_taxels[38].vector.x = 0.017016;
    Sensors_Array[5].sensor_taxels[38].vector.y = 0.007277;
    Sensors_Array[5].sensor_taxels[38].vector.z = 0.022411;
    //Row5
    Sensors_Array[5].sensor_taxels[39].vector.x = -0.008665;
    Sensors_Array[5].sensor_taxels[39].vector.y = 0.005852;
    Sensors_Array[5].sensor_taxels[39].vector.z = 0.029059;
    Sensors_Array[5].sensor_taxels[40].vector.x = -0.003707;
    Sensors_Array[5].sensor_taxels[40].vector.y = 0.005852;
    Sensors_Array[5].sensor_taxels[40].vector.z = 0.028605;
    Sensors_Array[5].sensor_taxels[41].vector.x = 0.001996;
    Sensors_Array[5].sensor_taxels[41].vector.y = 0.005852;
    Sensors_Array[5].sensor_taxels[41].vector.z = 0.028207;
    Sensors_Array[5].sensor_taxels[42].vector.x = 0.007183;
    Sensors_Array[5].sensor_taxels[42].vector.y = 0.005852;
    Sensors_Array[5].sensor_taxels[42].vector.z = 0.027754;
    Sensors_Array[5].sensor_taxels[43].vector.x = 0.012715;
    Sensors_Array[5].sensor_taxels[43].vector.y = 0.005852;
    Sensors_Array[5].sensor_taxels[43].vector.z = 0.027174;
    Sensors_Array[5].sensor_taxels[44].vector.x = 0.017398;
    Sensors_Array[5].sensor_taxels[44].vector.y = 0.005852;
    Sensors_Array[5].sensor_taxels[44].vector.z = 0.026779;
    //Row6
    Sensors_Array[5].sensor_taxels[45].vector.x = -0.008457;
    Sensors_Array[5].sensor_taxels[45].vector.y = 0.001986;
    Sensors_Array[5].sensor_taxels[45].vector.z = 0.031433;
    Sensors_Array[5].sensor_taxels[46].vector.x = -0.003497;
    Sensors_Array[5].sensor_taxels[46].vector.y = 0.001986;
    Sensors_Array[5].sensor_taxels[46].vector.z = 0.030997;
    Sensors_Array[5].sensor_taxels[47].vector.x = 0.012924;
    Sensors_Array[5].sensor_taxels[47].vector.y = 0.001986;
    Sensors_Array[5].sensor_taxels[47].vector.z = 0.029562;
    Sensors_Array[5].sensor_taxels[48].vector.x = 0.017605;
    Sensors_Array[5].sensor_taxels[48].vector.y = 0.001986;
    Sensors_Array[5].sensor_taxels[48].vector.z = 0.029153;
    //Row7
    Sensors_Array[5].sensor_taxels[49].vector.x = -0.008405;
    Sensors_Array[5].sensor_taxels[49].vector.y = -0.002328;
    Sensors_Array[5].sensor_taxels[49].vector.z = 0.03203;
    Sensors_Array[5].sensor_taxels[50].vector.x = -0.003445;
    Sensors_Array[5].sensor_taxels[50].vector.y = -0.002328;
    Sensors_Array[5].sensor_taxels[50].vector.z = 0.031595;
    Sensors_Array[5].sensor_taxels[51].vector.x = 0.012976;
    Sensors_Array[5].sensor_taxels[51].vector.y = -0.002328;
    Sensors_Array[5].sensor_taxels[51].vector.z = 0.030159;
    Sensors_Array[5].sensor_taxels[52].vector.x = 0.017658;
    Sensors_Array[5].sensor_taxels[52].vector.y = -0.002328;
    Sensors_Array[5].sensor_taxels[52].vector.z = 0.029749;
    //Row8
    Sensors_Array[5].sensor_taxels[53].vector.x = -0.008396;
    Sensors_Array[5].sensor_taxels[53].vector.y = -0.006292;
    Sensors_Array[5].sensor_taxels[53].vector.z = 0.032132;
    Sensors_Array[5].sensor_taxels[54].vector.x = -0.003436;
    Sensors_Array[5].sensor_taxels[54].vector.y = -0.006292;
    Sensors_Array[5].sensor_taxels[54].vector.z = 0.031698;
    Sensors_Array[5].sensor_taxels[55].vector.x = 0.012987;
    Sensors_Array[5].sensor_taxels[55].vector.y = -0.006292;
    Sensors_Array[5].sensor_taxels[55].vector.z = 0.030278;
    Sensors_Array[5].sensor_taxels[56].vector.x = 0.017668;
    Sensors_Array[5].sensor_taxels[56].vector.y = -0.006292;
    Sensors_Array[5].sensor_taxels[56].vector.z = 0.029869;
    //Row9
    Sensors_Array[5].sensor_taxels[57].vector.x = -0.008394;
    Sensors_Array[5].sensor_taxels[57].vector.y = -0.010743;
    Sensors_Array[5].sensor_taxels[57].vector.z = 0.032153;
    Sensors_Array[5].sensor_taxels[58].vector.x = -0.003434;
    Sensors_Array[5].sensor_taxels[58].vector.y = -0.010743;
    Sensors_Array[5].sensor_taxels[58].vector.z = 0.031719;
    Sensors_Array[5].sensor_taxels[59].vector.x = 0.012987;
    Sensors_Array[5].sensor_taxels[59].vector.y = -0.010743;
    Sensors_Array[5].sensor_taxels[59].vector.z = 0.030282;
    Sensors_Array[5].sensor_taxels[60].vector.x = 0.017668;
    Sensors_Array[5].sensor_taxels[60].vector.y = -0.010743;
    Sensors_Array[5].sensor_taxels[60].vector.z = 0.029873;
    //Row10
    Sensors_Array[5].sensor_taxels[61].vector.x = -0.008394;
    Sensors_Array[5].sensor_taxels[61].vector.y = -0.015192;
    Sensors_Array[5].sensor_taxels[61].vector.z = 0.032153;
    Sensors_Array[5].sensor_taxels[62].vector.x = -0.003434;
    Sensors_Array[5].sensor_taxels[62].vector.y = -0.015192;
    Sensors_Array[5].sensor_taxels[62].vector.z = 0.031719;
    Sensors_Array[5].sensor_taxels[63].vector.x = 0.012987;
    Sensors_Array[5].sensor_taxels[63].vector.y = -0.015192;
    Sensors_Array[5].sensor_taxels[63].vector.z = 0.030282;
    Sensors_Array[5].sensor_taxels[64].vector.x = 0.017668;
    Sensors_Array[5].sensor_taxels[64].vector.y = -0.015192;
    Sensors_Array[5].sensor_taxels[64].vector.z = 0.029873;
    //Pinky Proximal Front

    //Row1
    Sensors_Array[17].sensor_taxels[0].vector.x = -0.024275;
    Sensors_Array[17].sensor_taxels[0].vector.y = 0.008283;
    Sensors_Array[17].sensor_taxels[0].vector.z = 0.012385;
    Sensors_Array[17].sensor_taxels[1].vector.x = -0.01934;
    Sensors_Array[17].sensor_taxels[1].vector.y = 0.008283;
    Sensors_Array[17].sensor_taxels[1].vector.z = 0.011953;
    Sensors_Array[17].sensor_taxels[2].vector.x = -0.014452;
    Sensors_Array[17].sensor_taxels[2].vector.y = 0.008283;
    Sensors_Array[17].sensor_taxels[2].vector.z = 0.011525;
    Sensors_Array[17].sensor_taxels[3].vector.x = -0.00773;
    Sensors_Array[17].sensor_taxels[3].vector.y = 0.008283;
    Sensors_Array[17].sensor_taxels[3].vector.z = 0.010937;
    Sensors_Array[17].sensor_taxels[4].vector.x = -0.002418;
    Sensors_Array[17].sensor_taxels[4].vector.y = 0.008283;
    Sensors_Array[17].sensor_taxels[4].vector.z = 0.010521;
    Sensors_Array[17].sensor_taxels[5].vector.x = 0.002881;
    Sensors_Array[17].sensor_taxels[5].vector.y = 0.008283;
    Sensors_Array[17].sensor_taxels[5].vector.z = 0.010008;
    Sensors_Array[17].sensor_taxels[6].vector.x = 0.008014;
    Sensors_Array[17].sensor_taxels[6].vector.y = 0.008283;
    Sensors_Array[17].sensor_taxels[6].vector.z = 0.009564;
    Sensors_Array[17].sensor_taxels[7].vector.x = 0.013002;
    Sensors_Array[17].sensor_taxels[7].vector.y = 0.008283;
    Sensors_Array[17].sensor_taxels[7].vector.z = 0.009122;
    Sensors_Array[17].sensor_taxels[8].vector.x = 0.019755;
    Sensors_Array[17].sensor_taxels[8].vector.y = 0.008283;
    Sensors_Array[17].sensor_taxels[8].vector.z = 0.008553;
    Sensors_Array[17].sensor_taxels[9].vector.x = 0.024652;
    Sensors_Array[17].sensor_taxels[9].vector.y = 0.008283;
    Sensors_Array[17].sensor_taxels[9].vector.z = 0.008093;
    Sensors_Array[17].sensor_taxels[10].vector.x = 0.029593;
    Sensors_Array[17].sensor_taxels[10].vector.y = 0.008283;
    Sensors_Array[17].sensor_taxels[10].vector.z = 0.007671;
    //Row2
    Sensors_Array[17].sensor_taxels[11].vector.x = -0.02393;
    Sensors_Array[17].sensor_taxels[11].vector.y = 0.008418;
    Sensors_Array[17].sensor_taxels[11].vector.z = 0.016317;
    Sensors_Array[17].sensor_taxels[12].vector.x = -0.018996;
    Sensors_Array[17].sensor_taxels[12].vector.y = 0.008418;
    Sensors_Array[17].sensor_taxels[12].vector.z = 0.015885;
    Sensors_Array[17].sensor_taxels[13].vector.x = -0.014108;
    Sensors_Array[17].sensor_taxels[13].vector.y = 0.008418;
    Sensors_Array[17].sensor_taxels[13].vector.z = 0.015457;
    Sensors_Array[17].sensor_taxels[14].vector.x = -0.007386;
    Sensors_Array[17].sensor_taxels[14].vector.y = 0.008418;
    Sensors_Array[17].sensor_taxels[14].vector.z = 0.014869;
    Sensors_Array[17].sensor_taxels[15].vector.x = -0.002074;
    Sensors_Array[17].sensor_taxels[15].vector.y = 0.008418;
    Sensors_Array[17].sensor_taxels[15].vector.z = 0.014454;
    Sensors_Array[17].sensor_taxels[16].vector.x = 0.003225;
    Sensors_Array[17].sensor_taxels[16].vector.y = 0.008418;
    Sensors_Array[17].sensor_taxels[16].vector.z = 0.013941;
    Sensors_Array[17].sensor_taxels[17].vector.x = 0.008358;
    Sensors_Array[17].sensor_taxels[17].vector.y = 0.008418;
    Sensors_Array[17].sensor_taxels[17].vector.z = 0.013497;
    Sensors_Array[17].sensor_taxels[18].vector.x = 0.013346;
    Sensors_Array[17].sensor_taxels[18].vector.y = 0.008418;
    Sensors_Array[17].sensor_taxels[18].vector.z = 0.013055;
    Sensors_Array[17].sensor_taxels[19].vector.x = 0.020099;
    Sensors_Array[17].sensor_taxels[19].vector.y = 0.008418;
    Sensors_Array[17].sensor_taxels[19].vector.z = 0.012486;
    Sensors_Array[17].sensor_taxels[20].vector.x = 0.025033;
    Sensors_Array[17].sensor_taxels[20].vector.y = 0.008418;
    Sensors_Array[17].sensor_taxels[20].vector.z = 0.012054;
    Sensors_Array[17].sensor_taxels[21].vector.x = 0.029937;
    Sensors_Array[17].sensor_taxels[21].vector.y = 0.008418;
    Sensors_Array[17].sensor_taxels[21].vector.z = 0.011604;
    //Row3
    Sensors_Array[17].sensor_taxels[22].vector.x = -0.023585;
    Sensors_Array[17].sensor_taxels[22].vector.y = 0.008329;
    Sensors_Array[17].sensor_taxels[22].vector.z = 0.02027;
    Sensors_Array[17].sensor_taxels[23].vector.x = -0.01865;
    Sensors_Array[17].sensor_taxels[23].vector.y = 0.008329;
    Sensors_Array[17].sensor_taxels[23].vector.z = 0.019838;
    Sensors_Array[17].sensor_taxels[24].vector.x = -0.013762;
    Sensors_Array[17].sensor_taxels[24].vector.y = 0.008329;
    Sensors_Array[17].sensor_taxels[24].vector.z = 0.019411;
    Sensors_Array[17].sensor_taxels[25].vector.x = -0.00704;
    Sensors_Array[17].sensor_taxels[25].vector.y = 0.008329;
    Sensors_Array[17].sensor_taxels[25].vector.z = 0.018822;
    Sensors_Array[17].sensor_taxels[26].vector.x = -0.001728;
    Sensors_Array[17].sensor_taxels[26].vector.y = 0.008329;
    Sensors_Array[17].sensor_taxels[26].vector.z = 0.018407;
    Sensors_Array[17].sensor_taxels[27].vector.x = 0.003571;
    Sensors_Array[17].sensor_taxels[27].vector.y = 0.008329;
    Sensors_Array[17].sensor_taxels[27].vector.z = 0.017894;
    Sensors_Array[17].sensor_taxels[28].vector.x = 0.008703;
    Sensors_Array[17].sensor_taxels[28].vector.y = 0.008329;
    Sensors_Array[17].sensor_taxels[28].vector.z = 0.01745;
    Sensors_Array[17].sensor_taxels[29].vector.x = 0.013692;
    Sensors_Array[17].sensor_taxels[29].vector.y = 0.008329;
    Sensors_Array[17].sensor_taxels[29].vector.z = 0.017009;
    Sensors_Array[17].sensor_taxels[30].vector.x = 0.020443;
    Sensors_Array[17].sensor_taxels[30].vector.y = 0.008329;
    Sensors_Array[17].sensor_taxels[30].vector.z = 0.016418;
    Sensors_Array[17].sensor_taxels[31].vector.x = 0.02534;
    Sensors_Array[17].sensor_taxels[31].vector.y = 0.008329;
    Sensors_Array[17].sensor_taxels[31].vector.z = 0.015957;
    Sensors_Array[17].sensor_taxels[32].vector.x = 0.030283;
    Sensors_Array[17].sensor_taxels[32].vector.y = 0.008329;
    Sensors_Array[17].sensor_taxels[32].vector.z = 0.015557;
    //Row4
    Sensors_Array[17].sensor_taxels[33].vector.x = -0.009048;
    Sensors_Array[17].sensor_taxels[33].vector.y = 0.007277;
    Sensors_Array[17].sensor_taxels[33].vector.z = 0.024677;
    Sensors_Array[17].sensor_taxels[34].vector.x = -0.00409;
    Sensors_Array[17].sensor_taxels[34].vector.y = 0.007277;
    Sensors_Array[17].sensor_taxels[34].vector.z = 0.024221;
    Sensors_Array[17].sensor_taxels[35].vector.x = 0.001613;
    Sensors_Array[17].sensor_taxels[35].vector.y = 0.007277;
    Sensors_Array[17].sensor_taxels[35].vector.z = 0.02383;
    Sensors_Array[17].sensor_taxels[36].vector.x = 0.0068;
    Sensors_Array[17].sensor_taxels[36].vector.y = 0.007277;
    Sensors_Array[17].sensor_taxels[36].vector.z = 0.023376;
    Sensors_Array[17].sensor_taxels[37].vector.x = 0.012334;
    Sensors_Array[17].sensor_taxels[37].vector.y = 0.007277;
    Sensors_Array[17].sensor_taxels[37].vector.z = 0.022812;
    Sensors_Array[17].sensor_taxels[38].vector.x = 0.017016;
    Sensors_Array[17].sensor_taxels[38].vector.y = 0.007277;
    Sensors_Array[17].sensor_taxels[38].vector.z = 0.022411;
    //Row5
    Sensors_Array[17].sensor_taxels[39].vector.x = -0.008665;
    Sensors_Array[17].sensor_taxels[39].vector.y = 0.005852;
    Sensors_Array[17].sensor_taxels[39].vector.z = 0.029059;
    Sensors_Array[17].sensor_taxels[40].vector.x = -0.003707;
    Sensors_Array[17].sensor_taxels[40].vector.y = 0.005852;
    Sensors_Array[17].sensor_taxels[40].vector.z = 0.028605;
    Sensors_Array[17].sensor_taxels[41].vector.x = 0.001996;
    Sensors_Array[17].sensor_taxels[41].vector.y = 0.005852;
    Sensors_Array[17].sensor_taxels[41].vector.z = 0.028207;
    Sensors_Array[17].sensor_taxels[42].vector.x = 0.007183;
    Sensors_Array[17].sensor_taxels[42].vector.y = 0.005852;
    Sensors_Array[17].sensor_taxels[42].vector.z = 0.027754;
    Sensors_Array[17].sensor_taxels[43].vector.x = 0.012715;
    Sensors_Array[17].sensor_taxels[43].vector.y = 0.005852;
    Sensors_Array[17].sensor_taxels[43].vector.z = 0.027174;
    Sensors_Array[17].sensor_taxels[44].vector.x = 0.017398;
    Sensors_Array[17].sensor_taxels[44].vector.y = 0.005852;
    Sensors_Array[17].sensor_taxels[44].vector.z = 0.026779;
    //Row6
    Sensors_Array[17].sensor_taxels[45].vector.x = -0.008457;
    Sensors_Array[17].sensor_taxels[45].vector.y = 0.001986;
    Sensors_Array[17].sensor_taxels[45].vector.z = 0.031433;
    Sensors_Array[17].sensor_taxels[46].vector.x = -0.003497;
    Sensors_Array[17].sensor_taxels[46].vector.y = 0.001986;
    Sensors_Array[17].sensor_taxels[46].vector.z = 0.030997;
    Sensors_Array[17].sensor_taxels[47].vector.x = 0.012924;
    Sensors_Array[17].sensor_taxels[47].vector.y = 0.001986;
    Sensors_Array[17].sensor_taxels[47].vector.z = 0.029562;
    Sensors_Array[17].sensor_taxels[48].vector.x = 0.017605;
    Sensors_Array[17].sensor_taxels[48].vector.y = 0.001986;
    Sensors_Array[17].sensor_taxels[48].vector.z = 0.029153;
    //Row7
    Sensors_Array[17].sensor_taxels[49].vector.x = -0.008405;
    Sensors_Array[17].sensor_taxels[49].vector.y = -0.002328;
    Sensors_Array[17].sensor_taxels[49].vector.z = 0.03203;
    Sensors_Array[17].sensor_taxels[50].vector.x = -0.003445;
    Sensors_Array[17].sensor_taxels[50].vector.y = -0.002328;
    Sensors_Array[17].sensor_taxels[50].vector.z = 0.031595;
    Sensors_Array[17].sensor_taxels[51].vector.x = 0.012976;
    Sensors_Array[17].sensor_taxels[51].vector.y = -0.002328;
    Sensors_Array[17].sensor_taxels[51].vector.z = 0.030159;
    Sensors_Array[17].sensor_taxels[52].vector.x = 0.017658;
    Sensors_Array[17].sensor_taxels[52].vector.y = -0.002328;
    Sensors_Array[17].sensor_taxels[52].vector.z = 0.029749;
    //Row8
    Sensors_Array[17].sensor_taxels[53].vector.x = -0.008396;
    Sensors_Array[17].sensor_taxels[53].vector.y = -0.006292;
    Sensors_Array[17].sensor_taxels[53].vector.z = 0.032132;
    Sensors_Array[17].sensor_taxels[54].vector.x = -0.003436;
    Sensors_Array[17].sensor_taxels[54].vector.y = -0.006292;
    Sensors_Array[17].sensor_taxels[54].vector.z = 0.031698;
    Sensors_Array[17].sensor_taxels[55].vector.x = 0.012987;
    Sensors_Array[17].sensor_taxels[55].vector.y = -0.006292;
    Sensors_Array[17].sensor_taxels[55].vector.z = 0.030278;
    Sensors_Array[17].sensor_taxels[56].vector.x = 0.017668;
    Sensors_Array[17].sensor_taxels[56].vector.y = -0.006292;
    Sensors_Array[17].sensor_taxels[56].vector.z = 0.029869;
    //Row9
    Sensors_Array[17].sensor_taxels[57].vector.x = -0.008394;
    Sensors_Array[17].sensor_taxels[57].vector.y = -0.010743;
    Sensors_Array[17].sensor_taxels[57].vector.z = 0.032153;
    Sensors_Array[17].sensor_taxels[58].vector.x = -0.003434;
    Sensors_Array[17].sensor_taxels[58].vector.y = -0.010743;
    Sensors_Array[17].sensor_taxels[58].vector.z = 0.031719;
    Sensors_Array[17].sensor_taxels[59].vector.x = 0.012987;
    Sensors_Array[17].sensor_taxels[59].vector.y = -0.010743;
    Sensors_Array[17].sensor_taxels[59].vector.z = 0.030282;
    Sensors_Array[17].sensor_taxels[60].vector.x = 0.017668;
    Sensors_Array[17].sensor_taxels[60].vector.y = -0.010743;
    Sensors_Array[17].sensor_taxels[60].vector.z = 0.029873;
    //Row10
    Sensors_Array[17].sensor_taxels[61].vector.x = -0.008394;
    Sensors_Array[17].sensor_taxels[61].vector.y = -0.015192;
    Sensors_Array[17].sensor_taxels[61].vector.z = 0.032153;
    Sensors_Array[17].sensor_taxels[62].vector.x = -0.003434;
    Sensors_Array[17].sensor_taxels[62].vector.y = -0.015192;
    Sensors_Array[17].sensor_taxels[62].vector.z = 0.031719;
    Sensors_Array[17].sensor_taxels[63].vector.x = 0.012987;
    Sensors_Array[17].sensor_taxels[63].vector.y = -0.015192;
    Sensors_Array[17].sensor_taxels[63].vector.z = 0.030282;
    Sensors_Array[17].sensor_taxels[64].vector.x = 0.017668;
    Sensors_Array[17].sensor_taxels[64].vector.y = -0.015192;
    Sensors_Array[17].sensor_taxels[64].vector.z = 0.029873;
    //Index Medial Front

    //Row1
    Sensors_Array[21].sensor_taxels[0].vector.x = 0.018017;
    Sensors_Array[21].sensor_taxels[0].vector.y = 0.01202;
    Sensors_Array[21].sensor_taxels[0].vector.z = 0.011304;
    Sensors_Array[21].sensor_taxels[1].vector.x = 0.009442;
    Sensors_Array[21].sensor_taxels[1].vector.y = 0.01202;
    Sensors_Array[21].sensor_taxels[1].vector.z = 0.01053;
    Sensors_Array[21].sensor_taxels[2].vector.x = 0.003686;
    Sensors_Array[21].sensor_taxels[2].vector.y = 0.01202;
    Sensors_Array[21].sensor_taxels[2].vector.z = 0.010026;
    Sensors_Array[21].sensor_taxels[3].vector.x = -0.001835;
    Sensors_Array[21].sensor_taxels[3].vector.y = 0.01202;
    Sensors_Array[21].sensor_taxels[3].vector.z = 0.009543;
    //Row2
    Sensors_Array[21].sensor_taxels[4].vector.x = 0.017651;
    Sensors_Array[21].sensor_taxels[4].vector.y = 0.012084;
    Sensors_Array[21].sensor_taxels[4].vector.z = 0.01549;
    Sensors_Array[21].sensor_taxels[5].vector.x = 0.009076;
    Sensors_Array[21].sensor_taxels[5].vector.y = 0.012084;
    Sensors_Array[21].sensor_taxels[5].vector.z = 0.014715;
    Sensors_Array[21].sensor_taxels[6].vector.x = 0.00332;
    Sensors_Array[21].sensor_taxels[6].vector.y = 0.012084;
    Sensors_Array[21].sensor_taxels[6].vector.z = 0.014212;
    Sensors_Array[21].sensor_taxels[7].vector.x = -0.002201;
    Sensors_Array[21].sensor_taxels[7].vector.y = 0.012084;
    Sensors_Array[21].sensor_taxels[7].vector.z = 0.013729;
    //Row3
    Sensors_Array[21].sensor_taxels[8].vector.x = 0.017397;
    Sensors_Array[21].sensor_taxels[8].vector.y = 0.012048;
    Sensors_Array[21].sensor_taxels[8].vector.z = 0.019322;
    Sensors_Array[21].sensor_taxels[9].vector.x = 0.008742;
    Sensors_Array[21].sensor_taxels[9].vector.y = 0.012048;
    Sensors_Array[21].sensor_taxels[9].vector.z = 0.01854;
    Sensors_Array[21].sensor_taxels[10].vector.x = 0.002985;
    Sensors_Array[21].sensor_taxels[10].vector.y = 0.012048;
    Sensors_Array[21].sensor_taxels[10].vector.z = 0.018036;
    Sensors_Array[21].sensor_taxels[11].vector.x = -0.002535;
    Sensors_Array[21].sensor_taxels[11].vector.y = 0.012048;
    Sensors_Array[21].sensor_taxels[11].vector.z = 0.017554;
    //Row4
    Sensors_Array[21].sensor_taxels[12].vector.x = 0.008212;
    Sensors_Array[21].sensor_taxels[12].vector.y = 0.011029;
    Sensors_Array[21].sensor_taxels[12].vector.z = 0.024592;
    Sensors_Array[21].sensor_taxels[13].vector.x = 0.002456;
    Sensors_Array[21].sensor_taxels[13].vector.y = 0.011029;
    Sensors_Array[21].sensor_taxels[13].vector.z = 0.024089;
    Sensors_Array[21].sensor_taxels[14].vector.x = -0.003064;
    Sensors_Array[21].sensor_taxels[14].vector.y = 0.011029;
    Sensors_Array[21].sensor_taxels[14].vector.z = 0.023606;
    //Row5
    Sensors_Array[21].sensor_taxels[15].vector.x = 0.007803;
    Sensors_Array[21].sensor_taxels[15].vector.y = 0.009308;
    Sensors_Array[21].sensor_taxels[15].vector.z = 0.029273;
    Sensors_Array[21].sensor_taxels[16].vector.x = 0.002047;
    Sensors_Array[21].sensor_taxels[16].vector.y = 0.009308;
    Sensors_Array[21].sensor_taxels[16].vector.z = 0.028769;
    Sensors_Array[21].sensor_taxels[17].vector.x = -0.003474;
    Sensors_Array[21].sensor_taxels[17].vector.y = 0.009308;
    Sensors_Array[21].sensor_taxels[17].vector.z = 0.028287;
    //Row6
    Sensors_Array[21].sensor_taxels[18].vector.x = 0.007614;
    Sensors_Array[21].sensor_taxels[18].vector.y = 0.004825;
    Sensors_Array[21].sensor_taxels[18].vector.z = 0.031433;
    Sensors_Array[21].sensor_taxels[19].vector.x = 0.001858;
    Sensors_Array[21].sensor_taxels[19].vector.y = 0.004825;
    Sensors_Array[21].sensor_taxels[19].vector.z = 0.03093;
    Sensors_Array[21].sensor_taxels[20].vector.x = -0.003662;
    Sensors_Array[21].sensor_taxels[20].vector.y = 0.004825;
    Sensors_Array[21].sensor_taxels[20].vector.z = 0.030447;
    //Row7
    Sensors_Array[21].sensor_taxels[21].vector.x = 0.007562;
    Sensors_Array[21].sensor_taxels[21].vector.y = -0.000529;
    Sensors_Array[21].sensor_taxels[21].vector.z = 0.032032;
    Sensors_Array[21].sensor_taxels[22].vector.x = 0.001806;
    Sensors_Array[21].sensor_taxels[22].vector.y = -0.000529;
    Sensors_Array[21].sensor_taxels[22].vector.z = 0.031528;
    Sensors_Array[21].sensor_taxels[23].vector.x = -0.003715;
    Sensors_Array[21].sensor_taxels[23].vector.y = -0.000529;
    Sensors_Array[21].sensor_taxels[23].vector.z = 0.031045;
    //Row8
    Sensors_Array[21].sensor_taxels[24].vector.x = 0.007542;
    Sensors_Array[21].sensor_taxels[24].vector.y = -0.007143;
    Sensors_Array[21].sensor_taxels[24].vector.z = 0.032078;
    Sensors_Array[21].sensor_taxels[25].vector.x = -0.015341;
    Sensors_Array[21].sensor_taxels[25].vector.y = -0.007143;
    Sensors_Array[21].sensor_taxels[25].vector.z = 0.030076;
    Sensors_Array[21].sensor_taxels[26].vector.x = -0.022805;
    Sensors_Array[21].sensor_taxels[26].vector.y = -0.007143;
    Sensors_Array[21].sensor_taxels[26].vector.z = 0.029423;
    //Middle Medial Front

    //Row1
    Sensors_Array[4].sensor_taxels[0].vector.x = 0.018017;
    Sensors_Array[4].sensor_taxels[0].vector.y = 0.01202;
    Sensors_Array[4].sensor_taxels[0].vector.z = 0.011304;
    Sensors_Array[4].sensor_taxels[1].vector.x = 0.009442;
    Sensors_Array[4].sensor_taxels[1].vector.y = 0.01202;
    Sensors_Array[4].sensor_taxels[1].vector.z = 0.01053;
    Sensors_Array[4].sensor_taxels[2].vector.x = 0.003686;
    Sensors_Array[4].sensor_taxels[2].vector.y = 0.01202;
    Sensors_Array[4].sensor_taxels[2].vector.z = 0.010026;
    Sensors_Array[4].sensor_taxels[3].vector.x = -0.001835;
    Sensors_Array[4].sensor_taxels[3].vector.y = 0.01202;
    Sensors_Array[4].sensor_taxels[3].vector.z = 0.009543;
    //Row2
    Sensors_Array[4].sensor_taxels[4].vector.x = 0.017651;
    Sensors_Array[4].sensor_taxels[4].vector.y = 0.012084;
    Sensors_Array[4].sensor_taxels[4].vector.z = 0.01549;
    Sensors_Array[4].sensor_taxels[5].vector.x = 0.009076;
    Sensors_Array[4].sensor_taxels[5].vector.y = 0.012084;
    Sensors_Array[4].sensor_taxels[5].vector.z = 0.014715;
    Sensors_Array[4].sensor_taxels[6].vector.x = 0.00332;
    Sensors_Array[4].sensor_taxels[6].vector.y = 0.012084;
    Sensors_Array[4].sensor_taxels[6].vector.z = 0.014212;
    Sensors_Array[4].sensor_taxels[7].vector.x = -0.002201;
    Sensors_Array[4].sensor_taxels[7].vector.y = 0.012084;
    Sensors_Array[4].sensor_taxels[7].vector.z = 0.013729;
    //Row3
    Sensors_Array[4].sensor_taxels[8].vector.x = 0.017397;
    Sensors_Array[4].sensor_taxels[8].vector.y = 0.012048;
    Sensors_Array[4].sensor_taxels[8].vector.z = 0.019322;
    Sensors_Array[4].sensor_taxels[9].vector.x = 0.008742;
    Sensors_Array[4].sensor_taxels[9].vector.y = 0.012048;
    Sensors_Array[4].sensor_taxels[9].vector.z = 0.01854;
    Sensors_Array[4].sensor_taxels[10].vector.x = 0.002985;
    Sensors_Array[4].sensor_taxels[10].vector.y = 0.012048;
    Sensors_Array[4].sensor_taxels[10].vector.z = 0.018036;
    Sensors_Array[4].sensor_taxels[11].vector.x = -0.002535;
    Sensors_Array[4].sensor_taxels[11].vector.y = 0.012048;
    Sensors_Array[4].sensor_taxels[11].vector.z = 0.017554;
    //Row4
    Sensors_Array[4].sensor_taxels[12].vector.x = 0.008212;
    Sensors_Array[4].sensor_taxels[12].vector.y = 0.011029;
    Sensors_Array[4].sensor_taxels[12].vector.z = 0.024592;
    Sensors_Array[4].sensor_taxels[13].vector.x = 0.002456;
    Sensors_Array[4].sensor_taxels[13].vector.y = 0.011029;
    Sensors_Array[4].sensor_taxels[13].vector.z = 0.024089;
    Sensors_Array[4].sensor_taxels[14].vector.x = -0.003064;
    Sensors_Array[4].sensor_taxels[14].vector.y = 0.011029;
    Sensors_Array[4].sensor_taxels[14].vector.z = 0.023606;
    //Row5
    Sensors_Array[4].sensor_taxels[15].vector.x = 0.007803;
    Sensors_Array[4].sensor_taxels[15].vector.y = 0.009308;
    Sensors_Array[4].sensor_taxels[15].vector.z = 0.029273;
    Sensors_Array[4].sensor_taxels[16].vector.x = 0.002047;
    Sensors_Array[4].sensor_taxels[16].vector.y = 0.009308;
    Sensors_Array[4].sensor_taxels[16].vector.z = 0.028769;
    Sensors_Array[4].sensor_taxels[17].vector.x = -0.003474;
    Sensors_Array[4].sensor_taxels[17].vector.y = 0.009308;
    Sensors_Array[4].sensor_taxels[17].vector.z = 0.028287;
    //Row6
    Sensors_Array[4].sensor_taxels[18].vector.x = 0.007614;
    Sensors_Array[4].sensor_taxels[18].vector.y = 0.004825;
    Sensors_Array[4].sensor_taxels[18].vector.z = 0.031433;
    Sensors_Array[4].sensor_taxels[19].vector.x = 0.001858;
    Sensors_Array[4].sensor_taxels[19].vector.y = 0.004825;
    Sensors_Array[4].sensor_taxels[19].vector.z = 0.03093;
    Sensors_Array[4].sensor_taxels[20].vector.x = -0.003662;
    Sensors_Array[4].sensor_taxels[20].vector.y = 0.004825;
    Sensors_Array[4].sensor_taxels[20].vector.z = 0.030447;
    //Row7
    Sensors_Array[4].sensor_taxels[21].vector.x = 0.007562;
    Sensors_Array[4].sensor_taxels[21].vector.y = -0.000529;
    Sensors_Array[4].sensor_taxels[21].vector.z = 0.032032;
    Sensors_Array[4].sensor_taxels[22].vector.x = 0.001806;
    Sensors_Array[4].sensor_taxels[22].vector.y = -0.000529;
    Sensors_Array[4].sensor_taxels[22].vector.z = 0.031528;
    Sensors_Array[4].sensor_taxels[23].vector.x = -0.003715;
    Sensors_Array[4].sensor_taxels[23].vector.y = -0.000529;
    Sensors_Array[4].sensor_taxels[23].vector.z = 0.031045;
    //Row8
    Sensors_Array[4].sensor_taxels[24].vector.x = 0.007542;
    Sensors_Array[4].sensor_taxels[24].vector.y = -0.007143;
    Sensors_Array[4].sensor_taxels[24].vector.z = 0.032078;
    Sensors_Array[4].sensor_taxels[25].vector.x = -0.015341;
    Sensors_Array[4].sensor_taxels[25].vector.y = -0.007143;
    Sensors_Array[4].sensor_taxels[25].vector.z = 0.030076;
    Sensors_Array[4].sensor_taxels[26].vector.x = -0.022805;
    Sensors_Array[4].sensor_taxels[26].vector.y = -0.007143;
    Sensors_Array[4].sensor_taxels[26].vector.z = 0.029423;
    //Pinky Medial Front

    //Row1
    Sensors_Array[16].sensor_taxels[0].vector.x = 0.018017;
    Sensors_Array[16].sensor_taxels[0].vector.y = 0.01202;
    Sensors_Array[16].sensor_taxels[0].vector.z = 0.011304;
    Sensors_Array[16].sensor_taxels[1].vector.x = 0.009442;
    Sensors_Array[16].sensor_taxels[1].vector.y = 0.01202;
    Sensors_Array[16].sensor_taxels[1].vector.z = 0.01053;
    Sensors_Array[16].sensor_taxels[2].vector.x = 0.003686;
    Sensors_Array[16].sensor_taxels[2].vector.y = 0.01202;
    Sensors_Array[16].sensor_taxels[2].vector.z = 0.010026;
    Sensors_Array[16].sensor_taxels[3].vector.x = -0.001835;
    Sensors_Array[16].sensor_taxels[3].vector.y = 0.01202;
    Sensors_Array[16].sensor_taxels[3].vector.z = 0.009543;
    //Row2
    Sensors_Array[16].sensor_taxels[4].vector.x = 0.017651;
    Sensors_Array[16].sensor_taxels[4].vector.y = 0.012084;
    Sensors_Array[16].sensor_taxels[4].vector.z = 0.01549;
    Sensors_Array[16].sensor_taxels[5].vector.x = 0.009076;
    Sensors_Array[16].sensor_taxels[5].vector.y = 0.012084;
    Sensors_Array[16].sensor_taxels[5].vector.z = 0.014715;
    Sensors_Array[16].sensor_taxels[6].vector.x = 0.00332;
    Sensors_Array[16].sensor_taxels[6].vector.y = 0.012084;
    Sensors_Array[16].sensor_taxels[6].vector.z = 0.014212;
    Sensors_Array[16].sensor_taxels[7].vector.x = -0.002201;
    Sensors_Array[16].sensor_taxels[7].vector.y = 0.012084;
    Sensors_Array[16].sensor_taxels[7].vector.z = 0.013729;
    //Row3
    Sensors_Array[16].sensor_taxels[8].vector.x = 0.017397;
    Sensors_Array[16].sensor_taxels[8].vector.y = 0.012048;
    Sensors_Array[16].sensor_taxels[8].vector.z = 0.019322;
    Sensors_Array[16].sensor_taxels[9].vector.x = 0.008742;
    Sensors_Array[16].sensor_taxels[9].vector.y = 0.012048;
    Sensors_Array[16].sensor_taxels[9].vector.z = 0.01854;
    Sensors_Array[16].sensor_taxels[10].vector.x = 0.002985;
    Sensors_Array[16].sensor_taxels[10].vector.y = 0.012048;
    Sensors_Array[16].sensor_taxels[10].vector.z = 0.018036;
    Sensors_Array[16].sensor_taxels[11].vector.x = -0.002535;
    Sensors_Array[16].sensor_taxels[11].vector.y = 0.012048;
    Sensors_Array[16].sensor_taxels[11].vector.z = 0.017554;
    //Row4
    Sensors_Array[16].sensor_taxels[12].vector.x = 0.008212;
    Sensors_Array[16].sensor_taxels[12].vector.y = 0.011029;
    Sensors_Array[16].sensor_taxels[12].vector.z = 0.024592;
    Sensors_Array[16].sensor_taxels[13].vector.x = 0.002456;
    Sensors_Array[16].sensor_taxels[13].vector.y = 0.011029;
    Sensors_Array[16].sensor_taxels[13].vector.z = 0.024089;
    Sensors_Array[16].sensor_taxels[14].vector.x = -0.003064;
    Sensors_Array[16].sensor_taxels[14].vector.y = 0.011029;
    Sensors_Array[16].sensor_taxels[14].vector.z = 0.023606;
    //Row5
    Sensors_Array[16].sensor_taxels[15].vector.x = 0.007803;
    Sensors_Array[16].sensor_taxels[15].vector.y = 0.009308;
    Sensors_Array[16].sensor_taxels[15].vector.z = 0.029273;
    Sensors_Array[16].sensor_taxels[16].vector.x = 0.002047;
    Sensors_Array[16].sensor_taxels[16].vector.y = 0.009308;
    Sensors_Array[16].sensor_taxels[16].vector.z = 0.028769;
    Sensors_Array[16].sensor_taxels[17].vector.x = -0.003474;
    Sensors_Array[16].sensor_taxels[17].vector.y = 0.009308;
    Sensors_Array[16].sensor_taxels[17].vector.z = 0.028287;
    //Row6
    Sensors_Array[16].sensor_taxels[18].vector.x = 0.007614;
    Sensors_Array[16].sensor_taxels[18].vector.y = 0.004825;
    Sensors_Array[16].sensor_taxels[18].vector.z = 0.031433;
    Sensors_Array[16].sensor_taxels[19].vector.x = 0.001858;
    Sensors_Array[16].sensor_taxels[19].vector.y = 0.004825;
    Sensors_Array[16].sensor_taxels[19].vector.z = 0.03093;
    Sensors_Array[16].sensor_taxels[20].vector.x = -0.003662;
    Sensors_Array[16].sensor_taxels[20].vector.y = 0.004825;
    Sensors_Array[16].sensor_taxels[20].vector.z = 0.030447;
    //Row7
    Sensors_Array[16].sensor_taxels[21].vector.x = 0.007562;
    Sensors_Array[16].sensor_taxels[21].vector.y = -0.000529;
    Sensors_Array[16].sensor_taxels[21].vector.z = 0.032032;
    Sensors_Array[16].sensor_taxels[22].vector.x = 0.001806;
    Sensors_Array[16].sensor_taxels[22].vector.y = -0.000529;
    Sensors_Array[16].sensor_taxels[22].vector.z = 0.031528;
    Sensors_Array[16].sensor_taxels[23].vector.x = -0.003715;
    Sensors_Array[16].sensor_taxels[23].vector.y = -0.000529;
    Sensors_Array[16].sensor_taxels[23].vector.z = 0.031045;
    //Row8
    Sensors_Array[16].sensor_taxels[24].vector.x = 0.007542;
    Sensors_Array[16].sensor_taxels[24].vector.y = -0.007143;
    Sensors_Array[16].sensor_taxels[24].vector.z = 0.032078;
    Sensors_Array[16].sensor_taxels[25].vector.x = -0.015341;
    Sensors_Array[16].sensor_taxels[25].vector.y = -0.007143;
    Sensors_Array[16].sensor_taxels[25].vector.z = 0.030076;
    Sensors_Array[16].sensor_taxels[26].vector.x = -0.022805;
    Sensors_Array[16].sensor_taxels[26].vector.y = -0.007143;
    Sensors_Array[16].sensor_taxels[26].vector.z = 0.029423;
    //Thumb Fingertip

    //Straight wrapp up
    //Row1
    Sensors_Array[8].sensor_taxels[0].vector.x = 0.015337;
    Sensors_Array[8].sensor_taxels[0].vector.y = -0.006754;
    Sensors_Array[8].sensor_taxels[0].vector.z = -0.00513;
    Sensors_Array[8].sensor_taxels[1].vector.x = 0.015239;
    Sensors_Array[8].sensor_taxels[1].vector.y = 0.001668;
    Sensors_Array[8].sensor_taxels[1].vector.z = -0.005106;
    Sensors_Array[8].sensor_taxels[2].vector.x = 0.015289;
    Sensors_Array[8].sensor_taxels[2].vector.y = 0.010175;
    Sensors_Array[8].sensor_taxels[2].vector.z = -0.004527;
    Sensors_Array[8].sensor_taxels[3].vector.x = 0.016136;
    Sensors_Array[8].sensor_taxels[3].vector.y = 0.0139;
    Sensors_Array[8].sensor_taxels[3].vector.z = 0.00504;
    Sensors_Array[8].sensor_taxels[4].vector.x = 0.016712;
    Sensors_Array[8].sensor_taxels[4].vector.y = 0.0139;
    Sensors_Array[8].sensor_taxels[4].vector.z = 0.011619;
    Sensors_Array[8].sensor_taxels[5].vector.x = 0.017287;
    Sensors_Array[8].sensor_taxels[5].vector.y = 0.0139;
    Sensors_Array[8].sensor_taxels[5].vector.z = 0.018198;
    Sensors_Array[8].sensor_taxels[6].vector.x = 0.018274;
    Sensors_Array[8].sensor_taxels[6].vector.y = 0.011735;
    Sensors_Array[8].sensor_taxels[6].vector.z = 0.028435;
    Sensors_Array[8].sensor_taxels[7].vector.x = 0.018342;
    Sensors_Array[8].sensor_taxels[7].vector.y = 0.00314;
    Sensors_Array[8].sensor_taxels[7].vector.z = 0.029222;
    Sensors_Array[8].sensor_taxels[8].vector.x = 0.018251;
    Sensors_Array[8].sensor_taxels[8].vector.y = -0.005069;
    Sensors_Array[8].sensor_taxels[8].vector.z = 0.02932;
    //Row2
    Sensors_Array[8].sensor_taxels[9].vector.x = 0.021649;
    Sensors_Array[8].sensor_taxels[9].vector.y = -0.006754;
    Sensors_Array[8].sensor_taxels[9].vector.z = -0.005683;
    Sensors_Array[8].sensor_taxels[10].vector.x = 0.021551;
    Sensors_Array[8].sensor_taxels[10].vector.y = 0.001668;
    Sensors_Array[8].sensor_taxels[10].vector.z = -0.005659;
    Sensors_Array[8].sensor_taxels[11].vector.x = 0.021601;
    Sensors_Array[8].sensor_taxels[11].vector.y = 0.010175;
    Sensors_Array[8].sensor_taxels[11].vector.z = -0.005079;
    Sensors_Array[8].sensor_taxels[12].vector.x = 0.022448;
    Sensors_Array[8].sensor_taxels[12].vector.y = 0.0139;
    Sensors_Array[8].sensor_taxels[12].vector.z = 0.004488;
    Sensors_Array[8].sensor_taxels[13].vector.x = 0.023024;
    Sensors_Array[8].sensor_taxels[13].vector.y = 0.0139;
    Sensors_Array[8].sensor_taxels[13].vector.z = 0.011067;
    Sensors_Array[8].sensor_taxels[14].vector.x = 0.023599;
    Sensors_Array[8].sensor_taxels[14].vector.y = 0.0139;
    Sensors_Array[8].sensor_taxels[14].vector.z = 0.017646;
    Sensors_Array[8].sensor_taxels[15].vector.x = 0.024586;
    Sensors_Array[8].sensor_taxels[15].vector.y = 0.011735;
    Sensors_Array[8].sensor_taxels[15].vector.z = 0.027883;
    Sensors_Array[8].sensor_taxels[16].vector.x = 0.024654;
    Sensors_Array[8].sensor_taxels[16].vector.y = 0.00314;
    Sensors_Array[8].sensor_taxels[16].vector.z = 0.028669;
    Sensors_Array[8].sensor_taxels[17].vector.x = 0.024563;
    Sensors_Array[8].sensor_taxels[17].vector.y = -0.005069;
    Sensors_Array[8].sensor_taxels[17].vector.z = 0.028768;
    //Row3
    Sensors_Array[8].sensor_taxels[18].vector.x = 0.027724;
    Sensors_Array[8].sensor_taxels[18].vector.y = -0.001668;
    Sensors_Array[8].sensor_taxels[18].vector.z = -0.006199;
    Sensors_Array[8].sensor_taxels[19].vector.x = 0.027775;
    Sensors_Array[8].sensor_taxels[19].vector.y = 0.010175;
    Sensors_Array[8].sensor_taxels[19].vector.z = -0.005619;
    Sensors_Array[8].sensor_taxels[20].vector.x = 0.028622;
    Sensors_Array[8].sensor_taxels[20].vector.y = 0.0139;
    Sensors_Array[8].sensor_taxels[20].vector.z = 0.003948;
    Sensors_Array[8].sensor_taxels[21].vector.x = 0.029197;
    Sensors_Array[8].sensor_taxels[21].vector.y = 0.0139;
    Sensors_Array[8].sensor_taxels[21].vector.z = 0.010527;
    Sensors_Array[8].sensor_taxels[22].vector.x = 0.029773;
    Sensors_Array[8].sensor_taxels[22].vector.y = 0.0139;
    Sensors_Array[8].sensor_taxels[22].vector.z = 0.017106;
    Sensors_Array[8].sensor_taxels[23].vector.x = 0.030759;
    Sensors_Array[8].sensor_taxels[23].vector.y = 0.011735;
    Sensors_Array[8].sensor_taxels[23].vector.z = 0.027343;
    Sensors_Array[8].sensor_taxels[24].vector.x = 0.030828;
    Sensors_Array[8].sensor_taxels[24].vector.y = 0.00314;
    Sensors_Array[8].sensor_taxels[24].vector.z = 0.028129;
    //Row4
    Sensors_Array[8].sensor_taxels[25].vector.x = 0.033486;
    Sensors_Array[8].sensor_taxels[25].vector.y = 0.010175;
    Sensors_Array[8].sensor_taxels[25].vector.z = -0.006199;
    Sensors_Array[8].sensor_taxels[26].vector.x = 0.034333;
    Sensors_Array[8].sensor_taxels[26].vector.y = 0.0139;
    Sensors_Array[8].sensor_taxels[26].vector.z = 0.003448;
    Sensors_Array[8].sensor_taxels[27].vector.x = 0.034909;
    Sensors_Array[8].sensor_taxels[27].vector.y = 0.0139;
    Sensors_Array[8].sensor_taxels[27].vector.z = 0.010027;
    Sensors_Array[8].sensor_taxels[28].vector.x = 0.035484;
    Sensors_Array[8].sensor_taxels[28].vector.y = 0.0139;
    Sensors_Array[8].sensor_taxels[28].vector.z = 0.016606;
    Sensors_Array[8].sensor_taxels[29].vector.x = 0.03647;
    Sensors_Array[8].sensor_taxels[29].vector.y = 0.011735;
    Sensors_Array[8].sensor_taxels[29].vector.z = 0.026843;
    // Curved Face
    //Row1
    Sensors_Array[8].sensor_taxels[30].vector.x = 0.043079;
    Sensors_Array[8].sensor_taxels[30].vector.y = 0.013442;
    Sensors_Array[8].sensor_taxels[30].vector.z = -0.000058;
    Sensors_Array[8].sensor_taxels[31].vector.x = 0.043633;
    Sensors_Array[8].sensor_taxels[31].vector.y = 0.013442;
    Sensors_Array[8].sensor_taxels[31].vector.z = 0.006267;
    Sensors_Array[8].sensor_taxels[32].vector.x = 0.044186;
    Sensors_Array[8].sensor_taxels[32].vector.y = 0.013442;
    Sensors_Array[8].sensor_taxels[32].vector.z = 0.012593;
    Sensors_Array[8].sensor_taxels[33].vector.x = 0.04474;
    Sensors_Array[8].sensor_taxels[33].vector.y = 0.013442;
    Sensors_Array[8].sensor_taxels[33].vector.z = 0.018919;
    //Row2
    Sensors_Array[8].sensor_taxels[34].vector.x = 0.04288;
    Sensors_Array[8].sensor_taxels[34].vector.y = 0.007274;
    Sensors_Array[8].sensor_taxels[34].vector.z = -0.000041;
    Sensors_Array[8].sensor_taxels[35].vector.x = 0.043434;
    Sensors_Array[8].sensor_taxels[35].vector.y = 0.007274;
    Sensors_Array[8].sensor_taxels[35].vector.z = 0.006285;
    Sensors_Array[8].sensor_taxels[36].vector.x = 0.043987;
    Sensors_Array[8].sensor_taxels[36].vector.y = 0.007274;
    Sensors_Array[8].sensor_taxels[36].vector.z = 0.012611;
    Sensors_Array[8].sensor_taxels[37].vector.x = 0.044541;
    Sensors_Array[8].sensor_taxels[37].vector.y = 0.007274;
    Sensors_Array[8].sensor_taxels[37].vector.z = 0.018936;
    //Row3
    Sensors_Array[8].sensor_taxels[38].vector.x = 0.040359;
    Sensors_Array[8].sensor_taxels[38].vector.y = 0.002132;
    Sensors_Array[8].sensor_taxels[38].vector.z = -0.0000482;
    Sensors_Array[8].sensor_taxels[39].vector.x = 0.040912;
    Sensors_Array[8].sensor_taxels[39].vector.y = 0.002132;
    Sensors_Array[8].sensor_taxels[39].vector.z = 0.006808;
    Sensors_Array[8].sensor_taxels[40].vector.x = 0.041465;
    Sensors_Array[8].sensor_taxels[40].vector.y = 0.002132;
    Sensors_Array[8].sensor_taxels[40].vector.z = 0.013133;
    Sensors_Array[8].sensor_taxels[41].vector.x = 0.042019;
    Sensors_Array[8].sensor_taxels[41].vector.y = 0.002132;
    Sensors_Array[8].sensor_taxels[41].vector.z = 0.019459;
    //Row4
    Sensors_Array[8].sensor_taxels[42].vector.x = 0.037108;
    Sensors_Array[8].sensor_taxels[42].vector.y = -0.002795;
    Sensors_Array[8].sensor_taxels[42].vector.z = -0.000766;
    Sensors_Array[8].sensor_taxels[43].vector.x = 0.037662;
    Sensors_Array[8].sensor_taxels[43].vector.y = -0.002795;
    Sensors_Array[8].sensor_taxels[43].vector.z = 0.007092;
    Sensors_Array[8].sensor_taxels[44].vector.x = 0.038215;
    Sensors_Array[8].sensor_taxels[44].vector.y = -0.002795;
    Sensors_Array[8].sensor_taxels[44].vector.z = 0.013418;
    Sensors_Array[8].sensor_taxels[45].vector.x = 0.038769;
    Sensors_Array[8].sensor_taxels[45].vector.y = -0.002795;
    Sensors_Array[8].sensor_taxels[45].vector.z = 0.019744;
    //Row5
    Sensors_Array[8].sensor_taxels[46].vector.x = 0.033168;
    Sensors_Array[8].sensor_taxels[46].vector.y = -0.007189;
    Sensors_Array[8].sensor_taxels[46].vector.z = 0.001111;
    Sensors_Array[8].sensor_taxels[47].vector.x = 0.033721;
    Sensors_Array[8].sensor_taxels[47].vector.y = -0.007189;
    Sensors_Array[8].sensor_taxels[47].vector.z = 0.007437;
    Sensors_Array[8].sensor_taxels[48].vector.x = 0.034275;
    Sensors_Array[8].sensor_taxels[48].vector.y = -0.007189;
    Sensors_Array[8].sensor_taxels[48].vector.z = 0.013762;
    Sensors_Array[8].sensor_taxels[49].vector.x = 0.034828;
    Sensors_Array[8].sensor_taxels[49].vector.y = -0.007189;
    Sensors_Array[8].sensor_taxels[49].vector.z = 0.020088;
    //Row6
    Sensors_Array[8].sensor_taxels[50].vector.x = 0.028594;
    Sensors_Array[8].sensor_taxels[50].vector.y = -0.010913;
    Sensors_Array[8].sensor_taxels[50].vector.z = 0.001511;
    Sensors_Array[8].sensor_taxels[51].vector.x = 0.029148;
    Sensors_Array[8].sensor_taxels[51].vector.y = -0.010913;
    Sensors_Array[8].sensor_taxels[51].vector.z = 0.007837;
    Sensors_Array[8].sensor_taxels[52].vector.x = 0.029701;
    Sensors_Array[8].sensor_taxels[52].vector.y = -0.010913;
    Sensors_Array[8].sensor_taxels[52].vector.z = 0.014163;
    Sensors_Array[8].sensor_taxels[53].vector.x = 0.030255;
    Sensors_Array[8].sensor_taxels[53].vector.y = -0.010913;
    Sensors_Array[8].sensor_taxels[53].vector.z = 0.020488;
    //Row7
    Sensors_Array[8].sensor_taxels[54].vector.x = 0.023521;
    Sensors_Array[8].sensor_taxels[54].vector.y = -0.01392;
    Sensors_Array[8].sensor_taxels[54].vector.z = 0.001955;
    Sensors_Array[8].sensor_taxels[55].vector.x = 0.024075;
    Sensors_Array[8].sensor_taxels[55].vector.y = -0.01392;
    Sensors_Array[8].sensor_taxels[55].vector.z = 0.008281;
    Sensors_Array[8].sensor_taxels[56].vector.x = 0.024628;
    Sensors_Array[8].sensor_taxels[56].vector.y = -0.01392;
    Sensors_Array[8].sensor_taxels[56].vector.z = 0.014606;
    Sensors_Array[8].sensor_taxels[57].vector.x = 0.025181;
    Sensors_Array[8].sensor_taxels[57].vector.y = -0.01392;
    Sensors_Array[8].sensor_taxels[57].vector.z = 0.020932;
    //Row8
    Sensors_Array[8].sensor_taxels[58].vector.x = 0.018044;
    Sensors_Array[8].sensor_taxels[58].vector.y = -0.016096;
    Sensors_Array[8].sensor_taxels[58].vector.z = 0.002434;
    Sensors_Array[8].sensor_taxels[59].vector.x = 0.018598;
    Sensors_Array[8].sensor_taxels[59].vector.y = -0.016096;
    Sensors_Array[8].sensor_taxels[59].vector.z = 0.00876;
    Sensors_Array[8].sensor_taxels[60].vector.x = 0.019151;
    Sensors_Array[8].sensor_taxels[60].vector.y = -0.016096;
    Sensors_Array[8].sensor_taxels[60].vector.z = 0.015086;
    Sensors_Array[8].sensor_taxels[61].vector.x = 0.019705;
    Sensors_Array[8].sensor_taxels[61].vector.y = -0.016096;
    Sensors_Array[8].sensor_taxels[61].vector.z = 0.021411;
    //Row9
    Sensors_Array[8].sensor_taxels[62].vector.x = 0.012303;
    Sensors_Array[8].sensor_taxels[62].vector.y = -0.017413;
    Sensors_Array[8].sensor_taxels[62].vector.z = 0.002936;
    Sensors_Array[8].sensor_taxels[63].vector.x = 0.012857;
    Sensors_Array[8].sensor_taxels[63].vector.y = -0.017413;
    Sensors_Array[8].sensor_taxels[63].vector.z = 0.009262;
    Sensors_Array[8].sensor_taxels[64].vector.x = 0.01341;
    Sensors_Array[8].sensor_taxels[64].vector.y = -0.017413;
    Sensors_Array[8].sensor_taxels[64].vector.z = 0.015588;
    Sensors_Array[8].sensor_taxels[65].vector.x = 0.013963;
    Sensors_Array[8].sensor_taxels[65].vector.y = -0.017413;
    Sensors_Array[8].sensor_taxels[65].vector.z = 0.021914;

    //Index Fingertip

    //Straight wrapp up
    //Row1
    Sensors_Array[20].sensor_taxels[0].vector.x = 0.015337;
    Sensors_Array[20].sensor_taxels[0].vector.y = -0.006754;
    Sensors_Array[20].sensor_taxels[0].vector.z = -0.00513;
    Sensors_Array[20].sensor_taxels[1].vector.x = 0.015239;
    Sensors_Array[20].sensor_taxels[1].vector.y = 0.001668;
    Sensors_Array[20].sensor_taxels[1].vector.z = -0.005106;
    Sensors_Array[20].sensor_taxels[2].vector.x = 0.015289;
    Sensors_Array[20].sensor_taxels[2].vector.y = 0.010175;
    Sensors_Array[20].sensor_taxels[2].vector.z = -0.004527;
    Sensors_Array[20].sensor_taxels[3].vector.x = 0.016136;
    Sensors_Array[20].sensor_taxels[3].vector.y = 0.0139;
    Sensors_Array[20].sensor_taxels[3].vector.z = 0.00504;
    Sensors_Array[20].sensor_taxels[4].vector.x = 0.016712;
    Sensors_Array[20].sensor_taxels[4].vector.y = 0.0139;
    Sensors_Array[20].sensor_taxels[4].vector.z = 0.011619;
    Sensors_Array[20].sensor_taxels[5].vector.x = 0.017287;
    Sensors_Array[20].sensor_taxels[5].vector.y = 0.0139;
    Sensors_Array[20].sensor_taxels[5].vector.z = 0.018198;
    Sensors_Array[20].sensor_taxels[6].vector.x = 0.018274;
    Sensors_Array[20].sensor_taxels[6].vector.y = 0.011735;
    Sensors_Array[20].sensor_taxels[6].vector.z = 0.028435;
    Sensors_Array[20].sensor_taxels[7].vector.x = 0.018342;
    Sensors_Array[20].sensor_taxels[7].vector.y = 0.00314;
    Sensors_Array[20].sensor_taxels[7].vector.z = 0.029222;
    Sensors_Array[20].sensor_taxels[8].vector.x = 0.018251;
    Sensors_Array[20].sensor_taxels[8].vector.y = -0.005069;
    Sensors_Array[20].sensor_taxels[8].vector.z = 0.02932;
    //Row2
    Sensors_Array[20].sensor_taxels[9].vector.x = 0.021649;
    Sensors_Array[20].sensor_taxels[9].vector.y = -0.006754;
    Sensors_Array[20].sensor_taxels[9].vector.z = -0.005683;
    Sensors_Array[20].sensor_taxels[10].vector.x = 0.021551;
    Sensors_Array[20].sensor_taxels[10].vector.y = 0.001668;
    Sensors_Array[20].sensor_taxels[10].vector.z = -0.005659;
    Sensors_Array[20].sensor_taxels[11].vector.x = 0.021601;
    Sensors_Array[20].sensor_taxels[11].vector.y = 0.010175;
    Sensors_Array[20].sensor_taxels[11].vector.z = -0.005079;
    Sensors_Array[20].sensor_taxels[12].vector.x = 0.022448;
    Sensors_Array[20].sensor_taxels[12].vector.y = 0.0139;
    Sensors_Array[20].sensor_taxels[12].vector.z = 0.004488;
    Sensors_Array[20].sensor_taxels[13].vector.x = 0.023024;
    Sensors_Array[20].sensor_taxels[13].vector.y = 0.0139;
    Sensors_Array[20].sensor_taxels[13].vector.z = 0.011067;
    Sensors_Array[20].sensor_taxels[14].vector.x = 0.023599;
    Sensors_Array[20].sensor_taxels[14].vector.y = 0.0139;
    Sensors_Array[20].sensor_taxels[14].vector.z = 0.017646;
    Sensors_Array[20].sensor_taxels[15].vector.x = 0.024586;
    Sensors_Array[20].sensor_taxels[15].vector.y = 0.011735;
    Sensors_Array[20].sensor_taxels[15].vector.z = 0.027883;
    Sensors_Array[20].sensor_taxels[16].vector.x = 0.024654;
    Sensors_Array[20].sensor_taxels[16].vector.y = 0.00314;
    Sensors_Array[20].sensor_taxels[16].vector.z = 0.028669;
    Sensors_Array[20].sensor_taxels[17].vector.x = 0.024563;
    Sensors_Array[20].sensor_taxels[17].vector.y = -0.005069;
    Sensors_Array[20].sensor_taxels[17].vector.z = 0.028768;
    //Row3
    Sensors_Array[20].sensor_taxels[18].vector.x = 0.027724;
    Sensors_Array[20].sensor_taxels[18].vector.y = -0.001668;
    Sensors_Array[20].sensor_taxels[18].vector.z = -0.006199;
    Sensors_Array[20].sensor_taxels[19].vector.x = 0.027775;
    Sensors_Array[20].sensor_taxels[19].vector.y = 0.010175;
    Sensors_Array[20].sensor_taxels[19].vector.z = -0.005619;
    Sensors_Array[20].sensor_taxels[20].vector.x = 0.028622;
    Sensors_Array[20].sensor_taxels[20].vector.y = 0.0139;
    Sensors_Array[20].sensor_taxels[20].vector.z = 0.003948;
    Sensors_Array[20].sensor_taxels[21].vector.x = 0.029197;
    Sensors_Array[20].sensor_taxels[21].vector.y = 0.0139;
    Sensors_Array[20].sensor_taxels[21].vector.z = 0.010527;
    Sensors_Array[20].sensor_taxels[22].vector.x = 0.029773;
    Sensors_Array[20].sensor_taxels[22].vector.y = 0.0139;
    Sensors_Array[20].sensor_taxels[22].vector.z = 0.017106;
    Sensors_Array[20].sensor_taxels[23].vector.x = 0.030759;
    Sensors_Array[20].sensor_taxels[23].vector.y = 0.011735;
    Sensors_Array[20].sensor_taxels[23].vector.z = 0.027343;
    Sensors_Array[20].sensor_taxels[24].vector.x = 0.030828;
    Sensors_Array[20].sensor_taxels[24].vector.y = 0.00314;
    Sensors_Array[20].sensor_taxels[24].vector.z = 0.028129;
    //Row4
    Sensors_Array[20].sensor_taxels[25].vector.x = 0.033486;
    Sensors_Array[20].sensor_taxels[25].vector.y = 0.010175;
    Sensors_Array[20].sensor_taxels[25].vector.z = -0.006199;
    Sensors_Array[20].sensor_taxels[26].vector.x = 0.034333;
    Sensors_Array[20].sensor_taxels[26].vector.y = 0.0139;
    Sensors_Array[20].sensor_taxels[26].vector.z = 0.003448;
    Sensors_Array[20].sensor_taxels[27].vector.x = 0.034909;
    Sensors_Array[20].sensor_taxels[27].vector.y = 0.0139;
    Sensors_Array[20].sensor_taxels[27].vector.z = 0.010027;
    Sensors_Array[20].sensor_taxels[28].vector.x = 0.035484;
    Sensors_Array[20].sensor_taxels[28].vector.y = 0.0139;
    Sensors_Array[20].sensor_taxels[28].vector.z = 0.016606;
    Sensors_Array[20].sensor_taxels[29].vector.x = 0.03647;
    Sensors_Array[20].sensor_taxels[29].vector.y = 0.011735;
    Sensors_Array[20].sensor_taxels[29].vector.z = 0.026843;
    // Curved Face
    //Row1
    Sensors_Array[20].sensor_taxels[30].vector.x = 0.043079;
    Sensors_Array[20].sensor_taxels[30].vector.y = 0.013442;
    Sensors_Array[20].sensor_taxels[30].vector.z = -0.000058;
    Sensors_Array[20].sensor_taxels[31].vector.x = 0.043633;
    Sensors_Array[20].sensor_taxels[31].vector.y = 0.013442;
    Sensors_Array[20].sensor_taxels[31].vector.z = 0.006267;
    Sensors_Array[20].sensor_taxels[32].vector.x = 0.044186;
    Sensors_Array[20].sensor_taxels[32].vector.y = 0.013442;
    Sensors_Array[20].sensor_taxels[32].vector.z = 0.012593;
    Sensors_Array[20].sensor_taxels[33].vector.x = 0.04474;
    Sensors_Array[20].sensor_taxels[33].vector.y = 0.013442;
    Sensors_Array[20].sensor_taxels[33].vector.z = 0.018919;
    //Row2
    Sensors_Array[20].sensor_taxels[34].vector.x = 0.04288;
    Sensors_Array[20].sensor_taxels[34].vector.y = 0.007274;
    Sensors_Array[20].sensor_taxels[34].vector.z = -0.000041;
    Sensors_Array[20].sensor_taxels[35].vector.x = 0.043434;
    Sensors_Array[20].sensor_taxels[35].vector.y = 0.007274;
    Sensors_Array[20].sensor_taxels[35].vector.z = 0.006285;
    Sensors_Array[20].sensor_taxels[36].vector.x = 0.043987;
    Sensors_Array[20].sensor_taxels[36].vector.y = 0.007274;
    Sensors_Array[20].sensor_taxels[36].vector.z = 0.012611;
    Sensors_Array[20].sensor_taxels[37].vector.x = 0.044541;
    Sensors_Array[20].sensor_taxels[37].vector.y = 0.007274;
    Sensors_Array[20].sensor_taxels[37].vector.z = 0.018936;
    //Row3
    Sensors_Array[20].sensor_taxels[38].vector.x = 0.040359;
    Sensors_Array[20].sensor_taxels[38].vector.y = 0.002132;
    Sensors_Array[20].sensor_taxels[38].vector.z = -0.0000482;
    Sensors_Array[20].sensor_taxels[39].vector.x = 0.040912;
    Sensors_Array[20].sensor_taxels[39].vector.y = 0.002132;
    Sensors_Array[20].sensor_taxels[39].vector.z = 0.006808;
    Sensors_Array[20].sensor_taxels[40].vector.x = 0.041465;
    Sensors_Array[20].sensor_taxels[40].vector.y = 0.002132;
    Sensors_Array[20].sensor_taxels[40].vector.z = 0.013133;
    Sensors_Array[20].sensor_taxels[41].vector.x = 0.042019;
    Sensors_Array[20].sensor_taxels[41].vector.y = 0.002132;
    Sensors_Array[20].sensor_taxels[41].vector.z = 0.019459;
    //Row4
    Sensors_Array[20].sensor_taxels[42].vector.x = 0.037108;
    Sensors_Array[20].sensor_taxels[42].vector.y = -0.002795;
    Sensors_Array[20].sensor_taxels[42].vector.z = -0.000766;
    Sensors_Array[20].sensor_taxels[43].vector.x = 0.037662;
    Sensors_Array[20].sensor_taxels[43].vector.y = -0.002795;
    Sensors_Array[20].sensor_taxels[43].vector.z = 0.007092;
    Sensors_Array[20].sensor_taxels[44].vector.x = 0.038215;
    Sensors_Array[20].sensor_taxels[44].vector.y = -0.002795;
    Sensors_Array[20].sensor_taxels[44].vector.z = 0.013418;
    Sensors_Array[20].sensor_taxels[45].vector.x = 0.038769;
    Sensors_Array[20].sensor_taxels[45].vector.y = -0.002795;
    Sensors_Array[20].sensor_taxels[45].vector.z = 0.019744;
    //Row5
    Sensors_Array[20].sensor_taxels[46].vector.x = 0.033168;
    Sensors_Array[20].sensor_taxels[46].vector.y = -0.007189;
    Sensors_Array[20].sensor_taxels[46].vector.z = 0.001111;
    Sensors_Array[20].sensor_taxels[47].vector.x = 0.033721;
    Sensors_Array[20].sensor_taxels[47].vector.y = -0.007189;
    Sensors_Array[20].sensor_taxels[47].vector.z = 0.007437;
    Sensors_Array[20].sensor_taxels[48].vector.x = 0.034275;
    Sensors_Array[20].sensor_taxels[48].vector.y = -0.007189;
    Sensors_Array[20].sensor_taxels[48].vector.z = 0.013762;
    Sensors_Array[20].sensor_taxels[49].vector.x = 0.034828;
    Sensors_Array[20].sensor_taxels[49].vector.y = -0.007189;
    Sensors_Array[20].sensor_taxels[49].vector.z = 0.020088;
    //Row6
    Sensors_Array[20].sensor_taxels[50].vector.x = 0.028594;
    Sensors_Array[20].sensor_taxels[50].vector.y = -0.010913;
    Sensors_Array[20].sensor_taxels[50].vector.z = 0.001511;
    Sensors_Array[20].sensor_taxels[51].vector.x = 0.029148;
    Sensors_Array[20].sensor_taxels[51].vector.y = -0.010913;
    Sensors_Array[20].sensor_taxels[51].vector.z = 0.007837;
    Sensors_Array[20].sensor_taxels[52].vector.x = 0.029701;
    Sensors_Array[20].sensor_taxels[52].vector.y = -0.010913;
    Sensors_Array[20].sensor_taxels[52].vector.z = 0.014163;
    Sensors_Array[20].sensor_taxels[53].vector.x = 0.030255;
    Sensors_Array[20].sensor_taxels[53].vector.y = -0.010913;
    Sensors_Array[20].sensor_taxels[53].vector.z = 0.020488;
    //Row7
    Sensors_Array[20].sensor_taxels[54].vector.x = 0.023521;
    Sensors_Array[20].sensor_taxels[54].vector.y = -0.01392;
    Sensors_Array[20].sensor_taxels[54].vector.z = 0.001955;
    Sensors_Array[20].sensor_taxels[55].vector.x = 0.024075;
    Sensors_Array[20].sensor_taxels[55].vector.y = -0.01392;
    Sensors_Array[20].sensor_taxels[55].vector.z = 0.008281;
    Sensors_Array[20].sensor_taxels[56].vector.x = 0.024628;
    Sensors_Array[20].sensor_taxels[56].vector.y = -0.01392;
    Sensors_Array[20].sensor_taxels[56].vector.z = 0.014606;
    Sensors_Array[20].sensor_taxels[57].vector.x = 0.025181;
    Sensors_Array[20].sensor_taxels[57].vector.y = -0.01392;
    Sensors_Array[20].sensor_taxels[57].vector.z = 0.020932;
    //Row8
    Sensors_Array[20].sensor_taxels[58].vector.x = 0.018044;
    Sensors_Array[20].sensor_taxels[58].vector.y = -0.016096;
    Sensors_Array[20].sensor_taxels[58].vector.z = 0.002434;
    Sensors_Array[20].sensor_taxels[59].vector.x = 0.018598;
    Sensors_Array[20].sensor_taxels[59].vector.y = -0.016096;
    Sensors_Array[20].sensor_taxels[59].vector.z = 0.00876;
    Sensors_Array[20].sensor_taxels[60].vector.x = 0.019151;
    Sensors_Array[20].sensor_taxels[60].vector.y = -0.016096;
    Sensors_Array[20].sensor_taxels[60].vector.z = 0.015086;
    Sensors_Array[20].sensor_taxels[61].vector.x = 0.019705;
    Sensors_Array[20].sensor_taxels[61].vector.y = -0.016096;
    Sensors_Array[20].sensor_taxels[61].vector.z = 0.021411;
    //Row9
    Sensors_Array[20].sensor_taxels[62].vector.x = 0.012303;
    Sensors_Array[20].sensor_taxels[62].vector.y = -0.017413;
    Sensors_Array[20].sensor_taxels[62].vector.z = 0.002936;
    Sensors_Array[20].sensor_taxels[63].vector.x = 0.012857;
    Sensors_Array[20].sensor_taxels[63].vector.y = -0.017413;
    Sensors_Array[20].sensor_taxels[63].vector.z = 0.009262;
    Sensors_Array[20].sensor_taxels[64].vector.x = 0.01341;
    Sensors_Array[20].sensor_taxels[64].vector.y = -0.017413;
    Sensors_Array[20].sensor_taxels[64].vector.z = 0.015588;
    Sensors_Array[20].sensor_taxels[65].vector.x = 0.013963;
    Sensors_Array[20].sensor_taxels[65].vector.y = -0.017413;
    Sensors_Array[20].sensor_taxels[65].vector.z = 0.021914;

    //Middle Fingertip

    //Straight wrapp up
    //Row1
    Sensors_Array[3].sensor_taxels[0].vector.x = 0.015337;
    Sensors_Array[3].sensor_taxels[0].vector.y = -0.006754;
    Sensors_Array[3].sensor_taxels[0].vector.z = -0.00513;
    Sensors_Array[3].sensor_taxels[1].vector.x = 0.015239;
    Sensors_Array[3].sensor_taxels[1].vector.y = 0.001668;
    Sensors_Array[3].sensor_taxels[1].vector.z = -0.005106;
    Sensors_Array[3].sensor_taxels[2].vector.x = 0.015289;
    Sensors_Array[3].sensor_taxels[2].vector.y = 0.010175;
    Sensors_Array[3].sensor_taxels[2].vector.z = -0.004527;
    Sensors_Array[3].sensor_taxels[3].vector.x = 0.016136;
    Sensors_Array[3].sensor_taxels[3].vector.y = 0.0139;
    Sensors_Array[3].sensor_taxels[3].vector.z = 0.00504;
    Sensors_Array[3].sensor_taxels[4].vector.x = 0.016712;
    Sensors_Array[3].sensor_taxels[4].vector.y = 0.0139;
    Sensors_Array[3].sensor_taxels[4].vector.z = 0.011619;
    Sensors_Array[3].sensor_taxels[5].vector.x = 0.017287;
    Sensors_Array[3].sensor_taxels[5].vector.y = 0.0139;
    Sensors_Array[3].sensor_taxels[5].vector.z = 0.018198;
    Sensors_Array[3].sensor_taxels[6].vector.x = 0.018274;
    Sensors_Array[3].sensor_taxels[6].vector.y = 0.011735;
    Sensors_Array[3].sensor_taxels[6].vector.z = 0.028435;
    Sensors_Array[3].sensor_taxels[7].vector.x = 0.018342;
    Sensors_Array[3].sensor_taxels[7].vector.y = 0.00314;
    Sensors_Array[3].sensor_taxels[7].vector.z = 0.029222;
    Sensors_Array[3].sensor_taxels[8].vector.x = 0.018251;
    Sensors_Array[3].sensor_taxels[8].vector.y = -0.005069;
    Sensors_Array[3].sensor_taxels[8].vector.z = 0.02932;
    //Row2
    Sensors_Array[3].sensor_taxels[9].vector.x = 0.021649;
    Sensors_Array[3].sensor_taxels[9].vector.y = -0.006754;
    Sensors_Array[3].sensor_taxels[9].vector.z = -0.005683;
    Sensors_Array[3].sensor_taxels[10].vector.x = 0.021551;
    Sensors_Array[3].sensor_taxels[10].vector.y = 0.001668;
    Sensors_Array[3].sensor_taxels[10].vector.z = -0.005659;
    Sensors_Array[3].sensor_taxels[11].vector.x = 0.021601;
    Sensors_Array[3].sensor_taxels[11].vector.y = 0.010175;
    Sensors_Array[3].sensor_taxels[11].vector.z = -0.005079;
    Sensors_Array[3].sensor_taxels[12].vector.x = 0.022448;
    Sensors_Array[3].sensor_taxels[12].vector.y = 0.0139;
    Sensors_Array[3].sensor_taxels[12].vector.z = 0.004488;
    Sensors_Array[3].sensor_taxels[13].vector.x = 0.023024;
    Sensors_Array[3].sensor_taxels[13].vector.y = 0.0139;
    Sensors_Array[3].sensor_taxels[13].vector.z = 0.011067;
    Sensors_Array[3].sensor_taxels[14].vector.x = 0.023599;
    Sensors_Array[3].sensor_taxels[14].vector.y = 0.0139;
    Sensors_Array[3].sensor_taxels[14].vector.z = 0.017646;
    Sensors_Array[3].sensor_taxels[15].vector.x = 0.024586;
    Sensors_Array[3].sensor_taxels[15].vector.y = 0.011735;
    Sensors_Array[3].sensor_taxels[15].vector.z = 0.027883;
    Sensors_Array[3].sensor_taxels[16].vector.x = 0.024654;
    Sensors_Array[3].sensor_taxels[16].vector.y = 0.00314;
    Sensors_Array[3].sensor_taxels[16].vector.z = 0.028669;
    Sensors_Array[3].sensor_taxels[17].vector.x = 0.024563;
    Sensors_Array[3].sensor_taxels[17].vector.y = -0.005069;
    Sensors_Array[3].sensor_taxels[17].vector.z = 0.028768;
    //Row3
    Sensors_Array[3].sensor_taxels[18].vector.x = 0.027724;
    Sensors_Array[3].sensor_taxels[18].vector.y = -0.001668;
    Sensors_Array[3].sensor_taxels[18].vector.z = -0.006199;
    Sensors_Array[3].sensor_taxels[19].vector.x = 0.027775;
    Sensors_Array[3].sensor_taxels[19].vector.y = 0.010175;
    Sensors_Array[3].sensor_taxels[19].vector.z = -0.005619;
    Sensors_Array[3].sensor_taxels[20].vector.x = 0.028622;
    Sensors_Array[3].sensor_taxels[20].vector.y = 0.0139;
    Sensors_Array[3].sensor_taxels[20].vector.z = 0.003948;
    Sensors_Array[3].sensor_taxels[21].vector.x = 0.029197;
    Sensors_Array[3].sensor_taxels[21].vector.y = 0.0139;
    Sensors_Array[3].sensor_taxels[21].vector.z = 0.010527;
    Sensors_Array[3].sensor_taxels[22].vector.x = 0.029773;
    Sensors_Array[3].sensor_taxels[22].vector.y = 0.0139;
    Sensors_Array[3].sensor_taxels[22].vector.z = 0.017106;
    Sensors_Array[3].sensor_taxels[23].vector.x = 0.030759;
    Sensors_Array[3].sensor_taxels[23].vector.y = 0.011735;
    Sensors_Array[3].sensor_taxels[23].vector.z = 0.027343;
    Sensors_Array[3].sensor_taxels[24].vector.x = 0.030828;
    Sensors_Array[3].sensor_taxels[24].vector.y = 0.00314;
    Sensors_Array[3].sensor_taxels[24].vector.z = 0.028129;
    //Row4
    Sensors_Array[3].sensor_taxels[25].vector.x = 0.033486;
    Sensors_Array[3].sensor_taxels[25].vector.y = 0.010175;
    Sensors_Array[3].sensor_taxels[25].vector.z = -0.006199;
    Sensors_Array[3].sensor_taxels[26].vector.x = 0.034333;
    Sensors_Array[3].sensor_taxels[26].vector.y = 0.0139;
    Sensors_Array[3].sensor_taxels[26].vector.z = 0.003448;
    Sensors_Array[3].sensor_taxels[27].vector.x = 0.034909;
    Sensors_Array[3].sensor_taxels[27].vector.y = 0.0139;
    Sensors_Array[3].sensor_taxels[27].vector.z = 0.010027;
    Sensors_Array[3].sensor_taxels[28].vector.x = 0.035484;
    Sensors_Array[3].sensor_taxels[28].vector.y = 0.0139;
    Sensors_Array[3].sensor_taxels[28].vector.z = 0.016606;
    Sensors_Array[3].sensor_taxels[29].vector.x = 0.03647;
    Sensors_Array[3].sensor_taxels[29].vector.y = 0.011735;
    Sensors_Array[3].sensor_taxels[29].vector.z = 0.026843;
    // Curved Face
    //Row1
    Sensors_Array[3].sensor_taxels[30].vector.x = 0.043079;
    Sensors_Array[3].sensor_taxels[30].vector.y = 0.013442;
    Sensors_Array[3].sensor_taxels[30].vector.z = -0.000058;
    Sensors_Array[3].sensor_taxels[31].vector.x = 0.043633;
    Sensors_Array[3].sensor_taxels[31].vector.y = 0.013442;
    Sensors_Array[3].sensor_taxels[31].vector.z = 0.006267;
    Sensors_Array[3].sensor_taxels[32].vector.x = 0.044186;
    Sensors_Array[3].sensor_taxels[32].vector.y = 0.013442;
    Sensors_Array[3].sensor_taxels[32].vector.z = 0.012593;
    Sensors_Array[3].sensor_taxels[33].vector.x = 0.04474;
    Sensors_Array[3].sensor_taxels[33].vector.y = 0.013442;
    Sensors_Array[3].sensor_taxels[33].vector.z = 0.018919;
    //Row2
    Sensors_Array[3].sensor_taxels[34].vector.x = 0.04288;
    Sensors_Array[3].sensor_taxels[34].vector.y = 0.007274;
    Sensors_Array[3].sensor_taxels[34].vector.z = -0.000041;
    Sensors_Array[3].sensor_taxels[35].vector.x = 0.043434;
    Sensors_Array[3].sensor_taxels[35].vector.y = 0.007274;
    Sensors_Array[3].sensor_taxels[35].vector.z = 0.006285;
    Sensors_Array[3].sensor_taxels[36].vector.x = 0.043987;
    Sensors_Array[3].sensor_taxels[36].vector.y = 0.007274;
    Sensors_Array[3].sensor_taxels[36].vector.z = 0.012611;
    Sensors_Array[3].sensor_taxels[37].vector.x = 0.044541;
    Sensors_Array[3].sensor_taxels[37].vector.y = 0.007274;
    Sensors_Array[3].sensor_taxels[37].vector.z = 0.018936;
    //Row3
    Sensors_Array[3].sensor_taxels[38].vector.x = 0.040359;
    Sensors_Array[3].sensor_taxels[38].vector.y = 0.002132;
    Sensors_Array[3].sensor_taxels[38].vector.z = -0.0000482;
    Sensors_Array[3].sensor_taxels[39].vector.x = 0.040912;
    Sensors_Array[3].sensor_taxels[39].vector.y = 0.002132;
    Sensors_Array[3].sensor_taxels[39].vector.z = 0.006808;
    Sensors_Array[3].sensor_taxels[40].vector.x = 0.041465;
    Sensors_Array[3].sensor_taxels[40].vector.y = 0.002132;
    Sensors_Array[3].sensor_taxels[40].vector.z = 0.013133;
    Sensors_Array[3].sensor_taxels[41].vector.x = 0.042019;
    Sensors_Array[3].sensor_taxels[41].vector.y = 0.002132;
    Sensors_Array[3].sensor_taxels[41].vector.z = 0.019459;
    //Row4
    Sensors_Array[3].sensor_taxels[42].vector.x = 0.037108;
    Sensors_Array[3].sensor_taxels[42].vector.y = -0.002795;
    Sensors_Array[3].sensor_taxels[42].vector.z = -0.000766;
    Sensors_Array[3].sensor_taxels[43].vector.x = 0.037662;
    Sensors_Array[3].sensor_taxels[43].vector.y = -0.002795;
    Sensors_Array[3].sensor_taxels[43].vector.z = 0.007092;
    Sensors_Array[3].sensor_taxels[44].vector.x = 0.038215;
    Sensors_Array[3].sensor_taxels[44].vector.y = -0.002795;
    Sensors_Array[3].sensor_taxels[44].vector.z = 0.013418;
    Sensors_Array[3].sensor_taxels[45].vector.x = 0.038769;
    Sensors_Array[3].sensor_taxels[45].vector.y = -0.002795;
    Sensors_Array[3].sensor_taxels[45].vector.z = 0.019744;
    //Row5
    Sensors_Array[3].sensor_taxels[46].vector.x = 0.033168;
    Sensors_Array[3].sensor_taxels[46].vector.y = -0.007189;
    Sensors_Array[3].sensor_taxels[46].vector.z = 0.001111;
    Sensors_Array[3].sensor_taxels[47].vector.x = 0.033721;
    Sensors_Array[3].sensor_taxels[47].vector.y = -0.007189;
    Sensors_Array[3].sensor_taxels[47].vector.z = 0.007437;
    Sensors_Array[3].sensor_taxels[48].vector.x = 0.034275;
    Sensors_Array[3].sensor_taxels[48].vector.y = -0.007189;
    Sensors_Array[3].sensor_taxels[48].vector.z = 0.013762;
    Sensors_Array[3].sensor_taxels[49].vector.x = 0.034828;
    Sensors_Array[3].sensor_taxels[49].vector.y = -0.007189;
    Sensors_Array[3].sensor_taxels[49].vector.z = 0.020088;
    //Row6
    Sensors_Array[3].sensor_taxels[50].vector.x = 0.028594;
    Sensors_Array[3].sensor_taxels[50].vector.y = -0.010913;
    Sensors_Array[3].sensor_taxels[50].vector.z = 0.001511;
    Sensors_Array[3].sensor_taxels[51].vector.x = 0.029148;
    Sensors_Array[3].sensor_taxels[51].vector.y = -0.010913;
    Sensors_Array[3].sensor_taxels[51].vector.z = 0.007837;
    Sensors_Array[3].sensor_taxels[52].vector.x = 0.029701;
    Sensors_Array[3].sensor_taxels[52].vector.y = -0.010913;
    Sensors_Array[3].sensor_taxels[52].vector.z = 0.014163;
    Sensors_Array[3].sensor_taxels[53].vector.x = 0.030255;
    Sensors_Array[3].sensor_taxels[53].vector.y = -0.010913;
    Sensors_Array[3].sensor_taxels[53].vector.z = 0.020488;
    //Row7
    Sensors_Array[3].sensor_taxels[54].vector.x = 0.023521;
    Sensors_Array[3].sensor_taxels[54].vector.y = -0.01392;
    Sensors_Array[3].sensor_taxels[54].vector.z = 0.001955;
    Sensors_Array[3].sensor_taxels[55].vector.x = 0.024075;
    Sensors_Array[3].sensor_taxels[55].vector.y = -0.01392;
    Sensors_Array[3].sensor_taxels[55].vector.z = 0.008281;
    Sensors_Array[3].sensor_taxels[56].vector.x = 0.024628;
    Sensors_Array[3].sensor_taxels[56].vector.y = -0.01392;
    Sensors_Array[3].sensor_taxels[56].vector.z = 0.014606;
    Sensors_Array[3].sensor_taxels[57].vector.x = 0.025181;
    Sensors_Array[3].sensor_taxels[57].vector.y = -0.01392;
    Sensors_Array[3].sensor_taxels[57].vector.z = 0.020932;
    //Row8
    Sensors_Array[3].sensor_taxels[58].vector.x = 0.018044;
    Sensors_Array[3].sensor_taxels[58].vector.y = -0.016096;
    Sensors_Array[3].sensor_taxels[58].vector.z = 0.002434;
    Sensors_Array[3].sensor_taxels[59].vector.x = 0.018598;
    Sensors_Array[3].sensor_taxels[59].vector.y = -0.016096;
    Sensors_Array[3].sensor_taxels[59].vector.z = 0.00876;
    Sensors_Array[3].sensor_taxels[60].vector.x = 0.019151;
    Sensors_Array[3].sensor_taxels[60].vector.y = -0.016096;
    Sensors_Array[3].sensor_taxels[60].vector.z = 0.015086;
    Sensors_Array[3].sensor_taxels[61].vector.x = 0.019705;
    Sensors_Array[3].sensor_taxels[61].vector.y = -0.016096;
    Sensors_Array[3].sensor_taxels[61].vector.z = 0.021411;
    //Row9
    Sensors_Array[3].sensor_taxels[62].vector.x = 0.012303;
    Sensors_Array[3].sensor_taxels[62].vector.y = -0.017413;
    Sensors_Array[3].sensor_taxels[62].vector.z = 0.002936;
    Sensors_Array[3].sensor_taxels[63].vector.x = 0.012857;
    Sensors_Array[3].sensor_taxels[63].vector.y = -0.017413;
    Sensors_Array[3].sensor_taxels[63].vector.z = 0.009262;
    Sensors_Array[3].sensor_taxels[64].vector.x = 0.01341;
    Sensors_Array[3].sensor_taxels[64].vector.y = -0.017413;
    Sensors_Array[3].sensor_taxels[64].vector.z = 0.015588;
    Sensors_Array[3].sensor_taxels[65].vector.x = 0.013963;
    Sensors_Array[3].sensor_taxels[65].vector.y = -0.017413;
    Sensors_Array[3].sensor_taxels[65].vector.z = 0.021914;
    //Pinky Fingertip

    //Straight wrapp up
    //Row1
    Sensors_Array[15].sensor_taxels[0].vector.x = 0.015337;
    Sensors_Array[15].sensor_taxels[0].vector.y = -0.006754;
    Sensors_Array[15].sensor_taxels[0].vector.z = -0.00513;
    Sensors_Array[15].sensor_taxels[1].vector.x = 0.015239;
    Sensors_Array[15].sensor_taxels[1].vector.y = 0.001668;
    Sensors_Array[15].sensor_taxels[1].vector.z = -0.005106;
    Sensors_Array[15].sensor_taxels[2].vector.x = 0.015289;
    Sensors_Array[15].sensor_taxels[2].vector.y = 0.010175;
    Sensors_Array[15].sensor_taxels[2].vector.z = -0.004527;
    Sensors_Array[15].sensor_taxels[3].vector.x = 0.016136;
    Sensors_Array[15].sensor_taxels[3].vector.y = 0.0139;
    Sensors_Array[15].sensor_taxels[3].vector.z = 0.00504;
    Sensors_Array[15].sensor_taxels[4].vector.x = 0.016712;
    Sensors_Array[15].sensor_taxels[4].vector.y = 0.0139;
    Sensors_Array[15].sensor_taxels[4].vector.z = 0.011619;
    Sensors_Array[15].sensor_taxels[5].vector.x = 0.017287;
    Sensors_Array[15].sensor_taxels[5].vector.y = 0.0139;
    Sensors_Array[15].sensor_taxels[5].vector.z = 0.018198;
    Sensors_Array[15].sensor_taxels[6].vector.x = 0.018274;
    Sensors_Array[15].sensor_taxels[6].vector.y = 0.011735;
    Sensors_Array[15].sensor_taxels[6].vector.z = 0.028435;
    Sensors_Array[15].sensor_taxels[7].vector.x = 0.018342;
    Sensors_Array[15].sensor_taxels[7].vector.y = 0.00314;
    Sensors_Array[15].sensor_taxels[7].vector.z = 0.029222;
    Sensors_Array[15].sensor_taxels[8].vector.x = 0.018251;
    Sensors_Array[15].sensor_taxels[8].vector.y = -0.005069;
    Sensors_Array[15].sensor_taxels[8].vector.z = 0.02932;
    //Row2
    Sensors_Array[15].sensor_taxels[9].vector.x = 0.021649;
    Sensors_Array[15].sensor_taxels[9].vector.y = -0.006754;
    Sensors_Array[15].sensor_taxels[9].vector.z = -0.005683;
    Sensors_Array[15].sensor_taxels[10].vector.x = 0.021551;
    Sensors_Array[15].sensor_taxels[10].vector.y = 0.001668;
    Sensors_Array[15].sensor_taxels[10].vector.z = -0.005659;
    Sensors_Array[15].sensor_taxels[11].vector.x = 0.021601;
    Sensors_Array[15].sensor_taxels[11].vector.y = 0.010175;
    Sensors_Array[15].sensor_taxels[11].vector.z = -0.005079;
    Sensors_Array[15].sensor_taxels[12].vector.x = 0.022448;
    Sensors_Array[15].sensor_taxels[12].vector.y = 0.0139;
    Sensors_Array[15].sensor_taxels[12].vector.z = 0.004488;
    Sensors_Array[15].sensor_taxels[13].vector.x = 0.023024;
    Sensors_Array[15].sensor_taxels[13].vector.y = 0.0139;
    Sensors_Array[15].sensor_taxels[13].vector.z = 0.011067;
    Sensors_Array[15].sensor_taxels[14].vector.x = 0.023599;
    Sensors_Array[15].sensor_taxels[14].vector.y = 0.0139;
    Sensors_Array[15].sensor_taxels[14].vector.z = 0.017646;
    Sensors_Array[15].sensor_taxels[15].vector.x = 0.024586;
    Sensors_Array[15].sensor_taxels[15].vector.y = 0.011735;
    Sensors_Array[15].sensor_taxels[15].vector.z = 0.027883;
    Sensors_Array[15].sensor_taxels[16].vector.x = 0.024654;
    Sensors_Array[15].sensor_taxels[16].vector.y = 0.00314;
    Sensors_Array[15].sensor_taxels[16].vector.z = 0.028669;
    Sensors_Array[15].sensor_taxels[17].vector.x = 0.024563;
    Sensors_Array[15].sensor_taxels[17].vector.y = -0.005069;
    Sensors_Array[15].sensor_taxels[17].vector.z = 0.028768;
    //Row3
    Sensors_Array[15].sensor_taxels[18].vector.x = 0.027724;
    Sensors_Array[15].sensor_taxels[18].vector.y = -0.001668;
    Sensors_Array[15].sensor_taxels[18].vector.z = -0.006199;
    Sensors_Array[15].sensor_taxels[19].vector.x = 0.027775;
    Sensors_Array[15].sensor_taxels[19].vector.y = 0.010175;
    Sensors_Array[15].sensor_taxels[19].vector.z = -0.005619;
    Sensors_Array[15].sensor_taxels[20].vector.x = 0.028622;
    Sensors_Array[15].sensor_taxels[20].vector.y = 0.0139;
    Sensors_Array[15].sensor_taxels[20].vector.z = 0.003948;
    Sensors_Array[15].sensor_taxels[21].vector.x = 0.029197;
    Sensors_Array[15].sensor_taxels[21].vector.y = 0.0139;
    Sensors_Array[15].sensor_taxels[21].vector.z = 0.010527;
    Sensors_Array[15].sensor_taxels[22].vector.x = 0.029773;
    Sensors_Array[15].sensor_taxels[22].vector.y = 0.0139;
    Sensors_Array[15].sensor_taxels[22].vector.z = 0.017106;
    Sensors_Array[15].sensor_taxels[23].vector.x = 0.030759;
    Sensors_Array[15].sensor_taxels[23].vector.y = 0.011735;
    Sensors_Array[15].sensor_taxels[23].vector.z = 0.027343;
    Sensors_Array[15].sensor_taxels[24].vector.x = 0.030828;
    Sensors_Array[15].sensor_taxels[24].vector.y = 0.00314;
    Sensors_Array[15].sensor_taxels[24].vector.z = 0.028129;
    //Row4
    Sensors_Array[15].sensor_taxels[25].vector.x = 0.033486;
    Sensors_Array[15].sensor_taxels[25].vector.y = 0.010175;
    Sensors_Array[15].sensor_taxels[25].vector.z = -0.006199;
    Sensors_Array[15].sensor_taxels[26].vector.x = 0.034333;
    Sensors_Array[15].sensor_taxels[26].vector.y = 0.0139;
    Sensors_Array[15].sensor_taxels[26].vector.z = 0.003448;
    Sensors_Array[15].sensor_taxels[27].vector.x = 0.034909;
    Sensors_Array[15].sensor_taxels[27].vector.y = 0.0139;
    Sensors_Array[15].sensor_taxels[27].vector.z = 0.010027;
    Sensors_Array[15].sensor_taxels[28].vector.x = 0.035484;
    Sensors_Array[15].sensor_taxels[28].vector.y = 0.0139;
    Sensors_Array[15].sensor_taxels[28].vector.z = 0.016606;
    Sensors_Array[15].sensor_taxels[29].vector.x = 0.03647;
    Sensors_Array[15].sensor_taxels[29].vector.y = 0.011735;
    Sensors_Array[15].sensor_taxels[29].vector.z = 0.026843;
    // Curved Face
    //Row1
    Sensors_Array[15].sensor_taxels[30].vector.x = 0.043079;
    Sensors_Array[15].sensor_taxels[30].vector.y = 0.013442;
    Sensors_Array[15].sensor_taxels[30].vector.z = -0.000058;
    Sensors_Array[15].sensor_taxels[31].vector.x = 0.043633;
    Sensors_Array[15].sensor_taxels[31].vector.y = 0.013442;
    Sensors_Array[15].sensor_taxels[31].vector.z = 0.006267;
    Sensors_Array[15].sensor_taxels[32].vector.x = 0.044186;
    Sensors_Array[15].sensor_taxels[32].vector.y = 0.013442;
    Sensors_Array[15].sensor_taxels[32].vector.z = 0.012593;
    Sensors_Array[15].sensor_taxels[33].vector.x = 0.04474;
    Sensors_Array[15].sensor_taxels[33].vector.y = 0.013442;
    Sensors_Array[15].sensor_taxels[33].vector.z = 0.018919;
    //Row2
    Sensors_Array[15].sensor_taxels[34].vector.x = 0.04288;
    Sensors_Array[15].sensor_taxels[34].vector.y = 0.007274;
    Sensors_Array[15].sensor_taxels[34].vector.z = -0.000041;
    Sensors_Array[15].sensor_taxels[35].vector.x = 0.043434;
    Sensors_Array[15].sensor_taxels[35].vector.y = 0.007274;
    Sensors_Array[15].sensor_taxels[35].vector.z = 0.006285;
    Sensors_Array[15].sensor_taxels[36].vector.x = 0.043987;
    Sensors_Array[15].sensor_taxels[36].vector.y = 0.007274;
    Sensors_Array[15].sensor_taxels[36].vector.z = 0.012611;
    Sensors_Array[15].sensor_taxels[37].vector.x = 0.044541;
    Sensors_Array[15].sensor_taxels[37].vector.y = 0.007274;
    Sensors_Array[15].sensor_taxels[37].vector.z = 0.018936;
    //Row3
    Sensors_Array[15].sensor_taxels[38].vector.x = 0.040359;
    Sensors_Array[15].sensor_taxels[38].vector.y = 0.002132;
    Sensors_Array[15].sensor_taxels[38].vector.z = -0.0000482;
    Sensors_Array[15].sensor_taxels[39].vector.x = 0.040912;
    Sensors_Array[15].sensor_taxels[39].vector.y = 0.002132;
    Sensors_Array[15].sensor_taxels[39].vector.z = 0.006808;
    Sensors_Array[15].sensor_taxels[40].vector.x = 0.041465;
    Sensors_Array[15].sensor_taxels[40].vector.y = 0.002132;
    Sensors_Array[15].sensor_taxels[40].vector.z = 0.013133;
    Sensors_Array[15].sensor_taxels[41].vector.x = 0.042019;
    Sensors_Array[15].sensor_taxels[41].vector.y = 0.002132;
    Sensors_Array[15].sensor_taxels[41].vector.z = 0.019459;
    //Row4
    Sensors_Array[15].sensor_taxels[42].vector.x = 0.037108;
    Sensors_Array[15].sensor_taxels[42].vector.y = -0.002795;
    Sensors_Array[15].sensor_taxels[42].vector.z = -0.000766;
    Sensors_Array[15].sensor_taxels[43].vector.x = 0.037662;
    Sensors_Array[15].sensor_taxels[43].vector.y = -0.002795;
    Sensors_Array[15].sensor_taxels[43].vector.z = 0.007092;
    Sensors_Array[15].sensor_taxels[44].vector.x = 0.038215;
    Sensors_Array[15].sensor_taxels[44].vector.y = -0.002795;
    Sensors_Array[15].sensor_taxels[44].vector.z = 0.013418;
    Sensors_Array[15].sensor_taxels[45].vector.x = 0.038769;
    Sensors_Array[15].sensor_taxels[45].vector.y = -0.002795;
    Sensors_Array[15].sensor_taxels[45].vector.z = 0.019744;
    //Row5
    Sensors_Array[15].sensor_taxels[46].vector.x = 0.033168;
    Sensors_Array[15].sensor_taxels[46].vector.y = -0.007189;
    Sensors_Array[15].sensor_taxels[46].vector.z = 0.001111;
    Sensors_Array[15].sensor_taxels[47].vector.x = 0.033721;
    Sensors_Array[15].sensor_taxels[47].vector.y = -0.007189;
    Sensors_Array[15].sensor_taxels[47].vector.z = 0.007437;
    Sensors_Array[15].sensor_taxels[48].vector.x = 0.034275;
    Sensors_Array[15].sensor_taxels[48].vector.y = -0.007189;
    Sensors_Array[15].sensor_taxels[48].vector.z = 0.013762;
    Sensors_Array[15].sensor_taxels[49].vector.x = 0.034828;
    Sensors_Array[15].sensor_taxels[49].vector.y = -0.007189;
    Sensors_Array[15].sensor_taxels[49].vector.z = 0.020088;
    //Row6
    Sensors_Array[15].sensor_taxels[50].vector.x = 0.028594;
    Sensors_Array[15].sensor_taxels[50].vector.y = -0.010913;
    Sensors_Array[15].sensor_taxels[50].vector.z = 0.001511;
    Sensors_Array[15].sensor_taxels[51].vector.x = 0.029148;
    Sensors_Array[15].sensor_taxels[51].vector.y = -0.010913;
    Sensors_Array[15].sensor_taxels[51].vector.z = 0.007837;
    Sensors_Array[15].sensor_taxels[52].vector.x = 0.029701;
    Sensors_Array[15].sensor_taxels[52].vector.y = -0.010913;
    Sensors_Array[15].sensor_taxels[52].vector.z = 0.014163;
    Sensors_Array[15].sensor_taxels[53].vector.x = 0.030255;
    Sensors_Array[15].sensor_taxels[53].vector.y = -0.010913;
    Sensors_Array[15].sensor_taxels[53].vector.z = 0.020488;
    //Row7
    Sensors_Array[15].sensor_taxels[54].vector.x = 0.023521;
    Sensors_Array[15].sensor_taxels[54].vector.y = -0.01392;
    Sensors_Array[15].sensor_taxels[54].vector.z = 0.001955;
    Sensors_Array[15].sensor_taxels[55].vector.x = 0.024075;
    Sensors_Array[15].sensor_taxels[55].vector.y = -0.01392;
    Sensors_Array[15].sensor_taxels[55].vector.z = 0.008281;
    Sensors_Array[15].sensor_taxels[56].vector.x = 0.024628;
    Sensors_Array[15].sensor_taxels[56].vector.y = -0.01392;
    Sensors_Array[15].sensor_taxels[56].vector.z = 0.014606;
    Sensors_Array[15].sensor_taxels[57].vector.x = 0.025181;
    Sensors_Array[15].sensor_taxels[57].vector.y = -0.01392;
    Sensors_Array[15].sensor_taxels[57].vector.z = 0.020932;
    //Row8
    Sensors_Array[15].sensor_taxels[58].vector.x = 0.018044;
    Sensors_Array[15].sensor_taxels[58].vector.y = -0.016096;
    Sensors_Array[15].sensor_taxels[58].vector.z = 0.002434;
    Sensors_Array[15].sensor_taxels[59].vector.x = 0.018598;
    Sensors_Array[15].sensor_taxels[59].vector.y = -0.016096;
    Sensors_Array[15].sensor_taxels[59].vector.z = 0.00876;
    Sensors_Array[15].sensor_taxels[60].vector.x = 0.019151;
    Sensors_Array[15].sensor_taxels[60].vector.y = -0.016096;
    Sensors_Array[15].sensor_taxels[60].vector.z = 0.015086;
    Sensors_Array[15].sensor_taxels[61].vector.x = 0.019705;
    Sensors_Array[15].sensor_taxels[61].vector.y = -0.016096;
    Sensors_Array[15].sensor_taxels[61].vector.z = 0.021411;
    //Row9
    Sensors_Array[15].sensor_taxels[62].vector.x = 0.012303;
    Sensors_Array[15].sensor_taxels[62].vector.y = -0.017413;
    Sensors_Array[15].sensor_taxels[62].vector.z = 0.002936;
    Sensors_Array[15].sensor_taxels[63].vector.x = 0.012857;
    Sensors_Array[15].sensor_taxels[63].vector.y = -0.017413;
    Sensors_Array[15].sensor_taxels[63].vector.z = 0.009262;
    Sensors_Array[15].sensor_taxels[64].vector.x = 0.01341;
    Sensors_Array[15].sensor_taxels[64].vector.y = -0.017413;
    Sensors_Array[15].sensor_taxels[64].vector.z = 0.015588;
    Sensors_Array[15].sensor_taxels[65].vector.x = 0.013963;
    Sensors_Array[15].sensor_taxels[65].vector.y = -0.017413;
    Sensors_Array[15].sensor_taxels[65].vector.z = 0.021914;

    for (size_t sensor_nb = 0; sensor_nb < NUM_SENSORS; sensor_nb++)
    {
        for (size_t taxel = 0; taxel < Sensors_Array[sensor_nb].sensor_taxels.size(); taxel++)
        {
            visualization_msgs::Marker marker;
            Sensors_Array[sensor_nb].sensor_markers.push_back(marker);
        }
        
    }

    for (size_t sensor_nb = 0; sensor_nb < NUM_SENSORS; sensor_nb++)
    {
        for (size_t taxel = 0; taxel < Sensors_Array[sensor_nb].sensor_taxels.size(); taxel++)
        {
            markers_array.markers.push_back(Sensors_Array[sensor_nb].sensor_markers[taxel]);
        }
    }

    std::array<ros::Subscriber, NUM_SENSORS> subscribers;
    for (size_t i=0; i < subscribers.size(); i++)
    {
        subscribers[i] = node.subscribe<bici_ros_sensor_reader::TactileData>("sensor_" + std::to_string(i+NUM_SENSOR_MIN)+"_readings", 5, callback);
    }
    // Subscriber for the joint states
    ros::Subscriber joint_subscriber = node.subscribe<sensor_msgs::JointState>("joint_states", 5, joint_callback);
    // Publisher for the rviz markers
    ros::Publisher markers_pub = node.advertise<visualization_msgs::MarkerArray>("visualization_markers", 1000);
    //Server for the coordinates saving activation
    ros::ServiceServer saving_service = node.advertiseService("saving_command",saving_activation_function);
    //Server for the taxels data averaging activation
    ros::ServiceServer avg_service = node.advertiseService("avg_command",avg_activation_function);
    while(ros::ok()){

    markers_pub.publish(markers_array);
    ros::spinOnce();
    rate.sleep();
    }
        return 0;
        };