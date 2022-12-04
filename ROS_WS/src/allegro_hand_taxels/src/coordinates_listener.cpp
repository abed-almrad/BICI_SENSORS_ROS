#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <bici_ros_sensor_reader/TactileData.h>
#include <vector>
#include<algorithm>
#include <map>
#include <iostream>
#include <fstream>
#include <cstring>
#include "allegro_hand_taxels/coordinates_saving.h"
#include <string>
#include <filesystem>
#include <unistd.h>

#define NUM_SENSORS 22
#define NUM_SENSOR_MIN 8
std::ofstream myfile;
std::ofstream myfile_inactive_taxels;
using namespace std;
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
    std::vector<float> avg; // Array of taxels average values for each sensor
    size_t taxel_values_nb = 0; // A variable to keep track of the number of values used to initially to calculate the average for each taxel
    bool sensor_averaged = false;
    std::vector<geometry_msgs::TransformStamped> transformStamped_taxels;
    std::vector<std::vector<float>> taxels_coordinates;
    std::vector<std::vector<float>> inactive_taxels_coordinates;
    Tactile_Sensor(uint8_t sensor_num, int size, string sensor_name){
        //Any other number can be used for initialization
        this->sensor_num = sensor_num;
        this->size = size;
        this->sensor_name = sensor_name;
        this->data.resize(size);
        this->avg.resize(size);
        this->transformStamped_taxels.resize(size);
        this->taxels_coordinates.resize(size, std::vector<float>(3, 0));//Vector monitoring the sensor's activated taxels positions
        this->inactive_taxels_coordinates.resize(size, std::vector<float>(3, 0));//Vector monitoring the sensor's inactive taxels positions
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
vector<vector<float>> points_storage; //Container for the contact points to be stored
vector<vector<float>> inactive_points_storage; //Container for the inactive taxel points to be stored
string act_cmd = "false";
int reg_iter_count;
tf2_ros::Buffer tfBuffer;
tf2_ros::TransformListener *tfListener;
map <string, visualization_msgs::Marker> markers_map; // A dictionary for the markers placed on the taxels
visualization_msgs::MarkerArray markers_array; //Array of markers (e.g. a marker is created for each taxel)
int taxel_values_nb_max = 100; // Number of values used to initially to calculate the average for each taxel

//Method for the coordinates saving activation

bool saving_activation_function(allegro_hand_taxels::coordinates_saving::Request &req,
                                       allegro_hand_taxels::coordinates_saving::Response &res)
{
    res.saving_activation_response = req.saving_activation;
    std::cout << "Requested saving activation boolean string: " << req.saving_activation << std::endl;
    act_cmd = res.saving_activation_response;
    reg_iter_count = req.iteration_nb;
    std::cout<<"sending back response: "<< act_cmd << std::endl;
    return true;


}

//One common callback function
void callback(const bici_ros_sensor_reader::TactileData msg)
{
    Sensors_Array[msg.sensor_num-8].timestamp = msg.timestamp;
    Sensors_Array[msg.sensor_num-8].data = msg.data;
    if (Sensors_Array[msg.sensor_num-8].taxel_values_nb < taxel_values_nb_max)
    {
        std::transform(Sensors_Array[msg.sensor_num-8].avg.begin(), 
                       Sensors_Array[msg.sensor_num-8].avg.end(),
                       msg.data.begin(), Sensors_Array[msg.sensor_num-8].avg.begin(),
                       std::plus<float>()); // Running sum for the taxels values
        Sensors_Array[msg.sensor_num-8].taxel_values_nb += 1;
        return;
    }
    else if ((Sensors_Array[msg.sensor_num-8].taxel_values_nb == taxel_values_nb_max) && !Sensors_Array[msg.sensor_num-8].sensor_averaged)
    {
        int k = taxel_values_nb_max;
        std::transform(Sensors_Array[msg.sensor_num-8].avg.begin(),
                       Sensors_Array[msg.sensor_num-8].avg.end(),
                       Sensors_Array[msg.sensor_num-8].avg.begin(), [k](float &c){ return c/k; });
        std::cout << "\n" << "For sensor number " << msg.sensor_num-0 << ", the baseline values are: "; 
        for (size_t i = 0; i < Sensors_Array[msg.sensor_num-8].avg.size(); i++)
        {
            std::cout << Sensors_Array[msg.sensor_num-8].avg[i] << ", "; 
        }
        std::cout << "." << std::endl;
        Sensors_Array[msg.sensor_num-8].sensor_averaged = true;    
    }
    
    

    
    //cout<<act_cmd<<endl;
    if (act_cmd == "true") // Activation of a save command through a ros node
    {
        string full_path = filesystem::current_path();
        size_t pos = full_path.find("ROS_WS");
        string path = full_path.substr(0,pos+6)+"/Pt_Cloud_Scripts/points_coordinates_"+to_string(reg_iter_count)+".csv";
        string path_inactive_taxels = full_path.substr(0,pos+6)+"/Pt_Cloud_Scripts/inactive_points_coordinates_"+to_string(reg_iter_count)+".csv";
        cout<<"THE SAVING PATH FOR THE ACTIVE PT CLOUD IS: " + path << endl;
        myfile.open(path.c_str());
        myfile_inactive_taxels.open(path_inactive_taxels.c_str());
        for(int j=0;j<NUM_SENSORS;j++){
            //cout << Sensors_Array[j].taxels_coordinates.size() << endl;
            //cout << Sensors_Array[j].taxels_coordinates[0].size() << endl;
            for(int taxel=0;taxel<Sensors_Array[j].taxels_coordinates.size();taxel++){
                points_storage.push_back(Sensors_Array[j].taxels_coordinates[taxel]);
                //cout<< "X_Storage"<< points_storage[taxel][0] << endl;
                //cout<< "Y_Storage"<< points_storage[taxel][1] << endl;
                //cout<< "Z_Storage"<< points_storage[taxel][2] << endl;
                inactive_points_storage.push_back(Sensors_Array[j].inactive_taxels_coordinates[taxel]);
                //cout<< "X_Storage"<< inactive_points_storage[taxel][0] << endl;
                //cout<< "Y_Storage"<< inactive_points_storage[taxel][1] << endl;
                //cout<< "Z_Storage"<< inactive_points_storage[taxel][2] << endl;
            }
        }
        //Writing active taxels data to a csv file
        for (size_t i = 0; i < points_storage.size(); i++)
        {
            for (size_t j = 0; j < points_storage[i].size(); j++)
            {
                cout<< "Active_Contact_Coordinates_Storage "<< points_storage[i][j] << endl;
                myfile << points_storage[i][j] << ',';
                //cout << "Recorded active taxel coordinate" << points_storage[i][j] << endl;
            }
            myfile << '\n';
            
        }
        myfile.close();
        points_storage.clear();

        //Writing inactive taxels data to a csv file
        for (size_t i = 0; i < inactive_points_storage.size(); i++)
        {
            for (size_t j = 0; j < inactive_points_storage[i].size(); j++)
            {
                cout<< "Inactive_Contact_Coordinates_Storage "<< inactive_points_storage[i][j] << endl;
                myfile_inactive_taxels << inactive_points_storage[i][j] << ',';
                //cout << "Recorded inactive taxel coordinate" << inactive_points_storage[i][j] << endl;
            }
            myfile_inactive_taxels << '\n';
            
        }
        myfile_inactive_taxels.close();
        inactive_points_storage.clear();

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
            visualization_msgs::Marker marker;
            marker.ns = Sensors_Array[msg.sensor_num-8].sensor_name+"_sub_taxel";
            marker.id= taxel;
            marker.header.frame_id = Sensors_Array[msg.sensor_num-8].sensor_name+"_sub_taxel_"+to_string(taxel);
            marker.header.stamp = ros::Time(0);
            marker.type = visualization_msgs::Marker::SPHERE;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.position.x = 0;
            marker.pose.position.y = 0;
            marker.pose.position.z = 0;
            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0;
            marker.pose.orientation.w = 1.0;
            marker.scale.x = 0.003;
            marker.scale.y = 0.003;
            marker.scale.z = 0.003;
            marker.color.r = 1.0f;
            marker.color.g = 0.2f;
            marker.color.b = 0.2f;
            marker.color.a = 1.0;
            marker.lifetime = ros::Duration();
            markers_array.markers.push_back(marker);
            
        }
        else{
            //cout << "marker color should come back to normal" << endl;
            visualization_msgs::Marker marker;
            marker.ns = Sensors_Array[msg.sensor_num-8].sensor_name+"_sub_taxel";
            marker.id= taxel;
            marker.header.frame_id = Sensors_Array[msg.sensor_num-8].sensor_name+"_sub_taxel_"+to_string(taxel);
            marker.header.stamp = ros::Time(0);
            marker.type = visualization_msgs::Marker::SPHERE;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.position.x = 0;
            marker.pose.position.y = 0;
            marker.pose.position.z = 0;
            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0;
            marker.pose.orientation.w = 1.0;
            marker.scale.x = 0.003;
            marker.scale.y = 0.003;
            marker.scale.z = 0.003;
            marker.color.r = 1.0f;
            marker.color.g = 0.2f;
            marker.color.b = 0.2f;
            marker.color.a = 0.0;
            marker.lifetime = ros::Duration();
            markers_array.markers.push_back(marker);
             
        }
    
       if (tfBuffer.canTransform("palm_link", Sensors_Array[msg.sensor_num-8].sensor_name+"_sub_taxel_"+std::to_string(taxel), ros::Time(0))) 
       {
           //cout<< tfBuffer.canTransform(Sensors_Array[msg.sensor_num-8].sensor_name+"_taxel_"+std::to_string(taxel),"palm_link", ros::Time(0))<<endl;
           try
           {
               Sensors_Array[msg.sensor_num-8].transformStamped_taxels[taxel] = tfBuffer.lookupTransform("palm_link", Sensors_Array[msg.sensor_num-8].sensor_name+"_sub_taxel_"+std::to_string(taxel), ros::Time(0));                                                                                                                                                    
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

            if(abs(Sensors_Array[msg.sensor_num-8].data[taxel]-Sensors_Array[msg.sensor_num-8].avg[taxel])>= Sensors_Array[msg.sensor_num-8].threshold)
            {
                Sensors_Array[msg.sensor_num-8].taxels_coordinates[taxel][0] = Sensors_Array[msg.sensor_num-8].transformStamped_taxels[taxel].transform.translation.x;
                //cout<< "X Coordinate " << Sensors_Array[msg.sensor_num-8].taxels_coordinates[taxel][0] << endl; 
                Sensors_Array[msg.sensor_num-8].taxels_coordinates[taxel][1] = Sensors_Array[msg.sensor_num-8].transformStamped_taxels[taxel].transform.translation.y;
                //cout<< "Y Coordinate " << Sensors_Array[msg.sensor_num-8].taxels_coordinates[taxel][1] << endl;
                Sensors_Array[msg.sensor_num-8].taxels_coordinates[taxel][2] = Sensors_Array[msg.sensor_num-8].transformStamped_taxels[taxel].transform.translation.z;
                //cout<< "Z Coordinate " << Sensors_Array[msg.sensor_num-8].taxels_coordinates[taxel][2] << endl;

                Sensors_Array[msg.sensor_num-8].inactive_taxels_coordinates[taxel][0] = 0;
                Sensors_Array[msg.sensor_num-8].inactive_taxels_coordinates[taxel][1] = 0;
                Sensors_Array[msg.sensor_num-8].inactive_taxels_coordinates[taxel][2] = 0;
            }
            else
            {
                Sensors_Array[msg.sensor_num-8].inactive_taxels_coordinates[taxel][0] = Sensors_Array[msg.sensor_num-8].transformStamped_taxels[taxel].transform.translation.x;
                //cout<< "X Coordinate " << Sensors_Array[msg.sensor_num-8].taxels_coordinates[taxel][0] << endl; 
                Sensors_Array[msg.sensor_num-8].inactive_taxels_coordinates[taxel][1] = Sensors_Array[msg.sensor_num-8].transformStamped_taxels[taxel].transform.translation.y;
                //cout<< "Y Coordinate " << Sensors_Array[msg.sensor_num-8].taxels_coordinates[taxel][1] << endl;
                Sensors_Array[msg.sensor_num-8].inactive_taxels_coordinates[taxel][2] = Sensors_Array[msg.sensor_num-8].transformStamped_taxels[taxel].transform.translation.z;
                //cout<< "Z Coordinate " << Sensors_Array[msg.sensor_num-8].taxels_coordinates[taxel][2] << endl;
                        
                Sensors_Array[msg.sensor_num-8].taxels_coordinates[taxel][0] = 0;
                Sensors_Array[msg.sensor_num-8].taxels_coordinates[taxel][1] = 0;
                Sensors_Array[msg.sensor_num-8].taxels_coordinates[taxel][2] = 0;
            }
            
        }
        else
        {
            std::cout << "No transformation is available" << std::endl;
        }
        
    }

}



int main(int argc, char** argv){
    ros::init(argc, argv, "my_tf2_listener");
    ros::NodeHandle node;
    ros::Rate rate(1.0);
    tfListener = new tf2_ros::TransformListener(tfBuffer);
    std::array<ros::Subscriber, NUM_SENSORS> subscribers;
    for (size_t i=0; i < subscribers.size(); i++)
    {
        subscribers[i] = node.subscribe<bici_ros_sensor_reader::TactileData>("sensor_" + std::to_string(i+NUM_SENSOR_MIN)+"_readings", 5, callback);
    }
    // Publisher for the rviz markers
    ros::Publisher markers_pub = node.advertise<visualization_msgs::MarkerArray>("visualization_markers", 1000);
    //Server for the coordinates saving activation
    ros::ServiceServer service = node.advertiseService("saving_command",saving_activation_function);

    while(ros::ok()){

    markers_pub.publish(markers_array);
    ros::spinOnce();
    rate.sleep();
    }
        return 0;
        };