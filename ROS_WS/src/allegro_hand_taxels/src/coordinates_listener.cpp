#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <bici_ros_sensor_reader/TactileData.h>
#include <vector>
#include <map>
#include <iostream>
#include <fstream>
#include <cstring>
#include <std_msgs/String.h>
#include <string>
#include <filesystem>
#include <unistd.h>

#define NUM_SENSORS 22
#define NUM_SENSOR_MIN 8
std::ofstream myfile;
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
    uint16_t threshold = 1500;  // Capacitance raw count threshold to detect contact.
                               // This might be later on initialized individually for each
                               //sensor or even taxel, after tuning
    uint32_t timestamp = 0; // This can be assigned to any random number
    std::vector<float> data;
    std::vector<geometry_msgs::TransformStamped> transformStamped_taxels;
    std::vector<std::vector<float>> taxels_coordinates;
    Tactile_Sensor(uint8_t sensor_num, int size, string sensor_name){
        //Any other number can be used for initialization
        this->sensor_num = sensor_num;
        this->size = size;
        this->sensor_name = sensor_name;
        this->data.resize(size);
        this->transformStamped_taxels.resize(size);
        this->taxels_coordinates.resize(size, std::vector<float>(3, 0));//Vector monitoring the sensor's taxels positions

    }
};

//The set of integrated sensors:

Tactile_Sensor sensor1(8, PF_SIZE, "mpf");
Tactile_Sensor sensor2(9, MB_SIZE, "mmb");
Tactile_Sensor sensor3(10, PB_SIZE, "mpb");
Tactile_Sensor sensor4(11, FT_SIZE, "pft");
Tactile_Sensor sensor5(12, MF_SIZE, "pmf");
Tactile_Sensor sensor6(13, PF_SIZE, "ppf");
Tactile_Sensor sensor7(14, MB_SIZE, "pmb");
Tactile_Sensor sensor8(15, PB_SIZE, "ppb");
Tactile_Sensor sensor9(16, FT_SIZE, "tft");
Tactile_Sensor sensor10(17, TMF_SIZE, "tmf");
Tactile_Sensor sensor11(18, PF_SIZE, "tpf");
Tactile_Sensor sensor12(19, TMB_SIZE, "tmb");
Tactile_Sensor sensor13(20, PB_SIZE, "tpb");
Tactile_Sensor sensor14(21, PALM_SIZE, "palm");
Tactile_Sensor sensor15(22, BOH_SIZE, "boh");
Tactile_Sensor sensor16(23, FT_SIZE, "ift");
Tactile_Sensor sensor17(24, MF_SIZE, "imf");
Tactile_Sensor sensor18(25, PF_SIZE, "ipf");
Tactile_Sensor sensor19(26, MB_SIZE, "imb");
Tactile_Sensor sensor20(27, PB_SIZE, "ipb");
Tactile_Sensor sensor21(28, FT_SIZE, "mft");
Tactile_Sensor sensor22(29, MF_SIZE, "mmf");



Tactile_Sensor Sensors_Array[NUM_SENSORS]{sensor1, sensor2, sensor3, sensor4, sensor5, sensor6, sensor7, sensor8,
sensor9, sensor10, sensor11, sensor12, sensor13, sensor14, sensor15, sensor16, sensor17, sensor18, sensor19,
sensor20, sensor21, sensor22};
vector<vector<float>> points_storage; //Container for the contact points to be stored
string act_cmd = "false";
tf2_ros::Buffer tfBuffer;
tf2_ros::TransformListener *tfListener;
map <string, visualization_msgs::Marker> markers_map; // A dictionary for the markers placed on the taxels
visualization_msgs::MarkerArray markers_array; //Array of markers (e.g. a marker is created for each taxel)
//One common callback function
void callback(const bici_ros_sensor_reader::TactileData msg){
    Sensors_Array[msg.sensor_num-8].timestamp = msg.timestamp;
    Sensors_Array[msg.sensor_num-8].data = msg.data;
    //cout<<act_cmd<<endl;
    if (act_cmd == "true") // Activation of a save command through a ros node
    {
        for(int j=0;j<NUM_SENSORS;j++){
            //cout << Sensors_Array[j].taxels_coordinates.size() << endl;
            //cout << Sensors_Array[j].taxels_coordinates[0].size() << endl;
            for(int taxel=0;taxel<Sensors_Array[j].taxels_coordinates.size();taxel++){
                points_storage.push_back(Sensors_Array[j].taxels_coordinates[taxel]);
                //cout<< "X_Storage"<< points_storage[taxel][0] << endl;
                //cout<< "Y_Storage"<< points_storage[taxel][1] << endl;
                //cout<< "Z_Storage"<< points_storage[taxel][2] << endl;
            }
        }
        //Writing data to a csv file
        for (size_t i = 0; i < points_storage.size(); i++)
        {
            for (size_t j = 0; j < points_storage[i].size(); j++)
            {
                cout<< "Coordinates_Storage "<< points_storage[i][j] << endl;
                myfile << points_storage[i][j] << ',';
                //cout << "Recorded coordinate" << points_storage[i][j] << endl;
            }
            myfile << '\n';
            
        }
        myfile.close();
        act_cmd = "false";
        points_storage.clear();
        ros::shutdown();

    }
     
    for(int taxel=0;taxel<Sensors_Array[msg.sensor_num-8].size;taxel++){
       if (Sensors_Array[msg.sensor_num-8].data[taxel]<= Sensors_Array[msg.sensor_num-8].threshold)
       {
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
       
       if ((Sensors_Array[msg.sensor_num-8].data[taxel]<= Sensors_Array[msg.sensor_num-8].threshold) && tfBuffer.canTransform("palm_link", Sensors_Array[msg.sensor_num-8].sensor_name+"_sub_taxel_"+std::to_string(taxel), ros::Time(0))) 
       {
           //cout<< tfBuffer.canTransform(Sensors_Array[msg.sensor_num-8].sensor_name+"_taxel_"+std::to_string(taxel),"palm_link", ros::Time(0))<<endl;
           try{
               Sensors_Array[msg.sensor_num-8].transformStamped_taxels[taxel] = tfBuffer.lookupTransform("palm_link", Sensors_Array[msg.sensor_num-8].sensor_name+"_sub_taxel_"+std::to_string(taxel), ros::Time(0));                                                                                                                                                    
           }
           catch (ros::Exception &ex) {
            ROS_WARN("%s",ex.what());
            ros::Duration(1.0).sleep();
            continue;
            }
            Sensors_Array[msg.sensor_num-8].taxels_coordinates[taxel][0] = Sensors_Array[msg.sensor_num-8].transformStamped_taxels[taxel].transform.translation.x;
            //cout<< "X Coordinate " << Sensors_Array[msg.sensor_num-8].taxels_coordinates[taxel][0] << endl; 
            Sensors_Array[msg.sensor_num-8].taxels_coordinates[taxel][1] = Sensors_Array[msg.sensor_num-8].transformStamped_taxels[taxel].transform.translation.y;
            //cout<< "Y Coordinate " << Sensors_Array[msg.sensor_num-8].taxels_coordinates[taxel][1] << endl;
            Sensors_Array[msg.sensor_num-8].taxels_coordinates[taxel][2] = Sensors_Array[msg.sensor_num-8].transformStamped_taxels[taxel].transform.translation.z;
            //cout<< "Z Coordinate " << Sensors_Array[msg.sensor_num-8].taxels_coordinates[taxel][2] << endl;
       }
        
    }

}

void saving_callback(const std_msgs::String msg){
    act_cmd = msg.data;
}


int main(int argc, char** argv){
    ros::init(argc, argv, "my_tf2_listener");
    ros::NodeHandle node;
    ros::Rate rate(1.0);
    tfListener = new tf2_ros::TransformListener(tfBuffer);
    std::array<ros::Subscriber, NUM_SENSORS> subscribers;
    string full_path = filesystem::current_path();
    size_t pos = full_path.find("ROS_WS");
    string path = full_path.substr(0,pos+6)+"/Pt_Cloud_Scripts/points_coordinates.csv";
    //cout<<"THE PATH IS: " + path<< endl;
    myfile.open(path.c_str());
    ros::Subscriber saving_subscriber; //A subscriber to the saving_activation command
    for (size_t i=0; i < subscribers.size(); i++)
    {
        subscribers[i] = node.subscribe<bici_ros_sensor_reader::TactileData>("sensor_" + std::to_string(i+NUM_SENSOR_MIN)+"_readings", 5, callback);
    }
    saving_subscriber = node.subscribe<std_msgs::String>("saving_command", 5, saving_callback);
    // Publisher for the rviz markers
    ros::Publisher markers_pub = node.advertise<visualization_msgs::MarkerArray>("visualization_markers", 1000);

    while(ros::ok()){

    markers_pub.publish(markers_array);
    ros::spinOnce();
    rate.sleep();
    }
        return 0;
        };