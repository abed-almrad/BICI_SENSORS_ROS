//This script is used just for debugging and testing
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <bici_ros_sensor_reader/TactileData.h>
#include <algorithm>

using namespace std;
#define NUM_SENSOR_MIN 8
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


#define SENSORS_NUM 20

class Tactile_Sensor {
  public:
    uint8_t sensor_num;        // ID of the sensor
    int size;                  // The number of taxels
    string sensor_name;          // The link on which the sensor is installed
    uint32_t timestamp = 0; // This can be assigned to any random number
    std::vector<float> data;
    Tactile_Sensor(uint8_t sensor_num, int size, string sensor_name){
        //Any other number can be used for initialization
        this->sensor_num = sensor_num;
        this->size = size;
        this->sensor_name = sensor_name;
        this->data.resize(size);
        fill_n(this->data.begin(),this->size, 1.0);
        //this->data = {1};

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
//Tactile_Sensor sensor11(18, PF_SIZE, "tpf");
Tactile_Sensor sensor12(19, TMB_SIZE, "tmb");
//Tactile_Sensor sensor13(20, PB_SIZE, "tpb");
Tactile_Sensor sensor14(21, PALM_SIZE, "palm");
Tactile_Sensor sensor15(22, BOH_SIZE, "boh");
Tactile_Sensor sensor16(23, FT_SIZE, "ift");
Tactile_Sensor sensor17(24, MF_SIZE, "imf");
Tactile_Sensor sensor18(25, PF_SIZE, "ipf");
Tactile_Sensor sensor19(26, MB_SIZE, "imb");
Tactile_Sensor sensor20(27, PB_SIZE, "ipb");
Tactile_Sensor sensor21(28, FT_SIZE, "mft");
Tactile_Sensor sensor22(29, MF_SIZE, "mmf");



Tactile_Sensor Sensors_Array[SENSORS_NUM]{
  sensor1, 
  sensor2, 
  sensor3,
  sensor4, 
  sensor5, 
  sensor6, 
  sensor7, 
  sensor8,
  sensor9, 
  sensor10, 
  //sensor11, 
  sensor12, 
  //sensor13, 
  sensor14, 
  sensor15,
  sensor16, 
  sensor17, 
  sensor18, 
  sensor19,
  sensor20,
  sensor21, 
  sensor22
  };

int main(int argc, char **argv)
{
  uint8_t sensor_number;
  ros::init(argc, argv, "sensor_emulator");
  ros::NodeHandle nh;
  bici_ros_sensor_reader::TactileData msg;
  std::array<ros::Publisher,SENSORS_NUM> publishers;

  for (size_t i = 0; i < SENSORS_NUM; i++)
  {
    sensor_number = Sensors_Array[i].sensor_num;
   //publishers[i] = nh.advertise<bici_ros_sensor_reader::TactileData>("sensor_"+std::to_string(i+NUM_SENSOR_MIN)+"_readings", 1000);
   publishers[i] = nh.advertise<bici_ros_sensor_reader::TactileData>("sensor_"+std::to_string(sensor_number)+"_readings", 1000);
  }
   
  ros::Rate rate(10);
  while (ros::ok()){

  for (size_t i = 0; i < SENSORS_NUM; i++)
  {
    msg.timestamp = Sensors_Array[i].timestamp;
    msg.sensor_num = Sensors_Array[i].sensor_num;
    msg.data = Sensors_Array[i].data;
    publishers[i].publish(msg);
  }
    ros::spinOnce();
    rate.sleep();
  }


  return 0;
}
// %EndTag(FULLTEXT)%