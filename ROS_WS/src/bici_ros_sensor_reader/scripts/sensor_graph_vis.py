#!/usr/bin/env python3

import rospy
import matplotlib.pyplot as plt
from bici_ros_sensor_reader.msg import TactileData
import numpy as np
import time

taxels = {}
#P.S. Make sure if the thumb sensors are identical to those of the other fingers
pf = [8, 13, 25]       #proximal front sensor
pb = [10, 15, 27]      #proximal back sensor
mf = [12, 24, 29]      #medial front sensor
mb = [9, 14, 26]       #medial back sensor
ft = [11, 16, 23, 28]      #fingertip sensor
palm = [21]                #palm sensor
boh = [22]                 #back of hand sensor
tmb = [19]                 #Back of thumb sensor
tmf = [17]                 #Front of thumb sensor
taxels.update(dict.fromkeys(pf, 30))
taxels.update(dict.fromkeys(pb, 30))
taxels.update(dict.fromkeys(mf, 30))
taxels.update(dict.fromkeys(ft, 30))
taxels.update(dict.fromkeys(palm, 30))
taxels.update(dict.fromkeys(boh, 30))
taxels.update(dict.fromkeys(mb, 30))
taxels.update(dict.fromkeys(tmb, 30))
taxels.update(dict.fromkeys(tmf, 30))

class SensorData:
    def __init__(self, sensor_num):
        self.sensor_num = sensor_num
        self.taxel_values = np.zeros(shape=taxels[sensor_num])


def callback(msg, sd):
    for i in range(taxels[sd.sensor_num]):
        sd.taxel_values[i] = msg.data[i]
    print("14th taxel value: ", sd.taxel_values[13])


if __name__ == '__main__':
    # ask for user input about what sensor number you want to visualize
    sensor_num = int(input("Please enter a sensor number from 8-29: "))
    if (sensor_num >= 8) and (sensor_num <= 29):
        print("You entered: " + str(sensor_num))
    else:
        sensor_num = int(input("Try it again, friend, something from 8-29: "))

    # initialize a SensorDataVis object
    sd = SensorData(sensor_num)

    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("sensor_" + str(sensor_num) + "_readings", TactileData, callback, sd)
    rospy.spin()








