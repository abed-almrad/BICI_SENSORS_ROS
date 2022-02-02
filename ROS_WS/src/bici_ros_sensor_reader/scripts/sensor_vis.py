#!/usr/bin/env python3

import rospy
import seaborn as sns
import matplotlib.pyplot as plt
from bici_ros_sensor_reader.msg import TactileData
import numpy as np



class SensorDataVis:
    def __init__(self, sensor_num):
        self.sensor_num = sensor_num
        #self.data_min = np.inf
        #self.data_max = -np.inf

        taxels = {}
        #P.S. Make sure if the thumb sensors are identical to those of the other fingers
        self.pf = [8, 13, 25]       #proximal front sensor
        self.pb = [10, 15, 27]      #proximal back sensor
        self.mf = [12, 24, 29]      #medial front sensor
        self.mb = [9, 14, 26]       #medial back sensor
        self.ft = [11, 16, 23, 28]      #fingertip sensor
        self.palm = [21]                #palm sensor
        self.boh = [22]                 #back of hand sensor
        self.tmb = [19]                 #Back of thumb sensor
        self.tmf = [17]                 #Front of thumb sensor
        taxels.update(dict.fromkeys(self.pf, (10, 11)))
        taxels.update(dict.fromkeys(self.pb, (11, 11)))
        taxels.update(dict.fromkeys(self.mf, (8, 6)))
        taxels.update(dict.fromkeys(self.ft, (13, 9)))
        taxels.update(dict.fromkeys(self.palm, (13, 13)))
        taxels.update(dict.fromkeys(self.boh, (11,12)))
        taxels.update(dict.fromkeys(self.mb, (9,9)))
        taxels.update(dict.fromkeys(self.tmb, (14,7)))
        taxels.update(dict.fromkeys(self.tmf, (10,7)))

        self.taxels = taxels

    def norm_tactile(self, data):
        #**************************************
        #***************************************
    #P.S. Data normalization is still not final and needs to be revisited
        #**************************************
        #***************************************

        #self.data_min = min(self.data_min, min(data))
        #self.data_max = max(self.data_max, max(data))
        #normalized_data = (data - self.data_min)/(self.data_max-self.data_min)
        normalized_data = (data - data.min())/(data.max()-data.min())
        #print(normalized_data)
        return normalized_data#.clip(0, 1)

    def transform_pb(self, data):
        data = np.asarray(data)
        #print(data)
        normalized_data = self.norm_tactile(data)
        #print("Normalized data: " + str(normalized_data) + "\n")
        #Slicing and fake inactive taxels insertion
        slice1 = normalized_data[0:5]
        slice1 = np.append(slice1,[0])

        slice2 = normalized_data[5:10]
        slice2 = np.append(slice2,[0])

        slice3 = normalized_data[10:19]
        slice3 = np.append(slice3,[0,0])

        slice4 = normalized_data[19:28]
        slice4 = np.append(slice4,[0])

        slice5 = normalized_data[28:50]
        slice5 = np.append(slice5,[0,0,0])

        slice6 = normalized_data[50:56]
        slice6 = np.append(slice6,[0,0,0,0,0])

        slice7 = normalized_data[56:62]
        slice7 = np.append(slice7,[0,0,0,0,0])

        slice8 = normalized_data[62:64]
        slice8 = np.append(slice8,[0,0])

        slice9 = normalized_data[64:66]
        slice9 = np.append(slice9,[0,0,0,0,0])

        slice10 = normalized_data[66:68]
        slice10 = np.append(slice10,[0,0])

        slice11 = normalized_data[68:70]
        slice11 = np.append(slice11,[0,0,0,0,0])

        slice12 = normalized_data[70:72]
        slice12 = np.append(slice12,[0,0])

        slice13 = normalized_data[72:74]
        slice13 = np.append(slice13,[0,0,0,0,0])

        slice14 = normalized_data[74:76]
        slice14 = np.append(slice14,[0,0])

        slice15 = normalized_data[76:78]
        slice15 = np.append(slice15,[0,0])

        #Resulting data
        modified_data = np.concatenate((slice1,slice2,slice3,slice4,slice5,slice6,slice7,slice8,slice9,slice10,slice11,slice12,slice13,slice14,slice15))
        modified_data = np.reshape(modified_data, self.taxels[self.sensor_num])
        #print("Modified data: " + str(modified_data) + "\n")
        return modified_data

    def transform_pf(self, data):
        data = np.asarray(data)
        #print(data)
        normalized_data = self.norm_tactile(data)
        #print("Normalized data: " + str(normalized_data) + "\n")
        #Slicing and fake inactive taxels insertion
        slice1 = normalized_data[0:33]
        slice1 = np.append(slice1,[0,0,0])

        slice2 = normalized_data[33:39]
        slice2 = np.append(slice2,[0,0,0,0,0])

        slice3 = normalized_data[39:45]
        slice3 = np.append(slice3,[0,0,0,0,0])

        slice4 = normalized_data[45:47]
        slice4 = np.append(slice4,[0,0])

        slice5 = normalized_data[47:49]
        slice5 = np.append(slice5,[0,0,0,0,0])

        slice6 = normalized_data[49:51]
        slice6 = np.append(slice6,[0,0])

        slice7 = normalized_data[51:53]
        slice7 = np.append(slice7,[0,0,0,0,0])

        slice8 = normalized_data[53:55]
        slice8 = np.append(slice8,[0,0])

        slice9 = normalized_data[55:57]
        slice9 = np.append(slice9,[0,0,0,0,0])

        slice10 = normalized_data[57:59]
        slice10 = np.append(slice10,[0,0])

        slice11 = normalized_data[59:61]
        slice11 = np.append(slice11,[0,0,0,0,0])

        slice12 = normalized_data[61:63]
        slice12 = np.append(slice12,[0,0])

        slice13 = normalized_data[63:65]
        slice13 = np.append(slice13,[0,0])

        #Resulting data
        modified_data = np.concatenate((slice1,slice2,slice3,slice4,slice5,slice6,slice7,slice8,slice9,slice10,slice11,slice12,slice13))
        modified_data = np.reshape(modified_data, self.taxels[self.sensor_num])
        #print("Modified data: " + str(modified_data) + "\n")
        return modified_data

    def transform_boh(self, data):
        data = np.asarray(data)
        #print(data)
        normalized_data = self.norm_tactile(data)
        #print("Normalized data: " + str(normalized_data) + "\n")
        #Slicing and fake inactive taxels insertion
        slice1 = normalized_data[0:111]
        slice1 = np.append(slice1,[0,0,0,0,0,0])

        slice2 = normalized_data[111:114]
        slice2 = np.append(slice2,[0])

        slice3 = normalized_data[114:116]
        slice3 = np.append(slice3,[0,0,0,0,0,0])

        slice4 = normalized_data[116:118]
        slice4 = np.append(slice4,[0])
        #Resulting data
        modified_data = np.concatenate((slice1,slice2,slice3,slice4))
        modified_data = np.reshape(modified_data, self.taxels[self.sensor_num])
        modified_data = np.flip(modified_data, axis=1)
        #print("Modified data: " + str(modified_data) + "\n")
        return modified_data

    def transform_mf(self, data):
        data = np.asarray(data)
        #print(data)
        normalized_data = self.norm_tactile(data)
        #print("Normalized data: " + str(normalized_data) + "\n")
        #Slicing and fake inactive taxels insertion
        slice1 = normalized_data[0:4]
        slice1 = np.append(slice1,[0,0])

        slice2 = normalized_data[4:8]
        slice2 = np.append(slice2,[0,0])

        slice3 = normalized_data[8:12]
        slice3 = np.append(slice3,[0,0,0])

        slice4 = normalized_data[12:15]
        slice4 = np.append(slice4,[0,0,0])

        slice5 = normalized_data[15:18]
        slice5 = np.append(slice5,[0,0,0])

        slice6 = normalized_data[18:21]
        slice6 = np.append(slice6,[0,0,0])

        slice7 = normalized_data[21:24]
        slice7 = np.append(slice7,[0,0,0])

        slice8 = normalized_data[24:25]
        slice8 = np.append(slice8,[0,0])

        slice9 = normalized_data[25:27]
        
        #Resulting data
        modified_data = np.concatenate((slice1,slice2,slice3,slice4,slice5,slice6,slice7,slice8,slice9))
        modified_data = np.reshape(modified_data, self.taxels[self.sensor_num])
        #print("Modified data: " + str(modified_data) + "\n")
        modified_data = np.flip(modified_data, axis=1)
        return modified_data

    def transform_ft(self, data):
        data = np.asarray(data)
        #print(data)
        normalized_data = self.norm_tactile(data)
        #print("Normalized data: " + str(normalized_data) + "\n")
        #Slicing and fake inactive taxels insertion
        slice1 = normalized_data[0:18]
        slice1 = np.append(slice1,[0])

        slice2 = normalized_data[18:25]
        slice2 = np.append(slice2,[0,0,0])

        slice3 = normalized_data[25:30]
        slice3 = np.append(slice3,[0,0,0,0,0])

        slice4 = normalized_data[30:34]
        slice4 = np.append(slice4,[0,0,0,0,0])

        slice5 = normalized_data[34:38]
        slice5 = np.append(slice5,[0,0,0,0,0])

        slice6 = normalized_data[38:42]
        slice6 = np.append(slice6,[0,0,0,0,0])

        slice7 = normalized_data[42:46]
        slice7 = np.append(slice7,[0,0,0,0,0])

        slice8 = normalized_data[46:50]
        slice8 = np.append(slice8,[0,0,0,0,0])

        slice9 = normalized_data[50:54]
        slice9 = np.append(slice9,[0,0,0,0,0])

        slice10 = normalized_data[54:58]
        slice10 = np.append(slice10,[0,0,0,0,0])

        slice11 = normalized_data[58:62]
        slice11 = np.append(slice11,[0,0,0,0,0])

        slice12 = normalized_data[62:66]
        slice12 = np.append(slice12,[0,0])

        #Resulting data
        modified_data = np.concatenate((slice1,slice2,slice3,slice4,slice5,slice6,slice7,slice8,slice9,slice10,slice11,slice12))
        modified_data = np.reshape(modified_data, self.taxels[self.sensor_num])
        modified_data = np.flip(modified_data, axis=1)
        #print("Modified data: " + str(modified_data) + "\n")
        return modified_data
    
    def transform_palm(self, data):
        data = np.asarray(data)
        #print(data)
        normalized_data = self.norm_tactile(data)
        #print("Normalized data: " + str(normalized_data) + "\n")
        #Slicing and fake inactive taxels insertion
        slice = np.zeros(1)
        lower_matrix_slices = [slice]*6
        for i in range(6):
            lower_matrix_slices[i] = [normalized_data[i],normalized_data[6+i],normalized_data[12+i],normalized_data[18+i],
            normalized_data[24+i],normalized_data[30+i],normalized_data[36+i],normalized_data[42+i],normalized_data[48+i],normalized_data[54+i],
            normalized_data[60+i]]
        upper_matrix_slices = [slice]*6
        for i in range(5):
            upper_matrix_slices[i] = [normalized_data[66+i],normalized_data[72+i],normalized_data[78+i],normalized_data[84+i],
            normalized_data[90+i],normalized_data[96+i],normalized_data[102+i]]
            upper_matrix_slices[i] = np.append(upper_matrix_slices[i],[0,0,0,0,0,0])

        upper_matrix_slices[5] = [normalized_data[66+5],normalized_data[72+5],normalized_data[78+5],normalized_data[84+5],
        normalized_data[90+5],normalized_data[96+5],normalized_data[102+5]]
        upper_matrix_slices[5] = np.append(upper_matrix_slices[5],[0,0,0,0,0,0,0])

        lower_matrix_slices[0] = np.append(lower_matrix_slices[0],[normalized_data[108],normalized_data[110]])
        lower_matrix_slices[1] = np.append(lower_matrix_slices[1],[normalized_data[109],normalized_data[111]])        
        lower_matrix_slices[2] = np.append(lower_matrix_slices[2],[0,0])
        lower_matrix_slices[3] = np.append(lower_matrix_slices[3],[0,0])
        lower_matrix_slices[4] = np.append(lower_matrix_slices[4],[normalized_data[112],normalized_data[114]]) 
        lower_matrix_slices[5] = np.append(lower_matrix_slices[5],[normalized_data[113],normalized_data[115]])   

        final_row_slice = normalized_data[116:121]
        final_row_slice = np.append(final_row_slice,[0,0,0,0,0,0,0])

        #Resulting data
        modified_data = lower_matrix_slices[0]
        for i in range(1,6):
            modified_data = np.concatenate((modified_data,lower_matrix_slices[i]))
        for i in range(6):
            modified_data = np.concatenate((modified_data,upper_matrix_slices[i])) 
        modified_data = np.concatenate((modified_data,final_row_slice))      
        modified_data = np.reshape(modified_data, self.taxels[self.sensor_num])
        #print("Modified data: " + str(modified_data) + "\n")
        return modified_data

    def transform_mb(self, data):
        data = np.asarray(data)
        #print(data)
        normalized_data = self.norm_tactile(data)
        #print("Normalized data: " + str(normalized_data) + "\n")
        #Slicing and fake inactive taxels insertion
        normalized_data = np.insert(normalized_data,0,[0,0,0,0,0,0,0,0,0,0])

        slice0 = normalized_data[0:10]

        slice1 = normalized_data[10:14]
        slice1 = np.append(slice1,[0,0,0,0,0])

        slice2 = normalized_data[14:18]
        slice2 = np.append(slice2,[0,0,0,0])

        slice3 = normalized_data[18:24]
        slice3 = np.append(slice3,[0,0,0,0,0,0])

        slice4 = normalized_data[24:27]
        slice4 = np.append(slice4,[0,0,0,0,0,0])

        slice5 = normalized_data[27:30]
        slice5 = np.append(slice5,[0,0,0,0,0,0])

        slice6 = normalized_data[30:32]
        slice6 = np.append(slice6,[0])

        slice7 = normalized_data[32:35]
        slice7 = np.append(slice7,[0,0,0])

        slice8 = normalized_data[35:36]
        slice8 = np.append(slice8,[0,0])

        slice9 = normalized_data[36:39]
        slice9 = np.append(slice9,[0,0,0])

        slice10 = normalized_data[39:40]
        slice10 = np.append(slice10,[0,0,0,0,0])
        #Resulting data
        modified_data = np.concatenate((slice0,slice1,slice2,slice3,slice4,slice5,slice6,slice7,slice8,slice9,slice10))
        modified_data = np.reshape(modified_data, self.taxels[self.sensor_num])
        #print("Modified data: " + str(modified_data) + "\n")
        modified_data = np.flip(modified_data, axis=1)
        return modified_data

    def transform_tmb(self, data):
        data = np.asarray(data)
        #print(data)
        normalized_data = self.norm_tactile(data)
        #print("Normalized data: " + str(normalized_data) + "\n")
        #Slicing and fake inactive taxels insertion
        slice1 = normalized_data[0:4]
        slice1 = np.append(slice1,[0,0,0])

        slice2 = normalized_data[4:9]
        slice2 = np.append(slice2,[0,0,0])

        slice3 = normalized_data[9:14]
        slice3 = np.append(slice3,[0,0])

        slice4 = normalized_data[14:19]
        slice4 = np.append(slice4,[0])

        slice5 = normalized_data[19:33]
        slice5 = np.append(slice5,[0,0,0])

        slice6 = normalized_data[33:37]
        slice6 = np.append(slice6,[0,0,0])

        slice7 = normalized_data[37:41]
        slice7 = np.append(slice7,[0,0,0])

        slice8 = normalized_data[41:42]
        slice8 = np.append(slice8,[0,0,0,0,0,0])

        slice9 = normalized_data[42:43]
        slice9 = np.append(slice9,[0,0,0,0,0,0])

        slice10 = normalized_data[43:44]
        slice10 = np.append(slice10,[0,0,0,0,0,0])

        slice11 = normalized_data[44:45]
        slice11 = np.append(slice11,[0,0,0,0,0,0])

        slice12 = normalized_data[45:46]
        slice12 = np.append(slice12,[0,0,0,0,0,0])

        slice13 = normalized_data[46:47]
        slice13 = np.append(slice13,[0,0,0])

        #Resulting data
        modified_data = np.concatenate((slice1,slice2,slice3,slice4,slice5,slice6,slice7,slice8,slice9,slice10,slice11,slice12,slice13))
        modified_data = np.reshape(modified_data, self.taxels[self.sensor_num])
        modified_data = np.flip(modified_data, axis=1)
        #print("Modified data: " + str(modified_data) + "\n")
        return modified_data

    def transform_tmf(self, data):
        data = np.asarray(data)
        #print(data)
        normalized_data = self.norm_tactile(data)
        #print("Normalized data: " + str(normalized_data) + "\n")
        #Slicing and fake inactive taxels insertion
        slice1 = normalized_data[0:4]
        slice1 = np.append(slice1,[0,0,0])

        slice2 = normalized_data[6:20]
        slice2 = np.append(slice2,[0,0])

        slice3 = normalized_data[20:23]
        slice3 = np.append(slice3,[0,0,0,0])

        slice4 = normalized_data[23:26]
        slice4 = np.append(slice4,[0,0,0,0])

        slice5 = normalized_data[26:29]
        slice5 = np.append(slice5,[0,0,0,0,0])

        slice6 = normalized_data[5:6]
        slice6 = np.append(slice6,[0,0,0,0,0,0])

        slice7 = normalized_data[4:5]
        slice7 = np.append(slice7,[0,0,0,0,0,0])

        slice8 = normalized_data[30:31]
        slice8 = np.append(slice8,[0,0,0,0,0,0])

        slice9 = normalized_data[29:30]
        slice9 = np.append(slice9,[0,0,0])

        #Resulting data
        modified_data = np.concatenate((slice1,slice2,slice3,slice4,slice5,slice6,slice7,slice8,slice9))
        modified_data = np.reshape(modified_data, self.taxels[self.sensor_num])
        modified_data = np.flip(modified_data, axis=1)
        #print("Modified data: " + str(modified_data) + "\n")
        return modified_data

def callback(msg, sdv):
    if sdv.sensor_num in sdv.pb:
        tactile_data = sdv.transform_pb(msg.data)
    elif sensor_num in sdv.pf:
        tactile_data = sdv.transform_pf(msg.data)
    elif sensor_num in sdv.boh:
        tactile_data = sdv.transform_boh(msg.data)
    elif sensor_num in sdv.mf:
        tactile_data = sdv.transform_mf(msg.data)
    elif sensor_num in sdv.ft:
        tactile_data = sdv.transform_ft(msg.data)
    elif sensor_num in sdv.palm:
        tactile_data = sdv.transform_palm(msg.data)
    elif sensor_num in sdv.mb:
        tactile_data = sdv.transform_mb(msg.data)
    elif sensor_num in sdv.tmb:
        tactile_data = sdv.transform_tmb(msg.data)
    elif sensor_num in sdv.tmf:
        tactile_data = sdv.transform_tmf(msg.data)
        
    else:
        print('issue with sensor number... check it out.')
    im.set_array(tactile_data)
    #fig.canvas.draw()
    #plt.show()
    fig.canvas.draw_idle()
def listener(sdv):
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("sensor_" + str(sensor_num) + "_readings", TactileData, callback, sdv)
    #rospy.spin()

if __name__ == '__main__':
    # ask for user input about what sensor number you want to visualize
    sensor_num = int(input("Please enter a sensor number from 8-29: "))
    if (sensor_num >= 8) and (sensor_num <= 29):
        print("You entered: " + str(sensor_num))
    else:
        sensor_num = int(input("Try it again, friend, something from 8-29: "))

    # initialize a SensorDataVis object
    sdv = SensorDataVis(sensor_num)

    # intialize figure and start the listener node
    fig, ax = plt.subplots(figsize=(11,11))
    im = ax.imshow(np.random.random(sdv.taxels[sdv.sensor_num]), cmap='autumn')
    listener(sdv)
    plt.colorbar(im, ax=ax)
    plt.show(block=True)
    plt.colorbar()




