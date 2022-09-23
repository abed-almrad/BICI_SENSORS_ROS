import numpy as np
import math

threshold = 0.005
further_filtered_contact = np.array([])
#Creating the numpy vector of points from a previously saved csv file
xyz = np.genfromtxt('contacts_coordinates.csv',dtype= float, delimiter=',')
xyz = xyz[:,0:3]
#print(xyz.shape)
xyz_filtered = np.genfromtxt('filtered_contacts_coordinates.csv',dtype= float, delimiter=',')
xyz_filtered = xyz_filtered[:,0:3]
#print(xyz_filtered.shape)
for i in range(xyz_filtered.shape[0]):
    for j in range(xyz.shape[0]):
        if(math.dist(xyz_filtered[i],xyz[j])<threshold):
            further_filtered_contact = np.append(further_filtered_contact,xyz_filtered[i])
            break

further_filtered_contact = further_filtered_contact.reshape(-1,3)
np.savetxt(str(threshold)+'_filtered_contacts_coordinates.csv',further_filtered_contact,delimiter=',')
