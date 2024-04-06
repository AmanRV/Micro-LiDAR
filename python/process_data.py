import serial
import math
import open3d as o3d
import numpy as np

#Constants
ROTATIONS = 3 #number of rotations
ANGLE = 2.8125 #angle of each step in rotation

#Variables
measurements = [] #stores measurement data
map = [] #2d map array with x y z format

s = serial.Serial('COM6', 115200) #set COM port and BAUD rate


s.write(b's')

while(True):

    # Read a line from the serial buffer
    line = s.readline()

    #Stop transmission if 'x' is recieved
    if line.decode('utf-8').strip() == 'x':
        break
    
    # Check if the line is not empty
    if line:
        # add measurement to array
        measurements.append(line.decode('utf-8').strip())

s.close()


z = 100 #initial z level
counter = 0

for i in range(len(measurements)): #loop through every measurement
    x = float(measurements[i]) * math.sin(math.radians(ANGLE)*(i%(len(measurements)/ROTATIONS)))  # Ensure counter stays within 0 to 7
    y = float(measurements[i]) * math.cos(math.radians(ANGLE)*(i%(len(measurements)/ROTATIONS)))  # Ensure counter stays within 0 to 7


    if (i) % (len(measurements) / ROTATIONS) == 0:  # Check if it's time to increase z value
        z += 600
    
    map.append([x, y, z]) #add calculated x,y,z value to map array

    counter = counter + 1



points = np.array(map)

# Create a PointCloud object
pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(points)

# Compute k-nearest neighbors
kdtree = o3d.geometry.KDTreeFlann(pcd)
k = 15  # Choose the number of nearest neighbors to connect 
lines = []

for i in range(len(points)):
    [k, idx, _] = kdtree.search_knn_vector_3d(pcd.points[i], k)
    for j in range(1, k):  # Start from 1 to avoid connecting to itself
        lines.append([i, idx[j]])

# Create LineSet object
line_set = o3d.geometry.LineSet()
line_set.points = o3d.utility.Vector3dVector(points)
line_set.lines = o3d.utility.Vector2iVector(lines)

window_size = (1280, 720) 

# Visualize the PointCloud with lines connecting k-nearest neighbors
o3d.visualization.draw_geometries([pcd, line_set], window_name="Open3D", width=window_size[0], height=window_size[1])