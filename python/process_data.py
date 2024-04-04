import serial
import math
import open3d as o3d
import numpy as np

ROTATIONS = 1
ANGLE = 11.25
measurements = []
map = []

s = serial.Serial('COM6', 115200)

s.reset_input_buffer()
s.reset_output_buffer()

s.write(b's')

while(True):
    # Read a line from the serial buffer
    line = s.readline()

    if line.decode('utf-8').strip() == 'x':
        break
    
    # Check if the line is not empty
    if line:
        # Print the line
        measurements.append(line.decode('utf-8').strip())

s.close()


z = 10
counter = 0
for i in range(len(measurements)):
    x = float(measurements[i]) * math.sin(math.radians(ANGLE)*(i%(len(measurements)/ROTATIONS)))  # Ensure counter stays within 0 to 7
    y = float(measurements[i]) * math.cos(math.radians(ANGLE)*(i%(len(measurements)/ROTATIONS)))  # Ensure counter stays within 0 to 7


    if (i) % (len(measurements) / ROTATIONS) == 0:  # Check if it's time to increase z
        z += 50
    
    map.append([x, y, z])

    counter = counter + 1



points = np.array(map)
# Create a PointCloud object
pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(points)

# Compute k-nearest neighbors
kdtree = o3d.geometry.KDTreeFlann(pcd)
k = 10  # Choose the number of nearest neighbors to connect
lines = []

for i in range(len(points)):
    [k, idx, _] = kdtree.search_knn_vector_3d(pcd.points[i], k)
    for j in range(1, k):  # Start from 1 to avoid connecting to itself
        lines.append([i, idx[j]])

# Create LineSet object
line_set = o3d.geometry.LineSet()
line_set.points = o3d.utility.Vector3dVector(points)
line_set.lines = o3d.utility.Vector2iVector(lines)

# Visualize the PointCloud with lines connecting k-nearest neighbors
o3d.visualization.draw_geometries([pcd, line_set])