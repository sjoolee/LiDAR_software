import serial
import math
import numpy as np
import open3d as o3d


num_scans = 3 #number of scans
resolution = 32 #resolution number
scan_distance = 100 #between scan distance

# get the number of scans (inputted by the user) and send to microcontroller
flag = 0
while(flag == 0):
    placeholder = -1
    try:
        placeholder = int(input("Specify number of scans: "))
    except:
        pass
    if(0 < placeholder <= 100):
        flag=1
        num_scans = placeholder
        print("You have entered " + str(num_scans) + " scans")
    else:
        print("Please enter another valid number")



# Set serial parameters and open port
s = serial.Serial('COM6', 115200, timeout=10)
                            
print("Opening: " + s.name) #will show you the port that is opening

# reset the buffers of the UART port to delete the remaining data in the buffers
s.reset_output_buffer()
s.reset_input_buffer()

# wait for user's signal to start the program
input("Press Enter to start communication...")

# send the characters 'sl' to MCU via UART which will signal the MCU to start the transmission
s.write(b'sl')



# create a double array of dimension num_scans * resolution
scanned_data = []
for i in range(num_scans):
    scan = []
    for j in range (resolution):
        x = s.readline()
        number = int(x.decode())
        print(number)
        scan.append(number)
    scanned_data.append(scan)
#close the port
print("Closing: " + s.name)
s.close()

# print raw data
print("Raw data\n")
print(scanned_data)





# convert to cartesian coordinates (x,y) from polar (r,theta) 
f = open("data.xyz", "w")    #create a new file for writing 
cartesian_data = []
for i in range(num_scans):
    z_coordinate= i*scan_distance 
    for j in range (resolution):
        angle = math.radians(j*360/resolution)        # convert to radians
        x = scanned_data[i][j]*math.sin(angle)        # x coordinate == RsinTheta
        y = scanned_data[i][j]*math.cos(angle)        # y coordinate == RcosTheta
        cartesian_data.append([x,y,z_coordinate])
        f.write('{} {} {}\n'.format(x,y,z_coordinate))
f.close()

# print formatted data
print("Formatted data\n")
print(cartesian_data)


# plot in open3d and read the created file data   
point_cloud = o3d.io.read_point_cloud("data.xyz", format="xyz")

#Numerically demonstrates the point cloud data    
print("The point cloud data array:")
print(np.asarray(point_cloud.points))

#Graphically representing point cloud data     
print("Graphical representation of the point_cloud: ")
o3d.visualization.draw_geometries([point_cloud])


# number of vertices is the same as the resolution
num_vertices = resolution

#Gives each vertex a unique number
yz_slice_vertex = []
for x in range(0,num_vertices*num_scans):
    yz_slice_vertex.append([x])


#Define coordinates to connect lines in each yz_coordinateslice        
lines = []  
for x in range(0,num_vertices*num_scans,num_vertices):
    for i in range(num_vertices):
        lines.append([yz_slice_vertex[x+i], yz_slice_vertex[x+((i+1)%num_vertices)]])

#Define coordinates to connect lines between current and next yz_coordinateslice
for x in range(0,num_vertices*(num_scans-1),num_vertices):
    for i in range(num_vertices):
        lines.append([yz_slice_vertex[x+i], yz_slice_vertex[x+num_vertices+i]])

#This line maps the lines to the 3d coordinate vertices
line_set = o3d.geometry.LineSet(points=o3d.utility.Vector3dVector(np.asarray(point_cloud.points)),lines=o3d.utility.Vector2iVector(lines))

#Lets see what our point cloud data with lines looks like graphically       
o3d.visualization.draw_geometries([line_set])

