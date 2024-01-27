import csv
import math 
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Enter the path of data.csv
path = "C://Users//bsgx043//Desktop//Seokchang//Forward-and-Inverse-kinematics//test//bend_data.csv"
path2 = "C://Users//bsgx043//Desktop//Seokchang//Forward-and-Inverse-kinematics//test//yaw_data.csv"
path3 = "C://Users//bsgx043//Desktop//Seokchang//Forward-and-Inverse-kinematics//test//bend_base_data.csv"

# Data reading
# row = how much the motor move? = # resolution = # * 0.29296875 deg 
# 1 col = orientation of the end-effector (rad)
# 2-3 col = position of the end-effetor (x,z)-yawing and (y,z)-bending (mm)
f = open(path,'r')
bend_data = list(csv.reader(f, delimiter=","))
bend_data = np.array(bend_data)
f.close
f = open(path2,'r')
yaw_data = list(csv.reader(f, delimiter=","))
yaw_data = np.array(yaw_data)
f.close
f = open(path3,'r')
bend_base_data = list(csv.reader(f, delimiter=","))
bend_base_data = np.array(bend_base_data)
f.close

# Right/Left = x
# Down/up = y
def forwardKinematics(yaw, bend):

    global bend_data
    global yaw_data
    global bend_base_data
    
    # Direction setup
    if bend < 0 :
        bend = -bend
        ben_pos = np.array([0, (-1)*float(bend_data[bend][1]), float(bend_data[bend][2])]) # (x,y,z)
        ben_base_pos = np.array([0, (-1)*float(bend_base_data[bend][1]), float(bend_base_data[bend][2])])
        xA = float(bend_base_data[bend][0])
    else:
        ben_pos = np.array([0, float(bend_data[bend][1]), float(bend_data[bend][2])])
        ben_base_pos = np.array([0, float(bend_base_data[bend][1]), float(bend_base_data[bend][2])])   
        xA = (-1)*float(bend_base_data[bend][0])

    if yaw < 0 :
        yaw = -yaw
        yaw_pos = np.array([(-1)*float(yaw_data[yaw][1]), 0, float(yaw_data[yaw][2])])
        yA = (-1)*float(yaw_data[yaw][0])
        yaw_pos = np.array([yaw_pos[0]-(4.5*math.sin(abs(yA))), 0, yaw_pos[2]+(4.5*math.sin(abs(yA)))]) # Rigid part(yaw-bend) is set to 4.5mm
        
    else:
        yaw_pos = np.array([float(yaw_data[yaw][1]), 0, float(yaw_data[yaw][2])])
        yA = float(yaw_data[yaw][0])
        yaw_pos = np.array([yaw_pos[0]+(4.5*math.sin(abs(yA))), 0, yaw_pos[2]+(4.5*math.sin(abs(yA)))]) # Rigid part(yaw-bend) is set to 4.5mm
    
    # Rotation for yawing and bending
    rot_mat_x = np.array([[1, 0, 0], [0, math.cos(xA), -math.sin(xA)], [0, math.sin(xA), math.cos(xA)]])
    rot_mat_y = np.array([[math.cos(yA), 0, math.sin(yA)], [0, 1, 0], [-math.sin(yA), 0, math.cos(yA)]])
    rot_pos_yaw = np.dot(rot_mat_x,yaw_pos)
    rot_pos_bend = np.dot(rot_mat_y,ben_pos)
    rot_pos_bend_base = np.dot(rot_mat_y,ben_base_pos)
 
    # Translation the bending joint to tip of the yawing joint
    tran_pos = rot_pos_yaw-rot_pos_bend_base
    # tran_pos = np.array([rot_pos_yaw[0]-rot_pos_bend_base[0], rot_pos_yaw[1]-rot_pos_bend_base[1], rot_pos_yaw[2]-rot_pos_bend_base[2]])
    ee_pos = rot_pos_bend+tran_pos
    # ee_pos = np.array([rot_pos_bend[0]-tran_pos[0], rot_pos_bend[1]-tran_pos[1], rot_pos_bend[2]-tran_pos[2]]) 

    return yaw_pos, ben_base_pos, ben_pos, ee_pos, yA, xA

yawPos, BendBase, BendEnd, EEPos, yA, xA = forwardKinematics(0, 10)

print("EE Position", EEPos)
print("xA: ", xA)
print("yA: ", yA)

X00 = np.array([0, 0])
Y00 = np.array([0, 0])
Z00 = np.array([0, 138])

X01 = np.array([0, yawPos[0]])
Y01 = np.array([0, yawPos[1]])
Z01 = np.array([138, yawPos[2]])

X12 = np.array([yawPos[0], BendBase[0]])
Y12 = np.array([yawPos[1], BendBase[1]])
Z12 = np.array([yawPos[2], BendBase[2]])

X23 = np.array([BendBase[0], BendEnd[0]])
Y23 = np.array([BendBase[1], BendEnd[1]])
Z23 = np.array([BendBase[2], BendEnd[2]])

X34 = np.array([BendEnd[0], EEPos[0]])
Y34 = np.array([BendEnd[1], EEPos[1]])
Z34 = np.array([BendEnd[2], EEPos[2]])


# Creating figure
fig = plt.figure()

# Adding 3D subplot
ax = fig.add_subplot(111, projection='3d')

# Plotting the lines
ax.plot3D(X00, Y00, Z00, 'gray')
ax.plot3D(X01, Y01, Z01, 'red')
ax.plot3D(X12, Y12, Z12, 'gray')
ax.plot3D(X23, Y23, Z23, 'blue')
ax.plot3D(X34, Y34, Z34, 'gray')

# Setting aspect ratio to be equal
max_range = np.array([X00.max()-X00.min(), Y00.max()-Y00.min(), Z00.max()-Z00.min()]).max() / 2.0
mid_x = (X00.max()+X00.min()) * 0.5
mid_y = (Y00.max()+Y00.min()) * 0.5
mid_z = (Z00.max()+Z00.min()) * 0.5
ax.set_xlim(mid_x - max_range, mid_x + max_range)
ax.set_ylim(mid_y - max_range, mid_y + max_range)
ax.set_zlim(mid_z - max_range, mid_z + max_range)

# Labeling axes
ax.set_xlabel('X axis')
ax.set_ylabel('Y axis')
ax.set_zlabel('Z axis')

# Title
ax.set_title('FK results')

# Show plot
plt.show()



############ The FK is based on superposition derived from look up tables. This means that we are only able to
############ predict the location of the end effector, we are not able to determine the pose of the rest
############ of the robot.