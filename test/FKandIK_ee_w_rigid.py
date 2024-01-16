import csv
import math 
import numpy as np

# Enter the path of data.csv
path = "C:/Users/LG/Desktop/bend_data.csv"
path2 = "C:/Users/LG/Desktop/yaw_data.csv"
path3 = "C:/Users/LG/Desktop/bend_base_data.csv"

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

    return ee_pos
'''
# Examples for testing the forward kinematics
yaw = 3
bend = 5

pos = forwardKinematics(3, 5)
pos2 = forwardKinematics(-3,-5)
print(pos)
print(pos2)
'''

def inverseKinematics(x_goal,y_goal,z_goal):

    global bend_data
    global yaw_data
    
    # SETTING at main code # current motor value(how much did it move?)
    global cur_yaw
    global cur_bend
    
    goal_pos = np.array([x_goal, y_goal, z_goal])
    
    yaw_st = cur_yaw
    bend_st = cur_bend
    
    cur_pos = forwardKinematics(yaw_st, bend_st)
    subs = cur_pos - goal_pos
    
    cur_err = np.linalg.norm(subs)
    
    if subs[0] < 0: # Desired position is on rightside of current position
        yaw_count = 1
    elif subs[0] > 0: # leftside
        yaw_count = -1
    else:
        yaw_count = 0
    
    if subs[1] < 0: # upside
        bend_count = 1
    elif subs[1] > 0:
        bend_count = -1 # downside
    else:
        bend_count = 0
        
    flag = True

    while(abs(yaw_st)+2 <= len(yaw_data)):
        
        if flag == False:
            break
        
        bend_st = cur_bend #intialization
        yaw_st = yaw_st + yaw_count
        
        while(abs(bend_st)+2 <= len(bend_data)):
            
            bend_st = bend_st + bend_count

            mod_pos = forwardKinematics(yaw_st, bend_st)
            mod_err = np.linalg.norm(mod_pos-goal_pos)                     
            
            if cur_err <= mod_err:
                if cur_err < 1: # Threshold
                    flag = False
                    break
                
            cur_err = mod_err
                        
    return cur_err, yaw_st, (bend_st-1)

# Examples for testing the inverse kinemaitcs
pos = forwardKinematics(3,5)
pos2 = forwardKinematics(-15,20)

cur_yaw = -4
cur_bend = -5

cur_err, yaw_st, bend_st = inverseKinematics(pos2[0],pos2[1],pos2[2])

print('Goal position: ',pos2)
print('Yawing: ',yaw_st,'(',yaw_st*(0.29296875),'deg)')
print('Bending: ',bend_st,'(',bend_st*(0.29296875),'deg)')
print('Current postion: ',forwardKinematics(yaw_st,bend_st))
print('Current error: ',cur_err,'(mm)')
