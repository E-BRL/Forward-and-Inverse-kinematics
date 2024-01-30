
import os
import sys
import time
import keyboard
import threading
import csv
import tkinter
import math 
import numpy as np


# Dynamixel toolkit required for stepper motor interaction.
from dynamixel_sdk import *

######################################################
#################### Setup OS ########################
######################################################

if os.name == 'nt':     #Windows OS
    import msvcrt
    def getch():
        return msvcrt.getch().decode()
else:
    import sys, tty, termios                    #Linux or other UNIX OS
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

######################################################
##################### VARIABLES ######################
######################################################

# Motor setting
DXL1_ID                    = 6   # Steering(Flexion) 6
DXL2_ID                    = 1   # Yawing 1
DEVICENAME                 = 'COM6' # ex) Windows: "COM1", Linux: "/dev/ttyUSB0", Mac: "/dev/tty.usbserial-*"
BAUDRATE                   = 1000000

MOVING_SPEED               = 31  # 0~1023 x 0.111 rpm (ex-1023:114 rpm) 
                                   # AX-12 : No Load Speed	59 [rev/min] (at 12V)
MOVING_SPEED_SLOW          = 50
DXL_MOVING_STATUS_THRESHOLD = 5

# Control table address
ADDR_MX_TORQUE_ENABLE      = 24
ADDR_MX_GOAL_POSITION      = 30
ADDR_MX_PRESENT_POSITION   = 36
ADDR_MX_MOVING_SPEED       = 32
ADDR_MX_PRESENT_SPEED      = 38

TORQUE_ENABLE              = 1
TORQUE_DISABLE             = 0

PROTOCOL_VERSION           = 1.0

# Data Byte Length
LEN_MX_GOAL_POSITION       = 4
LEN_MX_PRESENT_POSITION    = 4
LEN_MX_MOVING              = 1

# Motor input limit
steering_angle_limit_min = -305 # about -90 deg rotation of the disk
steering_angle_limit_max = 305 # about +90 deg rotation of the disk
yawing_angle_limit = 100 # about +-30 deg rotation of the disk

# Motor initial position (150 deg for each motor)
dxl1_init = 512
dxl2_init = 512


# Initialize global variables
dxl1_goal_position, dxl2_goal_position = 0, 0
dxl1_goal_angle,    dxl2_goal_angle    = 0, 0
dxl1_present_position, dxl2_present_position = 0, 0

scale_factor_steering = 1
scale_factor_yawing   = 1

########################################################
################# Import trajectory ####################
########################################################

# Enter the path of data.csv
path = "C:/Users/LG/Desktop/trajectoryNeedle.csv"
f = open(path,'r')
trajectoryData = list(csv.reader(f, delimiter=","))
trajectoryData = np.array(trajectoryData)
f.close


#####################################################
############### Define Functions ####################
#####################################################

def angleLimits(dxl1_goal_angle, dxl2_goal_angle):
    
    # Ensure dxl_goal_angle stays within limits (unit: control input)
    dxl1_goal_angle = max(min(dxl1_goal_angle, scale_factor_steering * steering_angle_limit_max),
        scale_factor_steering * steering_angle_limit_min)
    dxl2_goal_angle = max(min(dxl2_goal_angle, scale_factor_yawing * yawing_angle_limit),
        -scale_factor_yawing * yawing_angle_limit)
    
    return dxl1_goal_angle, dxl2_goal_angle

def setMoveSpeed(moving_speed):
    global DXL1_ID, DXL2_ID, portHandler

    # Set move speed for DXL1
    dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL1_ID, ADDR_MX_MOVING_SPEED, moving_speed)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))

    # Set Fmove speed for DXL2
    dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL2_ID, ADDR_MX_MOVING_SPEED, moving_speed)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))   
        

def setGoalPositions(dxl1_goal_angle, dxl2_goal_angle):
    global DXL1_ID, DXL2_ID, portHandler

    # Apply desired angle to motor DXL1
    dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL1_ID, ADDR_MX_GOAL_POSITION, dxl1_goal_angle)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))

    # Apply desired angle to motor DXL2
    dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL2_ID, ADDR_MX_GOAL_POSITION, dxl2_goal_angle)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))

######################################################
################# MOTOR CONNECTION ###################
######################################################
        
# Dynamixel setting
portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)

# Open port
if portHandler.openPort():
    print("Succeeded to open the port")
else:
    print("Failed to open the port")
    print("Press any key to terminate...")
    getch()
    quit()

# Set port baudrate
if portHandler.setBaudRate(BAUDRATE):
    print("Succeeded to change the baudrate")
else:
    print("Failed to change the baudrate")
    print("Press any key to terminate...")
    getch()
    quit()

# Enable Dynamixel Torque
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL1_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))
else:
    print("Dynamixel#%d has been successfully connected" % DXL1_ID)

dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL2_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))
else:
    print("Dynamixel#%d has been successfully connected" % DXL2_ID)


#####################################################
############## Initialise Motor Position ############
#####################################################

## 1. Motor Positioning
print("====Positioning Motors to Initial Position=====")

# Set Slow Move Speed
setMoveSpeed(MOVING_SPEED_SLOW)


# Write goal position : initial position = 512 control input
setGoalPositions(dxl1_init, dxl2_init)


while True:
    # Read present position
    dxl1_present_position, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, DXL1_ID, ADDR_MX_PRESENT_POSITION)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    dxl2_present_position, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, DXL2_ID, ADDR_MX_PRESENT_POSITION)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))        

    if not ((abs(dxl1_init - dxl1_present_position) > DXL_MOVING_STATUS_THRESHOLD) and (abs(dxl2_init - dxl2_present_position) > DXL_MOVING_STATUS_THRESHOLD)):
            break

######################################################
time.sleep(0.2)

print("")
user_input = input(">> Enter to start trajectory")
print("")

# Set Fast move speed
setMoveSpeed(MOVING_SPEED)


#####################################################  
############# Work through trajectory ###############
#####################################################

#initialise count
count = 0
while count <= len(trajectoryData):
    
    #yaw motor value from trajectory
    dxl1_goal_angle = int(trajectoryData[count][0])
    
    #bend motor value from trajectory
    dxl2_goal_angle = int(trajectoryData[count][1])
    
    #Apply limits to desired angles
    dxl1_goal_angle, dxl2_goal_angle = angleLimits(dxl1_goal_angle, dxl2_goal_angle)
    
    #Apply desired angles to motors
    setGoalPositions(dxl1_goal_angle, dxl2_goal_angle)

    #increment count
    count = count+1
        
    #20ms wait time
    time.sleep(0.02) 
    
    

##########################################################
################ Quit the Program ########################
##########################################################

# Disable Dynamixel Torque
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL1_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))

dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL2_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))

# Close port
portHandler.closePort()

sys.exit(0)    