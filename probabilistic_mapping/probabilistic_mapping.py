"""line_follower controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
import numpy as np
import matplotlib.pyplot as plt
import math

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getDevice('motorname')
#  ds = robot.getDevice('dsname')
#  ds.enable(timestep)

# Ground facing sensors
g_s = []
for i in range(3):
    g_s.append(robot.getDevice(f'gs{i}'))
    g_s[-1].enable(timestep)

# Lidar
lidar = robot.getDevice("lidar")
lidar.enable(timestep)
lidar.enablePointCloud()

# GPS
gps = robot.getDevice("gps")
gps.enable(timestep)

# Compass
compass = robot.getDevice("compass")
compass.enable(timestep)

# Display
display = robot.getDevice("display")


motorL = robot.getDevice('left wheel motor')
motorR = robot.getDevice('right wheel motor')
motorL.setPosition(float('inf'))
motorR.setPosition(float('inf'))
MAX_SPEED = 3

omegaz = 0

r = 0.0201
d = 0.052
map = np.zeros((300, 300))
# xw = 0
# yw = 0
# alpha = 1.56764

def world2map(xw, yw):
    w_P_f = [-0.306 + 0.5, -0.25 - 0.5]          # floor origin at corner
    
    # Coordinate transform
    xf = xw + w_P_f[0]
    yf = -(yw + w_P_f[1])         # Since the frame is rotated
    
    # Scaling
    scale_factor = 300
    px = int(xf*scale_factor)
    py = int(yf*scale_factor)
    
    return [px if px<300 and px>0 else 299, py if py<300 and py>0 else 299]

# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    
    gs_v = []
    for i in range(3):
        gs_v.append(g_s[i].getValue())    
    
    dark = 350
    bright = 550
    
    # LINE FOLLOWING CONTROL
    
    if (gs_v[0] > bright and gs_v[1] >bright):
        # Turn right if two leftmost sensors are out of line
        # print("RIGHT TURN")
        lv = 0.25*MAX_SPEED
        rv = -0.1*MAX_SPEED
    
    elif (gs_v[1] > bright and gs_v[2] >bright):
        # Turn left if two rightmost sensors are out of line
        # print("LEFT TURN")
        lv = -0.1*MAX_SPEED
        rv = 0.25*MAX_SPEED
    
    elif (gs_v[0] < dark and gs_v[1] < dark and gs_v[2] < dark):
        # Stop when all sensors are within line
        # print("STOP")
        lv = 0.0
        rv = 0.0
        # Print error since robot stops at last
        # print(f'error : {error}')
        
    else :
        # print("FORWARD")
        lv = MAX_SPEED
        rv = MAX_SPEED
    
    
    phildot = lv
    phirdot = rv
    
    motorL.setVelocity(phildot)
    motorR.setVelocity(phirdot)
    
    ## ODOMETRY CODE
    xw = gps.getValues()[0]                                           # Set initial position according to gps
    yw = gps.getValues()[1]
    alpha = np.arctan2(compass.getValues()[0], compass.getValues()[1]) # Set initial orientation according to compass
    
    ## Incremental changes in robot frame
    # dx =  (r/2)*(phildot + phirdot)*(timestep/1000) # movement in x axis
    # dw = (r/d)*(phirdot - phildot)*(timestep/1000)  # rotation about z axis
    # omegaz += dw   # Suming up rotations to get actual rotation of robot frame w.r.t. world frame
    
    ## Position and orientation calculations
    
    # alpha = alpha + dw
    # xw = xw + np.cos(alpha)*dx
    # yw = yw + np.sin(alpha)*dx
    
    # error = np.sqrt(xw**2 + yw**2)
    
    ## MAPPING
    angles = np.linspace(np.pi, -np.pi, 360)
    ranges = np.array(lidar.getRangeImage())
    ranges = [1.1 if math.isinf(x) else x for x in ranges]
    pos_r = np.array([[np.multiply(ranges, np.cos(angles))],
                      [np.multiply(ranges, np.sin(angles))]]).reshape(2, 360)
    
    pos_r = np.vstack([pos_r, np.ones((360,))])
    
    w_T_r = np.array([[np.cos(alpha), - np.sin(alpha), xw],
                      [np.sin(alpha), np.cos(alpha), yw],
                      [0, 0, 1]])
    
    # print(pos_r)
    pos_w = w_T_r @ pos_r
    
    
    # DISPLAY TRAJECTORY
    display.setColor(0x00FF00)
    display.setAlpha(1)
    px, py = world2map(xw, yw)
    display.drawPixel(px, py)
    
    
    # DISPLAY MAP
    
    
    
    for i in range(len(pos_w[0])):
        pos_disp = world2map(pos_w[0][i], pos_w[1][i])
        px = pos_disp[0]
        py = pos_disp[1]

        if map[px][py] < 1:
            map[px][py] += 0.01                            # Probabilistic Mapping, increment intensity of obstacle
        
        v = int(map[px][py]*255)         # Obstacle Intensity
        color_val = v*256**2 + v*256 + v
        display.setColor(color_val)
        # display.setAlpha(v/255)
        print(v)
        display.drawPixel(pos_disp[0], pos_disp[1])
    
    
    
       
    # DISPLAY
    
    # print(f'omegaz = {np.rad2deg(omegaz)}')
    # print(f'xw = {xw}, yw = {yw}')
    # print("-----------------------------")
    
    pass


