"""line_follower controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
import numpy as np

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getDevice('motorname')
#  ds = robot.getDevice('dsname')
#  ds.enable(timestep)

g_s = []
for i in range(3):
    g_s.append(robot.getDevice(f'gs{i}'))
    g_s[-1].enable(timestep)

motorL = robot.getDevice('left wheel motor')
motorR = robot.getDevice('right wheel motor')
motorL.setPosition(float('inf'))
motorR.setPosition(float('inf'))
MAX_SPEED = 3

xw = 0
yw = 0
alpha = np.pi/2
omegaz = 0

r = 0.0201
d = 0.052

# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    
    gs_v = []
    for i in range(3):
        gs_v.append(g_s[i].getValue())    
    
    dark = 350
    bright = 550
    
    # LINE FOLLOWING CODE
    
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
        print(f'error : {error}')
        
    else :
        # print("FORWARD")
        lv = MAX_SPEED
        rv = MAX_SPEED
    
    
    phildot = lv
    phirdot = rv
    
    motorL.setVelocity(phildot)
    motorR.setVelocity(phirdot)
    
    ## ODOMETRY CODE
    
    # Incremental changes in robot frame
    dx =  (r/2)*(phildot + phirdot)*(timestep/1000) # movement in x axis
    dw = (r/d)*(phirdot - phildot)*(timestep/1000)  # rotation about z axis
    omegaz += dw   # Suming up rotations to get actual rotation of robot frame w.r.t. world frame
    
    # Position and orientation calculations
    
    alpha = alpha + dw
    xw = xw + np.cos(alpha)*dx
    yw = yw + np.sin(alpha)*dx
    
    error = np.sqrt(xw**2 + yw**2)
    
    # DISPLAY
    
    print(f'omegaz = {np.rad2deg(omegaz)}')
    print(f'xw = {xw}, yw = {yw}')
    print("-----------------------------")
    
    pass

# Enter here exit cleanup code.
