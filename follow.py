"""
VERSION CONTROL
VERSION gh01


See Changelog file in this directory.

"""

# import the necessary packages
from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil # Needed for command message definitions
import time
import numpy as np
from PiVideoStream import PiVideoStream
from imutils.video import FPS
from picamera.array import PiRGBArray
from picamera import PiCamera
import argparse
import imutils
import cv2
import sys
from datetime import datetime, timedelta
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib import style
from pid_controller.pid import PID

#global current_pan_pwm
#current_pan_pwm = 1427

global bearingAB
bearingAB = 0.0

global requested_height
requested_height = -3.0

global current_height
current_height = 0.0

global velocity
velocity = 0.0

#global framecount
#framecount = 0


#--------------------------SEND VELOCITY VECTORS IN NED FRAME--------------------
# vx is north. vy is east. vz is down.  This is unrelated to the yaw of the UAV.
def send_ned_velocity(velocity_x, velocity_y, velocity_z):
    """
    Move vehicle in direction based on specified velocity vectors.
    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
        0b0000111111000111, # type_mask (only speeds enabled)
        0, 0, 0, # x, y, z positions (not used)
        velocity_x, velocity_y, velocity_z, # x, y, z velocity in m/s
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)

    vehicle.send_mavlink(msg)



#--------------------------SEND YAW IN LOCAL NED FRAME--------------------
# This sets the absolute yaw, in degrees, measured clockwise from North.
# Heading in range 0-360 degrees.  No negs!
def condition_yaw(heading, clockwise, relative=False): # In degrees
    if relative:
        is_relative=1 #yaw relative to direction of travel
    else:
        is_relative=0 #yaw is an absolute angle
    # create the CONDITION_YAW command using command_long_encode()
    msg = vehicle.message_factory.command_long_encode(
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
        0, #confirmation
        heading,    # param 1, yaw in degrees
        0,          # param 2, yaw speed deg/s
        clockwise,          # param 3, direction -1 ccw, 1 cw
        is_relative, # param 4, relative offset 1, absolute angle 0
        0, 0, 0)    # param 5 ~ 7 not used
    # send command to vehicle
    vehicle.send_mavlink(msg)


#--------------------------VERTICAL ANGLE FROM VERTICAL IMAGE OFFSET--------------------
# Returns the angle (radians) in the vertical plane between a target and the centreline in the
# camera image. 

def vert_image_angle (y):
    #print y
    vertical_angle_of_view = 48.8 # degrees
    vertical_resolution = 240 # pixels
    pix_per_degree = vertical_resolution / vertical_angle_of_view

    vertical_angle = (y / pix_per_degree)
    # print vertical_angle
    vertical_angle = np.radians(vertical_angle)
    

    return (vertical_angle)

#--------------------------HORIZONTAL ANGLE FROM HORIZONTAL IMAGE OFFSET----------------
# Returns the angle (radians) in the horizontal plane between a target and the centreline in the
# camera image. x is the x coordinate of the target, origin top left of frame.

def horiz_image_angle (x):
    horiz_angle_of_view = 62.2 # degrees
    horiz_resolution = 320 # pixels
    pix_per_degree = horiz_resolution / horiz_angle_of_view
    
    horiz_angle = np.radians(x / pix_per_degree)

    return (horiz_angle)

"""
#--------------------------GET TRANSMITTER-SET HEIGHT ---------------------------------
# Returns the requested height in metres controlled by the left pot of the transmitter.
# It is CRITICAL that the range is not set to allow flight height below 1 metre.

def get_requested_height():
    minpwm = 1000
    maxpwm = 1900
    midpwm = (minpwm+maxpwm)/2
    height_range = 4.0 # metres
    pwm_per_metre = (maxpwm-minpwm) / height_range
    calibrated_height = -3 # Upwards is negative!

    current_pwm = vehicle.channels['7'] # left pot
    delta_pwm = midpwm - current_pwm # Neg delta is down.
    delta_height = delta_pwm / pwm_per_metre
    # Current settings - full ccw -1m, full cw -5m
    requested_height = calibrated_height + delta_height

    return (requested_height)
"""
#--------------------------GET TRANSMITTER-SET HEIGHT ---------------------------------
# Returns the requested height in metres controlled by the 3 way switch of the transmitter.
# It is CRITICAL that the range is not set to allow flight height below 1 metre.

def get_requested_height():

    global requested_height

    current_pwm = vehicle.channels['8'] # 3 pos switch. Note switch orientation.
    print current_pwm

    #print current_pwm
    if current_pwm < 1200: # physical switch up
        requested_height = requested_height - 0.002  # Recall neg is upwards.
    elif current_pwm > 1700:
        requested_height = requested_height + 0.002


    if requested_height < -5.0:
        requested_height = -5.0
    elif requested_height > -1.0:
        requested_height = -1.0

    return (requested_height)


#--------------------------GET BEARING -------------------------------------
# Returns the yaw (radians) as measured from the centreline.  Positive is clockwise.

def bearing(loc1, loc2):

    na = loc1[0]
    ea = loc1[1]
    nb = loc2[0]
    eb = loc2[1]

    #print na, nb, ea, eb
    deltan = nb - na
    deltae = eb - ea

    #alpha12 = np.arctan(deltae/deltan)
    alpha12 = np.arctan2(deltae,deltan)
    #print 'Alpha12', alpha12

    #if alpha12 < 0:
        #alpha12 = alpha12 + (2*np.pi)

    return (alpha12)

#--------------------------SET UP CONNECTION TO VEHICLE----------------------------------

# Parse the arguments  
parser = argparse.ArgumentParser(description='Commands vehicle using vehicle.simple_goto.')
parser.add_argument('--connect', 
                   help="Vehicle connection target string. If not specified, SITL automatically started and used.")
args = parser.parse_args()

connection_string = args.connect

# Connect to the physical UAV or to the simulator on the network
if not connection_string:
    print ('Connecting to pixhawk.')
    vehicle = connect('/dev/serial0', baud=57600, wait_ready= True)
else:
    print ('Connecting to vehicle on: %s' % connection_string)
    vehicle = connect(connection_string, wait_ready=True)



#--------------------------SET UP VIDEO THREAD ----------------------------------

# created a *threaded *video stream, allow the camera sensor to warmup,
# and start the FPS counter
print('[INFO] sampling THREADED frames from `picamera` module...')
vs = PiVideoStream().start()
time.sleep(2.0)


#-------------- FUNCTION DEFINITION TO ARM AND TAKE OFF TO GIVEN ALTITUDE ---------------
def arm_and_takeoff(aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude.
    """

    print ('Basic pre-arm checks')
    # Don't try to arm until autopilot is ready
    while not vehicle.is_armable:
        print ('Waiting for vehicle to initialise...')
        time.sleep(1)

        
    print ('Arming motors')
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True    

    # Confirm vehicle armed before attempting to take off
    while not vehicle.armed:      
        print ('Waiting for arming...')
        time.sleep(1)

    print ('Taking off!')
    vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude

    while True:
        # print "Global Location (relative altitude): %s" % vehicle.location.global_relative_frame
        if vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95: 
            break
        time.sleep(1)


#-------------- FUNCTION DEFINITION TO FLY IN VEHICLE STATE TRACKING  ---------------------
def tracking (vstate):

    global current_pan_pwm
    global bearingAB
    global current_height
    global requested_height
    global velocity
    
    print vstate

 
    #The vehicle process images and maintains all data ready to fly autonomously.
    #However, it does not send velocity vectors unless GUIDED flight mode is selected.

    red1Good = red2Good = False # Set True when returned target offset is reliable.
    target = None # Initialise tuple returned from video stream
    camera_pitch = 0

    # Initialise the FPS counter.
    #fps = FPS().start()

    while vstate == "tracking":

        # Get height above ground using rangefinder.  This is reported in metres beween 0.2 and 7m.
        # The actual height may be more than this, although guided mode should only be selected when
        # the existing height is between 1 and 3 m.

        #current_height = vehicle.rangefinder.distance * -1.0
        current_height = vehicle.location.global_relative_frame.alt  * -1 # Use this line for SITL
        #current_height = -3.0
        #time.sleep(5)

        # grab the frame from the threaded video stream and return coords A and B wrt centrelines
        # Pass in data to be overlaid onto display.
        target = vs.read(vstate,current_height,bearingAB,camera_pitch,velocity)
        #framecount = framecount + 1
        coordA = target[0] # The nearest lock
        coordA_Good = target[1]
        coordB = target[2] # The farthest lock
        coordB_Good = target[3]
        #print 'Coords', coordA, coordB
        #print 'Status', coordA_Good, ' ',coordB_Good
        #time.sleep(0.2)
        

        #Set requested Height Over Ground
        #In metres, negative upwards.
        requestedHOG = -10.0
        #requestedHOG = get_requested_height() # This increments/decrements the global variable depending on TX switch.
        #print 'Current height: ',current_height, ' Requested height: ', requestedHOG
        
        # Calculate required z velocity to achieved requestedHOG
        #Remember +z is DOWN!
        #Use PID controller to obtain altitude adjustment.  
        zError = requestedHOG - current_height
        vz = altpid(zError)
        #print vz
            
        #print 'UAV Attitude', vehicle.attitude
        #print 'Gimbal', vehicle.gimbal

        # Calculate overall pitch and pan (yaw) as inputs to calculation of locations.        
        #pitch_from_gimbal =   #np.radians(-45) #)  # radians from horizontal, down is negative
        camera_pitch = np.radians(vehicle.gimbal.pitch) #pitch_from_gimbal #+ vehicle.attitude.pitch
        #print 'V pitch: ', vehicle.attitude.pitch, ' Cam pitch', camera_pitch, 'Gimbal pitch', pitch_from_gimbal

        pan_from_gimbal = np.radians(vehicle.gimbal.yaw) # Pan in radians wrt vehicle
        pan_from_yaw = vehicle.attitude.yaw
        #print 'Yaw', pan_from_yaw

        if coordA_Good == True:

            pitch_from_image = vert_image_angle(coordA[1])
            total_pitch = (camera_pitch + pitch_from_image) # Down is negative, in radians            
            #print 'total pitch A', total_pitch

            pan_from_image = horiz_image_angle(coordA[0])
            total_pan = pan_from_yaw + pan_from_gimbal + pan_from_image
            #print 'total pan A', total_pan
            #print 'Ay', pitch_from_image, 'Ax', pan_from_image
            
            radius = np.abs(current_height / np.tan(total_pitch))
            #print 'Radius A', radius
            coordA_north = radius * np.cos(total_pan)
            coordA_east = radius * np.sin(total_pan)
            locA = (coordA_north, coordA_east) # Relative to current position NED
            #ax1.plot(coordA_north, coordA_east,'bo')
            #plt.draw()
            #plt.show(block=False)
        else:
            locA = (0,0)
            

        if coordB_Good == True:

            pitch_from_image = vert_image_angle(coordB[1])
            total_pitch = (camera_pitch + pitch_from_image) # Down is negative, in radians
            #print 'vimage', pitch_from_image, '  total pitch B', total_pitch

            pan_from_image = horiz_image_angle(coordB[0])
            total_pan = pan_from_yaw + pan_from_gimbal + pan_from_image
            #print 'total pan B', total_pan
            #print 'By', pitch_from_image, 'Bx', pan_from_image
            
            radius = np.abs(current_height / np.tan(total_pitch))
            #print 'Radius B', radius
            coordB_north = radius * np.cos(total_pan)
            coordB_east = radius * np.sin(total_pan)
            locB = (coordB_north, coordB_east) # Relative to current position NED
        else:
            locB = (0,0)

        # Set coordinates to follow depending on whether we have a lock on one point or two.
        # If we only have one lock, follow that.
        if coordA_Good == True and coordB_Good == False:
            locB = locA
            locA = (0,0)
            coordB_Good == True

        # If B is good and A is bad, locA and locB will already be set up correctly.         
        
        if coordA_Good == True and coordB_Good == True:

            #vmax = 1.75 # The sideslip allows more than this maximum.
            fwd = 2 # The larger this value, the smaller the sideslip.
            

            # Calculate the bearing of the line.  Returned as a +ve in radians.
            bearingAB = (bearing(locA,locB))
            #print 'Locations', locA, locB, bearingAB

            # Calculate the bearing from current position to closest point on line at LocA
            bearingOA = (bearing((0,0), locA)) # np.arctan2(locA[1], locA[0])
            #print np.degrees(bearingAB), np.degrees(bearingOA)

            # Mix the two components to move along the line but also manoevre over it. 
            vnp = (np.cos(bearingAB) + (np.cos(bearingOA)/fwd))
            vep = (np.sin(bearingAB)  + (np.sin(bearingOA)/fwd))

            """
            # Calculate velocity based on bearing alignment and how far can see ahead.            
            # A simple treatment of stopping distances is made.
            # The vehicle must be able to 'see' at least x seconds ahead at the current velocity.
            # The maximum velocity is set (set at vmax) to ensure this is always the case.

            stop_time = 2.0 # So reduce this value to increase max velocity.
            
            visible_distance = np.sqrt( (locB[0]*locB[0]) + (locB[1]*locB[1]) )
            vmax = (visible_distance / stop_time) 
            if vmax > 3.0: # failsafe
                vmax = 3.0

            #uav_bearing = np.arctan2(vep, vnp)
            #delta_bearing = np.abs(bearingAB-uav_bearing)
            #bearing_factor = ((np.pi/2) - delta_bearing) / (np.pi/2)
            #bearing_factor = 1

            # The bearing factor reduces the speed further depending on the misalignment of the vehicle with the line.
            delta_bearing = np.abs(vehicle.attitude.yaw - bearingAB)
            bearing_factor = 1 / (1 + delta_bearing)
            #bearing_factor = 1
            """

            v = 0.8 #Fix velocity in m/s
            #v = (vmax*bearing_factor)/(1+(1/fwd))
            
            vn = vnp * v
            ve = vep * v
            velocity = np.sqrt((vn*vn)+(ve*ve))
            #print 'Delta bearing: ', delta_bearing, ' Visible distance: ', visible_distance,' vmax: ', vmax, bearing_factor, velocity

            # We wish to maintain locB (top coordinate) in the centre of the image by panning the camera.
            # The amount to pan is given by the x coordinate of LocB
            #Use PID controller to make gimbal pan adjustment. Currently only P is used.
              
            #panError = vehicle.gimbal.yaw+ np.degrees(pan_from_image)
            #panAdjust = vehicle.gimbal.yaw + (0.8*(np.degrees(pan_from_image)))
            #print 'pan from image', np.degrees(pan_from_image), '  adjust ', panAdjust    
            #vehicle.gimbal.rotate(-45,0,panAdjust)
            #time.sleep(1)
            
            #ms = time.time()*1000.0
            #f.write('\n' + str(ms) + ',' + str(height))

            #if framecount%10 == 0:
                # Append location data to logfile
                #locNth = vehicle.location.local_frame.north
                #locEst = vehicle.location.local_frame.east
                #locdata = str(locA)
                #locdata = locdata.strip('(')
                #locdata = locdata.strip(')')
                #f.write('\n' + 'A,'+str(coordA_north)+ ',' + str(coordA_east)+','+str(locNth)+','+str(locEst)+ ',' + str(bearingAB)+','+str(height)+','+str(total_pitch)+','+str(pitch_from_gimbal)+','+str(pitch_from_image)+','+str(total_pan)+','+str(pan_from_yaw)+','+str(pan_from_image))
                #locdata = str(locB)
                #locdata = locdata.strip('(')
                #locdata = locdata.strip(')')
                #f.write('\n' + 'B,'+str(coordB_north)+ ',' + str(coordB_east)+','+str(locNth)+','+str(locEst)+ ',' + str(bearingAB)+','+str(height)+','+str(total_pitch)+','+str(pitch_from_gimbal)+','+str(pitch_from_image)+','+str(total_pan)+','+str(pan_from_yaw)+','+str(pan_from_image))
            
            # Check if operator has transferred to autopilot using TX switch.
            if vehicle.mode == "GUIDED":
                send_ned_velocity (vn,ve,vz) # MODIFIED FOR MAAXX34. NO X,Y MOVEMENT. NO YAW.
                
                if (np.degrees(bearingAB-vehicle.attitude.yaw)) > 2.0:
                    if bearingAB < 0:
                        bearingAB = bearingAB + (2*np.pi)
                    condition_yaw(np.degrees(bearingAB), 1) # clockwise
                    
                elif (np.degrees(bearingAB-vehicle.attitude.yaw)) < -2.0:
                    if bearingAB < 0:
                        bearingAB = bearingAB + (2*np.pi)
                    condition_yaw(np.degrees(bearingAB), -1) # anticlockwise
                
        else:
            vstate = "lost"
            break

        # update the FPS counter
        #fps.update()
        # time.sleep(1)

    #f.close()
    # stop the timer and display FPS information
    #fps.stop()
    #print('Elasped time in tracking state: {:.2f}'.format(fps.elapsed()))
    #print('Approx. FPS: {:.2f}'.format(fps.fps()))


    return vstate


#-------------- FUNCTION DEFINITION TO FLY IN VEHICLE STATE LOST  ---------------------
def lost (vstate):
    
    global current_pan_pwm
    global bearingAB
    global current_height
    global requested_height
    global camera_pitch
    global velocity
    
    print vstate
    vstate = "lost"

    
    #The vehicle process images and returns to tracking if any lock is found.
    # Meanwhile it rotates clockwise and maintains height.

    # Return camera to centre
    vehicle.gimbal.rotate(-45,0,0)

    red1Good = red2Good = False # Set True when returned target offset is reliable.
    target = None # Initialise tuple returned from video stream

    # Initialise the FPS counter.
    #fps = FPS().start()

    while vstate == "lost":

        # grab the frame from the threaded video stream and return coords A and B wrt centrelines
        target = vs.read(vstate,current_height,bearingAB,camera_pitch,velocity)
        coordA = target[0]
        coordA_Good = target[1]
        coordB = target[2]
        coordB_Good = target[3]

        if coordA_Good == True or coordB_Good == True:
            vstate = "tracking"
            break

        else:        

            # Check if operator has transferred to autopilot using TX switch.
            if vehicle.mode == "GUIDED":
                send_ned_velocity (0,0,0)

                condition_yaw(2, 1, True)
                # print np.degrees(vehicle.attitude.yaw)
                time.sleep(0.2)      
                
        # update the FPS counter
        #fps.update()


    # stop the timer and display FPS information
    #fps.stop()
    #print('Elasped time in tracking state: {:.2f}'.format(fps.elapsed()))
    #print('Approx. FPS: {:.2f}'.format(fps.fps()))

    return vstate




# MAIN PROGRAM

# Initialise PID instance for altitude control
altpid = PID(p=1.0, i=0.004, d=3.0)

# Initialise PID instance for gimbal pan control
panpid = PID(p=0.4, i=0.01, d=0.01)
print vehicle.gimbal
print 'Rotating gimbal'
vehicle.gimbal.rotate(-45,0,0)

time.sleep(3)

vstate = "tracking" # Set the vehicle state to tracking in the finite state machine.

# Set up graph to plot location data
#style.use('fivethirtyeight')
#fig = plt.figure()
#ax1 = fig.add_subplot(1,1,1)
#ani = animation.FuncAnimation(fig, animate, interval=1000)
#plt.show(block=False)



# If on simulator, arm and take off.
if connection_string:

    print ('Basic pre-arm checks')
    # Don't try to arm until autopilot is ready
    while not vehicle.is_armable:
        print ('Waiting for vehicle to initialise...')
        time.sleep(1)

    print ('Arming motors')
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True    


    # Confirm vehicle armed before attempting to take off
    while not vehicle.armed:      
        print ('Waiting for arming...')
        time.sleep(1)

    # Get airborne and hover
    arm_and_takeoff(10)
    print "Reached target altitude - currently in Guided mode on altitude hold"
    
# This needs to be commented out prior to real flight!
# vehicle.mode = VehicleMode("GUIDED")


while True :

    # MODIFIED FOR MAAXX34. Force tracking mode to test height control.
    vstate = "tracking"

    if vstate == "tracking":
        # Enter tracking state
        vstate = tracking(vstate)
        #plt.show()

    else:
        # Enter lost state
        vstate = lost(vstate)


    
"""

#---------------------------- RETURN TO HOME AND CLEAN UP ----------------------------


# Initiate return to home
print "Returning to Launch"
vehicle.mode = VehicleMode("RTL")
print "Pause for 10s before closing vehicle"
time.sleep(10)

"""

#Close vehicle object before exiting script
print "Close vehicle object"
vehicle.close()

# do a bit of cleanup
cv2.destroyAllWindows()
vs.stop()
