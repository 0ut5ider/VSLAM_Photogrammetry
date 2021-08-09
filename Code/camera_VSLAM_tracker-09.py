#!/usr/bin/python3

#####################################################
##          T265 VSLAM and camera trigger          ##
#####################################################
# 
#
#
# References::  https://raw.githubusercontent.com/thien94/vision_to_mavros/master/scripts/t265_to_mavlink.py
#               https://github.com/WuSiYu/python-i2clcd
#
#               Coordinate System Reference
#               https://github.com/IntelRealSense/realsense-ros/issues/912
#

import csv
import math
import numpy as np
import sys
import datetime
import os.path
import array as arr
import transformations as tf
import RPi.GPIO as GPIO # Import Raspberry Pi GPIO library
import time
from time import sleep
import threading
from pprint import pprint
import numpy as np
import signal
import pyrealsense2.pyrealsense2 as rs
import i2clcd
import gphoto2 as gp

#######################################
# Parameters
#######################################

# Transformation to convert different camera orientations to NED convention. Replace camera_orientation_default for your configuration.
#   0: Forward, USB port to the right
#   1: Downfacing, USB port to the right 
#   2: Forward, 45 degree tilted down
# Important note for downfacing camera: you need to tilt the vehicle's nose up a little - not flat - before you run the script, otherwise the initial yaw will be randomized, read here for more details: https://github.com/IntelRealSense/librealsense/issues/4080. Tilt the vehicle to any other sides and the yaw might not be as stable.
camera_orientation = 0

# lock for thread synchronization
lock = threading.Lock()

# default exit code is failure - a graceful termination with a
# terminate signal is possible.
exit_code = 1

# button setup
pin_number_button_1 = 16
pin_number_camera_shutter = 18

GPIO.setwarnings(False) # Ignore warning for now
GPIO.setmode(GPIO.BOARD) # Use physical pin numbering

GPIO.setup(pin_number_button_1, GPIO.IN, pull_up_down=GPIO.PUD_UP) # Set pin to be an input pin and set initial value to be pulled low (off)
GPIO.setup(pin_number_camera_shutter, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# Initialize the LCD I2CLCD
lcd = i2clcd.i2clcd(i2c_bus=1, i2c_addr=0x27, lcd_width=20)
lcd.init()

line1 = ''
line2 = ''
line3 = ''
line4 = ''

image_capture_start_time = 0

#  Should be at least 6 seconds for Sony A7R2, since it takes about 5.5 seconds to take and save image. 
#  This delay should be longer then that.
intervalometer_delay = 10

#  Create the flight log file
image_root_path = '/home/pi/Project01/images'
csv_output_file_name = 'data.csv'
csv_target = os.path.join(image_root_path, csv_output_file_name)

file = open(csv_target, 'a', newline='', encoding='UTF-8')
csv_writer = csv.writer(file)
if not os.path.isfile(csv_target):
    csv_writer.writerow(['Image Name', 'X', 'Y', 'Z', 'Roll', 'Pitch', 'Yaw', 'Date/Time'])

# Offset in NED coordinates; Distance from DSLR imagins sensor to T265
body_offset_enabled = 1
body_offset_x = 0.066       # In meters (m)
body_offset_y = 0           # In meters (m)
body_offset_z = 0.050      # In meters (m)

#######################################
# Global variables
#######################################

# T265 Camera-related variables
pipe = None
pose_sensor = None

# Data variables
data = None
current_confidence_level = None
H_aeroRef_aeroBody = None
V_aeroRef_aeroBody = None
heading_north_yaw = None

Raw_RPY = None
NED_RPY = None
Raw_xyz = None
NED_xyz = None


# Global scale factor, position x y z will be scaled up/down by this factor
scale_factor = 1.0

# Enable using yaw from compass to align north (zero degree is facing north)
compass_enabled = 0

# Main execution variable
main_loop_should_quit = False
thread_should_quit = False

# Initialize the start position
x_start_position = 0
y_start_position = 0
z_start_position = 0

lcd_update_interval = 0.500

# Button handling
button_menu_select_state = 'Released'
button_menu_select_image_capture_start_time = 0
button_menu_select_on_time = 0
short_press = False
long_press = False

first_time = True
intevelometer_on = False

file_number = 0

debug_enable = 0


#######################################
# Functions - System
#######################################

# gracefully terminate the script if an interrupt signal (e.g. ctrl-c)
# is received.  This is considered to be abnormal termination.

def sigint_handler(sig, frame):
    global main_loop_should_quit
    main_loop_should_quit = True
signal.signal(signal.SIGINT, sigint_handler)

# gracefully terminate the script if a terminate signal is received
# (e.g. kill -TERM).  
def sigterm_handler(sig, frame):
    global main_loop_should_quit
    main_loop_should_quit = True
    global exit_code
    exit_code = 0
    
# Replacement of the standard print() function to flush the output
def progress(string):
    print(string, file=sys.stdout)
    sys.stdout.flush()    

#######################################
# Functions - T265
#######################################

def realsense_connect():
    global pipe, pose_sensor, cfg
    
    # Declare RealSense pipeline, encapsulating the actual device and sensors
    pipe = rs.pipeline()

    # Build config object before requesting data
    cfg = rs.config()

    # Enable the stream we are interested in
    cfg.enable_stream(rs.stream.pose) # Positional data
    cfg.disable_stream(rs.stream.fisheye, 1) # Left camera
    cfg.disable_stream(rs.stream.fisheye, 2) # Right camera

    # Configure callback for relocalization event
    device = cfg.resolve(pipe).get_device()
    pose_sensor = device.first_pose_sensor()
#    pose_sensor.set_notifications_callback(realsense_notification_callback)

    # Start streaming with requested config
    pipe.start(cfg)
    print("Connected to T265")

#######################################
# Functions - Button Handling
#######################################
        
def button_select_callback(channel):
    global button_menu_select_state
    global button_menu_select_image_capture_start_time
    global button_menu_select_on_time
    global main_loop_should_quit
    global long_press
    global short_press
    flag = True
    
    if button_menu_select_state == 'Released' and flag == True:
#        print("Button 1 was push")
        button_menu_select_image_capture_start_time = time.time()
        button_menu_select_state = 'Pushed'
        flag = False
        
    if button_menu_select_state == 'Pushed' and flag == True:
        button_menu_select_on_time = time.time() - button_menu_select_image_capture_start_time
#        print('Button 1 was released and on for ', button_menu_select_on_time)
        button_menu_select_state = 'Released'


    if button_menu_select_on_time > 0 and button_menu_select_on_time <= 1:
        short_press = True     # Button short press
        button_menu_select_on_time = 0

    if button_menu_select_on_time > 1.5 and button_menu_select_on_time <= 5:
        long_press = True     # Button Long press
        button_menu_select_on_time = 0

    if button_menu_select_on_time > 9:
        print ("Goodbye")
#        lcd.clear()
#        lcd.print_line ("Program Terminated", line=0)
        main_loop_should_quit = True # Exit program

def button_handling():
    GPIO.add_event_detect(pin_number_camera_shutter, GPIO.FALLING, callback = intervalometer_call, bouncetime = 500) # Setup event on rising edge
    GPIO.add_event_detect(pin_number_button_1, GPIO.FALLING, callback = button_select_callback, bouncetime = 500) # Setup event on rising edge

#######################################
# Functions - Update Displays and files
#######################################

def update_position_data():
    x_position = NED_xyz[0] - x_start_position
    y_position = NED_xyz[1] - y_start_position
    z_position = NED_xyz[2] - z_start_position
   
    create_lcd_message()
    update_console()
    
def create_lcd_message():
    global line1
    global line2
    global line3
    global line4
    
    if intevelometer_on == True:
        time_remaining = image_capture_start_time + intervalometer_delay - time.time()
        line1 = "FN " + f"{file_number:3.0f}" + " :" + f"{time_remaining:2.0f}"
        #print ("Timer " + f"{time_remaining:2.0f}")
        #print (" Timer :" + str(time_remaining))
    if intevelometer_on == False:
        line1 = "INT OFF   "       
    if button_menu_select_state != 'Released':
        on_time = time.time() - button_menu_select_image_capture_start_time
        line1 = line1 + f"Timer  {on_time:1.0f}"
    if button_menu_select_state == 'Released':
        line1 = line1 + f" CL: {data.tracker_confidence:.0f}"

    line2 = f"FWD:{NED_xyz[0]:3.3f}m R:{NED_RPY[0]:0.1f}"
    line3 = f"LEF:{NED_xyz[1]:3.3f}m P:{NED_RPY[1]:0.1f}"
    line4 = f"DWN:{NED_xyz[2]:3.3f}m Y:{NED_RPY[2]:0.1f}" 
 
def update_console():
    print('Frame number: ', pose.frame_number, f' Position X:{NED_xyz[0]:.3f} Y:{NED_xyz[1]:.3f} Z: {NED_xyz[2]:.3f} :: Roll: {NED_RPY[0]:.1f} Pitch:{NED_RPY[1]:.1f}  Yaw:{NED_RPY[2]:.1f}  ', end = "\r")

def update_lcd():
    while not main_loop_should_quit:
        lcd.print_line(line1, line=0)   
        lcd.print_line(line2, line=1)
        lcd.print_line(line3, line=2)
        lcd.print_line(line4, line=3)
        sleep (0.1)

def update_csv(file_name):
    x_position = NED_xyz[0] - x_start_position
    y_position = NED_xyz[1] - y_start_position
    z_position = NED_xyz[2] - z_start_position
    
    dt = datetime.datetime.now()
    saved_date_time = (dt.strftime('%Y-%m-%d %H:%M:%S'))
    
    file = open(csv_target, 'a', newline='', encoding='UTF-8')
    csv_writer = csv.writer(file)
    csv_writer.writerow([file_name, x_position, y_position, z_position, NED_RPY[0], NED_RPY[1], NED_RPY[2], saved_date_time])
    file.close()

#######################################
# Functions - Intervalometer
#######################################

def intervalometer_run():
    global file_number
    global image_capture_start_time
    global intevelometer_on
    
    while not main_loop_should_quit: 
        if intevelometer_on == True:
            try: 
                camera = gp.Camera()
                camera.init()
                
                image_capture_start_time = time.time()
                
                file_number += 1
                base_name = 'argus-'
                extension = '.jpg'
                new_file_name = base_name + "%05d" % file_number + extension 
                
                time_1 = (time.time() - image_capture_start_time)
                #print('2.  Capturing image - Time: ', (time.time() - image_capture_start_time), "             ")

                file_path = camera.capture(gp.GP_CAPTURE_IMAGE) 
                
                time_2 = (time.time() - image_capture_start_time)
                #print('3.  Image capture finished - Time: ', (time.time() - image_capture_start_time), "             ")
                
                target = os.path.join(image_root_path, new_file_name)
                # print('Copying image to', target)
                camera_file = camera.file_get(
                   file_path.folder, file_path.name, gp.GP_FILE_TYPE_NORMAL)
                camera_file.save(target)           
                    
                update_csv(new_file_name)            
                
                camera.exit()
                time_3 = (time.time() - image_capture_start_time)
                print(f'Image copy finished - ImageCapture: {time_2:0.2f}s, ImageSave: {(time_3-time_2):0.2f}s, TotalTime: {time_3:0.2f}s :: Images captured: {file_number:0.0f}')

                sleep_delay = intervalometer_delay - (time.time() - image_capture_start_time)

                if sleep_delay > 0.1:
                    sleep(sleep_delay)
            except:
                camera.exit()
                intevelometer_on = False

def intervalometer_call(channel):
    global intevelometer_on
    global first_time
    
    print("Trigger button Pressed")
    if first_time == True:
        intevelometer_on = True
        first_time = False
        print("Intervalometer ON")

    elif first_time == False:  
        intevelometer_on = False
        first_time = True
        print("Intervalometer OFF")

#######################################
# Main code loop
#######################################
signal.signal(signal.SIGTERM, sigterm_handler)

# Initial LCD display message
lcd.print_line ("Loaded the Code.....", line=0)
lcd.print_line ("                    ", line=1)
lcd.print_line ("Connect the T265", line=2)
lcd.print_line ("Press button to cont", line=3)

while GPIO.input(pin_number_button_1):   # Wait for button to be pushed before going farther
    sleep(0.1)

try:
    print("INFO: pyrealsense2 version: %s" % str(rs.__version__))
except Exception:
    # fail silently
    pass

# Connect to T265 camera
realsense_connect()

update_lcd_thread = threading.Thread(target = update_lcd, daemon = True)
update_lcd_thread.start()

intervalometer_thread = threading.Thread(target = intervalometer_run, daemon = True)
intervalometer_thread.start()

button_handling()  # Start the button interrupts

if compass_enabled == 1:
    time.sleep(1) # Wait a short while for yaw to be correctly initiated
    
if camera_orientation == 0:     # Forward, USB port to the right
    H_aeroRef_T265Ref   = np.array([[0,0,-1,0],[1,0,0,0],[0,-1,0,0],[0,0,0,1]])
    H_T265body_aeroBody = np.linalg.inv(H_aeroRef_T265Ref)
elif camera_orientation == 1:   # Downfacing, USB port to the right
    H_aeroRef_T265Ref   = np.array([[0,0,-1,0],[1,0,0,0],[0,-1,0,0],[0,0,0,1]])
    H_T265body_aeroBody = np.array([[0,1,0,0],[1,0,0,0],[0,0,-1,0],[0,0,0,1]])
elif camera_orientation == 2:   # 45degree forward
    H_aeroRef_T265Ref   = np.array([[0,0,-1,0],[1,0,0,0],[0,-1,0,0],[0,0,0,1]])
    H_T265body_aeroBody = (tf.euler_matrix(m.pi/4, 0, 0)).dot(np.linalg.inv(H_aeroRef_T265Ref))
else:                           # Default is facing forward, USB port to the right
    H_aeroRef_T265Ref   = np.array([[0,0,-1,0],[1,0,0,0],[0,-1,0,0],[0,0,0,1]])
    H_T265body_aeroBody = np.linalg.inv(H_aeroRef_T265Ref)

try:
    while not main_loop_should_quit:
        try:
            # Wait for the next set of frames from the camera
            frames = pipe.wait_for_frames()
        except:
            pipe.stop()
            pipe.start(cfg)
        # Fetch pose frame
        pose = frames.get_pose_frame()    

        # Process data
        if pose:
            with lock:
                # Pose data consists of translation and rotation
                data = pose.get_pose_data()  
                
                # In transformations, Quaternions w+ix+jy+kz are represented as [w, x, y, z]!
                H_T265Ref_T265body = tf.quaternion_matrix([data.rotation.w, data.rotation.x, data.rotation.y, data.rotation.z]) 
                H_T265Ref_T265body[0][3] = data.translation.x * scale_factor
                H_T265Ref_T265body[1][3] = data.translation.y * scale_factor
                H_T265Ref_T265body[2][3] = data.translation.z * scale_factor
                
                # Transform to aeronautic coordinates NED (body AND reference frame!)
                # +X FWD  (from T265 -z)
                # +Y LEFT (from T265 x)
                # +Z DOWN (from T265 -y)
                # Roll around world X axis (FWD)
                # Pitch around world y axis (Left)
                # Yaw arouns world z axis (down)
                H_aeroRef_aeroBody = H_aeroRef_T265Ref.dot( H_T265Ref_T265body.dot( H_T265body_aeroBody))
                        
                # Take offsets from body's center of gravity (or IMU) to camera's origin into account
                if body_offset_enabled == 1:
                    H_body_camera = tf.euler_matrix(0, 0, 0, 'sxyz')
                    H_body_camera[0][3] = body_offset_x
                    H_body_camera[1][3] = body_offset_y
                    H_body_camera[2][3] = body_offset_z
                    H_camera_body = np.linalg.inv(H_body_camera)
                    H_aeroRef_aeroBody = H_body_camera.dot(H_aeroRef_aeroBody.dot(H_camera_body))

                # Realign heading to face north using initial compass data
                if compass_enabled == 1:
                    H_aeroRef_aeroBody = H_aeroRef_aeroBody.dot( tf.euler_matrix(0, 0, heading_north_yaw, 'sxyz'))

                NED_xyz = np.array( tf.translation_from_matrix( H_aeroRef_aeroBody))
                NED_RPY = np.array( tf.euler_from_matrix( H_aeroRef_aeroBody, 'sxyz')) * 180 / math.pi      # in Degrees

                # Confidence level value from T265  
                current_confidence_level = float(data.tracker_confidence)         
               
                update_position_data()    
     
                if long_press == True:
                    long_press = False
                    pipe.stop()
                    pipe.start(cfg)
                    #x_start_position = NED_xyz[0]
                    #y_start_position = NED_xyz[1]
                    #z_start_position = NED_xyz[2]       
                
                # Show debug messages here
                if debug_enable == 1:
                    os.system('clear') # This helps in displaying the messages to be more readable
                    # Raw_RPY = np.array( tf.euler_from_matrix( H_T265Ref_T265body, 'sxyz')) * 180 / math.pi
                    # progress("DEBUG: Raw RPY[deg] roll: {:2.1f}  pitch: {:2.1f}  yaw: {:2.1f}".format(Raw_RPY[0], Raw_RPY[1], Raw_RPY[2] ))
                    progress("DEBUG: NED RPY[deg] roll: {:2.1f}  pitch: {:2.1f}  yaw: {:2.1f}".format(NED_RPY[0], NED_RPY[1], NED_RPY[2]))
                    # Raw_xyz = np.array( [data.translation.x, data.translation.y, data.translation.z])
                    # progress("DEBUG: Raw pos x: {:2.3f}  y: {:2.3f}  z: {:2.3f}".format(Raw_xyz[0], Raw_xyz[1], Raw_xyz[2] ))
                    progress("DEBUG: NED pos x: {:2.3f}  y: {:2.3f}  z: {:2.3f}".format(NED_xyz[0], NED_xyz[1], NED_xyz[2] ))                
        
except Exception as e:
    print(e)

except:
    send_msg_to_gcs('ERROR IN SCRIPT')  
    print("Unexpected error: %s" % sys.exc_info()[0])

finally:
    print('Closing the script...')
    lcd.print_line('', line=0)   
    lcd.print_line('    Goodbye    ', line=1)
    lcd.print_line('', line=2)
    lcd.print_line('', line=3)
    
    # Stop the threads
    update_lcd_thread.join()  
    intervalometer_thread.join()
    
    # start a timer in case stopping everything nicely doesn't work.
    signal.setitimer(signal.ITIMER_REAL, 5)  # seconds...
    file.close()
    pipe.stop()
    camera.exit()
    print("INFO: Realsense pipeline and vehicle object closed.")
    sys.exit(exit_code)