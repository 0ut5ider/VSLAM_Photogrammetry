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
import os
from pathlib import Path
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
import gphoto2 as gp

from luma.core.interface.serial import i2c
from luma.core.interface.parallel import bitbang_6800
from luma.core.render import canvas
from luma.oled.device import sh1106
from PIL import ImageFont

#######################################
# Parameters
#######################################
software_version = 15

# OLED initialization and configuration
serial = i2c(port=1, address=0x3C)
device = sh1106(serial)

# font_path = str('CCRedAlert.ttf')
font_path = str(Path(__file__).resolve().parent.joinpath('fonts', 'C&C Red Alert [INET].ttf'))
font12 = ImageFont.truetype(font_path, 12)
font14 = ImageFont.truetype(font_path, 14)
font16 = ImageFont.truetype(font_path, 16)

# lock for thread synchronization
lock = threading.Lock()

# default exit code is failure - a graceful termination with a
# terminate signal is possible.
exit_code = 1

# button setup
pin_number_menu_button = 18
pin_number_select_button = 16

GPIO.setwarnings(False) # Ignore warning for now
GPIO.setmode(GPIO.BOARD) # Use physical pin numbering

GPIO.setup(pin_number_menu_button, GPIO.IN, pull_up_down=GPIO.PUD_UP) # Set pin to be an input pin and set initial value to be pulled low (off)
GPIO.setup(pin_number_select_button, GPIO.IN, pull_up_down=GPIO.PUD_UP)

image_capture_start_time = 0
image_captured = False

#  Should be at least 6 seconds for Sony A7R2, since it takes about 5.5 seconds to take and save image. 
#  This delay should be longer then that.
intervalometer_delay = 5

#  Create the flight log file
image_root_path = '/home/pi/Project01/images'
csv_output_file_name = 'data.csv'
csv_target = os.path.join(image_root_path, csv_output_file_name)

file = open(csv_target, 'a', newline='', encoding='UTF-8')
csv_writer = csv.writer(file)
if not os.path.isfile(csv_target):
    csv_writer.writerow(['Image Name', 'X', 'Y', 'Z', 'Roll', 'Pitch', 'Yaw', 'Date/Time'])

# Transformation to convert different camera orientations to NED convention. Replace camera_orientation_default for your configuration.
#   0: Forward, USB port to the right
#   1: Downfacing, USB port to the right 
#   2: Forward, 45 degree tilted down
# Important note for downfacing camera: you need to tilt the vehicle's nose up a little - not flat - before you run the script, otherwise the initial yaw will be randomized, read here for more details: https://github.com/IntelRealSense/librealsense/issues/4080. Tilt the vehicle to any other sides and the yaw might not be as stable.
camera_orientation = 0

# Offset in NED coordinates; Distance from DSLR imagins sensor to T265
body_offset_enabled = 1
body_offset_x = 0.066       # In meters (m)
body_offset_y = 0           # In meters (m)
body_offset_z = 0.060      # In meters (m)

#######################################
# Global variables
#######################################

# T265 Camera-related variables
pipe = None
pose_sensor = None

menu = False
menu_select = False
menu_option = 0
current_menu_navigation = []

# Data variables
data = None
current_confidence_level = None
H_aeroRef_aeroBody = None
V_aeroRef_aeroBody = None
heading_north_yaw = None

Raw_RPY = None
NED_RPY = [0, 0, 0]
Raw_xyz = None
NED_xyz = [0, 0, 0]
global_position = [0, 0, 0]
global_position_offset = [0, 0, 0]  # Initialize the start position

line1 = ''

# Global scale factor, position x y z will be scaled up/down by this factor
scale_factor = 1.0

# Enable using yaw from compass to align north (zero degree is facing north)
compass_enabled = 0

# Main execution variable
main_loop_should_quit = False
thread_should_quit = False

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
        
def button_menu_press(channel):
    global menu_option
    global menu
    
    menu = True
    menu_option += 1
    #print ("Menu button was pushed")
    
def button_select_press(channel):
    global menu_select
    menu_select = True

def button_handling():
    GPIO.add_event_detect(pin_number_select_button, GPIO.FALLING, callback = button_menu_press, bouncetime = 500) # Setup event on rising edge
    GPIO.add_event_detect(pin_number_menu_button, GPIO.FALLING, callback = button_select_press, bouncetime = 500) # Setup event on rising edge

#######################################
# Functions - Update Displays and files
#######################################

def update_position_data():
    global global_position
    
    global_position[0] = NED_xyz[0] - global_position_offset[0]
    global_position[1] = NED_xyz[1] - global_position_offset[1]
    global_position[2] = NED_xyz[2] - global_position_offset[2]
   
def update_console():
    print('Frame number: ', pose.frame_number, f' Position X:{global_position[0]:.3f} Y:{global_position[1]:.3f} Z: {global_position[2]:.3f} :: Roll: {NED_RPY[0]:.1f} Pitch:{NED_RPY[1]:.1f}  Yaw:{NED_RPY[2]:.1f}  ', end = "\r")

def update_oled():
    while not main_loop_should_quit:
        with canvas(device) as draw:
            if menu == False:
                oled_xyz_disp(draw)
            elif menu == True:
                main_menu_display(draw)
        #sleep (0.05) # Update 20 times per second

def main_menu_display(draw):
    global menu_option
    global current_menu_navigation
    global menu_select
    current_menu_text_options = []
    
    # Draw menu options

    # Menu Title, function name    
    current_menu_text_options.append(["Start/Stop Intervalometer", "intervalometer_control"])
    current_menu_text_options.append(["Reset Odometry","odometry_reset"])
    current_menu_text_options.append(["Restart T265","restart_t265"])
    current_menu_text_options.append(["Set Delay for Intervalometer","intervalometer_set_delay"])
    current_menu_text_options.append(["Clear image folder", "image_folder_reset"])
    current_menu_text_options.append(["Back", "main_menu_back"])
    current_menu_text_options.append(["Exit", "exit_program"])
    
    total_menu_options = len(current_menu_text_options)
    if menu_option > total_menu_options:
        menu_option = 1
    
    menu_counter = 0
    for x in range(len(current_menu_text_options)):
        menu_display_position = 9 * x
        draw.text((7, menu_display_position), current_menu_text_options[x][0], font=font12, fill="white")
    
    # Draw the selector
    selector_position = 9 * (menu_option - 1)
    draw.text((0,selector_position), ">", font=font12, fill="white")
    
    current_menu_navigation.append([(menu_option - 1),0])
    if menu_select == True:
        function_name = str(current_menu_text_options[(menu_option - 1)][1])
        globals()[function_name]()
        menu_select = False

def image_folder_reset():
    global menu
    global file_number
    
    os.system('rm -rf images/*')
    file_number = 0
    with canvas(device) as draw:
        draw.text((0,18), "Image folder Cleared", font=font16, fill="white")
        draw.text((0,35), "File number reset", font=font16, fill="white")
    # print(" Clear image folder")
    sleep(5)
    menu = False
    
def restart_t265():
    global pipe
    global menu
    
    pipe.stop()
    pipe.start(cfg)
    with canvas(device) as draw:
        draw.text((0,25), "T265 restarted", font=font16, fill="white")
    # print(" T265 has been restarted")
    sleep(5)   
    menu = False    
    
def intervalometer_set_delay():
    global menu
    global intervalometer_delay
    
    if intevelometer_on == True:
        with canvas(device) as draw:
            draw.text((0,25), "Stop the Intervelometer", font=font14, fill="white")
        sleep(2)
    if intevelometer_on == False:
        if intervalometer_delay > 12:
            intervalometer_delay = 5
        else:
            intervalometer_delay += 1
        with canvas(device) as draw:
            draw.text((0,20), "Intervalometer delay ", font=font14, fill="white")
            draw.text((0,35), "set to " + f"{intervalometer_delay:2.0f} seconds", font=font14, fill="white")
        sleep(2)
    # print("intervalometer_setup function executed")
    menu = False

def odometry_reset():
    global global_position_offset
    global menu
    
    global_position_offset = NED_xyz

    with canvas(device) as draw:
        draw.text((0,25), "New Origin established", font=font14, fill="white")
    sleep(5)
    # print("odometry_reset function executed")
    menu = False

def intervalometer_control():
    global menu
    global intevelometer_on
    global intervalometer_thread
    
    if intevelometer_on == False:
        intevelometer_on = True
        intervalometer_thread.start()
    elif intevelometer_on == True:
        intevelometer_on = False
        intervalometer_thread.join()
    #sleep(5)
    menu = False
    
def exit_program():
    global menu
    global main_loop_should_quit
    
    main_loop_should_quit = True
    menu = False
    
def main_menu_back():
    global menu
    menu = False

def oled_xyz_disp(draw):

    if intevelometer_on == True:
        time_remaining = image_capture_start_time + intervalometer_delay - time.time()

    if intevelometer_on == True:
        draw.text((0,0), "Image:" + f"{file_number:3.0f}", font=font12, fill="white")
        draw.text((0,10), "Int Timer:" + f"{time_remaining:2.0f}", font=font12, fill="white")
    if intevelometer_on == False:
        draw.text((0,0), "Int OFF", font=font12, fill="white")
 

    try:
        draw.text((85,0), f" CL: {data.tracker_confidence:.0f}", font=font12, fill="white")
    except:
        pass
        
    draw.text((0,35), "X:", font=font12, fill="white")
    draw.text((0,45), "Y:", font=font12, fill="white")
    draw.text((0,55), "Z:", font=font12, fill="white")

    draw.text((10,35), f"{global_position[0]: 3.3f}m", font=font12, fill="white")
    draw.text((10,45), f"{global_position[1]: 3.3f}m", font=font12, fill="white")
    draw.text((10,55), f"{global_position[2]: 3.3f}m", font=font12, fill="white") 
   
    draw.text((60,35), "Roll :", font=font12, fill="white")
    draw.text((60,45), "Pitc:", font=font12, fill="white")
    draw.text((60,55), "Yaw:", font=font12, fill="white")
    
    draw.text((85,35), f"{NED_RPY[0]: 0.1f}\u00b0", font=font12, fill="white")
    draw.text((85,45), f"{NED_RPY[1]: 0.1f}\u00b0", font=font12, fill="white")
    draw.text((85,55), f"{NED_RPY[2]: 0.1f}\u00b0", font=font12, fill="white")

def update_csv(file_name):
    x_position = global_position[0]
    y_position = global_position[1]
    z_position = global_position[2]
    
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
    global image_captured
    
    # while not main_loop_should_quit: 
    while intevelometer_on == True:
        # if intevelometer_on == True:
        try: 
            camera = gp.Camera()
            camera.init()

            file_number += 1
            base_name = 'argus-'
            extension = '.jpg'
            new_file_name = base_name + "%05d" % file_number + extension 
            
            time_1 = (time.time() - image_capture_start_time)
            #print('2.  Capturing image - Time: ', (time.time() - image_capture_start_time), "             ")
            
            update_csv(new_file_name)
            image_capture_start_time = time.time()
            file_path = camera.capture(gp.GP_CAPTURE_IMAGE) 
            
            time_2 = (time.time() - image_capture_start_time)
            #print('3.  Image capture finished - Time: ', (time.time() - image_capture_start_time), "             ")
            
            target = os.path.join(image_root_path, new_file_name)
            # print('Copying image to', target)
           
            camera_file = camera.file_get(
               file_path.folder, file_path.name, gp.GP_FILE_TYPE_NORMAL)
            camera_file.save(target)           
            camera.exit()
            image_captured = True
            time_3 = (time.time() - image_capture_start_time)
            print(f'Image copy finished - ImageCapture: {time_2:0.2f}s, ImageSave: {(time_3-time_2):0.2f}s, TotalTime: {time_3:0.2f}s :: Images captured: {file_number:0.0f}')

            sleep_delay = intervalometer_delay - (time.time() - image_capture_start_time)

            if sleep_delay > 0.1:
                sleep(sleep_delay)
        except:
            camera.exit()
            intevelometer_on = False
            image_captured = False


#######################################
# Main code loop
#######################################
signal.signal(signal.SIGTERM, sigterm_handler)

# Initial LCD display message
text = []
text.append(f"Loaded version {software_version}")
text.append(" ")
text.append("Press button to continue")
while GPIO.input(pin_number_menu_button):   # Wait for button to be pushed before going farther
    with canvas(device) as draw:
        draw.text((0,0), text[0], font=font14, fill="white")
        draw.text((0,20), text[1], font=font14, fill="white")
        draw.text((0,40), text[2], font=font14, fill="white")
    sleep(0.1)
device.clear()

try:
    print("INFO: pyrealsense2 version: %s" % str(rs.__version__))
except Exception:
    # fail silently
    pass

# Connect to T265 camera
realsense_connect()

update_oled_thread = threading.Thread(target = update_oled, daemon = True)  # Setup the oled update thread
update_oled_thread.start()  # Start the oled update thread

intervalometer_thread = threading.Thread(target = intervalometer_run, daemon = True)  # Setup the interfalometer thread
# intervalometer_thread.start()

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
    H_T265body_aeroBody = (tf.euler_matrix(math.pi/4, 0, 0)).dot(np.linalg.inv(H_aeroRef_T265Ref))
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
                # create_oled_message()
                update_console()        
     
                if menu == False:
                    menu_option = 0
     
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
    # camera.exit()
    intevelometer_on = False
    
    # Stop the threads
    update_oled_thread.join()  
    intervalometer_thread.join()
    
    # start a timer in case stopping everything nicely doesn't work.
    signal.setitimer(signal.ITIMER_REAL, 5)  # seconds...
    file.close()
    pipe.stop()

    print("INFO: Realsense pipeline and vehicle object closed.")
    sys.exit(exit_code)