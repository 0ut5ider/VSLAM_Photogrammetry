# Python script 

The current code requires the latest version of libgphoto2 to control the camera.

Hardware attached to the raspberry pi is two buttons and a 2004 LCD screen.
The hardware buttons are connected to GPIO pin# 16 and GPIO pin# 18. The for both buttons, the other side is connected to GND.

### Body Offset
The T265 sensor center will be different then the optical center of the camera you mount on top.
For the A7 series cameras, the offset between the two (based on the curent CAD model) is x=66cm, y=0m z=50cm
This needs to be adjusted for YOUR camera in order to obtain accurate data.

### Script operation
Upon starting, the script waits for a button push on GPIO pin# 16 before initializing the T265
Once the position data is displayed in console and LCD screen, an intervalometer can be started by preseing a button on GPIO pin# 18. 

The intervalometer is set for 10s (`intervalometer_delay`) 
A second press of the GPIO pin#18 will stop the intervalometer. 

The scriopt root folder also needs a folder called `images/`
This is where the flightlog file (called `data.txt`) will be saved along with all the images. 

When the intervalometer is running, a photo will be taken (using the gphoto2 library), image file downloaded to Raspberry Pi, renamed and saved in the `images/` folder.
A new row entry will be added to the log file with the `Image Name, x, y, z, roll, pitch, yaw, date/time`  

### CAUTION
Don't forget to clear out the images folder between photography sessions. The script will not clear the data in there, and some funky things may happen if you don't
