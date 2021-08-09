# VSLAM_Photogrammetry

I'm creating a device which records image pose data (position and rotation) in GPS denied environments. 
This iteration of the device uses the Intel RealSense T265 VSLAM sensor to do the heavy lifting and generate the pose data. 
The pose data is fed into a Raspberry Pi 3 where it is saved to a flight log file each time an image is taken.
The Raspberry Pi also conects to your camera of choice and triggers an images capture and then saves the images locally on the Pi's SD card. 

The current code on the Pi creates an intervalometer and triggers the camera every 10 sec. This gives you time to move the camera to the next position.

### NOTE
Do not use a raspberry Pi 4. There are problems with the T265 dropping connection with the Raspberry Pi 4 at random intervals. Raspberry Pi 3 works great. 


### FB Group
Join the FB group where I'll be posting more updates
https://www.facebook.com/groups/diyphotogrammetryhardware/
