# mdek_driver
MDEK 1001 - ROS nodes - serial, bluetooth, accelerometer

This package is intended for anyone who would like to get started using the MDEK1001 kit by Decawave within the ROS framework.

This package consists of 4 nodes :
ranges.py is simply for tag to anchor ranges through serial port, 
ranges_scan.py uses bluetooth to scan for anchors to get anchor to anchor ranges, 
ranges_manual.py does the same but the bluetooth adressess are entered manually, 
accels.py gives the acceleration data through serial port

UWB.msg is a message of tag to anchor ranges : rt1 = range tag to anchor 1, ra12 = range anchor 1 to anchor 2
ranges.launch has the path to the name of the serial device which could need to be changed
