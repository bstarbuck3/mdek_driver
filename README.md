# mdek_driver
MDEK 1001 - ROS nodes - serial, bluetooth, accelerometer

This package is intended for anyone who would like to get started using the MDEK1001 kit by Decawave within the ROS framework.

These codes were made by thoroughly examining the MDEK api user manuals and implementing the necessary commands in python for the purpose of evaluating their use in a localization application.

This package consists of 4 nodes :
ranges.py is simply for tag to anchor ranges through serial port, 
ranges_scan.py uses bluetooth to scan for anchors to get anchor to anchor ranges, 
ranges_manual.py does the same but the bluetooth adressess are entered manually, 
accels.py gives the acceleration data through serial port

UWB.msg is a ROS message of ranges : rt1 = range tag to anchor 1, ra12 = range anchor 1 to anchor 2.
ranges.launch should be used to run the python file that you choose.
