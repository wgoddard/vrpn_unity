This demo project has two scenes, MotionNode and GPSKlaus.

The MotionNode scene should show a single cube with a red square on its rear.
For demonstration, the MotionNodeToUnity prefab has been given a Y rotation set of 40 deg. 
The cube should be oriented forward in the Unity coordinate system when the MotionNode sensor is facing 40 deg clockwise from due North.
Follow the MotionNode installation procedure and start the daemon by running the MotionNode application.
The VRPN server will connect to the device and initiate reading.

The GPSKlaus scene should show a red dot over the Klaus courtyard.
Each GPS device uses one of the serial ports located in /dev on OSX.
Configure the vrpn.cfg.txt file in the resources directory to match the serial port.

ahill@gatech.edu
