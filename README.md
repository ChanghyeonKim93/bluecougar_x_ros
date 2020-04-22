# bluecougar_x_ros
A ROS C++ wrapper for Matrix Vision GigE vision camera, mvBlueCOUGAR-X series.
I tested it with ROS KINETIC + linux Ubuntu 16.04 LTS. A tested camera model is exactly mvBlueCOUGAR-X-104iG.

## Feature
I construct this code for 'hardware sync. by external trigger signal'. This node supports hardware trigger mode. 
We use a digIn0+ pin on the camera 12-pin outlet as a trigger pin. 
We provide trigger signal (0~5V digital, 0~0.3V low / 3.0V~12.0V high) from the Arduino UNO digital signal.

## Pre-requisite
Before using, camera driver (mvGenI) need to be installed. It can be found at the manufacturer site. (Matrix Vision).

## Camera hardware configuration
To use mvBlueCOUGAR-X, gigabits ethernet should be supported for both your LAN card(neccesary) and router(if required).
The class of all ethernet cables should be higher than CAT. 5E (category 5e). We recommand to use CAT. 6 cables for stable operations.


## Questions & issues
Please contact to e-mail (hyun91015@gmail.com).
