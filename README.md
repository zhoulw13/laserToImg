# laserToImg
## Introduction
a ros node
## Hardware
laserscan, usb camera
## Function
plot the 3D data from laserscan into camera's 2D image real time, and can split image saved into patches with a depth.which can be used for machine learning or deep learning
## Usage
### 1. data collect
`rosrun laserToImg laserToImg_node`
* press 's' to start collect, before that you can ajust the laserscan and camera

* press space bar to exit

### 2. split
`rosrun laserToImg split_node`

## Other
the camera should be pre-calibrated, temporarily you can only change my code in laserToImg.cpp for your parameters.
