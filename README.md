# Bluefox3 ROS driver

## Description

The driver is admittedly pretty bare-bones so far.
I've only implemented stuff that I needed at the moment due to a lack of time, but expanding the driver e.g. to enable configuration of more camera parameters should be pretty straight-forward.
Pull-requests welcome!

*Features so far:*
* image and camera info publication as ROS messages
* mirror mode configuration*
* exposure time configuration*
* autoexposure mode configuration*

(* = dynamically reconfigurable parameters)

## Prequisites

The `mvIMPACT_Acquire` library corresponding to your camera is required to be installed and in present path (must be findable by CMake).
C++ standard >= `c++17` is required for compilation.
Tested with ROS Melodic, but should work with older/newer versions as well.
