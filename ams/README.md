# ROS APM Controller 
A repo for controlling APM quadcopter via MAVLINK using mavros with a mocap system.
## Considerations
The quadrotor's needs to started in the coincided with the mocap's frame when creating a trackable in motive.
## ams_common
Contains basic functions and enums for all nodes to use.
## ams_manager
A interface to the quadrotor with basic functions such as arming/disarming and changing modes.  Also, it makes
to change modes (ie. When performing a take-off, once the aircraft has reached 1 m.  It switches to hover mode).  In
addition, it performs the trajectory.
## ams_quad_controller
The a PID controller that sends out a RC message to control the pose (position and yaw) of the quadrotor.
## ams_telep
A basic teleop that sends out a RC message.
