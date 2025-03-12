# Soft Robotics mGrip P2

## URCaps

To control the gripper, you need to install the SRI_coDrive URCaps onto the UR. The current version is 1.1.1, although later versions may still work with the present ROS2 service. 

## Putting the gripper and the UR together

Currently, only one UR5e out of the ones in the laboratory support this gripper as it's the only one with the correct attachment for the compressed air. Since one end of the cable should already be attached to the robot, we'll just handle how to assemble the gripper onto the flange. 

Simply take the end-effector, and instead of aligning the fifth smaller hole with the one on the flange, turn it 180 degrees. This will allow the gripper to rotate more freely. 

Screw in the 4 screws and you are done!

## Parameters

The server has different parameters:

- `host`: the IP address of the UR onto which the soft gripper is attached;
- `port`: the port to which to connect;
- `gripper_service_name`: the name of the service onto which to send the service requests;

## Notice

The server also uses the service `io_and_status_controller/resend_robot_program` to send the UR scripts
to the robot. For this reason, it may be necessary to add a namespace. 

Also, one needs to be connected to the UR device using the ROS2 UR driver wrapper so that the previous service is functioning.