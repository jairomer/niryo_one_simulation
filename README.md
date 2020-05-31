# Niryo One ROS controlled simulation. 

## What is this?
If you are searching for a ROS controlled simulation for the Niryo One in CoppeliaSIM then consider yourself lucky because that is what this repository is supposed to contain. 

## How to use it?

**It is assumed that you have an operational and well configured ROS kinetic environment, so make sure you have a ros master in a well known machine and that the ros interface is being imported as a dynamic library by coppeliaSIM.**

0. Start roscore and move the ros library from '.../CoppeliaSim/compiledRosPlugins/' to '.../CoppeliaSim/'. 
1. Start CoppeliaSim. 
2. Add as much Niryo One robots as you want. 
3. On each of them:
    - Remove the threaded script.
    - Add a non threaded script. 
    - Copy the code from the only LUA script on this repository and paste it on the non threaded script. 
    - Change the robotID variable to a different number on each robot. 
4. Run the simulation. 
5. If the ROS master was found, then a class of topics for each robot should have appeared. 
6. You can now publish and subscribe data to these topics using 'rostopic'.

#### Example: Close and open the Gripper. 

```
rostopic pub /coppeliaSIM/NiryoOne_0/GripperCommandSub std_msgs/Bool "data: false" # Open the gripper for Niryo with robotID 0.
rostopic pub /coppeliaSIM/NiryoOne_0/GripperCommandSub std_msgs/Bool "data: true" # Close the gripper for Niryo with robotID 0.
```
