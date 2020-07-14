# Niryo One ROS controlled simulation. 

## What is this?
If you are searching for a ROS controlled simulation for the Niryo One in CoppeliaSIM then consider yourself lucky because that is what this repository is supposed to contain. 

## How to use it?

**It is assumed that you have an operational and well configured ROS kinetic environment, so make sure you have a ROS master in a well known machine.**

0. Prepare for current setup. 
 - Configuration for the environment is contained in the docker run command on `run_container.sh`
 - All relevant hosts on the ROS network should be added as hosts. 
 - Set the ROS master uri to the one on your setup.
 - Set the ROS ip to the one of the interface connecting to the ROS network.
 - Connect host to the ROS network with an operational ROS master.
1. Start CoppeliaSim within ROS environment container: `./run_container.sh`
 - If the ROS master is unreachable or not operational, the ROS interface will not launch. 
2. Search for a Niryo One model on the robot list and put it on the simulator. 
3. Right click on the threaded script, edit, and remove. 
4. Add a new **Non-Threaded** script.
5. Copy and paste contents of the lua script in this repository.
6. Run the simulation and adapt the speed accordingly to achieve realtime feedback.

