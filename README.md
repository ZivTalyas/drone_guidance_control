# Developing a tracking algorithm of a drone's linear acceleration profile in an environment without GPS
This project implements the article "Direct Acceleration Feedback Control of Quadrotor Aerial Vehicles",
this article represents how to use a control method that is based on acceleration to create a controller for controlling the desired acceleration.

**Note**: The acceleration that discusses in this file is **only** a linear acceleration.

## Overview
The project includes end-to-end scripts for flying the drone, those scripts are applied with the help of ROS2, Python, and MAVSDK. The scripts include generating known/random acceleration trajectories, acceleration, and orientations measurements, and implementing a direct acceleration feedback controller that calculates the roll, pitch, yaw, and thrust values to receive the desired acceleration. After that, sending the roll, pitch, yaw, and thrust values to the realization on the drone.

## Major scripts:
1. **trajectory_maker.py**  : Generating known/random acceleration trajectories.
2. **attitude_quaternion.py** : Orientations measurements. Measuring of the orientations angles in quaternion value
3. **acceleration_frd.py** : Acceleration measurements. Measuring the acceleration values in body acceleration, then transforming to the world's acceleration values
4. **close_loop_control.py** : Implement of direct acceleration feedback controller that calculates the roll, pitch, yaw, and thrust values to receive the desired acceleration, this script also sends the roll, pitch, yaw, and thrust values to the drone. After that, sending the roll, pitch, yaw, and thrust values to the realization on the drone.

## Side scripts:
1. **graph_errors.py** : Creating multiple types of graphs.
2. **acceleration_tune_node.py** : Test trajectory for calibration of the system.

## How to run the project?
**Note**: Before starting, make sure you have already installed the Arena simulator, QGroundControl, and Terminator. Furthermore, you must build your ROS2 folder that includes the project.
1. Open the Terminator.
2. Open inside the Terminator 6 sub-windows.
3. Run the following commands to run QGroundControl inside one of the sub-windows. (first sub-window)
```bash
cd Downloads
```
```bash
./QGroundControl.AppImage 
```
4. Run the following command to run Arena simulator inside one of the sub-windows. (second sub-window)
```bash
ros2 launch arena_backend_multicopter multicopter.launch.py 
```
5. Inside the rest of the sub-windows (the 4 last sub-window), enter the directory of where you set your project folder.
```bash
Example for directory :  ros2_ws/src/my_pk_pkg/my_project
```
6. Then, inside each window run those commands, **in the same order**: 
```bash
./attitude_quaternion.py
```
```bash
./trajectory_maker.py
```
```bash
./acceleration_frd.py
```
```bash
./close_loop_control.py
```
7. You should now be able to see that the simulation is working. 
