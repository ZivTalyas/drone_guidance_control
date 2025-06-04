#!/usr/bin/env python3

import asyncio
from mavsdk import System
import rclpy
from rclpy.node import Node
import numpy as np
import math
from mavsdk.offboard import (Attitude, OffboardError)
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3

from std_msgs.msg import Float32MultiArray
import aiogrpc

import time



class NumberPublisherNode(Node):
    def __init__(self):
        super().__init__("acceleration_frd")
        self.attitude_quaternion_publisher_ = self.create_publisher(Float32MultiArray, "acceleration_frd", 10)
        self.angles_maker_timer_ = self.create_timer(0.022, self.acceleration_frd_ros)
        self.get_logger().info("acceleration frd publisher has been started.")

######## publisher #########
    async def acceleration_frd_ros(self):
        global acceleration_value
        new_msg = Float32MultiArray()
        new_msg.data = acceleration_value
        self.attitude_quaternion_publisher_.publish(new_msg) 

######## getting the linear acceleration to each axis from the imu #########
async def acceleration_frd():
        global acceleration_value
        global acceleration_final_measure
        global random_number
        acceleration_x = 0
        acceleration_y = 0
        acceleration_z = 0

        async for imu in drone.telemetry.imu(): 
            acceleration_x = imu.acceleration_frd.forward_m_s2
            acceleration_y = imu.acceleration_frd.right_m_s2
            acceleration_z = imu.acceleration_frd.down_m_s2
            theta = math.atan2(acceleration_y, math.sqrt(acceleration_x**2 + acceleration_z**2)) ##roll angle
            phi = math.atan2(acceleration_x, math.sqrt(acceleration_y**2 + acceleration_z**2)) ##pitch angle
        # Calculate yaw angle (assumed to be provided externally)
            psi = math.radians(-57.2957795)  # Example yaw angle of 58 degrees
            # psi = math.radians(0.0)  # Example yaw angle of 58 degrees
            # Construct rotation matrix based on roll, pitch, and yaw angles
            R = np.array([[math.cos(psi) * math.cos(theta),
                        math.cos(psi) * math.sin(phi) * math.sin(theta) - math.sin(psi) * math.cos(phi),
                        math.cos(psi) * math.cos(phi) * math.sin(theta) + math.sin(psi) * math.sin(phi)],
                        [math.sin(psi) * math.cos(theta),
                        math.sin(psi) * math.sin(phi) * math.sin(theta) + math.cos(psi) * math.cos(phi),
                        math.sin(psi) * math.cos(phi) * math.sin(theta) - math.cos(psi) * math.sin(phi)],
                        [-math.sin(theta),
                        math.cos(theta) * math.sin(phi),
                        math.cos(theta) * math.cos(phi)]])
            # Calculate gravity components
            G = np.array([[0],
                        [0],
                        [0]])  # Assuming gravity is 9.81 m/s^2
            # Construct linear acceleration vector
            a_body = np.array([[acceleration_x],
                            [acceleration_y],
                            [acceleration_z]])
            # Rotate acceleration from body frame to world frame
            a_world = np.dot(R, a_body) + G
            acceleration_value = [a_world[0,0], -a_world[1,0], -a_world[2,0]]
            return acceleration_value

######## connection to drone #########
async def run():
    global start_time
    global acceleration_final_measure
    global random_number
    # Init the drone
    await drone.connect(system_address="udp://:14540")
    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"-- Connected to drone!")
            break

    print("-- Arming")
    await drone.action.arm()
    # await asyncio.sleep(5)
    start_time = time.time()
    for i in range(10000):
        acceleration_measure = await acceleration_frd() ### list of acceleration values from the IMU ###
        acceleration_measure = np.array(acceleration_measure)  ### array of acceleration values from the IMU ###  
        acceleration_final_measure = np.vstack((acceleration_final_measure, acceleration_measure)) ### used to stack the sequence of Numpy arrays vertically and return the single array ###
        end_time = time.time()
        if (end_time - start_time > 60):
            break   
    if (random_number == 0):
        print("printing values")
        np.savetxt("acceleration_measure.txt",acceleration_final_measure) 
        random_number = 1


async def spin_once(number_publisher_node):
    rclpy.spin_once(number_publisher_node, timeout_sec=0)

async def run_ros2(number_publisher_node):
    while True:
        await spin_once(number_publisher_node)
        await asyncio.sleep(0.022)

######## main function which make ROS2 and mavsdk communication to run parallel #########
async def main(args=None):
    global drone 
    global random_number
    global acceleration_final_measure ### array of acceleartion values ###
    global acceleration_value ### list of acceleartion values ###
    drone = System()
    random_number = 0
    acceleration_value = [0.0, 0.0, 0.0]
    acceleration_final_measure = np.array([0.0, 0.0, 0.0])
    rclpy.init(args=args)
    number_publisher_node = NumberPublisherNode()
    task_ros2 = asyncio.create_task(run_ros2(number_publisher_node))
    task_mavsdk = asyncio.create_task(run())
    await asyncio.gather(task_ros2, task_mavsdk)
    rclpy.spin(number_publisher_node)
    number_publisher_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    # Start the main function
    asyncio.run(main())
    # Runs the event loop until the program is canceled with e.g. CTRL-C
    asyncio.get_event_loop().run_forever()
