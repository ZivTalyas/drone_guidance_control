#!/usr/bin/env python3

import asyncio
from mavsdk import System
import rclpy
from rclpy.node import Node
import numpy as np
from mavsdk.offboard import (Attitude, OffboardError)
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
import random
from std_msgs.msg import Float32MultiArray
import time



class NumberPublisherNode(Node):
    def __init__(self):
        super().__init__("trajectory_maker")
        self.trajectory_maker_ = [0.0, 0.0, 18.0]
        self.trajectory_maker_publisher_ = self.create_publisher(Float32MultiArray, "trajectory_maker", 10)
        self.trajectory_maker_timer_ = self.create_timer(0.022, self.publish_number)
        self.get_logger().info("trajectory publisher has been started.")

        
######## publisher #########
    def publish_number(self):
        global start_time
        end_time = time.time()
        msg = Float32MultiArray()
        if end_time - start_time < 20 :  ######## zero step of the drone, acceleration in z axis -> only thrust ##########
            msg.data = self.trajectory_maker_
            self.trajectory_maker_publisher_.publish(msg)
            print("starting step 0 , time:", end_time-start_time)
        elif end_time - start_time > 20 and end_time - start_time < 32: ######## first step of the drone, acceleration in x axis and acceleration in z axis ##########
            end_time_for_print_step_1 = time.time()
            print("starting step 1 , time:", end_time_for_print_step_1-start_time) 
            self.trajectory_maker_ = [1.5, 0.0, 11.0]
        elif end_time - start_time > 32 and end_time - start_time < 42: ######## second step of the drone, acceleration in z axis ##########
            end_time_for_print_step_2 = time.time()
            print("starting step 2 , time:", end_time_for_print_step_2-start_time)
            self.trajectory_maker_ = [0.0, 0.0, 18.0]
        elif end_time - start_time > 42 and end_time - start_time < 54: 
            end_time_for_print_step_3 = time.time()
            print("starting step 3 , time:", end_time_for_print_step_3-start_time) ######## third step of the drone, acceleration in y axis and acceleeration in z axis ##########
            self.trajectory_maker_ = [0.0, 1.5, 11.0]
        else:
            end_time_for_print_step_4 = time.time()
            print("starting step 4 , time:", end_time_for_print_step_4-start_time)
            self.trajectory_maker_ = [0.0, 0.0, 18.0]

        msg.data = self.trajectory_maker_
        self.trajectory_maker_publisher_.publish(msg)

def step_trajectory():
    choose_option = random.choice([1,2])
    if choose_option == 1:
       trajectory_maker_ = [random.uniform(-3.0, 3.0), 0.0, 12.0] 
    if choose_option == 2:
        trajectory_maker_ = [0.0, random.uniform(-3.0, 3.0), 12.0]
    return trajectory_maker_

def cos_trajectory():
    choose_option = random.choice([1,2])
    if choose_option == 1:
       trajectory_maker_ = [3.0*np.cos(random.uniform(-np.pi, np.pi)), 0.0, 12.0] 
    if choose_option == 2:
        trajectory_maker_ = [0.0, 3.0*np.cos(random.uniform(-np.pi, np.pi)), 12.0]
    return trajectory_maker_

def sin_trajectory():
    choose_option = random.choice([1,2])
    if choose_option == 1:
       trajectory_maker_ = [3.0*np.sin(random.uniform(-np.pi, np.pi)), 0.0, 12.0] 
    if choose_option == 2:
        trajectory_maker_ = [0.0, 3.0*np.sin(random.uniform(-np.pi, np.pi)), 12.0]
    return trajectory_maker_

def main_publisher(args=None):
    rclpy.init(args=args)
    node = NumberPublisherNode()
    rclpy.spin(node)
    rclpy.shutdown()


async def run():
    global start_time
    start_time = time.time()
    main_publisher()

if __name__ == "__main__":
    # Start the main function
    asyncio.ensure_future(run())
    # Runs the event loop until the program is canceled with e.g. CTRL-C
    asyncio.get_event_loop().run_forever()