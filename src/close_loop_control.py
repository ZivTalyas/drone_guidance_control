#!/usr/bin/env python3

import asyncio
from mavsdk import System
import rclpy
from rclpy.node import Node
import numpy as np
from numpy.linalg import norm 
from mavsdk.offboard import (Attitude, OffboardError)
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
import time

from std_msgs.msg import Float32MultiArray
 

class NumberPublisherNode(Node):
    def __init__(self):
        super().__init__("close_loop_control")
        self.acceleraion_desgired = None
        self.acceleraion_measured = None
        self.attitude_quaternion = None
        self.trajectory_maker_subscriber_ = self.create_subscription(Float32MultiArray, "trajectory_maker", self.close_loop_control_desigred, 10)
        self.attitude_quaternion_subscriber_ = self.create_subscription(Float32MultiArray, "attitude_quaternion", self.close_loop_control_quaternion, 10)
        self.acceleration_frd_subscriber_ = self.create_subscription(Float32MultiArray, "acceleration_frd", self.close_loop_control_measured, 10)
        self.close_loop_control_publisher_ = self.create_publisher(Float32MultiArray, "close_loop_control", 10)
        self.get_logger().info("close loop control has been started.")

    def close_loop_control_desigred(self, msg): 
            self.acceleraion_desgired = np.array(msg.data)
    
    def close_loop_control_quaternion(self, msg): 
            self.attitude_quaternion = msg.data 
            # sinr_cosp = 2.0 * (self.attitude_quaternion[0] * self.attitude_quaternion[1]  + self.attitude_quaternion[2] * self.attitude_quaternion[3])
            # cosr_cosp = 1.0 - 2.0 * (self.attitude_quaternion[1] * self.attitude_quaternion[1] + self.attitude_quaternion[2] * self.attitude_quaternion[2])
            # roll = np.arctan2(sinr_cosp, cosr_cosp)
            
            # sinp = 2.0 * (self.attitude_quaternion[0]  * self.attitude_quaternion[2] - self.attitude_quaternion[3] * self.attitude_quaternion[1])
            # if np.abs(sinp) >= 1:
            #     pitch = np.pi / 2.0 * np.sign(sinp)
            # else:
            #     pitch = np.arcsin(sinp)
            
            # if (np.degrees(roll) < 0.1 and np.degrees(roll) > -0.1):
            #     roll = 0.0
            # if (np.degrees(pitch) < 0.1 and np.degrees(pitch) > -0.1):
            #     pitch = 0.0
            # self.attitude_quaternion = [np.degrees(roll), np.degrees(pitch), 0.0]
            
    async def close_loop_control_measured(self, msg):
            global counter_step
            thrust_max = 207.36
            new_msg = Float32MultiArray()
            self.acceleraion_measured = np.array(msg.data)
            exp_angle = error_control(self.acceleraion_desgired, self.acceleraion_measured)
            if (exp_angle[0] == 'pitch'):
                 traj_angle = [0, exp_angle[1], 0 ,exp_angle[0],exp_angle[2]]
            if (exp_angle[0] == 'roll'):
               traj_angle = [exp_angle[1],0, 0 ,exp_angle[0],exp_angle[2]]
            if (exp_angle[0] == 'no angle'):
                 traj_angle = [0, 0, 0 ,exp_angle[0],exp_angle[2]]
            difference_angle = np.empty([1,3])
            if (exp_angle[0] == 'pitch'):
                difference_angle[0,0] = self.attitude_quaternion[0]
                difference_angle[0,1] = traj_angle[1] - self.attitude_quaternion[1]
                difference_angle[0,2] = 0.0
            if (exp_angle[0] == 'roll'):
                difference_angle[0,0] = traj_angle[0] - self.attitude_quaternion[0]
                difference_angle[0,1] = self.attitude_quaternion[1]
                difference_angle[0,2] = 0.0 
            if (exp_angle[0] == 'no angle'):
                difference_angle[0,0] = self.attitude_quaternion[0]
                difference_angle[0,1] = self.attitude_quaternion[1]
                difference_angle[0,2] = 0.0     
            drone_step = [difference_angle[0,0], difference_angle[0,1], difference_angle[0,2],traj_angle[4]/thrust_max]
            print("-- Step number ",counter_step, "-- Go up at ", drone_step[3], " thrust")
            print(drone_step[0], drone_step[1], drone_step[2], drone_step[3])
            if (traj_angle[4] > thrust_max):
                raise Exception('no valid thrust - error:',  traj_angle[4])
            new_msg.data = drone_step
            self.close_loop_control_publisher_.publish(new_msg)
            await drone.offboard.set_attitude(Attitude(drone_step[0], drone_step[1], drone_step[2], drone_step[3]))
            counter_step += 1
            
            
                      

def error_control(acceleraion_desgired, acceleraion_measured):
    error = np.empty([1,3])
    error[0,0] = acceleraion_desgired[0] - acceleraion_measured[0]
    error[0,1] = acceleraion_desgired[1] - acceleraion_measured[1]
    error[0,2] = acceleraion_desgired[2] - acceleraion_measured[2]
    error_list = [error[0,0],error[0,1],error[0,2]]
    return da_controller(error_list, acceleraion_desgired)

def da_controller(error_list, acceleraion_desgired):
    global counter_number
    beta = 160
    alpha = 8 #same as the mass
    time = 0.022 #time between measurements
    Z_r = [0, 0, 1]
    F_r_x = alpha * acceleraion_desgired[0] + beta * time * error_list[0] 
    F_r_y = alpha * acceleraion_desgired[1] + beta * time * error_list[1] 
    F_r_z = alpha * acceleraion_desgired[2] + beta * time * error_list[2] 
    F_r = [F_r_x, F_r_y, F_r_z]
    f_r = Z_r[0] * F_r[0] + Z_r[1] * F_r[1] + Z_r[2] * F_r[2] 
    array_F_r = np.array([Z_r[0] * F_r[0], Z_r[1] * F_r[1], Z_r[2] * F_r[2]])
    norma_F_r = norm(array_F_r, 2)
    z_r = [F_r[0] / norma_F_r, F_r[1] / norma_F_r, F_r[2] / norma_F_r]
    result = [f_r, z_r[0], z_r[1], z_r[2]]
    if (z_r[0] < 0.1 and z_r[0] > -0.1):
         z_r[0] = 0
    if (z_r[1] < 0.1 and z_r[1] > -0.1):
          z_r[1] = 0 
    angle_type = None                        
    if (z_r[0] > 0 or z_r[0] < 0) and (z_r[1] == 0): 
        angle_type = 'pitch'
    elif (z_r[1] > 0 or z_r[1] < 0) and (z_r[0] == 0): 
        angle_type = 'roll'    
    else: 
        angle_type = 'no angle' 
        angle = 0    
    dot_product = np.dot(np.array(Z_r), np.array(z_r))
    norma_Z_r = norm(np.array(Z_r), 2)
    norma_z_r = norm(np.array(z_r), 2)
    if angle_type == 'pitch':
         if z_r[0] >= 0:
            angle = np.degrees(np.arccos(dot_product/(norma_Z_r * norma_z_r)))
         else:
             angle = -np.degrees(np.arccos(dot_product/(norma_Z_r * norma_z_r)))
    if angle_type == 'roll':
        if z_r[1] >= 0:
            angle = np.degrees(np.arccos(dot_product/(norma_Z_r * norma_z_r)))
        else:
             angle = -np.degrees(np.arccos(dot_product/(norma_Z_r * norma_z_r)))
    
    result = [angle_type, angle ,f_r - 1]
    counter_number += 1
    return result


async def run():
    await drone.connect(system_address="udp://:14540")
    
    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"-- Connected to drone!")
            break

    print("-- Arming")
    await drone.action.arm()

    print("-- Setting initial set point step - (0.0, 0.0, 0.0, 0.0)")
    await drone.offboard.set_attitude(Attitude(0.0, 0.0, 0.0, 0.0))


    print("-- Starting offboard")
    try:
        await drone.offboard.start()
    except OffboardError as error:
        print(f"Starting offboard mode failed with error code: \
            {error._result.result}")
        print("-- Disarming")
        await drone.action.disarm()
        return


async def spin_once(number_publisher_node):
    rclpy.spin_once(number_publisher_node, timeout_sec=0)

async def run_ros2(number_publisher_node):
    while True:
        await spin_once(number_publisher_node)
        await asyncio.sleep(0.022)

async def main(args=None):
    global drone
    global steps
    global counter_number
    global drone_step
    global counter_step 
    counter_number = 0 
    counter_step = 1
    drone = System()
    drone_step = [0.0, 0.0, 0.0, 0.0]
    steps = 0
    rclpy.init(args=args)
    number_publisher_node = NumberPublisherNode()
    task_ros2 = asyncio.create_task(run_ros2(number_publisher_node))
    task_mavsdk = asyncio.create_task(run())
    await asyncio.gather(task_ros2, task_mavsdk)
    number_publisher_node.destroy_node()
    rclpy.shutdown()    


    
if __name__ == "__main__":
    # Start the main function
    asyncio.run(main())
    # Runs the event loop until the program is canceled with e.g. CTRL-C
    asyncio.get_event_loop().run_forever()
