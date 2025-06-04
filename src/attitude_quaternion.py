#!/usr/bin/env python3

import asyncio
from mavsdk import System
import time
import rclpy
from rclpy.node import Node
import numpy as np
from mavsdk.offboard import (Attitude, OffboardError)
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion


from arena_common_interfaces.msg import ArenaPawnState
from arena_common_interfaces.msg import Rotator
# from arena_common_interfaces.msg._rotator  
from std_msgs.msg import Float32MultiArray
import time



class NumberPublisherNode(Node):
    def __init__(self):
        super().__init__("attitude_quaternion")
        self.imu_attitude = [0.0,0.0,0.0]
        self.subscribe_imu = self.create_subscription(ArenaPawnState,'multicopter001/arena/state',self.imu_data,10)
        self.attitude_quaternion_publisher_ = self.create_publisher(Float32MultiArray, "attitude_quaternion", 10)
        self.angles_maker_timer_ = self.create_timer(0.022, self.attitude_quaternion)
        self.get_logger().info("attitude quaternion publisher has been started.")

    def imu_data(self,msg):
        self.imu_attitude = [msg.rotation.roll,msg.rotation.pitch,msg.rotation.yaw]
        # if (1 != 0):
        #     raise Exception(self.imu_attitude)

    async def attitude_quaternion(self):
        # time.sleep(1)    
        new_msg = Float32MultiArray()         
        new_msg.data = self.imu_attitude
        self.attitude_quaternion_publisher_.publish(new_msg)    


async def run():
    global quaternion_value
    # Init the drone
    await drone.connect(system_address="udp://:14540")   
    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"-- Connected to drone!")
            break
    print("-- Arming")
    await drone.action.arm()
    # start_time = time.time()
    # for i in range(10000):
    #     quaternion_value = await attitude_quaternion() ### list of quaternion values from the IMU ###
    #     end_time = time.time()
    #     if (end_time - start_time > 60):
    #         break   


async def spin_once(number_publisher_node):
    rclpy.spin_once(number_publisher_node, timeout_sec=0)

async def run_ros2(number_publisher_node):
    while True:
        await spin_once(number_publisher_node)
        await asyncio.sleep(0.022)

######## main function which make ROS2 and mavsdk communication to run parallel #########
async def main(args=None):
    global quaternion_value
    global drone 
    drone = System()
    quaternion_value = [0.0, 0.0, 0.0, 0.0] 
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
