#!/usr/bin/env python3

import asyncio
from mavsdk import System
import rclpy
from rclpy.node import Node
import numpy as np
from mavsdk.offboard import (Attitude, OffboardError)
import rospy
from sensor_msgs.msg import Imu



async def run():
    # Init the drone
    drone = System()
    await drone.connect(system_address="udp://:14540")
    
    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"-- Connected to drone!")
            break

    print("-- Arming")
    await drone.action.arm()

   
    # Start the tasks
    #asyncio.ensure_future(print_imu(drone))
    
    
    print("-- Setting initial setpoint")
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

    print("-- Go up at 70% thrust")
    await drone.offboard.set_attitude(Attitude(0.0, 0.0, 0.0, 0.7))
    #print_imu(drone)
    await asyncio.sleep(20)
    

    print("-- Roll 30 at 60% thrust")
    await drone.offboard.set_attitude(Attitude(10.0, 0.0, 0.0, 0.6))
    #print_imu(drone)
    await asyncio.sleep(20)

    print("-- Roll -30 at 60% thrust")
    await drone.offboard.set_attitude(Attitude(-10.0, 0.0, 0.0, 0.6))
    #print_imu(drone)
    await asyncio.sleep(20)

    print("-- Hover at 60% thrust")
    await drone.offboard.set_attitude(Attitude(0.0, 0.0, 0.0, 0.6))
    #print_imu(drone)
    await asyncio.sleep(20)

    print("-- Stopping offboard")
    try:
        await drone.offboard.stop()
    except OffboardError as error:
        print(f"Stopping offboard mode failed with error code: \
              {error._result.result}")

    await drone.action.land()


#async def print_imu(drone):
    #async for imu in drone.telemetry.subscribe_scaled_imu(): 
        #acceleration_x = imu.acceleration_frd.forward_m_s2
        #acceleration_y = imu.acceleration_frd.right_m_s2
        #acceleration_z = imu.acceleration_frd.down_m_s2
        #angular_velocity__x = imu.angular_velocity_frd.forward_rad_s
        #angular_velocity_y = imu.angular_velocity_frd.right_rad_s
        #angular_velocity_z = imu.angular_velocity_frd.down_rad_s
        #imu_matrix = np.c_[acceleration_x, acceleration_y, acceleration_z, angular_velocity__x, angular_velocity_y,angular_velocity_z]
       
        #print(imu_matrix)
 # Initialize the node
rospy.init_node("scaled_imu_node")

def scaled_imu_callback(msg):
    # This function will be called every time the scaled IMU data is updated
    print(f"Acceleration: {msg.linear_acceleration}")
    print(f"Angular velocity: {msg.angular_velocity}")
    print(f"Quaternion: {msg.orientation}")

# Subscribe to the scaled_imu topic
sub = rospy.Subscriber("scaled_imu", Imu, scaled_imu_callback)

# Spin to keep the node alive
rospy.spin()       


if __name__ == "__main__":
    # Start the main function
    asyncio.ensure_future(run())
    # Runs the event loop until the program is canceled with e.g. CTRL-C
    asyncio.get_event_loop().run_forever()