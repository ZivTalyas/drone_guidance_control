#!/usr/bin/env python3

import asyncio
from mavsdk import System
import rclpy
from rclpy.node import Node
import numpy as np
from mavsdk.offboard import (Attitude, OffboardError)



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
    asyncio.ensure_future(print_imu(drone))
    #asyncio.ensure_future(print_imu_filter(drone))
    
    
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
    await drone.offboard.set_attitude(Attitude(2, 0.0, 0.0, 0.7))
    imu_raw = print_imu(drone)
    #imu_filter = print_imu_filter(drone)
    await asyncio.sleep(5)

    print("-- Stopping offboard")
    try:
        await drone.offboard.stop()
    except OffboardError as error:
        print(f"Stopping offboard mode failed with error code: \
              {error._result.result}")

    await drone.action.land()
    
    #imu = np.c_[imu_raw,imu_filter]
    mat = np.matrix(imu_raw)
    with open('for_graph', 'wb') as f:
        for line in mat:
            np.savetxt(f, line, fmt='%.5f')




async def print_imu(drone):
    async for imu in drone.telemetry.imu(): 
        #acceleration_x = imu.acceleration_frd.forward_m_s2
        #acceleration_y = imu.acceleration_frd.
        acceleration_z = np.array([0])
        angular_velocity_x = np.array([0])
        acceleration_z = np.vstack(0,imu.acceleration_frd.down_m_s2)
        angular_velocity_x = np.vstack(0,imu.angular_velocity_frd.forward_rad_s)
        #angular_velocity_y = imu.angular_velocity_frd.right_rad_s
        #angular_velocity_z = imu.angular_velocity_frd.down_rad_s
        
       
        #print(imu_matrix)
    imu_matrix = np.c_[acceleration_z, angular_velocity_x]    
    return(imu_matrix)
"""
async def print_imu_filter(drone):
    async for imu in drone.telemetry.imu.scaled_imu(): 
        #acceleration_x = imu.acceleration_frd.forward_m_s2
        #acceleration_y = imu.acceleration_frd.right_m_s2
        acceleration_z = np.array([])
        angular_velocity_x = np.array([])
        acceleration_z = np.vstack(imu.acceleration_frd.down_m_s2)
        angular_velocity_x = np.vstack(imu.angular_velocity_frd.forward_rad_s)
        #angular_velocity_y = imu.angular_velocity_frd.right_rad_s
        #angular_velocity_z = imu.angular_velocity_frd.down_rad_s
    
       #print(imu_matrix)
    imu_matrix = np.c_[acceleration_z, angular_velocity_x]   
    return(imu_matrix)"""

if __name__ == "__main__":
    # Start the main function
    asyncio.ensure_future(run())
    # Show the plot
    # Runs the event loop until the program is canceled with e.g. CTRL-C
    asyncio.get_event_loop().run_forever()
