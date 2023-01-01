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
    #asyncio.ensure_future(thrust(drone))
    
    
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
    print_imu(drone)
    #thrust(drone)
    await asyncio.sleep(20)
    

    print("-- Roll 30 at 60% thrust")
    await drone.offboard.set_attitude(Attitude(10.0, 0.0, 0.0, 0.6))
    print_imu(drone)
    #thrust(drone)
    await asyncio.sleep(20)

    print("-- Roll -30 at 60% thrust")
    await drone.offboard.set_attitude(Attitude(-10.0, 0.0, 0.0, 0.6))
    print_imu(drone)
    #thrust(drone)
    await asyncio.sleep(20)

    print("-- Hover at 60% thrust")
    await drone.offboard.set_attitude(Attitude(0.0, 0.0, 0.0, 0.6))
    print_imu(drone)
    #thrust(drone)
    await asyncio.sleep(20)

    print("-- Stopping offboard")
    try:
        await drone.offboard.stop()
    except OffboardError as error:
        print(f"Stoppinprint_img offboard mode failed with error code: \
              {error._result.result}")

    await drone.action.land()


async def print_imu(drone):
    async for attitude_quaternion in drone.telemetry.attitude_quaternion(): 
        q = attitude_quaternion
        print(q)


if __name__ == "__main__":
    # Start the main function
    asyncio.ensure_future(run())
    # Runs the event loop until the program is canceled with e.g. CTRL-C
    asyncio.get_event_loop().run_forever()