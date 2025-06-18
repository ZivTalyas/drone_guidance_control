#!/usr/bin/env python3

import asyncio
from mavsdk import System
import rclpy
from rclpy.node import Node
import numpy as np
from mavsdk.offboard import (Attitude, OffboardError)
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3

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

    print("-- Setting initial setpoint")
    await drone.offboard.set_attitude(Attitude(0.0, 0.0, 0.0, 0.0))

    current_yaw = await get_yaw_from_quaternion(drone)
    print(f"Initial Yaw: {current_yaw:.2f} degrees")

    print("-- Starting offboard")
    try:
        await drone.offboard.start()
    except OffboardError as error:
        print(f"Starting offboard mode failed with error code: {error._result.result}")
        print("-- Disarming")
        await drone.action.disarm()
        return

    print("-- Go up at full thrust (intended: 1.0, was 70% in comment)")
    await drone.offboard.set_attitude(Attitude(0.0, 0.0, 0.0, 1.0))
    current_yaw = await get_yaw_from_quaternion(drone)
    print(f"Yaw during ascent: {current_yaw:.2f} degrees")

    # Loop for data gathering (simplified from original)
    # The original loop called imu_quaternion 750 times, which is slow if each call re-iterates.
    # If continuous data is needed, it's better to stream it.
    # For this tuning script, getting a few points or short averages might be more practical.
    # Here, we'll just print yaw periodically.
    print("-- Monitoring yaw for a few seconds...")
    for i in range(10): # Approx 10 seconds if get_yaw_from_quaternion takes ~1s due to __anext__
        current_yaw = await get_yaw_from_quaternion(drone)
        print(f"Loop {i+1}/10: Current Yaw: {current_yaw:.2f} degrees")
        # await asyncio.sleep(1) # Removed, as get_yaw_from_quaternion itself will introduce some delay.
                               # Add back if faster periodic sampling is needed with actual sleep.

    await asyncio.sleep(5) # Original had sleep(10)

    print("-- Set Attitude: Roll 30 deg, Pitch 10 deg, Full Thrust")
    await drone.offboard.set_attitude(Attitude(30.0, 10.0, 0.0, 1.0))
    current_yaw = await get_yaw_from_quaternion(drone)
    print(f"Yaw after roll/pitch command: {current_yaw:.2f} degrees")
    await asyncio.sleep(10) # Original had sleep(20)

    print("-- Set Attitude: Roll 30 deg, Pitch 0 deg, 60% Thrust (Hover-like)")
    await drone.offboard.set_attitude(Attitude(30.0, 0.0, 0.0, 0.6))
    current_yaw = await get_yaw_from_quaternion(drone)
    print(f"Yaw at 30 deg roll, 60% thrust: {current_yaw:.2f} degrees")
    await asyncio.sleep(10) # Original had sleep(20)

    print("-- Stopping offboard")
    try:
        await drone.offboard.stop()
    except OffboardError as error:
        print(f"Stopping offboard mode failed with error code: {error._result.result}")

    print("-- Landing")
    await drone.action.land()
    print("Tune script finished.")


async def get_yaw_from_quaternion(drone):
    """
    Fetches a single attitude quaternion snapshot from the drone
    and converts it to Yaw in degrees.
    """
    try:
        attitude_q = await drone.telemetry.attitude_quaternion().__anext__()
        # Convert quaternion to Yaw (degrees)
        # ZYX Euler convention (Yaw around Z, Pitch around Y, Roll around X)
        # sin(yaw)cos(pitch) = 2 * (w*z + x*y)
        # cos(yaw)cos(pitch) = 1 - 2 * (y^2 + z^2)
        siny_cosp = 2.0 * (attitude_q.w * attitude_q.z + attitude_q.x * attitude_q.y)
        cosy_cosp = 1.0 - 2.0 * (attitude_q.y * attitude_q.y + attitude_q.z * attitude_q.z)
        yaw_rad = np.arctan2(siny_cosp, cosy_cosp)
        return np.degrees(yaw_rad)
    except StopAsyncIteration:
        print("Failed to get attitude quaternion data: telemetry stream ended.")
        return None
    except Exception as e:
        print(f"Error getting attitude quaternion: {e}")
        return None

def main(args=None):
    """
    Main entry point for the acceleration tuning script.
    This allows it to be launched as a ROS2 executable if desired,
    though it doesn't use rclpy itself.
    """
    print("Starting acceleration_tune_node script...")
    # This script is pure asyncio MAVSDK, no rclpy.init(args=args) needed here.
    asyncio.run(run())
    print("acceleration_tune_node script finished.")

if __name__ == "__main__":
    # asyncio.run(run()) # Keep the direct asyncio.run for calling script directly
    main() # Allow calling as module with main()