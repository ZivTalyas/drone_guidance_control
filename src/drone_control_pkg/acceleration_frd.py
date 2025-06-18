#!/usr/bin/env python3

import asyncio
import rclpy
import numpy as np
import math
from std_msgs.msg import Float32MultiArray
import time

from drone_utils import RosNode, DroneConnection # Import utilities

# Define named constant for yaw angle
PSI_RAD = math.radians(-57.2957795)

class AccelerationFRDNode(RosNode):
    """
    ROS2 node to read IMU data from a drone, perform coordinate transformation,
    and publish the acceleration in the FRD frame.
    It also saves the acceleration data to a file.
    """
    def __init__(self):
        """
        Initializes the AccelerationFRDNode.
        """
        super().__init__("acceleration_frd_node")
        self.drone = None # Will be set by main_runner
        self.acceleration_value = [0.0, 0.0, 0.0] # FRD acceleration values
        self.acceleration_final_measure = np.array([0.0, 0.0, 0.0]) # To store all measurements
        self.save_data_flag = True # Flag to control saving data once
        self.start_time = time.time()

        self.publisher_ = self.create_publisher(Float32MultiArray, "acceleration_frd", 10)
        # The timer will now call a method that includes the MAVSDK logic
        self.timer_ = self.create_timer(0.022, self.process_and_publish_acceleration)
        self.get_logger().info("AccelerationFRDNode publisher has been started.")

    def set_drone(self, drone_obj):
        """
        Sets the drone object for MAVSDK communication.

        Args:
            drone_obj (mavsdk.System): The connected MAVSDK drone object.
        """
        self.drone = drone_obj
        self.get_logger().info("Drone object set in AccelerationFRDNode.")

    async def _get_imu_snapshot(self):
        """
        Asynchronously fetches a single IMU data snapshot from the drone.

        Returns:
            mavsdk.telemetry.Imu: IMU data object, or None if error/timeout.
        """
        if not self.drone:
            self.get_logger().warn("Drone object not yet set. Cannot get IMU data.")
            return None
        try:
            imu_data = await asyncio.wait_for(self.drone.telemetry.imu().__anext__(), timeout=1.0)
            return imu_data
        except asyncio.TimeoutError:
            self.get_logger().warn("Timeout waiting for IMU data snapshot.")
            return None
        except Exception as e:
            self.get_logger().error(f"Error fetching IMU data snapshot: {e}")
            return None

    def _transform_acceleration(self, acc_f, acc_r, acc_d):
        """
        Transforms acceleration from body FRD frame to a world-like frame based on
        derived roll/pitch and a fixed yaw.

        Args:
            acc_f (float): Forward acceleration.
            acc_r (float): Right acceleration.
            acc_d (float): Down acceleration.

        Returns:
            list[float]: Transformed acceleration [x, y, z].
        """
        # TODO: Critical Review Needed for Transformation:
        # The current transformation derives roll (theta) and pitch (phi) angles
        # directly from acceleration components. This is non-standard.
        # For accurate transformation from body (FRD) to world (e.g., NED or ENU) frame,
        # the drone's actual attitude (roll, pitch, yaw from telemetry.attitude_euler()
        # or telemetry.attitude_quaternion()) should be used to construct the rotation matrix.
        # The current PSI_RAD is also a fixed yaw, which might not reflect the drone's actual heading.
        # This implementation retains the original logic for structural refactoring,
        # but it needs to be replaced with a proper attitude-based transformation for correctness.

        theta = math.atan2(acc_r, math.sqrt(acc_f**2 + acc_d**2)) # Roll-like angle from accel
        phi = math.atan2(acc_f, math.sqrt(acc_r**2 + acc_d**2))   # Pitch-like angle from accel
        psi = PSI_RAD # Use the defined constant for yaw

        # Rotation matrix from body to world (based on derived angles)
        R = np.array([
            [math.cos(psi) * math.cos(theta),
             math.cos(psi) * math.sin(phi) * math.sin(theta) - math.sin(psi) * math.cos(phi),
             math.cos(psi) * math.cos(phi) * math.sin(theta) + math.sin(psi) * math.sin(phi)],
            [math.sin(psi) * math.cos(theta),
             math.sin(psi) * math.sin(phi) * math.sin(theta) + math.cos(psi) * math.cos(phi),
             math.sin(psi) * math.cos(phi) * math.sin(theta) - math.cos(psi) * math.sin(phi)],
            [-math.sin(theta),
             math.cos(theta) * math.sin(phi),
             math.cos(theta) * math.cos(phi)]
        ])

        a_body = np.array([[acc_f], [acc_r], [acc_d]])

        # Assuming G (gravity vector in world frame) is [0,0,0] as per original logic.
        # This implies acc_f, acc_r, acc_d are either already gravity-compensated
        # or the transformation is to a frame where gravity's effect is ignored/handled differently.
        G = np.array([[0], [0], [0]])
        a_world = np.dot(R, a_body) + G

        # Transform to the specific output format [world_x, -world_y, -world_z]
        return [a_world[0,0], -a_world[1,0], -a_world[2,0]]

    async def _process_imu_data(self):
        """
        Reads IMU data, performs coordinate transformation, and updates acceleration_value.
        This is the primary method called by the timer.
        """
        imu_data = await self._get_imu_snapshot()
        if not imu_data:
            return

        acc_f = imu_data.acceleration_frd.forward_m_s2
        acc_r = imu_data.acceleration_frd.right_m_s2
        acc_d = imu_data.acceleration_frd.down_m_s2

        self.acceleration_value = self._transform_acceleration(acc_f, acc_r, acc_d)

        # Store data for saving
        current_measurement = np.array(self.acceleration_value)
        self.acceleration_final_measure = np.vstack((self.acceleration_final_measure, current_measurement))


    async def process_and_publish_acceleration(self):
        """
        Called by the ROS timer. Reads IMU, transforms, publishes, and handles data saving.
        """
        await self._process_imu_data()

        new_msg = Float32MultiArray()
        new_msg.data = self.acceleration_value
        self.publisher_.publish(new_msg)
        # self.get_logger().info(f"Published acceleration: {self.acceleration_value}")

        # Data saving logic from original run()
        elapsed_time = time.time() - self.start_time
        if elapsed_time > 60 and self.save_data_flag:
            self.get_logger().info("60 seconds elapsed. Saving acceleration data...")
            try:
                np.savetxt("acceleration_measure.txt", self.acceleration_final_measure)
                self.get_logger().info("Acceleration data saved to acceleration_measure.txt")
            except Exception as e:
                self.get_logger().error(f"Failed to save data: {e}")
            self.save_data_flag = False # Prevent saving multiple times
            # Optionally, you could stop the node or drone here if the task is complete.
            # For example:
            # self.get_logger().info("Data collection complete, shutting down.")
            # rclpy.shutdown()


async def main(args=None):
    """
    Main function to initialize and run the ROS2 node and MAVSDK communication.
    """
    rclpy.init(args=args)

    drone_node = AccelerationFRDNode()
    drone_connection = DroneConnection() # Using default UDP address

    # We need a way to run the ROS spinning and the MAVSDK connection concurrently.
    # The drone_utils.main_runner is one way, or we can manage asyncio loop here.

    loop = asyncio.get_event_loop()
    try:
        # Connect and arm the drone first
        drone = loop.run_until_complete(drone_connection.connect_and_arm())
        drone_node.set_drone(drone) # Pass the connected drone to the node

        # Now that the drone is connected and set in the node, we can spin the node.
        # The node's timer will handle periodic MAVSDK calls.
        rclpy.spin(drone_node)

    except ConnectionError as e:
        drone_node.get_logger().error(f"MAVSDK Connection error: {e}")
    except KeyboardInterrupt:
        drone_node.get_logger().info("User interrupted, shutting down.")
    except Exception as e:
        drone_node.get_logger().error(f"An unexpected error occurred in main: {e}")
    finally:
        drone_node.get_logger().info("Performing cleanup...")
        if drone_connection.drone and loop.is_running() and \
           loop.run_until_complete(drone_connection.drone.core.is_connected()):
            loop.run_until_complete(drone_connection.disarm_and_land())

        if not drone_node._destroyed: # Check if node not already destroyed
            drone_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        # loop.close() # Closing loop can be problematic if other async tasks are managed by rclpy

if __name__ == "__main__":
    asyncio.run(main())
    # No need for get_event_loop().run_forever() as main now handles the lifecycle.
