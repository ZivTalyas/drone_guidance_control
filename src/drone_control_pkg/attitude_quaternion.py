#!/usr/bin/env python3

import asyncio
import rclpy
from std_msgs.msg import Float32MultiArray
from arena_common_interfaces.msg import ArenaPawnState # For subscription
# Note: mavsdk.System is not directly used in the node's logic after refactoring,
# but DroneConnection handles it.
# from mavsdk import System

from drone_utils import RosNode, DroneConnection # Import utilities

class AttitudeQuaternionNode(RosNode):
    """
    ROS2 node that subscribes to ArenaPawnState to get Euler angles (roll, pitch, yaw)
    and publishes them as a Float32MultiArray.
    Despite the name "AttitudeQuaternionNode" (matching the file name),
    it currently processes and publishes Euler angles.
    The MAVSDK drone connection is established, but the drone object itself
    is not directly used by this node's periodic tasks after initialization.
    """
    def __init__(self):
        """
        Initializes the AttitudeQuaternionNode.
        """
        super().__init__("attitude_quaternion_node") # Renamed node
        self.euler_angles = [0.0, 0.0, 0.0] # Stores roll, pitch, yaw

        # Subscription to ArenaPawnState to get attitude data
        self.create_subscription(
            ArenaPawnState,
            'multicopter001/arena/state', # Topic name from original code
            self.arena_state_callback,
            10)

        # Publisher for the Euler angles
        # Topic name "attitude_quaternion" is kept from original for consistency,
        # but it carries Euler angles.
        self.publisher_ = self.create_publisher(
            Float32MultiArray,
            "attitude_quaternion",
            10)

        # Timer to periodically publish the received Euler angles
        self.publish_timer_ = self.create_timer(0.022, self.publish_euler_angles)

        self.get_logger().info("AttitudeQuaternionNode has been started and is subscribing to ArenaPawnState.")

    def set_drone(self, drone_obj):
        """
        Sets the drone object. Currently not used by this node's periodic tasks
        but available for future MAVSDK interactions if needed.

        Args:
            drone_obj (mavsdk.System): The connected MAVSDK drone object.
        """
        # self.drone = drone_obj # Store if needed, but it's not used right now.
        self.get_logger().info("Drone object set in AttitudeQuaternionNode (though not actively used in callbacks).")

    def arena_state_callback(self, msg):
        """
        Callback for the ArenaPawnState subscription.
        Updates the stored Euler angles.
        """
        self.euler_angles = [msg.rotation.roll, msg.rotation.pitch, msg.rotation.yaw]
        # self.get_logger().debug(f"Received Euler angles: {self.euler_angles}")

    def publish_euler_angles(self):
        """
        Called by the timer to publish the currently stored Euler angles.
        """
        new_msg = Float32MultiArray()
        new_msg.data = self.euler_angles
        self.publisher_.publish(new_msg)
        # self.get_logger().debug(f"Published Euler angles: {self.euler_angles}")


async def main(args=None):
    """
    Main function to initialize and run the ROS2 node and MAVSDK communication.
    """
    rclpy.init(args=args)

    attitude_node = AttitudeQuaternionNode()
    # DroneConnection is instantiated to ensure drone is connected and armed,
    # as this might be a system-level requirement even if this node doesn't directly use the drone object.
    drone_connection = DroneConnection()

    loop = asyncio.get_event_loop()
    try:
        # Connect and arm the drone.
        # The drone object returned by connect_and_arm isn't strictly needed by attitude_node's
        # current logic, but we call set_drone for completeness / future use.
        drone_obj = loop.run_until_complete(drone_connection.connect_and_arm())
        attitude_node.set_drone(drone_obj) # Node has access if needed

        rclpy.spin(attitude_node)

    except ConnectionError as e:
        attitude_node.get_logger().error(f"MAVSDK Connection error: {e}")
    except KeyboardInterrupt:
        attitude_node.get_logger().info("User interrupted, shutting down.")
    except Exception as e:
        attitude_node.get_logger().error(f"An unexpected error occurred in main: {e}")
    finally:
        attitude_node.get_logger().info("Performing cleanup...")
        if drone_connection.drone and loop.is_running() and \
           loop.run_until_complete(drone_connection.drone.core.is_connected()):
            loop.run_until_complete(drone_connection.disarm_and_land())

        if not attitude_node._destroyed:
            attitude_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    asyncio.run(main())
