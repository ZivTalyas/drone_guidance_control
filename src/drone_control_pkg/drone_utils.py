#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from mavsdk import System
import asyncio

class DroneConnection:
    """
    A class to manage the MAVSDK connection and arming of a drone.
    """
    def __init__(self, drone_url='udp://:14540'):
        """
        Initializes the DroneConnection.

        Args:
            drone_url (str): The URL for connecting to the drone.
        """
        self.drone_url = drone_url
        self.drone = System()

    async def connect_and_arm(self):
        """
        Connects to the drone and arms it.

        Returns:
            mavsdk.System: The connected and armed drone object.

        Raises:
            ConnectionError: If connection or arming fails.
        """
        print(f"Connecting to drone at {self.drone_url}...")
        await self.drone.connect(system_address=self.drone_url)

        print("Waiting for drone to connect...")
        async for state in self.drone.core.connection_state():
            if state.is_connected:
                print("Drone connected!")
                break

        print("Waiting for drone to have a global position estimate...")
        async for health in self.drone.telemetry.health():
            if health.is_global_position_ok and health.is_home_position_ok:
                print("Global position estimate OK")
                break

        print("-- Arming")
        try:
            await self.drone.action.arm()
            print("Drone armed.")
        except Exception as e:
            raise ConnectionError(f"Failed to arm the drone: {e}")

        return self.drone

    async def disarm_and_land(self):
        """
        Disarms the drone and lands it.
        """
        print("-- Disarming")
        await self.drone.action.disarm()
        print("Drone disarmed.")
        # Note: Actual landing logic might be more complex and involve checking altitude, etc.
        # This is a simplified version.
        try:
            print("-- Landing")
            await self.drone.action.land()
            print("Drone landing.")
        except Exception as e:
            print(f"Landing failed: {e}")


class RosNode(Node):
    """
    A base class for ROS2 nodes.
    """
    def __init__(self, node_name):
        """
        Initializes the ROS2 node.

        Args:
            node_name (str): The name of the ROS2 node.
        """
        super().__init__(node_name)
        self.get_logger().info(f"{node_name} started.")

def run_ros_node(node_class, *args, **kwargs):
    """
    Initializes and spins a ROS2 node.

    Args:
        node_class: The class of the ROS2 node to run.
        *args: Positional arguments to pass to the node constructor.
        **kwargs: Keyword arguments to pass to the node constructor.
    """
    rclpy.init()
    node = node_class(*args, **kwargs)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt, shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

async def main_runner(node_instance, drone_connection):
    """
    A generic runner for nodes that need to connect to the drone
    and then perform some actions.

    Args:
        node_instance (RosNode): An instance of a ROS2 node.
        drone_connection (DroneConnection): An instance of DroneConnection.
    """
    try:
        drone = await drone_connection.connect_and_arm()
        node_instance.set_drone(drone) # Assumes node has a set_drone method

        # Placeholder for the node's main logic
        # For example, you might call a method on the node_instance
        # that starts its primary operations.
        # await node_instance.run_main_logic()

        # Keep the node alive until shutdown or specific condition
        # This part needs to be tailored to how your node operates.
        # If rclpy.spin is used elsewhere, this might not be needed
        # or might need to be integrated with the ROS spin loop.
        while rclpy.ok():
            await asyncio.sleep(0.1)

    except ConnectionError as e:
        node_instance.get_logger().error(f"Connection error: {e}")
    except Exception as e:
        node_instance.get_logger().error(f"An unexpected error occurred: {e}")
    finally:
        if drone_connection.drone and await drone_connection.drone.core.is_connected():
            await drone_connection.disarm_and_land()
        node_instance.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    # This is an example of how you might use these utilities.
    # You would typically call this from your specific node's main script.

    # Example Node (replace with your actual node)
    class MyDroneNode(RosNode):
        def __init__(self):
            super().__init__('my_drone_node')
            self.drone = None

        def set_drone(self, drone_obj):
            self.drone = drone_obj
            self.get_logger().info("Drone object set in node.")

        async def run_main_logic(self):
            # Your node's specific logic here
            self.get_logger().info("Running main logic of MyDroneNode.")
            # Example: subscribe to telemetry, publish commands, etc.
            await asyncio.sleep(10) # Keep running for 10 seconds
            self.get_logger().info("Main logic finished.")

    # --- Main execution part ---
    # This part would usually be in your specific node's executable script.

    # rclpy.init() # Initialize rclpy once
    # drone_node = MyDroneNode()
    # drone_connector = DroneConnection()

    # loop = asyncio.get_event_loop()
    # try:
    #     # Run the connection and main logic
    #     loop.run_until_complete(main_runner(drone_node, drone_connector))
    # except KeyboardInterrupt:
    #     drone_node.get_logger().info("Program interrupted by user.")
    # finally:
    #     # Cleanup, disarm, land, shutdown ROS
    #     if drone_connector.drone and loop.run_until_complete(drone_connector.drone.core.is_connected()):
    #         loop.run_until_complete(drone_connector.disarm_and_land())

    #     if rclpy.ok() and not drone_node._destroyed: # Check if node not already destroyed
    #         drone_node.destroy_node()
    #     if rclpy.ok():
    #         rclpy.shutdown()
    #     loop.close()
    print("drone_utils.py executed. This script is intended to be imported, not run directly.")
