#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
import random # For unused trajectory functions
from std_msgs.msg import Float32MultiArray
import time

from drone_utils import RosNode # Using RosNode base class

# Configuration for the trajectory segments
# Each segment has a duration and the acceleration vector [x, y, z]
# The last segment's duration can be float('inf') to hold indefinitely.
TRAJECTORY_SEGMENTS = [
    {"name": "Phase 0: Initial Ascent", "duration": 20.0, "acceleration": [0.0, 0.0, 18.0]},
    {"name": "Phase 1: Forward Acceleration", "duration": 12.0, "acceleration": [1.5, 0.0, 11.0]},
    {"name": "Phase 2: Hold Z Acceleration", "duration": 10.0, "acceleration": [0.0, 0.0, 18.0]},
    {"name": "Phase 3: Rightward Acceleration", "duration": 12.0, "acceleration": [0.0, 1.5, 11.0]},
    {"name": "Phase 4: Hold Z Acceleration", "duration": float('inf'), "acceleration": [0.0, 0.0, 18.0]},
]

class TrajectoryMakerNode(RosNode):
    """
    ROS2 node that publishes a time-based trajectory of desired acceleration vectors.
    The trajectory is defined by a sequence of segments, each with a duration
    and an acceleration vector.
    """
    def __init__(self):
        """
        Initializes the TrajectoryMakerNode.
        """
        super().__init__("trajectory_maker_node") # Renamed node

        self.current_acceleration = [0.0, 0.0, 0.0] # Current acceleration to publish
        self.publisher_ = self.create_publisher(Float32MultiArray, "trajectory_maker", 10)

        self.start_time = time.time() # Records when the node (and trajectory) starts
        self.current_segment_index = 0
        self.time_in_current_segment = 0.0

        # Initialize with the first segment's acceleration
        if TRAJECTORY_SEGMENTS:
            self.current_acceleration = TRAJECTORY_SEGMENTS[0]["acceleration"]

        self.timer_ = self.create_timer(0.022, self.publish_trajectory_point) # Timer period from original
        self.get_logger().info(f"TrajectoryMakerNode started. Publishing trajectory based on {len(TRAJECTORY_SEGMENTS)} segments.")

    def _get_acceleration_for_elapsed_time(self, elapsed_time_total):
        """
        Determines the target acceleration vector based on elapsed time and trajectory segments.

        Args:
            elapsed_time_total (float): Total time elapsed since the trajectory started.

        Returns:
            list[float]: The target acceleration vector [x, y, z].
                         Returns the first segment's acceleration if TRAJECTORY_SEGMENTS is empty
                         or if time is negative (though time should always be positive).
        """
        if not TRAJECTORY_SEGMENTS:
            # Should not happen if constructor initializes from it, but as a safeguard:
            return self.current_acceleration

        time_cursor = 0.0
        selected_acceleration = TRAJECTORY_SEGMENTS[0]["acceleration"] # Default to first
        new_segment_index = 0

        for i, segment in enumerate(TRAJECTORY_SEGMENTS):
            if elapsed_time_total < time_cursor + segment["duration"]:
                selected_acceleration = segment["acceleration"]
                new_segment_index = i
                break
            time_cursor += segment["duration"]
            # If loop finishes, it means elapsed_time is beyond all defined finite durations
            # The last segment (potentially with inf duration) will be selected by default if not broken earlier
            selected_acceleration = segment["acceleration"] # Keep updating to last processed segment
            new_segment_index = i


        if self.current_segment_index != new_segment_index:
            self.get_logger().info(f"Entering trajectory segment: {TRAJECTORY_SEGMENTS[new_segment_index]['name']} "
                                   f"(Time: {elapsed_time_total:.2f}s, Accel: {selected_acceleration})")
            self.current_segment_index = new_segment_index

        return selected_acceleration

    def publish_trajectory_point(self):
        """
        Called by the timer to determine and publish the current desired acceleration
        based on the predefined trajectory segments and elapsed time.
        """
        elapsed_time_total = time.time() - self.start_time
        self.current_acceleration = self._get_acceleration_for_elapsed_time(elapsed_time_total)

        msg = Float32MultiArray()
        msg.data = self.current_acceleration
        self.publisher_.publish(msg)
        # self.get_logger().debug(f"Published Accel: {self.current_acceleration} at time {elapsed_time_total:.2f}s")

# --- Unused alternative trajectory generation functions (kept from original for reference) ---
# These functions are not currently used by the TrajectoryMakerNode.
def step_trajectory():
    """Generates a random step trajectory component."""
    choose_option = random.choice([1,2])
    if choose_option == 1:
       trajectory_maker_ = [random.uniform(-3.0, 3.0), 0.0, 12.0]
    if choose_option == 2:
        trajectory_maker_ = [0.0, random.uniform(-3.0, 3.0), 12.0]
    return trajectory_maker_

def cos_trajectory():
    """Generates a random cosine-based trajectory component."""
    choose_option = random.choice([1,2])
    if choose_option == 1:
       trajectory_maker_ = [3.0*np.cos(random.uniform(-np.pi, np.pi)), 0.0, 12.0]
    if choose_option == 2:
        trajectory_maker_ = [0.0, 3.0*np.cos(random.uniform(-np.pi, np.pi)), 12.0]
    return trajectory_maker_

def sin_trajectory():
    """Generates a random sine-based trajectory component."""
    choose_option = random.choice([1,2])
    if choose_option == 1:
       trajectory_maker_ = [3.0*np.sin(random.uniform(-np.pi, np.pi)), 0.0, 12.0]
    if choose_option == 2:
        trajectory_maker_ = [0.0, 3.0*np.sin(random.uniform(-np.pi, np.pi)), 12.0]
    return trajectory_maker_
# --- End of unused functions ---

def main(args=None):
    """
    Main function to initialize and run the TrajectoryMakerNode.
    """
    rclpy.init(args=args)
    node = TrajectoryMakerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("TrajectoryMakerNode interrupted by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()