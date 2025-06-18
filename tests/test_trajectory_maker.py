import unittest
import time # For TrajectoryMakerNode's self.start_time initialization if not mocking rclpy

# Assuming rclpy is not initialized here for pure unit testing.
# If TrajectoryMakerNode had rclpy.create_node dependent calls in __init__ beyond super(),
# we'd need to mock rclpy.
# from unittest.mock import MagicMock, patch

# rclpy = MagicMock() # Mock rclpy if it's imported and used at module level or in constructor directly

# Need to ensure drone_utils.RosNode can be instantiated or mocked if it does ROS things in constructor
# For now, assuming drone_utils.RosNode's __init__ is simple or can be bypassed if needed.
# If `super().__init__` in TrajectoryMakerNode calls rclpy methods, we need to handle that.
# A common way is to mock `rclpy.node.Node` or `RosNode`'s parent if it does ROS init.

# Let's try importing the specific things we need first.
from drone_control_pkg.trajectory_maker import TRAJECTORY_SEGMENTS, TrajectoryMakerNode

# If rclpy is initialized when `RosNode` (parent of TrajectoryMakerNode) is imported,
# tests might fail in environments where ROS context isn't available/initialized.
# For this exercise, we'll assume we can instantiate TrajectoryMakerNode for its internal logic,
# and RosNode's __init__ is benign or we can mock its dependencies if errors arise.
# A simple RosNode might just call super().__init__(node_name) on rclpy.node.Node.
# We might need to mock `rclpy.create_publisher` and `create_timer` if called in __init__.

class TestTrajectoryMaker(unittest.TestCase):

    def test_trajectory_segments_structure(self):
        """
        Tests the structure and content of TRAJECTORY_SEGMENTS.
        """
        self.assertIsInstance(TRAJECTORY_SEGMENTS, list)
        self.assertTrue(len(TRAJECTORY_SEGMENTS) > 0, "TRAJECTORY_SEGMENTS should not be empty.")

        for i, segment in enumerate(TRAJECTORY_SEGMENTS):
            self.assertIsInstance(segment, dict, f"Segment {i} is not a dictionary.")
            self.assertIn("name", segment, f"Segment {i} missing 'name'.")
            self.assertIsInstance(segment["name"], str, f"Segment {i} 'name' is not a string.")

            self.assertIn("duration", segment, f"Segment {i} missing 'duration'.")
            self.assertIsInstance(segment["duration"], (float, int), f"Segment {i} 'duration' is not a number.")
            self.assertTrue(segment["duration"] > 0, f"Segment {i} 'duration' must be positive.")

            self.assertIn("acceleration", segment, f"Segment {i} missing 'acceleration'.")
            self.assertIsInstance(segment["acceleration"], list, f"Segment {i} 'acceleration' is not a list.")
            self.assertEqual(len(segment["acceleration"]), 3, f"Segment {i} 'acceleration' must have 3 elements.")
            for val in segment["acceleration"]:
                self.assertIsInstance(val, (float, int), f"Acceleration value {val} in segment {i} is not a number.")

        # Check that the last segment has float('inf') duration if others are finite and it's meant to be the hold
        finite_durations = [s["duration"] for s in TRAJECTORY_SEGMENTS[:-1]]
        if not any(d == float('inf') for d in finite_durations): # if all but last are finite
             self.assertEqual(TRAJECTORY_SEGMENTS[-1]["duration"], float('inf'),
                             "The last segment should have an infinite duration if it's a final hold phase.")


    def test_trajectory_maker_node_initialization(self):
        """
        Tests the initial state of TrajectoryMakerNode.
        """
        # Mocking ROS2 specific calls that would happen in RosNode's __init__ or TrajectoryMakerNode's __init__
        # For this test, we are primarily interested in the non-ROS logic.
        # If `super().__init__` or publisher/timer creation requires full ROS init, this test would need more mocking.
        try:
            # Attempt to create node. This might fail if rclpy is not initialized
            # or if rclpy.node.Node methods are called without proper setup.
            # We'll assume for now that we can instantiate it to test its non-ROS methods.
            # If not, specific mocks for rclpy.create_publisher etc. are needed.
            node = TrajectoryMakerNode()
        except Exception as e:
            # This path suggests that instantiating TrajectoryMakerNode requires a ROS context
            # or specific rclpy functions to be available/mocked.
            # For "basic unit tests" focusing on internal logic, we might need to skip this
            # or use more extensive mocking.
            self.fail(f"Failed to instantiate TrajectoryMakerNode for testing. "
                      f"Consider mocking rclpy dependencies if this is due to ROS context. Error: {e}")

        self.assertIsNotNone(node.start_time)
        self.assertEqual(node.current_segment_index, 0)
        if TRAJECTORY_SEGMENTS:
            self.assertEqual(node.current_acceleration, TRAJECTORY_SEGMENTS[0]["acceleration"])
        else:
            self.assertEqual(node.current_acceleration, [0.0,0.0,0.0]) # Default if no segments


    def test_get_acceleration_for_elapsed_time(self):
        """
        Tests the logic for selecting trajectory segments based on elapsed time.
        """
        try:
            node = TrajectoryMakerNode() # As above, this instantiation might need mocks
        except Exception as e:
            self.fail(f"Failed to instantiate TrajectoryMakerNode for testing _get_acceleration_for_elapsed_time. Error: {e}")

        # Test times within each segment and at boundaries
        # Segment 0: 0 to 20s (exclusive of 20 for this segment if next starts at 20)
        # Accel: [0.0, 0.0, 18.0]
        self.assertEqual(node._get_acceleration_for_elapsed_time(0), [0.0, 0.0, 18.0])
        self.assertEqual(node._get_acceleration_for_elapsed_time(10), [0.0, 0.0, 18.0])
        self.assertEqual(node._get_acceleration_for_elapsed_time(19.99), [0.0, 0.0, 18.0])

        # Segment 1: 20 to 32s (20 + 12)
        # Accel: [1.5, 0.0, 11.0]
        self.assertEqual(node._get_acceleration_for_elapsed_time(20), [1.5, 0.0, 11.0])
        self.assertEqual(node._get_acceleration_for_elapsed_time(30), [1.5, 0.0, 11.0])
        self.assertEqual(node._get_acceleration_for_elapsed_time(31.99), [1.5, 0.0, 11.0])

        # Segment 2: 32 to 42s (32 + 10)
        # Accel: [0.0, 0.0, 18.0]
        self.assertEqual(node._get_acceleration_for_elapsed_time(32), [0.0, 0.0, 18.0])
        self.assertEqual(node._get_acceleration_for_elapsed_time(41.99), [0.0, 0.0, 18.0])

        # Segment 3: 42 to 54s (42 + 12)
        # Accel: [0.0, 1.5, 11.0]
        self.assertEqual(node._get_acceleration_for_elapsed_time(42), [0.0, 1.5, 11.0])
        self.assertEqual(node._get_acceleration_for_elapsed_time(53.99), [0.0, 1.5, 11.0])

        # Segment 4: > 54s (infinite duration)
        # Accel: [0.0, 0.0, 18.0]
        self.assertEqual(node._get_acceleration_for_elapsed_time(54), [0.0, 0.0, 18.0])
        self.assertEqual(node._get_acceleration_for_elapsed_time(100), [0.0, 0.0, 18.0]) # Well beyond

        # Test with a really large time, should stick to last segment
        self.assertEqual(node._get_acceleration_for_elapsed_time(10000), TRAJECTORY_SEGMENTS[-1]["acceleration"])


if __name__ == '__main__':
    # It's better to run tests using `python -m unittest tests.test_trajectory_maker`
    # or `pytest tests/test_trajectory_maker.py`
    # However, this allows running the single file directly.
    unittest.main()
