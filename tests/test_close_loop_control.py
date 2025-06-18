import unittest
from unittest.mock import MagicMock, patch
import numpy as np
from numpy.testing import assert_allclose # For comparing float arrays

# Import the class and constants to be tested
from drone_control_pkg.close_loop_control import CloseLoopControlNode, DA_CONTROLLER_ALPHA, DA_CONTROLLER_BETA, DA_CONTROLLER_TIME_STEP

# Since CloseLoopControlNode's __init__ creates subscribers and publishers,
# rclpy needs to be initialized or its functions need to be mocked.
# We will mock the rclpy calls made during instantiation.

@patch('rclpy.create_subscription')
@patch('rclpy.create_publisher')
@patch('rclpy.node.Node.get_logger') # Mocks get_logger called by RosNode parent or self
class TestCloseLoopControl(unittest.TestCase):

    def setUp(self):
        """
        Set up an instance of CloseLoopControlNode for testing.
        The event_loop argument is not directly used by _da_controller, so None is passed.
        """
        # The patches for rclpy ensure that ROS-specific calls during __init__ are replaced with mocks.
        # We need to ensure that super().__init__(...) in CloseLoopControlNode, which calls RosNode's init,
        # and then rclpy.node.Node's init, doesn't fail.
        # The @patch for get_logger should handle logger calls.
        # If RosNode.__init__ itself does more rclpy calls, they might need patching too.
        # For now, this setup assumes RosNode.__init__ is relatively simple after get_logger is mocked.

        # Mock the event_loop argument required by CloseLoopControlNode constructor
        mock_event_loop = None
        self.node = CloseLoopControlNode(event_loop=mock_event_loop)

        # Reset call count for da_controller if needed between tests, though it's usually per instance
        self.node.da_controller_call_count = 0


    def test_da_controller_no_error_no_desired_accel(self, mock_get_logger, mock_create_pub, mock_create_sub):
        """
        Test _da_controller with zero error and zero desired acceleration.
        Expected: 'no angle', angle should be 0, thrust should be near 0 (or m*g if that's baseline).
        The current _da_controller calculates F_r_z based on alpha * desired_accel_list[2] + beta * time * error_list[2].
        If desired_accel_list[2] (world Z) is 0, and error_list[2] is 0, then F_r_z (and thus thrust_magnitude) should be 0.
        This implies the controller is for accelerations *in addition* to gravity.
        """
        error_list = [0.0, 0.0, 0.0]
        desired_accel_list = [0.0, 0.0, 0.0] # Desired acceleration in addition to gravity

        angle_type, angle_deg, thrust_magnitude = self.node._da_controller(error_list, desired_accel_list)

        self.assertEqual(angle_type, 'no angle')
        self.assertAlmostEqual(angle_deg, 0.0, places=5)
        self.assertAlmostEqual(thrust_magnitude, 0.0, places=5)
        self.assertEqual(self.node.da_controller_call_count, 1)

    def test_da_controller_pitch_scenario(self, mock_get_logger, mock_create_pub, mock_create_sub):
        """
        Test _da_controller for a pitching scenario.
        Error in X, positive desired X acceleration.
        e.g., error_x = 0.1, desired_accel_x = 1.0. Other components zero.
        F_r_x = alpha * 1.0 + beta * time * 0.1
        F_r_y = 0
        F_r_z = 0 (assuming desired Z relative to gravity is 0)
        This should result in a 'pitch' angle.
        """
        error_list = [0.1, 0.0, 0.0] # Small positive error in X
        desired_accel_list = [1.0, 0.0, 0.0] # Desired positive X acceleration

        # Expected F_r_x = 8.0 * 1.0 + 160.0 * 0.022 * 0.1 = 8.0 + 0.352 = 8.352
        # F_r_y = 0, F_r_z = 0
        # F_r_vector = [8.352, 0, 0]
        # norm_F_r = 8.352
        # z_r_desired_body_axis = [1, 0, 0]
        # Angle type should be 'pitch'.
        # Angle calculation: dot_product = np.dot([0,0,1], [1,0,0]) = 0.
        # cos_angle = 0 => angle_rad = pi/2 (90 degrees).
        # Since z_r_desired_body_axis[0] (which is 1.0) >= 0, angle is positive.
        # So, expected angle_deg is 90.0.
        # thrust_magnitude = F_r_vector[2] = 0.0

        angle_type, angle_deg, thrust_magnitude = self.node._da_controller(error_list, desired_accel_list)

        self.assertEqual(angle_type, 'pitch')
        self.assertAlmostEqual(angle_deg, 90.0, places=5)
        self.assertAlmostEqual(thrust_magnitude, 0.0, places=5)

    def test_da_controller_roll_scenario(self, mock_get_logger, mock_create_pub, mock_create_sub):
        """
        Test _da_controller for a rolling scenario.
        Error in Y, positive desired Y acceleration.
        e.g., error_y = -0.1, desired_accel_y = 1.0. Other components zero.
        F_r_x = 0
        F_r_y = alpha * 1.0 + beta * time * (-0.1)
        F_r_z = 0
        This should result in a 'roll' angle.
        """
        error_list = [0.0, -0.1, 0.0] # Small negative error in Y
        desired_accel_list = [0.0, 1.0, 0.0] # Desired positive Y acceleration

        # Expected F_r_y = 8.0 * 1.0 + 160.0 * 0.022 * (-0.1) = 8.0 - 0.352 = 7.648
        # F_r_x = 0, F_r_z = 0
        # F_r_vector = [0, 7.648, 0]
        # norm_F_r = 7.648
        # z_r_desired_body_axis = [0, 1, 0]
        # Angle type should be 'roll'.
        # Angle calculation: dot_product = np.dot([0,0,1], [0,1,0]) = 0.
        # cos_angle = 0 => angle_rad = pi/2 (90 degrees).
        # Since z_r_desired_body_axis[1] (which is 1.0) >= 0, angle is positive.
        # So, expected angle_deg is 90.0.
        # thrust_magnitude = F_r_vector[2] = 0.0

        angle_type, angle_deg, thrust_magnitude = self.node._da_controller(error_list, desired_accel_list)

        self.assertEqual(angle_type, 'roll')
        self.assertAlmostEqual(angle_deg, 90.0, places=5)
        self.assertAlmostEqual(thrust_magnitude, 0.0, places=5)

    def test_da_controller_pure_z_thrust(self, mock_get_logger, mock_create_pub, mock_create_sub):
        """
        Test _da_controller with only Z-axis desired acceleration (e.g., for hover or ascent).
        Error is zero. Desired accel is [0, 0, 9.81] (if 9.81 is baseline for hover against gravity).
        F_r_x = 0, F_r_y = 0
        F_r_z = alpha * 9.81
        z_r_desired_body_axis should be [0,0,1]. Angle type 'no angle', angle 0.
        Thrust magnitude = alpha * 9.81.
        """
        error_list = [0.0, 0.0, 0.0]
        # Let's use a value like 2.0 for desired Z accel beyond gravity compensation,
        # as alpha * g would be the part that DA_CONTROLLER_ALPHA handles implicitly if desired_accel_list is "additional".
        # The interpretation of desired_accel_list (is it total world frame accel, or additional to g?) matters.
        # Based on F_r_z = alpha * desired_accel_list[2] + ..., if desired_accel_list[2] is for *additional* accel,
        # then for hover, desired_accel_list[2] would be 0, and thrust_magnitude would be 0 from this controller,
        # implying base thrust is handled elsewhere.
        # If desired_accel_list[2] is *total* world frame desired (e.g. 0 for hover, positive for up),
        # then F_r_z = alpha * 0 for hover, which is also problematic if alpha is mass.
        # The original code's TRAJECTORY_SEGMENTS for trajectory_maker.py has Z values like 10, 14, 18.
        # These are likely m/s^2. If drone mass is 8kg (alpha), then 8*10=80N. Max thrust was ~207N.
        # Let's assume desired_accel_list is what the drone should achieve in world frame.
        desired_z_accel = 10.0
        desired_accel_list = [0.0, 0.0, desired_z_accel]

        angle_type, angle_deg, thrust_magnitude = self.node._da_controller(error_list, desired_accel_list)

        self.assertEqual(angle_type, 'no angle')
        self.assertAlmostEqual(angle_deg, 0.0, places=5)
        self.assertAlmostEqual(thrust_magnitude, DA_CONTROLLER_ALPHA * desired_z_accel, places=5)

    def test_da_controller_thresholding_of_z_r(self, mock_get_logger, mock_create_pub, mock_create_sub):
        """
        Test the thresholding part: if z_r_temp[0] or z_r_temp[1] are very small, they are set to 0.
        If error_list = [0.001, 0.001, 0.0] and desired_accel_list = [0.0, 0.0, 10.0]
        F_r_x = beta * time * 0.001 = 160 * 0.022 * 0.001 = 0.00352
        F_r_y = beta * time * 0.001 = 0.00352
        F_r_z = alpha * 10.0 = 80.0
        F_r_vector = [0.00352, 0.00352, 80.0]
        norm_F_r is approx 80.0.
        z_r_desired_body_axis approx [0.000044, 0.000044, 1.0]
        These x,y components are less than 0.1, so z_r_temp becomes [0,0,1].
        Result should be 'no angle'.
        """
        error_list = [0.001, 0.001, 0.0]
        desired_accel_list = [0.0, 0.0, 10.0]

        angle_type, angle_deg, thrust_magnitude = self.node._da_controller(error_list, desired_accel_list)

        self.assertEqual(angle_type, 'no angle')
        self.assertAlmostEqual(angle_deg, 0.0, places=5)
        # thrust_magnitude should be alpha * 10.0 + beta * time * 0.0 = 80.0
        self.assertAlmostEqual(thrust_magnitude, DA_CONTROLLER_ALPHA * 10.0, places=5)


if __name__ == '__main__':
    unittest.main()
