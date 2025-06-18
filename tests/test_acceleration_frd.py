import unittest
from unittest.mock import MagicMock, patch
import numpy as np
from numpy.testing import assert_allclose, assert_array_almost_equal
import math

from drone_control_pkg.acceleration_frd import AccelerationFRDNode, PSI_RAD

@patch('rclpy.create_subscription') # For RosNode parent in AccelerationFRDNode
@patch('rclpy.create_publisher')    # For AccelerationFRDNode constructor
@patch('rclpy.create_timer')        # For AccelerationFRDNode constructor
@patch('rclpy.node.Node.get_logger')# For RosNode parent and AccelerationFRDNode constructor
class TestAccelerationFRD(unittest.TestCase):

    def setUp(self, mock_get_logger, mock_create_timer, mock_create_pub, mock_create_sub):
        """
        Set up an instance of AccelerationFRDNode for testing.
        """
        # Mock necessary rclpy calls for instantiation
        self.node = AccelerationFRDNode()

    def test_transform_acceleration_zero_inputs(self, mock_get_logger, mock_create_timer, mock_create_pub, mock_create_sub):
        """
        Test _transform_acceleration with zero input accelerations.
        In this case, derived theta and phi will be 0.
        The transformation will only involve the fixed PSI_RAD yaw.
        """
        acc_f, acc_r, acc_d = 0.0, 0.0, 0.0

        # Expected behavior: theta=0, phi=0. Rotation is effectively just by psi around Z.
        # R matrix for theta=0, phi=0:
        # R = [[cos(psi), -sin(psi), 0],
        #      [sin(psi),  cos(psi), 0],
        #      [0,         0,        1]]
        # a_body = [0,0,0].T
        # a_world = R * a_body + G = [0,0,0].T
        # Output format is [a_world[0,0], -a_world[1,0], -a_world[2,0]]

        expected_transformed_accel = [0.0, 0.0, 0.0]
        actual_transformed_accel = self.node._transform_acceleration(acc_f, acc_r, acc_d)
        assert_allclose(actual_transformed_accel, expected_transformed_accel, atol=1e-7)

    def test_transform_acceleration_forward_input(self, mock_get_logger, mock_create_timer, mock_create_pub, mock_create_sub):
        """
        Test _transform_acceleration with acceleration purely forward.
        acc_f = 1.0, acc_r = 0, acc_d = 0.
        Derived theta (roll-like) = atan2(0, sqrt(1^2+0^2)) = atan2(0,1) = 0.
        Derived phi (pitch-like) = atan2(1, sqrt(0^2+0^2)) = atan2(1,0) = pi/2.
        """
        acc_f, acc_r, acc_d = 1.0, 0.0, 0.0

        # With theta=0, phi=pi/2:
        # cos(theta)=1, sin(theta)=0
        # cos(phi)=0, sin(phi)=1
        # R = [[cos(psi)*1, cos(psi)*1*0 - sin(psi)*0, cos(psi)*0*0 + sin(psi)*1],
        #      [sin(psi)*1, sin(psi)*1*0 + cos(psi)*0, sin(psi)*0*0 - cos(psi)*1],
        #      [-0,         1*1,                      1*0]]
        # R = [[cos(psi),  0,  sin(psi)],
        #      [sin(psi),  0, -cos(psi)],
        #      [0,         1,  0]]
        # a_body = [1,0,0].T
        # a_world = R * a_body = [cos(psi), sin(psi), 0].T

        psi = PSI_RAD
        expected_x = math.cos(psi)
        expected_y = math.sin(psi)
        expected_z = 0.0
        # Output format [world_x, -world_y, -world_z]
        expected_transformed_accel = [expected_x, -expected_y, -expected_z]

        actual_transformed_accel = self.node._transform_acceleration(acc_f, acc_r, acc_d)
        assert_allclose(actual_transformed_accel, expected_transformed_accel, atol=1e-7)

    def test_transform_acceleration_rightward_input(self, mock_get_logger, mock_create_timer, mock_create_pub, mock_create_sub):
        """
        Test _transform_acceleration with acceleration purely rightward.
        acc_f = 0, acc_r = 1.0, acc_d = 0.
        Derived theta (roll-like) = atan2(1, sqrt(0^2+0^2)) = atan2(1,0) = pi/2.
        Derived phi (pitch-like) = atan2(0, sqrt(1^2+0^2)) = atan2(0,1) = 0.
        """
        acc_f, acc_r, acc_d = 0.0, 1.0, 0.0

        # With theta=pi/2, phi=0:
        # cos(theta)=0, sin(theta)=1
        # cos(phi)=1, sin(phi)=0
        # R = [[0, cos(psi)*0*1 - sin(psi)*1, cos(psi)*1*1 + sin(psi)*0], -> [0, -sin(psi), cos(psi)]
        #      [0, sin(psi)*0*1 + cos(psi)*1, sin(psi)*1*1 - cos(psi)*0], -> [0,  cos(psi), sin(psi)]
        #      [-1, 0,                        0]]                       -> [-1, 0, 0]
        # a_body = [0,1,0].T
        # a_world = R * a_body = [-sin(psi), cos(psi), 0].T

        psi = PSI_RAD
        expected_x = -math.sin(psi)
        expected_y = math.cos(psi)
        expected_z = 0.0
        # Output format [world_x, -world_y, -world_z]
        expected_transformed_accel = [expected_x, -expected_y, -expected_z]

        actual_transformed_accel = self.node._transform_acceleration(acc_f, acc_r, acc_d)
        assert_allclose(actual_transformed_accel, expected_transformed_accel, atol=1e-7)

    def test_transform_acceleration_downward_input(self, mock_get_logger, mock_create_timer, mock_create_pub, mock_create_sub):
        """
        Test _transform_acceleration with acceleration purely downward.
        acc_f = 0, acc_r = 0, acc_d = 1.0.
        Derived theta (roll-like) = atan2(0, sqrt(0^2+1^2)) = 0.
        Derived phi (pitch-like) = atan2(0, sqrt(0^2+1^2)) = 0.
        This is the same case as zero_inputs for theta and phi, but a_body is different.
        """
        acc_f, acc_r, acc_d = 0.0, 0.0, 1.0

        # Expected behavior: theta=0, phi=0. Rotation is effectively just by psi around Z.
        # R matrix for theta=0, phi=0:
        # R = [[cos(psi), -sin(psi), 0],
        #      [sin(psi),  cos(psi), 0],
        #      [0,         0,        1]]
        # a_body = [0,0,1].T
        # a_world = R * a_body + G = [0, 0, 1].T
        # Output format is [a_world[0,0], -a_world[1,0], -a_world[2,0]]

        expected_transformed_accel = [0.0, 0.0, -1.0] # world_z is 1, so -world_z is -1

        actual_transformed_accel = self.node._transform_acceleration(acc_f, acc_r, acc_d)
        assert_allclose(actual_transformed_accel, expected_transformed_accel, atol=1e-7)


if __name__ == '__main__':
    unittest.main()
