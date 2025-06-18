#!/usr/bin/env python3

import asyncio
import rclpy
import numpy as np
from numpy.linalg import norm
from mavsdk.offboard import (Attitude, OffboardError)
from std_msgs.msg import Float32MultiArray
import threading # For managing asyncio loop

from drone_utils import RosNode, DroneConnection

# Controller parameters (previously hardcoded in da_controller)
DA_CONTROLLER_BETA = 160.0
DA_CONTROLLER_ALPHA = 8.0  # Corresponds to mass
DA_CONTROLLER_TIME_STEP = 0.022 # Time between measurements

THRUST_MAX = 207.36 # Max thrust constant

class CloseLoopControlNode(RosNode):
    """
    ROS2 node for closed-loop control of a drone.
    It subscribes to desired acceleration, measured acceleration, and attitude quaternions.
    It then calculates and sends attitude commands to the drone via MAVSDK.
    """
    def __init__(self, event_loop):
        """
        Initializes the CloseLoopControlNode.

        Args:
            event_loop: The asyncio event loop used for MAVSDK communication.
        """
        super().__init__("close_loop_control_node")
        self.drone = None # MAVSDK drone object
        self.event_loop = event_loop # asyncio event loop

        self.acceleration_desired = None
        self.acceleration_measured = None
        self.attitude_euler = None # Storing as Euler: [roll, pitch, yaw] from subscription

        # Counters from original global scope
        self.counter_step = 1
        self.da_controller_call_count = 0 # Replaces global counter_number

        # Subscribers
        self.create_subscription(
            Float32MultiArray,
            "trajectory_maker",
            self.desired_acceleration_callback,
            10)
        self.create_subscription(
            Float32MultiArray,
            "attitude_quaternion", # This topic actually carries Euler angles
            self.attitude_callback,
            10)
        self.create_subscription(
            Float32MultiArray,
            "acceleration_frd",
            self.measured_acceleration_callback, # This will trigger control logic
            10)

        # Publisher
        self.publisher_ = self.create_publisher(Float32MultiArray, "close_loop_control", 10)
        self.get_logger().info("CloseLoopControlNode has been started.")

    def set_drone(self, drone_obj):
        """
        Sets the MAVSDK drone object.
        """
        self.drone = drone_obj
        self.get_logger().info("Drone object set in CloseLoopControlNode.")

    async def initialize_offboard(self):
        """
        Sets initial attitude and starts MAVSDK offboard mode.
        Should be called after drone connection and arming.
        """
        if not self.drone:
            self.get_logger().error("Drone object not set. Cannot initialize offboard mode.")
            return False

        self.get_logger().info("Setting initial setpoint (0,0,0,0) and starting offboard mode.")
        await self.drone.offboard.set_attitude(Attitude(0.0, 0.0, 0.0, 0.0))
        try:
            await self.drone.offboard.start()
            self.get_logger().info("Offboard mode started.")
            return True
        except OffboardError as error:
            self.get_logger().error(f"Starting offboard mode failed: {error}")
            # Consider disarming if offboard start fails critically
            # await self.drone.action.disarm()
            return False

    def desired_acceleration_callback(self, msg):
        """Callback for desired acceleration from trajectory_maker."""
        self.acceleration_desired = np.array(msg.data)
        # self.get_logger().debug(f"Desired Accel: {self.acceleration_desired}")

    def attitude_callback(self, msg):
        """
        Callback for attitude data (Euler angles) from attitude_quaternion topic.
        The original code commented out quaternion to Euler conversion, implying msg.data is already Euler.
        It's critical that this callback receives Euler angles in DEGREES for the control logic to work as expected.
        If the upstream 'attitude_quaternion' topic actually sends quaternions or radians,
        conversion logic would be needed here.
        """
        # Assuming msg.data is [roll_deg, pitch_deg, yaw_deg]
        self.attitude_euler = msg.data
        # self.get_logger().debug(f"Attitude Euler (degrees): {self.attitude_euler}")

    def measured_acceleration_callback(self, msg):
        """
        Callback for measured acceleration. This is the main trigger for control logic.
        This method is synchronous as it's a ROS callback. MAVSDK calls are scheduled.
        """
        self.acceleration_measured = np.array(msg.data)
        # self.get_logger().debug(f"Measured Accel: {self.acceleration_measured}")

        if self.acceleration_desired is None or self.attitude_euler is None:
            self.get_logger().warn("Waiting for desired acceleration and attitude data.")
            return

        # --- Control Logic (adapted from original global functions) ---
        exp_angle_type, exp_angle_val, f_r_val = self._error_control(
            self.acceleration_desired, self.acceleration_measured
        )

        traj_angle_info = [0.0, 0.0, 0.0, exp_angle_type, f_r_val] # roll, pitch, yaw, type, thrust_magnitude
        if exp_angle_type == 'pitch':
            traj_angle_info[1] = exp_angle_val # Set pitch
        elif exp_angle_type == 'roll':
            traj_angle_info[0] = exp_angle_val # Set roll

        # Calculate difference_angle (desired vs current attitude)
        # Note: original code used degrees from self.attitude_quaternion directly.
        # Assuming self.attitude_euler is [roll_deg, pitch_deg, yaw_deg]
        current_roll_deg = self.attitude_euler[0]
        current_pitch_deg = self.attitude_euler[1]
        # current_yaw_deg = self.attitude_euler[2] # Yaw not used in diff calc

        diff_roll_deg = 0.0
        diff_pitch_deg = 0.0

        if exp_angle_type == 'pitch':
            diff_roll_deg = current_roll_deg # Maintain current roll
            diff_pitch_deg = traj_angle_info[1] - current_pitch_deg
        elif exp_angle_type == 'roll':
            diff_roll_deg = traj_angle_info[0] - current_roll_deg
            diff_pitch_deg = current_pitch_deg # Maintain current pitch
        else: # 'no angle'
            diff_roll_deg = current_roll_deg
            diff_pitch_deg = current_pitch_deg

        # Normalize thrust by max thrust
        thrust_normalized = traj_angle_info[4] / THRUST_MAX
        if thrust_normalized > 1.0:
            self.get_logger().warn(f"Calculated thrust {traj_angle_info[4]} exceeds max {THRUST_MAX}. Clamping to 1.0.")
            thrust_normalized = 1.0
        elif thrust_normalized < 0.0: # Should not happen with typical control
             self.get_logger().warn(f"Calculated negative thrust {traj_angle_info[4]}. Clamping to 0.0.")
             thrust_normalized = 0.0

        # MAVSDK Attitude expects Roll, Pitch, Yaw in degrees, Thrust (0 to 1.0)
        attitude_command = Attitude(diff_roll_deg, diff_pitch_deg, 0.0, thrust_normalized) # Assuming target yaw is 0 for now

        self.get_logger().info(
            f"Step {self.counter_step}: CMD Roll: {attitude_command.roll_deg:.2f}, "
            f"Pitch: {attitude_command.pitch_deg:.2f}, Yaw: {attitude_command.yaw_deg:.2f}, "
            f"Thrust: {attitude_command.thrust_value:.2f}"
        )

        # Publish the command for monitoring
        # Original drone_step was [diff_angle_roll, diff_angle_pitch, diff_angle_yaw (0), thrust_norm]
        published_msg = Float32MultiArray()
        published_msg.data = [attitude_command.roll_deg, attitude_command.pitch_deg, attitude_command.yaw_deg, attitude_command.thrust_value]
        self.publisher_.publish(published_msg)

        # Schedule the MAVSDK call to the asyncio event loop
        if self.drone and self.event_loop:
            asyncio.run_coroutine_threadsafe(
                self.drone.offboard.set_attitude(attitude_command),
                self.event_loop
            )
        else:
            self.get_logger().warn("Drone or event loop not available for set_attitude.")

        self.counter_step += 1

    def _error_control(self, desired_accel, measured_accel):
        """
        Calculates acceleration error and passes to the DA controller.
        (Replaces global error_control function)
        """
        error = desired_accel - measured_accel # Element-wise for numpy arrays
        return self._da_controller(error.tolist(), desired_accel.tolist())

    def _da_controller(self, error_list, desired_accel_list):
        """
        Discrete Adaptive (DA) controller logic.
        (Replaces global da_controller function)
        """
        self.da_controller_call_count += 1

        # Using class constants/parameters
        beta = DA_CONTROLLER_BETA
        alpha = DA_CONTROLLER_ALPHA
        time_step = DA_CONTROLLER_TIME_STEP

        Z_r_world = np.array([0, 0, 1]) # Reference Z vector in world frame

        F_r_x = alpha * desired_accel_list[0] + beta * time_step * error_list[0]
        F_r_y = alpha * desired_accel_list[1] + beta * time_step * error_list[1]
        F_r_z = alpha * desired_accel_list[2] + beta * time_step * error_list[2]

        F_r_vector = np.array([F_r_x, F_r_y, F_r_z]) # Desired total force vector

        # Magnitude of the desired total force vector projected onto the world Z-axis
        # This is effectively the required thrust magnitude to counteract gravity and achieve desired Z acceleration.
        # The original code: f_r = Z_r[0] * F_r[0] + Z_r[1] * F_r[1] + Z_r[2] * F_r[2]
        # This is F_r_vector[2] if Z_r_world is [0,0,1]
        thrust_magnitude = F_r_vector[2] # Assuming F_r_z includes gravity compensation term (e.g. alpha * (desired_acc_z + g))
                                      # If desired_accel_list[2] is *world* Z accel, then F_r_z is basically m*(a_z+g)

        # The vector F_r represents the desired force the drone should exert.
        # The drone achieves this by tilting and applying thrust.
        # z_r is the desired orientation of the drone's Z-axis (thrust vector direction) in world frame.
        norm_F_r = norm(F_r_vector)
        if norm_F_r < 1e-6: # Avoid division by zero if F_r is near zero
            self.get_logger().warn("DA Controller: Desired force vector F_r is near zero.")
            z_r_desired_body_axis = np.array([0,0,1]) # Default to hover/no tilt
        else:
            z_r_desired_body_axis = F_r_vector / norm_F_r

        # Determine angle_type and angle (desired roll or pitch)
        # This part of the original code determines which axis (roll or pitch) primarily controls
        # the direction of z_r_desired_body_axis away from the world Z-axis [0,0,1].
        # And calculates the angle of tilt.

        # Simplified: angle between Z_r_world and z_r_desired_body_axis projection on XY plane
        # The original logic for angle_type and angle is a bit convoluted.
        # Let's re-evaluate how attitude (roll, pitch) is derived from z_r_desired_body_axis.
        # z_r_desired_body_axis = [zx, zy, zz]
        # Required roll (phi) and pitch (theta) to align drone's Z-axis with z_r_desired_body_axis:
        # Assuming small angles and a desired yaw of 0:
        # pitch_rad = asin(-z_r_desired_body_axis[0])
        # roll_rad  = asin(z_r_desired_body_axis[1] / cos(pitch_rad))
        # This is for NED frame. If FRD, signs might change.
        # The original code seems to be trying to isolate a dominant axis for tilt.

        # For now, replicating the original logic for angle determination:
        z_r_temp = z_r_desired_body_axis.copy()
        if abs(z_r_temp[0]) < 0.1: z_r_temp[0] = 0.0
        if abs(z_r_temp[1]) < 0.1: z_r_temp[1] = 0.0

        angle_type = 'no angle'
        angle_deg = 0.0

        if (z_r_temp[0] != 0.0) and (z_r_temp[1] == 0.0): # Pure pitch case
            angle_type = 'pitch'
        elif (z_r_temp[1] != 0.0) and (z_r_temp[0] == 0.0): # Pure roll case
            angle_type = 'roll'

        # Calculate angle magnitude
        dot_product = np.dot(Z_r_world, z_r_desired_body_axis)
        # norma_Z_r_world is 1
        norma_z_r_desired = norm(z_r_desired_body_axis) # Should be 1 if normalized correctly

        if norma_z_r_desired < 1e-6: # Avoid division by zero
            angle_rad = 0.0
        else:
            # Ensure dot_product is within arccos valid range [-1, 1]
            cos_angle = np.clip(dot_product / norma_z_r_desired, -1.0, 1.0)
            angle_rad = np.arccos(cos_angle)

        angle_deg_magnitude = np.degrees(angle_rad)

        if angle_type == 'pitch':
            angle_deg = angle_deg_magnitude if z_r_desired_body_axis[0] >= 0 else -angle_deg_magnitude
        elif angle_type == 'roll':
            # Original logic: if z_r[1] >= 0, angle is positive.
            # This depends on frame conventions (e.g. right-hand rule for roll).
            # Assuming positive roll for positive z_r_desired_body_axis[1] (rightward tilt for y component)
            angle_deg = angle_deg_magnitude if z_r_desired_body_axis[1] >= 0 else -angle_deg_magnitude

        # Original code had `f_r - 1`. The meaning of '-1' is unclear without more context on units
        # or if `f_r` was expected to be normalized around 1. Assuming thrust_magnitude is what's needed.
        # If the original `f_r` was already normalized thrust, then this is different.
        # Let's assume thrust_magnitude is the value to be used.
        # The original `result = [angle_type, angle ,f_r - 1]`
        # The third term was used as traj_angle[4] -> drone_step[3] -> thrust_normalized.
        # So, it was indeed the thrust magnitude.
        # The "-1" might have been an offset if f_r was some normalized value around a hover thrust of 1.
        # For now, using thrust_magnitude directly.
        # TODO: Re-verify the term `f_r - 1` from original code if issues arise.
        # The `f_r` in original code was `Z_r[0] * F_r[0] + Z_r[1] * F_r[1] + Z_r[2] * F_r[2]`, which is `F_r_vector[2]`.
        # So, the original thrust term was `F_r_vector[2] - 1`.
        # This is unusual. If F_r_vector[2] is, for example, m*g = 8 * 9.81 = 78.48 N for hover,
        # then 78.48 - 1 = 77.48. This fixed offset of 1 Newton (if units are N) is strange.
        # It's more likely that `f_r` was intended to be a normalized thrust value (e.g. 1.0 for hover),
        # and the controller adjusts this. Or there's a misunderstanding of the original units/intent.
        # For now, will use thrust_magnitude and it will be normalized later by THRUST_MAX.

        return angle_type, angle_deg, thrust_magnitude

    def _calculate_attitude_command(self, exp_angle_type, exp_angle_val, f_r_val):
        """
        Calculates the final attitude command (roll, pitch, yaw, thrust)
        based on controller outputs and current attitude.
        """
        traj_angle_info = [0.0, 0.0, 0.0, exp_angle_type, f_r_val] # roll, pitch, yaw, type, thrust_magnitude
        if exp_angle_type == 'pitch':
            traj_angle_info[1] = exp_angle_val # Set pitch
        elif exp_angle_type == 'roll':
            traj_angle_info[0] = exp_angle_val # Set roll

        current_roll_deg = self.attitude_euler[0]
        current_pitch_deg = self.attitude_euler[1]

        diff_roll_deg = 0.0
        diff_pitch_deg = 0.0

        if exp_angle_type == 'pitch':
            diff_roll_deg = current_roll_deg
            diff_pitch_deg = traj_angle_info[1] - current_pitch_deg
        elif exp_angle_type == 'roll':
            diff_roll_deg = traj_angle_info[0] - current_roll_deg
            diff_pitch_deg = current_pitch_deg
        else: # 'no angle'
            diff_roll_deg = current_roll_deg
            diff_pitch_deg = current_pitch_deg

        thrust_normalized = traj_angle_info[4] / THRUST_MAX
        if thrust_normalized > 1.0:
            self.get_logger().warn(f"Calculated thrust {traj_angle_info[4]} exceeds max {THRUST_MAX}. Clamping to 1.0.")
            thrust_normalized = 1.0
        elif thrust_normalized < 0.0:
             self.get_logger().warn(f"Calculated negative thrust {traj_angle_info[4]}. Clamping to 0.0.")
             thrust_normalized = 0.0

        # MAVSDK Attitude expects Roll, Pitch, Yaw in degrees, Thrust (0 to 1.0)
        # Assuming target yaw is 0.0 for now, as in original logic.
        return Attitude(diff_roll_deg, diff_pitch_deg, 0.0, thrust_normalized)

    def measured_acceleration_callback(self, msg):
        """
        Callback for measured acceleration. This is the main trigger for control logic.
        This method is synchronous as it's a ROS callback. MAVSDK calls are scheduled.
        """
        self.acceleration_measured = np.array(msg.data)

        if self.acceleration_desired is None or self.attitude_euler is None:
            self.get_logger().warn("Waiting for desired acceleration and attitude Euler data.")
            return

        exp_angle_type, exp_angle_val, f_r_val = self._error_control(
            self.acceleration_desired, self.acceleration_measured
        )

        attitude_command = self._calculate_attitude_command(exp_angle_type, exp_angle_val, f_r_val)

        self.get_logger().info(
            f"Step {self.counter_step}: CMD Roll: {attitude_command.roll_deg:.2f}, "
            f"Pitch: {attitude_command.pitch_deg:.2f}, Yaw: {attitude_command.yaw_deg:.2f}, "
            f"Thrust: {attitude_command.thrust_value:.2f}"
        )

        published_msg = Float32MultiArray()
        published_msg.data = [attitude_command.roll_deg, attitude_command.pitch_deg, attitude_command.yaw_deg, attitude_command.thrust_value]
        self.publisher_.publish(published_msg)

        if self.drone and self.event_loop:
            asyncio.run_coroutine_threadsafe(
                self.drone.offboard.set_attitude(attitude_command),
                self.event_loop
            )
        else:
            self.get_logger().warn("Drone or event loop not available for set_attitude.")

        self.counter_step += 1


# Global variable for the asyncio event loop thread
MAVSDK_EVENT_LOOP = None
MAVSDK_THREAD = None

def mavsdk_loop_runner(loop):
    """Runs the asyncio event loop."""
    asyncio.set_event_loop(loop)
    loop.run_forever()

async def main(args=None):
    """
    Main function to initialize and run the ROS2 node and MAVSDK communication.
    """
    global MAVSDK_EVENT_LOOP, MAVSDK_THREAD

    rclpy.init(args=args)

    MAVSDK_EVENT_LOOP = asyncio.new_event_loop()
    MAVSDK_THREAD = threading.Thread(target=mavsdk_loop_runner, args=(MAVSDK_EVENT_LOOP,), daemon=True)
    MAVSDK_THREAD.start()

    node = CloseLoopControlNode(event_loop=MAVSDK_EVENT_LOOP)
    drone_connection = DroneConnection(drone_url='udp://:14540')

    try:
        connect_future = asyncio.run_coroutine_threadsafe(drone_connection.connect_and_arm(), MAVSDK_EVENT_LOOP)
        drone = connect_future.result()
        node.set_drone(drone)

        init_offboard_future = asyncio.run_coroutine_threadsafe(node.initialize_offboard(), MAVSDK_EVENT_LOOP)
        init_offboard_successful = init_offboard_future.result()

        if not init_offboard_successful:
            node.get_logger().error("Failed to initialize offboard mode. Shutting down.")
            # Perform cleanup similar to finally block but earlier
            if MAVSDK_EVENT_LOOP.is_running():
                MAVSDK_EVENT_LOOP.call_soon_threadsafe(MAVSDK_EVENT_LOOP.stop)
            if MAVSDK_THREAD.is_alive():
                MAVSDK_THREAD.join(timeout=5.0)
            if not node._destroyed: # pylint: disable=protected-access
                node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
            return

        rclpy.spin(node)

    except ConnectionError as e:
        node.get_logger().error(f"MAVSDK Connection error: {e}")
    except KeyboardInterrupt:
        node.get_logger().info("User interrupted, shutting down.")
    except Exception as e:
        node.get_logger().error(f"An unexpected error occurred in main: {e}", exc_info=True)
    finally:
        node.get_logger().info("Performing cleanup...")
        if drone_connection.drone and MAVSDK_EVENT_LOOP and MAVSDK_EVENT_LOOP.is_running():
            disarm_future = asyncio.run_coroutine_threadsafe(drone_connection.disarm_and_land(), MAVSDK_EVENT_LOOP)
            try:
                disarm_future.result(timeout=5.0)
            except TimeoutError:
                node.get_logger().warn("Timeout during disarm_and_land().")
            except Exception as e:
                node.get_logger().error(f"Error during disarm_and_land: {e}")

        if node and not node._destroyed: # pylint: disable=protected-access
            node.destroy_node()

        if MAVSDK_EVENT_LOOP and MAVSDK_EVENT_LOOP.is_running():
            MAVSDK_EVENT_LOOP.call_soon_threadsafe(MAVSDK_EVENT_LOOP.stop)
        if MAVSDK_THREAD and MAVSDK_THREAD.is_alive():
            MAVSDK_THREAD.join(timeout=5.0)

        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    asyncio.run(main())
