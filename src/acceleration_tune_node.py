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

   
    # Start the tasks
    asyncio.ensure_future(imu_quaternion(drone))
    
    print("-- Setting initial setpoint")
    await drone.offboard.set_attitude(Attitude(0.0, 0.0, 0.0, 0.0))
    quaternion_value = await imu_quaternion(drone)
    print(quaternion_value)
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
    await drone.offboard.set_attitude(Attitude(0.0, 0.0, 0.0, 1.0))
    quaternion_value = await imu_quaternion(drone)
    print(quaternion_value)
    # matrix = np.zeros((1,6), dtype = float)
    i=0
    for i in range(750):
        imu_matrix = await imu_quaternion(drone)
        print(i,imu_matrix)
        # matrix = np.vstack((matrix,imu_matrix))
        i=+1
    #np.savetxt("Matrix_Graph.txt",matrix)
    await asyncio.sleep(10)
    
    """
    print("-- Roll 30 at 60% thrust")
    await drone.offboard.set_attitude(Attitude(30.0, 0.0, 0.0, 0.6))
    a = main()
    #imu_matrix = await print_imu(drone)
    #matrix = np.vstack((matrix,imu_matrix))
    await asyncio.sleep(20)
    """

    print("-- Roll -30 at 60% thrust")
    await drone.offboard.set_attitude(Attitude(30.0, 10.0, 0.0, 1.0))
    quaternion_value = await imu_quaternion(drone)
    print(quaternion_value)
    #matrix = np.zeros((1,6), dtype = float)
    #i=0
    #a = 0
    #for i in range(1000):
        #imu_matrix = await print_imu(drone)
        #if imu_matrix[0, 2] < a:
            #a = imu_matrix[0, 2]
            #print(a)
        #print(i,imu_matrix)
        #matrix = np.vstack((matrix,imu_matrix))
        #i=+1
    #imu_matrix = await print_imu(drone)
    #matrix = np.vstack((matrix,imu_matrix))
    await asyncio.sleep(20)
    
    print("-- Hover at 60% thrust")
    await drone.offboard.set_attitude(Attitude(30.0, 0.0, 0.0, 0.6))
    #a = main()
    #imu_matrix = await print_imu(drone)
    #matrix = np.vstack((matrix,imu_matrix))
    await asyncio.sleep(20)
    

    print("-- Stopping offboard")
    try:
        await drone.offboard.stop()
    except OffboardError as error:
        print(f"Stopping offboard mode failed with error code: \
            {error._result.result}")

    await drone.action.land()
    #print(np.shape(matrix))
    #mat = np.matrix(matrix)
    #np.savetxt("Matrix_Graph.txt",mat)


#async def print_imu(drone):
    #async for imu in drone.telemetry.imu(): 
        #acceleration_x = imu.acceleration_frd.forward_m_s2
        #acceleration_y = imu.acceleration_frd.right_m_s2
        #acceleration_z = imu.acceleration_frd.down_m_s2
        #angular_velocity__x = imu.angular_velocity_frd.forward_rad_s
        #angular_velocity_y = imu.angular_velocity_frd.right_rad_s
        #angular_velocity_z = imu.angular_velocity_frd.down_rad_s
        #imu_matrix = np.c_[acceleration_x, acceleration_y, acceleration_z, angular_velocity__x, angular_velocity_y,angular_velocity_z]
        #print(imu_matrix)
        #return(imu_matrix)

async def imu_quaternion(drone):
    counter = 0
    quaternion_w = 0
    quaternion_x = 0
    quaternion_y = 0
    quaternion_z = 0    
    async for attitude in drone.telemetry.attitude_quaternion():     
        quaternion_w += attitude.w
        quaternion_x += attitude.x
        quaternion_y += attitude.y
        quaternion_z += attitude.z
        # counter += 1   
        # if counter == 100:
        quaternion_value = [quaternion_w, quaternion_x, quaternion_y, quaternion_z]
            #if ((quaternion_value) != 0.0):
                #raise Exception(quaternion_value)
        siny_cosp = 2.0 * (quaternion_value[0]  * quaternion_value[3] + quaternion_value[1] * quaternion_value[2])
        cosy_cosp = 1.0 - 2.0 * (quaternion_value[2] * quaternion_value[2] + quaternion_value[3] * quaternion_value[3])
        yaw = np.degrees(np.arctan2(siny_cosp, cosy_cosp))
        return yaw

"""class AccelerationNode(Node):
    def __init__(self):
        super().__init__('acceleration_node')
        self.subscriber_ = self.create_subscription(Imu, "raw_imu", self.callback, 10)

    def callback(self, msg):
        acceleration = msg.data.linear_acceleration
        self.get_logger().info("Acceleration: x: %f, y: %f, z: %f" % (acceleration.x, acceleration.y, acceleration.z))

def main(args=None):
    rclpy.init(args=args)
    acceleration_node = AccelerationNode()
    rclpy.spin(acceleration_node)
    rclpy.shutdown()"""

if __name__ == "__main__":
    # Start the main function
    asyncio.ensure_future(run())
    # Runs the event loop until the program is canceled with e.g. CTRL-C
    asyncio.get_event_loop().run_forever()