#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import QoSProfile
import numpy as np
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Twist

from sensor_msgs.msg import Imu
from sensor_msgs.msg import Range
from nav_msgs.msg import Odometry

def jacobin_matrix(phi):
    J = np.array([[np.cos(phi), -np.sin(phi), 0],
                  [np.sin(phi),  np.cos(phi), 0],
                  [0,            0,           1]])
    return J

class MrImuOrTF(Node): # <--- CHANGE ME
    def __init__(self):
        super().__init__("mr_imu_or_tf") # <--- CHANGE ME

                # publishers and subscribers
        qos = QoSProfile(depth=10)
    
        # subscriber
        self.imu_sub = self.create_subscription(
            msg_type=Imu,
            topic="imu",
            callback=self.imu_callback,
            qos_profile=qos
        )
        
        self.ultrasonic_sub = self.create_subscription(
            msg_type=Range,
            topic="ultrasonic",
            callback=self.ultrasonic_callback,
            qos_profile=qos
        )
        
        self.range_pub = self.create_publisher(
            msg_type=Range,
            topic="range",
            qos_profile=qos
        )
        
        self.imu = Imu()
        self.ultrasonic = Range()
        self.range = Range()
        
        # tf broadcaster
        # Create a TF broadcaster to publish TransformStamped messages onto /tf
        # Used to maintain coordinate frame relationships in ROS 2
        self.tf_broadcaster = TransformBroadcaster(self)
        
        self.t = TransformStamped()
        self.t.header.frame_id = 'odom'
        self.t.child_frame_id = 'base_link'

        # time variables for integration
        self.dt = 0.1
        self.timer = self.create_timer(self.dt, self.timer_callback)
        
    def imu_callback(self, msg):
        self.get_logger().info(f"Orientation x: {msg.orientation.x}, y: {msg.orientation.y}, z: {msg.orientation.z}, w: {msg.orientation.w}")
        #self.get_logger().info(f"Angular Velocity x: {msg.angular_velocity.x}, y: {msg.angular_velocity.y}, z: {msg.angular_velocity.z}")
        #self.get_logger().info(f"Linear Acceleration x: {msg.linear_acceleration.x}, y: {msg.linear_acceleration.y}, z: {msg.linear_acceleration.z}")
        
        self.imu = msg
    
    def ultrasonic_callback(self, msg):
        self.get_logger().info(f"Ultrasonic Range: {msg.range} m")
        self.ultrasonic = msg

    def timer_callback(self):
                
        # publish tf message
        self.or_tf_broadcaster()
        
    def or_tf_broadcaster(self):
        
        # header
        now = self.get_clock().now()
        self.t.header.stamp = now.to_msg()
               
        # position and orientation
        self.t.transform.translation.x = 0.0
        self.t.transform.translation.y = 0.0
        self.t.transform.translation.z = 0.0
        self.t.transform.rotation.x = self.imu.orientation.x
        self.t.transform.rotation.y = self.imu.orientation.y
        self.t.transform.rotation.z = self.imu.orientation.z
        self.t.transform.rotation.w = self.imu.orientation.w

        # send the transformation
        self.tf_broadcaster.sendTransform(self.t)
        
        self.range.header.stamp = now.to_msg()
        self.range.header.frame_id = "base_link"
        self.range.radiation_type = self.ultrasonic.radiation_type
        self.range.field_of_view = self.ultrasonic.field_of_view
        self.range.min_range = self.ultrasonic.min_range
        self.range.max_range = self.ultrasonic.max_range
        self.range.range = self.ultrasonic.range

        self.range_pub.publish(self.range)

def main():
    rclpy.init()
    node = MrImuOrTF() # <--- CHANGE ME
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()