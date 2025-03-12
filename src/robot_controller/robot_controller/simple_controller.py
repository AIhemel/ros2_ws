#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import TwistStamped,TransformStamped
from sensor_msgs.msg import JointState
from rclpy.time import Time
import numpy as np
from rclpy.constants import S_TO_NS
import math
from nav_msgs.msg import Odometry
from tf_transformations import quaternion_from_euler
from tf2_ros import TransformBroadcaster


class SimpleController(Node):

    def __init__(self):
        super().__init__("simple_controller")

        # Declare parameters
        self.declare_parameter("wheel_radius", 0.033)  # Wheel radius in meters
        self.declare_parameter("robot_length", 0.38)   # Half of the robot's length
        self.declare_parameter("robot_width", 0.30)    # Half of the robot's width

        # Get parameter values
        self.wheel_radius = self.get_parameter("wheel_radius").get_parameter_value().double_value
        self.robot_length = self.get_parameter("robot_length").get_parameter_value().double_value
        self.robot_width = self.get_parameter("robot_width").get_parameter_value().double_value

        self.get_logger().info(f"Using wheel radius: {self.wheel_radius:.3f} m")
        self.get_logger().info(f"Using robot length: {self.robot_length:.3f} m")
        self.get_logger().info(f"Using robot width: {self.robot_width:.3f} m")
        
        # Initialize previous wheel positions
        self.FL_wheel_prev_pos_ = 0.0
        self.FR_wheel_prev_pos_ = 0.0
        self.RL_wheel_prev_pos_ = 0.0
        self.RR_wheel_prev_pos_ = 0.0
        self.prev_time_ = self.get_clock().now()
        self.x_ = 0.0
        self.y_ = 0.0
        self.theta_ = 0.0
                # Fill the TF message
        self.br_ = TransformBroadcaster(self)
        self.transform_stamped_ = TransformStamped()
        self.transform_stamped_.header.frame_id = "odom"
        self.transform_stamped_.child_frame_id = "base_footprint"

        self.prev_time_ = self.get_clock().now()

        # Initialize odometry message
        self.odom_msg_ = Odometry()
        self.odom_msg_.header.frame_id = "odom"
        self.odom_msg_.child_frame_id = "base_footprint"

        L, W, r = self.robot_length, self.robot_width, self.wheel_radius

        # Speed conversion matrix for Mecanum wheels
        self.speed_conversion = (1 / r) * np.array([
            [ 1,  1,  (L + W)],  # Front Left (FL)
            [ 1, -1, -(L + W)],  # Front Right (FR)
            [ 1, -1,  (L + W)],  # Rear Left (RL)
            [ 1,  1, -(L + W)]   # Rear Right (RR)
        ])

        # Compute the pseudo-inverse of the transformation matrix
        self.speed_conversion_inv = np.linalg.pinv(self.speed_conversion.T)

        self.get_logger().info(f"Speed conversion matrix:\n{self.speed_conversion}")
        self.get_logger().info(f"Inverse speed conversion matrix:\n{self.speed_conversion_inv}")

        # ROS2 Publishers and Subscribers
        self.wheel_cmd_pub_ = self.create_publisher(Float64MultiArray, "/simple_velocity_controller/commands", 10)
        self.vel_sub = self.create_subscription(TwistStamped, "/robot_controller/cmd_vel", self.velCallback, 10)
        self.joint_sub_ = self.create_subscription(JointState, "joint_states", self.jointCallback, 10)
        self.odom_pub_ = self.create_publisher(Odometry, "robot_controller/odom", 10)

    def velCallback(self, msg):
        """ Callback function to process velocity commands and compute wheel speeds. """

        # Extract velocity components from the received message
        robot_speed = np.array([[msg.twist.linear.x],  # Forward/backward
                                [msg.twist.linear.y],  # Left/right strafing
                                [msg.twist.angular.z]])  # Rotation

        self.get_logger().info(f"Received velocity command: Vx={msg.twist.linear.x:.2f}, Vy={msg.twist.linear.y:.2f}, W={msg.twist.angular.z:.2f}")

        # Compute the correct wheel speeds using the pseudo-inverse
        wheel_speeds = np.matmul(self.speed_conversion, robot_speed)

        self.get_logger().info(f"Computed wheel speeds: {wheel_speeds.flatten()}")

        # Publish wheel speeds
        wheel_speed_msg = Float64MultiArray()
        wheel_speed_msg.data = wheel_speeds.flatten().tolist()
        self.wheel_cmd_pub_.publish(wheel_speed_msg)
    
    def jointCallback(self, msg):
        """ Callback function to compute linear and angular velocity from wheel encoder data. """

        # Extract wheel positions from JointState
        dp_FL = msg.position[0] - self.FL_wheel_prev_pos_
        dp_FR = msg.position[1] - self.FR_wheel_prev_pos_
        dp_RL = msg.position[2] - self.RL_wheel_prev_pos_
        dp_RR = msg.position[3] - self.RR_wheel_prev_pos_

        # Compute time difference (dt)
        dt = (Time.from_msg(msg.header.stamp) - self.prev_time_).nanoseconds / S_TO_NS
        if dt == 0:
            return

        # Update previous wheel positions and timestamp
        self.FL_wheel_prev_pos_ = msg.position[0]
        self.FR_wheel_prev_pos_ = msg.position[1]
        self.RL_wheel_prev_pos_ = msg.position[2]
        self.RR_wheel_prev_pos_ = msg.position[3]
        self.prev_time_ = Time.from_msg(msg.header.stamp)

        # Compute velocity components
        Vx = (self.wheel_radius / 4.0) * (dp_FL + dp_FR + dp_RL + dp_RR) / dt
        Vy = (self.wheel_radius / 4.0) * (-dp_FL + dp_FR + dp_RL - dp_RR) / dt
        W = (self.wheel_radius / (4.0 * (self.robot_length + self.robot_width))) * (dp_FL - dp_FR + dp_RL - dp_RR) / dt

        # Update robot pose
        self.theta_ += W * dt
        self.x_ += (Vx * math.cos(self.theta_) - Vy * math.sin(self.theta_)) * dt
        self.y_ += (Vx * math.sin(self.theta_) + Vy * math.cos(self.theta_)) * dt

        # Publish Odometry
        q = quaternion_from_euler(0, 0, self.theta_)
        self.odom_msg_.header.stamp = self.get_clock().now().to_msg()
        self.odom_msg_.pose.pose.position.x = self.x_
        self.odom_msg_.pose.pose.position.y = self.y_
        self.odom_msg_.pose.pose.orientation.x = q[0]
        self.odom_msg_.pose.pose.orientation.y = q[1]
        self.odom_msg_.pose.pose.orientation.z = q[2]
        self.odom_msg_.pose.pose.orientation.w = q[3]
        self.odom_msg_.twist.twist.linear.x = Vx
        self.odom_msg_.twist.twist.linear.y = Vy
        self.odom_msg_.twist.twist.angular.z = W
        self.odom_pub_.publish(self.odom_msg_)
        
        # TF
        self.transform_stamped_.transform.translation.x = self.x_
        self.transform_stamped_.transform.translation.y = self.y_
        self.transform_stamped_.transform.rotation.x = q[0]
        self.transform_stamped_.transform.rotation.y = q[1]
        self.transform_stamped_.transform.rotation.z = q[2]
        self.transform_stamped_.transform.rotation.w = q[3]
        self.transform_stamped_.header.stamp = self.get_clock().now().to_msg()
        self.br_.sendTransform(self.transform_stamped_)

def main():
    rclpy.init()
    simple_controller = SimpleController()
    rclpy.spin(simple_controller)
    simple_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
