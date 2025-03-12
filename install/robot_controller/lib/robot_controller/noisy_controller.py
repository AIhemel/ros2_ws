#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import JointState
from rclpy.time import Time
import numpy as np
from rclpy.constants import S_TO_NS
import math
from nav_msgs.msg import Odometry
from tf_transformations import quaternion_from_euler
from tf2_ros import TransformBroadcaster
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


class NoisyController(Node):

    def __init__(self):
        super().__init__("noisy_controller")  # Correct naming convention

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
        self.prev_time_ = None  # Correctly handle first-time updates
        self.x_ = 0.0
        self.y_ = 0.0
        self.theta_ = 0.0

        # QoS settings for reliable subscription
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.joint_sub_ = self.create_subscription(JointState, "joint_states", self.jointCallback, qos_profile)
        self.odom_pub_ = self.create_publisher(Odometry, "robot_controller/odom_noisy", qos_profile)

        # TF Broadcaster
        self.br_ = TransformBroadcaster(self)
        self.transform_stamped_ = TransformStamped()
        self.transform_stamped_.header.frame_id = "odom"
        self.transform_stamped_.child_frame_id = "base_footprint_noisy"

        # Initialize odometry message
        self.odom_msg_ = Odometry()
        self.odom_msg_.header.frame_id = "odom"
        self.odom_msg_.child_frame_id = "base_footprint_ekf"

    def jointCallback(self, msg):
        """ Callback function to compute linear and angular velocity from wheel encoder data. """
        # Ensure previous time is initialized correctly
        if self.prev_time_ is None:
            self.prev_time_ = Time.from_msg(msg.header.stamp)
            return

        # Add noise to wheel readings
        wheel_encoder_fl = msg.position[0] + np.random.normal(0, 0.005)
        wheel_encoder_fr = msg.position[1] + np.random.normal(0, 0.005)
        wheel_encoder_rl = msg.position[2] + np.random.normal(0, 0.005)
        wheel_encoder_rr = msg.position[3] + np.random.normal(0, 0.005)

        # Extract wheel positions from JointState
        dp_FL = wheel_encoder_fl - self.FL_wheel_prev_pos_
        dp_FR = wheel_encoder_fr - self.FR_wheel_prev_pos_
        dp_RL = wheel_encoder_rl - self.RL_wheel_prev_pos_
        dp_RR = wheel_encoder_rr - self.RR_wheel_prev_pos_

        # Compute time difference (dt)
        current_time = Time.from_msg(msg.header.stamp)
        dt = (current_time - self.prev_time_).nanoseconds / S_TO_NS
        if dt <= 1e-6:  # Prevent issues with very small dt
            return

        # Update previous wheel positions and timestamp
        self.FL_wheel_prev_pos_ = msg.position[0]
        self.FR_wheel_prev_pos_ = msg.position[1]
        self.RL_wheel_prev_pos_ = msg.position[2]
        self.RR_wheel_prev_pos_ = msg.position[3]
        self.prev_time_ = current_time

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
    noisy_controller = NoisyController()
    rclpy.spin(noisy_controller)
    noisy_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
