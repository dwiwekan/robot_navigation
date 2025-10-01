#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import math

from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped


class WallFollowNode(Node):
    """
    Wall Following Node
    Subscribes to:
        /scan - LaserScan messages from lidar
    Publishes to:
        /drive - AckermannDriveStamped messages for vehicle control
    """
    def __init__(self):
        super().__init__('wall_follow_node')
        
        # Wall following parameters
        self.DESIRED_DISTANCE = 1.0  # meters - desired distance from wall
        self.FORWARD_SPEED = 1.5     # m/s - constant forward speed
        
        # Control parameters - tune these!
        self.USE_PID = True  # Set to False for bang-bang control
        
        # Bang-bang control parameters
        self.STEERING_ANGLE_HIGH = 0.4  # radians
        self.STEERING_ANGLE_LOW = -0.4  # radians
        
        # PID control parameters
        self.KP = 1.0   # Proportional gain
        self.KI = 0.0   # Integral gain  
        self.KD = 0.1   # Derivative gain
        
        # PID state variables
        self.previous_error = 0.0
        self.integral_error = 0.0
        self.last_time = None
        
        self.scan_subscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        
        self.drive_publisher = self.create_publisher(
            AckermannDriveStamped,
            '/drive',
            10
        )
        
        controller_type = "PID" if self.USE_PID else "Bang-Bang"
        self.get_logger().info(f"Wall Follow Node started! Using {controller_type} control")

    def scan_callback(self, scan_msg):
        wall_distance = self.get_wall_distance(scan_msg)
        
        if wall_distance is None:
            self.get_logger().warn("No wall detected!")
            return

        error = self.DESIRED_DISTANCE - wall_distance
    
        # Choose control method
        if self.USE_PID:
            steering_angle = self.pid_control(error)
        else:
            steering_angle = self.bang_bang_control(error)
        self.publish_drive_command(steering_angle)
        self.get_logger().info(f"Wall distance: {wall_distance:.2f}m, Error: {error:.2f}m, Steering: {steering_angle:.2f}rad")

    def get_wall_distance(self, scan_msg):
        """
        Calculate distance to the wall on the right side of the vehicle
        Args:
            scan_msg: LaserScan message
        Returns:
            float: Distance to wall in meters, or None if no wall detected
            
        TODO: Implement wall distance calculation
        Hints:
        - Focus on scan points to the right side (negative angles)
        - Use a range of angles (e.g., -60 to -30 degrees)
        - Filter out invalid ranges
        - Return the minimum valid distance in this range
        """
        
        # YOUR CODE HERE
        # Step 1: Define the angular range for wall detection (right side)
        # Example: -90 to -30 degrees (or adjust based on your needs)

        # Step 2: Convert angles to indices
        # Use scan_msg.angle_min, scan_msg.angle_increment
        
        # Step 3: Extract ranges in this angular sector

        # Step 4: Filter out invalid ranges

        # Step 5: Return the minimum distance (closest wall point)

        # Placeholder return - replace with your implementation
        return None

    def bang_bang_control(self, error):
        """
        Implement bang-bang (on/off) control
        Args:
            error: Distance error (desired - actual)
        Returns:
            float: Steering angle in radians
        TODO: Implement bang-bang control logic
        Hints:
        - If error > 0: vehicle is too far from wall, steer towards wall
        - If error < 0: vehicle is too close to wall, steer away from wall
        - Use self.STEERING_ANGLE_HIGH and self.STEERING_ANGLE_LOW
        """
        
        # YOUR CODE HERE
        # Step 1: Check if error is positive or negative
        # Step 2: Return appropriate steering angle
        
        # Placeholder return - replace with your implementation
        return 0.0

    def pid_control(self, error):
        """
        Implement PID control
        Args:
            error: Distance error (desired - actual)
        Returns:
            float: Steering angle in radians
            
        TODO: Implement PID control logic
        Hints:
        - P term: proportional to current error
        - I term: integral of error over time
        - D term: derivative of error (change rate)
        - Steering = KP * error + KI * integral + KD * derivative
        - Remember to update integral_error and previous_error
        """
        
        current_time = self.get_clock().now().nanoseconds / 1e9
        
        # YOUR CODE HERE
        # Step 1: Calculate time difference (dt)
        if self.last_time is None:
            dt = 0.01  # First iteration
        else:
            dt = current_time - self.last_time
        
        # Step 2: Calculate proportional term
        
        # Step 3: Calculate integral term
        # Update self.integral_error
        
        # Step 4: Calculate derivative term
        # Use self.previous_error
        
        # Step 5: Combine PID terms
        
        # Step 6: Update state variables
        self.previous_error = error
        self.last_time = current_time
        
        # Step 7: Limit steering angle (optional)
        # steering_angle = max(-0.4, min(0.4, steering_angle))
        
        # Placeholder return - replace with your implementation
        return 0.0

    def publish_drive_command(self, steering_angle):
        """
        Publish drive command with given steering angle
        """
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = self.get_clock().now().to_msg()
        drive_msg.drive.speed = self.FORWARD_SPEED
        drive_msg.drive.steering_angle = steering_angle
        
        self.drive_publisher.publish(drive_msg)


def main(args=None):
    rclpy.init(args=args)
    wall_follow_node = WallFollowNode()
    
    try:
        rclpy.spin(wall_follow_node)
    except KeyboardInterrupt:
        wall_follow_node.get_logger().info("Wall Follow Node stopped by user")
    finally:
        wall_follow_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()