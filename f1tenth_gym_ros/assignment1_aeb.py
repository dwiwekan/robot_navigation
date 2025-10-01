#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped


class AEBNode(Node):
    """
    Automatic Emergency Braking Node
    Subscribes to:
        /scan - LaserScan messages from lidar
        /ego_racecar/odom - Odometry messages for vehicle speed
    
    Publishes to:
        /drive - AckermannDriveStamped messages for vehicle control
    """
    
    def __init__(self):
        super().__init__('aeb_node')
        
        # AEB Parameters
        self.TTC_THRESHOLD = 1.0  # seconds - increased for stability
        self.TTC_CLEAR_THRESHOLD = 1.5  # seconds - hysteresis for clearing brake
        self.SPEED_THRESHOLD = 0.1  # m/s
        self.ANGULAR_RANGE = np.pi / 12  # ±15 degrees
        self.MIN_SAFE_DISTANCE = 0.5  # meters - minimum distance to obstacles
        
        # Driving Parameters
        self.FORWARD_SPEED = 1.0  # m/s - constant forward speed when safe
        
        # Vehicle state
        self.current_speed = 0.0
        self.emergency_brake_active = False
        
        self.scan_subscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        self.odom_subscriber = self.create_subscription(
            Odometry,
            '/ego_racecar/odom',
            self.odom_callback,
            10
        )
        self.drive_publisher = self.create_publisher(
            AckermannDriveStamped,
            '/drive',
            10
        )
        self.get_logger().info("AEB Node started! TTC threshold: %.2f seconds" % self.TTC_THRESHOLD)
    def odom_callback(self, msg):
        self.current_speed = msg.twist.twist.linear.x

    def scan_callback(self, scan_msg):
        if self.current_speed < self.SPEED_THRESHOLD:
            return

        # Calculate TTC for obstacles in front
        min_ttc = self.calculate_ttc(scan_msg)
        self.aeb_callback(min_ttc)

    def calculate_ttc(self, scan_msg):
        """
        Calculate Time-to-Collision (TTC) for obstacles in the vehicle's path
        
        Args:
            scan_msg: LaserScan message containing range data
            
        Returns:
            float: Minimum TTC in seconds, or float('inf') if no collision threat
            
        TODO: Implement TTC calculation
        Hints:
        - Focus on scan points directly in front of the vehicle
        - Use angle_min, angle_max, and angle_increment from scan_msg
        - Consider a narrow angular range (e.g., ±15 degrees from front)
        - TTC = distance / speed (if speed > 0)
        - Handle invalid ranges (inf, nan)
        """
        
        # YOUR CODE HERE
        # Step 1: Define the angular range to check (e.g., front-facing ±15 degrees)
        angle_min = scan_msg.angle_min
        angle_increment = scan_msg.angle_increment
        ranges = np.array(scan_msg.ranges)    
            
        # Step 2: Find the indices corresponding to this angular range
        # Hint: Use scan_msg.angle_min, scan_msg.angle_increment
        
        # Step 3: Extract ranges in this angular sector
        
        # Step 4: Filter out invalid ranges (inf, nan, zero, max range)
        
        # Step 5: Calculate TTC for each valid range
        # TTC = distance / speed
        
        # Step 6: Return the minimum TTC
        
        # Placeholder return - replace with your implementation
        return float('inf')

    def aeb_callback(self, ttc):
        """
        Implement emergency braking logic based on TTC
        
        Args:
            ttc: Time-to-collision in seconds
            
        TODO: Implement emergency braking logic
        Hints:
        - Compare ttc with self.TTC_THRESHOLD
        - Publish emergency stop command if needed
        - Log emergency brake activation/deactivation
        """
        
        # YOUR CODE HERE
        # Step 1: Check if TTC is below threshold
        
        # Step 2: If emergency brake needed:
        #   - Set emergency_brake_active flag
        #   - Create AckermannDriveStamped message with zero speed
        #   - Publish the emergency stop command
        #   - Log the emergency brake activation
        
        # Step 3: If no emergency needed, reset the flag
        
        # Example message creation (uncomment and modify):
        # drive_msg = AckermannDriveStamped()
        # drive_msg.header.stamp = self.get_clock().now().to_msg()
        # drive_msg.drive.speed = 0.0  # Emergency stop
        # drive_msg.drive.steering_angle = 0.0
        # self.drive_publisher.publish(drive_msg)
        pass


def main(args=None):
    rclpy.init(args=args)
    aeb_node = AEBNode()
    
    try:
        rclpy.spin(aeb_node)
    except KeyboardInterrupt:
        aeb_node.get_logger().info("AEB Node stopped by user")
    finally:
        aeb_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()