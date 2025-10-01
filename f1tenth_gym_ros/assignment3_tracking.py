#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import math
import csv

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray


class TrackingNode(Node):
    """
    Path Tracking Node
    Subscribes to:
        /ego_racecar/odom - Odometry for vehicle pose

    Publishes to:
        /drive - AckermannDriveStamped for vehicle control
        /waypoints_viz - Visualization of waypoints
    """
    
    def __init__(self):
        super().__init__('tracking_node') 
        # Control parameters
        self.USE_PURE_PURSUIT = True  # Set to False for Stanley control
        self.FORWARD_SPEED = 2.0      # m/s
        # Pure Pursuit parameters
        self.LOOKAHEAD_DISTANCE = 1.5  # meters
        self.WHEELBASE = 0.33          # meters (vehicle wheelbase)
        # Stanley parameters  
        self.K_E = 0.3    # Cross-track error gain
        self.K_S = 2.0    # Soft normalization constant
        # Vehicle state
        self.current_pose = None
        self.current_speed = 0.0
        # Waypoints
        self.waypoints = []
        self.current_waypoint_index = 0
        # Load waypoints (you can create your own CSV file)
        self.load_waypoints()
        # Create subscribers
        self.odom_subscriber = self.create_subscription(
            Odometry,
            '/ego_racecar/odom',
            self.odom_callback,
            10
        )
        # Create publishers
        self.drive_publisher = self.create_publisher(
            AckermannDriveStamped,
            '/drive',
            10
        )
        self.marker_publisher = self.create_publisher(
            MarkerArray,
            '/waypoints_viz',
            10
        )
        # Visualization timer
        self.viz_timer = self.create_timer(1.0, self.publish_waypoints_viz)
        
        controller_type = "Pure Pursuit" if self.USE_PURE_PURSUIT else "Stanley"
        self.get_logger().info(f"Tracking Node started! Using {controller_type} control")
        self.get_logger().info(f"Loaded {len(self.waypoints)} waypoints")

    def load_waypoints(self):
        self.waypoints = [
            [11.7673, 2.31115],
            [22.4065, 7.1491],
            [23.0923, 11.3637],
            [20.3229, 23.2846],
            [16.0845, 25.0789],
            [-21.4153, 24.5743]
        ]

    def odom_callback(self, msg):
        self.current_pose = [
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            self.quaternion_to_yaw(msg.pose.pose.orientation)
        ]
        self.current_speed = msg.twist.twist.linear.x
        if self.current_pose is not None and len(self.waypoints) > 0:
            self.control_vehicle()

    def quaternion_to_yaw(self, quat):
        siny_cosp = 2 * (quat.w * quat.z + quat.x * quat.y)
        cosy_cosp = 1 - 2 * (quat.y * quat.y + quat.z * quat.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def control_vehicle(self):
        if len(self.waypoints) == 0:
            self.get_logger().warn("No waypoints set! Robot will stop.")
            self.publish_stop_command()
            return
        
        # Update current waypoint
        self.update_current_waypoint()
        
        if self.current_waypoint_index >= len(self.waypoints):
            self.get_logger().info("Reached final waypoint! Robot will stop.")
            self.publish_stop_command()
            return
        
        # Choose control algorithm
        if self.USE_PURE_PURSUIT:
            steering_angle = self.pure_pursuit_control()
        else:
            steering_angle = self.stanley_control()
        
        # Publish drive command
        self.publish_drive_command(steering_angle)

    def update_current_waypoint(self):
        """
        Update the target waypoint based on vehicle position
        TODO: Implement waypoint switching logic
        Hints:
        - Calculate distance to current target waypoint
        - Switch to next waypoint when close enough (e.g., < 1.0 meter)
        - Avoid going backwards in the waypoint list
        """
        
        # YOUR CODE HERE
        # Step 1: Get current position
        # Step 2: Calculate distance to current target waypoint
        # Step 3: If close enough, advance to next waypoint
        
        pass

    def pure_pursuit_control(self):
        """
        Implement Pure Pursuit control algorithm
        Returns:
            float: Steering angle in radians
        TODO: Implement Pure Pursuit algorithm
        Hints:
        - Find the target point at lookahead distance
        - Calculate the angle to the target point
        - Use Pure Pursuit formula: delta = atan(2*L*sin(alpha)/ld)
        - Where L is wheelbase, alpha is angle to target, ld is lookahead distance
        """
        
        if self.current_waypoint_index >= len(self.waypoints):
            return 0.0
        
        # YOUR CODE HERE
        # Step 1: Find the lookahead point
        # - Start from current waypoint index
        # - Find point that is approximately LOOKAHEAD_DISTANCE away
        
        # Step 2: Calculate angle to target point
        # - Use atan2(target_y - current_y, target_x - current_x)
        # - Subtract current vehicle heading
        
        # Step 3: Apply Pure Pursuit formula
        # steering_angle = atan(2 * WHEELBASE * sin(alpha) / lookahead_distance)
        
        # Placeholder return - replace with your implementation
        return 0.0

    def stanley_control(self):
        """
        Implement Stanley control algorithm
        Returns:
            float: Steering angle in radians
        TODO: Implement Stanley controller
        Hints:
        - Find the closest point on the path
        - Calculate cross-track error (perpendicular distance to path)
        - Calculate heading error (difference between vehicle and path heading)
        - Stanley formula: delta = heading_error + atan(K_e * cross_track_error / (K_s + speed))
        """
        if self.current_waypoint_index >= len(self.waypoints) - 1:
            return 0.0
        
        # YOUR CODE HERE
        # Step 1: Find the closest point on the path
        # - Get current and next waypoint
        # - Calculate closest point on line segment between them
        
        # Step 2: Calculate cross-track error
        # - Perpendicular distance from vehicle to path
        # - Sign indicates which side of path
        
        # Step 3: Calculate heading error
        # - Difference between vehicle heading and path heading
        
        # Step 4: Apply Stanley formula
        # steering = heading_error + atan(K_e * cross_track_error / (K_s + speed))
        
        # Placeholder return - replace with your implementation
        return 0.0

    def find_lookahead_point(self):
        """Find lookahead point for Pure Pursuit"""
        current_x, current_y = self.current_pose[0], self.current_pose[1]
        # Search for lookahead point
        for i in range(self.current_waypoint_index, len(self.waypoints)):
            wp_x, wp_y = self.waypoints[i]
            distance = math.sqrt((wp_x - current_x)**2 + (wp_y - current_y)**2)
            if distance >= self.LOOKAHEAD_DISTANCE:
                return wp_x, wp_y
        # Return last waypoint if no suitable point found
        return self.waypoints[-1]

    def publish_drive_command(self, steering_angle):
        """Publish drive command"""
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = self.get_clock().now().to_msg()
        drive_msg.drive.speed = self.FORWARD_SPEED
        drive_msg.drive.steering_angle = max(-0.4, min(0.4, steering_angle))
        
        self.drive_publisher.publish(drive_msg)

    def publish_stop_command(self):
        """Publish stop command when no waypoints or finished"""
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = self.get_clock().now().to_msg()
        drive_msg.drive.speed = 0.0
        drive_msg.drive.steering_angle = 0.0
        
        self.drive_publisher.publish(drive_msg)

    def publish_waypoints_viz(self):
        """Publish waypoints for visualization in RViz"""
        marker_array = MarkerArray()
        
        for i, (x, y) in enumerate(self.waypoints):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "waypoints"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = float(x)
            marker.pose.position.y = float(y)
            marker.pose.position.z = 0.1
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.2
            
            # Color based on whether it's the current target
            if i == self.current_waypoint_index:
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
            else:
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0
            marker.color.a = 1.0
            
            marker_array.markers.append(marker)
        
        self.marker_publisher.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    tracking_node = TrackingNode()
    
    try:
        rclpy.spin(tracking_node)
    except KeyboardInterrupt:
        tracking_node.get_logger().info("Tracking Node stopped by user")
    finally:
        tracking_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()