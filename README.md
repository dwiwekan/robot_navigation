# Robot Navigation Class by Professor Junghyun Oh
---
## Guideline how to use this Repo
Student need to git clone this repo and put it on your workspace
```bash
git clone https://github.com/dwiwekan/robot_navigation.git
```

There's several changing from the original one, such as:
```
/config/assignment1_aeb.yaml
/config/assignment2_wall_follow.yaml
/config/assignment3_tracking.yaml
/config/sim.yaml
/launch/assignment1_aeb.launch.py
/launch/assignment2_wall_follow.launch.py
/launch/assignment3_tracking.launch.py
/setup.py
```
Students can freely modify the code or use the framework here, however this framework is the best practice to implements the assignment

After you edit the code, remember to always do
```bash
colcon build --packages-select f1tenth_gym_ros
```
Then test your code!

## Launching the Simulation

1. `tmux` is included in the contianer, so you can create multiple bash sessions in the same terminal.
2. To launch the simulation, make sure you source both the ROS2 setup script and the local workspace setup script. Run the following in the bash session from the container:
```bash
$ source /opt/ros/foxy/setup.bash
$ source install/local_setup.bash
$ ros2 launch f1tenth_gym_ros gym_bridge_launch.py
```
A rviz window should pop up showing the simulation either on your host system or in the browser window depending on the display forwarding you chose.

## Assignment 1 Automatic Emergency Breaking
Assignment 1: Implement AEB. Students should calculate the Time-to-Collision (TTC) and stop the robot if it falls below a certain threshold.

→ Give the robot move straight, the student can implement AEB when the sensor detect object in front of <br>
→ There are several function student need to modify
```Python
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
```
and
```Python
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
```

To test your code, you can run
```bash
ros2 launch f1tenth_gym_ros assignment1_aeb.launch.py
```


## Assignment 2 
Assignment 2: Implement Wall Following. Students should use both bang-bang control and PID control to make the robot follow a wall and compare the results.

→ The student need to applied the wall control with PID and bang-bang control, and compare the result <br>
→ There are several function student need to modify
```Python
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
```
and
```Python
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
```
and
```Python
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
```

To test your code, you can run this
Run with PID Control
```bash
ros2 launch f1tenth_gym_ros assignment2_wall_follow.launch.py
```
Run with Bang-bang control
```bash
ros2 launch f1tenth_gym_ros assignment2_wall_follow.launch.py use_pid:=false
```

## Assignment 3 Robot Tracking
Assignment 3: Implement Robot Tracking. Given a set of waypoints the robot must visit, students should implement both Pure Pursuit and Stanley control methods and compare the results.

Given 6 waypoints that's the robot need to follow and report if the robot success to go to the waypoint
```Python
        self.waypoints = [
            [11.7673, 2.31115],
            [22.4065, 7.1491],
            [23.0923, 11.3637],
            [20.3229, 23.2846],
            [16.0845, 25.0789],
            [-21.4153, 24.5743]
        ]
```

→ There are several function student need to modify
```Python
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
```
and
```Python
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
```
and
```Python
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
```

Run with Pure Pursuit
```bash
ros2 launch f1tenth_gym_ros assignment3_tracking.launch.py
```
Run with Stanley control
```bash
ros2 launch f1tenth_gym_ros assignment3_tracking.launch.py use_pure_pursuit:=false
```