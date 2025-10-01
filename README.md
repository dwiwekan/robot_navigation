# Robot Navigation Course
**Instructor:** Professor Junghyun Oh

A hands-on ROS2-based course for learning autonomous vehicle navigation algorithms including emergency braking, wall following, and path tracking.

---

## 📋 Table of Contents
- [Getting Started](#getting-started)
- [Environment Setup](#environment-setup)
- [Assignment 1: Automatic Emergency Braking](#assignment-1-automatic-emergency-braking-aeb)
- [Assignment 2: Wall Following](#assignment-2-wall-following)
- [Assignment 3: Robot Tracking](#assignment-3-robot-tracking)
- [Development Workflow](#development-workflow)

---

## 🚀 Getting Started

### Repository Setup

Clone this repository into your ROS2 workspace:

```bash
git clone https://github.com/dwiwekan/robot_navigation.git
```

### Framework Structure

This repository provides a structured framework for implementing navigation algorithms. Students may modify the code freely, though the provided structure follows best practices for the assignments.

**Key files modified from the original:**
- `/config/assignment1_aeb.yaml`
- `/config/assignment2_wall_follow.yaml`
- `/config/assignment3_tracking.yaml`
- `/config/sim.yaml`
- `/launch/assignment1_aeb.launch.py`
- `/launch/assignment2_wall_follow.launch.py`
- `/launch/assignment3_tracking.launch.py`
- `/setup.py`

---

## 🛠️ Environment Setup

### Building the Package

After editing any code, rebuild the package:

```bash
colcon build --packages-select f1tenth_gym_ros
```

### Launching the Simulation

The container includes `tmux` for managing multiple bash sessions.

1. **Start a new terminal session**
2. **Source the ROS2 environment:**
   ```bash
   source /opt/ros/foxy/setup.bash
   source install/local_setup.bash
   ```
3. **Launch the simulator:**
   ```bash
   ros2 launch f1tenth_gym_ros gym_bridge_launch.py
   ```

An RViz window will appear showing the simulation environment.

---

## 📝 Assignment 1: Automatic Emergency Braking (AEB)

### Objective
Implement an emergency braking system that calculates Time-to-Collision (TTC) and stops the robot when collision risk exceeds a threshold.

### Your Task

The robot moves straight ahead. You must implement AEB functionality to detect obstacles and brake automatically.

#### Functions to Implement

**1. Calculate Time-to-Collision**

```python
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

**2. Emergency Braking Logic**

```python
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

### Testing

```bash
ros2 launch f1tenth_gym_ros assignment1_aeb.launch.py
```

---

## 📝 Assignment 2: Wall Following

### Objective
Implement wall-following behavior using two control strategies (Bang-Bang and PID) and compare their performance.

### Your Task

Make the robot follow a wall using both control methods and analyze the differences in performance.

#### Functions to Implement

**1. Wall Distance Calculation**

```python
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
    # Step 2: Convert angles to indices
    # Step 3: Extract ranges in this angular sector
    # Step 4: Filter out invalid ranges
    # Step 5: Return the minimum distance
    
    return None
```

**2. Bang-Bang Control**

```python
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
    return 0.0
```

**3. PID Control**

```python
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
    # Step 2: Calculate proportional term
    # Step 3: Calculate integral term
    # Step 4: Calculate derivative term
    # Step 5: Combine PID terms
    # Step 6: Update state variables
    # Step 7: Limit steering angle (optional)
    
    return 0.0
```

### Testing

**PID Control:**
```bash
ros2 launch f1tenth_gym_ros assignment2_wall_follow.launch.py
```

**Bang-Bang Control:**
```bash
ros2 launch f1tenth_gym_ros assignment2_wall_follow.launch.py use_pid:=false
```

### Comparison Task
Analyze and report the differences in performance between Bang-Bang and PID control strategies.

---

## 📝 Assignment 3: Robot Tracking

### Objective
Implement path tracking using Pure Pursuit and Stanley control methods. Guide the robot through a series of waypoints and compare the two approaches.

### Waypoints

The robot must navigate through these 6 waypoints in order:

```python
self.waypoints = [
    [11.7673, 2.31115],
    [22.4065, 7.1491],
    [23.0923, 11.3637],
    [20.3229, 23.2846],
    [16.0845, 25.0789],
    [-21.4153, 24.5743]
]
```

Report whether the robot successfully reaches each waypoint.

#### Functions to Implement

**1. Waypoint Management**

```python
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

**2. Pure Pursuit Control**

```python
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
    # Step 2: Calculate angle to target point
    # Step 3: Apply Pure Pursuit formula
    
    return 0.0
```

**3. Stanley Control**

```python
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
    - Stanley formula: delta = heading_error + atan(K_e * CTE / (K_s + speed))
    """
    
    if self.current_waypoint_index >= len(self.waypoints) - 1:
        return 0.0
    
    # YOUR CODE HERE
    # Step 1: Find the closest point on the path
    # Step 2: Calculate cross-track error
    # Step 3: Calculate heading error
    # Step 4: Apply Stanley formula
    
    return 0.0
```

### Testing

**Pure Pursuit:**
```bash
ros2 launch f1tenth_gym_ros assignment3_tracking.launch.py
```

**Stanley Control:**
```bash
ros2 launch f1tenth_gym_ros assignment3_tracking.launch.py use_pure_pursuit:=false
```

### Comparison Task
Compare and report the performance differences between Pure Pursuit and Stanley control methods.

---

## 💻 Development Workflow

1. **Edit your code** in the appropriate assignment file
2. **Build the package:**
   ```bash
   colcon build --packages-select f1tenth_gym_ros
   ```
3. **Source the workspace:**
   ```bash
   source install/local_setup.bash
   ```
4. **Launch and test** using the assignment-specific launch command
5. **Iterate** until your implementation works correctly

**Happy Coding! 🚗💨**