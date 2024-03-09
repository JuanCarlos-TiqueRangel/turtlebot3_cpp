## TUTLEBOT3 using simple MPC without constrains

This header file contains a class called turtlebot3_MPC that implements a simple Model Predictive Controller (MPC) for the TurtleBot3 robot. The MPC is used to control the robot's velocity and orientation to follow a given trajectory. The class controller also includes a Proportional-Integral-Derivative (PID) component for compare the robot's motion.

# Dependencies
* `eigen3`: a C++ template library for linear algebra.
* `rclcpp`: the ROS 2 C++ client library.
* `std_msgs`: a ROS 2 package for standard message definitions.
* `geometry_msgs`: a ROS 2 package for geometry-related message definitions.
* `nav_msgs`: a ROS 2 package for navigation-related message definitions.
* `sensor_msgs`: a ROS 2 package for sensor-related message definitions.

# Installation 

1. Clone the repository to your ROS2 workspace:
    ```
    cd ~/your_ros2_ws/src`
    git clone https://github.com/JuanCarlos-TiqueRangel/turtlebot3_cpp.git
    ```
2. Build the package:
    ```
    cd ~/your_ros2_ws
    colcon build --packages-select turtlebot3_cpp
    ```

# Member Functions
The turtlebot3_MPC class has the following member functions:

### turtlebot3_MPC()
Constructor of the turtlebot3_MPC class.

### double get_pos_x()
Returns the current x-coordinate position of the robot.

### double get_pos_y()
Returns the current y-coordinate position of the robot.

### double get_x_linear_speed()
Returns the current linear velocity of the robot in the x direction.

### double get_y_linear_speed()
Returns the current linear velocity of the robot in the y direction.

### double get_z_linear_speed()
Returns the current linear velocity of the robot in the z direction.

### double get_w_velocity()
Returns the current angular velocity of the robot.

### double get_yaw()
Returns the current orientation of the robot in radians.

### double simple_MPC(double w, double yaw, double ref)
* A simple model predictive control (MPC) function that controls the robot's movement.
* Takes three arguments:
    * w - Desired angular velocity of the robot.
    * yaw - Current orientation of the robot in radians.
    * ref - Desired orientation of the robot in radians.
* Returns the control action uk_mpc. This control tecnique comes from the MPC strategy from Predictive Control With Constraints: MacIejowski book. Also there is a related paper of a real implementation in a skid-steering robot [here](https://ieeexplore.ieee.org/abstract/document/9633291).

### double simple_PID(double error)
* A simple proportional-integral-derivative (PID) function that controls the robot's movement.
* Takes one argument:
    * error - Error in the orientation of the robot.
* Returns the control action uk_pid.

# Future Work 
constraints and MIMO controller will be add.

# Notes
* This file assumes that the robot is using ROS2 as the middleware.
* This file has been tested with the TurtleBot3 robot platform.

## License
This code is released under the MIT License. Feel free to modify and use it in your own projects.
