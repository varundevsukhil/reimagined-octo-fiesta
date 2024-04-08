# Freefly Technical Challenge: Thoughts & Design Considerations

I took nearly 5 hours to implement a working solution to this technical challenge, and nearly similar additional time configuring my computer to run the simulation and communications stack. When you attempt to repeat this exercise on your local computer, ensure that your `$HOME/.bashrc` does not set the environment variables for `ROS_DOMAIN_ID`, `ROS_LOCALHOST_ONLY`, or `RMW_IMPLEMENTATION`.

## Block diagram
I will be referring to this system block diagram throughout this document as the "diagram".

![](/docs/system_block_diagram.png)

The solution in this repository will need to be executed using a capable companion computer that is connected using a Serial link to the PX4. This method ensures that the PX4 controller is unencumbered with heavier than designed compute conditions and the existence of the node heartbeat ensures that any failure (logical or otherwise) does not cascade into the safety-critical components of the drone (including the PX4 flight computer). If this solution were to fail midway, the human controller (pilot) will be notified of the circumstance via telemetry and the mode change alarm already designed in the Px4 stack.

## Technical choices
Throughout this exercise, I faced multiple forks in the road towards implementation. I have very little doubt that the fork(s) not chosen by me would not work, and there is plenty of forums where fellow enthusiasts describe their successful projects using the elements that I have foregone. The following are some of my choices and my justifications: 

### Gazebo over jMAVSim
jMAVSim is designed mainly for simple simulations, while Gazebo offers offers the ability to customize the drone, its frame and sensors, and provides access to a very detailed physics simulation plugin. The noise levels for the virtual sensors are parameterized and this allowed my to implement a solution with a smaller [Sim2Real](https://medium.com/@sim30217/sim2real-fa835321342a) gap.

### ROS2 over MAVROS and MAVSDK
My solution to this challenge is compartmentalized, and this allows for graceful handling of failures. ROS2 (along with the official PX4 communication nodes and documentation) helped me implement my solution without losing the compartmentalization feature. MAVROS offers more detailed control over the PX4 stack, but the existence of a `roscore` meant that there was a single point of failure. MAVSDK is a similar but different implementation of how ROS2 and PX4 function.

### Python (`rclpy`) over C++14 (`rclcpp.h`)
I used Python because I wanted to stick to the 4 to 8 hour timeline for this project. `rclpy` is a core Python module that ships with ROS2 and handles basic node functions such as subscribing and publishing. `rclcpp.h` is the C++ equivalent to `rclpy`. This challenge was simple and straightforward enough that the speed advantage of C++ would not be very observable.

### CSV over text file for mission waypoints
CSV files are parsed better than text files, and CSV is not more complex than text for the customer's engineering team to implement. Additionally, CSV also lends itself more easily to analytics and visualization tools. The CSV header in my solution is a simple 4 column set that contains (a) the waypoint sequence (`seq`), (b) the `x` position of the goal, (c) `y` position of the goal, and (d) the goal altitude (`alt`).

### Local over GNSS coordinate system for navigation
GNSS (or, GPS) works well for localization, but even with RTK precision, GNSS alone can not be used as a coordinate reference frame. I assume that if this challenge is to be implemented in real life, the engineers on site would have access to a hypothetical radar localization equipment that would publish the drone's position in the local coordinate system. Additionally, the following XKCD comic shows why the GNSS numbers are unreasonable.

![](/docs/coordinate_precision.png)