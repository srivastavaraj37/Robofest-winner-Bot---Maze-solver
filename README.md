Maze Solver Bot (Robofest 5.0 Winner)

This project is an autonomous maze-solving robot built using ESP32. The main idea behind the bot is to make it capable of exploring an unknown maze completely on its own, understanding the structure, and then solving it efficiently using the shortest path. The system does not rely on any external control or communication during operation.

This robot secured 1st place at Gujarat Robofest 5.0.

Demo Video: Click here to watch
https://drive.google.com/file/d/1Y3SoVk7yhP7JwMxXjzXYTzWj-ubeMt1x/view?usp=sharing

Working:

The robot follows a two-phase approach — exploration and optimized run.

In the exploration phase, the bot moves through the maze using the Trémaux algorithm. While navigating, it continuously reads data from the ToF sensors placed at the front and sides to detect walls, junctions, and open paths. Based on this, it identifies nodes (changes in wall patterns) and stores the entire maze structure in memory as a grid. It also keeps track of visited paths to avoid unnecessary loops.

Once the maze is fully explored, the bot switches to the optimization phase. Here, it applies the Flood Fill algorithm on the stored map to compute the shortest path from the start to the goal. During the second run, the robot follows this optimized path, which significantly reduces traversal time compared to the exploration phase.

The system is also capable of handling unexpected situations. If a path becomes blocked or an obstacle is detected, the bot can recompute and switch to another valid shortest path from its current position instead of restarting.

Sensing and Control:

The robot uses a combination of sensors and feedback mechanisms for accurate navigation. A multi-zone ToF sensor (VL53L7CX) is placed at the front for detecting walls and obstacles ahead, while multiple VL53L0X sensors on the sides are used for wall following and maintaining alignment within the maze.

For motion control, N20 DC motors with encoders are used. The encoder feedback helps in tracking distance and maintaining consistent movement. An MPU6050 IMU is also integrated to improve turning accuracy and reduce drift. The overall control is handled by the ESP32, which processes sensor data and executes the navigation logic in real time.

Hardware Used:

ESP32 (main controller)
VL53L7CX ToF sensor (front, multi-zone detection)
VL53L0X ToF sensors (side wall tracking)
N20 DC gear motors with encoders
TB6612FNG motor driver
MPU6050 IMU
Li-Po battery with regulated power supply
Custom PCB for integration

Key Highlights:

The robot is fully autonomous and does not use WiFi or Bluetooth during operation.
It builds its own internal map of the maze and uses it for decision making.
Combination of Trémaux (for exploration) and Flood Fill (for optimization) ensures efficient navigation.
Maintains path memory and avoids redundant traversal.
Handles obstacles and blocked paths by dynamically selecting alternate routes.
Mechanical and electronic design is kept compact and stable for reliable movement inside narrow corridors.

Project Structure:

ESP32 code – contains the complete control logic, sensor handling, and pathfinding algorithms
CAD – design files for the robot chassis and components
Images and GIFs – visual representation of the bot and its working
Simulations – testing of algorithms before implementation on hardware
Videos – actual runs of the robot in different maze configurations

Result:

1st Place – Gujarat Robofest 5.0

Author:

Raj Srivastava
National Institute of Technology, Rourkela
