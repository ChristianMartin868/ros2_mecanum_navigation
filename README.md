# ros2_mecanum_navigation

ROS 2 node for **corridor / wall following with two-point (bang-bang) control**
of a mecanum robot, based on 8 ToF distance sensors. **Port** of the ROS 1
study project from the *Autonomous Systems* module (2020).

> Uses the **ROS 2** API (`rclcpp`, `rclcpp::Node`, wall timer).
> Built with `ament_cmake` / `colcon`.

## How it works

`src/mecanum_navigation_node.cpp`:

- Subscribes to `robot1/tof` (`std_msgs/msg/Float32MultiArray`) → stores the
  sensor values in `tof[0..7]`.
- Publishes `geometry_msgs/msg/Twist` on `robot1/cmd_vel` at **100 Hz**.
- Drives forward at a constant speed (`linear.x = 1.5`).
- **Two-point control:** control value `e = (tof[4]−tof[5]) + (tof[2]−tof[3])`
  - `e < 0` → `angular.z = −0.7` (turn right),
  - `e > 0` → `angular.z = +0.7` (turn left),
  - `e == 0` → no rotation.

  This balances the two lateral sensor pairs (left vs. right), so the robot
  stays centred in the corridor / follows a wall.

## Topics

| Direction | Topic            | Type                             |
|-----------|------------------|----------------------------------|
| sub       | `robot1/tof`     | `std_msgs/msg/Float32MultiArray` |
| pub       | `robot1/cmd_vel` | `geometry_msgs/msg/Twist`        |

## Build & run

In a colcon workspace (e.g. ROS 2 Humble / Jazzy):

```bash
# Clone the repo into <ws>/src/, then from the workspace root:
colcon build --packages-select ros2_mecanum_navigation
source install/setup.bash
ros2 run ros2_mecanum_navigation mecanum_navigation_node
```

Feed in test ToF data (8 values):

```bash
ros2 topic pub /robot1/tof std_msgs/msg/Float32MultiArray "{data: [1,1,1,0,1,0,1,1]}"
# and watch cmd_vel:
ros2 topic echo /robot1/cmd_vel
```

Dependencies: ROS 2 with `geometry_msgs`, `std_msgs`, `rclcpp`.

## Simulator (demo)

`scripts/mecanum_sim.py` (executable `mecanum_sim`) is a standalone 2D PyGame
simulator: an outer wall plus an inner rectangular wall, with the robot in the
corridor between them. It computes 8 ToF distances by ray casting against the
walls (`robot1/tof`) and integrates `robot1/cmd_vel` into a pose. Together with
the C++ node the control loop closes and the robot circles the inner wall.

It also visualises the controller: a fading trail of the path, an HUD (`v`, `ω`,
the control value `e`, turn direction, FPS), colour-coded sensor rays (the left
pair green, the right pair red, the rest dimmed) and live left/right balance
bars. The window auto-sizes to the screen height.

Sensor index layout (angle relative to heading):
`0` front, `1` back, `2` +45°, `3` −45°, `4` +90°, `5` −90°, `6` +135°,
`7` −135°. The controller uses `e = (tof[4]−tof[5]) + (tof[2]−tof[3])`.

Requires PyGame: `sudo apt install -y python3-pygame`.

Start in two terminals (run `source install/setup.bash` in both first):

```bash
# Terminal 1: simulator (opens the PyGame window, ESC quits)
ros2 run ros2_mecanum_navigation mecanum_sim

# Terminal 2: controller
ros2 run ros2_mecanum_navigation mecanum_navigation_node
```

Headless test (no window): set `export SDL_VIDEODRIVER=dummy` before
`ros2 run mecanum_sim`; the pose is then written to the log.

## Differences from the ROS 1 original

- `ros::init` + global `NodeHandle`/publisher → `rclcpp::init` +
  `rclcpp::Node` subclass with member publisher/subscriber.
- `n.subscribe` / `n.advertise` → `create_subscription` / `create_publisher`.
- `std_msgs::Float32MultiArray` / `geometry_msgs::Twist` → `…::msg::…`
  (headers `…/msg/*.hpp`); `ConstPtr` → `…::SharedPtr`.
- `while(ros::ok()) { … ros::spinOnce(); loop_rate.sleep(); }` with
  `ros::Rate(100)` → `create_wall_timer(10ms, …)` + `rclcpp::spin`.
- Global `float tof[8]` → member `std::array<float, 8>` (zero-initialised).
- ToF copy loop: now bounds-checked (`std::min(data.size(), 8)`) so an
  oversized array cannot overflow the buffer. Behaviour otherwise identical.
- Removed the unused `geometry_msgs/Pose.h` include from the original.
- Build: catkin → ament_cmake (`package.xml` format 3, `ament_package`).
