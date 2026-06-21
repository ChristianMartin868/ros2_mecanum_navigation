# ros2_mecanum_navigation

ROS-2-Node für **Korridor-/Wandfolge per Zweipunktregelung (Bang-Bang)** eines
Mecanum-Roboters auf Basis von 8 ToF-Abstandssensoren. **Portierung** des
ROS-1-Studienprojekts aus dem Modul *Autonome Systeme* (2020).

> Verwendet die **ROS 2**-API (`rclcpp`, `rclcpp::Node`, Wall-Timer).
> Gebaut mit `ament_cmake` / `colcon`.

## Funktionsweise

`src/mecanum_navigation_node.cpp`:

- Abonniert `robot1/tof` (`std_msgs/msg/Float32MultiArray`) → speichert die
  Sensorwerte in `tof[0..7]`.
- Veröffentlicht mit **100 Hz** `geometry_msgs/msg/Twist` auf `robot1/cmd_vel`.
- Fährt konstant vorwärts (`linear.x = 1.5`).
- **Zweipunktregelung:** Stellgröße `e = (tof[4]−tof[5]) + (tof[2]−tof[3])`
  - `e < 0` → `angular.z = −0.7` (nach rechts),
  - `e > 0` → `angular.z = +0.7` (nach links),
  - `e == 0` → keine Drehung.

  Damit werden zwei seitliche Sensorpaare ausbalanciert (links vs. rechts), der
  Roboter hält sich mittig im Gang bzw. folgt einer Wand.

## Topics

| Richtung | Topic            | Typ                            |
|----------|------------------|--------------------------------|
| sub      | `robot1/tof`     | `std_msgs/msg/Float32MultiArray` |
| pub      | `robot1/cmd_vel` | `geometry_msgs/msg/Twist`      |

## Bauen & Ausführen

In einem colcon-Workspace (z. B. ROS 2 Humble / Jazzy):

```bash
# Repo nach <ws>/src/ klonen, dann im Workspace-Root:
colcon build --packages-select ros2_mecanum_navigation
source install/setup.bash
ros2 run ros2_mecanum_navigation mecanum_navigation_node
```

Testweise ToF-Daten einspeisen (8 Werte):

```bash
ros2 topic pub /robot1/tof std_msgs/msg/Float32MultiArray "{data: [1,1,1,0,1,0,1,1]}"
# und cmd_vel beobachten:
ros2 topic echo /robot1/cmd_vel
```

Abhängigkeiten: ROS 2 mit `geometry_msgs`, `std_msgs`, `rclcpp`.

## Unterschiede zum ROS-1-Original

- `ros::init` + globale `NodeHandle`/Publisher → `rclcpp::init` +
  `rclcpp::Node`-Subklasse mit Member-Publisher/-Subscriber.
- `n.subscribe` / `n.advertise` → `create_subscription` / `create_publisher`.
- `std_msgs::Float32MultiArray` / `geometry_msgs::Twist` → `…::msg::…`
  (Header `…/msg/*.hpp`); `ConstPtr` → `…::SharedPtr`.
- `while(ros::ok()) { … ros::spinOnce(); loop_rate.sleep(); }` mit
  `ros::Rate(100)` → `create_wall_timer(10ms, …)` + `rclcpp::spin`.
- Globaler `float tof[8]` → Member `std::array<float, 8>` (null-initialisiert).
- ToF-Kopierschleife: jetzt mit Bounds-Schutz (`std::min(data.size(), 8)`),
  damit ein zu großes Array keinen Pufferüberlauf verursacht. Verhalten sonst
  identisch.
- Ungenutzter Include `geometry_msgs/Pose.h` aus dem Original entfernt.
- Build: catkin → ament_cmake (`package.xml` format 3, `ament_package`).
