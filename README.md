# Physical AI PubSub Package
[日本語版](README.ja.md)

A ROS2 package for example publisher/subscriber communication with the CRANE+ V2 robot.

## Features

- Joint state monitoring via subscriber
- Robot movement control via publisher

## Installation
1. Setup Environment

Follow https://github.com/AI-Robot-Book-Humble/docker-ros2-desktop-ai-robot-book-humble to setup the Docker environment
Launch CRANE+ V2 manipulator in Gazebo by running
```bash
ros2 launch crane_plus_gazebo crane_plus_with_table.launch.py
```

2. Clone this repository into your ROS2 workspace:

```bash
cd ~/pai_ws/src
git clone https://github.com/matsuolab/physicalai_pubsub.git
```

3. Install dependencies:

```bash
cd ~/pai_ws
rosdep install --from-paths src --ignore-src -r -y
```

4. Build the package:

```bash
colcon build --symlink-install --packages-select physicalai_pubsub
```

5. Source your workspace:

```bash
source install/setup.bash
```

## Usage

### State Subscriber

To monitor joint states:
```bash
ros2 run physicalai_pubsub state_subscriber
```

### Action Publisher

To execute a sequence of movements:
```bash
ros2 run physicalai_pubsub action_publisher
```

## License

This software is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.

## Author

[Tatsuya Kamijo](https://tatsukamijo.github.io/)
