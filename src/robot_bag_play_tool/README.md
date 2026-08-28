# robot_bag_play_tool

Qt-based ROS 2 bag playback tool for selectively interrupting bag topics while debugging fusion algorithms.

## Features

- Open a ROS 2 bag directory.
- Read topic name, message type, message count, storage id, serialization format, bag size, and bag time range automatically.
- Draw a lightweight rqt_bag-style timeline with topic rows, a red cursor, wheel pan, Ctrl+wheel zoom, and click-to-seek.
- Start, stop, pause, and resume each topic from the GUI.
- Play all topics or only checked topics.
- Publish enabled topics through one global scheduler so playback keeps bag timestamp order and a shared `/clock`.
- Show live topic frequency, current message timestamp, parsed tree view, and raw bytes.
- Playback speed: `0.1x`, `0.25x`, `0.5x`, `1x`, `1.5x`, `2x`, `5x`, `10x`.
- File, Playback, and View menus for the core MVP workflow.

## Build

```bash
colcon build --packages-select robot_bag_play_tool
source install/setup.bash
```

## Run

```bash
ros2 run robot_bag_play_tool robot_bag_play_tool
```

Select the bag folder that contains `metadata.yaml`.
