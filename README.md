# Tea Picking Dual Arm

茶叶采摘双臂，目前计划实现遥操作功能

## 构建

```bash
source /opt/ros/humble/setup.bash

colcon build --symlink-install
source install/setup.bash
```

## 运行

```bash
ros2 launch rm_driver rm_65_driver.launch.py 
```