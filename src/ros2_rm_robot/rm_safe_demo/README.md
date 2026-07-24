# RealMan 安全联调工具

本包用于按“状态 → RViz 同步 → MoveIt 只规划 → 真机低速小动作”的顺序联调 RM65/RM75。
`safe_bringup.launch.py` 默认设置 `allow_trajectory_execution:=false`，因此 MoveIt 的 Execute 在第一阶段不可用。

## 使用前确认

- 实体急停可触达，机械臂周围无人、无障碍物，末端负载可靠固定。
- PC 网卡与机械臂处于同一网段；驱动配置中的 `arm_ip` 和 `udp_ip` 与现场一致。
- RM65 默认机械臂 IP 为 `192.168.1.18`；本工程 RM75 配置为 `192.168.1.19`，必须按实机确认。
- 每个新终端执行 `source /opt/ros/humble/setup.bash && source install/setup.bash`。

## 1. 编译

```bash
colcon build --packages-select rm_safe_demo --symlink-install
source install/setup.bash
```

## 2. 只读状态、RViz 和 MoveIt 规划

RM65：

```bash
ros2 launch rm_safe_demo safe_bringup.launch.py arm_type:=rm65 allow_trajectory_execution:=false
```

RM75 将 `rm65` 改为 `rm75`。另开终端监视状态：

```bash
ros2 run rm_safe_demo state_monitor
```

检查 `/joint_states` 持续更新，并缓慢手动拖动机械臂验证 RViz 同步。然后在 MoveIt 中设置较近目标，只点击 **Plan**，不点击 Execute。

## 3. 开启轨迹执行

结束上一条 launch，确认规划、模型和关节状态全部正确后重新启动：

```bash
ros2 launch rm_safe_demo safe_bringup.launch.py arm_type:=rm65 allow_trajectory_execution:=true
```

先做预演。RM65 使用 `arm_dof:=6`；RM75 使用 `arm_dof:=7`：

```bash
ros2 run rm_safe_demo small_joint_move --ros-args \
  -p arm_dof:=6 -p joint_index:=6 -p delta_deg:=2.0 -p speed:=5
```

预演打印的当前角和目标角正确、现场安全后，才添加执行开关：

```bash
ros2 run rm_safe_demo small_joint_move --ros-args \
  -p arm_dof:=6 -p joint_index:=6 -p delta_deg:=2.0 -p speed:=5 -p execute:=true
```

节点限制单次偏移不超过 3 度、速度不超过 10%，并检查状态超时和关节软限位。

## 4. 随时停止

任意已 source 的终端执行：

```bash
ros2 run rm_safe_demo stop_motion
```

也可以直接发布厂商停止话题：

```bash
ros2 topic pub --once /rm_driver/move_stop_cmd std_msgs/msg/Empty "{}"
```

软件停止不能替代实体急停。若停止失败、网络中断或运动异常，立即使用实体急停。
