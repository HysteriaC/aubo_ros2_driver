# aubo_ros2_driver

遨博机器人ROS2驱动

## 在 rviz 中查看 aubo 机器人模型（以 aubo_i5 为例）

```bash
ros2 launch aubo_description aubo_viewer.launch.py
```

## 驱动真实机械臂前建议优先完成 URDF 校准

建议先根据机器人当前控制器返回的校准补偿生成校准版 URDF，再进行实机驱动、MoveIt 规划和轨迹验证。

如果直接使用未校准的默认 URDF，可能存在以下风险：

- 规划模型与真实机器人运动学参数不一致，末端位姿存在偏差。
- RViz、MoveIt 中显示的姿态和实机反馈不完全一致，影响问题定位。
- TCP 验证、轨迹复现、离线点位比对等依赖模型精度的功能，结果可能不可靠。

推荐方法：

```bash
cd /root/Desktop/aubo_ros2_ws
python3 src/aubo_description/scripts/calibrate_urdf_dh.py \
  --robot-model aubo_i5 \
  --robot-ip 192.168.127.128
colcon build --packages-select aubo_description
```

说明：

- 生成结果默认写入 `src/aubo_description/urdf/<robot_model>_calibrated.urdf`。
- `--robot-ip` 需要显式传入。
- 运行前请确认当前 Python 环境可以导入 `numpy` 和 `pyaubo_sdk`。
- 生成后需要单独重新编译 `aubo_description` 包。

## 驱动真实机械臂 aubo_i5（修改机器人对应 `robot_ip`、`aubo_type`）

```bash
source install/setup.bash
ros2 launch aubo_ros2_driver aubo_control.launch.py aubo_type:=aubo_i5 robot_ip:=192.168.127.128 \
  use_fake_hardware:=false
ros2 launch aubo_moveit_config aubo_moveit.launch.py aubo_type:=aubo_i5
```

### 伺服模式参数

驱动启动后会进入 SDK 的伺服运动模式，用于持续接收 ros2_control 下发的关节目标并执行 `servoJoint` 控制。本驱动使用 `MotionControl.setServoModeSelect()` / `getServoModeSelect()` 进入和确认伺服模式，不再使用旧的 `setServoMode(true/false)` 接口。

`aubo_control.launch.py` 支持通过 `servo_mode` 选择进入模式，默认值为 `1`。通常无需显式传入；如果现场需要切换到其它伺服模式，可以在启动驱动时追加参数。

`common_interface` 中定义的取值如下：

| `servo_mode` | 含义 |
| --- | --- |
| `0` | 退出伺服模式，驱动停止伺服时固定使用该值 |
| `1` | 截断式规划伺服模式，默认进入模式 |
| `2` | 透传模式，直接下发 |
| `3` | 透传模式，缓存下发 |
| `4` | 1ms 透传模式，缓存下发 |
| `5` | 规划伺服模式 |
| `6` | 截断式规划伺服模式，可以叠加力控 |
| `7` | 规划伺服模式，可以叠加力控 |

其中模式 `1` 添加路点后会实时调整目标点和规划路线，目标点被更新后不保证经过之前设定的目标点；模式 `5` 会保证经过所有目标点。

```bash
ros2 launch aubo_ros2_driver aubo_control.launch.py aubo_type:=aubo_i5 robot_ip:=192.168.127.128 \
  use_fake_hardware:=false servo_mode:=2
```

不同控制器和 SDK 版本对模式支持可能存在差异，实际可用取值请以当前控制器配套 SDK 文档为准。

## 驱动真实机械臂 aubo_i5 单点轨迹执行 demo（修改机器人对应 `robot_ip`、`aubo_type`）

```bash
source install/setup.bash
ros2 launch aubo_ros2_driver aubo_control.launch.py aubo_type:=aubo_i5 robot_ip:=192.168.127.128 \
  use_fake_hardware:=false
ros2 launch ros_joints_plan joints_plan.launch.py aubo_type:=aubo_i5
```

## JSON-RPC 调试服务（修改机器人对应 `robot_ip`）

`/jsonrpc_service` 是面向调试和高级排查的 JSON-RPC 透传入口，默认连接机器人
WebSocket 端口 `9012`。常规 IO、拖动示教、Payload、TCP offset 等用户能力优先使用
上面的 ROS topic/service；这些能力会通过 ros2_control controller 与驱动协同，避免绕过
运动控制链路。

```bash
source install/setup.bash
ros2 launch aubo_ros2_driver aubo_client.launch.py robot_ip:=127.0.0.1 log_level:=info
```

## 调用示例

```bash
source install/setup.bash
ros2 service call /jsonrpc_service aubo_msgs/srv/JsonRpc \
"{cls: 'RobotState', func: 'getTcpPose', params: '[]'}"
```

## 响应示例

```bash
requester: making request: aubo_msgs.srv.JsonRpc_Request(cls='RobotState', func='getTcpPose', params='[]')

response:
aubo_msgs.srv.JsonRpc_Response(result='[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]', error='None')
```

## 异常响应示例（输入错误类 `RobotStat`）

```bash
requester: making request: aubo_msgs.srv.JsonRpc_Request(cls='RobotStat', func='getTcpPose', params='[]')

response:
aubo_msgs.srv.JsonRpc_Response(result='None', error='{"code": -32601, "message": "method not found: rob1.RobotStat.getTcpPose"}')
```

## aubo_sdk 接口参考文档

[aubo_sdk developer](https://docs.aubo-robotics.cn/arcs_api/index.html)
