# `camera_aim` 真机调试文档

## 1. 目的

本文用于指导 `camera_aim` 在真机环境下的联调，重点验证以下链路：

`/k230/animals/targets -> 目标校验 -> 目标选择 -> 对准控制 -> /k230/animals/capture_ready -> /k230/animals/record_result -> /k230/animals/scan_point_done`

本文默认控制端代码、消息定义、YAML 配置已经更新到当前版本。

---

## 2. 真机调试前安全检查

真机调试前，至少确认以下事项：

- [ ] 螺旋桨安装方向正确
- [ ] 遥控器、数传、飞控、电池状态正常
- [ ] PX4 / MAVROS 已正常连接
- [ ] `OFFBOARD` 切换、解锁、降落链路已经单独验证过
- [ ] 真机首次联调时，建议关闭自动起飞任务，先做地面空转或低空短悬停测试
- [ ] 已明确异常中止手段
  - [ ] 遥控器切回安全模式
  - [ ] `/stop_mission`
  - [ ] 飞控原生紧急降落/返航方式
- [ ] 已知当前真机 `camera_aim` 策略
  - [ ] 空网格连续空帧后结束
  - [ ] `record_result` 超时后跳过当前目标
  - [ ] 单个 scan point 总超时后强制结束

建议：

- 首次真机调试把 `camera_aim_scan_point_timeout_s` 保守设短一点，例如 `15~20s`
- 首次真机调试只验证单目标或空网格，不要一开始就测多目标

---

## 3. 推荐调试顺序

建议按以下顺序推进：

1. 静态检查
2. 不起飞，仅确认话题流和消息类型
3. 起飞但不进入 `camera_aim`
4. 进入 `camera_aim`，测试空网格
5. 单目标成功闭环
6. `record_result` 超时
7. 多目标闭环

---

## 4. 编译与环境加载

```bash
cd ~/drone_ws
colcon build --packages-select drone_msgs drone_control drone_mission drone_bringup
source /opt/ros/humble/setup.bash
source ~/drone_ws/install/setup.bash
```

如果刚修改过 `ground_mission.yaml`，建议一起重新编译 `drone_mission`，确保安装空间中的配置同步更新。

---

## 5. 启动命令

启动控制端：

```bash
ros2 launch drone_bringup run_offboard.launch.py enable_offboard_control:=true
```

启动后应关注：

- `mission_controller_node`
- `route_comm_node`
- `tf_bridge_node`

正常情况下应能看到：

- TF 就绪
- mission YAML 加载成功
- `动作执行器初始化完成`

---

## 6. 常用观察命令

### 6.1 任务与控制状态

查看任务状态：

```bash
ros2 topic echo /task/status
```

查看 mission 状态播报：

```bash
ros2 topic echo /mission_status
```

查看飞控状态：

```bash
ros2 topic echo /mavros/state
```

查看当前位置：

```bash
ros2 topic echo /mavros/local_position/pose
```

### 6.2 视觉输入与输出

查看视觉目标列表：

```bash
ros2 topic echo /k230/animals/targets
```

查看控制端拍照请求：

```bash
ros2 topic echo /k230/animals/capture_ready
```

查看视觉结果回流：

```bash
ros2 topic echo /k230/animals/record_result
```

查看 scan point 完成通知：

```bash
ros2 topic echo /k230/animals/scan_point_done
```

### 6.3 频率与接口检查

查看目标列表频率：

```bash
ros2 topic hz /k230/animals/targets
```

查看消息定义：

```bash
ros2 interface show drone_msgs/msg/K230AnimalTargets
ros2 interface show drone_msgs/msg/K230CaptureReady
ros2 interface show drone_msgs/msg/K230RecordResult
ros2 interface show drone_msgs/msg/K230ScanPointDone
```

---

## 7. 任务启动与停止命令

开始任务：

```bash
ros2 topic pub --once /start_mission std_msgs/msg/Empty "{}"
```

停止任务：

```bash
ros2 topic pub --once /stop_mission std_msgs/msg/Empty "{}"
```

说明：

- `start_mission` 建议只在飞控连接正常、当前位置稳定后再发送
- 如果 `camera_aim` 行为异常，先用 `/stop_mission` 中断，再决定是否切飞控模式或手动降落

---

## 8. 真机最小调试流程

### 8.1 第一步：不上锁前检查消息链路

此阶段先不要起飞，确认：

- [ ] `/k230/animals/targets` 是否有数据
- [ ] `ros2 topic hz /k230/animals/targets` 是否稳定
- [ ] `/k230/animals/targets` 中 `scan_point_index`、`target_count` 是否合理
- [ ] `/k230/animals/record_result` 能否被正常接收

如果接口都不通，不要进入真机飞行联调。

### 8.2 第二步：起飞与悬停基础检查

先确认：

- [ ] 不发视觉话题时，系统不会异常乱飞
- [ ] `OFFBOARD` 和悬停本身正常
- [ ] `/mavros/local_position/pose` 稳定

### 8.3 第三步：空网格测试

建议先在真实环境找一个没有目标的 scan point。

预期：

- 进入 `camera_aim`
- 日志出现：
  - `camera_aim 等待 /k230/animals/targets 首帧`
  - `camera_aim 空网格确认中`
- 连续空帧超过 `camera_aim_no_target_confirm_s`
- 发布 `/k230/animals/scan_point_done`

### 8.4 第四步：单目标成功测试

建议先只选一个清晰、稳定、无遮挡目标。

预期：

- 当前目标进入 `TRACKING`
- 日志出现 `camera_aim 对准中`
- 达到阈值后发布 `/k230/animals/capture_ready`
- 日志出现 `等待视觉端 record_result`
- 收到 `record_result=captured`
- 发布 `/k230/animals/scan_point_done`

### 8.5 第五步：超时测试

可选，需谨慎：

- 不回 `record_result`，确认当前目标会 `SKIPPED`
- 观察是否继续处理下一个目标或结束当前 scan point

首次真机可不做这一步，留到地面或低风险环境验证。

---

## 9. 关键日志与含义

以下日志出现时，说明系统处于对应阶段：

### 9.1 等待目标首帧

```text
camera_aim 等待 /k230/animals/targets 首帧
```

说明：

- 当前已经进入 `camera_aim`
- 但还没收到有效 `/k230/animals/targets`

### 9.2 空网格确认

```text
camera_aim 空网格确认中
```

说明：

- 当前 scan point 已收到空目标帧
- 系统正在判断是否为空网格

### 9.3 目标选择

```text
camera_aim 选择新目标进入 TRACKING
```

说明：

- 当前开始处理一个新的目标

### 9.4 对准中

```text
camera_aim 对准中
```

说明：

- 系统正根据 `err_x / err_y` 做闭环微调

### 9.5 等待结果

```text
camera_aim 等待 record_result
```

说明：

- 已发出 `capture_ready`
- 当前目标进入 `CAPTURE_REQUESTED`

### 9.6 超时跳过

```text
目标等待 record_result 超时
当前视觉目标超时丢失
```

说明：

- 当前目标被标记为 `SKIPPED`

### 9.7 scan point 强制结束

```text
当前 scan point 处理总时长已超过 ...
```

说明：

- 当前 scan point 超过总时长限制
- 剩余目标会被强制收口

---

## 10. 真机调试建议参数

首轮真机建议保守参数：

```yaml
system:
  use_camera_aim: true
  camera_aim_target_timeout_s: 0.5
  camera_aim_stable_cycles: 20
  camera_aim_max_step: 0.03
  camera_aim_wait_first_targets_timeout_s: 2.0
  camera_aim_no_target_confirm_s: 2.0
  camera_aim_record_result_timeout_s: 5.0
  camera_aim_scan_point_timeout_s: 15.0
```

说明：

- 真机首次建议把 `camera_aim_max_step` 稍微缩小
- 真机首次建议把 `camera_aim_scan_point_timeout_s` 缩短

---

## 11. 常见问题排查

### 11.1 看不到 `K230` 自定义消息

重新编译并加载环境：

```bash
cd ~/drone_ws
colcon build --packages-select drone_msgs
source /opt/ros/humble/setup.bash
source ~/drone_ws/install/setup.bash
```

检查：

```bash
ros2 interface list | grep K230
```

### 11.2 launch 后任务 YAML 加载失败

重新编译 `drone_mission`：

```bash
cd ~/drone_ws
colcon build --packages-select drone_mission drone_bringup drone_control
source ~/drone_ws/install/setup.bash
```

### 11.3 `camera_aim` 没有明显日志

检查：

- [ ] 是否真的进入了 `camera_aim` 动作
- [ ] `use_camera_aim` 是否为 `true`
- [ ] `ground_mission.yaml` 是否为最新测试版
- [ ] `/k230/animals/targets` 是否真的有数据

---

## 12. 真机调试结束条件

满足以下条件可认为真机联调基本通过：

- [ ] 空网格能稳定结束
- [ ] 单目标能完成 `capture_ready -> record_result -> scan_point_done`
- [ ] 多目标能逐个处理
- [ ] 当前目标丢失不会导致整个 `camera_aim` 直接退出
- [ ] `record_result` 超时能跳过当前目标
- [ ] scan point 总超时能强制收口
- [ ] 全程无人机姿态与位置变化可控，没有明显发散

