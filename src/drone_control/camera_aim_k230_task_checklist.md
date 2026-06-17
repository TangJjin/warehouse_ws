# `camera_aim` 对接 K230 视觉多目标任务清单

## 1. 目标

将 `drone_control` 当前基于 `/camera_aiming_center` 的单目标对准流程，改造成基于 `/k230/animals/targets` 的多目标处理流程，并在满足拍照条件后发布 `/k230/animals/capture_ready`。

当前已实现的闭环：

`targets -> 校验 -> 建队列 -> 选目标 -> 对准 -> capture_ready -> record_result -> scan_point_done`

并已补充空网格判定、当前目标等待结果超时、单网格总超时等收口逻辑。

---

## 2. 当前现状

### 2.1 现有控制链

- `src/drone_control/src/mission_controller_node.cpp`
  - 从 YAML 解析 `camera_aim` 动作。
- `src/drone_control/include/drone_control/drone_action.hpp`
  - 定义 `ActionType::CAMERA_AIM`。
- `src/drone_control/include/drone_control/action_executor.hpp`
  - 订阅 `/camera_aiming_center`
  - 使用 `geometry_msgs/msg/Point` 作为像素偏差输入
  - 在 `executeCameraAim()` 中做 PID 修正并发送 `/mavros/setpoint_position/local`

### 2.2 与视觉交接文档的差距

- 视觉端已改为发布 `/k230/animals/targets`
- 当前控制端没有订阅 `drone_msgs/msg/K230AnimalTargets`
- 当前控制端没有：
  - `scan_point_index` 维度的目标管理
  - `label + label_instance_id` 目标标识
  - `capture_ready` 发布逻辑
  - 目标状态机
  - 帧有效性校验

---

## 3. 本次改造范围

### 3.1 第一阶段必须完成

- 订阅 `/k230/animals/targets`
- 校验目标帧是否合法
- 维护当前 `scan_point` 的目标队列
- 从 `pending` 目标中选择一个进行对准
- 使用 `err_x/err_y` 或 `norm_x/norm_y` 驱动 `camera_aim`
- 达到拍照条件后发布 `/k230/animals/capture_ready`
- 对同一目标只发布一次 `capture_ready`

### 3.2 当前已补齐的第二阶段

- 已订阅 `/k230/animals/record_result`
- 已实现完整状态流转：
  - `capture_requested -> captured / failed / skipped`
- 已发布 `/k230/animals/scan_point_done`
- 已实现以下收口策略：
  - 空网格连续空帧确认后结束
  - `record_result` 超时后跳过当前目标
  - 单个 `scan_point` 超过总时长后强制结束

---

## 4. 设计约束

- 不允许新增第二个节点去抢发 `/mavros/setpoint_position/local`
- 新视觉对准逻辑必须接入现有 `ActionExecutor`
- 旧 `camera_aim` 最好保留兼容开关，避免影响当前任务流程
- 对外坐标系按 MAVROS 的 ENU 理解，机体系误差转位移时必须确认方向符号

---

## 5. 任务拆分

## 任务 A: 明确消息与接口

- [x] 确认使用的输入消息：
  - `drone_msgs/msg/K230AnimalTargets`
  - `drone_msgs/msg/K230AnimalTarget`
- [x] 新增输出消息：
  - 建议新增 `drone_msgs/msg/K230CaptureReady`
- [x] 明确 `capture_ready` 字段最小集：
  - `frame_seq`
  - `scan_point_index`
  - `label`
  - `label_instance_id`
  - `capture_ready`

产出：

- 控制端和视觉端都能引用的正式消息定义

---

## 任务 B: 定义控制端内部数据结构

- [x] 定义单个视觉目标的内部 key
  - 当前实现：`scan_point_index + label + label_instance_id`
- [x] 定义目标状态枚举
  - `pending`
  - `tracking`
  - `capture_requested`
  - 预留：`captured / failed / skipped`
- [x] 定义当前活动扫描点状态
  - `active_scan_point_index`
  - 当前目标队列
  - 当前选中的目标
  - 是否已经对该目标发送过 `capture_ready`

建议落点：

- 优先在 `action_executor.hpp` 内新增结构体
- 如果字段明显增多，再抽成独立头文件

---

## 任务 C: 接入 `/k230/animals/targets`

- [x] 在 `ActionExecutor` 内新增订阅器
- [x] 接收一帧 `K230AnimalTargets` 后只做缓存，不直接控制飞机
- [x] 增加最新目标帧时间戳，供超时判断

建议落点：

- `src/drone_control/include/drone_control/action_executor.hpp`

---

## 任务 D: 实现目标帧校验

收到 `/k230/animals/targets` 后按顺序校验：

- [x] `scan_point_index >= 0`
- [x] `target_count == targets.size()`
- [x] 同一 `label` 下 `label_instance_id` 连续

校验失败策略：

- [x] 记录日志，说明丢帧原因
- [x] 本帧不入队
- [x] 当前若正在等待该帧，不立即发布 `capture_ready`

---

## 任务 E: 建立目标队列

- [x] 以 `scan_point_index` 为单位维护目标集合
- [x] 新 `scan_point_index` 到来时，决定是：
  - 清空旧队列并切换到新扫描点
  - 还是拒绝切换并等待当前点完成
- [x] 同一扫描点重复帧到来时，更新目标缓存但不重复创建已存在目标
- [x] 缺失的旧目标如何处理，先采用保守策略：
  - 当前阶段只更新本帧可见目标
  - 不立即把丢失目标判失败

第一版建议：

- 只维护一个 `active_scan_point_index`
- 收到新扫描点时，如果旧扫描点尚未完成，先打印警告并丢弃新点

---

## 任务 F: 改造 `executeCameraAim()`

目标：

- 不再依赖旧 `/camera_aiming_center`
- 改为读取“当前选中目标”的偏差字段

具体事项：

- [x] 从队列中选第一个 `pending` 目标进入 `tracking`
- [x] 使用该目标的 `err_x/err_y` 或 `norm_x/norm_y` 作为误差输入
- [x] 保留现有 PID 和位移限幅逻辑
- [x] `target_pose` 继续承担高度/保持轴锚点作用
- [x] 当目标数据超时或当前目标丢失时，退回保守策略
  - 当前实现：仅跳过当前目标，继续当前 `scan_point` 的后续目标处理
- [x] 已支持空网格判定
- [x] 已支持 `record_result` 等待超时
- [x] 已支持单网格总超时

第一版建议：

- 继续使用像素误差阈值判定是否对准
- 暂不把 `bbox_area` 作为硬门槛

---

## 任务 G: 定义拍照条件与触发逻辑

- [x] 设定对准阈值
  - `abs(err_x) < threshold_x`
  - `abs(err_y) < threshold_y`
- [x] 设定稳定计数
  - 连续 N 个控制周期满足才算对准完成
- [x] 明确是否需要额外距离条件
  - 第一版可不强依赖 `bbox_area`

建议新增参数：

- [x] `camera_aim_stable_cycles`
- [x] `camera_aim_target_timeout_s`
- [x] `camera_aim_max_step`
- [x] `camera_aim_wait_first_targets_timeout_s`
- [x] `camera_aim_no_target_confirm_s`
- [x] `camera_aim_record_result_timeout_s`
- [x] `camera_aim_scan_point_timeout_s`
- [ ] `camera_aim_err_x_tolerance`
- [ ] `camera_aim_err_y_tolerance`

说明：

- 当前仍沿用 `DroneAction::getCameraAimTolerance()` 作为 `err_x/err_y` 阈值
- 后续如果需要把 X/Y 容差拆开，再单独补参数

---

## 任务 H: 发布 `/k230/animals/capture_ready`

- [x] 新增 publisher
- [x] 在目标首次达到拍照条件时发布
- [x] 每个目标只发布一次
- [x] 发布后把目标状态切到 `capture_requested`

发布内容：

- [x] `frame_seq`
- [x] `scan_point_index`
- [x] `label`
- [x] `label_instance_id`
- [x] `capture_ready = true`

建议落点：

- `ActionExecutor`

---

## 任务 I: 配置与兼容

- [x] 保留 `use_camera_aim`
- [ ] 新增模式或参数区分：
  - 旧模式：`/camera_aiming_center`
  - 新模式：`/k230/animals/targets`
- [x] 更新示例 YAML 配置
- [x] 明确 `camera_aim` 动作在新模式下的配置语义

建议：

- 增加一个布尔参数，例如 `camera_aim_use_k230_targets`

---

## 任务 J: 联调日志与调试辅助

- [x] 打印当前活动 `scan_point_index`
- [x] 打印当前处理目标 key
- [x] 打印状态迁移
- [x] 打印丢帧原因
- [x] 打印 `capture_ready` 发布时间点

建议日志级别：

- 正常流程用 `INFO`
- 高频过程信息用 `THROTTLE`
- 异常帧和超时用 `WARN`

---

## 6. 预计修改文件

### 必改

- [x] `src/drone_control/include/drone_control/action_executor.hpp`
- [ ] `src/drone_control/src/mission_controller_node.cpp`
- [ ] `src/drone_control/include/drone_control/drone_action.hpp`
- [ ] `src/drone_control/CMakeLists.txt`
- [ ] `src/drone_control/package.xml`

### 可能新增

- [x] `src/drone_msgs/msg/K230CaptureReady.msg`
- [x] `src/drone_msgs/msg/K230RecordResult.msg`
- [x] `src/drone_msgs/msg/K230ScanPointDone.msg`
- [ ] `src/drone_control/include/drone_control/camera_aim_types.hpp`
- [x] `src/drone_control/config/*.yaml` 中的相关参数项

### 第二阶段可能再改

- [ ] 进一步细化 `record_success` 与 `result_state` 一致性校验
- [ ] 如有必要，补 `camera_aim_use_k230_targets` 兼容开关

---

## 7. 建议执行顺序

1. 先补消息定义和编译依赖
2. 在 `ActionExecutor` 中接入 `targets` 订阅
3. 完成帧校验和目标缓存
4. 改造 `executeCameraAim()`
5. 打通 `capture_ready` 发布
6. 更新 YAML 与日志
7. 最后再做仿真/联调验证

---

## 8. 验收标准

### 第一阶段通过标准

- [x] 控制端成功订阅 `/k230/animals/targets`
- [x] 非法帧不会触发飞行对准
- [x] 同一扫描点能建立目标队列
- [x] `camera_aim` 能选择一个目标进入 `tracking`
- [x] 达到阈值后发布一次 `/k230/animals/capture_ready`
- [x] 同一目标不会重复发布 `capture_ready`
- [x] 全过程没有新增第二个 setpoint 发布源

### 第二阶段通过标准

- [x] 能根据 `record_result` 进入 `captured / failed / skipped`
- [x] 当前扫描点所有目标终态后发布 `scan_point_done`
- [x] 空网格能在连续空帧后结束
- [x] `record_result` 超时后能跳过当前目标
- [x] 单网格总超时后能强制收口

---

## 9. 风险点

- `err_x/err_y` 到机体系位移的方向映射可能与当前相机安装方向不一致
- 新目标帧频率与控制周期不一致，可能导致抖动
- 扫描点切换时如果策略不清晰，容易出现旧目标未完成就被新目标打断
- 如果 `/k230/animals/targets` 因扫描范围 gating 长时间不转发，控制端只能按“首帧等待超时”退出，不能误判为空网格
- `err_x/err_y` 阈值当前仍共用 `DroneAction::getCameraAimTolerance()`，X/Y 独立容差尚未拆参

---

## 10. 后续建议

## 10. 今日完成情况

当前已完成的任务：

- 已完成任务 A 到任务 J 的主链路开发
- 已新增并接入以下消息
  - `K230CaptureReady.msg`
  - `K230RecordResult.msg`
  - `K230ScanPointDone.msg`
- 已在 `ActionExecutor` 内接入
  - `/k230/animals/targets`
  - `/k230/animals/record_result`
  - `/k230/animals/capture_ready`
  - `/k230/animals/scan_point_done`
- 已完成目标帧校验、目标缓存、目标选择、对准控制、拍照请求、拍照结果回流与扫描点结束
- 已补空网格、`record_result` 超时、单网格总超时等异常收口逻辑
- 已将新增参数补入 YAML 配置
- 已新增联调测试清单 `camera_aim_k230_integration_test_checklist.md`

当前控制端已具备的行为：

- 视觉端发布 `K230AnimalTargets`
- 控制端校验消息有效性
- 控制端建立当前 `scan_point` 目标队列
- 空网格在连续空帧确认后直接发布 `scan_point_done`
- 非空网格在 `camera_aim` 动作执行期间自动逐个处理目标
- `executeCameraAim()` 使用当前目标的 `err_x/err_y` 做闭环控制
- 对准稳定后发布 `/k230/animals/capture_ready`
- 发布后目标状态切到 `CAPTURE_REQUESTED`
- 视觉端回 `record_result` 后推进到 `CAPTURED / FAILED / SKIPPED`
- 当前目标丢失或等待结果超时时，仅跳过当前目标，不结束整个 `scan_point`
- 当前 `scan_point` 所有目标终态后发布 `/k230/animals/scan_point_done`
- 当前语义已更新为：一个 `camera_aim` 动作处理一个 `scan_point` 内的全部目标

## 11. 今日代码思路与流程

### 11.1 整体主线

当前代码把原来的单目标旧链路：

`/camera_aiming_center -> camera_aim_diff_ -> executeCameraAim()`

升级成了新的多目标闭环链路：

`/k230/animals/targets -> 校验 -> active_scan_point_ -> current_target -> executeCameraAim() -> capture_ready -> record_result -> scan_point_done`

### 11.2 消息输入层

- 视觉端通过 `/k230/animals/targets` 发布 `K230AnimalTargets`
- `ActionExecutor::k230TargetsCallback()` 接收消息
- 回调先做三项校验：
  - `scan_point_index >= 0`
  - `target_count == targets.size()`
  - 同一 `label` 下 `label_instance_id` 连续

### 11.3 状态缓存层

- `ActiveScanPointContext` 保存当前活动扫描点上下文
- `VisionTargetEntry` 保存单个目标的消息与状态
- `targets_by_key` 负责按 key 存目标
- `target_order` 负责按顺序选目标
- `current_target_key` 表示当前正在处理的单个目标
- `empty_targets_since` 用于空网格连续空帧确认
- `scan_point_start_time` 用于单网格总超时保护
- `capture_requested_time` 用于单目标等待结果超时保护

### 11.4 目标选择层

- 只有当前 `mission` 真正在执行 `camera_aim` 动作时，才会从 `pending` 中选目标
- 目标一旦被选中，就切到 `TRACKING`
- 当前实现语义为：一个 `camera_aim` 动作负责处理当前 `scan_point` 的全部目标

### 11.5 控制执行层

- `executeCameraAim()` 不再依赖旧的 `/camera_aiming_center`
- 它会依次处理以下分支：
  - 首帧 `/targets` 等待
  - 空网格连续空帧确认
  - 当前目标等待 `record_result`
  - 单网格总超时
  - 自动选择下一个 `PENDING`
  - 对当前 `TRACKING` 目标做 PID 闭环控制

### 11.6 异常处理层

- 如果当前目标长时间未更新：
  - 当前目标会被标记为 `SKIPPED`
  - 清空 `current_target_key`
  - 继续处理下一个目标
- 如果 `capture_ready` 后长时间收不到 `record_result`：
  - 当前目标会被标记为 `SKIPPED`
  - 清空 `current_target_key`
  - 继续处理下一个目标
- 如果一个 `scan_point` 长时间处理不完：
  - 所有未终态目标会被统一标记为 `SKIPPED`
  - 强制发布 `scan_point_done`

### 11.7 成功闭环层

- 当 `err_x / err_y` 连续若干周期落入阈值内：
  - 发布 `/k230/animals/capture_ready`
  - 目标状态切到 `CAPTURE_REQUESTED`
- 继续等待视觉端回传 `record_result`
- 当全部目标终态后：
  - 发布 `/k230/animals/scan_point_done`
  - 结束当前 `camera_aim` 动作

### 11.8 参数化结果

当前已经参数化的控制项：

- `camera_aim_target_timeout_s`
- `camera_aim_stable_cycles`
- `camera_aim_max_step`
- `camera_aim_wait_first_targets_timeout_s`
- `camera_aim_no_target_confirm_s`
- `camera_aim_record_result_timeout_s`
- `camera_aim_scan_point_timeout_s`

仍未参数化、后续可补的项：

- `camera_aim_err_x_tolerance`
- `camera_aim_err_y_tolerance`

## 12. 明日继续项

后续建议优先继续：

- [ ] 跑完 `camera_aim_k230_integration_test_checklist.md` 中的核心联调用例
- [ ] 视联调结果决定是否补 `record_success` 与 `result_state` 一致性校验
- [ ] 视联调结果决定是否拆分 `camera_aim_err_x_tolerance` / `camera_aim_err_y_tolerance`
- [ ] 视兼容需求决定是否补 `camera_aim_use_k230_targets` 模式开关

当前阶段结论：

- [x] 已确认 `capture_ready`、`record_result`、`scan_point_done` 消息与闭环流程
- [x] 已确认当前对准使用 `err_x/err_y`
- [x] 已确认当前 `camera_aim` 动作语义是“处理一个 `scan_point` 内的全部目标”

## 13. 调试命令

### 13.1 编译与环境加载

```bash
cd ~/drone_ws
colcon build --packages-select drone_msgs drone_control drone_mission drone_bringup
source /opt/ros/humble/setup.bash
source ~/drone_ws/install/setup.bash
```

### 13.2 启动 Gazebo 联调环境

```bash
ros2 launch drone_bringup run_offboard.launch.py enable_offboard_control:=true
```

### 13.3 启动任务

```bash
ros2 topic pub --once /start_mission std_msgs/msg/Empty "{}"
```

### 13.4 常用观察命令

查看控制端拍照请求：

```bash
ros2 topic echo /k230/animals/capture_ready
```

查看控制端扫描点完成通知：

```bash
ros2 topic echo /k230/animals/scan_point_done
```

查看控制端任务状态：

```bash
ros2 topic echo /mission_status
```

查看任务执行摘要：

```bash
ros2 topic echo /task/status
```

查看视觉输入：

```bash
ros2 topic echo /k230/animals/targets
```

查看视觉结果回流：

```bash
ros2 topic echo /k230/animals/record_result
```

查看接口是否已正确加载：

```bash
ros2 interface list | grep K230
ros2 interface show drone_msgs/msg/K230CaptureReady
ros2 interface show drone_msgs/msg/K230RecordResult
ros2 interface show drone_msgs/msg/K230ScanPointDone
```

### 13.5 空网格测试命令

```bash
ros2 topic pub -r 5 /k230/animals/targets drone_msgs/msg/K230AnimalTargets "{stamp: {sec: 0, nanosec: 0}, frame_seq: 1, scan_point_index: 0, scan_point_x: 0.0, scan_point_y: 0.0, target_count: 0, targets: []}"
```

预期：

- 不发布 `/k230/animals/capture_ready`
- 连续空帧超过 `camera_aim_no_target_confirm_s` 后发布 `/k230/animals/scan_point_done`

### 13.6 单目标成功测试命令

先持续发布一个偏差较大的目标：

```bash
ros2 topic pub -r 10 /k230/animals/targets drone_msgs/msg/K230AnimalTargets "{stamp: {sec: 0, nanosec: 0}, frame_seq: 2, scan_point_index: 0, scan_point_x: 0.0, scan_point_y: 0.0, target_count: 1, targets: [{label: 'tiger', label_instance_id: 1, score: 0.95, confirmed: true, stable_frames: 10, cx: 420, cy: 300, err_x: 100, err_y: 60, norm_x: 0.20, norm_y: 0.12, x1: 360, y1: 240, x2: 480, y2: 360, bbox_w: 120, bbox_h: 120, bbox_area: 14400}]}"
```

再切换成接近中心的目标：

```bash
ros2 topic pub -r 10 /k230/animals/targets drone_msgs/msg/K230AnimalTargets "{stamp: {sec: 0, nanosec: 0}, frame_seq: 3, scan_point_index: 0, scan_point_x: 0.0, scan_point_y: 0.0, target_count: 1, targets: [{label: 'tiger', label_instance_id: 1, score: 0.95, confirmed: true, stable_frames: 20, cx: 322, cy: 242, err_x: 2, err_y: 2, norm_x: 0.004, norm_y: 0.004, x1: 280, y1: 200, x2: 364, y2: 284, bbox_w: 84, bbox_h: 84, bbox_area: 7056}]}"
```

当控制端发出 `capture_ready` 后，回传成功结果：

```bash
ros2 topic pub --once /k230/animals/record_result drone_msgs/msg/K230RecordResult "{stamp: {sec: 0, nanosec: 0}, frame_seq: 3, scan_point_index: 0, label: 'tiger', label_instance_id: 1, record_success: true, result_state: 'captured', image_name: 'tiger_001.jpg'}"
```

### 13.7 单目标失败测试命令

```bash
ros2 topic pub --once /k230/animals/record_result drone_msgs/msg/K230RecordResult "{stamp: {sec: 0, nanosec: 0}, frame_seq: 3, scan_point_index: 0, label: 'tiger', label_instance_id: 1, record_success: false, result_state: 'failed', image_name: 'tiger_failed.jpg'}"
```

### 13.8 单目标跳过测试命令

```bash
ros2 topic pub --once /k230/animals/record_result drone_msgs/msg/K230RecordResult "{stamp: {sec: 0, nanosec: 0}, frame_seq: 3, scan_point_index: 0, label: 'tiger', label_instance_id: 1, record_success: false, result_state: 'skipped', image_name: ''}"
```

### 13.9 多目标测试命令

```bash
ros2 topic pub -r 10 /k230/animals/targets drone_msgs/msg/K230AnimalTargets "{stamp: {sec: 0, nanosec: 0}, frame_seq: 10, scan_point_index: 0, scan_point_x: 0.0, scan_point_y: 0.0, target_count: 3, targets: [{label: 'wolf', label_instance_id: 1, score: 0.92, confirmed: true, stable_frames: 12, cx: 322, cy: 242, err_x: 2, err_y: 2, norm_x: 0.004, norm_y: 0.004, x1: 280, y1: 200, x2: 364, y2: 284, bbox_w: 84, bbox_h: 84, bbox_area: 7056}, {label: 'wolf', label_instance_id: 2, score: 0.91, confirmed: true, stable_frames: 12, cx: 380, cy: 250, err_x: 60, err_y: 10, norm_x: 0.12, norm_y: 0.02, x1: 340, y1: 210, x2: 420, y2: 290, bbox_w: 80, bbox_h: 80, bbox_area: 6400}, {label: 'wolf', label_instance_id: 3, score: 0.90, confirmed: true, stable_frames: 12, cx: 260, cy: 220, err_x: -60, err_y: -20, norm_x: -0.12, norm_y: -0.04, x1: 220, y1: 180, x2: 300, y2: 260, bbox_w: 80, bbox_h: 80, bbox_area: 6400}]}"
```

依次回传结果：

```bash
ros2 topic pub --once /k230/animals/record_result drone_msgs/msg/K230RecordResult "{stamp: {sec: 0, nanosec: 0}, frame_seq: 10, scan_point_index: 0, label: 'wolf', label_instance_id: 1, record_success: true, result_state: 'captured', image_name: 'wolf_1.jpg'}"
ros2 topic pub --once /k230/animals/record_result drone_msgs/msg/K230RecordResult "{stamp: {sec: 0, nanosec: 0}, frame_seq: 10, scan_point_index: 0, label: 'wolf', label_instance_id: 2, record_success: false, result_state: 'failed', image_name: 'wolf_2.jpg'}"
ros2 topic pub --once /k230/animals/record_result drone_msgs/msg/K230RecordResult "{stamp: {sec: 0, nanosec: 0}, frame_seq: 10, scan_point_index: 0, label: 'wolf', label_instance_id: 3, record_success: false, result_state: 'skipped', image_name: ''}"
```

### 13.10 `record_result` 超时测试

操作：

- 先按“单目标成功测试命令”让控制端发布一次 `/k230/animals/capture_ready`
- 然后不要发送 `/k230/animals/record_result`

预期：

- 超过 `camera_aim_record_result_timeout_s` 后，当前目标进入 `SKIPPED`

### 13.11 scan point 总超时测试

持续发布目标，但不让所有目标收口：

```bash
ros2 topic pub -r 10 /k230/animals/targets drone_msgs/msg/K230AnimalTargets "{stamp: {sec: 0, nanosec: 0}, frame_seq: 20, scan_point_index: 0, scan_point_x: 0.0, scan_point_y: 0.0, target_count: 2, targets: [{label: 'elephant', label_instance_id: 1, score: 0.95, confirmed: true, stable_frames: 10, cx: 322, cy: 242, err_x: 2, err_y: 2, norm_x: 0.004, norm_y: 0.004, x1: 250, y1: 160, x2: 390, y2: 320, bbox_w: 140, bbox_h: 160, bbox_area: 22400}, {label: 'tiger', label_instance_id: 1, score: 0.94, confirmed: true, stable_frames: 10, cx: 360, cy: 260, err_x: 40, err_y: 20, norm_x: 0.08, norm_y: 0.04, x1: 320, y1: 220, x2: 400, y2: 300, bbox_w: 80, bbox_h: 80, bbox_area: 6400}]}"
```

然后不回完整结果，等待超过 `camera_aim_scan_point_timeout_s`。

### 13.12 非法帧测试命令

`target_count` 与 `targets.size()` 不一致：

```bash
ros2 topic pub --once /k230/animals/targets drone_msgs/msg/K230AnimalTargets "{stamp: {sec: 0, nanosec: 0}, frame_seq: 30, scan_point_index: 0, scan_point_x: 0.0, scan_point_y: 0.0, target_count: 2, targets: [{label: 'wolf', label_instance_id: 1, score: 0.9, confirmed: true, stable_frames: 8, cx: 320, cy: 240, err_x: 0, err_y: 0, norm_x: 0.0, norm_y: 0.0, x1: 280, y1: 200, x2: 360, y2: 280, bbox_w: 80, bbox_h: 80, bbox_area: 6400}]}"
```

`label_instance_id` 不连续：

```bash
ros2 topic pub --once /k230/animals/targets drone_msgs/msg/K230AnimalTargets "{stamp: {sec: 0, nanosec: 0}, frame_seq: 31, scan_point_index: 0, scan_point_x: 0.0, scan_point_y: 0.0, target_count: 2, targets: [{label: 'wolf', label_instance_id: 1, score: 0.9, confirmed: true, stable_frames: 8, cx: 320, cy: 240, err_x: 0, err_y: 0, norm_x: 0.0, norm_y: 0.0, x1: 280, y1: 200, x2: 360, y2: 280, bbox_w: 80, bbox_h: 80, bbox_area: 6400}, {label: 'wolf', label_instance_id: 3, score: 0.88, confirmed: true, stable_frames: 8, cx: 360, cy: 240, err_x: 40, err_y: 0, norm_x: 0.08, norm_y: 0.0, x1: 320, y1: 200, x2: 400, y2: 280, bbox_w: 80, bbox_h: 80, bbox_area: 6400}]}"
```
