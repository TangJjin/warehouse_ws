# `camera_aim` 控制端交付清单

## 1. 交付结论

对照视觉端交接文档 [k230_current_vision_and_control_handoff(1).md](</home/bosen/桌面/k230_current_vision_and_control_handoff(1).md:1>)，当前控制端主链路已经**基本完成交付**。

当前已完成的控制端闭环为：

`/k230/animals/targets -> 校验 -> scan point / target 队列 -> 对准 -> /k230/animals/capture_ready -> /k230/animals/record_result -> /k230/animals/scan_point_done`

结论：

- [x] 已完成视觉端要求的 `targets` 订阅与校验
- [x] 已完成 `capture_ready` 发布
- [x] 已完成 `record_result` 订阅
- [x] 已完成 `scan_point_done` 发布
- [x] 已完成多目标与空网格场景的控制端状态机
- [ ] 尚未完成真实联调验收
- [ ] 尚未补充少量非核心增强项（如一致性校验、兼容开关）

---

## 2. 已完成内容

### 2.1 目标列表接入

控制端已订阅：

- `/k230/animals/targets`
- 消息类型：`drone_msgs/msg/K230AnimalTargets`

已实现的校验：

- [x] `scan_point_index >= 0`
- [x] `target_count == targets.size()`
- [x] 同一 `label` 下 `label_instance_id` 连续

非法帧处理策略：

- [x] 本帧直接丢弃
- [x] 打印 `WARN`
- [x] 不进入目标缓存

---

### 2.2 控制端目标缓存与状态机

控制端内部以 `scan_point_index` 为单位维护当前活动扫描点。

当前目标 key：

```text
scan_point_index + label + label_instance_id
```

当前目标状态：

```text
PENDING
TRACKING
CAPTURE_REQUESTED
CAPTURED
FAILED
SKIPPED
```

当前语义：

- 一个 `camera_aim` 动作负责处理**一个 scan point 内的全部目标**
- 不再是“一个 `camera_aim` 只处理一个目标”

---

### 2.3 `capture_ready` 发布

控制端已发布：

- 话题：`/k230/animals/capture_ready`
- 消息类型：`drone_msgs/msg/K230CaptureReady`

消息字段：

```text
uint32 frame_seq
int32 scan_point_index
string label
uint32 label_instance_id
bool capture_ready
```

当前发布语义：

- 当前目标连续若干周期满足对准阈值后发布
- 同一目标只发布一次
- 发布后当前目标进入 `CAPTURE_REQUESTED`

---

### 2.4 `record_result` 回流处理

控制端已订阅：

- 话题：`/k230/animals/record_result`
- 消息类型：`drone_msgs/msg/K230RecordResult`

当前处理语义：

- `captured -> CAPTURED`
- `failed -> FAILED`
- `skipped -> SKIPPED`

匹配方式：

- `scan_point_index`
- `label`
- `label_instance_id`

收到结果后：

- 若当前目标进入终态，则清空 `current_target_key`
- 若仍有 `PENDING` 目标，则继续处理下一个
- 若当前 scan point 所有目标终态，则发布 `scan_point_done`

---

### 2.5 `scan_point_done` 发布

控制端已发布：

- 话题：`/k230/animals/scan_point_done`
- 消息类型：`drone_msgs/msg/K230ScanPointDone`

消息字段：

```text
builtin_interfaces/Time stamp
int32 scan_point_index
bool scan_point_done
```

当前发布语义：

- 当前 scan point 内所有目标都进入终态后发布
- 空网格确认后也会发布
- 每个 scan point 只发布一次

---

## 3. 当前 `camera_aim` 行为说明

### 3.1 空网格

若视觉端在已进入 scan point 范围后持续发布：

```text
target_count = 0
targets = []
```

控制端行为：

- 原地保持当前位置
- 连续空帧超过 `camera_aim_no_target_confirm_s`
- 判定为空网格
- 发布 `/k230/animals/scan_point_done`
- 结束当前 `camera_aim`

### 3.2 单目标

控制端行为：

- 选该目标进入 `TRACKING`
- 使用 `err_x / err_y` 做闭环对准
- 满足阈值后发布 `capture_ready`
- 等待 `record_result`
- 目标进入终态后发布 `scan_point_done`

### 3.3 多目标

控制端行为：

- 一个 scan point 内的目标按队列逐个处理
- 当前目标结束后自动选下一个 `PENDING`
- 所有目标终态后发布 `scan_point_done`

---

## 4. 已实现的超时与异常收口

### 4.1 首帧等待超时

参数：

```text
camera_aim_wait_first_targets_timeout_s
```

行为：

- 进入 `camera_aim` 后若长时间收不到 `/k230/animals/targets`
- 结束当前 `camera_aim`
- 不误判为空网格

### 4.2 当前目标视觉丢失

参数：

```text
camera_aim_target_timeout_s
```

行为：

- 当前 `TRACKING` 目标长时间未更新
- 当前目标进入 `SKIPPED`
- 继续处理后续目标

### 4.3 等待 `record_result` 超时

参数：

```text
camera_aim_record_result_timeout_s
```

行为：

- 当前目标已进入 `CAPTURE_REQUESTED`
- 长时间收不到 `record_result`
- 当前目标进入 `SKIPPED`
- 继续处理后续目标

### 4.4 单网格总超时

参数：

```text
camera_aim_scan_point_timeout_s
```

行为：

- 当前 scan point 总处理时间超过上限
- 所有未终态目标强制置为 `SKIPPED`
- 发布 `scan_point_done`
- 结束当前 `camera_aim`

---

## 5. 当前使用的主要参数

当前已接入参数：

```text
camera_aim_target_timeout_s
camera_aim_stable_cycles
camera_aim_max_step
camera_aim_wait_first_targets_timeout_s
camera_aim_no_target_confirm_s
camera_aim_record_result_timeout_s
camera_aim_scan_point_timeout_s
```

当前示例 YAML 已更新：

- [ground_mission.yaml](/home/bosen/drone_ws/src/drone_mission/config/ground_mission.yaml:1)
- [sample.yaml](/home/bosen/drone_ws/src/drone_control/config/sample.yaml:1)

---

## 6. 对视觉端的接口说明

### 6.1 视觉端发给控制端

- `/k230/animals/targets`
- `/k230/animals/record_result`

### 6.2 控制端发给视觉端

- `/k230/animals/capture_ready`
- `/k230/animals/scan_point_done`

当前已满足交接文档中 5.1、5.2、5.3 提到的核心接口要求。

---

## 7. 目前仍建议注意的点

以下内容不阻塞本次控制端交付，但建议联调时确认：

- [ ] `record_success` 与 `result_state` 的一致性当前未做强校验
- [ ] `camera_aim_err_x_tolerance` / `camera_aim_err_y_tolerance` 尚未拆分独立参数
- [ ] 旧 `/camera_aiming_center` 订阅仍保留在代码中，但当前主逻辑已切到 `/k230/animals/targets`
- [ ] 是否需要增加 `camera_aim_use_k230_targets` 兼容开关尚未决定

---

## 8. 当前是否可交付给队友

可以交付，但结论应写清楚：

- **控制端代码主功能已完成**
- **接口层面已满足当前视觉端交接要求**
- **当前最主要剩余工作是联调验证，不是主功能补代码**

建议随本清单一起交付给队友的文件：

- [camera_aim_k230_task_checklist.md](/home/bosen/drone_ws/src/drone_control/camera_aim_k230_task_checklist.md:1)
- [camera_aim_k230_integration_test_checklist.md](/home/bosen/drone_ws/src/drone_control/camera_aim_k230_integration_test_checklist.md:1)

---

## 9. 建议队友联调重点

建议视觉端/控制端联调时优先验证以下 5 条：

- [ ] 空网格：持续空帧后能发布 `scan_point_done`
- [ ] 单目标：`capture_ready -> record_result(captured) -> scan_point_done`
- [ ] 多目标：同一 scan point 内多个目标可逐个处理
- [ ] `record_result` 超时：当前目标 `SKIPPED`
- [ ] 单网格总超时：剩余目标强制收口并发布 `scan_point_done`

