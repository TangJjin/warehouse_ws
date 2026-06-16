# `camera_aim` 对接 K230 联调用例清单

## 1. 目标

验证 `drone_control` 当前 `camera_aim` 多目标闭环逻辑是否满足以下主流程：

`/k230/animals/targets -> 目标校验 -> scan point 管理 -> 目标选择 -> 对准 -> /k230/animals/capture_ready -> /k230/animals/record_result -> /k230/animals/scan_point_done`

---

## 2. 联调前置条件

- [ ] `colcon build --packages-select drone_msgs drone_control` 编译通过
- [ ] 控制端已加载最新 `action_executor.hpp`
- [ ] 已确认 `drone_msgs` 中以下消息可用
  - [ ] `K230AnimalTargets`
  - [ ] `K230AnimalTarget`
  - [ ] `K230CaptureReady`
  - [ ] `K230RecordResult`
  - [ ] `K230ScanPointDone`
- [ ] 已确认控制端参数已生效
  - [ ] `camera_aim_target_timeout_s`
  - [ ] `camera_aim_stable_cycles`
  - [ ] `camera_aim_max_step`
  - [ ] `camera_aim_wait_first_targets_timeout_s`
  - [ ] `camera_aim_no_target_confirm_s`
  - [ ] `camera_aim_record_result_timeout_s`
  - [ ] `camera_aim_scan_point_timeout_s`
- [ ] `camera_aim` 动作已在 mission 中可触发

---

## 3. 观测项

联调过程中重点观察以下内容：

- [ ] `/k230/animals/targets`
- [ ] `/k230/animals/capture_ready`
- [ ] `/k230/animals/record_result`
- [ ] `/k230/animals/scan_point_done`
- [ ] `/mission_status`
- [ ] 控制端日志中以下关键事件
  - [ ] 当前 scan point 激活
  - [ ] 当前目标选择
  - [ ] `capture_ready` 发布
  - [ ] `record_result` 更新
  - [ ] 当前目标跳过
  - [ ] 空网格判定
  - [ ] `scan_point_done` 发布
  - [ ] scan point 总超时强制结束

---

## 4. 基础用例

### 用例 1：首帧等待超时

目的：
验证进入 `camera_aim` 后，若长时间收不到 `/k230/animals/targets`，动作能按首帧等待超时退出。

输入：

- [ ] 触发一个 `camera_aim` 动作
- [ ] 在 `camera_aim_wait_first_targets_timeout_s` 时间内不转发任何 `/k230/animals/targets`

预期：

- [ ] 无 `capture_ready` 发布
- [ ] 无 `scan_point_done` 发布
- [ ] 控制端保持当前位置
- [ ] `camera_aim` 动作结束
- [ ] 日志包含“等待 `/k230/animals/targets` 首帧超时”

---

### 用例 2：空网格判定

目的：
验证已进入扫描点范围后，若持续收到空目标帧，控制端会将该网格判为空网格并结束。

输入：

- [ ] 触发一个 `camera_aim` 动作
- [ ] 持续发布同一 `scan_point_index` 的 `/k230/animals/targets`
- [ ] `target_count = 0`
- [ ] `targets = []`

预期：

- [ ] 控制端不发布 `capture_ready`
- [ ] 连续空帧超过 `camera_aim_no_target_confirm_s` 后发布 `scan_point_done`
- [ ] `camera_aim` 动作结束
- [ ] 日志包含“当前 scan point 连续空目标，已判定为空网格”

---

### 用例 3：单目标拍照成功

目的：
验证单目标正常闭环。

输入：

- [ ] 触发一个 `camera_aim` 动作
- [ ] 发布同一 `scan_point_index` 的非空 `/k230/animals/targets`
- [ ] 仅包含 1 个目标
- [ ] 控制误差逐步收敛到阈值内
- [ ] 控制端发布 `capture_ready` 后，视觉端回传 `record_result = captured`

预期：

- [ ] 当前目标进入 `TRACKING`
- [ ] 对准稳定后发布 1 次 `capture_ready`
- [ ] 当前目标状态进入 `CAPTURE_REQUESTED`
- [ ] 收到 `record_result` 后目标进入 `CAPTURED`
- [ ] 当前 scan point 所有目标终态后发布 `scan_point_done`
- [ ] `camera_aim` 动作结束

---

### 用例 4：单目标拍照失败

目的：
验证视觉端明确返回失败时，控制端状态收口正确。

输入：

- [ ] 同用例 3，直到 `capture_ready` 发布
- [ ] 视觉端回传 `record_result = failed`

预期：

- [ ] 当前目标状态进入 `FAILED`
- [ ] 若该 scan point 无其他目标，则发布 `scan_point_done`
- [ ] `camera_aim` 动作结束
- [ ] 日志包含目标结果更新为 `failed`

---

### 用例 5：单目标跳过

目的：
验证视觉端明确返回跳过时，控制端状态收口正确。

输入：

- [ ] 同用例 3，直到 `capture_ready` 发布
- [ ] 视觉端回传 `record_result = skipped`

预期：

- [ ] 当前目标状态进入 `SKIPPED`
- [ ] 若该 scan point 无其他目标，则发布 `scan_point_done`
- [ ] `camera_aim` 动作结束

---

## 5. 多目标用例

### 用例 6：同一网格多目标全部成功

目的：
验证一个 scan point 内多个目标可逐个处理。

输入：

- [ ] 触发一个 `camera_aim` 动作
- [ ] 发布同一 `scan_point_index` 下多个目标
  - [ ] 示例：3 个目标（如 3 只狼）
- [ ] 每个目标对准完成后都回传 `record_result = captured`

预期：

- [ ] 控制端一次只选择 1 个目标进入 `TRACKING`
- [ ] 第 1 个目标终态后自动切换到下一个 `PENDING`
- [ ] 所有目标都终态后才发布 `scan_point_done`
- [ ] 整个过程中只执行 1 次 `camera_aim` 动作，不需要 mission 追加多个 `camera_aim`

---

### 用例 7：多目标部分失败 / 部分跳过

目的：
验证部分目标失败或跳过时，仍能继续同一网格的剩余目标。

输入：

- [ ] 同一 `scan_point_index` 下发布多个目标
- [ ] 第 1 个目标返回 `captured`
- [ ] 第 2 个目标返回 `failed`
- [ ] 第 3 个目标返回 `skipped`

预期：

- [ ] 第 1 个目标进入 `CAPTURED`
- [ ] 第 2 个目标进入 `FAILED`
- [ ] 第 3 个目标进入 `SKIPPED`
- [ ] 所有目标终态后发布 `scan_point_done`

---

## 6. 超时与异常用例

### 用例 8：`record_result` 超时

目的：
验证发出 `capture_ready` 后，视觉端长期不返回结果时，控制端会跳过当前目标。

输入：

- [ ] 某个目标对准成功并发布 `capture_ready`
- [ ] 在 `camera_aim_record_result_timeout_s` 时间内不发送 `record_result`

预期：

- [ ] 当前目标被置为 `SKIPPED`
- [ ] 日志包含“目标等待 `record_result` 超时”
- [ ] 若有下一个 `PENDING` 目标，则继续处理下一个目标
- [ ] 若没有剩余目标，则尝试 `scan_point_done`

---

### 用例 9：当前目标视觉丢失

目的：
验证目标在对准阶段长时间不更新时，控制端会跳过当前目标，而不是结束整个 `camera_aim`。

输入：

- [ ] 当前已有 `TRACKING` 目标
- [ ] 停止更新该目标，超过 `camera_aim_target_timeout_s`

预期：

- [ ] 当前目标被置为 `SKIPPED`
- [ ] 日志包含“当前视觉目标超时丢失”
- [ ] 若有后续 `PENDING` 目标，则继续处理下一个目标
- [ ] 不应直接结束整个 `camera_aim`

---

### 用例 10：scan point 总超时

目的：
验证一个网格处理时间过长时，控制端会强制收口。

输入：

- [ ] 构造多个目标或让某些目标迟迟不进入终态
- [ ] 让 scan point 处理时间超过 `camera_aim_scan_point_timeout_s`

预期：

- [ ] 所有未终态目标被置为 `SKIPPED`
- [ ] 发布 `scan_point_done`
- [ ] `camera_aim` 动作结束
- [ ] 日志包含“当前 scan point 处理总时长已超过”

---

## 7. 消息有效性用例

### 用例 11：非法 `scan_point_index`

输入：

- [ ] 发布 `scan_point_index < 0` 的 `/k230/animals/targets`

预期：

- [ ] 本帧被丢弃
- [ ] 不触发目标缓存
- [ ] 不触发 `capture_ready`
- [ ] 日志包含校验失败原因

---

### 用例 12：`target_count` 与 `targets.size()` 不一致

输入：

- [ ] 发布 `target_count != targets.size()` 的 `/k230/animals/targets`

预期：

- [ ] 本帧被丢弃
- [ ] 不触发目标缓存
- [ ] 日志包含校验失败原因

---

### 用例 13：`label_instance_id` 不连续

输入：

- [ ] 同一 `label` 下故意发布不连续的 `label_instance_id`
  - [ ] 例如只发 `1, 3`

预期：

- [ ] 本帧被丢弃
- [ ] 不触发目标缓存
- [ ] 日志包含 `label_instance_id` 不连续告警

---

## 8. 联调完成标准

- [ ] 空网格能稳定发布 `scan_point_done`
- [ ] 单目标闭环可走通
- [ ] 多目标闭环可走通
- [ ] `record_result` 超时可跳过当前目标
- [ ] 当前目标丢失不会结束整个 `camera_aim`
- [ ] 单网格总超时可强制收口
- [ ] 非法帧不会触发错误控制
- [ ] 全过程仅由 `ActionExecutor` 发布 `/mavros/setpoint_position/local`

---

## 9. 联调记录

### 本轮环境

- 日期：
- 分支/代码版本：
- mission 配置：
- 视觉端版本：

### 测试结果

- 用例 1：
- 用例 2：
- 用例 3：
- 用例 4：
- 用例 5：
- 用例 6：
- 用例 7：
- 用例 8：
- 用例 9：
- 用例 10：
- 用例 11：
- 用例 12：
- 用例 13：

### 遗留问题

- 

