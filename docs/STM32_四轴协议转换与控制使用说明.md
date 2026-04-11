# STM32 四轴 Y42 CAN 协议转换与控制使用说明

## 1. 目标与架构

本工程实现了以下链路：

- ROS 上位机 <-> CAN1 <-> STM32F407VET6
- STM32F407VET6 <-> CAN2 <-> 4 个 ZDT Y42 步进驱动器

STM32 的角色：

- 接收 ROS 统一控制帧（标准帧 ID = 0x100）
- 转换为 Y42 驱动协议（扩展帧 ID = (Addr << 8) | Packet）
- 管理四轴控制任务、状态轮询、心跳保护
- 通过 CAN1 标准帧 ID = 0x101 上报状态给 ROS
- 通过 CAN1 标准帧 ID = 0x102 发送命令回执
- 通过 CAN1 标准帧 ID = 0x103 发送通信统计

---

## 2. 关键文件与作用

### 2.1 协议与通信

- `Core/Inc/can_protocol.h`
  - ROS 控制帧定义（0x100）
  - ROS 指令字段说明

- `Core/Src/can_protocol.c`
  - CAN1 接收解析入口 `CAN1_RxCallback`
  - 将 ROS 指令转为 `Motor_Command_t` 入队

- `Core/Inc/zdt_can_driver.h`
  - Y42 协议功能码定义
  - 扩展帧分包规则
  - 通信接口声明（速度/位置/急停/同步/读取/AA 多机命令）

- `Core/Src/zdt_can_driver.c`
  - CAN2 扩展帧拆包发送
  - 响应匹配与读取回调
  - 超时重传
  - 应答策略切换
  - 等待应答请求队列调度

### 2.2 电机控制

- `Core/Inc/motor_control.h`
  - 四轴状态结构体 `motors[4]`
  - 控制接口（速度、位置、配置、同步触发、多机命令透传）

- `Core/Src/motor_control.c`
  - 四轴控制逻辑
  - 速度/位置/状态轮询
  - 命令队列消费
  - 心跳超时急停

### 2.3 系统集成

- `Core/Src/main.c`
  - CAN 初始化、过滤器、通知启用
  - 三个任务启动：MotorControl/Status/Heartbeat
  - 0x101 状态上报
  - 0x06 紧急事件上报

- `Core/Src/stm32f4xx_it.c`
  - CAN1/CAN2 IRQ 分发
  - Rx FIFO0 回调分别转入 ROS 解析和驱动响应处理

---

## 3. ROS 控制帧定义（CAN1, ID=0x100）

数据长度固定 8 字节：

- byte0: cmd
- byte1: motor_idx (0~3，或 0xFF 表示全部电机)
- byte2~5: value（int32，小端）
- byte6: param0
- byte7: param1

### 3.1 指令清单

- `0x01` 设速度
  - value: 目标速度（rpm，int32，正负表示方向）
  - param0: accel 档位（0~255）
  - param1: sync 标志（0 立即执行，1 缓存，等待同步触发）

- `0x02` 设位置（Emm 位置模式）
  - value: 脉冲数（int32，正负表示方向）
  - param0: accel 档位（0~255）
  - param1 bit0~1: 运动模式（0 相对上一目标，1 绝对坐标，2 相对当前位置）
  - param1 bit2: sync 标志（0 立即执行，1 缓存）

- `0x03` 急停
  - value/param 忽略

- `0x04` 设置速度档位配置（后续位置模式会使用）
  - value: speed_rpm（建议 0~3000）
  - param0: accel 档位

- `0x05` 触发同步开始
  - 对已缓存的同步命令统一触发（对应驱动 `FF 66`）

- `0x06` 设置应答策略
  - param0 = 1: 等待应答（默认）
  - param0 = 0: fire-and-forget（不等待应答，吞吐更高）

- `0x07` 设置状态上报掩码
  - value bit0: 基础状态(type=0x01)
  - value bit1: 实时位置(type=0x02)
  - value bit2: 实时速度(type=0x03)
  - value bit3: 目标值(type=0x04 + type=0x05)
  - 例如 value=0x07 表示上报 基础+位置+速度

- `0x08` 设置四轴地址映射
  - value 按 little-endian 拆成 4 个字节
  - byte0: motor0 对应驱动地址
  - byte1: motor1 对应驱动地址
  - byte2: motor2 对应驱动地址
  - byte3: motor3 对应驱动地址
  - 例如 value = 0x04030201 表示 idx0~3 对应地址 1/2/3/4

---

## 4. 驱动侧协议映射（CAN2）

### 4.1 帧类型

- 扩展帧（IDE=EXT）
- ExtID = `(slave_addr << 8) | packet_index`
- packet_index 从 0 开始，长命令自动拆包

### 4.2 已对齐功能码

- `F3` 电机使能
- `F6` 速度模式（Emm）
- `FD` 位置模式（Emm）
- `FE` 立即停止
- `FF` 触发多机同步运动
- `35` 读取实时转速
- `36` 读取实时位置
- `3A` 读取电机状态标志
- `AA` 多电机命令（广播）

### 4.3 速度单位说明

- ROS 下发速度命令 `0x01` 和位置模式速度配置 `0x04` 仍以 RPM 表示。
- STM32 在发送到 Y42 驱动时，会自动转换为 0.1 RPM 单位。
- CAN1 状态上报中：
  - `type=0x03` 实时速度使用 0.1 RPM 单位
  - `type=0x04` 目标速度使用 0.1 RPM 单位

### 4.4 超时重传与排队

- 挂起请求超时：20ms
- 最大重试：3 次
- 当前实现为“1 条在途请求 + 小型等待队列”模型
- 前一条请求收到应答或超时清除后，下一条自动派发

---

## 5. 四轴控制接口（程序内）

以下接口可直接在业务代码中调用：

- `motor_set_velocity(idx, vel)`
- `motor_set_velocity_ex(idx, vel, accel, sync)`
- `motor_set_position(idx, pulse, ...)`
- `motor_set_position_ex(idx, pulse, accel, mode, sync)`
- `motor_set_speed_profile(idx, speed_rpm, accel)`
- `motor_trigger_sync_motion()`
- `motor_stop_all()`
- `motor_send_multi_cmd(cmd_stream, len)`
- `motor_set_response_policy(wait_ack)`

四轴索引与驱动地址映射：

- idx 0 -> addr 1
- idx 1 -> addr 2
- idx 2 -> addr 3
- idx 3 -> addr 4

说明：

- 上述为默认映射。
- 运行时可通过 ROS 命令 `0x08` 重配置四轴地址映射。

---

## 6. AA 多电机命令使用方案

接口：

- `zdt_send_multi_motor_command(const uint8_t *cmd_stream, uint16_t stream_len)`
- `motor_send_multi_cmd(const uint8_t *cmd_stream, uint16_t stream_len)`

cmd_stream 是若干条子命令拼接，不含头尾，函数内部自动封装：

- 头：`00 AA len_hi len_lo`
- 尾：`6B`

示例场景（四轴同时下发不同动作）：

- 电机1 位置命令（FD ...）
- 电机2 速度命令（F6 ...）
- 电机3 位置命令（FD ...）
- 电机4 读取位置（36 6B）

将以上子命令拼成 `cmd_stream` 一次发送，可减少总线占用并提升同步性。

---

## 7. 任务与周期

- MotorControl_Task（10ms）
  - 处理 ROS 命令队列
  - 执行控制命令
  - 执行超时重传轮询

- Status_Task（50ms）
  - 分相轮询 4 轴状态/位置/速度
  - 上报 ROS 状态帧（0x101）

- Heartbeat_Task（20ms）
  - 监控 ROS 心跳
  - 超时触发 `motor_stop_all()`

---

## 8. 状态上报（CAN1, ID=0x101）

`0x101` 按子帧 type 分流，且由 `0x07` 指令按位控制是否发送：

- 子帧 `type=0x01`
  - 电机状态标志、故障标志等

- 子帧 `type=0x02`
  - 实时位置反馈（int32）

- 子帧 `type=0x03`
  - 实时速度反馈（int32，单位 0.1 RPM）

- 子帧 `type=0x04`
  - 目标速度（int32，单位 0.1 RPM）

- 子帧 `type=0x05`
  - 目标位置（int32）

- 子帧 `type=0x06`
  - 紧急事件
  - byte1: motor_idx
  - byte2: status_flags
  - byte3~6: 故障发生时速度（int32，单位 0.1 RPM）

建议 ROS 端按 `byte0` 的 type 分流解析。

### 8.1 命令回执（CAN1, ID=0x102）

- byte0~1: seq（STM32 接收命令后分配的递增序号）
- byte2: cmd
- byte3: result（0 成功，1 失败）

建议 ROS 端记录 seq 连续性，用于检测命令处理链路是否出现拥塞/丢失。

### 8.2 通信统计（CAN1, ID=0x103）

`0x103` 周期上报两种子帧：

- type=0x01
  - rx_count（接收命令数，16bit）
  - enqueue_drop_count（队列拥塞丢弃，16bit）
  - timeout_drop_count（超时重试耗尽丢弃，16bit）

- type=0x02
  - exec_ok_count（执行成功数，16bit）
  - exec_fail_count（执行失败数，16bit）
  - last_seq（最近命令序号，16bit）
  - last_result（最近结果）

---

## 9. 上电联调步骤

1. 确认 4 个驱动地址分别为 1/2/3/4，或准备通过 `0x08` 下发实际地址映射。  
2. 驱动 CAN 口配置正确（协议与波特率与当前固件一致）。  
3. 上电后观察 STM32 是否正常启动 3 个任务。  
4. ROS 发送 `0x04` 设置每轴速度档位。  
5. 发送 `0x01` 或 `0x02` 指令测试单轴。  
6. 测试 `0x05` 同步触发（先缓存，后触发）。  
7. 测试 `0x03` 急停与心跳超时保护。  

---

## 10. 常见问题排查

### 10.1 电机不动作

- 检查驱动是否使能（上电初始化已发送 F3）。
- 检查地址是否与 idx 映射一致。
- 检查 CAN2 是否为扩展帧收发。
- 检查应答策略是否设置为 fire-and-forget 导致无法从应答判断结果。

### 10.2 速度显示与上位机不一致

- 检查 ROS 端是否把 `0x101` 的 `type=0x03` 和 `type=0x04` 按 0.1 RPM 解析。
- 检查上位机是否错误地把驱动侧速度字段当作 RPM 原值使用。

### 10.3 只收到部分状态

- 检查 CAN1 上报带宽与 ROS 接收处理速度。
- 检查 Status_Task 周期是否被其他高优先级任务挤占。

### 10.4 同步运动不一致

- 确保控制命令的 sync 标志为 1（缓存）。
- 确保最后发送了 `0x05` 触发同步。
- 多条命令之间建议保留几毫秒间隔，避免设备侧粘包。

---

## 11. 建议控制方案（四轴）

### 方案 A：实时速度控制

- 上位机周期发送 `0x01`（每轴或广播）
- 使用 `param0` 动态调节加速度
- 心跳超时自动急停

适合：底盘、连续轨迹速度控制。

### 方案 B：位置步进控制 + 同步触发

- 先用 `0x04` 设置各轴速度档位
- 用 `0x02` 下发各轴位置（sync=1）
- 用 `0x05` 同步触发

适合：机械臂、平台同步定位、四轴协调动作。

### 方案 C：AA 多机批量命令

- 使用 `motor_send_multi_cmd()` 一次打包多轴不同命令
- 减少总线帧数，提升同步性

适合：高频多轴复合动作。

---

## 12. 二次开发建议执行结果

已按顺序执行并落地：

1. 命令序号与回执统计
  - 已新增 `0x102` ACK 与 `0x103` 统计帧。
  - 固件为每条入队命令分配递增 seq，并在执行后回执 result。

2. 0x101 可配置上报
  - 已新增 `0x07` 指令按位配置上报内容。
  - 支持基础状态/位置/速度/目标值的组合上报。

3. AA 上位机打包器
  - 已新增脚本：`tools/aa_packer.py`
  - 可拼装 speed/position/stop/read-* 子命令并输出 hex 或 C 数组。

### 12.1 AA 打包器示例

命令示例（输出 cmd_stream 十六进制）：

```bash
python tools/aa_packer.py \
  --speed 1 1200 10 0 \
  --position 2 20000 800 12 0 1 \
  --read-pos 3 \
  --out hex
```

如果需要输出完整 AA 封装（`AA + len + stream + 6B`）：

```bash
python tools/aa_packer.py \
  --speed 1 1200 10 0 \
  --position 2 20000 800 12 0 1 \
  --wrap-aa \
  --out c
```
