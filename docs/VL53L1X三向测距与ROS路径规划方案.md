# VL53L1X 三向测距与 ROS 路径规划方案

## 1. 目标与边界

本方案为现有 `STM32F407VET6` 电机控制器增加 3 个 VL53L1X ToF 测距传感器：

| 传感器 ID | 安装方向 | ROS frame_id | 主要用途 |
|---:|---|---|---|
| 0 | 前方 | `front_range_link` | 前向障碍物预警、急停、局部避障 |
| 1 | 左侧 | `left_range_link` | 左侧净空检测、局部避障 |
| 2 | 右侧 | `right_range_link` | 右侧净空检测、局部避障 |

系统目标如下：

1. STM32 周期采集三路距离，完成数据有效性判定和滤波。
2. STM32 经现有 CAN1（`500 kbit/s`）把测距数据上报 ROS 主机。
3. ROS 将数据转换为标准 `sensor_msgs/Range` 和诊断信息，供局部避障使用。
4. 距离过近时，STM32 不等待 ROS 决策，直接执行本地减速/急停保护。
5. 完整建图、定位和路径规划由 ROS 主机负责；STM32 不处理激光雷达或 Nav2 算法。

> 三个单点 ToF 只能提供近距离、三个方向的障碍物信息，**不能替代 2D 激光雷达**。若需要可靠的全局建图和导航，应额外在 ROS 主机上接入 2D 激光雷达，并使用本方案作为冗余防撞层。

当前工程已有链路：`ROS 主机 <-> CAN1 <-> STM32 <-> CAN2 <-> Y42 驱动器`。CAN1 命令 ID 为 `0x100`，状态 ID 为 `0x101`，因此测距数据应使用独立 ID，不能混入电机状态帧。

---

## 2. 硬件方案

### 2.1 I2C 总线

在 STM32CubeMX 中启用 `I2C1`：

- 固定引脚：`PB8 = I2C1_SCL`、`PB9 = I2C1_SDA`；固件与 `.ioc` 已按此配置。
- I2C 速率：先使用 `400 kHz`；线缆较长、干扰明显时降至 `100 kHz`。
- 所有模块与 STM32 必须共地。
- SCL/SDA 只保留**一组**到 `3.3 V` 的上拉电阻，典型值 `4.7 kΩ`。确认模块本身是否已经带上拉，避免多组并联导致上拉过强。
- 仅使用 I/O 为 `3.3 V` 兼容的 VL53L1X 模块。原始芯片电压不是 3.3 V；若使用裸模块，必须确认稳压和电平转换设计。

### 2.2 XSHUT 引脚与 I2C 地址

三个 VL53L1X 默认 I2C 地址都为 `0x29`，不能同时上电后直接挂在同一总线上。每个模块必须连接独立的 `XSHUT` 到 STM32 空闲 GPIO：

```text
STM32 GPIO_XSHUT_FRONT -> 前向 VL53L1X XSHUT
STM32 GPIO_XSHUT_LEFT  -> 左侧 VL53L1X XSHUT
STM32 GPIO_XSHUT_RIGHT -> 右侧 VL53L1X XSHUT
```

本实现固定使用 `PE0 = XSHUT_FRONT`、`PE1 = XSHUT_LEFT`、`PE2 = XSHUT_RIGHT`。它们不与当前 CAN、PWM、编码器或 SWD 配置冲突，但接线前仍须确认这三个引脚已在实际板卡引出。不要复用 PA11/PA12（CAN1）、PB12/PB13（CAN2）、PB0/PD14（PWM）、PE9/PE11（编码器）、PA13/PA14（SWD）。

启动时的地址分配流程：

1. 将 3 个 `XSHUT` 全部拉低，使传感器复位。
2. 仅拉高前向传感器的 `XSHUT`，等待其启动；以默认地址 `0x29` 初始化后改为 `0x30`。
3. 保持前向传感器工作，拉高左侧传感器；以 `0x29` 初始化并改为 `0x31`。
4. 保持前两者工作，拉高右侧传感器；以 `0x29` 初始化并改为 `0x32`。
5. 三路传感器均以连续测距模式运行。

建议常量：

```c
#define VL53L1X_ADDR_FRONT  0x30U
#define VL53L1X_ADDR_LEFT   0x31U
#define VL53L1X_ADDR_RIGHT  0x32U
```

地址在掉电或 XSHUT 复位后会恢复默认值，因此每次上电都必须重新执行上述分配过程。

### 2.3 机械安装

- 前方传感器朝机器人正前方，光轴应近似水平。
- 左右传感器朝相应侧向，避免被车体外壳、轮胎或线缆遮挡。
- 三个模块安装高度尽量一致，并记录各自相对 `base_link` 的位置和朝向，供 ROS 发布静态 TF。
- 避免透明玻璃、黑色吸光材料、强日光和多径反射环境；这些情况会降低 ToF 可靠性。
- VL53L1X 存在近距离盲区和有限视场角。机器人倒车方向没有传感器覆盖，倒车必须限速；如需无人值守倒车，应补充后向传感器。

---

## 3. STM32 固件设计

### 3.1 新增文件与构建配置

新增独立模块，避免把传感器状态堆积在 `main.c`：

```text
Core/Inc/distance_sensor.h
Core/Src/distance_sensor.c
```

在 `CMakeLists.txt` 的 `target_sources` 中加入：

```cmake
Core/Src/distance_sensor.c
```

CubeMX 重新生成前，确认自定义代码位于 `USER CODE BEGIN/END` 区域，且工程保持 `ProjectManager.KeepUserCode=true`。

当前工程的 HAL 驱动目录不包含 I2C 源文件，因此 `distance_sensor.c` 以 STM32F407 I2C1 寄存器实现 `400 kHz` 轮询总线，未依赖 `HAL_I2C_*`。`.ioc` 仍记录 I2C1 和引脚配置；若未来通过 CubeMX 重新生成并补齐 HAL I2C 驱动，必须保留 `distance_sensor.c` 的 PB8/PB9 配置，且不能让生成的 I2C 初始化与该模块同时改写 I2C1 寄存器。

`distance_sensor.h` 建议暴露以下接口：

```c
#define DISTANCE_SENSOR_COUNT 3U

typedef enum {
    DIST_SENSOR_FRONT = 0U,
    DIST_SENSOR_LEFT = 1U,
    DIST_SENSOR_RIGHT = 2U
} distance_sensor_id_t;

typedef struct {
    uint16_t distance_mm;
    uint16_t sigma_mm;
    uint32_t timestamp_ms;
    uint8_t sensor_id;
    uint8_t valid;
    uint8_t status;
    uint8_t sequence;
} distance_sensor_sample_t;

HAL_StatusTypeDef distance_sensor_init(I2C_HandleTypeDef *hi2c);
void distance_sensor_poll(void);
uint8_t distance_sensor_get_latest(uint8_t sensor_id,
                                   distance_sensor_sample_t *sample);
uint8_t distance_sensor_is_emergency(void);
```

### 3.2 VL53L1X 驱动和采样策略

优先使用 ST 的 VL53L1X 官方 ULD（Ultra Lite Driver）或经过验证的 HAL 移植驱动。驱动必须支持：

- 设备初始化；
- 修改 I2C 地址；
- 启动/停止连续测距；
- 检查 `data_ready`；
- 读取距离、量测状态和质量指标；
- 清除中断/启动下一次测量。

初始参数建议：

| 参数 | 建议初值 | 说明 |
|---|---:|---|
| 测距模式 | Long 或 Short，现场验证后确定 | Long 量程更长，强光/串扰环境可能需要 Short |
| Timing budget | 20–33 ms | 优先稳定性；不要一开始追求极高频率 |
| 单路目标采样率 | 20 Hz | 三路总计约 60 CAN 帧/s |
| 最大有效距离 | 3000 mm | 作为初始软件上限，按场地调整 |
| 最小有效距离 | 50 mm | 需按实际模块盲区标定 |
| 滤波 | 最近 3 或 5 个有效样本中值滤波 | 抑制单次跳变，避免平均值拖慢急停 |

推荐在初始化结束后让三路传感器并行连续测量，并在 `SensorTask` 中轮询读取已完成的数据。不要在中断、CAN 回调或 `MotorCtrlTask` 中执行 I2C 读操作。

### 3.3 FreeRTOS 任务

在 `Core/Src/main.c` 增加 `SensorTask`：

| 任务 | 周期 / 优先级 | 职责 |
|---|---|---|
| `MotorCtrlTask` | 10 ms / 高 | 处理电机命令；不得被传感器阻塞 |
| `HeartbeatTask` | 20 ms / 高 | 上位机掉线停车 |
| `SensorTask` | 10–20 ms / Normal | 轮询三路 ToF、滤波、上报、本地安全判断 |
| `StatusTask` | 20 ms / Normal | 驱动状态查询和电机状态上报 |

建议 `SensorTask` 使用 `256–384 * 4` 字节栈起步，并在目标板上监测剩余栈空间。伪代码：

```c
for (;;) {
    distance_sensor_poll();

    for (uint8_t id = 0; id < DISTANCE_SENSOR_COUNT; ++id) {
        if (distance_sensor_get_new_sample(id, &sample)) {
            CAN1_SendDistance(&sample);
        }
    }

    if (distance_sensor_is_emergency()) {
        (void)motor_stop_all();
        CAN1_SendDistanceEmergency(...);
    }

    osDelay(10U);
}
```

> 当前 `Core/Src/main.c` 中 `StatusTask` 的创建语句被注释。启用或恢复该任务前，必须检查其 CAN1 帧率与传感器帧率，实机观察 `can1_tx_fail_count`；不要盲目同时提高所有状态上报频率。

### 3.4 CAN1 多任务发送

当前 `CAN1_TrySend()` 可能被多个任务调用。传感器任务加入后，应确保 CAN1 发送访问串行化：

- 推荐方式：建立 CAN1 发送消息队列和单独的 `CanTxTask`；电机命令回执、状态、传感器数据都入队。
- 最小修改方式：使用 CMSIS-RTOS mutex 保护 `HAL_CAN_AddTxMessage()`。
- 电机急停、回执和故障事件优先级必须高于周期性距离数据。
- 周期性距离帧允许丢弃旧样本，**不应阻塞电机控制任务等待发送邮箱**。

CAN 标准帧 ID 的数值越小，仲裁优先级越高。保留已有控制 `0x100`、状态 `0x101` 的优先级；距离数据使用更低优先级的 `0x110`。

---

## 4. CAN1 测距协议

### 4.1 新增 CAN ID

在 `Core/Inc/can_protocol.h` 中新增：

```c
#define ROS_CAN_DISTANCE_ID       0x110U
#define ROS_CAN_SENSOR_DIAG_ID    0x111U
```

CAN 参数：标准帧、数据帧、DLC=8、500 kbit/s、little-endian。

### 4.2 `0x110`：距离样本帧

每个传感器的新样本独立上报一帧：

| 字节 | 字段 | 类型 | 说明 |
|---:|---|---|---|
| 0 | `type` | `uint8` | 固定为 `0x01`，表示距离样本 |
| 1 | `sensor_id` | `uint8` | `0` 前、`1` 左、`2` 右 |
| 2..3 | `distance_mm` | `uint16 LE` | 中值滤波后的距离，单位 mm |
| 4..5 | `sigma_mm` | `uint16 LE` | 量测不确定度/质量；驱动不支持时填 `0xFFFF` |
| 6 | `status` | `uint8` | 状态位，见下表 |
| 7 | `sequence` | `uint8` | 每个传感器独立递增，溢出回绕 |

`status` 位定义：

| 位 | 名称 | 含义 |
|---:|---|---|
| bit0 | `VALID` | 距离与质量检查通过 |
| bit1 | `OUT_OF_RANGE` | 超出配置量程 |
| bit2 | `TIMEOUT` | 指定时间内无新测量 |
| bit3 | `I2C_ERROR` | I2C/HAL 通信错误 |
| bit4 | `LOW_QUALITY` | sigma 或驱动状态不满足阈值 |
| bit5 | `EMERGENCY` | 此样本触发本地紧急停车 |
| bit6..7 | 保留 | 必须为 0 |

无效数据约定：

```text
distance_mm = 0xFFFF
sigma_mm    = 0xFFFF
VALID        = 0
```

示例：前方传感器得到 `850 mm`、`sigma=10 mm`、数据有效、第 `44` 帧：

```text
CAN ID: 0x110
DLC:    8
Data:   01 00 52 03 0A 00 01 2C
```

### 4.3 `0x111`：传感器诊断帧

建议仅在初始化、状态变化或低频心跳时发送，避免占用总线：

| 字节 | 含义 |
|---:|---|
| 0 | `0x01`：传感器诊断类型 |
| 1 | `sensor_id`，或 `0xFF` 表示总线级诊断 |
| 2 | 当前错误码 / 最近 HAL 状态 |
| 3 | 连续错误次数（饱和到 255） |
| 4..5 | 最近一次有效距离，`uint16 LE` |
| 6 | 最近样本的 `status` |
| 7 | `sequence` |

---

## 5. 本地安全策略

### 5.1 原则

ROS 的路径规划和局部避障有通信、调度和计算延迟。STM32 不能只把危险距离上报后等待 ROS 回复；本地必须保留独立停车能力。

当前工程已有：

- 上位机心跳超时后调用 `motor_stop_all()`；
- 电机故障事件后调用 `motor_stop_all()`。

传感器急停应复用同一安全路径，并新增一条 CAN 事件/状态供 ROS 记录。

### 5.2 阈值设计

安全距离必须根据实测最大速度、最大减速度、采集周期和执行延迟确定：

```text
d_safe = v² / (2 × a) + v × t_reaction + d_margin
```

其中：

- `v`：当前底盘线速度；
- `a`：实测可实现的最低减速度；
- `t_reaction`：测距周期 + FreeRTOS 调度 + CAN/电机停止执行延迟；
- `d_margin`：轮胎打滑、传感器误差、载荷变化余量。

实施初期可配置三档，而不是将数值写死：

| 等级 | 示例含义 | STM32 行为 | ROS 行为 |
|---|---|---|---|
| 警告 | 距离小于 `warn_mm` | 正常上报，置预警状态 | 降速、更新局部路径 |
| 制动 | 距离小于 `brake_mm` | 发送制动事件；按底盘能力降速 | 停止前向命令、重新规划 |
| 急停 | 距离小于 `estop_mm` | 立即 `motor_stop_all()` | 记录事件，等待安全恢复流程 |

前向传感器只在机器人向前运动时参与前向停车判定；左/右传感器应结合实际运动方向和底盘外廓使用。没有后向传感器时，STM32 无法检测倒车障碍物，倒车速度必须有单独、保守的上限。

### 5.3 传感器故障策略

- 单路连续错误超过阈值：发送 `0x111` 诊断帧，并在 ROS 侧显示故障。
- 前向传感器失效且机器人执行前进：建议本地限速或停车，具体由风险等级决定。
- 左右传感器失效：禁止依赖该方向进行贴边通行；ROS 需降低对应方向的避障置信度。
- 不要把 `0xFFFF`、超时或低质量值误当成“无障碍物”。

---

## 6. ROS 侧集成要求

ROS 端应按 `0x110` 解码并发布：

```text
/front/range  -> sensor_msgs/Range, frame_id=front_range_link
/left/range   -> sensor_msgs/Range, frame_id=left_range_link
/right/range  -> sensor_msgs/Range, frame_id=right_range_link
/diagnostics  -> diagnostic_msgs/DiagnosticArray
```

单位转换：

```text
CAN distance_mm / 1000.0 = ROS Range.range（m）
CAN sigma_mm    / 1000.0 = 诊断中记录的误差（m）
```

还必须提供：

1. `base_link -> front_range_link`、`left_range_link`、`right_range_link` 的静态 TF，安装坐标不得猜测，应写成 launch 参数；
2. sequence 检测和超时检测，旧数据不能长期用于障碍物判断；
3. 无效测距发布为诊断告警，不能伪造为远距离空旷数据；
4. Nav2 使用时明确配置 Range 传感器层，或将有效数据转换为合适的代价地图输入；默认 `obstacle_layer` 主要消费 `LaserScan`/`PointCloud2`，不应假设它会自动使用 `Range`；
5. 完整导航仍应接入 `/scan`（2D 雷达）、`/odom`、`/imu/data` 和 TF。

ROS AI 的详细任务提示词见 [`ROS端VL53L1X兼容实现提示词.md`](ROS端VL53L1X兼容实现提示词.md)。

---

## 7. 实施与验收顺序

1. **硬件检查**：确认 3.3 V 电平、共地、I2C 上拉、XSHUT 独立控制和安装无遮挡。
2. **CubeMX 配置**：启用 I2C1、3 路 XSHUT GPIO，重新生成工程并确认现有 CAN/PWM/编码器引脚未被改动。
3. **单传感器验证**：默认地址 `0x29` 下读距离、测试最小/最大量程和强光场景。
4. **三传感器地址分配**：验证 `0x30`、`0x31`、`0x32` 可同时稳定读取。
5. **固件任务验证**：持续运行至少 30 分钟，检查 I2C 超时、错误计数和任务栈余量。
6. **CAN 验证**：用 CAN 分析仪确认 `0x110` 的字节序、帧率、sequence 连续性和总线无异常。
7. **ROS 验证**：确认三个 `Range` 话题、TF、诊断和 Nav2 代价地图输入正确。
8. **安全验证**：以不同速度、不同障碍物材质测试停止距离；断开传感器、断开 ROS 主机、塞满 CAN 总线，确认机器人都进入预期安全状态。

> 在未完成第 8 步前，不应将本方案作为人员附近的唯一安全保护。VL53L1X、CAN、ROS 和软件急停都不是经过功能安全认证的安全回路。
