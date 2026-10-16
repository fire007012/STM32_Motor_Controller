# CAN1 最小程序验证

用于直接验证 STM32 固件路径，避免仅根据主机错误帧推断故障部件。

## 构建和运行

```powershell
cmake -S . -B build/can-minimal-probe -G Ninja '-DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake' '-DCMAKE_BUILD_TYPE=Debug' '-DSTM32_DIAGNOSTIC_HSI_CLOCK=ON' '-DSTM32_DIAGNOSTIC_CAN_PROBE=ON'
cmake --build build/can-minimal-probe
```

固件：`build/can-minimal-probe/STM32_Motor_Controller.hex`。
这是独立诊断模式，板上不运行正常电机控制、舵机 PWM、VL53、CAN2 或 RTOS 任务。
关闭 `STM32_DIAGNOSTIC_CAN_PROBE` 可构建正常应用。

启动后先运行 CAN1 静默内部回环，对 32 帧逐一检查发送成功、接收 ID、DLC 和数据。
包含现场的 `0x101#0100000000000000`，其余覆盖不同 ID、0～8 字节数据长度及不同位模式。
内部回环不验证外部引脚、收发器、USB-CAN 或绝对位率精度。

然后切换到普通 CAN1 模式，按标称 HSI 时钟计算为 500 kbit/s、采样点 83.3%。
关闭自动重发，每约 1 秒只尝试发送一次标准帧：

```text
0x120#CA4E ss ss 55 AA 00 FF
```

其中两个 `ss` 字节为低字节在前的递增序号。程序不会发送 `0x101`、`0x110` 或
`0x111`。关闭重发是为了有界地观察每次发送结果，不是对原通信故障的修复。

主机抓取所有业务帧和错误帧，并保留接口详情：

```bash
timeout 8s candump -t d -e 'can0,0:0,#FFFFFFFF'
ip -details -statistics link show can0
```

若仍持续收到 `0x101`，需核实是否来自另一节点、另一块板或抓包来源。
若 `0x120` 能稳定接收且 STM32 发送成功计数增加，再比较完整应用的额外初始化和负载。
若最小程序仍发送失败，RTOS、测距和电机业务代码不是该失败的必要条件，仍需验证
基础时钟／位时序、GPIO 和物理通信路径，不能仅由内部回环成功宣称全部固件无误。

## 2026-10-16 上板结果

已通过 ST-Link 烧录并校验成功。静默内部回环 **32/32 通过**。
随后普通模式首次读取为 8 次尝试、8 次完成、0 次成功、8 次发送错误；再次读取为
40 次尝试、40 次完成、0 次成功、40 次发送错误。最近错误为 ACK 错误。
这说明在没有运行正常应用任务的情况下，仍可复现 CAN1 发送失败。

诊断结构体为 `can_probe_result`。本次构建地址为 `0x200004F0`，大小 64 字节；
重新编译后应使用 ELF 符号查找地址，不能假定地址固定。

### 外部时钟对照及双向验证

使用 `STM32_DIAGNOSTIC_HSE_DIRECT=ON` 且 `STM32_DIAGNOSTIC_CAN_PROBE=ON`
构建了临时时钟测量程序：HSE 直接作为 SYSCLK，不启用 PLL、CAN 或 PWM。
源码仍声明 HSE_VALUE=8 MHz，比较 HAL tick 与电脑计时得到：电脑约 10.01047 s 内
HAL 推进 10011 ms，估计 HSE 约为 **8.0004 MHz**。此方法受 SWD、主机计时及调度
误差影响，足以区分 8/25 MHz，不能替代频率计校准。
此前 HSI 模式的 20 s 粗测约慢 0.65%，因此诊断后续恢复外部晶振对照。

已构建并烧录 `build/can-minimal-probe-hse/STM32_Motor_Controller.hex`：
`STM32_DIAGNOSTIC_CAN_PROBE=ON`，`STM32_DIAGNOSTIC_HSI_CLOCK=OFF`，
`STM32_DIAGNOSTIC_HSE_DIRECT=OFF`。实际 RCC_PLLCFGR 为 `0x03402408`，
CAN1_BTR 为 `0x00180005`；内部回环仍为 32/32 通过，首次普通模式快照为
10 次发送、0 次成功、10 次发送错误，最近错误仍为 ACK 错误。

当前板上保留这份外部晶振最小程序，并追加主机接收计数。`can_probe_result` 现为
80 字节，新增 `rx_count`、`last_rx_id`、`last_rx_dlc`、`last_rx_data_low`。
主机可以发送 `cansend can0 321#1122334455667788`，再通过 SWD 检查是否收到 ID
`0x321`、DLC=8、低四字节为 `0x44332211`。诊断程序仅记录，不执行运动指令。

用户此前在 HSI 最小程序运行时抓包为空，接口为 ERROR-ACTIVE，RX/TX 均为 0，
错误累计也为 0。这不同于完整应用时的错误风暴，仍需使用外部晶振版本及主机主动
发送的结果继续定位；不能把“没有错误帧”当作通信恢复。

### 主机主动发送后的快照

用户将 can0 设为 500 kbit/s、关闭 listen-only，执行
`cansend can0 321#1122334455667788` 后立即查询状态：ERROR-ACTIVE，RX/TX
均为 0，错误统计均为 0。由于查询紧接发送，不能据此断言 USB-CAN 未发送或已损坏；
需等待数秒并保留同期抓包及接口统计，`cansend` 返回本身不等于收到总线 ACK。

随后未复位读取 STM32：`attempts=278`、`completed=278`、`tx_ok=0`、
`tx_error=278`、`rx_count=0`，未记录到 `0x321`。最近 CAN1_ESR 为
`0x00800033`（TEC=128、REC=0、ACK 错误、被动错误状态）。

此次 OpenOCD 的 dapdirect 连接失败，切换到 `interface/stlink-hla.cfg` 后成功读取，
没有复位芯片；因此连接失败本身也不能直接归因于 MCU 程序或接线。

用户随后使用正确的一行命令启动抓包、发送 `0x321` 并等待 8 秒。抓包仍为空，
主机 can0 为 ERROR-ACTIVE，TX/RX 和错误累计均为 0。之后再次读取 STM32：
`attempts=433`、`completed=433`、`tx_ok=0`、`tx_error=433`、`rx_count=0`，
CAN1_ESR 仍为 `0x00800033`。已确认本次主机测试后 STM32 未记录到该帧。

主机 TX 统计可能依赖驱动／固件的发送完成或 echo 返回，不能把 TX=0 单独解释为
USB 请求根本没有发出。结合双向均无成功接收，下一步需要主机当前内核日志、
USB-CAN 驱动及设备信息，以定位适配器是否正常启动、USB 控制／数据传输是否异常；
此前出现过的 gs_usb 设置位时序超时只是线索，不能代替当前证据。

### 主机驱动日志与 USB-CAN 自检

用户提供的当前主机信息为 Linux `5.15.0-139-generic`、驱动 `gs_usb`、USB 路径
`2-2.1:1.0`。内核识别 Product 为 `candleLight USB to CAN adapter`、Manufacturer
为 `bytewerk`，VID:PID 为 `1d50:606f`。`lsusb` 的数据库名称显示为 VMware Virtual
USB Mouse，不能据此认定设备实际是鼠标，应结合内核产品描述及绑定驱动判断。
主机存在 VMware 虚拟 USB Hub，适配器通过虚拟 UHCI 控制器接入。

日志在 2026-10-15 22:57:45～22:57:57 有四条
`Couldn't set bittimings (err=-110)`，为 USB 控制请求超时。最新一次设备连接为
2026-10-16 01:00:17，01:00:30 接口就绪，此后提供的片段没有新的同类超时。
因此历史超时不能直接证明当前 USB 数据传输失败。断开后的 `err=-19` 与设备已拔出
相符；虚拟机调度／perf 提示本身也不能证明 CAN 帧 CRC 错误的来源。

可在 ROS 主机运行 `bash tools/can_usb_loopback_check.sh can0`，使用 USB-CAN 自身的
控制器回环功能验证 Linux↔USB-CAN 的收发。脚本记录前后计数及带 RX/TX 标记的抓包，
退出时恢复正常 500 kbit/s 模式。应观察回环接收及计数变化，不能只把本地 TX echo 当成
接收成功。若适配器不支持该模式，配置失败仅表示该测试不可用，不能判作硬件故障。

### USB-CAN 回环现场结果

用户执行控制器回环测试，配置命令未报错，接口明确显示 `can <LOOPBACK>`。
等待 6 秒后抓包为空，RX packets=0、TX packets=0、TX dropped=2，其余错误累计为 0。
这里的 dropped 可能包含接口重置时清理的旧排队请求，不能等同于两个总线发送错误。

这是独立于 STM32 控制程序的测试：USB-CAN 自身的回环数据没有返回 Linux。
需要先解决或解释此问题，再评估完整 STM32 应用的外部 CAN 通信；它不能证明全部
STM32 代码均正确，也不能仅凭此区分适配器固件、驱动和 VMware 透传哪个环节失效。

建议先让 USB-CAN 完全断电再接入虚拟机，重载 `gs_usb`，在普通 500 kbit/s 模式
重新抓取板上最小程序的 `0x120`。若依旧失败，使用原生 Linux 或另一台主机进行同一
适配器回环对照：仅虚拟机失败时定位 VMware/客户机路径；多主机均失败时，进一步
核实 USB-CAN 板卡及固件匹配。没有型号依据前不应给适配器刷入任意固件。

### 适配器重置后接收恢复，但尚未确认 ACK

用户反馈执行 USB-CAN 断电／重接及 `gs_usb` 重载后的正常模式抓包，连续收到 8 帧
`0x120#CA4ED00355AA00FF` 至 `...D703...`，间隔约 1.001 s，can0 RX=8、64 字节，
错误累计为 0。适配器接收路径在这次操作后恢复；不能从多项操作共同完成后的结果
确定是断电、USB 重接还是驱动重载单独生效。

但随后 STM32 SWD 快照为 attempts=1009、completed=1009、tx_ok=0、tx_error=1009、
rx_count=0，CAN1_ESR=`0x00800033`。因此主机接收到数据不等于 STM32 已收到 ACK，
不能宣称双向通信完全恢复。保留最小程序，再测试主机发送 `0x321`，核对两端统计。

### 双向 ACK 错误及 GPIO 采样

用户随后在收到 `0x120` 的情况下发送 `0x321`。抓包出现该帧的 TX 记录，紧接着多条
`no-acknowledgement-on-tx`，主机进入 ERROR-PASSIVE；每约 1 s 还交替出现隐性位
错误与 ACK 错误。TX packets=1 与后续 ACK 错误同时存在，说明该驱动的 TX 记录／
统计不能单独作为总线发送成功的证明。STM32 随后快照仍为 rx_count=0、tx_ok=0，
其最近错误也为 ACK 错误。

诊断程序追加 TIM1→DMA2 Stream5/Channel6 采样：定时器 72 MHz、ARR=71，
以 1 MHz 读取 GPIOA_IDR，记录 1024 个 32 位字，覆盖约 1.024 ms。
采样只读 PA12（TX）和 PA11（RX）数字电平，不改变它们的 CAN AF9 引脚映射。
默认捕获第一帧后停止覆盖，`can_probe_trace_request` 通过 SWD 写入 1 可再次触发。
`can_probe_trace_meta` 依次记录 ready、已采样点数、DMA HISR 和帧序号。

上板采样为 ready=1、1024 点、HISR=0xC00（半传输／传输完成，未见 DMA 错误标志）。
序号 0 的 1024 个采样点中 TX 与 RX 电平全部相同。根据已知测试帧及标称位率推算，
首次采样 ACK 槽约在 216～218 μs，两点 RX 均为高；后续重新采样的 SOF 提前 1 μs，
ACK 槽约在 215～217 μs，两点 RX 也均为高。这与 CAN ESR 的 ACK 错误一致。
该数字采样不能测模拟电压、亚微秒脉冲或代替完整 CAN 解码，不据此断言具体故障部件。

分析命令：

```powershell
python tools/analyze_can_gpio_trace.py build/can-minimal-probe-hse/gpio-trace.bin --sequence 0 --pin-map pa11-pa12
```

输出同名 JSON、CSV、PNG。序号需从该次 `can_probe_trace_meta[3]` 获取，不可一律填 0。
分析工具假设采样从空闲开始，并根据标准帧 ID 0x120、已知 payload、CRC15 和位填充
推算 ACK 位置；它不是通用 CAN 解码器。

还添加了 PA11 的 EXTI 下降沿监测，仅查询 pending，禁用 NVIC 的 EXTI15_10 中断。每次发送结束
并额外等待 1 ms 后清除 pending，在约 1 s 的发送空闲期内观察是否有 RX 下降沿。
`can_probe_idle_rx_windows[0]` 为已观察窗口数，[1] 为出现下降沿的窗口数（非边沿总数）。
旧版曾读取到 12、194、332 个窗口而均为零边沿，但该版本屏蔽了 EXTI_IMR。
随后在发送前后设置硬件断点验证：即使 MCU 自发帧，PR11 仍为零；打开 EMR11 也未改善。
打开 IMR11、保持 NVIC 对应 IRQ 禁用后，同样的自发帧使 PR11 置位。因此旧版零边沿
结果无效，不能据此判断 ROS 信号未到达 PA11。主机发包后 CAN 接收计数仍为零、
ESR 为 `0x00800033` 的结果不依赖该监测器，仍有效。

已修正诊断程序并重新烧录校验。新版额外记录 `can_probe_tx_rx_windows`，其中 [0] 为自发
尝试数，[1] 为自发期间检测到 RX 下降沿的次数，用于验证边沿监测器有效。
应先确认自发边沿计数增长，再由主机重新发包并读回空闲窗口计数和 CAN 接收计数。
新版首次读回：自发 22 次、22 次均检测到 RX 边沿；21 个空闲窗口均未检测到边沿；
CAN 接收计数为零，发送成功为零，发送错误 22 次，ESR 仍为 `0x00800033`。
这是监测器修正后的基线，尚需与主机重新发送的时间对应。

用户随后在 ROS 执行五次 `cansend can0 321#1122334455667788`（每次间隔一秒），
反馈已发送且未报告命令错误。未复位读回如下：

| 项目 | 数值 |
| --- | --- |
| 自发尝试／完成 | 87 / 87 |
| 自发期间检测到 RX 下降沿 | 87 |
| 空闲观察窗口／有外部下降沿的窗口 | 86 / 0 |
| CAN 接收计数 | 0 |
| CAN 发送成功／错误 | 0 / 87 |
| CAN1_ESR | `0x00800033`（TEC=128、REC=0、ACK 错误） |
| EXTI_IMR / FTSR | 均为 `0x00000800` |
| NVIC_ISER1 | `0x00400000`，EXTI15_10 的 bit 8 未启用 |

这次边沿监测已通过自发帧验证；在覆盖本次主机发送的空闲窗口中未观察到外部 PA11
下降沿。不能用该结果直接指定哪个部件损坏；应对照 USB-CAN 外部发送路径、总线和
STM32 接收路径。优先更换已知正常的 USB-CAN 作对照，或测量收发器 RXD／PA11 的
实际波形。`cansend` 无报错只代表请求提交，不证明总线上有有效发送。
板上仍为最小诊断程序，未恢复 VL53／电机应用。

### 驱动重载后 USB-CAN 回环复测

用户无备用 USB-CAN、示波器或逻辑分析仪。为验证重载后已改变的适配器状态，
再次在 `<LOOPBACK>` 模式测试。此次收到 `TX 321`、一条控制器 ERROR-PASSIVE
错误帧，以及独立的 `RX 321`，数据均为 `1122334455667788`。
这次自身回环数据路径有效，先前重载前回环失败不能再代表当前状态。

输出在恢复普通模式前读取，显示 `<LOOPBACK>`、ERROR-PASSIVE；累计统计
RX packets=19581、TX packets=8、TX dropped=4、bus-off=13838。
这些统计未清零，缺少本次测试前快照，不据此计算本次新增错误或断言 6 秒内发生了
13838 次 bus-off。命令 EXIT trap 随后执行恢复普通模式的操作。

当前证据为：适配器控制器回环可收发、可解码 STM32 帧，但普通模式发送尚未在
STM32 PA11 的空闲窗口中观测到。回环不验证适配器外部收发器的驱动能力，
仍需确认 USB-CAN 板卡型号、外部 CAN 收发器型号及其使能／静默控制。
不能在型号不明时刷写任意 candleLight 固件，也不能由此单独指定损坏元件。

### 按用户要求恢复正常应用固件

2026-10-16，用户要求先烧回正常版本。已重新配置并构建
`build/vl53-recovery/STM32_Motor_Controller.hex`，三个 `STM32_DIAGNOSTIC_*`
选项均为 OFF；ELF 包含正常应用任务和 `distance_sensor_init`，不含 `can_probe` 符号。
通过 ST-Link 完成烧录，OpenOCD 报告 `Verified OK`，并已复位启动。
板上现为包含 VL53 恢复及完整状态上报修复的正常应用固件；此前最小诊断程序已被替换。
本次恢复烧录不代表外部 CAN 通信问题已解决，尚未收到恢复后的主机抓包结果。

新增静态缓冲区改变了 SRAM 布局。最新构建请重新使用 ELF 查找全部诊断符号地址。

### CAN1 引脚迁移（2026-10-16）

用户报告 PA11/PA12 短接击穿后，当前源码已改为 PD0=CAN1_RX、PD1=CAN1_TX，
复用仍为 AF9。上文 PA11/PA12 的现场记录属于迁移前历史，不代表新引脚测试结果。
最小程序同步改为 DMA 读取 GPIOD_IDR，RX 下降沿监测使用 PD0 对应的 EXTI0，
仅查询 pending，保持 EXTI0 的 NVIC 中断禁用。内部回环通过也不能证明外部链路正常。

分析工具默认读取新映射（RX bit 0、TX bit 1）；分析旧 GPIOA 采样时必须添加
`--pin-map pa11-pa12`，如上方历史采样命令所示。新采样不需要此参数。
