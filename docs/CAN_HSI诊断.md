# CAN 内部时钟诊断

适用于 ROS 主机收到重复的 `0x101#0100000000000000`，并同时报告
`ERROR-PASSIVE`、CRC、格式、位填充错误的情况。仅凭重复帧不能认定 VL53
初始化失败；这些协议错误需要先排查 CAN 时钟和物理链路。

## 诊断固件

- 文件：`build/can-hsi-diagnostic/STM32_Motor_Controller.hex`。
- 工程目标仍为 STM32F407；使用内部 16 MHz HSI，PLL 为 16 / 16 × 144 / 2。
- SYSCLK 为 72 MHz，APB1/APB2 为 36 MHz，CAN1/CAN2 仍为 500 kbit/s。
- 保留测距恢复、遥测和控制逻辑。默认构建仍使用外部 8 MHz HSE。
- 用途是比较外部晶振配置与内部时钟的表现。HSI 的精度受温度、电压影响，
  不能用本次诊断代替最终确认板上晶振及长期 CAN 时序验证。

PowerShell 构建命令（需要 ARM GCC、CMake、Ninja 在 PATH 中）：

```powershell
cmake -S . -B build/can-hsi-diagnostic -G Ninja '-DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake' '-DCMAKE_BUILD_TYPE=Debug' '-DSTM32_DIAGNOSTIC_HSI_CLOCK=ON'
cmake --build build/can-hsi-diagnostic
```

## 现场验证

已确认收发器型号为 TJA1050，VCC 接 5 V 符合该芯片供电要求。CAN1 接线为：

| STM32 / 电源 | TJA1050 模块 |
|---|---|
| PA12（CAN1_TX） | TXD / CTX |
| PA11（CAN1_RX） | RXD / CRX |
| GND | GND |
| 5 V | VCC |

CANH 接适配器 CANH，CANL 接 CANL。非隔离连接应有共同信号地。若模块引出 S
静默控制脚，应确认处于正常模式（TJA1050 的 S 为低电平）。总线两端各一只
120 Ω 终端，全部断电后测 CANH 与 CANL 之间应约为 60 Ω。

烧录上述诊断 HEX 后，在 ROS 主机重置接口并抓取原始帧：

```bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 500000 sample-point 0.833 listen-only off restart-ms 100
sudo ip link set can0 up
timeout 5s candump -t d -e 'can0,0:0,#FFFFFFFF'
ip -details -statistics link show can0
```

采样点设为 0.833 是便于对照；原先主机 0.875 与 STM32 0.833 不同本身不能证明故障。
`restart-ms` 只用于 BusOff 后恢复，不能修复电气或时钟错误。

若错误消失且出现多种 `0x101` 状态及 `0x110/0x111`，重点核实外部晶振频率、
HSE 配置和时钟质量。若仍有相同错误，继续检查接线、终端、收发器及 USB-CAN
适配器；这也不能完全排除时钟问题。距离数据的 `0xFFFF` 表示无效测距，不能当作有效值。

## 2026-10-16 现场反馈

内部时钟诊断 HEX 已通过 ST-Link 写入，OpenOCD 返回 `Verified OK` 并复位芯片。
随后主机抓包仍出现 CRC、格式、位填充错误，以及相同的
`0x101#0100000000000000`。因此改用 HSI 未改善现场故障，外部晶振频率假设的
优先级降低。用户重新连接 ST-Link 后，已在不暂停 CPU 的情况下读取到以下运行寄存器。
用户另反馈断电测得总线电阻为 60 Ω，信号地均已连接。

| 寄存器／地址 | 读数 | 解读 |
|---|---|---|
| RCC_CR / `0x40023800` | `0x03007983` | HSI 和 PLL 已就绪，HSE 未开启 |
| RCC_PLLCFGR / `0x40023804` | `0x03002410` | PLL 来源 HSI，M=16、N=144、P=2、Q=3 |
| RCC_CFGR / `0x40023808` | `0x0000900A` | 系统时钟使用 PLL，AHB 不分频，APB1/APB2 均二分频 |
| CAN1_BTR / `0x4000641C` | `0x00180005` | 普通模式，分频 6、BS1=9、BS2=2、SJW=1；按标称 HSI 计算为 500 kbit/s |
| CAN1_TSR / `0x40006408` | 连续三次为 `0x82000008` | 三个发送邮箱均未空闲 |
| CAN1_ESR / `0x40006418` | 首次 `0x00E80033` | TEC=232，REC=0，LEC=3（ACK 错误），处于被动错误状态，未 BusOff |
| CAN1_ESR 后续采样 | `0x00E80033` → `0x00980053` → `0x00300030` | 间隔约 300 ms，TEC=232→152→48；LEC 为 ACK→显性位错误→ACK；错误状态有变化 |
| CAN1 三个发送邮箱 | TIR 均为 `0x20200001`，DLC 均为 8，数据首字节依次为 `01/02/03` | 标准 ID 0x101 的三类状态均已请求发送，不能把现场现象解释为软件只生成了 01 |

HSI 的实际精度仍未经仪器测量。寄存器证明当前时钟来源和分频与诊断固件相符。
错误计数下降的过程中没有连续捕获 BOFF，不能仅靠这些离散快照断定发生过 BusOff 自动恢复。
用户已确认：寄存器采样期间 USB-CAN 始终连接，ROS 的 can0 也在运行。
因此上述 ACK 和位监测错误对应联机现场，已确认 STM32 的发送无法正常完成，
需要继续定位收发器及其 TXD/RXD 路径、总线信号质量和 USB-CAN 的接收／确认路径。
60 Ω 和共地检查通过不能代替波形及部件交叉验证。

`0x2000000C` 和 `0x20000004` 是 SocketCAN 错误帧标识，并非 STM32 发送的业务 ID。
偶尔能解析出 `0x101` 不能证明 STM32 已收到 ACK；重复首帧与硬件持续重发相符，
但需要结合 STM32 的 ESR、TSR、发送邮箱快照判断，不能仅由抓包确定。

下一步以最小链路排查：仅保留 STM32 CAN1、TJA1050、USB-CAN、两端终端和公共信号地，
检查 PA12→TXD、RXD→PA11，再抓取相同时段日志。若仍异常，优先交叉替换 TJA1050
模块或 USB-CAN，一次只替换一个部件。旧日志中的 gs_usb 位时序设置超时是适配器
路径的独立线索，尚不能据此认定适配器损坏。
