# CAN ACK 根因与修复（2026-10-16）

## 已确认的软件问题

执行应答 `0x102`、命令入队拒绝应答及舵机应答原先都与周期遥测共用发送队列。
队列满或 CAN1 的 100 ms 退避期间返回 `HAL_BUSY`，调用者未保留报文，造成应用应答丢失。
此外，原发送层遇到无 TXOK 的完成结果后会直接丢弃该帧。

现改用独立的 32 帧应答队列，优先于尚未提交的遥测发送，允许在退避期间入队。
应答只有在硬件 TXOK 后出队；发送错误后保留原序号和内容，按 100 ms 退避重试。
最长保留时间从首次入队起为 500 ms，不因重试刷新。只重发结果，不重放控制命令。
邮箱挂起超时中止硬件请求时保留应答；显式取消 CAN1 则清空应答和普通报文。

队列仍有容量边界：应答自身满时返回 `HAL_BUSY`，并记录 `reply_overflow`；过期记录
`reply_expired`。不阻塞电机任务等待应答，以免妨碍急停。断链或无界请求洪泛下不承诺送达。

## 硬件 ACK 的证据与限制

用户当前仅连接 STM32 CAN1、收发器、USB-CAN，ROS 未启动，STM32 上电仍有错误。
STM32 主动周期上报，不依赖 ROS；正常模式下 USB-CAN 应由硬件确认有效帧，也不依赖 ROS。

本次修复前重新读板得到：

| 项目 | 值 | 含义 |
|---|---|---|
| CAN1 MCR | `0x00010054` | NART=1，已关闭硬件自动重发 |
| CAN1 BTR | `0x00180005` | 普通模式，36 MHz 下为 500 kbit/s、采样点 83.3% |
| CAN1 ESR | `0x00800033` | TEC=128、REC=0、LEC=3，ACK 错误且处于错误被动状态 |
| RCC PLLCFGR / CFGR | `0x03402408` / `0x0000900A` | HSE PLL，配置与默认应用一致 |
| GPIOA MODER / AFRH | `0xAA800000` / `0x00099000` | PA11、PA12 为 AF9 |

用户同期主机输出为 can0 UP/LOWER_UP、500 kbit/s、采样点 87.5%、ERROR-PASSIVE，
restart-ms=0，没有显示 LISTEN-ONLY/LOOPBACK。RX packets=80104、TX packets=0；
error-warn=2228、error-pass=24429、bus-off=46760 是累计统计，不能当作本次新增事件。
采样点不同本身不证明配置错误；TX=0 也不能单独证明 USB 未提交发送。

这些证据确认外部发送未获得有效 ACK，但尚不能在 STM32 收发器、总线、USB-CAN 外部
收发器或适配器运行状态之间指定最终故障部件。应用队列修复不能据此称为硬件 ACK 已恢复。

## 本地诊断

已补齐两路 CAN SCE 中断，在 HAL 清除 ESR.LEC 前记录错误；普通服务刷新运行快照，
发送前另检查 ESR.BOFF。无需依赖故障中的 CAN 链路回传诊断。

`can_bus_diagnostics[0]` 为 CAN1，`[1]` 为 CAN2，每项按顺序包含 18 个 32 位字：

```text
tick mcr btr esr tsr
last_error_tick last_error_esr error_events
tx_ok tx_error arbitration_lost tx_timeouts
reply_retries reply_overflow reply_expired reply_pending
rx_frames last_rx_id
```

ESR 的 TEC=`(ESR >> 16) & 255`、REC=`ESR >> 24`、LEC=`(ESR >> 4) & 7`；
低三位为警告、错误被动、Bus-Off。`error_events` 是 SCE 中断观察次数，不能等同于
物理错误帧总数。`last_error_esr` 在成功收发后仍保留，`esr` 为当前快照。
`rx_frames` 只计 HAL 已取出的帧，不含硬件滤除或 FIFO 丢失的帧。

重新编译后以 ELF 查找地址，不要硬编码旧地址：

```powershell
arm-none-eabi-nm -S build/ack-root-cause/STM32_Motor_Controller.elf
```

本次普通连接不稳定，`connect_assert_srst` 后重新将调试速度设为 100 kHz 才可稳定暂停。
需要读运行状态时避免 `reset`，以免清零待验证的计数；使用已连接的调试会话读回。

## 验证

`python tools/run_host_tests.py` 验证独立队列容量、优先发送、真实拒绝应答路径、
发送错误恢复、挂起超时、过期、时钟回绕、错误寄存器保留及 Bus-Off 抑制发送，
并运行现有控制回归和 30 项 Python 测试。

正常应用构建：

```powershell
cmake -S . -B build/ack-root-cause -G Ninja '-DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake' '-DCMAKE_BUILD_TYPE=Debug' '-DSTM32_DIAGNOSTIC_CAN_PROBE=OFF' '-DSTM32_DIAGNOSTIC_HSI_CLOCK=OFF' '-DSTM32_DIAGNOSTIC_HSE_DIRECT=OFF'
cmake --build build/ack-root-cause --parallel 4
```

固件为 `build/ack-root-cause/STM32_Motor_Controller.hex`。软件回归和编译通过不能证明
外部收发已经恢复；烧录及运行结果须单独记录。

## 本次烧录与运行

已备份完整 512 KiB Flash 至 `build/ack-root-cause/before-fix-flash.bin`，SHA256 为
`24C90F6DC23DAB6E291E7E1C4EC1CE710F3DF9D0EC0CFAA74F7B125853C85621`。

修复版 HEX 的 SHA256 为
`B427FA0113FF3BF67D2F35FCCB80D62D3199C9F79224B6D49B71828EC72BE81E`。
第一次 RAM 批量写入算法超时，随后用 DAP 连接、禁用 RAM 工作区、逐次写入完成。
OpenOCD 报告 `verified 60676 bytes`，再通过 SYSRESETREQ 软件复位启动。
新固件约 8 秒时读取 CAN1：发送成功 0、发送失败 76、收到帧 0、
当前 ESR=`0x00800003`，中断保留的最近错误 ESR=`0x00800033`。
当前 LEC 为 0 是 HAL 已清除该字段，不表示错误根因消失；保留快照仍能定位 ACK 错误。

本次 ELF 中 `can_bus_diagnostics` 为 `0x20006544`、大小 `0x90`，仅对此次构建有效。
`build/ack-root-cause/read_live.py` 通过现有 OpenOCD Tcl 端口 6666 读取运行快照，
并保存到 `last-live-diagnostics.json` 与 `live-diagnostics.jsonl`，不暂停或复位 CPU。

主机重新配置 can0 时，两次返回 `RTNETLINK answers: Connection timed out`，
第三次无报错。随后几次 `can0 up` 与抓包命令因拼接而未执行成功，不能视作接口已启动
或复测通过。配置超时是 USB-CAN 控制链路的当前独立故障证据，不能归因于 `0x102`。
下一步需在接口真正 UP 后，用同期主机抓包和本地 tx_ok/rx_frames 判断硬件 ACK 是否恢复。

## 主机重启接口后的双向验证

主机随后成功将 can0 UP，500 kbit/s、采样点 0.833，仍报告被动错误、位填充、
隐性位及 ACK 错误，错误组之间约为 100 ms。硬件 ACK 未恢复。

临时把 CAN1 HAL 软件状态设为 READY，阻止发送服务提交新帧，同时保持 CAN 硬件
普通模式、接收中断及滤波器；HAL 接收函数也接受 READY。暂停后发送错误计数停止增长。
主机发送 `0x321#1122334455667788`，抓包显示该帧、TX packets 从 0 变为 1，
但 STM32 `rx_frames` 仍为 0，FIFO0 为空。因此本地 echo/发送统计不能证明板端已收到。

进一步设置 PA11 EXTI 下降沿锁存，启用 IMR、FTSR，保持 EXTI15_10 的 NVIC 禁用，
不改变 CAN AF9。先短暂恢复自发验证，PR11=`1`；再次暂停并清除，PR11=`0`。
用户随后再次执行相同 `cansend` 并确认已发送，读回 PR11 仍为 0、CAN 接收计数仍为 0。
此期间无自发帧干扰，检测器也已验证有效。测试结束已恢复原 EXTI 设置及正常发送服务。

现有证据把故障定位到外部发送／接收电平路径：USB-CAN 发送、CANH/CANL 链路、
板端收发器接收至 PA11。它不能单独区分这些部件，不能直接断言 USB-CAN 或 MCU 损坏。
硬件 ACK 是 CAN 外设收到有效帧时自动产生的，缺少独立的“ACK 固件”不是目前证据支持的原因。

## USB-CAN 完全断电对照

用户没有第二块收发器。随后在 STM32 暂停主动发送、PA11 边沿检测已通过自发验证并清零
的状态下，让 USB-CAN 完全断电约 5 秒并重新连接虚拟机，再配置普通 500 kbit/s 模式。
本次配置未超时，主机接口序号由 8 变为 9，显示 ERROR-ACTIVE、错误累计为 0，
提交 `0x321` 后 TX packets=1、RX packets=5。

同期 STM32 读回：PR11=0、rx_frames=0、FIFO0 为空。适配器统计不能替代这个板端结果；
暂停 STM32 发送后主机没有错误，也不能证明外部链路恢复。恢复正常发送后约 1.5 秒，
CAN1 tx_error 从 1257 增至 1274、tx_ok 仍为 0，最近错误仍为 ACK，rx_frames 仍为 0。
原 EXTI 配置及 HAL LISTENING 状态均已恢复，板上保留正常应用修复版。

最终尚未解决的是外部硬件 ACK，具体故障部件未获确证。需要 USB-CAN 板卡及外部收发器
型号／实物接线信息，或额外收发器、波形测量作对照。主控固件镜像已校验、CAN 模式和
引脚配置已读回，因此目前不支持仅凭错误帧认定主控“ACK 固件损坏”。
