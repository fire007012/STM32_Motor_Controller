# CAN / ROS 日志证据包（2026-10-15）

此包保存已存在的日志及本次会话采样摘录，不包含重新进行的实机测试。

- `ros-originals/`：14 份现存 ROS rosout 日志及节点输出，逐字节复制；来源路径、大小、SHA256 见 `manifest.json`。保留完整上下文及原始 Unix 时间戳。节点 stdout/stderr 曾混流的内容保持原样。
- `kernel-original.log`：通过 `journalctl -k` 导出的 2026-10-15 15:50–20:10 内核记录，时间显示为 UTC+08:00。具体命令见 `kernel-export-command.txt`。该范围最后可见内核事件为 19:55:05；不能视为期间始终有连续 CAN 抓包。
- `kernel-context.log`：上份文件中 USB 断连、gs_usb 错误及 can0 就绪行前后若干行的摘录；未改写原始日志行。
- `ros-enobufs-timeline.txt`：22 条 rosout 缓冲区满错误的索引，附来源行号、原始行及换算后的北京时间。此文件是派生索引，原始文件保持原样。
- `ip-link-session-excerpt.txt`：从本次会话工具输出摘取的 ERROR-PASSIVE 状态及接口计数，输出行尾空白已规整。原采样未写入日志文件，也未打印时间戳。
- `candump-session-excerpt.txt`：从本次会话工具输出逐字摘取的 CRC、格式、位填充错误帧。原命令没有输出时间戳；不是 ROS 日志，不是连续完整抓包。

限制：现有记录不能给出 CRC 错误与某次 USB 断连之间精确到秒的先后关系。USB 断连行本身不能区分手动拔插、虚拟机设备切换和异常断连。不能仅据这些材料认定缓冲区满、CAN 协议错误、USB 断连具有已证实的因果关系。
