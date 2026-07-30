# ESP32-S3 地面站使用说明

本说明适用于 `Waveshare ESP32-S3-Touch-LCD-7` 地面站程序。开发板包含 800x480 ST7262 RGB 屏、GT911 电容触摸和 CH422G IO 扩展器。

## 1. 功能范围

地面站负责：

- 选择并确认任务 1 或任务 2。
- 通过 UDP 向 ROS 发送任务选择请求。
- 显示车辆、ROS、无人机和视觉状态。
- 显示车辆速度、位移、循迹误差、转弯类型和质量标志。
- 显示链路数据龄、故障码和实际屏幕 FPS。

地面站不直接控制电机、飞控、电磁铁或载荷释放。任务按钮始终可用，但任务是否被接受由 ROS 回执决定。

## 2. Arduino 环境

### 2.1 推荐版本

| 项目 | 版本 |
| --- | --- |
| Arduino IDE | 2.x |
| ESP32 core | 推荐 `3.3.8` 或 `3.3.8-cn` |
| ESP32_Display_Panel | `1.0.4` |
| ESP32_IO_Expander | `1.1.1` |
| esp-lib-utils | `0.2.3` |
| LVGL | `8.4.0`，不要使用 9.x |

### 2.2 工具菜单

| 菜单 | 设置 |
| --- | --- |
| Board | `Waveshare ESP32-S3-Touch-LCD-7` |
| PSRAM | `Enabled` |
| Flash Mode | `QIO 80MHz` |
| Flash Size | `8MB (64Mb)` |
| Partition Scheme | `Huge APP (3MB No OTA/1MB SPIFFS)` |
| CPU Frequency | `240MHz` |
| USB CDC On Boot | `Disabled` |
| Upload Mode | `UART0 / Hardware CDC` |

未启用 PSRAM 时程序会在编译期报错，防止生成只能黑屏的固件。

## 3. 网络配置

首次实机使用前，将 `config_local.example.h` 复制为 `config_local.h`，填写：

- 离线热点 SSID 和 WPA2 密码。
- 与 ROS、小车一致的 32 字节 HMAC 密钥。
- ROS 和小车的实际 IP 地址。
- 本机、ROS 和小车的 UDP 端口。

示例端口为：

| 节点 | UDP 端口 |
| --- | ---: |
| ROS | `42000` |
| 小车 | `42001` |
| 地面站 | `42002` |

真实密码和密钥只能写入被忽略的 `config_local.h`，不要提交到仓库。

## 4. 编译与烧录

1. Arduino IDE 打开 `ground_station_esp32s3.ino`。
2. 按第 2.2 节选择开发板和工具参数。
3. 使用支持数据传输的 USB 线连接开发板。
4. 在“工具 > 端口”中选择开发板串口。
5. 点击“验证”，确认编译成功。
6. 点击“上传”，等待烧录完成。
7. 打开串口监视器，波特率选择 `115200`。
8. 按一次复位键，观察启动自检。

正常启动日志应包含：

```text
[自检] ESP32-S3 地面站启动，任务控件保持可用
[显示自检] PSRAM 总容量=8388608 字节，可用=...
[显示自检] RGB 回弹缓冲=16000 字节
[显示自检] 800x480 屏幕和 GT911 触摸已就绪
```

## 5. 界面说明

| 区域 | 内容 |
| --- | --- |
| 顶部 | CAR 和 ROS 数据龄 |
| 左侧 | TASK 1、TASK 2、ROS 已确认任务和待确认任务 |
| 中上 | 地面站本地状态和 ROS 任务阶段 |
| 中部 | 无人机链路、视觉和 ROS READY 状态 |
| 右侧 | 小车状态、转弯、位移、速度、循迹误差和质量标志 |
| 左下角 | 面板实际刷新帧率 `FPS n` |
| 底部 | 系统正常提示或 `FAULT 0xXXXX` |

FPS 按 RGB 面板 VSYNC 计数，每秒更新一次。正常情况下通常接近 60，刚启动时短暂显示 `FPS --` 属于正常现象。

## 6. 任务操作

1. 点击 `TASK 1` 或 `TASK 2`。
2. 弹出确认框后检查任务编号。
3. 点击 `CONFIRM` 提交，或点击 `CANCEL` 取消。
4. 提交后左侧显示 `PENDING TASK n`，程序每 250 ms 向 ROS 重发同一选择 ID。尚未收到车辆会话时请求中的车辆 ID 为 0，由 ROS 明确接受或拒绝。
5. ROS 返回匹配的确认后，`ROS CONFIRMED` 更新为对应任务并停止重发。

车辆运行、任务完成、链路陈旧或显示故障码时，任务按钮仍保持可用。ROS 可以根据当前阶段接受或拒绝新的选择请求。

## 7. 状态含义

| 显示 | 含义 |
| --- | --- |
| `BOOT WAITING` | 等待当前车辆会话或 ROS 状态 |
| `PRESTART` | 任务尚未启动 |
| `SELECT PENDING` | 已发送选择，等待 ROS 回执 |
| `SELECTED` | ROS 已确认选择 |
| `ARMED READY` | ROS 报告已准备 |
| `CAR RUNNING` | 小车正在运行 |
| `COMPLETE` | 任务完成 |
| `FAULT` | 收到故障或状态异常 |

## 8. 故障码

底部故障值可能由多个标志按位组合，例如 `0x0040` 表示数据陈旧。

| 位 | 数值 | 含义 |
| --- | ---: | --- |
| 0 | `0x0001` | Wi-Fi 超时 |
| 1 | `0x0002` | 小车丢线 |
| 2 | `0x0004` | 编码器不一致 |
| 3 | `0x0008` | PID 周期超时 |
| 4 | `0x0010` | 启动按钮卡住 |
| 5 | `0x0020` | 电机故障 |
| 6 | `0x0040` | 数据陈旧 |
| 7 | `0x0080` | 协议错误 |
| 8 | `0x0100` | 没有已确认任务 |
| 9 | `0x0200` | 欠压复位 |

## 9. 常见问题

### 9.1 屏幕完全不亮

- 确认开发板型号不是通用 `ESP32S3 Dev Module`。
- 确认 PSRAM 为 `Enabled`、Flash 为 8MB。
- 检查串口是否打印 PSRAM 和面板初始化错误。
- 烧录微雪官方 `08_DrawColorBar` 示例区分软件与硬件问题。

### 9.2 内容刷新后向下漂移

程序已按 Espressif 建议将 RGB bounce buffer 从 `width * 10` 增大为 `width * 20`，即 16000 字节，并保证 LVGL 与 Arduino 主循环运行在同一个 CPU 核心。

若仍漂移：

1. 确认串口显示 `RGB 回弹缓冲=16000 字节`。
2. 使用推荐 ESP32 core，不要混用 alpha 或不兼容版本。
3. 保持 QIO 80MHz、OPI PSRAM 和 240MHz CPU 设置。
4. 检查 5V 供电是否稳定，避免与电机、电磁铁等感性负载共用不稳定电源。
5. 烧录官方色条示例；官方示例也漂移时优先检查核心版本、供电和屏幕硬件。

### 9.3 触摸位置不正确

- 确认板型为微雪 7 寸专用板型。
- 不要手工覆盖 GT911 引脚。
- 记录实际触摸方向后再调整镜像或交换 XY 配置。

### 9.4 一直显示 CAR 或 ROS STALE

- 检查三端是否连接同一个离线热点。
- 检查 IP、UDP 端口和 HMAC 密钥是否完全一致。
- 检查热点是否启用了客户端隔离。
- 确认 ROS 使用 18 字节 `MISSION_STATUS` 负载。

### 9.5 编译提示缺少库或 API

- 检查库版本是否与第 2.1 节一致。
- 删除重复安装的旧版 LVGL 或显示库。
- 确认使用 LVGL 8.4.0，而不是 LVGL 9.x。

## 10. 调试模拟器

开发阶段可使用主机端 UDP 模拟器验证协议处理和状态机逻辑，无需 ESP32 硬件：

```bash
bash readonly/tests/build_ground_station_sim.sh
bash readonly/tests/run_ground_station_sim.sh
```

详见 `SIMULATOR.md`。

## 11. 资料

- 微雪 Arduino 开发说明：<https://docs.waveshare.net/ESP32-S3-Touch-LCD-7/Development-Environment-Setup-Arduino/>
- 微雪 RGB 屏示例：<https://docs.waveshare.net/docs/ESP32/ESP32-S3/ESP32-S3-Touch-LCD-7/Arduino/Arduino-RGB-LCD-Demo/>
- 项目构建说明：`../BUILD.md`
- 硬件检查表：`../HARDWARE_CHECKLIST.md`
- UDP 协议说明：`../shared_protocol/PROTOCOL_V1.md`
- 调试模拟器：`SIMULATOR.md`
