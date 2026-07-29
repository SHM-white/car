# ESP32-S3 陆空协同小车与地面站

本目录已实现任务文档要求的三部分代码：

- `embedded/shared_protocol/`：UDP v1 显式序列化、CRC16、HMAC-SHA256、序号防重放。
- `embedded/car_esp32s3/`：感为八路 I2C 灰度循迹 PID、编码器里程计、转弯分类、B/D/A 事件、物理按钮启动和锁存安全停车。
- `embedded/ground_station_esp32s3/`：微雪 7 英寸触屏界面、始终可用的任务 1/2 确认流程、ROS 回执和陈旧数据故障。

地面站安装、烧录、界面操作和故障排查见 [地面站使用说明](embedded/ground_station_esp32s3/GROUND_STATION_USER_GUIDE.md)。

车辆接线、配置、传感器校准、烧录、运行和故障排查见 [ESP32-S3 循迹小车使用说明](embedded/car_esp32s3/CAR_ESP32S3_USER_GUIDE.md)。

原来的 `car/car.ino` 保留为兼容入口，实际车辆工程入口是 `embedded/car_esp32s3/car_esp32s3.ino`。

## 烧录前必须完成

1. 将两个工程内的 `config_local.example.h` 分别复制为 `config_local.h`。
2. 填入离线 WPA2 热点、32 字节随机 HMAC 密钥、DHCP 保留地址、真实引脚和标定数据。
3. 核对电机驱动的制动电平、编码器方向、黑白线极性，架空车轮进行首次测试。
4. 按 [构建说明](embedded/BUILD.md) 安装微雪官方示例包内的显示依赖，选择 `Waveshare ESP32-S3-Touch-LCD-7`，并启用 PSRAM。
5. 安装本地共享库，并使用微雪要求的 ESP32 core `3.0.6` 以上版本编译。

示例配置只能用于编译和台架状态机测试，不能直接上车。真实 `config_local.h` 已被 `.gitignore` 排除。

## 已实现的安全边界

- HMI 只发送任务选择，不包含解锁、飞行、投放或任务启动命令。
- 车辆只由物理按钮从 `READY` 进入 `RUNNING`，安全停车后只能物理复位。
- 失联超过 1 秒、失线、编码器分歧、周期超时、欠压复位、电机初始化失败或按钮卡住都会锁存 `SAFE_STOP`。
- HMI 的任务按钮始终可用；地面站负责发送选择请求，ROS 根据当前任务阶段决定是否接受并返回状态。
- UDP 接收端同时校验来源 IP/端口、发送者、版本、长度、CRC、HMAC、启动纪元和递增序号。

## 本机验证

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File .\tests\run_tests.ps1
powershell -NoProfile -ExecutionPolicy Bypass -File .\tests\compile_sketches.ps1
```

第一条运行协议黄金向量与 HMI 状态机测试；第二条使用 Arduino/Wi-Fi 桩编译两套完整草图。桩编译不能替代 ESP32 核心编译和实物测试。

地面站界面使用 ASCII 标签以避免在比赛固件中额外占用中文字库空间，项目自有源码注释和交接文档均为中文。

本次地面站代码已用当前电脑的 ESP32 core `3.0.7-cn` 和微雪专用板型做过真实编译：占用 Flash 40%、静态 RAM 28%。车辆固件仍需按实际车辆板型复核。屏幕显示、触摸方向、车辆接线和闭网通信仍须按硬件清单台架验证。

注意：`MISSION_STATUS` 当前为 18 字节，末尾两字节由 ROS 填写无人机、视觉和就绪状态标志；ROS UDP 桥必须按 [协议文档](embedded/shared_protocol/PROTOCOL_V1.md) 同步编码。
