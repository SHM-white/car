# 固定构建环境

## 版本基线

| 项目 | 固定值 |
| --- | --- |
| Arduino CLI | `1.5.1` |
| Espressif Arduino ESP32 core | 本机实测 `3.0.7-cn`；官方要求 `3.0.6` 以上 |
| 车辆板型 | 待按实物确认，当前使用 `ESP32S3 Dev Module` |
| 地面站 | `Waveshare ESP32-S3-Touch-LCD-7` |
| 地面站 Arduino 板型 | `Waveshare ESP32-S3-Touch-LCD-7` |
| `ESP32_Display_Panel` | `1.0.4` |
| `ESP32_IO_Expander` | `1.1.1` |
| `esp-lib-utils` | `0.2.3` |
| `lvgl` | `8.4.0`，不能使用 9.x |
| 共享协议库 | 本仓库 `DTaskProtocol 1.0.0` |

微雪官方页面要求 ESP32 core 不低于 `3.0.6`，并指定 LVGL `8.4.0`。上述版本已经在当前电脑完成真实 ESP32-S3 编译；更换版本后必须重新执行屏幕与触摸台架测试。

## 地面站菜单

在 Arduino IDE 的“工具”菜单中为微雪地面站设置：

| 菜单 | 值 |
| --- | --- |
| Board | `Waveshare ESP32-S3-Touch-LCD-7` |
| PSRAM | `Enabled` |
| Flash Mode | `QIO 80MHz` |
| Flash Size | `8MB (64Mb)` |
| USB CDC On Boot | `Disabled` |
| Partition Scheme | `Huge APP (3MB No OTA/1MB SPIFFS)` |

对应 FQBN：

```text
esp32:esp32:waveshare_esp32_s3_touch_lcd_7:PSRAM=enabled,FlashMode=qio,FlashSize=8M,CDCOnBoot=default,PartitionScheme=huge_app
```

屏幕驱动使用库内置的 `BOARD_WAVESHARE_ESP32_S3_TOUCH_LCD_7` 配置，包含 800x480 ST7262 RGB565、GT911 I2C 触摸和 CH422G 扩展 IO；不应再手工填写这组显示引脚。

## 安装地面站库

从微雪官方页面下载 `ESP32-S3-Touch-LCD-7-Demo.zip`，将压缩包内 `Arduino/libraries` 下的库复制到 Arduino 项目文件夹的 `libraries` 目录。示例包已包含上表的相互兼容版本。

本工程内已经带有官方 `lvgl_v8_port.*` 与 `lv_conf.h`。`build_opt.h` 会为 ESP32 Arduino 构建统一添加 `LV_CONF_INCLUDE_SIMPLE`，无需修改已安装的 LVGL 源码。

## Arduino CLI

中国大陆网络可按微雪教程增加 Espressif 镜像索引：

```powershell
arduino-cli config add board_manager.additional_urls `
  https://jihulab.com/esp-mirror/espressif/arduino-esp32/-/raw/gh-pages/package_esp32_index_cn.json
arduino-cli core update-index
arduino-cli core install esp32:esp32@3.0.7-cn
```

不使用镜像索引时，安装官方同版本包 `esp32:esp32@3.0.7`。

将官方示例包中的 `Arduino/libraries` 记为 `$waveshareLibraries` 后编译：

```powershell
arduino-cli compile --fqbn esp32:esp32:esp32s3 `
  --library .\embedded\shared_protocol .\embedded\car_esp32s3

arduino-cli compile `
  --fqbn "esp32:esp32:waveshare_esp32_s3_touch_lcd_7:PSRAM=enabled,FlashMode=qio,FlashSize=8M,CDCOnBoot=default,PartitionScheme=huge_app" `
  --libraries $waveshareLibraries `
  --library .\embedded\shared_protocol `
  .\embedded\ground_station_esp32s3
```

## 本地配置

真实配置文件只允许保存在：

- `embedded/car_esp32s3/config_local.h`
- `embedded/ground_station_esp32s3/config_local.h`

不要把热点口令、HMAC 密钥、固定地址、车辆引脚或赛道标定值写回示例文件。两块板和 ROS 必须使用相同的协议版本；地面站只发送预启动任务选择，不拥有飞行控制权限。

## 已完成的编译验证

使用上述地面站 FQBN、ESP32 core `3.0.7-cn` 和固定库版本完成链接：

| 工程 | Flash | 静态 RAM |
| --- | ---: | ---: |
| 地面站 | `1272257 / 3145728`（40%） | `94376 / 327680`（28%） |
| 车辆通用 ESP32-S3 配置 | `910616 / 1310720`（69%） | `47036 / 327680`（14%） |

这只证明工具链、依赖和链接正确，不替代屏幕/触摸、供电、引脚、电机和闭网台架测试。

## 官方资料

- <https://docs.waveshare.net/ESP32-S3-Touch-LCD-7/Development-Environment-Setup-Arduino/>
- <https://files.waveshare.net/wiki/ESP32-S3-Touch-LCD-7/ESP32-S3-Touch-LCD-7-Demo.zip>
- <https://github.com/esp-arduino-libs/ESP32_Display_Panel>
