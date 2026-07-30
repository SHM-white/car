# Ground Station Debug Simulator

独立的 C++ 网络调试工具，通过真实 DTask UDP v1 协议驱动地面站 HMI 状态机，用于在没有硬件的情况下验证协议处理、状态转换和故障恢复。

## 1. 为什么需要这个工具

地面站固件运行在 ESP32-S3 + 7 寸触屏上，无法在开发机上直接运行。本模拟器通过真实 UDP 协议包模拟小车和 ROS 节点的行为，可以：

- 验证 HMI 状态机的状态转换是否正确
- 验证协议包的编码/解码是否一致
- 测试故障场景（链路陈旧、认证失败、序号异常等）
- 在 CI 中进行回归测试，无需硬件

## 2. 构建

```bash
# 在 WSL 中执行
cd /home/shm-white/ed
bash readonly/tests/build_ground_station_sim.sh
```

产出：`readonly/tests/ground_station_sim`

依赖：g++ (C++17)，POSIX sockets，DTaskProtocol 库（仓库自带）。

## 3. 运行

```bash
# 单个场景
./readonly/tests/ground_station_sim --scenario ping

# 所有场景
bash readonly/tests/run_ground_station_sim.sh
```

### 命令行参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `--scenario <name>` | ping | 场景名称 |
| `--seed <n>` | 42 | 随机种子 |
| `--duration-ms <n>` | 5000 | 最大运行时间 |
| `--car-port <n>` | 42001 | 模拟小车 UDP 端口 |
| `--ros-port <n>` | 42000 | 模拟 ROS UDP 端口 |
| `--hmi-port <n>` | 42002 | HMI 监听端口 |
| `--auth-key-hex <64>` | 0001...1F | 认证密钥（64 字符十六进制） |

## 4. 场景说明

| 场景 | 说明 | 验证点 |
|------|------|--------|
| `ping` | 发送一个 CAR_TELEMETRY，等待 HMI 响应 | 基本发送/接收链路 |
| `nominal` | 完整任务生命周期：PRESTART → SELECT → ARMED → RUNNING → COMPLETE | 全流程状态转换 |
| `stale_car` | 发送遥测后停止，模拟链路陈旧 | HMI 检测陈旧数据并进入 BOOT_WAITING |
| `stale_ros` | 停止发送 MISSION_STATUS | HMI 显示 ROS 陈旧 |
| `auth_mismatch` | 用错误密钥发送包 | HMI 拒绝认证失败的包 |
| `sequence_replay` | 发送重复序号 | HMI 拒绝重复包 |
| `boot_id_change` | 运行中更改 boot_id | HMI 重置状态机到 BOOT_WAITING |
| `sequence_wrap` | 序号接近 UINT32_MAX 后回绕 | 协议层正确处理序号回绕 |

### 场景退出码

- `0` = PASS（场景通过）
- `1` = FAIL（断言失败）
- `2` = 错误（参数错误、socket 失败等）

## 5. 与真实地面站配合

当需要与真实地面站固件配合测试时：

1. 烧录地面站固件到 ESP32-S3 开发板
2. 确保开发板和运行模拟器的机器在同一网络
3. 使用 `--hmi-port` 指定地面站的 UDP 端口
4. 地面站会响应 TASK_SELECTION 包，模拟器的 nominal 场景可以验证完整的选择确认流程

## 6. 证据记录

运行脚本会在 `.omo/evidence/ground-station-sim/<run_id>/` 下记录：

- `command.txt` — 运行参数
- `<scenario>.log` — 每个场景的完整输出
- `runner.log` — 运行器摘要
- `SUCCESS` 或 `FAILED` — 门禁标记

## 7. 扩展新场景

在 `ground_station_sim.cpp` 中：

1. 实现场景函数 `static int scenarioMyTest(SimConfig &cfg)`
2. 在 `main()` 的 `scenarios[]` 数组中添加条目
3. 重新编译

场景函数可以使用：
- `SimulatedPeer` 发送 CAR_TELEMETRY、MISSION_STATUS、HEARTBEAT
- `HmiReceiver` 接收 TASK_SELECTION、HEARTBEAT
- `sendRaw()` 发送原始字节（用于损坏测试）

## 8. 集成检查清单

当测试任务后端实现完成后，需要：

- [ ] 在 `DTaskProtocol.h` 中添加 `TEST_TASK = 3` 到 `MessageType` 或扩展 `TaskSelection`
- [ ] 在 `HmiStateMachine.h` 中修改 `chooseTask()` 接受任务 3
- [ ] 在 `UdpLink.h` 中确保新消息类型被正确路由
- [ ] 在模拟器中添加 `test_task_selection` 场景
- [ ] 更新本文档

## 9. 与硬件测试的关系

本模拟器是**离线门禁工具**，不替代硬件测试：

| 验证层级 | 工具 | 何时使用 |
|----------|------|----------|
| 协议正确性 | `protocol_tests` | 每次提交 |
| 状态机逻辑 | `protocol_tests` | 每次提交 |
| 网络集成 | `ground_station_sim` | 每次提交 |
| 硬件功能 | 串口日志 + 台架 | 烧录前 |
| 闭网集成 | 真实三端联调 | 赛前 |

## 10. 资料

- DTask 协议说明：`readonly/embedded/shared_protocol/PROTOCOL_V1.md`
- 地面站使用说明：`readonly/embedded/ground_station_esp32s3/GROUND_STATION_USER_GUIDE.md`
- 构建说明：`readonly/embedded/BUILD.md`
- 测试基础设施：`readonly/tests/`
