# ESP32 Arduino IDE Team Flow - Work Plan

## TL;DR (For humans)

**What you'll get:** Two Arduino IDE projects: a car controller that line-follows, estimates motion, classifies turns, and publishes UDP telemetry; and a touchscreen station that selects task 1/2 before launch and displays read-only mission status afterward.

**Why this approach:** The airborne computer hosts a closed Wi-Fi hotspot. Both ESP32-S3 boards use fixed, versioned UDP messages so the ROS bridge can reject stale, corrupt, or out-of-order data.

**What it will NOT do:** The touchscreen will not arm, fly, release payload, or alter a mission after vehicle start. Neither board depends on internet access.

**Effort:** Medium
**Risk:** Medium - actual motor, encoder, line-sensor, and display drivers must be selected locally before hardware integration.
**Decisions to sanity-check:** Wi-Fi STA to airborne hotspot; binary UDP v1 with CRC16; car-start locks task selection; ROS is the sole flight-control authority.

## Scope

### Must have

- Separate Arduino IDE projects for `car_esp32s3` and `ground_station_esp32s3` plus a shared protocol header/version document.
- Closed AP/UDP topology, connection recovery, heartbeat, stale-data presentation, and power-on self-tests.
- Car line-following, encoder displacement/velocity, straight/small-turn/large-turn classification, B/D/A event reporting, and one-button start.
- Ground-station pre-start task selection, mission/drone/car status display, and read-only lock after car start.

### Must NOT have (guardrails, anti-slop, scope boundaries)

- No internet, cloud service, OTA update, remote shell, or ROS control logic on either ESP32.
- No touchscreen command that arms motors, moves the aircraft, changes mode, drops payload, or changes task after start.
- No raw UDP packet is trusted without magic/version/length/CRC/sequence validation.

## Verification strategy

- Test decision: tests-after for Arduino unitable protocol/control functions, then bench and closed-network integration tests.
- Evidence: retain serial logs, packet captures, firmware build output, and a versioned test checklist under `.omo/evidence/esp32-team/` during execution.
- Test without propellers: the car and ground station are not permitted to start the FCU or ROS flight-command client.

## Execution strategy

### Parallel execution waves

- Wave 1: common protocol and Wi-Fi/AP contract.
- Wave 2: car motion/telemetry and touchscreen UI/status in parallel.
- Wave 3: ROS bridge integration, faults, and field rehearsal.

### Dependency matrix

| Todo | Depends on | Blocks  | Can parallelize with |
| ---- | ---------- | ------- | -------------------- |
| 1    | none       | 2, 3, 4 | none                 |
| 2    | 1          | 4, 5    | 3                    |
| 3    | 1          | 4, 5    | 2                    |
| 4    | 2, 3       | 5       | none                 |
| 5    | 4          | handoff | none                 |

## Todos

- [ ] 1. Set up both Arduino IDE projects and the closed-network protocol.
  What to do / Must NOT do: In Arduino IDE install/select the exact ESP32 core version for both ESP32-S3 boards; record board model, flash/PSRAM, USB port, partition scheme, and library versions. Create `embedded/car_esp32s3/`, `embedded/ground_station_esp32s3/`, and `embedded/shared_protocol/` with separate `config_local.h` files excluded from git. Use built-in `WiFi.h` and `WiFiUdp.h`; use explicit byte serialization and CRC16, never C struct casting. Define UDP v1 fields: magic, version, message type, payload length, sender boot ID, uint32 sequence, monotonic milliseconds, payload, CRC16. Define car-to-ROS/HMI telemetry, HMI-to-ROS pre-arm selection, ROS-to-HMI mission status, and optional ROS-to-car diagnostic packets; use configured unicast peer IP/ports supplied by the hotspot configuration, not broadcast.
  Parallelization: Wave 1 | Blocked by: none | Blocks: 2, 3, 4
  References: `.omo/plans/d2026-air-ground-collaboration.md:62`; `ros2_ws/src/ed_uav_interfaces/contracts/ros2_contract_manifest.json:1`; `docs/architecture/ROS2_CONTRACTS.md:72`.
  Acceptance criteria: Both boards join the offline AP after reset, reconnect after AP restart, and exchange a CRC-valid heartbeat; bad magic/version/length/CRC, repeated sequence, and stale packet are counted/rejected in serial logs.
  QA scenarios: Happy: AP restart/reconnect and 10-minute heartbeat soak. Failure: bad CRC, reordered sequence, duplicate packet, Wi-Fi loss, and full peer queue. Evidence `.omo/evidence/esp32-team/task-1-network-protocol/`.
  Commit: Y | `feat(embedded): establish ESP32 UDP protocol v1`
- [ ] 2. Implement and bench-test the car ESP32-S3 firmware.
  What to do / Must NOT do: Isolate hardware drivers behind `LineSensors`, `MotorDriver`, `Encoders`, `StartButton`, and `UdpTelemetry` modules; define pins and calibration constants only in `config_local.h`. Implement non-blocking loops: high-rate line sensor/motor PID, encoder integration, slower UDP heartbeat/telemetry, and safety supervision. Publish start state, heartbeat, cumulative displacement in meters, wheel velocity, turn class (`STRAIGHT`, `SMALL`, `LARGE`), B/D/A route events, and completion. Debounce the physical start button; exactly one accepted press transitions `READY -> RUNNING`; lost Wi-Fi continues local line following only for the configured grace interval then applies the documented safe stop/finish policy. Do not infer B/D/A solely from time; derive them from track landmarks/encoder-route calibration with debounced one-shot events.
  Parallelization: Wave 2 | Blocked by: 1 | Blocks: 4, 5
  References: `陆空协同无人机系统（D题）.pdf` pp.1-3; `.omo/plans/d2026-air-ground-collaboration.md:63`; `ros2_ws/src/ed_uav_mission/ed_uav_mission/executor.py:74`.
  Acceptance criteria: Prop-off car bench test shows stable line PID, correct signed displacement, correct three-class turn output, one start event, monotonic sequences, and exactly-once B/D/A events; reset and Wi-Fi-loss policy leave no false start/completion event.
  QA scenarios: Straight, known-radius small/large turn, missed line, encoder disconnect/noise, stuck start button, AP loss/reconnect, and brownout reset. Evidence `.omo/evidence/esp32-team/task-2-car/`.
  Commit: Y | `feat(car): publish line-following vehicle telemetry`
- [ ] 3. Implement and bench-test the touchscreen ground-station firmware.
  What to do / Must NOT do: Isolate display/touch driver choice in `config_local.h`; do not assume a screen library until the panel/controller is identified. Build screens for boot/AP health, task-1/task-2 selection, pre-start confirmation, live car/drone/vision/link status, and fault banner. Send a versioned mission selection UDP message only after explicit confirmation and only while local state is `PRESTART`; subscribe to ROS mission-status packets. When valid car-start telemetry is observed, lock task controls, make display read-only, and retain the selected task/lock reason through a display refresh. Display stale car/ROS packets as faults with last-update age; never synthesize a healthy position.
  Parallelization: Wave 2 | Blocked by: 1 | Blocks: 4, 5
  References: `陆空协同无人机系统（D题）.pdf` pp.2-3; `.omo/plans/d2026-air-ground-collaboration.md:111`; `ros2_ws/src/ed_uav_mission/ed_uav_mission/state_machine.py`.
  Acceptance criteria: Touch tests select either task only before start; task selection is retained across UI redraw; post-start attempts are rejected locally and never transmit a mutation; stale/invalid ROS or car status becomes visibly faulted.
  QA scenarios: invalid touch coordinates, double tap, selection before Wi-Fi, car-start during confirmation, malformed ROS packet, lost car heartbeat, and display reset. Evidence `.omo/evidence/esp32-team/task-3-hmi/`.
  Commit: Y | `feat(hmi): add locked preflight task selection and status UI`
- [ ] 4. Integrate both boards with the airborne ROS UDP bridge.
  What to do / Must NOT do: Configure the hotspot SSID/passphrase/addresses once in local provisioning files, bind the ROS UDP bridge to the exact v1 protocol, and map only validated packets to the D-task vehicle/state contracts. Confirm task selection is stored while the ROS executor is idle/unarmed; use car start, not HMI touch, as the mission trigger; return ROS mission status for display. Log sender boot-ID changes and force a fresh pre-start confirmation after car reboot before mission start. Do not allow either ESP32 to write `/fcu/flight_command` or access the FCU serial device.
  Parallelization: Wave 3 | Blocked by: 2, 3 | Blocks: 5
  References: `.omo/plans/d2026-air-ground-collaboration.md:63`; `ros2_ws/src/ed_uav_interfaces/contracts/ros2_contract_manifest.json:42`; `ros2_ws/src/ed_uav_fcu_bridge/ed_uav_fcu_bridge/serial_port.py`.
  Acceptance criteria: Closed-network integration proves the ROS bridge accepts valid state, rejects corrupt/stale/out-of-order data, locks selection at start, publishes status to HMI, and preserves FCU ownership boundaries.
  QA scenarios: car/HMI reboot, AP restart, UDP flood, packet loss/reorder, stale peers, task-change attack after start, and no-FCU-device test. Evidence `.omo/evidence/esp32-team/task-4-ros-integration/`.
  Commit: Y | `test(embedded): verify ESP32 to ROS integration boundaries`
- [ ] 5. Run team handoff and contest rehearsal.
  What to do / Must NOT do: Produce a one-page wiring/power/pin map per board, Arduino IDE setup screenshot/version record, AP provisioning checklist, packet table, serial log guide, and recovery checklist. Rehearse task 1 and task 2 with propellers removed: pre-start selection, car button start, status lock, B/D/A telemetry, fault presentation, and reset between runs. Field rehearsal may proceed only after the ROS plan's V7/camera/payload capability gates are green; no ESP32 test itself authorizes flight.
  Parallelization: Wave 3 | Blocked by: 4 | Blocks: handoff
  References: `.omo/plans/d2026-air-ground-collaboration.md:139`; `docs/testing/ACCEPTANCE.md`; `docs/deployment/BRINGUP_AND_ROLLBACK.md`.
  Acceptance criteria: A teammate can flash both boards from a clean Arduino IDE setup, join the closed AP, execute the prop-off checklist, and reproduce all expected UI/telemetry states from documentation alone.
  QA scenarios: new-machine build, wrong board/partition selection, missing library, incorrect AP credentials, swapped hardware, and power-cycle recovery. Evidence `.omo/evidence/esp32-team/task-5-handoff/`.
  Commit: Y | `docs(embedded): hand off ESP32 Arduino development workflow`

## Mandatory Addendum (review outcomes)

- Use WPA2 hotspot security plus MAC-based DHCP reservations. The provisioning record must define subnet/gateway, allowed endpoint pairs, AP client-isolation policy, and ownership of every UDP port; never depend on an unverified fixed IP.
- UDP v1 is authenticated as well as checksummed: bounded explicit serialization includes a sender ID, random boot epoch, `uint32` sequence, CRC16, and truncated HMAC-SHA256 tag from local provisioning. Bind a current authenticated session to source IP/port; compare sequence modulo wrap, use local steady receipt time for freshness, and reject replayed start frames after reset.
- Car telemetry is 20 Hz and stale after 0.75 s at the receiver. Define signed origin, filtering, turn thresholds/hysteresis, quality flags, and only legal `START -> B -> D -> A -> COMPLETE` events. Wi-Fi loss beyond 1.0 s, missed line, encoder disagreement, PID overrun, brownout, motor fault, or stuck button commands brake and latches `SAFE_STOP` until physical reset. Reconnection never resumes a stopped run.
- HMI states are `BOOT_LOCKED -> PRESTART -> SELECT_PENDING -> SELECTED -> ARMED_READY -> CAR_RUNNING/FAULT`. Selection includes selection ID plus car boot epoch and becomes visible only after ROS acknowledgement. On HMI reset/reconnect/lost authority it remains locked until ROS confirms a current pre-start epoch. Car start during selection pending/absent reports `NO_COMMITTED_SELECTION` and starts no UAV.
- The ROS UDP bridge is the sole authoritative start state machine: it accepts selection only while FCU is unarmed, then acknowledges it; the operator arms by the approved non-HMI path; exactly one authenticated car-start for the committed epoch dispatches `ExecuteMission`. Every other ordering is rejected with a displayed reason. Telemetry loss in any flight phase invokes the D-task hover/return/land policy.
- Add host-buildable golden-vector protocol tests, fake Wi-Fi/UDP/clock adapters, deterministic HMI state-machine tests, and ROS graph/file-descriptor tests proving only `ed_uav_fcu_bridge` owns `/fcu/flight_command` and FCU serial. Pin exact FQBN/core/library versions and require Arduino CLI clean-machine compilation for both sketches.

## Final verification wave

- [ ] F1. Protocol audit: validate bounded parsing, CRC/version/sequence handling, UDP port ownership, no secrets in source, and no FCU command path.
- [ ] F2. Car/HMI review: validate non-blocking loops, debounce, reset behavior, stale-data UI, and strict post-start HMI lock.
- [ ] F3. Prop-off rehearsal: agent captures both-board serial logs for select/start/B-D-A/fault/recovery flows on the closed AP.
- [ ] F4. Scope audit: verify no cloud/OTA/YOLO/flight-control behavior entered either Arduino project.

## Commit strategy

- Commit shared protocol first, car firmware second, HMI third, ROS integration fourth, and handoff documentation last.
- Do not commit SSID/passphrase, local IP assignments, motor pins, calibration constants, or board-specific credentials from `config_local.h`.

## Success criteria

- Both ESP32-S3 boards work from Arduino IDE on the airplane's closed hotspot with no internet dependency.
- The car publishes reliable motion/turn/event telemetry and has a deterministic local failure policy.
- The ground station selects task 1/2 only before launch, becomes read-only at car start, and accurately renders stale/fault status.
- ROS receives only valid UDP v1 telemetry and remains the only flight-control authority.
