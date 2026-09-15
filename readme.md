# Heavy-Haul Virtual Coupling VCU Platoon Simulator

This project is a research and HIL-oriented prototype for heavy-haul virtual train coupling. It models a five-train formation, runs a software-defined VCU control node, evaluates T2T communication behavior, and now includes an engineering-oriented packet boundary based on normalized onboard train-state snapshots.

For the current Chinese project description, completed HIL evidence, engineering boundaries, and next-stage plan, see [docs/project_status_zh.md](docs/project_status_zh.md).

The current code is best understood as:

```text
SIM / HIL physical world
        |
        v
world_server physics and feedback
        |
        v
vcu_node control, safety envelope, AoI, blind-run handling
        |
        v
normalized AtpSnapshot + bidirectional T2T peer snapshot
        |
        v
CommHealthState -> shadow CommSafetySupervisor -> shadow CommSafetyAction
```

It is not yet a certified ATP/TCMS integration. Real ATP/TCMS/MVB/CAN field mapping still requires the actual interface control document, ICD.

---

## Current Capabilities

- Five-train heavy-haul formation simulation with heterogeneous locomotives and wagon loads.
- Single active DUT VCU node, currently train `3`, controlling against the simulated physical world.
- SIM and HIL runtime modes selected by environment variable or command line.
- Position-map scenarios in SIM:
  - tunnel/blind-run area
  - NLOS curve area
  - downhill, uphill, and curve resistance
  - decoupling and new-formation tracking states
- HIL-ready UDP communication between `world_server` and `vcu_node`.
- Optional HIL impairment injection:
  - deterministic blackout
  - random packet loss
  - artificial latency and jitter
- Safety logic:
  - AoI tracking
  - blind-run Kalman prediction
  - uncertainty buffer
  - topography-aware safe-gap calculation
  - ATP trip and fail-safe lock states
- Engineering packet layer:
  - `AtpSnapshot`
  - `T2TFrameHeader`
  - `T2TAtpSnapshotFrame`
  - fixed-point position/speed/acceleration encoding
  - validity mask and CRC fields
- HAL scaffolding for:
  - CAN / SocketCAN / USB-CAN
  - MVB serial board interface
  - LoRa E22 serial safety-plane radio

---

## Repository Layout

| Path | Purpose |
|---|---|
| `src/world_server.cpp` | Physical world, fleet dynamics, feedback generation, HIL impairment injection |
| `src/vcu_node.cpp` | Main VCU control loop for DUT train 3 |
| `src/perception.cpp` | Packet decoding, Kalman tracker, safe-distance calculation |
| `src/infra/rt_system.cpp` | Real-time scheduling and CPU affinity setup |
| `include/network_proto.hpp` | UDP packets, T2T frame definitions, `AtpSnapshot`, protocol helpers |
| `include/train_dynamics.hpp` | Locomotive and wagon parameters, fleet configuration |
| `include/perception.hpp` | Sensor data structures and perception API |
| `include/can_hal.hpp` | SocketCAN and USB-CAN abstraction |
| `include/mvb_hal.hpp` | MVB serial device setup and frame read/write |
| `include/t2t_radio.hpp` | LoRa/E22 serial radio wrapper |
| `include/safety_config.hpp` | Safety thresholds and control constants |
| `include/comm_health.hpp` | Normalized peer communication-health contract |
| `include/comm_safety_supervisor.hpp` | Shadow communication-safety state-machine interface |
| `include/comm_safety_action.hpp` | Shadow communication-safety action resolver interface |
| `src/comm_safety_supervisor.cpp` | Hysteretic AoI-based safety policy implementation |
| `src/comm_safety_action.cpp` | State-to-action matrix implementation |
| `tools/t2t_snapshot_dump.cpp` | Passive T2T snapshot receiver and protocol validator |
| `tools/mock_peer.cpp` | Mock peer OBU: receives local snapshots, publishes modeled peer snapshots, and sends peer health |
| `tests/test_comm_safety_supervisor.cpp` | Boundary, recovery, and fail-safe latch tests |
| `tests/test_comm_safety_action.cpp` | Shadow action-matrix tests |
| `run_tmux.sh` | SIM-mode tmux dashboard launcher |
| `Makefile` | Builds runtime binaries and protocol test tools |
| `fleet_log.csv` | World-server fleet log output |
| `vcu_telemetry.csv` | VCU telemetry output |

---

## Architecture

### SIM Mode

```text
world_server
  - integrates all five train dynamics
  - applies track topology and scenario map
  - injects SIM tunnel/NLOS behavior
  - sends WorldFeedbackPacket

vcu_node
  - represents DUT train 3
  - receives front-train gap and speed
  - estimates front state with Kalman tracker
  - computes force command
  - reports ForceReportPacket to world_server
```

SIM includes artificial packet behavior. In `vcu_node`, non-HIL mode may reject received packets using `PerceptionEngine::get_packet_error_rate(...)`. In `world_server`, SIM also applies position-dependent latency and tunnel link loss.

### HIL Mode

```text
physical/HIL host running world_server
        ^
        | UDP port 9000
        v
OBU / industrial PC running vcu_node
```

In HIL, the tunnel/NLOS SIM logic is disabled by default. Real UDP arrival determines whether feedback is available. Bad communication scenarios are only injected when you explicitly set `HIL_*` variables.

### Engineering Packet Boundary

The project now avoids directly mixing simulation variables and future ATP/TCMS data into the T2T payload. The intended engineering boundary is:

```text
ATP / TCMS / MVB / CAN authorized data
        |
        v
HAL or gateway decoding
        |
        v
AtpSnapshot / normalized onboard state
        |
        v
T2TFrameHeader + typed payload + CRC
        |
        v
peer OBU
```

Current fallback behavior:

```text
SIM/HIL internal position and velocity
        |
        v
build_snapshot_from_sim(...)
        |
        v
AtpSnapshot
        |
        v
optional T2TAtpSnapshotFrame
```

Real ATP parsing is not implemented because the project does not yet have the ATP/TCMS ICD. The correct future work is to add `build_snapshot_from_atp_gateway(...)` or replace the current MVB/CAN field mapping using the authorized ICD.

---

## Protocol Summary

### World Feedback

`WorldFeedbackPacket` is sent from `world_server` to `vcu_node`.

Main fields:

```text
true_gap
front_vel
ego_pos
ego_vel
flags
tx_timestamp_us
```

### Force Report

`ForceReportPacket` is sent from `vcu_node` to `world_server`.

Main fields:

```text
train_id
cmd_force
status_flag
current_vel
current_pos
```

### Normalized Snapshot

`AtpSnapshot` is the normalized onboard-state snapshot.

Important principles:

- fixed-point integer units
- timestamp
- sequence number
- validity mask
- source identifier
- CRC

Examples:

```text
train_pos_cm
speed_cmps
accel_cmps2
brake_pipe_pressure_kpa
atp_status
brake_command
eb_reason
```

### Engineering T2T Frame

`T2TAtpSnapshotFrame` wraps:

```text
T2TFrameHeader
T2TAtpSnapshotPayload
frame_crc32
```

This is optional output from `vcu_node`, enabled by `T2T_SNAPSHOT_IP`.

---

## Build

Requirements:

- Linux
- `g++`
- pthreads
- Socket/CAN headers
- optional: `tmux`, `socat`, `can-utils`

Build:

```bash
make
```

Clean:

```bash
make clean
```

The current `Makefile` uses:

```text
-O3 -march=alderlake -mavx2 -msse4.2 -pthread
```

If the target CPU is not Alder Lake/x86 with these features, adjust `CXXFLAGS` in `Makefile`.

---

## Run SIM Mode

Recommended:

```bash
./run_tmux.sh
```

The script builds the project, starts virtual CAN/serial helpers, launches `world_server`, then starts `vcu_node`.

Manual SIM run:

Terminal 1:

```bash
SDVCU_MODE=SIM ./world_server --sim
```

Terminal 2:

```bash
SDVCU_MODE=SIM LORA_DEV=/tmp/lora-node ./vcu_node 3 --sim
```

Note: the current `vcu_node` source still fixes DUT id to train `3`. The positional argument is kept for launch compatibility but is not yet a full dynamic train-id selector in the active source.

---

## Run HIL Mode

On the world-server host:

```bash
SDVCU_MODE=HIL SERVER_BIND_IP=10.42.0.1 ./world_server --hil
```

On the VCU/OBU host:

```bash
SDVCU_MODE=HIL SERVER_IP=10.42.0.1 ./vcu_node 3 --hil
```

Startup rule:

```text
Start world_server first, then start vcu_node.
```

The code includes startup grace on the VCU side, but the correct HIL procedure is still server-first.

---

## HIL Fault Injection

HIL fault injection is controlled by environment variables on `world_server`. It is off by default.

### Clean HIL

```bash
SDVCU_MODE=HIL SERVER_BIND_IP=10.42.0.1 ./world_server --hil
```

### Latency and Jitter

```bash
SDVCU_MODE=HIL SERVER_BIND_IP=10.42.0.1 \
HIL_DELAY_BASE_MS=40 HIL_DELAY_JITTER_MS=10 \
./world_server --hil
```

### Random Packet Loss

```bash
SDVCU_MODE=HIL SERVER_BIND_IP=10.42.0.1 \
HIL_RANDOM_DROP_PROB=0.2 \
./world_server --hil
```

### Blind-Run Recovery Test

Drop feedback for 3 seconds:

```bash
SDVCU_MODE=HIL SERVER_BIND_IP=10.42.0.1 \
HIL_DROP_START_S=180 HIL_DROP_DURATION_S=3 HIL_DROP_PROB=1.0 \
./world_server --hil
```

Expected:

```text
AoI rises
VCU enters BLIND_RUN
feedback resumes
VCU recovers
```

### Fail-Safe Test

Drop feedback for longer than the fail-safe AoI threshold:

```bash
SDVCU_MODE=HIL SERVER_BIND_IP=10.42.0.1 \
HIL_DROP_START_S=180 HIL_DROP_DURATION_S=6 HIL_DROP_PROB=1.0 \
./world_server --hil
```

Expected:

```text
AoI > 5000 ms
VCU enters FAIL_SAFE_LOCK
```

---

## Optional T2T Snapshot Frame Output

To transmit the engineering-style normalized snapshot frame:

```bash
SDVCU_MODE=HIL SERVER_IP=10.42.0.1 \
T2T_SNAPSHOT_IP=10.42.0.33 T2T_SNAPSHOT_PORT=9100 \
./vcu_node 3 --hil
```

This sends:

```text
T2TFrameHeader + AtpSnapshot + frame_crc32
```

The source data currently comes from SIM/HIL fallback:

```text
simulated_my_pos
simulated_my_vel
local_accel_mps2
report_status
```

The protocol is ready for future ATP/TCMS/MVB/CAN mapping, but real ATP decoding must wait for the authorized ICD.

### Passive Snapshot Dump

On the receiving host:

```bash
./t2t_snapshot_dump 9100 3
```

Arguments:

```text
9100  listen port
3     expected source train id
```

This validates the T2T frame, checks CRC, records the latest peer snapshot, and prints peer AoI and sequence-gap statistics.

### Mock Peer OBU

`mock_peer` is the first step toward a bidirectional T2T HIL loop. It behaves like a simple peer OBU:

```text
receive T2TAtpSnapshotFrame
validate header and CRC
store latest peer state
track peer AoI and sequence gaps
optionally inject receiver-side delay/drop
optionally send PeerHealthPacket periodically
publish a modeled peer AtpSnapshot back to the VCU
```

The safety ownership is deliberately asymmetric only in the test harness, not in the VCU decision model:

```text
peer AtpSnapshot received by VCU
        |
        v
local receive supervision: CRC, identity, destination, sequence, AoI
        |
        v
CommHealthState -> shadow CommSafetySupervisor -> shadow CommSafetyAction

PeerHealthPacket
        |
        v
cross-check and diagnostic input only
```

Start a mock peer receiver:

```bash
./mock_peer 9100 3
```

Enable receiver-side degradation inside the mock peer:

```bash
MOCK_PEER_RX_DROP_PROB=0.2 \
MOCK_PEER_RX_DELAY_BASE_MS=30 \
MOCK_PEER_RX_DELAY_JITTER_MS=20 \
./mock_peer 9100 3
```

Send periodic `PeerHealthPacket` back to a VCU host:

```bash
MOCK_PEER_HEALTH_IP=10.42.0.33 \
MOCK_PEER_HEALTH_PORT=9101 \
./mock_peer 9100 3
```

If `MOCK_PEER_HEALTH_IP` is not set, `mock_peer` can automatically use the source IP of the latest received snapshot:

```bash
MOCK_PEER_HEALTH_AUTO_TARGET=1 \
MOCK_PEER_HEALTH_PORT=9101 \
./mock_peer 9100 3
```

The mock output prints the actual health target and byte count:

```text
[PEER_HEALTH] seq=... target=10.42.0.33:9101 bytes=48 ...
```

`vcu_node` has a non-blocking `PeerHealthPacket` receiver. It writes peer health to `vcu_telemetry.csv` as a secondary diagnostic cross-check; it does not directly drive the controller, braking, or communication-safety state. The local peer-snapshot receiver is the primary evidence source.

On the VCU side:

```bash
SDVCU_MODE=HIL SERVER_IP=10.42.0.1 \
T2T_SNAPSHOT_IP=10.42.0.1 T2T_SNAPSHOT_PORT=9100 \
VCU_PEER_HEALTH_PORT=9101 \
./vcu_node 3 --hil
```

With this setup:

```text
vcu_node sends AtpSnapshot to mock_peer on 9100
mock_peer publishes a modeled Train 2 AtpSnapshot to vcu_node on 9100
mock_peer sends PeerHealthPacket back to vcu_node on 9101
vcu_node locally validates and logs peer snapshot freshness, source, sequence gaps, duplicates, replay attempts, and invalid frames
vcu_node logs PeerHealth as cross-check telemetry
```

For a two-host HIL test, run this on the PC/mock-peer host:

```bash
MOCK_PEER_HEALTH_IP=10.42.0.33 \
MOCK_PEER_HEALTH_PORT=9101 \
MOCK_PEER_SNAPSHOT_IP=10.42.0.33 \
MOCK_PEER_SNAPSHOT_PORT=9100 \
./mock_peer 9100 3
```

Run this on the VCU host:

```bash
SDVCU_MODE=HIL SERVER_IP=10.42.0.1 \
T2T_SNAPSHOT_IP=10.42.0.1 T2T_SNAPSHOT_PORT=9100 \
VCU_EXPECTED_PEER_ID=2 VCU_EXPECTED_PEER_IP=10.42.0.1 \
VCU_PEER_SNAPSHOT_PORT=9100 VCU_PEER_HEALTH_PORT=9101 \
./vcu_node 3 --hil
```

`PeerSnapshotRxAoI` is the local authoritative age of the most recently accepted peer state. `PeerHealthRxAoI` is only the age of the diagnostic health message.

`vcu_node` aggregates receiver observations into `CommHealthState` (Phase 2A), then evaluates it through the hysteretic `CommSafetySupervisor` (Phase 2B) and `CommSafetyAction` resolver (Phase 2C-1). All three stages are currently shadow-only: they are logged but do not affect force, EB, ATP trip, uncertainty, legacy `NetworkHealth`, or fail-safe behavior.

```text
snapshot_rx_aoi_ms         age of the last fully accepted peer snapshot
missing_snapshot_count     total inferred missing sequence numbers
current_burst_loss_count   missing frames immediately before the latest accepted frame
max_burst_loss_count       largest observed single loss burst
crc_error_count            invalid snapshot/frame CRC count
peer health fields         secondary cross-check information only
```

`SeqGapCount` means the number of discontinuity events. It is intentionally different from `MissingSnapshotCount`: a jump from sequence 100 to 105 is one gap event but four missing snapshots.

When both `VCU_COMM_SAFETY_SHADOW=1` and `VCU_COMM_ACTION_SHADOW=1` are set, the VCU also resolves the proposed communication state into telemetry-only `CommSafetyAction` values:

```text
NORMAL          traction permitted, cooperative following permitted
DEGRADED        traction ratio limited, added uncertainty
BLIND_RUN       traction inhibited, cooperative following disabled, blind-run requested
FAIL_SAFE_LOCK  traction inhibited, fail-safe brake requested
```

This is still shadow output. No current force, EB, ATP-trip, uncertainty, or fail-safe control path reads these action fields.

Troubleshooting:

```text
mock_peer rx_count increasing, but vcu_node PeerHealthValid=0 and Invalid=0
    -> PC is not delivering PeerHealthPacket to vcu_node's UDP socket.

mock_peer [PEER_HEALTH] target points to PC IP instead of VCU IP
    -> set MOCK_PEER_HEALTH_IP to the VCU host IP, or use auto target.

mock_peer [PEER_HEALTH] bytes=44, but vcu_node Rx count stays 0
    -> check VCU_PEER_HEALTH_PORT, VCU_PEER_BIND_IP, firewall, and whether the sample runs on the expected network interface.

vcu_node Invalid increases
    -> the port is receiving a packet, but it is not PeerHealthPacket size/header.
```

---

## Environment Variables

### Common

| Variable | Default | Meaning |
|---|---:|---|
| `SDVCU_MODE` | `SIM` | Runtime mode: `SIM` or `HIL` |
| `LORA_DEV` | `/tmp/lora-node` if present, else `/dev/ttyUSB0` | LoRa serial device |

### World Server

| Variable | Default | Meaning |
|---|---:|---|
| `SERVER_BIND_IP` | `0.0.0.0` in SIM, `192.168.1.10` in HIL | UDP bind IP |
| `HIL_DROP_START_S` | `-1` | Start time of HIL feedback blackout |
| `HIL_DROP_DURATION_S` | `0` | Duration of blackout |
| `HIL_DROP_PROB` | `0` | Drop probability during blackout |
| `HIL_DROP_TARGET_ID` | `3` | Train id targeted by blackout |
| `HIL_RANDOM_DROP_PROB` | `0` | Random feedback drop probability |
| `HIL_DELAY_BASE_MS` | `0` | Artificial HIL feedback delay |
| `HIL_DELAY_JITTER_MS` | `0` | Artificial delay jitter range |

### VCU Node

| Variable | Default | Meaning |
|---|---:|---|
| `SERVER_IP` | `127.0.0.1` in SIM, `192.168.1.10` in HIL | World-server target IP |
| `VCU_STARTUP_GRACE_MS` | from `SafetyConfig` | Grace window before first feedback |
| `VCU_HIL_DEGRADED_AOI_MS` | `30` | HIL latency threshold for degraded status |
| `T2T_SNAPSHOT_IP` | unset | Enables optional normalized T2T snapshot output |
| `T2T_SNAPSHOT_PORT` | `9100` | UDP target port for snapshot output |
| `VCU_PEER_HEALTH_RX` | `1` | Enable `PeerHealthPacket` receiver |
| `VCU_PEER_BIND_IP` | `0.0.0.0` | Local bind IP for peer health receiver |
| `VCU_PEER_HEALTH_PORT` | `9101` | Local UDP port for `PeerHealthPacket` |
| `VCU_EXPECTED_PEER_ID` | unset | Expected mock peer sender id; unset means accept any |
| `VCU_PEER_SNAPSHOT_RX` | `1` | Enable peer `T2TAtpSnapshotFrame` receiver |
| `VCU_PEER_SNAPSHOT_PORT` | `9100` | Local UDP port for peer snapshots |
| `VCU_PEER_SNAPSHOT_BIND_IP` | `0.0.0.0` | Local bind IP for peer snapshot receiver |
| `VCU_EXPECTED_PEER_IP` | unset | Expected IPv4 source of peer snapshots; unset means accept any |
| `VCU_PEER_MAX_TIMESTAMP_AGE_MS` | `0` | Optional peer timestamp-age rejection; enable only after PTP/TAI synchronization is verified |
| `VCU_COMM_SAFETY_SHADOW` | `0` | Enable proposed communication safety state logging; never changes vehicle control |
| `VCU_COMM_SAFETY_STARTUP_GRACE_MS` | `8000` | Delay shadow-policy evaluation after VCU boot |
| `VCU_COMM_DEGRADED_AOI_MS` | `300` | Research/shadow AoI threshold for proposed `DEGRADED` |
| `VCU_COMM_BLIND_AOI_MS` | `1000` | Research/shadow AoI threshold for proposed `BLIND_RUN` |
| `VCU_COMM_FAIL_SAFE_AOI_MS` | `5000` | Research/shadow AoI threshold for proposed latched `FAIL_SAFE_LOCK` |
| `VCU_COMM_RECOVER_NORMAL_AOI_MS` | `150` | AoI ceiling for `DEGRADED -> NORMAL` recovery snapshots |
| `VCU_COMM_RECOVER_DEGRADED_AOI_MS` | `500` | AoI ceiling for `BLIND_RUN -> DEGRADED` recovery snapshots |
| `VCU_COMM_RECOVERY_GOOD_SNAPSHOTS` | `3` | Consecutive accepted snapshots required for each recovery transition |
| `VCU_COMM_ACTION_SHADOW` | `0` | Enable telemetry-only `CommSafetyAction` resolution; requires communication safety shadow mode |
| `VCU_COMM_ACTION_DEGRADED_TRACTION_RATIO` | `0.60` | Research/shadow traction ratio proposed for `DEGRADED` |
| `VCU_COMM_ACTION_DEGRADED_UNCERTAINTY_M` | `30` | Research/shadow extra uncertainty proposed for `DEGRADED` |
| `VCU_COMM_ACTION_BLIND_TRACTION_RATIO` | `0` | Research/shadow traction ratio proposed for `BLIND_RUN` |
| `VCU_COMM_ACTION_BLIND_UNCERTAINTY_M` | `100` | Research/shadow extra uncertainty proposed for `BLIND_RUN` |
| `VCU_COMM_ACTION_FAIL_SAFE_UNCERTAINTY_M` | `150` | Research/shadow extra uncertainty proposed for `FAIL_SAFE_LOCK` |

### Mock Peer

| Variable | Default | Meaning |
|---|---:|---|
| `MOCK_PEER_LISTEN_PORT` | `9100` | UDP port for incoming `T2TAtpSnapshotFrame` |
| `MOCK_PEER_EXPECTED_SOURCE_ID` | unset | Expected source train id; unset means accept any |
| `MOCK_PEER_ID` | `2` | Sender id used in `PeerHealthPacket` |
| `MOCK_PEER_HEALTH_IP` | unset | Enables health packet output when set |
| `MOCK_PEER_HEALTH_AUTO_TARGET` | `1` | Auto-reply to the latest snapshot source IP when explicit health IP is unset |
| `MOCK_PEER_HEALTH_PORT` | `9101` | UDP target port for peer health output |
| `MOCK_PEER_HEALTH_PERIOD_MS` | `1000` | Peer health transmit period |
| `MOCK_PEER_FRESH_TIMEOUT_MS` | `1000` | AoI freshness threshold used by mock health |
| `MOCK_PEER_SNAPSHOT_TX` | `1` | Enable modeled peer snapshot transmission |
| `MOCK_PEER_SNAPSHOT_IP` | unset | Explicit VCU target IP for modeled peer snapshots |
| `MOCK_PEER_SNAPSHOT_AUTO_TARGET` | `1` | Auto-target the source IP of the latest received VCU snapshot |
| `MOCK_PEER_SNAPSHOT_PORT` | `9100` | UDP target port for modeled peer snapshots |
| `MOCK_PEER_SNAPSHOT_PERIOD_MS` | `50` | Modeled peer snapshot transmission period |
| `MOCK_PEER_SNAPSHOT_GAP_M` | `400` | Position offset used to model the peer train ahead of the received VCU state |
| `MOCK_PEER_SNAPSHOT_TX_DROP_PROB` | `0` | Probability of dropping modeled peer snapshots after sequence allocation |
| `MOCK_PEER_RX_DROP_PROB` | `0` | Mock receiver-side packet drop probability |
| `MOCK_PEER_RX_DELAY_BASE_MS` | `0` | Mock receiver-side base delay |
| `MOCK_PEER_RX_DELAY_JITTER_MS` | `0` | Mock receiver-side random delay range |

---

## Logs

### `fleet_log.csv`

Generated by `world_server`.

Contains:

```text
time
position / velocity / force / status for trains 0-4
```

### `vcu_telemetry.csv`

Generated by `vcu_node`.

Contains:

```text
AoI
burst loss
time jitter
estimated gap
safe gap
state
effective delay
uncertainty buffer
safety margin
tunnel / NLOS / blind-run indicators
```

---

## Status Codes

| Code | Meaning |
|---:|---|
| `0` | Normal/follower |
| `1` | `ATP_TRIP` |
| `2` | `DEGRADED_UU` |
| `3` | `BLIND_RUN` |
| `4` | `DECOUPLING` |
| `5` | `LEADER_NEW` |
| `6` | `SIM_COMPLETE` |
| `99` | `FAIL_SAFE_LOCK` |

---

## Current Limitations

- Active `vcu_node.cpp` is still centered on DUT train `3`.
- `NodeRole`, `PeerStatePacket`, `PeerSafetyPacket`, `PeerFormationPacket`, and `PeerHealthPacket` exist as protocol scaffolding, but the active VCU node currently uses the normalized `AtpSnapshot` output path rather than a complete dual-OBU role-switching controller.
- Real ATP/TCMS parsing is not implemented.
- Current MVB/CAN snapshot conversion helpers are fallback mappings based on available local frame structures, not a vendor ICD.
- HIL physical ego state is still not sourced from a real odometry/ATP/TCMS gateway by default.
- The communication protocol is research/HIL-oriented and not a certified safety communication stack.

---

## Engineering Roadmap

Recommended next steps:

1. Completed: validate symmetric bidirectional T2T HIL:
   `vcu_node -> AtpSnapshot -> mock_peer -> peer AtpSnapshot -> vcu_node`, with `PeerHealthPacket` as cross-check telemetry.
2. Completed: aggregate VCU-local snapshot freshness and packet-validation results into telemetry-only `CommHealthState`.
3. Completed: run `CommHealthState.snapshot_rx_aoi_ms` through a hysteretic, shadow-only `CommSafetySupervisor`.
4. Completed: resolve proposed communication states into telemetry-only `CommSafetyAction` values.
5. Completed: perform core physical-shadow HIL checks: clean link, 50% loss observation, 3 s recovery path, and 6 s fail-safe latch path.
6. Connect reviewed communication safety actions to vehicle control, beginning with positive-traction inhibit/limit only.
7. Add deterministic, optional snapshot-only blackout and delay/jitter/burst-loss test profiles to `mock_peer`.
8. Define peer reboot/session-epoch handling before accepting a sequence reset.
9. Obtain ATP/TCMS/MVB/CAN ICDs for the real train interface.
10. Add `build_snapshot_from_atp_gateway(...)`.
11. Make `vcu_node` choose data source by mode:
   - SIM fallback
   - HIL fallback
   - MVB/CAN gateway
   - ATP/TCMS gateway
12. Cross-check peer-reported safety state against the local safety envelope.
13. Replace raw UDP research transport with authenticated, replay-protected safety communication.
14. Move CSV logging to an async logger for real-time operation.

---

## Research Statement

This repository demonstrates a staged migration from simulation variables to an engineering-oriented T2T communication boundary:

```text
simulation/HIL control scaffold
        ->
normalized onboard snapshot
        ->
typed T2T application frame
        ->
future ATP/TCMS/MVB/CAN authorized data integration
```

The current `AtpSnapshot` design should be treated as a normalized interface contract. Its field mapping must be finalized against real train-network ICDs before claiming real ATP integration.
