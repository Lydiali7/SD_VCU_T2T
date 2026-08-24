# Heavy-Haul Virtual Coupling VCU Platoon Simulator

This project is a research and HIL-oriented prototype for heavy-haul virtual train coupling. It models a five-train formation, runs a software-defined VCU control node, evaluates T2T communication behavior, and now includes an engineering-oriented packet boundary based on normalized onboard train-state snapshots.

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
normalized AtpSnapshot + optional T2T engineering frame
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
| `tools/t2t_snapshot_dump.cpp` | Passive T2T snapshot receiver and protocol validator |
| `tools/mock_peer.cpp` | Mock peer OBU: receives snapshots, tracks AoI, optionally sends peer health |
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

`vcu_node` now has a non-blocking `PeerHealthPacket` receiver. It observes peer health and writes it to `vcu_telemetry.csv`; it does not yet change braking or safety state.

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
mock_peer sends PeerHealthPacket back to vcu_node on 9101
vcu_node logs PeerHealthValid, PeerHealthRxAoI, PeerReportedAoI, source IP, received count, and peer seq-gap counters
```

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

1. Validate bidirectional T2T HIL:
   `vcu_node -> AtpSnapshot -> mock_peer -> PeerHealthPacket -> vcu_node`.
2. Feed peer AoI and peer health into the safety supervisor.
3. Drive `DEGRADED_UU`, `BLIND_RUN`, and `FAIL_SAFE_LOCK` from measured peer communication health.
4. Extend mock-peer tests to cover clean link, degraded link, blind-run recovery, and fail-safe lock.
5. Obtain ATP/TCMS/MVB/CAN ICDs for the real train interface.
6. Add `build_snapshot_from_atp_gateway(...)`.
7. Make `vcu_node` choose data source by mode:
   - SIM fallback
   - HIL fallback
   - MVB/CAN gateway
   - ATP/TCMS gateway
8. Cross-check peer-reported safety state against local safety envelope.
9. Replace raw UDP research transport with authenticated, replay-protected safety communication.
10. Move CSV logging to an async logger for real-time operation.

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
