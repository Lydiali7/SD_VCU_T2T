# ATP/TCMS Interface to AtpSnapshot Mapping

This document records how authorized train-network data should be normalized before it is used by the T2T application layer.

The rule is:

```text
ATP / ATO / TCMS / MVB / CAN raw interface
        ->
ATPAdapter
        ->
AtpSnapshot
        ->
T2TFrameHeader + typed payload
```

Do not directly forward proprietary ATP/ATO packets as T2T packets. The T2T packet should carry only normalized, timestamped, validity-marked data needed for virtual coupling control and safety supervision.

## Current Implementation Status

| Source | Code path | Status |
|---|---|---|
| SIM fallback | `ATPAdapter::from_fallback()` | Implemented |
| HIL fallback | `ATPAdapter::from_fallback()` | Implemented |
| MVB train line | `ATPAdapter::from_mvb()` | Skeleton/fallback mapping |
| CAN gateway | `ATPAdapter::from_can()` | Skeleton/fallback mapping |
| ATP gateway | `ATPAdapter::from_atp_gateway()` | Placeholder, requires ICD |
| TCMS gateway | Not implemented | Requires ICD |

## Field Mapping Draft

| ICD / train-network meaning | Typical unit | `AtpSnapshot` field | Validity bit | T2T usage |
|---|---:|---|---|---|
| Train identity, `NID_ENGINE`, device ID | enum/id | `source_id` or future train identity field | always implicit | Source verification |
| Timestamp / P1 safety timestamp | us/ms | `timestamp_us` | implicit | AoI, delay budget |
| Train cumulative distance / front position | cm | `train_pos_cm` | `SNAPSHOT_VALID_POS` | Role, gap, KF update |
| Train speed | cm/s | `speed_cmps` | `SNAPSHOT_VALID_SPEED` | Control, safety distance |
| Train acceleration | cm/s^2 | `accel_cmps2` | `SNAPSHOT_VALID_ACCEL` | Prediction |
| ATP permitted speed | cm/s | `permitted_speed_cmps` | `SNAPSHOT_VALID_TARGET_SPEED` | Overspeed supervision |
| Target speed | cm/s | `target_speed_cmps` | `SNAPSHOT_VALID_TARGET_SPEED` | Cruise/control target |
| Movement authority end / MA length | cm | `movement_authority_end_cm` | `SNAPSHOT_VALID_MOVEMENT_AUTHORITY` | ATP boundary supervision |
| Brake cylinder pressure | kPa | `brake_cylinder_pressure_kpa` | `SNAPSHOT_VALID_BRAKE_PRESS` | Brake-state monitoring |
| Brake pipe pressure | kPa | `brake_pipe_pressure_kpa` | `SNAPSHOT_VALID_BRAKE_PRESS` | Brake-state monitoring |
| ATP operating mode | enum | `atp_mode` | `SNAPSHOT_VALID_ATP_MODE` | Formation/authority logic |
| ATP status word | bitfield | `atp_status` | `SNAPSHOT_VALID_ATP_MODE` | Health and safety state |
| EB state | enum/bit | `brake_command` | `SNAPSHOT_VALID_EB_STATUS` | Emergency propagation |
| EB reason | enum/bitfield | `eb_reason` | `SNAPSHOT_VALID_EB_STATUS` | Fault diagnosis |
| Packet sequence | count | `seq` | implicit | Loss/replay detection |
| Packet CRC | CRC32 | `crc32` | implicit | Integrity check |

## ATP/ATO ICD Concepts Seen in the Provided Document

The uploaded ATP/ATO interface document appears to include concepts such as:

```text
ATP -> ATO messages
ATO -> ATP messages
ATP cycle around 200 ms
ATO cycle around 160 ms
CTCS/ETCS packet identifiers
RSSP-1 / P1 safety layer
CRC
LIFE_TIME
DELAY_TIME
speed in cm/s
pressure in kPa
LRBG / balise reference
NID_ENGINE
EB status
```

These are useful references for the normalized snapshot, but the exact byte offsets and enum values must come from the final authorized ICD.

## Recommended Additions After ICD Is Available

The current `AtpSnapshot` is intentionally compact. After the final ICD is available, consider adding these fields only if they are needed by T2T safety or formation control:

| Candidate field | Reason |
|---|---|
| `train_integrity_status` | Virtual coupling should know whether the reported train is complete |
| `formation_id` | Identify current virtual formation |
| `formation_index` | Position inside formation |
| `brake_state` | Distinguish service brake, EB, release, isolation |
| `traction_state` | Diagnose traction cut-off or traction availability |
| `life_time_ms` | Explicit RSSP/P1-style message lifetime |
| `delay_budget_ms` | Separate delay budget from AoI |
| `lrbg_id` | Absolute railway localization reference |

## Design Boundary

`AtpSnapshot` is not an ATP internal memory copy. It is a normalized application contract.

Future real-data integration should modify:

```text
ATPAdapter::from_atp_gateway()
ATPAdapter::from_mvb()
ATPAdapter::from_can()
```

It should not require rewriting:

```text
vcu_node control loop
T2TFrameHeader
T2TAtpSnapshotFrame
Peer safety/formation/health packet structure
```

