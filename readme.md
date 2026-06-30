# SD-VCU: Software-Defined Vehicle Control Unit
## Distributed Virtual Coupling & Heterogeneous Network Simulation

### 🚂 Project Overview
This project implements a **High-Fidelity Distributed Simulation** for Heavy-Haul Train Virtual Coupling (VC). Specifically optimized for **5,000-ton locomotives**, it enables a 5-train platoon to maintain a highly compressed **150m steady-state gap** at 80km/h. The coordination is achieved through a real-time **Heterogeneous T2T (Train-to-Train)** communication architecture (5G URLLC + 433MHz LoRa).

The system is engineered to maintain stability under extreme environmental uncertainties, including **30-second communication blackouts (long tunnels)** and dynamic topology reconfiguration (in-flight decoupling), mirroring the real-world challenges of SIL4 (Safety Integrity Level 4) heavy-haul rail automation.

### 🛠️ Core Technical Pillars

1. **Finite State Machine (FSM) & Dynamic Decoupling**
   - **Event-Driven Topology**: Utilizes a 4-state FSM (`LEADER_A`, `FOLLOWER`, `DECOUPLING`, `LEADER_B`) to manage the platoon's lifecycle. 
   - **Smooth Separation**: Actively identifies steady-state conditions to trigger decoupling, smoothly extending the gap from 150m to an 800m safe threshold without triggering extreme braking jerks.

2. **Signal Purification & The 30s "Blind Run"**
   - **Kalman Filtering (KF)**: To combat complete 5G signal loss in mountain tunnels, each VCU runs an independent 1D Kalman Filter. 
   - **Heterogeneous Sensor Fusion**: Fuses physical inertia predictions (Dead Reckoning) with sparse, 100ms-cycle LoRa fallback telemetry. This guarantees the gap deviation remains under **6.5m** during a full 30-second blackout, eliminating ghost braking.

3. **Bare-Metal OS Optimization & Determinism**
   - **Isolcpus & IRQ Affinity**: Deployed on a `PREEMPT_RT` Linux kernel. CPU cores 2 and 3 are isolated from the CFS scheduler, and network hardware interrupts are bound to non-isolated cores.
   - **Microsecond Jitter**: This bare-metal tuning locks the simulation control loop step at exactly $10ms \pm 50\mu s$, providing an ideal integration environment for the kinematic equations.

4. **Hardware-Accelerated Protocol & Safety**
   - **Cache-Line Alignment**: The T2T binary protocol is strictly aligned to **64 Bytes**, eliminating cross-cache-line penalty and maximizing CPU L1 cache hit rates.
   - **SSE4.2 Hardware CRC32 & 2oo2 Voting**: Ensures packet integrity at the silicon level. A 2-out-of-2 software redundancy check catches simulated bit-flip attacks in $<1\mu s$, triggering a Fail-Safe degraded mode.

### 📊 System Components
* **`src/world_server.cpp`**: The "Physics Engine." Iterates non-linear vehicle dynamics using the **Davis Equation** ($A+Bv+Cv^2$) and manages extreme actuator saturation limits.
* **`src/vcu_node.cpp`**: The "Distributed Brain." Independent processes representing each train's VCU, running the FSM, Kalman filters, and Feedforward-Feedback PI control laws.
* **`src/perception.cpp`**: The algorithm core. Handles rapid payload unpacking, dynamic safety envelope calculus, and SIMD-based redundancy checks.
* **`include/network_proto.hpp`**: Binary protocol definition utilizing extreme bit-field compression to pack velocity, position, and acceleration into a single 64-bit integer.
* **`scripts/`**: Contains `run_tmux.sh` for one-click virtual network topology generation (vcan0 & socat) and `plot_for_thesis.py` for generating publication-ready matplotlib charts.

### 🚀 Getting Started

#### Prerequisites
- **CPU**: x86_64 architecture with **SSE4.2** support.
- **OS**: Ubuntu 22.04 LTS (Optimized for **Real-time kernels** like `6.12-rt`).
- **Tools**: `cmake`, `tmux`, `socat`, `can-utils`, Python 3 (`pandas`, `matplotlib`).

#### Build Instructions
```bash
mkdir build && cd build
cmake ..
make -j$(nproc)