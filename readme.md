# SD-VCU: Software-Defined Vehicle Control Unit
## Distributed Virtual Coupling & Heavy-Haul Platoon Simulation (v2.1)

### 🚂 Project Overview
This project implements a **High-Fidelity Distributed Simulation** for Heavy-Haul Train Virtual Coupling (VC). Specifically optimized for **5,000-ton locomotives**, it enables a 5-train platoon to maintain a precise **50m "breathing" gap** at 80km/h. The coordination is achieved through real-time **T2T (Train-to-Train)** communication via UDP Multicast.

The system is engineered to maintain stability under extreme environmental uncertainties, including **5% network packet loss** and **high-variance Gaussian sensor noise**, mirroring the real-world challenges of modern heavy-haul rail automation.



### 🛠️ Core Technical Pillars

1. **Heavy-Haul Physics & Traction Headroom**
   - **Massive Inertia**: Simulates a **5,000,000 kg** point-mass model using the **Davis Equation** ($A+Bv+Cv^2$) for non-linear air and rolling resistance.
   - **Traction Headroom Strategy**: The Lead Train is capped at **400kN** (40% capacity), providing Followers with **600kN of "Catch-up Reserve"**. This physical margin is critical for maintaining string stability and counteracting network-induced delays.

2. **Signal Purification (Kalman Filtering)**
   - **Noise Resiliency**: To combat injected **Gaussian noise ($\sigma=1.5m$)**, each VCU node runs independent **1D Kalman Filters** for position, velocity, and acceleration. This ensures a stable control loop even when raw telemetry is volatile.

3. **SIMD-Accelerated Perception & Safety**
   - **AVX-512 Instruction Set**: Utilized for $O(1)$ bit-unpacking of telemetry data, maximizing throughput on x86 architectures.
   - **SSE4.2 Hardware CRC32**: Ensures packet integrity at the hardware level, meeting the rigorous demands of SIL4 (Safety Integrity Level 4) development.
   - **2oo2 (2-out-of-2) Voting**: Implements software-level redundancy checks to validate sensor consistency across parallel calculation paths.

4. **Control Theory & Fail-safe**
   - **Soft PID Tuning**: Specifically damped for heavy-haul inertia ($P=0.005, D=0.05$) to eliminate "Rubber Band" oscillations.
   - **Degraded Mode**: Automatic transition to a **Safe Coast/Braking Mode** if the T2T communication link is lost for $>500ms$.
   - **Elastic Safety Envelope**: Uses a dynamic threshold ($ds \cdot 0.8$) to filter out "Ghost Braking" caused by transient network spikes.

### 📊 System Components
* **`src/world_server.cpp`**: The "Physics Engine." Iterates vehicle dynamics at 100Hz and maintains a global real-time **Telemetry Dashboard**.
* **`src/vcu_node.cpp`**: The "Distributed Brain." Independent processes representing each train's VCU, running control laws and Kalman filters.
* **`src/perception.cpp`**: The algorithm core. Handles SIMD unpacking, safety distance calculus, and 2oo2 logic.
* **`include/network_proto.hpp`**: Binary protocol definition for T2T Multicast and Force Reporting.

### 🚀 Getting Started

#### Prerequisites
- **CPU**: x86_64 with **AVX-512** support (Intel Skylake-X/Ice Lake or newer, AMD Zen 4+).
- **OS**: Linux (Optimized for **Real-time kernels** like `PREEMPT_RT`).

#### Build Instructions
```bash
make clean && make all