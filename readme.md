# SD-VCU: Software-Defined Vehicle Control Unit
## 5-Train Distributed Virtual Coupling Simulation

### Project Overview
This project implements a high-performance, real-time distributed simulation for train virtual coupling (platooning). It moves beyond static distance protection to a dynamic, cooperative control model that compresses headways safely at high speeds.

### Core Modules
1. **Virtual Physics Engine ($F=ma$)**: A high-fidelity simulator running on a 10ms step, calculating real-time velocity and position based on VCU force outputs.
2. **Virtual T2T Bus**: A lock-free SPSC (Single-Producer Single-Consumer) communication backbone simulating sub-2ms wireless packet exchange between trains.
3. **Dynamic Safety Envelope**: A SIL4-inspired calculus that adjusts safety gaps in real-time based on relative velocity, braking performance (EMU vs. LOCO), and reaction latency.
4. **Multi-core Distributed VCU**: Parallelized control logic with thread-to-core affinity (Cores 1-4), utilizing PD-control laws to maintain string stability across a 5-train platoon.

### Tech Stack
- **Language**: C++
- **Hardware Acceleration**: AVX-512 (SIMD), SSE4.2 (Hardware CRC32)
- **Concurrency**: Lock-free queues, Pthread Affinity, Multithreading
- **Optimization**: O3, Memory Locking (mlockall)

### Simulation Scenarios
- **Auto-Coupling**: Trains automatically compress 50m initial gaps to ~6m dynamic safety buffers.
- **Emergency Braking**: Simulates a Leader brake event at 120km/h; Followers react within milliseconds to maintain safety bubbles without collision.