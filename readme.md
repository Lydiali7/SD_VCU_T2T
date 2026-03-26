# SD-VCU: Software-Defined Vehicle Control Unit
## Distributed Virtual Coupling & Platoon Simulation (v2.0 Networked)

### Project Overview
This project implements a **High-Fidelity Distributed Simulation** for Train Virtual Coupling (VC). By moving beyond traditional moving-block signaling, it enables a 5-train platoon to maintain "breathing" safety gaps at high speeds (120km/h+) through real-time **T2T (Train-to-Train)** communication via UDP Multicast.

The system simulates 450-ton EMU trainsets, managing their dynamics under realistic network conditions including jitter and packet loss.

### Core Technical Pillars
1. **Distributed Architecture**: 
   - **World Server**: A physics engine running at 100Hz, broadcasting global states via **UDP Multicast (239.0.0.1)**.
   - **VCU Nodes**: Independent Linux processes representing each train's controller, performing decentralized decision-making.
2. **String Stability (CACC)**: 
   - Implements **Feed-forward Control** by embedding leader acceleration in T2T packets, eliminating error amplification (the "Slinky Effect") across the platoon.
3. **Resilience & Fail-safe**:
   - **Fault Injection**: Simulated 5% random packet loss to test robustness.
   - **Degraded Mode**: Automatic transition to "Safe Coast" braking if T2T communication is lost for >50ms.
4. **Hardware Acceleration**: 
   - **AVX-512** for perception logic and **SSE4.2** for hardware-based CRC32 packet integrity checks.

### Project Structure
* `src/world_server.cpp`: The "God" process. Manages physics and global state broadcast.
* `src/vcu_node.cpp`: The "Brain" process. Runs the control law for an individual train.
* `src/main.cpp`: Legacy single-process multi-threaded simulator (Baseline).
* `include/perception.hpp`: Sensor fusion and safety envelope calculus.
* `include/network_proto.hpp`: UDP binary protocol definition.

### Tech Stack
- **Language**: C++
- **Networking**: UDP Multicast, Socket Programming
- **OS**: Linux (Optimized with `mlockall` and Pthread Affinity)
- **Build**: GNU Make

### How to Run(need different terminal)
1. **Build all targets**:
   ```bash
   make all
2. **Start world_server**
3. **Start the vcu_node one by one**
