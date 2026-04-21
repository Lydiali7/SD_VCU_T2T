'''mermaid
graph TD
    %% 定义样式
    classDef brain fill:#1f2937,stroke:#3b82f6,stroke-width:3px,color:#fff;
    classDef sensor fill:#065f46,stroke:#10b981,stroke-width:2px,color:#fff;
    classDef execute fill:#7f1d1d,stroke:#ef4444,stroke-width:2px,color:#fff;
    classDef network fill:#4c1d95,stroke:#8b5cf6,stroke-width:2px,color:#fff;

    %% 外部输入
    subgraph 外部输入源
        T2G[车地通信电台] -- "车站调度/限速指令" --> X86
        T2T_RX[T2T 接收模块\n(LoRa/UDP)] -- "前车位置/速度/故障标志" --> X86
    end
    class T2T_RX network;

    %% 传感器输入
    subgraph 底层感知输入 (CAN总线)
        CAN_A[CAN Path A\n多点激光+编码器] -- "Payload A" --> X86
        CAN_B[CAN Path B\n多点激光+编码器] -- "Payload B" --> X86
    end
    class CAN_A,CAN_B sensor;

    %% 核心计算
    X86(("x86 核心计算层\n(SD-VCU 大脑)"))
    class X86 brain;

    %% x86内部逻辑说明
    note_x86["1. AVX-512 2oo2 极速校验\n2. FTTI 硬件容错滤波\n3. 卡尔曼平滑 & 编队 PID\n4. 重载 ATP 安全包络推演"] -.-> X86

    %% 执行输出
    subgraph 物理执行输出 (MVB总线)
        X86 -- "Hex 控制字\n(牵引/常用制动/紧急制动)" --> MVB{MVB\n多功能车辆总线}
        MVB -- "下达执行" --> BCU[制动控制单元\n空气制动系统]
        MVB -- "下达执行" --> DCU[牵引逆变器\n电机系统]
        BCU -. "底层状态反馈" .-> MVB
        MVB -. "硬件状态" .-> X86
    end
    class MVB,BCU,DCU execute;

    %% 网络输出
    subgraph 编组网络输出 (T2T总线)
        X86 -- "本车实时坐标/加速度\n硬件故障锁定广播" --> T2T_TX[T2T 发送模块\n(LoRa/UDP 频分复用)]
    end
    class T2T_TX network;
    '''