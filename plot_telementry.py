import pandas as pd
import matplotlib.pyplot as plt

df = pd.read_csv('vcu_telemetry.csv')

fig, ax1 = plt.subplots(figsize=(12, 6))

# Primary Y-axis for Physical Metrics (Distance)
ax1.set_xlabel('Time (s)')
ax1.set_ylabel('Distance (m)', color='tab:blue')
ax1.plot(df['Time(s)'], df['EstimatedGap(m)'], label='Actual Gap', color='cyan')
ax1.plot(df['Time(s)'], df['SafeGap(m)'], label='Safety Envelope', color='blue', linestyle='--')
ax1.tick_params(axis='y', labelcolor='tab:blue')
ax1.legend(loc='upper left')

# Secondary Y-axis for Cyber Metrics (Network)
ax2 = ax1.twinx()  
ax2.set_ylabel('AoI (ms) / Burst Loss', color='tab:red')
ax2.plot(df['Time(s)'], df['AoI(ms)'], label='Age of Info', color='red', alpha=0.6)
ax2.bar(df['Time(s)'], df['BurstLossCount'] * 50, label='Burst Loss (Scaled)', color='orange', alpha=0.3, width=0.1)
ax2.tick_params(axis='y', labelcolor='tab:red')
ax2.legend(loc='upper right')

plt.title("Cyber-Physical Synergy: Network Degradation vs Safety Envelope")
plt.tight_layout()
plt.show()