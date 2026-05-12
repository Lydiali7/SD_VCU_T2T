import pandas as pd
import matplotlib.pyplot as plt

def generate_thesis_plots():
    filename = 'fleet_log.csv'
    try:
        df = pd.read_csv(filename).dropna(subset=['ID'])
    except FileNotFoundError:
        print(f"Error: {filename} not found.")
        return

    # Convert units
    df['Vel_kmh'] = df['Vel'] * 3.6
    df['Force_kN'] = df['Force'] / 1000.0

    # Apply academic style
    plt.style.use('seaborn-v0_8-whitegrid')
    #plt.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei'] # For Chinese labels in doc
    #plt.rcParams['axes.unicode_minus'] = False
    
    colors = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728', '#9467bd']

    # ---------------------------------------------------------
    # Figure 1: Startup & Cruise (0-100s)
    # ---------------------------------------------------------
    print("Generating Figure 1: Startup Profile...")
    fig, ax1 = plt.subplots(figsize=(10, 6), dpi=300)
    
    for i in range(5):
        train = df[(df['ID'] == i) & (df['Time'] <= 100)]
        ax1.plot(train['Time'], train['Vel_kmh'], label=f'Train {i} Velocity', color=colors[i], linewidth=2, alpha=0.8)

    ax1.set_xlabel('Simulation Time (s)', fontsize=12, fontweight='bold')
    ax1.set_ylabel('Velocity (km/h)', fontsize=12, fontweight='bold')
    ax1.set_xlim(0, 100)
    
    ax2 = ax1.twinx()
    for i in range(1, 5):
        train = df[(df['ID'] == i) & (df['Time'] <= 100)]
        ax2.plot(train['Time'], train['Gap'], linestyle='--', color=colors[i], alpha=0.5)
        
    ax2.set_ylabel('Actual Gap (m)', fontsize=12, fontweight='bold')
    ax2.set_ylim(180, 220)
    
    ax1.axvline(x=50, color='gray', linestyle=':', linewidth=1.5)
    ax1.annotate('t=50s Decoupling Trigger', xy=(50, 80), xytext=(55, 85),
                 arrowprops=dict(facecolor='black', shrink=0.05, width=1, headwidth=5))

    fig.legend(loc='upper left', bbox_to_anchor=(0.1, 0.9), ncol=2, fontsize=10)
    plt.title('Startup and Platoon Synchronization', fontsize=14, fontweight='bold', pad=15)
    plt.tight_layout()
    plt.savefig('fig3_8_startup.png', bbox_inches='tight')
    plt.close()

    # ---------------------------------------------------------
    # Figure 2: Dynamic Decoupling Topology (Full lifecycle)
    # ---------------------------------------------------------
    print("Generating Figure 2: Decoupling Profile...")
    fig, ax1 = plt.subplots(figsize=(10, 6), dpi=300)

    t0 = df[df['ID'] == 0]
    t3 = df[df['ID'] == 3]

    ax1.plot(t0['Time'], t0['Vel_kmh'], label='Leader (T0) Velocity', color='#1f77b4', linewidth=2.5)
    ax1.plot(t3['Time'], t3['Vel_kmh'], label='Decoupled (T3) Velocity', color='#d62728', linewidth=2.5)
    ax1.set_xlabel('Simulation Time (s)', fontsize=12, fontweight='bold')
    ax1.set_ylabel('Velocity (km/h)', fontsize=12, fontweight='bold')
    ax1.set_xlim(0, 450)
    
    ax2 = ax1.twinx()
    ax2.plot(t3['Time'], t3['Gap'], label='T3 Gap to Predecessor (m)', color='#2ca02c', linestyle='-.', linewidth=2.5)
    ax2.plot(t3['Time'], t3['Safe'], label='Dynamic Safe Envelope (m)', color='black', linestyle=':', linewidth=2)
    ax2.set_ylabel('Distance (m)', fontsize=12, fontweight='bold')
    
    ax1.axvline(x=50, color='gray', linestyle='--', linewidth=1.5)
    ax1.annotate('Decoupling Triggered', xy=(50, 75), xytext=(70, 50),
                 arrowprops=dict(facecolor='black', shrink=0.05, width=1.5, headwidth=6))
    ax2.axhline(y=800, color='red', linestyle='--', alpha=0.5)
    ax2.annotate('800m Safety Threshold', xy=(400, 800), xytext=(250, 850), color='red', fontsize=11, fontweight='bold')

    fig.legend(loc='lower right', bbox_to_anchor=(0.9, 0.15), fontsize=10)
    plt.title('FSM-Driven Dynamic Decoupling Topology', fontsize=14, fontweight='bold', pad=15)
    plt.tight_layout()
    plt.savefig('fig3_10_decoupling.png', bbox_inches='tight')
    plt.close()

    # ---------------------------------------------------------
    # Figure 3: Actuator Smoothness Verification
    # ---------------------------------------------------------
    print("Generating Figure 3: Actuator Force Profile...")
    fig, ax = plt.subplots(figsize=(10, 5), dpi=300)

    for i in range(4):
        train = df[(df['ID'] == i) & (df['Time'] <= 150)]
        ax.plot(train['Time'], train['Force_kN'], label=f'Train {i} Command Force', color=colors[i], linewidth=1.5, alpha=0.9)

    ax.set_xlabel('Simulation Time (s)', fontsize=12, fontweight='bold')
    ax.set_ylabel('Force (kN)', fontsize=12, fontweight='bold')
    ax.set_xlim(0, 150)
    ax.axhline(y=0, color='black', linewidth=1)
    ax.axvline(x=50, color='gray', linestyle='--', linewidth=1)

    ax.annotate('Smooth Braking Transition', xy=(55, -200), xytext=(65, -600),
                arrowprops=dict(facecolor='black', shrink=0.05, width=1.5, headwidth=6), fontsize=11)

    ax.legend(loc='upper right', fontsize=10)
    plt.title('Actuator Output Smoothness (0-150s)', fontsize=14, fontweight='bold', pad=15)
    plt.tight_layout()
    plt.savefig('fig3_11_force.png', bbox_inches='tight')
    plt.close()

    print("Success! High-DPI plots saved to current directory.")

if __name__ == "__main__":
    generate_thesis_plots()