import pandas as pd
import matplotlib.pyplot as plt

def plot_tunnel_micro_fluctuation():
    filename = 'fleet_log.csv'
    try:
        df = pd.read_csv(filename).dropna(subset=['ID'])
    except FileNotFoundError:
        print(f"Error: {filename} not found.")
        return

    plt.style.use('seaborn-v0_8-whitegrid')
    
    # Perfect time window covering steady state -> tunnel -> just before decoupling
    start_time = 145
    end_time = 200
    
    t3 = df[(df['ID'] == 3) & (df['Time'] >= start_time) & (df['Time'] <= end_time)]

    if t3.empty:
        print("Error: No data found for Train 3 in the specified time range.")
        return

    fig, ax = plt.subplots(figsize=(10, 5), dpi=300)

    # Plot Actual Gap
    ax.plot(t3['Time'], t3['Gap'], label='Actual Gap (T3)', color='#d62728', linewidth=2.5)

    # The real tunnel window: 160s to 190s
    tunnel_start = 160
    tunnel_end = 190
    ax.axvspan(tunnel_start, tunnel_end, color='gray', alpha=0.2, label='Tunnel Blackout (30s)')

    # Calculate mean gap only within the tunnel blackout to center the Y-axis
    tunnel_data = t3[(t3['Time'] >= tunnel_start) & (t3['Time'] <= tunnel_end)]
    gap_mean = tunnel_data['Gap'].mean()
    
    # Annotate FSM Trigger (10s of steady state prior to 160s)
    ax.axvline(x=tunnel_start, color='black', linestyle=':', linewidth=1.5)
    ax.annotate('FSM Trigger:\nSteady State > 10s', 
                xy=(tunnel_start, gap_mean + 4), xytext=(tunnel_start - 6, gap_mean + 4),
                fontsize=10, fontweight='bold', color='#333333')

    ax.set_xlabel('Simulation Time (s)', fontsize=12, fontweight='bold')
    ax.set_ylabel('Distance (m)', fontsize=12, fontweight='bold')
    
    # Lock Y-axis tight to show micro-fluctuation (+/- 5 meters)
    ax.set_ylim(gap_mean - 50, gap_mean + 50) 
    
    # Annotate Kalman Filter at the middle of the tunnel (175s)
    try:
        gap_at_175 = t3[t3['Time'] >= 175]['Gap'].iloc[0]
    except IndexError:
        gap_at_175 = gap_mean

    ax.annotate('Kalman Filter Active:\nSteady Inertial Slide', 
                xy=(175, gap_at_175),        
                xytext=(165, gap_at_175 + 2.0),           
                arrowprops=dict(facecolor='black', shrink=0.05, width=1.5, headwidth=6),
                fontsize=11, fontweight='bold')

    ax.legend(loc='lower right', fontsize=10)
    
    plt.title('Smooth Inertial Slide during FSM-Driven Tunnel Blackout', fontsize=14, fontweight='bold', pad=15)
    plt.tight_layout()
    plt.savefig('fig3_8_tunnel_micro.png', bbox_inches='tight')
    plt.close()
    
    print(f"Success: Actual Tunnel Gap is centered around {gap_mean:.1f}m. Image generated.")

if __name__ == "__main__":
    plot_tunnel_micro_fluctuation()