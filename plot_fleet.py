import pandas as pd
import matplotlib.pyplot as plt
import os

def main():
    filename = 'fleet_log.csv'
    if not os.path.exists(filename):
        print(f"Error: {filename} not found. Please run the simulation first.")
        return

    print(f"Loading data from {filename}...")
    df = pd.read_csv(filename)

    # Convert units for better readability
    df['Vel_kmh'] = df['Vel'] * 3.6
    df['Force_kN'] = df['Force'] / 1000.0

    print("Generating Velocity Profile...")
    plt.figure(figsize=(10, 6))
    for i in range(5):
        train_data = df[df['ID'] == i]
        plt.plot(train_data['Time'], train_data['Vel_kmh'], label=f'Train {i}')
    plt.title('Platoon Velocity Profile')
    plt.xlabel('Time (s)')
    plt.ylabel('Velocity (km/h)')
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.savefig('velocity_profile.png')
    plt.close()

    print("Generating Gap vs Safe Distance Analysis...")
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    axes = axes.flatten()
    for idx, i in enumerate(range(1, 5)):
        train_data = df[df['ID'] == i]
        axes[idx].plot(train_data['Time'], train_data['Gap'], label=f'Actual Gap (T{i})', color='blue')
        axes[idx].plot(train_data['Time'], train_data['Safe'], label=f'Safe Distance (T{i})', color='red', linestyle='--')
        axes[idx].set_title(f'Train {i} Spacing')
        axes[idx].set_xlabel('Time (s)')
        axes[idx].set_ylabel('Distance (m)')
        axes[idx].legend()
        axes[idx].grid(True)
    plt.tight_layout()
    plt.savefig('gap_analysis.png')
    plt.close()

    print("Generating Command Force Profile...")
    plt.figure(figsize=(10, 6))
    for i in range(5):
        train_data = df[df['ID'] == i]
        plt.plot(train_data['Time'], train_data['Force_kN'], label=f'Train {i}')
    plt.title('Platoon Command Force Profile')
    plt.xlabel('Time (s)')
    plt.ylabel('Force (kN)')
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.savefig('force_profile.png')
    plt.close()

    print("Success! Plots saved to current directory:")
    print("- velocity_profile.png")
    print("- gap_analysis.png")
    print("- force_profile.png")

if __name__ == "__main__":
    main()