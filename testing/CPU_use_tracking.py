import psutil
import time
import matplotlib.pyplot as plt

# Configuration
INTERVAL_SEC = 0.5  # Sample frequency

print("--------------------------------------------------")
print(f"Logging CPU core data every {INTERVAL_SEC}s into separate subplots...")
print("Press Ctrl+C when you want to stop and generate the grid graph.")
print("--------------------------------------------------")

timestamps = []
num_cores = psutil.cpu_count(logical=True)
core_data = {f"Core {i}": [] for i in range(num_cores)}

start_time = time.time()

try:
    while True:
        current_percentages = psutil.cpu_percent(interval=INTERVAL_SEC, percpu=True)
        elapsed_time = time.time() - start_time
        
        timestamps.append(elapsed_time)
        for i, pct in enumerate(current_percentages):
            core_data[f"Core {i}"].append(pct)

except KeyboardInterrupt:
    print("\n[+] Data collection stopped. Building your multi-plot analysis grid...")
    
    if len(timestamps) < 2:
        print("Error: Not enough data points gathered.")
    else:
        # Define a clean color palette for the 4 cores
        colors = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728']
        
        # Create a grid of subplots (num_cores rows, 1 column)
        fig, axes = plt.subplots(nrows=num_cores, ncols=1, figsize=(10, 2.5 * num_cores), sharex=True)
        
        # If the machine somehow returns a single core, wrap it in a list to prevent indexing errors
        if num_cores == 1:
            axes = [axes]
            
        for i in range(num_cores):
            core_label = f"Core {i}"
            y_values = core_data[core_label]
            ax = axes[i]
            
            # 1. Plot the real-time utilization timeline
            ax.plot(timestamps, y_values, color=colors[i % len(colors)], alpha=0.8, linewidth=1.5, label=f"{core_label} Load")
            
            # 2. Calculate and plot the horizontal average line
            core_avg = sum(y_values) / len(y_values)
            ax.axhline(core_avg, color='black', linestyle='--', linewidth=1.5, 
                       label=f"Avg: {core_avg:.1f}%")
            
            # Styling adjustments for each individual subplot
            ax.set_ylabel("CPU %", fontsize=10)
            ax.set_ylim(-5, 105)
            ax.grid(True, linestyle="--", alpha=0.5)
            ax.legend(loc="upper right", frameon=True, fontsize=9)
            ax.set_title(f"Processor Core {i} Timeline Analysis", fontsize=11, fontweight='bold', loc='left', pad=4)

        # Label the shared bottom X-axis
        axes[-1].set_xlabel("Elapsed Time (Seconds)", fontsize=11)
        
        # Save layout cleanly to disk
        output_filename = "cpu_subplots_analysis.png"
        plt.tight_layout()
        plt.savefig(output_filename, dpi=150)
        
        print("--------------------------------------------------")
        print(f"SUCCESS: Split subplot graph saved to: {output_filename}")
        print("--------------------------------------------------")