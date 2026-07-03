import time
import psutil
import matplotlib.pyplot as plt

# --- CONFIGURATION ---
SCRIPT_NAME = "main.py"
# Set this to "first" to track your oldest run, or "second" to track the one you launched next
TARGET_INSTANCE = "second" 
INTERVAL_SEC = 0.5

print("--------------------------------------------------")
print(f"Tracking isolated CPU for the {TARGET_INSTANCE.upper()} instance of {SCRIPT_NAME}")
print("Press Ctrl+C to stop and generate the graph.")
print("--------------------------------------------------")

timestamps = []
num_cores = psutil.cpu_count(logical=True)
core_data = {f"Core {i}": [] for i in range(num_cores)}

def get_sorted_instances(script):
    """Finds all root processes running the script and sorts them oldest-to-newest."""
    instances = []
    for proc in psutil.process_iter(['pid', 'cmdline', 'create_time']):
        try:
            cmd = proc.info['cmdline']
            if cmd and any(script in arg for arg in cmd):
                # Identify the root process by ensuring its parent isn't also running main.py
                parent = proc.parent()
                if parent and parent.cmdline() and any(script in arg for arg in parent.cmdline()):
                    continue
                instances.append({
                    'pid': proc.pid,
                    'create_time': proc.info['create_time'],
                    'proc_obj': proc
                })
        except (psutil.NoSuchProcess, psutil.AccessDenied):
            continue
            
    instances.sort(key=lambda x: x['create_time'])
    return instances

start_time = time.time()

try:
    while True:
        all_runs = get_sorted_instances(SCRIPT_NAME)
        target_pids = []
        
        if all_runs:
            if TARGET_INSTANCE == "first" and len(all_runs) >= 1:
                root_proc = all_runs[0]['proc_obj']
            elif TARGET_INSTANCE == "second" and len(all_runs) >= 2:
                root_proc = all_runs[1]['proc_obj']
            else:
                root_proc = None
                
            if root_proc:
                try:
                    target_pids.append(root_proc.pid)
                    for child in root_proc.children(recursive=True):
                        target_pids.append(child.pid)
                except (psutil.NoSuchProcess, psutil.AccessDenied):
                    pass

        core_totals = [0.0] * num_cores
        
        # Aggregate the core utilization by inspecting threads within our target family tree
        if target_pids:
            for pid in target_pids:
                try:
                    p = psutil.Process(pid)
                    # 1. Get total CPU usage percentage for this whole process block
                    total_proc_pct = p.cpu_percent(interval=None)
                    
                    # 2. Find out which cores this process's internal threads are active on
                    threads = p.threads()
                    if threads:
                        # Find out which core handles each thread (using psutil to check cpu_num)
                        active_cores = []
                        for t in threads:
                            try:
                                # Retrieve detailed thread metrics to see current core placement
                                t_info = p.cpu_num() if hasattr(p, 'cpu_num') else None
                                if t_info is not None:
                                    active_cores.append(t_info)
                            except (psutil.NoSuchProcess, psutil.AccessDenied):
                                continue
                        
                        # Fallback if cpu_num isn't picking up sub-threads cleanly
                        if not active_cores:
                            active_cores = [pid % num_cores] # Distribute evenly as fallback
                        
                        # 3. Distribute the total process CPU load evenly across its active cores
                        pct_per_thread = total_proc_pct / len(active_cores)
                        for core_idx in active_cores:
                            if core_idx < num_cores:
                                core_totals[core_idx] += pct_per_thread
                    else:
                        # If no sub-threads are exposed, assume it's running on its main allocated core
                        if hasattr(p, 'cpu_num'):
                            core_totals[p.cpu_num()] += total_proc_pct
                except (psutil.NoSuchProcess, psutil.AccessDenied):
                    continue
                    
        elapsed_time = time.time() - start_time
        timestamps.append(elapsed_time)
        
        for i in range(num_cores):
            core_data[f"Core {i}"].append(min(core_totals[i], 100.0))
            
        time.sleep(INTERVAL_SEC)

except KeyboardInterrupt:
    print(f"\n[+] Stopped. Generating graph for the {TARGET_INSTANCE} instance...")
    
    if len(timestamps) < 2:
        print("Error: Run longer to populate data points.")
    else:
        colors = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728']
        fig, axes = plt.subplots(nrows=num_cores, ncols=1, figsize=(10, 2.5 * num_cores), sharex=True)
        
        if num_cores == 1:
            axes = [axes]
            
        for i in range(num_cores):
            core_label = f"Core {i}"
            y_values = core_data[core_label]
            ax = axes[i]
            
            ax.plot(timestamps, y_values, color=colors[i % len(colors)], alpha=0.8, linewidth=1.5, label="Isolated Load")
            core_avg = sum(y_values) / len(y_values)
            ax.axhline(core_avg, color='black', linestyle='--', linewidth=1.5, label=f"Avg: {core_avg:.1f}%")
            
            ax.set_ylabel("CPU %", fontsize=10)
            ax.set_ylim(-5, 105)
            ax.grid(True, linestyle="--", alpha=0.5)
            ax.legend(loc="upper right", frameon=True, fontsize=9)
            ax.set_title(f"Processor {core_label} Profile ({TARGET_INSTANCE.upper()} Instance)", fontsize=11, fontweight='bold', loc='left', pad=4)

        axes[-1].set_xlabel("Elapsed Time (Seconds)", fontsize=11)
        output_filename = f"cpu_instance_{TARGET_INSTANCE}_analysis.png"
        plt.tight_layout()
        plt.savefig(output_filename, dpi=150)
        print(f"SUCCESS: Split graph saved to: {output_filename}")