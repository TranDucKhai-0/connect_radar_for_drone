#!/usr/bin/env python3
"""
Log Timing & Jitter Analyzer
===========================
This tool calculates the time intervals (delta time) and jitter between consecutive
data receptions in log files. It supports:
1. CSV logs (e.g. radar_log.csv, raw_radar_log.csv, cubefc_radar_log.csv).
2. SocketCAN candump logs.
3. Standard text logs with timestamp prefixes.

It runs out-of-the-box using built-in libraries if pandas/numpy/matplotlib are not installed.
"""

import os
import sys
import re
import argparse
import math
import csv

# Try importing analysis libraries
HAS_PANDAS = False
try:
    import pandas as pd
    import numpy as np
    HAS_PANDAS = True
except ImportError:
    pass

# Try importing matplotlib for plotting
HAS_MATPLOTLIB = False
try:
    import matplotlib.pyplot as plt
    HAS_MATPLOTLIB = True
except ImportError:
    pass


# Unified regex to parse candump lines
# Formats: 
#   (1689218204.123456) can0 60A#0500...
#   (1689218204.123456)  can0  60A   [8]  05 ...
CANDUMP_PATTERN = re.compile(r'^\s*\((?P<ts>\d+\.\d+)\)\s+\S+\s+[0-9A-Fa-f]+')

# General timestamp patterns:
# 1. ISO/Datestring with subseconds: 2026-07-15 15:47:03.123 or 15:47:03.123456
ISO_TIME_PATTERN = re.compile(r'(?P<hour>\d{2}):(?P<min>\d{2}):(?P<sec>\d{2})\.(?P<ms>\d+)')
# 2. Decimal at start of line: 12345.6789: log message
DECIMAL_START_PATTERN = re.compile(r'^\s*(?P<ts>\d+\.\d+)\b')


def parse_text_line_timestamp(line):
    """
    Tries to extract a timestamp in milliseconds from a raw text log line.
    Returns: Timestamp in ms (float), or None if not found.
    """
    # 1. Try candump format: (seconds.usec)
    match = CANDUMP_PATTERN.match(line)
    if match:
        return float(match.group('ts')) * 1000.0

    # 2. Try ISO / standard time format: HH:MM:SS.mmm
    match = ISO_TIME_PATTERN.search(line)
    if match:
        h = int(match.group('hour'))
        m = int(match.group('min'))
        s = int(match.group('sec'))
        ms_str = match.group('ms')[:6]  # Limit to microsecond precision
        ms_val = float(f"0.{ms_str}")
        # Convert time of day to total milliseconds
        total_ms = ((h * 3600) + (m * 60) + s + ms_val) * 1000.0
        return total_ms

    # 3. Try floating point number at start of line: 17129.2312 or epoch
    match = DECIMAL_START_PATTERN.match(line)
    if match:
        val = float(match.group('ts'))
        # If it looks like epoch seconds (e.g. 1.6e9 to 2.5e9)
        if 1e9 < val < 2.5e9:
            return val * 1000.0
        # Otherwise treat as seconds
        return val * 1000.0

    return None


def detect_unit_and_convert(raw_times, col_name):
    col_name_lower = col_name.lower()
    
    # 1. Check column name clues first
    if 'usec' in col_name_lower or 'micro' in col_name_lower:
        unit = 'usec (from col name)'
        converted_times = [t / 1000.0 for t in raw_times] if isinstance(raw_times, list) else raw_times / 1000.0
        return converted_times, unit
    elif 'ms' in col_name_lower or 'milli' in col_name_lower:
        unit = 'ms (from col name)'
        return raw_times, unit
    elif 'sec' in col_name_lower or 'second' in col_name_lower:
        unit = 'sec (from col name)'
        converted_times = [t * 1000.0 for t in raw_times] if isinstance(raw_times, list) else raw_times * 1000.0
        return converted_times, unit
        
    # 2. Fallback to magnitude check
    if HAS_PANDAS and not isinstance(raw_times, list):
        mean_val = np.mean(raw_times)
    else:
        mean_val = sum(raw_times) / len(raw_times)
        
    unit = 'sec (magnitude)'
    if mean_val > 1e14:  # Likely microseconds (epoch microsec)
        converted_times = [t / 1000.0 for t in raw_times] if isinstance(raw_times, list) else raw_times / 1000.0
        unit = 'usec (magnitude)'
    elif mean_val > 1e11:  # Likely milliseconds (epoch millisec)
        converted_times = raw_times
        unit = 'ms (magnitude)'
    elif mean_val > 1e8:  # Likely seconds (epoch sec)
        converted_times = [t * 1000.0 for t in raw_times] if isinstance(raw_times, list) else raw_times * 1000.0
        unit = 'sec (magnitude)'
    else:
        if HAS_PANDAS and not isinstance(raw_times, list):
            max_val = np.max(raw_times)
        else:
            max_val = max(raw_times)
            
        if max_val > 100000000:
            converted_times = [t / 1000.0 for t in raw_times] if isinstance(raw_times, list) else raw_times / 1000.0
            unit = 'usec (magnitude fallback)'
        elif max_val > 100000:
            converted_times = raw_times
            unit = 'ms (magnitude fallback)'
        else:
            converted_times = [t * 1000.0 for t in raw_times] if isinstance(raw_times, list) else raw_times * 1000.0
            unit = 'sec (magnitude fallback)'
            
    return converted_times, unit


def load_csv_timestamps_pandas(file_path, group_by_ts=True):
    """
    Reads CSV log and extracts timestamps using Pandas.
    """
    df = pd.read_csv(file_path)
    if df.empty:
        print("Error: CSV file is empty.")
        sys.exit(1)

    time_col = None
    col_lower = [c.lower() for c in df.columns]
    priorities = ['timestampusec', 'timeusec', 'timestamp_usec', 'time_usec',
                  'timestampms', 'timems', 'timestamp_ms', 'time_ms',
                  'timestamp', 'time', 'ts']
    
    for p in priorities:
        if p in col_lower:
            idx = col_lower.index(p)
            time_col = df.columns[idx]
            break

    if not time_col:
        for col in df.columns:
            if 'time' in col.lower() or 'ts' in col.lower():
                time_col = col
                break

    if not time_col:
        numeric_cols = df.select_dtypes(include=[np.number]).columns
        if len(numeric_cols) > 0:
            time_col = numeric_cols[0]
            print(f"Warning: No explicit timestamp column found. Using first numeric column: '{time_col}'")
        else:
            print(f"Error: Could not find any numeric timestamp column. Columns: {list(df.columns)}")
            sys.exit(1)

    print(f"Detected timestamp column: '{time_col}'")
    raw_times = df[time_col].dropna().values
    if len(raw_times) == 0:
        print("Error: Timestamp column is empty or contains only NaNs.")
        sys.exit(1)

    raw_times, unit = detect_unit_and_convert(raw_times, time_col)
    print(f"Detected timestamp unit: {unit}")

    if group_by_ts:
        unique_times = np.unique(raw_times)
        print(f"Grouped {len(raw_times)} rows into {len(unique_times)} unique frames.")
        return np.sort(unique_times)
    else:
        return np.sort(raw_times)


def load_csv_timestamps_fallback(file_path, group_by_ts=True):
    """
    Reads CSV log and extracts timestamps using built-in CSV module (no pandas/numpy required).
    """
    with open(file_path, 'r', encoding='utf-8', errors='ignore') as f:
        reader = csv.reader(f)
        header = next(reader, None)
        if not header:
            print("Error: CSV file is empty.")
            sys.exit(1)

        # Detect column index
        col_lower = [c.lower() for c in header]
        priorities = ['timestampusec', 'timeusec', 'timestamp_usec', 'time_usec',
                      'timestampms', 'timems', 'timestamp_ms', 'time_ms',
                      'timestamp', 'time', 'ts']
        time_idx = None
        for p in priorities:
            if p in col_lower:
                time_idx = col_lower.index(p)
                break
        
        if time_idx is None:
            # Fall back to search for time/ts keyword
            for i, col in enumerate(header):
                if 'time' in col.lower() or 'ts' in col.lower():
                    time_idx = i
                    break
        
        if time_idx is None:
            # Fall back to first column
            time_idx = 0
            
        print(f"Detected timestamp column: '{header[time_idx]}' (Fallback parser)")

        raw_times = []
        for row in reader:
            if not row or len(row) <= time_idx:
                continue
            val_str = row[time_idx].strip()
            if not val_str:
                continue
            try:
                raw_times.append(float(val_str))
            except ValueError:
                pass

        if not raw_times:
            print("Error: Could not parse any numeric timestamps from CSV.")
            sys.exit(1)

        raw_times.sort()

        raw_times, unit = detect_unit_and_convert(raw_times, header[time_idx])
        print(f"Detected timestamp unit: {unit}")

        if group_by_ts:
            unique_times = sorted(list(set(raw_times)))
            print(f"Grouped {len(raw_times)} rows into {len(unique_times)} unique frames.")
            return unique_times
        else:
            return sorted(raw_times)


def load_text_timestamps(file_path):
    """
    Reads a text log file line by line and extracts timestamps.
    Returns: Sorted list of timestamps in milliseconds.
    """
    timestamps = []
    line_count = 0
    parsed_count = 0

    with open(file_path, 'r', errors='ignore') as f:
        for line in f:
            line_count += 1
            ts_ms = parse_text_line_timestamp(line)
            if ts_ms is not None:
                timestamps.append(ts_ms)
                parsed_count += 1

    print(f"Read {line_count} lines from text file, extracted {parsed_count} timestamps.")
    if not timestamps:
        print("Error: Could not extract any timestamps from the text file.")
        sys.exit(1)

    timestamps.sort()
    return timestamps


def analyze_intervals(timestamps, target_cycle=None, threshold_multiplier=1.5, max_valid_interval=50.0, min_threshold=30.0):
    """
    Computes delta times and timing statistics. Supports both NumPy arrays and standard lists.
    Filters deltas below max_valid_interval for active transmission statistics (e.g. < 50ms).
    Also counts intervals below min_threshold (e.g. < 30ms).
    """
    n_samples = len(timestamps)
    if n_samples < 2:
        print("Error: Not enough data points to analyze intervals (minimum 2 required).")
        sys.exit(1)

    # Compute deltas in ms
    deltas = []
    for i in range(1, n_samples):
        deltas.append(timestamps[i] - timestamps[i-1])

    # Calculate duration
    duration_ms = timestamps[-1] - timestamps[0]
    duration_sec = duration_ms / 1000.0

    # Filter deltas for active cycle statistics (e.g. < 50ms)
    valid_deltas = [d for d in deltas if d < max_valid_interval]
    
    if not valid_deltas:
        print(f"Warning: No intervals found below the maximum valid threshold of {max_valid_interval} ms. Using all deltas.")
        valid_deltas = deltas

    # Basic stats on active cycle (using standard methods in case numpy is not loaded)
    mean_delta = sum(valid_deltas) / len(valid_deltas)
    
    # Median of valid active cycles
    s_deltas = sorted(valid_deltas)
    mid = len(s_deltas) // 2
    if len(s_deltas) % 2 == 0:
        median_delta = (s_deltas[mid - 1] + s_deltas[mid]) / 2.0
    else:
        median_delta = s_deltas[mid]
        
    min_delta = min(deltas)
    max_active_delta = max(valid_deltas)
    max_delta = max(deltas)
    
    # Standard deviation (jitter) of active cycles
    variance = sum((x - mean_delta) ** 2 for x in valid_deltas) / len(valid_deltas)
    jitter = math.sqrt(variance)

    # If target cycle is not provided, estimate it using the median of valid deltas
    if target_cycle is None:
        target_cycle = median_delta
        print(f"Estimated target cycle (median of valid < {max_valid_interval}ms): {target_cycle:.2f} ms")
    else:
        print(f"Target cycle: {target_cycle:.2f} ms")

    # Threshold for anomaly detection
    anomaly_threshold = target_cycle * threshold_multiplier

    # Identify anomalies out of ALL deltas
    anomaly_indices = [i for i, x in enumerate(deltas) if x > anomaly_threshold]
    num_anomalies = len(anomaly_indices)
    pct_anomalies = (num_anomalies / len(deltas)) * 100.0

    # Count fast cycles below min_threshold
    fast_indices = [i for i, x in enumerate(deltas) if x < min_threshold]
    num_fast_cycles = len(fast_indices)
    pct_fast_cycles = (num_fast_cycles / len(deltas)) * 100.0 if deltas else 0.0

    # Jitter relative to target cycle
    rel_jitter = (jitter / target_cycle) * 100.0 if target_cycle > 0 else 0.0

    stats = {
        'total_samples': n_samples,
        'total_intervals': len(deltas),
        'valid_intervals': len(valid_deltas),
        'duration_sec': duration_sec,
        'mean_delta_ms': mean_delta,
        'median_delta_ms': median_delta,
        'min_delta_ms': min_delta,
        'max_active_delta_ms': max_active_delta,
        'max_delta_ms': max_delta,
        'jitter_ms': jitter,
        'rel_jitter_pct': rel_jitter,
        'frequency_hz': 1000.0 / mean_delta if mean_delta > 0 else 0.0,
        'target_cycle_ms': target_cycle,
        'anomaly_threshold_ms': anomaly_threshold,
        'num_anomalies': num_anomalies,
        'pct_anomalies': pct_anomalies,
        'num_fast_cycles': num_fast_cycles,
        'pct_fast_cycles': pct_fast_cycles,
        'max_valid_interval_ms': max_valid_interval,
        'min_threshold_ms': min_threshold,
    }

    # Find anomalies details
    anomalies = []
    for idx in anomaly_indices:
        occ_sec = (timestamps[idx] - timestamps[0]) / 1000.0
        val = deltas[idx]
        anomalies.append({
            'index': int(idx),
            'elapsed_sec': occ_sec,
            'timestamp_ms': timestamps[idx],
            'delta_ms': val
        })
    
    # Sort anomalies by delta size descending
    anomalies = sorted(anomalies, key=lambda x: x['delta_ms'], reverse=True)

    return deltas, stats, anomalies


def print_dashboard(stats, anomalies, file_path, verbose=False):
    """
    Renders a CLI report. By default, it prints a clean, concise summary of key timing metrics.
    With verbose=True, it includes anomalies list and file paths.
    """
    # ANSI Colors
    C_BLUE = '\033[94m'
    C_GREEN = '\033[92m'
    C_YELLOW = '\033[93m'
    C_RED = '\033[91m'
    C_BOLD = '\033[1m'
    C_RESET = '\033[0m'

    if not verbose:
        # Simple, concise mode requested by the user
        print(f"\n{C_BLUE}{C_BOLD}=== THÔNG SỐ CHU KỲ NHẬN LOG ({os.path.basename(file_path)}) ==={C_RESET}")
        print(f" ├─ Bộ lọc hoạt động (Filter)         : < {stats['max_valid_interval_ms']:.1f} ms")
        print(f" ├─ Chu kỳ trung bình (Avg Cycle) : {C_BOLD}{stats['mean_delta_ms']:.2f} ms{C_RESET}")
        print(f" ├─ Tần số nhận (Frequency)       : {C_GREEN}{stats['frequency_hz']:.2f} Hz{C_RESET}")
        print(f" ├─ Chu kỳ ngắn nhất (Min Cycle)  : {stats['min_delta_ms']:.2f} ms")
        print(f" ├─ Chu kỳ hoạt động dài nhất    : {stats['max_active_delta_ms']:.2f} ms (trong bộ lọc)")
        print(f" ├─ Khoảng trễ lớn nhất (Max Gap) : {C_RED if stats['max_delta_ms'] > stats['max_valid_interval_ms'] else C_RESET}{stats['max_delta_ms']:.2f} ms{C_RESET} (ngắt kết nối/dừng log)")
        print(f" ├─ Độ lệch chu kỳ / Jitter       : {stats['jitter_ms']:.2f} ms ({stats['rel_jitter_pct']:.1f}%)")
        print(f" ├─ Số chu kỳ nhanh (< {stats['min_threshold_ms']:.1f} ms)  : {C_YELLOW if stats['num_fast_cycles'] > 0 else C_RESET}{stats['num_fast_cycles']}{C_RESET} / {stats['total_intervals']} frames ({stats['pct_fast_cycles']:.1f}%)")
        print(f" └─ Số khung hình lỗi (Anomalies) : {C_YELLOW if stats['num_anomalies'] > 0 else C_RESET}{stats['num_anomalies']}{C_RESET} / {stats['total_intervals']} frames")
        print("========================================================\n")
        return

    # Detailed verbose mode
    w = 70
    print("\n" + "=" * w)
    print(f"{C_BLUE}{C_BOLD}                   LOG RECEPTION TIMING & JITTER ANALYZER{C_RESET}")
    print(f" File: {os.path.basename(file_path)}")
    print(f" Path: {file_path}")
    print("=" * w)
    
    # Summary Table
    print(f"{C_BOLD}Summary Metrics (Valid Cycle Filter: < {stats['max_valid_interval_ms']:.1f} ms):{C_RESET}")
    print("-" * w)
    print(f" │ Total Samples/Frames : {stats['total_samples']:<15} | Elapsed Time : {stats['duration_sec']:.2f} s")
    print(f" │ Active Cycle Mean    : {stats['mean_delta_ms']:.2f} ms     | Median Cycle  : {stats['median_delta_ms']:.2f} ms")
    print(f" │ Min/Max Active Cycle : {stats['min_delta_ms']:.2f} / {stats['max_active_delta_ms']:.2f} ms")
    print(f" │ Absolute Max Gap     : {stats['max_delta_ms']:.2f} ms")
    
    # Jitter warning color
    jit_color = C_GREEN
    if stats['rel_jitter_pct'] > 20.0:
        jit_color = C_RED
    elif stats['rel_jitter_pct'] > 10.0:
        jit_color = C_YELLOW
        
    print(f" │ Jitter (Active Std)  : {jit_color}{stats['jitter_ms']:.2f} ms{C_RESET} ({jit_color}{stats['rel_jitter_pct']:.1f}%{C_RESET} of target)")
    print(f" │ Average Frequency    : {C_GREEN}{stats['frequency_hz']:.2f} Hz{C_RESET}")
    print("-" * w)

    # Anomaly/Packet Drop Metrics
    anomaly_color = C_GREEN
    if stats['num_anomalies'] > 0:
        anomaly_color = C_RED if stats['pct_anomalies'] > 1.0 else C_YELLOW

    print(f"{C_BOLD}Anomaly & Fast Cycle Detection:{C_RESET}")
    print("-" * w)
    print(f" │ Chu kỳ nhanh (< {stats['min_threshold_ms']:.1f} ms)  : {C_YELLOW if stats['num_fast_cycles'] > 0 else C_RESET}{stats['num_fast_cycles']}{C_RESET} ({stats['pct_fast_cycles']:.2f}% of total)")
    print(f" │ Anomalies/Drops (trễ)      : {anomaly_color}{stats['num_anomalies']}{C_RESET} ({anomaly_color}{stats['pct_anomalies']:.2f}% of total)")
    print("-" * w)

    # Top anomalies
    if anomalies:
        print(f"\n{C_BOLD}Top 10 Worst Timing Gaps / Anomalies:{C_RESET}")
        print(f"┌───────┬───────────────────┬───────────────────┬────────────────────┐")
        print(f"│ Index │ Event Time (sec)  │ Actual Gap (ms)   │ Deviation (x Nom)  │")
        print(f"├───────┼───────────────────┼───────────────────┼────────────────────┤")
        for i, anomaly in enumerate(anomalies[:10]):
            dev_mult = anomaly['delta_ms'] / stats['target_cycle_ms']
            print(f"│ {anomaly['index']:<5} │ {anomaly['elapsed_sec']:>16.2f}s │ {anomaly['delta_ms']:>16.2f}ms │ {dev_mult:>17.1f}x  │")
        print(f"└───────┴───────────────────┴───────────────────┴────────────────────┘")
    else:
        print(f"\n{C_GREEN}✔ Perfect Timing! No timing anomalies detected.{C_RESET}")
    
    print("=" * w + "\n")


def plot_dashboard(timestamps, deltas, stats, anomalies, output_img=None):
    """
    Plots a premium dark-themed dashboard of the results.
    """
    if not HAS_MATPLOTLIB:
        print("Warning: matplotlib is not installed. Skipping graphical plot.")
        return

    # Set styles for dark theme
    plt.style.use('dark_background')
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(15, 6.5))
    fig.patch.set_facecolor('#1a1a2e')
    
    fig.canvas.manager.set_window_title('Log Timing & Jitter Dashboard')
    fig.suptitle(f"Log timing analysis: {stats['total_samples']} frames over {stats['duration_sec']:.2f}s", 
                 color='white', fontsize=14, fontweight='bold', y=0.96)

    # Subplot 1: Timeline of delta times
    ax1.set_facecolor('#16213e')
    
    # Calculate elapsed times
    t0 = timestamps[0]
    elapsed_times = [(t - t0) / 1000.0 for t in timestamps[:-1]]
    
    # Plot cycle line
    ax1.plot(elapsed_times, deltas, color='#0f3460', alpha=0.8, label='Cycle Time')
    ax1.scatter(elapsed_times, deltas, color='#00ff88', s=10, alpha=0.6, edgecolors='none', label='Frames')

    # Draw nominal target cycle
    ax1.axhline(stats['target_cycle_ms'], color='#a0a0a0', linestyle='--', linewidth=1.2, 
                label=f"Nominal: {stats['target_cycle_ms']:.1f}ms")
    # Draw anomaly threshold
    ax1.axhline(stats['anomaly_threshold_ms'], color='#e94560', linestyle=':', linewidth=1.5,
                label=f"Anomaly limit: {stats['anomaly_threshold_ms']:.1f}ms")

    # Highlight anomalies on timeline
    if anomalies:
        anomaly_times = [a['elapsed_sec'] for a in anomalies]
        anomaly_deltas = [a['delta_ms'] for a in anomalies]
        ax1.scatter(anomaly_times, anomaly_deltas, color='#e94560', marker='x', s=45, zorder=10, 
                    linewidths=1.5, label=f"Anomalies ({len(anomalies)})")

    ax1.set_title('Delta Time between Receptions over Timeline', color='white', fontsize=11, fontweight='bold', pad=10)
    ax1.set_xlabel('Log Elapsed Time (seconds)', color='#a0a0a0', fontsize=9)
    ax1.set_ylabel('Interval / Delta Time (ms)', color='#a0a0a0', fontsize=9)
    ax1.tick_params(colors='#707070', labelsize=8)
    ax1.grid(True, alpha=0.15, color='#444444')
    ax1.legend(loc='upper right', framealpha=0.3, fontsize=8)

    # Subplot 2: Histogram Distribution
    ax2.set_facecolor('#16213e')
    
    # Create bin sizing
    # Use simple binning since numpy percentile might not be available or simple
    try:
        q25, q75 = np.percentile(deltas, [25, 75])
        iqr = q75 - q25
        bin_width = 2 * iqr * (len(deltas) ** (-1/3)) if iqr > 0 else 1.0
        num_bins = int((max(deltas) - min(deltas)) / bin_width) if bin_width > 0 else 10
        num_bins = min(max(num_bins, 15), 100)
    except Exception:
        num_bins = 20

    # Plot histogram
    n, bins, patches = ax2.hist(deltas, bins=num_bins, color='#0f3460', edgecolor='#16213e', 
                               alpha=0.75, density=True, label='Frequency Density')
    
    # Color code bars: highlight bins that are outliers
    for patch, left_edge in zip(patches, bins[:-1]):
        if left_edge > stats['anomaly_threshold_ms']:
            patch.set_facecolor('#e94560')
            patch.set_alpha(0.8)

    # Vertical lines for mean and median
    ax2.axvline(stats['mean_delta_ms'], color='#00ff88', linestyle='-', linewidth=1.5, 
                label=f"Mean: {stats['mean_delta_ms']:.1f}ms")
    ax2.axvline(stats['median_delta_ms'], color='#e94560', linestyle='-.', linewidth=1.2, 
                label=f"Median: {stats['median_delta_ms']:.1f}ms")

    ax2.set_title('Interval Distribution & Frequency Density', color='white', fontsize=11, fontweight='bold', pad=10)
    ax2.set_xlabel('Interval / Delta Time (ms)', color='#a0a0a0', fontsize=9)
    ax2.set_ylabel('Density', color='#a0a0a0', fontsize=9)
    ax2.tick_params(colors='#707070', labelsize=8)
    ax2.grid(True, alpha=0.15, color='#444444')
    
    # Display text summary inside the plot
    text_str = (
        f"Filter: < {stats['max_valid_interval_ms']:.1f} ms\n"
        f"Samples: {stats['total_samples']} (Active: {stats['valid_intervals']})\n"
        f"Mean: {stats['mean_delta_ms']:.2f} ms\n"
        f"Median: {stats['median_delta_ms']:.2f} ms\n"
        f"Jitter (Std): {stats['jitter_ms']:.2f} ms ({stats['rel_jitter_pct']:.1f}%)\n"
        f"Frequency: {stats['frequency_hz']:.2f} Hz\n"
        f"Max Active: {stats['max_active_delta_ms']:.2f} ms\n"
        f"Max Gap: {stats['max_delta_ms']:.2f} ms\n"
        f"Anomalies: {stats['num_anomalies']} ({stats['pct_anomalies']:.2f}%)"
    )
    ax2.text(0.95, 0.05, text_str, transform=ax2.transAxes, fontsize=8, color='#a0a0a0',
             verticalalignment='bottom', horizontalalignment='right',
             bbox=dict(boxstyle='round', facecolor='#1a1a2e', alpha=0.6, edgecolor='#444444'))

    ax2.legend(loc='upper right', framealpha=0.3, fontsize=8)

    plt.subplots_adjust(bottom=0.15, left=0.08, right=0.92, top=0.88, wspace=0.20)
    
    if output_img:
        plt.savefig(output_img, dpi=150, facecolor=fig.get_facecolor(), edgecolor='none')
        print(f"📊 Saved chart to: {output_img}")

    plt.show()


def main():
    parser = argparse.ArgumentParser(description="Analyze timing interval & jitter in radar/socketcan logs.")
    parser.add_argument("file", help="Path to log file (CSV, text or candump)")
    parser.add_argument("-t", "--target-cycle", type=float, default=None, 
                        help="Expected cycle time in milliseconds (if omitted, estimated via median)")
    parser.add_argument("-m", "--multiplier", type=float, default=1.5,
                        help="Multiplier threshold to detect cycle drops/anomalies (default: 1.5x)")
    parser.add_argument("--max-valid-interval", type=float, default=50.0,
                        help="Maximum valid interval in ms to include in active cycle & frequency calculations (default: 50.0 ms)")
    parser.add_argument("--min-threshold", type=float, default=30.0,
                        help="Minimum cycle threshold in ms to detect fast cycle anomalies (default: 30.0 ms)")
    parser.add_argument("--no-group", action="store_true",
                        help="Do not group identical timestamps for CSV logs (default: group to find frame cycles)")
    parser.add_argument("--no-plot", action="store_true",
                        help="Run in headless mode without plotting the matplotlib UI")
    parser.add_argument("-s", "--save-img", type=str, default=None,
                        help="Save generated plot to specified image path (e.g. output.png)")
    parser.add_argument("-v", "--verbose", action="store_true",
                        help="Print detailed anomaly report and full file path information")
    args = parser.parse_args()

    if not os.path.exists(args.file):
        print(f"Error: Log file '{args.file}' not found.")
        sys.exit(1)

    # Load timestamps
    is_csv = args.file.lower().endswith('.csv')
    if is_csv:
        if HAS_PANDAS:
            timestamps = load_csv_timestamps_pandas(args.file, group_by_ts=not args.no_group)
        else:
            timestamps = load_csv_timestamps_fallback(args.file, group_by_ts=not args.no_group)
    else:
        timestamps = load_text_timestamps(args.file)

    # Run analysis
    deltas, stats, anomalies = analyze_intervals(
        timestamps, 
        target_cycle=args.target_cycle, 
        threshold_multiplier=args.multiplier,
        max_valid_interval=args.max_valid_interval,
        min_threshold=args.min_threshold
    )

    # Print dashboard
    print_dashboard(stats, anomalies, args.file, verbose=args.verbose)

    # Plot dashboard
    if not args.no_plot:
        plot_dashboard(timestamps, deltas, stats, anomalies, output_img=args.save_img)
    elif args.save_img:
        # If no-plot is set but save-img is specified, plot in headless mode
        if HAS_MATPLOTLIB:
            import matplotlib
            matplotlib.use('Agg')
            plot_dashboard(timestamps, deltas, stats, anomalies, output_img=args.save_img)


if __name__ == "__main__":
    main()
