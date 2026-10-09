#!/usr/bin/env python3
"""
Radar Log Visualizer — Point cloud from radar_log.csv
Two subplots:
  - Left:  Cartesian 3D (X, Y, DroneAlt) — top-down FRD view
  - Right: 72-Sector Polar OBSTACLE_DISTANCE (simulates FC message)
Time-based slider with automatic timestamp interpolation for short/corrupted logs.
"""

import sys
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
from matplotlib.patches import Wedge
from matplotlib.collections import PatchCollection

# ---------------------------------------------------------
# 1. READ DATA
# ---------------------------------------------------------
file_path = sys.argv[1] if len(sys.argv) > 1 else 'radar_log.csv'

try:
    df = pd.read_csv(file_path)
except FileNotFoundError:
    print(f"Error: File not found: '{file_path}'.")
    print("Usage: python3 view_chart_point_cloud_radar.py <csv_file_path>")
    sys.exit(1)

required = ['TimestampUsec', 'X', 'Y', 'Z', 'Range', 'Angle', 'DroneAlt']
for col in required:
    if col not in df.columns:
        print(f"Error: Missing column '{col}' in CSV file. Existing columns: {list(df.columns)}")
        sys.exit(1)

df = df.dropna(subset=required)

if len(df) == 0:
    print("Error: CSV file has no valid data.")
    sys.exit(1)

# Sort by timestamp
df = df.sort_values('TimestampUsec').reset_index(drop=True)

# ---------------------------------------------------------
# 2. SYNTHESIZE REALISTIC TIMESTAMPS
# ---------------------------------------------------------
# When timestamps are identical (e.g. Excel truncated precision),
# auto-space points 100ms (100000 usec) apart so the visualization
# replays at a realistic pace.

CYCLE_USEC = 100_000  # 100ms = radar cycle time

raw_ts = df['TimestampUsec'].values.copy().astype(np.float64)
synth_ts = np.empty_like(raw_ts)
synth_ts[0] = raw_ts[0]

for i in range(1, len(raw_ts)):
    delta = raw_ts[i] - raw_ts[i - 1]
    if delta < CYCLE_USEC:
        # Timestamps are too close or identical — insert 100ms gap
        synth_ts[i] = synth_ts[i - 1] + CYCLE_USEC
    else:
        # Real gap preserved
        synth_ts[i] = synth_ts[i - 1] + delta

df['SynthTimestamp'] = synth_ts

t0 = synth_ts[0]
t_end = synth_ts[-1]

PADDING_USEC = 2_000_000  # 2 seconds padding in microseconds
POINT_LIFETIME_USEC = 200_000  # 0.2 seconds point lifetime

timeline_start = t0 - PADDING_USEC
timeline_end = t_end + PADDING_USEC
timeline_duration_sec = (timeline_end - timeline_start) / 1_000_000.0

# Slider resolution: 10ms steps for smooth scrubbing
SLIDER_STEP_USEC = 10_000  # 10ms
n_steps = int((timeline_end - timeline_start) / SLIDER_STEP_USEC)
if n_steps < 1:
    n_steps = 1

n_points = len(df)
print(f"Successfully read {n_points} points.")
print(f"Synthetic timeline: {timeline_duration_sec:.2f}s "
      f"(including 2s padding at start & end)")
print(f"Point lifetime: 0.2s | Slider steps: {n_steps}")

# ---------------------------------------------------------
# 3. COMPUTE 72-SECTOR (identical to SendDataToFcThread in C++)
# ---------------------------------------------------------
def compute_72_sectors(frame_df):
    distances = np.full(72, 65535, dtype=np.uint16)

    for _, obs in frame_df.iterrows():
        dist_cm = obs['Range'] * 100.0
        angle_deg = obs['Angle'] * (180.0 / np.pi)

        while angle_deg < 0:
            angle_deg += 360.0
        while angle_deg >= 360.0:
            angle_deg -= 360.0

        idx = int(angle_deg / 5.0 + 0.5) % 72

        if distances[idx] == 65535 or dist_cm < distances[idx]:
            distances[idx] = int(dist_cm)

    return distances

# ---------------------------------------------------------
# 4. SETUP FIGURE
# ---------------------------------------------------------
fig = plt.figure(figsize=(16, 8))
fig.patch.set_facecolor('#1a1a2e')
fig.suptitle('Radar Point Cloud Visualizer', color='white',
             fontsize=14, fontweight='bold')

ax1 = fig.add_subplot(121, projection='3d', facecolor='#16213e')
ax2 = fig.add_subplot(122, projection='polar', facecolor='#16213e')

plt.subplots_adjust(bottom=0.18, left=0.05, right=0.95, top=0.90, wspace=0.25)

# Slider (time-based)
ax_slider = plt.axes([0.15, 0.04, 0.7, 0.03], facecolor='#0f3460')
slider = Slider(ax_slider, 'Time', 0, n_steps,
                valinit=0, valstep=1, color='#e94560')

time_text = fig.text(0.5, 0.09, '', ha='center', va='center',
                     color='#e94560', fontsize=11, fontweight='bold')

# ---------------------------------------------------------
# 5. STYLE
# ---------------------------------------------------------
def style_3d_axis(ax, title, xlabel, ylabel, zlabel):
    ax.set_title(title, color='white', fontsize=11, pad=10)
    ax.set_xlabel(xlabel, color='#a0a0a0', fontsize=9)
    ax.set_ylabel(ylabel, color='#a0a0a0', fontsize=9)
    ax.set_zlabel(zlabel, color='#a0a0a0', fontsize=9)
    ax.tick_params(colors='#707070', labelsize=7)
    ax.xaxis.pane.fill = False
    ax.yaxis.pane.fill = False
    ax.zaxis.pane.fill = False
    ax.xaxis.pane.set_edgecolor('#333333')
    ax.yaxis.pane.set_edgecolor('#333333')
    ax.zaxis.pane.set_edgecolor('#333333')
    ax.grid(True, alpha=0.2)

style_3d_axis(ax1, 'Cartesian (X, Y, Alt)',
              'X Forward [m]', 'Y Right [m]', 'Alt [m]')
ax1.view_init(elev=90, azim=-90)
ax1.invert_zaxis()

ax2.set_title('FC Obstacle Distance Message',
              color='white', fontsize=10, pad=15)
ax2.set_facecolor('#16213e')
ax2.tick_params(colors='#707070', labelsize=7)
ax2.set_theta_zero_location('N')
ax2.set_theta_direction(-1)
ax2.grid(True, alpha=0.15, color='#444444')
ax2.set_rmax(40)
ax2.set_rlabel_position(45)

# Fixed axis limits
x_min, x_max = df['X'].min() - 1, df['X'].max() + 1
y_min, y_max = df['Y'].min() - 1, df['Y'].max() + 1
alt_min = df['DroneAlt'].min() - 0.5
alt_max = df['DroneAlt'].max() + 0.5

# Plot objects
scatter1 = [None]
drone_marker1 = [None]
sector_bars = [None]
point_scatter2 = [None]

# Precompute synthetic timestamps as numpy array for fast filtering
synth_arr = df['SynthTimestamp'].values

# ---------------------------------------------------------
# 6. UPDATE FUNCTION
# ---------------------------------------------------------
def update(val):
    step = int(slider.val)
    current_time = timeline_start + step * SLIDER_STEP_USEC

    # Find all points alive at current_time (within 0.2s lifetime window)
    # A point is visible if: current_time >= point_time AND current_time - point_time < LIFETIME
    mask = (synth_arr <= current_time) & (synth_arr > current_time - POINT_LIFETIME_USEC)
    visible_df = df[mask].copy()

    # Compute alpha based on age (newer = brighter)
    if not visible_df.empty:
        ages = current_time - visible_df['SynthTimestamp'].values
        # age=0 -> alpha=1.0 (newest), age=LIFETIME -> alpha=0.15 (oldest)
        visible_df['alpha'] = 1.0 - 0.85 * (ages / POINT_LIFETIME_USEC)
        visible_df['alpha'] = visible_df['alpha'].clip(0.15, 1.0)

    # Get latest known DroneAlt for drone marker
    past_mask = synth_arr <= current_time
    if past_mask.any():
        last_idx = np.where(past_mask)[0][-1]
        current_alt = df.iloc[last_idx]['DroneAlt']
    else:
        current_alt = df.iloc[0]['DroneAlt']

    # =============================================
    # PLOT 1: Cartesian 3D
    # =============================================
    if scatter1[0] is not None:
        scatter1[0].remove()
    if drone_marker1[0] is not None:
        drone_marker1[0].remove()

    if not visible_df.empty:
        scatter1[0] = ax1.scatter(
            visible_df['X'], visible_df['Y'], visible_df['DroneAlt'],
            c=visible_df['alpha'], cmap='plasma',
            marker='o', s=25, alpha=0.8, edgecolors='none',
            vmin=0, vmax=1
        )
    else:
        scatter1[0] = None

    drone_marker1[0] = ax1.scatter(
        [0], [0], [current_alt],
        c='#00ff88', marker='^', s=150,
        edgecolors='white', linewidths=1.0, zorder=10
    )
    ax1.set_xlim(x_min, x_max)
    ax1.set_ylim(y_min, y_max)
    ax1.set_zlim(alt_min, alt_max)

    # =============================================
    # PLOT 2: 72-Sector Polar
    # =============================================
    if sector_bars[0] is not None:
        sector_bars[0].remove()
    if point_scatter2[0] is not None:
        point_scatter2[0].remove()

    sectors = compute_72_sectors(visible_df) if not visible_df.empty else np.full(72, 65535, dtype=np.uint16)

    theta_centers = []
    radii = []
    colors = []

    for i in range(72):
        if sectors[i] < 65535:
            angle_rad = np.deg2rad(i * 5.0)
            dist_m = sectors[i] / 100.0
            theta_centers.append(angle_rad)
            radii.append(dist_m)
            normalized = min(dist_m / 40.0, 1.0)
            colors.append(plt.cm.RdYlGn(normalized))

    width = np.deg2rad(5.0)

    if theta_centers:
        sector_bars[0] = ax2.bar(
            theta_centers, radii, width=width,
            color=colors, alpha=0.75, edgecolor='#ffffff44', linewidth=0.5,
            bottom=0
        )
    else:
        sector_bars[0] = None

    if not visible_df.empty:
        raw_angles = visible_df['Angle'].values
        raw_angles_norm = raw_angles.copy()
        raw_angles_norm = np.where(raw_angles_norm < 0, raw_angles_norm + 2*np.pi, raw_angles_norm)
        raw_ranges = visible_df['Range'].values

        point_scatter2[0] = ax2.scatter(
            raw_angles_norm, raw_ranges,
            c='#ff4444', marker='x', s=40, zorder=10, linewidths=1.5
        )
    else:
        point_scatter2[0] = None

    ax2.set_rmax(40)

    # Status text
    elapsed = (current_time - timeline_start) / 1_000_000.0
    n_visible = len(visible_df)
    n_active_sectors = sum(1 for s in sectors if s < 65535)
    time_text.set_text(
        f't = {elapsed:.2f}s  |  {n_visible} visible point(s)  |  '
        f'{n_active_sectors}/72 sectors active'
    )

    fig.canvas.draw_idle()

# ---------------------------------------------------------
# 7. KEYBOARD SHORTCUTS
# ---------------------------------------------------------
def on_key(event):
    step = int(slider.val)
    if event.key == 'right' and step < n_steps:
        slider.set_val(step + 1)
    elif event.key == 'left' and step > 0:
        slider.set_val(step - 1)
    elif event.key == 'home':
        slider.set_val(0)
    elif event.key == 'end':
        slider.set_val(n_steps)

fig.canvas.mpl_connect('key_press_event', on_key)
slider.on_changed(update)

# Draw first frame
update(0)

fig.text(0.5, 0.005,
         '← → : Prev/Next step  |  Home/End : First/Last  |  Drag slider to scrub  |'
         '  × = Raw point  |  Bar = Sector sent to FC  |  Point lifetime: 0.2s',
         ha='center', color='#555555', fontsize=8)

plt.show()
