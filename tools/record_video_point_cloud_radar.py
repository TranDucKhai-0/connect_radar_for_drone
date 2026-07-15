#!/usr/bin/env python3
"""
Radar Log Video Recorder — Point cloud from radar_log.csv
Automatically generates an MP4 video (without UI sliders) using actual timestamps.
"""

import sys
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Wedge
from matplotlib.collections import PatchCollection
import matplotlib.animation as animation
from tqdm import tqdm

import argparse

# ---------------------------------------------------------
# 1. READ DATA
# ---------------------------------------------------------
parser = argparse.ArgumentParser(description="Radar Log Video Recorder")
parser.add_argument("csv_file", nargs='?', default="radar_log.csv", help="Path to radar_log.csv")
parser.add_argument("output", nargs='?', default="output_radar_video.mp4", help="Output video path")
parser.add_argument("--fps", type=int, default=60, help="Frames per second of output video (default: 60)")
parser.add_argument("--speed", type=float, default=1.0, help="Playback speed multiplier (e.g. 2.0 for 2x speed, 0.5 for half speed)")
args = parser.parse_args()

file_path = args.csv_file
output_video_path = args.output
FPS = args.fps
SPEED = args.speed

try:
    df = pd.read_csv(file_path)
except FileNotFoundError:
    print(f"Error: File not found: '{file_path}'.")
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
# 2. USE REAL TIMESTAMPS
# ---------------------------------------------------------
# The user wants to see the real timestamps without the 100ms forced gap.
raw_ts = df['TimestampUsec'].values.copy().astype(np.float64)

# We use the raw timestamps directly as the true timeline
df['SynthTimestamp'] = raw_ts

t0 = raw_ts[0]
t_end = raw_ts[-1]

PADDING_USEC = 1_000_000  # 1 second padding in microseconds at start and end
POINT_LIFETIME_USEC = 200_000  # 0.2 seconds point lifetime

timeline_start = t0 - PADDING_USEC
timeline_end = t_end + PADDING_USEC
timeline_duration_sec = (timeline_end - timeline_start) / 1_000_000.0

SLIDER_STEP_USEC = int((1_000_000 / FPS) * SPEED) 
n_steps = int((timeline_end - timeline_start) / SLIDER_STEP_USEC)
if n_steps < 1:
    n_steps = 1

n_points = len(df)
print(f"Successfully read {n_points} points.")
print(f"Real timeline duration: {timeline_duration_sec:.2f}s "
      f"(including 1s padding at start & end)")
print(f"Point lifetime: 0.2s | Generating video with {n_steps} frames at {FPS} FPS (Speed: {SPEED}x)...")

# ---------------------------------------------------------
# 3. COMPUTE 72-SECTOR
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
fig.suptitle('Radar Point Cloud Video Export', color='white',
             fontsize=14, fontweight='bold')

ax1 = fig.add_subplot(121, projection='3d', facecolor='#16213e')
ax2 = fig.add_subplot(122, projection='polar', facecolor='#16213e')

plt.subplots_adjust(bottom=0.10, left=0.05, right=0.95, top=0.90, wspace=0.25)

time_text = fig.text(0.5, 0.05, '', ha='center', va='center',
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

synth_arr = df['SynthTimestamp'].values

pbar = tqdm(total=n_steps, desc="Rendering Video")

# ---------------------------------------------------------
# 6. UPDATE FUNCTION FOR ANIMATION
# ---------------------------------------------------------
def update(frame_idx):
    current_time = timeline_start + frame_idx * SLIDER_STEP_USEC

    mask = (synth_arr <= current_time) & (synth_arr > current_time - POINT_LIFETIME_USEC)
    visible_df = df[mask].copy()

    if not visible_df.empty:
        ages = current_time - visible_df['SynthTimestamp'].values
        visible_df['alpha'] = 1.0 - 0.85 * (ages / POINT_LIFETIME_USEC)
        visible_df['alpha'] = visible_df['alpha'].clip(0.15, 1.0)

    past_mask = synth_arr <= current_time
    if past_mask.any():
        last_idx = np.where(past_mask)[0][-1]
        current_alt = df.iloc[last_idx]['DroneAlt']
    else:
        current_alt = df.iloc[0]['DroneAlt']

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

    elapsed = (current_time - timeline_start) / 1_000_000.0
    n_visible = len(visible_df)
    n_active_sectors = sum(1 for s in sectors if s < 65535)
    time_text.set_text(
        f't = {elapsed:.2f}s  |  {n_visible} visible point(s)  |  '
        f'{n_active_sectors}/72 sectors active'
    )

    pbar.update(1)

# ---------------------------------------------------------
# 7. CREATE ANIMATION AND SAVE
# ---------------------------------------------------------
print("Starting animation rendering...")
anim = animation.FuncAnimation(fig, update, frames=n_steps, blit=False)

try:
    writer = animation.FFMpegWriter(fps=FPS, metadata=dict(artist='RadarLogger'), bitrate=2000)
    anim.save(output_video_path, writer=writer)
    pbar.close()
    print(f"\\nVideo successfully saved to {output_video_path}")
except Exception as e:
    pbar.close()
    print(f"\\nFailed to save video using FFMpegWriter: {e}")
    print("Ensure ffmpeg is installed (sudo apt install ffmpeg). Trying alternative writer (Pillow)...")
    try:
        writer = animation.PillowWriter(fps=FPS)
        alt_path = output_video_path.replace('.mp4', '.gif')
        anim.save(alt_path, writer=writer)
        print(f"Saved as GIF to {alt_path}")
    except Exception as e2:
        print(f"Alternative writer also failed: {e2}")

