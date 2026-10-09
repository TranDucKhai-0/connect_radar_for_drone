#!/usr/bin/env python3
"""
CubeFC Radar Log Visualizer
Vẽ data từ file log cubefc_radar_log.csv:
  - Đồ thị 1 (Interactive): 
    * Trái: Cartesian 2D (Hệ trục FRD: X Forward làm trục tung hướng lên, Y Right làm trục hoành sang phải)
    * Phải: 72-Sector Polar Plot (Mô phỏng bản tin gửi FC, 0 độ hướng Bắc/Tiến, chiều kim đồng hồ)
    * Điều khiển: Slider, Play/Pause, các phím điều hướng.
  - Đồ thị 2 (Heatmap Timeline):
    * Thể hiện khoảng cách vật cản theo thời gian cho tất cả 72 sectors (trực quan hóa toàn bộ chuyến bay).
"""

import sys
import os
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, Button
from matplotlib.colors import ListedColormap

# ---------------------------------------------------------
# 1. TÌM VÀ ĐỌC DỮ LIỆU LOG
# ---------------------------------------------------------
def find_log_file():
    # Thử lấy file path từ dòng lệnh
    if len(sys.argv) > 1:
        return sys.argv[1]
    
    # Các đường dẫn mặc định để tìm kiếm
    search_paths = [
        'cubefc_radar_log.csv',
        'tools/cubefc_radar_log.csv',
        '../blackbox/cubefc_radar_log.csv',
        'blackbox/cubefc_radar_log.csv',
        '/usr/local/etc/connect_radar_for_drone/blackbox/cubefc_radar_log.csv'
    ]
    
    for path in search_paths:
        if os.path.exists(path):
            return path
            
    return None

file_path = find_log_file()
if not file_path:
    print("❌ Error: cubefc_radar_log.csv log file not found!")
    print("Usage:")
    print("  python3 tools/plot_cubefc_radar_log.py <csv_file_path>")
    sys.exit(1)

print(f"📂 Reading data from: {file_path}")

try:
    df = pd.read_csv(file_path)
except Exception as e:
    print(f"❌ Error reading CSV file: {e}")
    sys.exit(1)

# Chuẩn hoá tên cột (không phân biệt hoa thường)
col_mapping = {}
for col in df.columns:
    c_low = col.lower()
    if 'timestamp' in c_low:
        col_mapping[col] = 'TimestampUsec'
    elif 'sector' in c_low:
        col_mapping[col] = 'SectorIdx'
    elif 'distance' in c_low:
        col_mapping[col] = 'DistanceCm'

df = df.rename(columns=col_mapping)

required_cols = {'TimestampUsec', 'SectorIdx', 'DistanceCm'}
if not required_cols.issubset(df.columns):
    print(f"❌ Error: Missing columns in CSV file. Required: {required_cols}")
    print(f"Existing columns: {list(df.columns)}")
    sys.exit(1)

# Drop rows with missing data
df = df.dropna(subset=list(required_cols))

if df.empty:
    print("⚠️ Warning: CSV file contains no valid data.")
    sys.exit(1)

# Sắp xếp theo TimestampUsec
df = df.sort_values('TimestampUsec')

# Gom nhóm theo TimestampUsec để phân chia thành các frames
t0 = df['TimestampUsec'].min()
grouped = df.groupby('TimestampUsec')

frames = []
for ts, group in sorted(grouped):
    t_sec = (ts - t0) / 1000000.0
    sectors = group['SectorIdx'].values.astype(int)
    distances = group['DistanceCm'].values.astype(float)
    frames.append({
        'timestamp_usec': ts,
        't_sec': t_sec,
        'sectors': sectors,
        'distances': distances
    })

n_frames = len(frames)
duration_sec = frames[-1]['t_sec']
print(f"✅ Successfully read {len(df)} data rows, split into {n_frames} frames.")
print(f"⏱️ Total log duration: {duration_sec:.2f} seconds.")

# ---------------------------------------------------------
# 2. KHỞI TẠO ĐỒ THỊ 1: REAL-TIME INTERACTIVE VISUALIZER
# ---------------------------------------------------------
fig = plt.figure(figsize=(15, 8.5))
fig.patch.set_facecolor('#1a1a2e')
fig.canvas.manager.set_window_title('CubeFC Radar Log Playback')

# Subplot trái: Cartesian 2D (FRD: X Forward làm trục tung hướng lên, Y Right làm trục hoành sang phải)
ax_cartesian = fig.add_subplot(121, facecolor='#16213e')
# Subplot phải: Polar Plot 72-Sector
ax_polar = fig.add_subplot(122, projection='polar', facecolor='#16213e')

# Chừa khoảng trống bên dưới để đặt widgets
plt.subplots_adjust(bottom=0.22, left=0.08, right=0.92, top=0.90, wspace=0.25)

# Cấu hình phong cách trục Cartesian 2D
ax_cartesian.set_title('2D Obstacle Map (FRD Frame)', color='white', fontsize=12, pad=10)
ax_cartesian.set_xlabel('Y - Right [m]', color='#a0a0a0', fontsize=10)
ax_cartesian.set_ylabel('X - Forward [m]', color='#a0a0a0', fontsize=10)
ax_cartesian.tick_params(colors='#707070', labelsize=8)
ax_cartesian.grid(True, alpha=0.15, color='#444444')

# Vẽ vòng tròn khoảng cách làm la bàn quét radar
for r in [10, 20, 30, 40]:
    circle = plt.Circle((0, 0), r, color='#00ff88', fill=False, linestyle='--', alpha=0.15)
    ax_cartesian.add_artist(circle)
    ax_cartesian.text(0, r + 0.5, f"{r}m", color='#00ff88', fontsize=8, alpha=0.4, ha='center')

# Giới hạn trục Cartesian cố định từ -45m đến +45m để tránh rung lắc khung hình
ax_cartesian.set_xlim(-45, 45)
ax_cartesian.set_ylim(-5, 45)
ax_cartesian.set_aspect('equal')

# Cấu hình phong cách Polar Plot
ax_polar.set_title('72-Sector Distribution Sent to FC', color='white', fontsize=12, pad=15)
ax_polar.tick_params(colors='#707070', labelsize=8)
ax_polar.set_theta_zero_location('N')    # 0° Forward (North)
ax_polar.set_theta_direction(-1)         # Clockwise (CW) - FRD standard
ax_polar.grid(True, alpha=0.2, color='#444444')
ax_polar.set_rmax(40)                    # Max range 40m
ax_polar.set_rlabel_position(45)

# ---------------------------------------------------------
# CẤU HÌNH WIDGETS ĐIỀU KHIỂN
# ---------------------------------------------------------
# Timeline Slider
ax_slider = plt.axes([0.15, 0.10, 0.70, 0.03], facecolor='#0f3460')
slider = Slider(ax_slider, 'Time (s)', 0, n_frames - 1, valinit=0, valstep=1, color='#e94560')
slider.valtext.set_color('white')

# Play/Pause Button
ax_play = plt.axes([0.15, 0.04, 0.10, 0.04], facecolor='#0f3460')
btn_play = Button(ax_play, '▶ Play', color='#e94560', hovercolor='#ff5d7d')
btn_play.label.set_color('white')

# Back Button
ax_back = plt.axes([0.27, 0.04, 0.08, 0.04], facecolor='#0f3460')
btn_back = Button(ax_back, '⏮ Back', color='#0f3460', hovercolor='#1b5694')
btn_back.label.set_color('white')

# Forward Button
ax_fwd = plt.axes([0.37, 0.04, 0.08, 0.04], facecolor='#0f3460')
btn_fwd = Button(ax_fwd, 'Fwd ⏭', color='#0f3460', hovercolor='#1b5694')
btn_fwd.label.set_color('white')

# Text hiển thị thông tin frame
info_text = fig.text(0.70, 0.05, '', ha='left', va='center', color='#e94560', fontsize=10, fontweight='bold')

# Các đối tượng vẽ được lưu trữ để cập nhật nhanh (tránh vẽ lại toàn bộ subplot)
cartesian_scatter = [None]
drone_marker = [None]
polar_bars = [None]

# ---------------------------------------------------------
# 3. HÀM CẬP NHẬT FRAME (HÀM VẼ CHÍNH)
# ---------------------------------------------------------
def update_plot(val):
    idx = int(slider.val)
    if idx < 0 or idx >= n_frames:
        return
        
    frame = frames[idx]
    t_sec = frame['t_sec']
    sectors = frame['sectors']
    distances_m = frame['distances'] / 100.0  # Cm -> m
    
    # 3.1. Tính toán tọa độ Cartesian từ SectorIdx và Distance
    # Mỗi sector rộng 5 độ, tâm của sector là SectorIdx * 5
    angles_deg = sectors * 5.0
    angles_rad = np.deg2rad(angles_deg)
    
    # Chuyển đổi sang hệ trục FRD: 
    # X (Forward) = R * cos(Angle), Y (Right) = R * sin(Angle)
    # Lưu ý: Trên đồ thị 2D, Y được vẽ ngang (hoành), X vẽ dọc (tung).
    X = distances_m * np.cos(angles_rad)
    Y = distances_m * np.sin(angles_rad)
    
    # --- Cập nhật đồ thị Cartesian ---
    if cartesian_scatter[0] is not None:
        cartesian_scatter[0].remove()
    if drone_marker[0] is not None:
        drone_marker[0].remove()
        
    # Vẽ các điểm vật cản (màu sắc theo khoảng cách từ đỏ đến vàng, xanh lá)
    # Colormap RdYlGn_r: nhỏ (gần) là đỏ, lớn (xa) là xanh lá
    colors_cartesian = plt.cm.RdYlGn_r(np.clip(distances_m / 40.0, 0, 1))
    
    cartesian_scatter[0] = ax_cartesian.scatter(
        Y, X, c=colors_cartesian, marker='o', s=35, 
        edgecolors='white', linewidths=0.5, alpha=0.9, zorder=5
    )
    
    # Vẽ biểu tượng drone tại (0,0) hướng lên trên
    drone_marker[0] = ax_cartesian.scatter(
        [0], [0], c='#00ff88', marker='^', s=120, 
        edgecolors='white', linewidths=1.0, zorder=10, label='Drone'
    )
    
    # --- Cập nhật Polar Plot 72-Sector ---
    if polar_bars[0] is not None:
        for patch in polar_bars[0]:
            patch.remove()
        
    # Tạo các thanh bar biểu diễn sector trên đồ thị polar
    if len(sectors) > 0:
        width = np.deg2rad(5.0)  # Góc rộng mỗi sector là 5 độ
        colors_polar = plt.cm.RdYlGn_r(np.clip(distances_m / 40.0, 0, 1))
        
        polar_bars[0] = ax_polar.bar(
            angles_rad, distances_m, width=width,
            color=colors_polar, alpha=0.75, edgecolor='#ffffff33', linewidth=0.5,
            bottom=0, zorder=3
        )
    else:
        polar_bars[0] = None
        
    # Update text info
    info_text.set_text(
        f"⏱️ Time: {t_sec:.2f}s / {duration_sec:.2f}s\n"
        f"🖼️ Frame: {idx + 1} / {n_frames}\n"
        f"🔴 Obstacles: {len(sectors)}"
    )
    
    fig.canvas.draw_idle()

slider.on_changed(update_plot)

# ---------------------------------------------------------
# 4. CHỨC NĂNG PLAYBACK TỰ ĐỘNG
# ---------------------------------------------------------
is_playing = False
timer = None

def play_next():
    idx = int(slider.val)
    if idx < n_frames - 1:
        slider.set_val(idx + 1)
    else:
        slider.set_val(0)  # Vòng lặp lại từ đầu khi kết thúc

def on_timer_tick(event):
    if is_playing:
        play_next()

def toggle_play(event=None):
    global is_playing, timer
    if is_playing:
        is_playing = False
        btn_play.label.set_text('▶ Play')
        if timer:
            timer.stop()
    else:
        is_playing = True
        btn_play.label.set_text('❚❚ Pause')
        # Chu kỳ 100ms tương ứng với tốc độ log gốc (10Hz)
        if timer is None:
            timer = fig.canvas.new_timer(interval=100)
            timer.add_callback(on_timer_tick)
        timer.start()
        
btn_play.on_clicked(toggle_play)

# Các hàm phụ cho nút Lùi / Tiến
def step_backward(event=None):
    idx = int(slider.val)
    if idx > 0:
        slider.set_val(idx - 1)

def step_forward(event=None):
    idx = int(slider.val)
    if idx < n_frames - 1:
        slider.set_val(idx + 1)

btn_back.on_clicked(step_backward)
btn_fwd.on_clicked(step_forward)

# ---------------------------------------------------------
# BÀN PHÍM PHÍM TẮT
# ---------------------------------------------------------
def on_key_press(event):
    if event.key == ' ':
        toggle_play()
    elif event.key == 'right':
        step_forward()
    elif event.key == 'left':
        step_backward()
    elif event.key == 'home':
        slider.set_val(0)
    elif event.key == 'end':
        slider.set_val(n_frames - 1)

fig.canvas.mpl_connect('key_press_event', on_key_press)

# Vẽ frame đầu tiên
update_plot(0)

# Usage shortcut note at the bottom
fig.text(0.5, 0.015,
         'Shortcuts: [Space] Play/Pause  |  [←] [→] Prev/Next Frame  |  [Home]/[End] First/Last  |  Drag Slider to Scrub',
         ha='center', color='#888888', fontsize=9)

# ---------------------------------------------------------
# 5. KHỞI TẠO ĐỒ THỊ 2: SUMMARY FLIGHT HEATMAP TIMELINE
# ---------------------------------------------------------
print("📊 Generating Flight Summary Heatmap...")

# Create heatmap matrix of size 72 rows (sectors) x N columns (frames)
# Default value is NaN (no obstacle)
heatmap_matrix = np.full((72, n_frames), np.nan)

for col_idx, frame in enumerate(frames):
    for sector, dist_cm in zip(frame['sectors'], frame['distances']):
        # Giới hạn góc và khoảng cách
        heatmap_matrix[sector, col_idx] = dist_cm / 100.0  # cm -> m

# Tạo Figure mới cho Heatmap
fig_heatmap, ax_heatmap = plt.subplots(figsize=(14, 7))
fig_heatmap.patch.set_facecolor('#1a1a2e')
ax_heatmap.set_facecolor('#111122') # Nền tối cho những vùng không có vật cản
fig_heatmap.canvas.manager.set_window_title('CubeFC Radar Flight Heatmap')

# Lấy danh sách thời gian và góc (tâm của các ô)
times = np.array([frame['t_sec'] for frame in frames])
angles = np.arange(72) * 5.0  # 72 sector -> tâm từ 0 đến 355 độ

# Custom colormap với màu "bad" (màu đại diện cho NaN / không có vật cản) là màu nền tối
cmap = plt.cm.RdYlGn_r.copy()
cmap.set_bad(color='#111122')

# Vẽ heatmap dạng pcolormesh
mesh = ax_heatmap.pcolormesh(
    times, angles, heatmap_matrix, 
    cmap=cmap, vmin=2.0, vmax=40.0, shading='nearest'
)

# Labels and styling
ax_heatmap.set_title('Obstacle Distance History over Time and Sector Angle', color='white', fontsize=14, fontweight='bold', pad=15)
ax_heatmap.set_xlabel('Flight Time (seconds)', color='white', fontsize=11)
ax_heatmap.set_ylabel('Sector Angle (degrees)', color='white', fontsize=11)

ax_heatmap.tick_params(colors='#a0a0a0', labelsize=9)
ax_heatmap.set_yticks(np.arange(0, 361, 30))  # Tick every 30 degrees
ax_heatmap.set_ylim(0, 360)
if len(times) > 1:
    ax_heatmap.set_xlim(times[0], times[-1])

# Colorbar decoration
cbar = fig_heatmap.colorbar(mesh, ax=ax_heatmap, pad=0.03, shrink=0.8)
cbar.set_label('Obstacle Distance (m)', color='white', fontsize=10, labelpad=10)
cbar.ax.yaxis.set_tick_params(color='white')
plt.setp(plt.getp(cbar.ax.axes, 'yticklabels'), color='white')

ax_heatmap.grid(True, alpha=0.1, color='#ffffff')

plt.tight_layout()

# Hiển thị cả hai cửa sổ đồ thị
plt.show()
