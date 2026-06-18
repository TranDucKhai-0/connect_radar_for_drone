#!/usr/bin/env python3
"""
Radar Log Visualizer — Vẽ point cloud từ file radar_log.csv
Hai đồ thị:
  - Trái:  X, Y  (Cartesian FRD, nhìn từ trên xuống)
  - Phải:  72-Sector OBSTACLE_DISTANCE (mô phỏng y hệt bản tin gửi FC)
Thanh trượt ở dưới để kéo qua lại theo TimestampMs.
"""

import sys
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
from matplotlib.patches import Wedge
from matplotlib.collections import PatchCollection

# ---------------------------------------------------------
# 1. ĐỌC DỮ LIỆU
# ---------------------------------------------------------
file_path = sys.argv[1] if len(sys.argv) > 1 else 'radar_log.csv'

try:
    df = pd.read_csv(file_path)
except FileNotFoundError:
    print(f"Không tìm thấy file '{file_path}'.")
    print("Cách dùng: python3 plot_radar_log.py <đường_dẫn_file.csv>")
    sys.exit(1)

required = ['TimestampMs', 'X', 'Y', 'Z', 'Range', 'Angle', 'DroneAlt']
for col in required:
    if col not in df.columns:
        print(f"Thiếu cột '{col}' trong file CSV. Các cột hiện có: {list(df.columns)}")
        sys.exit(1)

df = df.dropna(subset=required)

# =========================================================
# KHỐI LỌC ĐIỂM RADAR THEO GÓC (Dễ dàng comment để tắt)
# =========================================================
# Radar 1: |Angle| <= 0.393 (Front)
# Radar 2: 1.178 <= Angle <= 1.963 (Right)
# Radar 3: |Angle| >= 2.749 (Back)
# Radar 4: -1.963 <= Angle <= -1.178 (Left)
if 'Angle' not in df.columns and 'X' in df.columns and 'Y' in df.columns:
    df['Angle'] = np.arctan2(df['Y'], df['X'])

if 'Angle' in df.columns:
    mask_r1 = df['Angle'].abs() <= 0.393
    mask_r2 = (df['Angle'] >= 1.178) & (df['Angle'] <= 1.963)
    mask_r3 = df['Angle'].abs() >= 2.749
    mask_r4 = (df['Angle'] >= -1.963) & (df['Angle'] <= -1.178)
    df = df[mask_r1 | mask_r2 | mask_r3 | mask_r4]
# =========================================================

# =========================================================
# KHỐI LỌC KHOẢNG CÁCH RANGE < 20M CHO RADAR TRÁI/PHẢI (Dễ dàng comment để tắt)
# =========================================================
if 'Range' not in df.columns and 'X' in df.columns and 'Y' in df.columns:
    df['Range'] = np.sqrt(df['X']**2 + df['Y']**2)

if 'Angle' in df.columns and 'Range' in df.columns:
    # Xác định các điểm thuộc Radar Trái (Radar 4) hoặc Phải (Radar 2)
    is_left_right = ((df['Angle'] >= 1.178) & (df['Angle'] <= 1.963)) | \
                    ((df['Angle'] >= -1.963) & (df['Angle'] <= -1.178))
    # Loại bỏ các điểm thuộc radar Trái/Phải có Range >= 20m
    df = df[~(is_left_right & (df['Range'] >= 20.0))]
# =========================================================

t0 = df['TimestampMs'].min()
df['t_sec'] = (df['TimestampMs'] - t0) / 1000.0

timestamps = sorted(df['TimestampMs'].unique())
n_frames = len(timestamps)

if n_frames == 0:
    print("File CSV không có dữ liệu hợp lệ (sau khi lọc góc).")
    sys.exit(1)

print(f"Đọc được {len(df)} điểm, {n_frames} frames, "
      f"từ t=0s đến t={(timestamps[-1] - t0)/1000:.1f}s")

# ---------------------------------------------------------
# 2. HÀM TÍNH 72-SECTOR (y hệt SendDataToFcThread trong C++)
# ---------------------------------------------------------
def compute_72_sectors(frame_df):
    """
    Mô phỏng chính xác thuật toán trong SendDataToFcThread:
    1. Khởi tạo mảng 72 phần tử = 65535 (UINT16_MAX = không có vật cản)
    2. Với mỗi obstacle: range*100 -> cm, angle radian -> degree -> normalize [0,360)
    3. Tính sector index = round(angle_deg / 5) % 72
    4. Giữ khoảng cách nhỏ nhất trong mỗi sector
    """
    distances = np.full(72, 65535, dtype=np.uint16)

    for _, obs in frame_df.iterrows():
        dist_cm = obs['Range'] * 100.0

        # Đổi radian sang degree
        angle_deg = obs['Angle'] * (180.0 / np.pi)

        # Chuẩn hoá [0, 360)
        while angle_deg < 0:
            angle_deg += 360.0
        while angle_deg >= 360.0:
            angle_deg -= 360.0

        # Index sector (round to nearest, y hệt code C++)
        idx = int(angle_deg / 5.0 + 0.5) % 72

        # Giữ min distance
        if distances[idx] == 65535 or dist_cm < distances[idx]:
            distances[idx] = int(dist_cm)

    return distances

# ---------------------------------------------------------
# 3. THIẾT LẬP ĐỒ THỊ
# ---------------------------------------------------------
fig = plt.figure(figsize=(16, 8))
fig.patch.set_facecolor('#1a1a2e')
fig.suptitle('Radar Point Cloud Visualizer', color='white',
             fontsize=14, fontweight='bold')

# Subplot trái: Cartesian 3D (X, Y, DroneAlt)
ax1 = fig.add_subplot(121, projection='3d', facecolor='#16213e')
# Subplot phải: 72-Sector Polar (mô phỏng OBSTACLE_DISTANCE)
ax2 = fig.add_subplot(122, projection='polar', facecolor='#16213e')

plt.subplots_adjust(bottom=0.18, left=0.05, right=0.95, top=0.90, wspace=0.25)

# Thanh slider
ax_slider = plt.axes([0.15, 0.04, 0.7, 0.03], facecolor='#0f3460')
slider = Slider(ax_slider, 'Frame', 0, n_frames - 1,
                valinit=0, valstep=1, color='#e94560')

time_text = fig.text(0.5, 0.09, '', ha='center', va='center',
                     color='#e94560', fontsize=11, fontweight='bold')

# ---------------------------------------------------------
# 4. STYLE
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

# Polar style
ax2.set_title('Bản tin gửi FC',
              color='white', fontsize=10, pad=15)
ax2.set_facecolor('#16213e')
ax2.tick_params(colors='#707070', labelsize=7)
ax2.set_theta_zero_location('N')    # 0° = phía trước (North on plot)
ax2.set_theta_direction(-1)         # Chiều kim đồng hồ (CW) — chuẩn FRD
ax2.grid(True, alpha=0.15, color='#444444')
ax2.set_rmax(40)                    # Max range 40m
ax2.set_rlabel_position(45)

# Giới hạn trục cố định cho 3D Cartesian
x_min, x_max = df['X'].min() - 1, df['X'].max() + 1
y_min, y_max = df['Y'].min() - 1, df['Y'].max() + 1
alt_min = df['DroneAlt'].min() - 0.5
alt_max = df['DroneAlt'].max() + 0.5

TRAIL_LENGTH = 20

# Lưu plot objects
scatter1 = [None]
drone_marker1 = [None]
sector_bars = [None]
point_scatter2 = [None]

# ---------------------------------------------------------
# 5. HÀM CẬP NHẬT
# ---------------------------------------------------------
def update(val):
    idx = int(slider.val)

    # Trail frames
    start_idx = max(0, idx - TRAIL_LENGTH + 1)
    trail_ts = timestamps[start_idx:idx + 1]
    current_ts = timestamps[idx]

    mask = df['TimestampMs'].isin(trail_ts)
    trail_df = df[mask].copy()

    # Current frame only (cho 72-sector)
    current_df = df[df['TimestampMs'] == current_ts]

    # Alpha theo tuổi
    ts_to_age = {ts: i for i, ts in enumerate(trail_ts)}
    trail_df['age'] = trail_df['TimestampMs'].map(ts_to_age)
    trail_df['alpha'] = 0.15 + 0.85 * (trail_df['age'] / max(len(trail_ts) - 1, 1))

    # =============================================
    # PLOT 1: Cartesian 3D (X, Y, DroneAlt)
    # =============================================
    if scatter1[0] is not None:
        scatter1[0].remove()
    if drone_marker1[0] is not None:
        drone_marker1[0].remove()

    current_alt = trail_df['DroneAlt'].iloc[-1] if not trail_df.empty else 0

    scatter1[0] = ax1.scatter(
        trail_df['X'], trail_df['Y'], trail_df['DroneAlt'],
        c=trail_df['alpha'], cmap='plasma',
        marker='o', s=25, alpha=0.8, edgecolors='none',
        vmin=0, vmax=1
    )
    drone_marker1[0] = ax1.scatter(
        [0], [0], [current_alt],
        c='#00ff88', marker='^', s=150,
        edgecolors='white', linewidths=1.0, zorder=10
    )
    ax1.set_xlim(x_min, x_max)
    ax1.set_ylim(y_min, y_max)
    ax1.set_zlim(alt_min, alt_max)

    # =============================================
    # PLOT 2: 72-Sector (y hệt FC)
    # =============================================
    # Xóa các bar cũ
    if sector_bars[0] is not None:
        sector_bars[0].remove()
    if point_scatter2[0] is not None:
        point_scatter2[0].remove()

    # Tính 72 sectors
    sectors = compute_72_sectors(current_df)

    # Vẽ các sector có vật cản dưới dạng bar trên polar plot
    theta_centers = []
    radii = []
    colors = []

    for i in range(72):
        if sectors[i] < 65535:
            angle_rad = np.deg2rad(i * 5.0)
            dist_m = sectors[i] / 100.0
            theta_centers.append(angle_rad)
            radii.append(dist_m)

            # Màu theo khoảng cách: gần = đỏ (nguy hiểm), xa = xanh (an toàn)
            normalized = min(dist_m / 40.0, 1.0)
            colors.append(plt.cm.RdYlGn(normalized))  # Red -> Yellow -> Green

    width = np.deg2rad(5.0)  # Mỗi sector rộng 5°

    if theta_centers:
        sector_bars[0] = ax2.bar(
            theta_centers, radii, width=width,
            color=colors, alpha=0.75, edgecolor='#ffffff44', linewidth=0.5,
            bottom=0
        )
    else:
        sector_bars[0] = None

    # Vẽ các điểm raw lên polar (để so sánh)
    if not current_df.empty:
        raw_angles = current_df['Angle'].values
        # Chuẩn hoá angle sang [0, 2π) giống code C++
        raw_angles_norm = raw_angles.copy()
        raw_angles_norm = np.where(raw_angles_norm < 0, raw_angles_norm + 2*np.pi, raw_angles_norm)
        raw_ranges = current_df['Range'].values

        point_scatter2[0] = ax2.scatter(
            raw_angles_norm, raw_ranges,
            c='#ff4444', marker='x', s=40, zorder=10, linewidths=1.5
        )
    else:
        point_scatter2[0] = None

    ax2.set_rmax(40)

    # Cập nhật text
    elapsed = (current_ts - t0) / 1000.0
    n_points = len(current_df)
    n_active_sectors = sum(1 for s in sectors if s < 65535)
    time_text.set_text(
        f't = {elapsed:.2f}s  |  Frame {idx + 1}/{n_frames}  |  '
        f'{n_points} point(s)  |  {n_active_sectors}/72 sectors active'
    )

    fig.canvas.draw_idle()

# ---------------------------------------------------------
# 6. PHÍM TẮT
# ---------------------------------------------------------
def on_key(event):
    idx = int(slider.val)
    if event.key == 'right' and idx < n_frames - 1:
        slider.set_val(idx + 1)
    elif event.key == 'left' and idx > 0:
        slider.set_val(idx - 1)
    elif event.key == 'home':
        slider.set_val(0)
    elif event.key == 'end':
        slider.set_val(n_frames - 1)

fig.canvas.mpl_connect('key_press_event', on_key)
slider.on_changed(update)

# Vẽ frame đầu tiên
update(0)

fig.text(0.5, 0.005,
         '← → : Lùi/Tiến frame  |  Home/End : Đầu/Cuối  |  Kéo thanh slider để scrub  |'
         '  × = Raw point  |  Bar = Sector gửi FC',
         ha='center', color='#555555', fontsize=8)

plt.show()
