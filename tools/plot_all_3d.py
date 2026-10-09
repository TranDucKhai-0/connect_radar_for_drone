import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import sys
import os

def main():
    # File mặc định nếu không truyền argument
    csv_file = "../blackbox/radar_log.csv"
    
    # Nếu người dùng truyền file path vào qua command line
    if len(sys.argv) > 1:
        csv_file = sys.argv[1]

    if not os.path.exists(csv_file):
        print(f"Error: CSV file not found: {csv_file}")
        print("Usage: python3 plot_all_3d.py <csv_file_path>")
        sys.exit(1)

    print(f"Reading data from: {csv_file}...")
    try:
        df = pd.read_csv(csv_file)
    except Exception as e:
        print(f"Error reading CSV file: {e}")
        sys.exit(1)

    # Check if required columns exist
    required_cols = {'X', 'Y', 'DroneAlt'}
    if not required_cols.issubset(df.columns):
        print(f"Error: CSV file is missing required columns. Requires: {required_cols}")
        sys.exit(1)

    # =========================================================
    # KHỐI LỌC ĐIỂM RADAR THEO GÓC (Dễ dàng comment để tắt)
    # =========================================================
    # Radar 1: |Angle| <= 0.393 (Front)
    # Radar 2: 1.178 <= Angle <= 1.963 (Right)
    # Radar 3: |Angle| >= 2.749 (Back)
    # Radar 4: -1.963 <= Angle <= -1.178 (Left)
    if 'Angle' not in df.columns and 'X' in df.columns and 'Y' in df.columns:
        import numpy as np
        df['Angle'] = np.arctan2(df['Y'], df['X'])

    if 'Angle' in df.columns:
        mask_r1 = df['Angle'].abs() <= 0.393
        mask_r2 = (df['Angle'] >= 1.178) & (df['Angle'] <= 1.963)
        mask_r3 = df['Angle'].abs() >= 2.749
        mask_r4 = (df['Angle'] >= -1.963) & (df['Angle'] <= -1.178)
        df = df[mask_r1 | mask_r2 | mask_r3 | mask_r4]
    # =========================================================
    # KHỐI LỌC KHOẢNG CÁCH RANGE < 20M CHO RADAR TRÁI/PHẢI (Dễ dàng comment để tắt)
    # =========================================================
    if 'Range' not in df.columns and 'X' in df.columns and 'Y' in df.columns:
        import numpy as np
        df['Range'] = np.sqrt(df['X']**2 + df['Y']**2)

    if 'Angle' in df.columns and 'Range' in df.columns:
        # Xác định các điểm thuộc Radar Trái (Radar 4) hoặc Phải (Radar 2)
        is_left_right = ((df['Angle'] >= 1.178) & (df['Angle'] <= 1.963)) | \
                        ((df['Angle'] >= -1.963) & (df['Angle'] <= -1.178))
        # Loại bỏ các điểm thuộc radar Trái/Phải có Range >= 20m
        # df = df[~(is_left_right & (df['Range'] >= 20.0))]
    # =========================================================

    print(f"Loaded {len(df)} data points. Plotting 3D...")

    # Initialize 3D plot
    fig = plt.figure(figsize=(10, 8))
    fig.canvas.manager.set_window_title('All 3D Radar Points')
    
    ax = fig.add_subplot(111, projection='3d')
    ax.set_facecolor('#1e1e2e')
    fig.patch.set_facecolor('#1e1e2e')

    # Vẽ toàn bộ điểm (không quan tâm thời gian)
    # Tô màu theo độ cao (DroneAlt) để dễ phân biệt
    scatter = ax.scatter(
        df['X'], 
        df['Y'], 
        df['DroneAlt'], 
        c=df['DroneAlt'], 
        cmap='plasma', 
        marker='o', 
        s=10, 
        alpha=0.6, 
        edgecolors='none',
        label='Radar Points'
    )

    # Draw drone at origin (0, 0, 0)
    ax.scatter(
        [0], [0], [0], 
        color='red', 
        marker='^', 
        s=100, 
        label='Drone Origin (0,0,0)'
    )

    # Add colorbar for altitude
    cbar = fig.colorbar(scatter, ax=ax, pad=0.1, shrink=0.7)
    cbar.set_label('Drone Altitude (m)', color='white')
    cbar.ax.yaxis.set_tick_params(color='white')
    plt.setp(plt.getp(cbar.ax.axes, 'yticklabels'), color='white')

    # Axis decoration
    ax.set_title('3D Radar Obstacle Map (X, Y, Alt)', color='white', fontsize=14, pad=15)
    ax.set_xlabel('X - Forward (m)', color='#a0a0a0', fontsize=10, labelpad=10)
    ax.set_ylabel('Y - Right (m)', color='#a0a0a0', fontsize=10, labelpad=10)
    ax.set_zlabel('Drone Altitude (m)', color='#a0a0a0', fontsize=10, labelpad=10)
    
    # Legend
    legend = ax.legend(facecolor='#1e1e2e', edgecolor='#333333', loc='upper left')
    plt.setp(legend.get_texts(), color='white')
    
    ax.tick_params(colors='#707070', labelsize=9)
    ax.xaxis.pane.fill = False
    ax.yaxis.pane.fill = False
    ax.zaxis.pane.fill = False
    ax.xaxis.pane.set_edgecolor('#333333')
    ax.yaxis.pane.set_edgecolor('#333333')
    ax.zaxis.pane.set_edgecolor('#333333')
    ax.grid(True, alpha=0.2)

    # Đảo ngược trục Z nếu muốn mô phỏng đúng FRD (tuỳ chọn)
    # Tuy nhiên vì Alt đã là độ cao dương (do user yêu cầu ở bản trước) nên cứ để bình thường
    
    # Góc nhìn mặc định
    ax.view_init(elev=30, azim=-45)

    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    main()
