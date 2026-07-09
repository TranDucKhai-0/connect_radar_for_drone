#!/usr/bin/env python3
"""
Radar Log 2D Visualizer
Chỉ vẽ đồ thị điểm trên mặt phẳng 2D dựa vào cột X và Y từ file radar_log.csv.
"""

import sys
import pandas as pd
import matplotlib.pyplot as plt

def main():
    file_path = sys.argv[1] if len(sys.argv) > 1 else 'radar_log.csv'
    
    try:
        df = pd.read_csv(file_path)
    except FileNotFoundError:
        print(f"Error: File not found: '{file_path}'.")
        print("Usage: python3 plot_radar_2d.py <csv_file_path>")
        sys.exit(1)
    
    # Check if X and Y columns exist
    if 'X' not in df.columns or 'Y' not in df.columns:
        print(f"Error: Missing column 'X' or 'Y' in CSV file. Existing columns: {list(df.columns)}")
        sys.exit(1)
        
    # Loại bỏ các hàng bị NaN ở cột X hoặc Y
    df = df.dropna(subset=['X', 'Y'])

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
    
    if len(df) == 0:
        print("Error: CSV file has no valid data (after removing NaNs, angle filtering, and range filtering).")
        sys.exit(1)
        
    print(f"Successfully read {len(df)} data points.")
    
    # Thiết lập đồ thị
    plt.figure(figsize=(10, 10))
    
    # Hệ trục FRD: X là tiến (Forward), Y là sang phải (Right).
    # Trên đồ thị 2D (nhìn từ trên xuống):
    # - Trục dọc (tung) sẽ là trục X (Tiến)
    # - Trục ngang (hoành) sẽ là trục Y (Sang phải)
    
    # Vẽ các điểm radar
    plt.scatter(df['Y'], df['X'], s=15, alpha=0.6, color='blue', label='Radar Points')
    
    # Đánh dấu vị trí drone tại góc toạ độ (0, 0)
    plt.scatter([0], [0], color='red', marker='^', s=100, label='Drone Origin (0,0)')
    
    # Add title and labels
    plt.title('2D Radar Plot - FRD Frame (X: Forward, Y: Right)', fontsize=14, fontweight='bold')
    plt.xlabel('Y Axis (Right) [m]', fontsize=12)
    plt.ylabel('X Axis (Forward) [m]', fontsize=12)
    
    # Tuỳ chỉnh lưới và trục
    plt.grid(True, linestyle='--', alpha=0.7)
    plt.axhline(0, color='black', linewidth=1)
    plt.axvline(0, color='black', linewidth=1)
    
    # Giữ tỉ lệ trục X, Y bằng nhau để hiển thị hình học chuẩn
    plt.axis('equal')
    
    plt.legend()
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    main()
