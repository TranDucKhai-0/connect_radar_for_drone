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
        print(f"Không tìm thấy file '{file_path}'.")
        print("Cách dùng: python3 plot_radar_2d.py <đường_dẫn_file.csv>")
        sys.exit(1)
    
    # Kiểm tra sự tồn tại của cột X và Y
    if 'X' not in df.columns or 'Y' not in df.columns:
        print(f"Thiếu cột 'X' hoặc 'Y' trong file CSV. Các cột hiện có: {list(df.columns)}")
        sys.exit(1)
        
    # Loại bỏ các hàng bị NaN ở cột X hoặc Y
    df = df.dropna(subset=['X', 'Y'])
    
    if len(df) == 0:
        print("File CSV không có dữ liệu hợp lệ (sau khi loại bỏ giá trị trống).")
        sys.exit(1)
        
    print(f"Đọc thành công {len(df)} điểm dữ liệu.")
    
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
    
    # Thêm tiêu đề và nhãn
    plt.title('Đồ thị Radar 2D - Hệ trục FRD (X: Tiến, Y: Sang phải)', fontsize=14, fontweight='bold')
    plt.xlabel('Trục Y (Sang phải) [m]', fontsize=12)
    plt.ylabel('Trục X (Tiến tới) [m]', fontsize=12)
    
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
