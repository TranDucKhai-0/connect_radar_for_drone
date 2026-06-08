#!/bin/bash

echo "################################"
echo "Build Connect Radar program"
cd ..
set -e

# Tự động chọn CMake hoặc g++ (dự phòng)
mkdir -p build
cd build
if command -v cmake &> /dev/null; then
    cmake ..
    make >> build.log || true
    cp connect_radar_for_drone ../radar_app || true
else
    g++ -std=c++17 -I../include ../src/*.cpp ../main.cpp -o ../radar_app -lpthread >> build.log || true
fi

cd ../service
echo "Build Completed"
echo "################################"

echo "Set permission for RadarApp"
# Cấp quyền thực thi
chmod +x ../radar_app || true
echo "################################"

echo "Install the service"

if [ -d /usr/local/etc/connect_radar_for_drone ]
then
    echo "Directory /usr/local/etc/connect_radar_for_drone exists"
else
    mkdir -p /usr/local/etc/connect_radar_for_drone
fi

# Tạo thư mục blackbox và thư mục chứa script
mkdir -p /usr/local/etc/connect_radar_for_drone/blackbox
mkdir -p /usr/local/etc/connect_radar_for_drone/service

FILE=../radar_app
if [[ -f "$FILE" ]]; then
    rm -rf /usr/local/etc/connect_radar_for_drone/build/
    mkdir -p /usr/local/etc/connect_radar_for_drone/build/
    cp -rf ../radar_app /usr/local/etc/connect_radar_for_drone/build/
else
    echo "Failed to install RadarApp"
fi

# Chép script cấu hình CAN vào thư mục hệ thống và cấp quyền chạy
cp setup_can.sh /usr/local/etc/connect_radar_for_drone/service/
chmod +x /usr/local/etc/connect_radar_for_drone/service/setup_can.sh

# Chép 2 file systemd service
cp can_setup.service /etc/systemd/system/
cp radar_setup.service /etc/systemd/system/

sudo systemctl daemon-reload

sudo systemctl enable can_setup.service
sudo systemctl start can_setup.service

sudo systemctl enable radar_setup.service
sudo systemctl start radar_setup.service

echo "Completed"
echo "################################"
