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
    # Binary sau cmake nằm tại build/connect_radar_for_drone hoặc build/radar_app
    [ -f connect_radar_for_drone ] && mv connect_radar_for_drone radar_app || true
else
    # Compile thẳng ra build/radar_app
    g++ -std=c++17 -I../include ../src/*.cpp ../main.cpp -o radar_app -lpthread >> build.log || true
fi
cd ..

echo "Build Completed"
echo "################################"

echo "Set permission for RadarApp"
chmod +x build/radar_app || true
echo "################################"

echo "Install the service"

mkdir -p /usr/local/etc/connect_radar_for_drone/blackbox
mkdir -p /usr/local/etc/connect_radar_for_drone/service
mkdir -p /usr/local/etc/connect_radar_for_drone/build

FILE=build/radar_app
if [[ -f "$FILE" ]]; then
    cp -f build/radar_app /usr/local/etc/connect_radar_for_drone/build/
else
    echo "Failed to install RadarApp — build/radar_app not found"
    exit 1
fi

# Chép script cấu hình CAN vào thư mục hệ thống và cấp quyền chạy
cp service/setup_can.sh /usr/local/etc/connect_radar_for_drone/service/
chmod +x /usr/local/etc/connect_radar_for_drone/service/setup_can.sh

# Chép 2 file systemd service
cp service/can_setup.service /etc/systemd/system/
cp service/radar_setup.service /etc/systemd/system/

sudo systemctl daemon-reload

sudo systemctl enable can_setup.service
sudo systemctl start can_setup.service

sudo systemctl enable radar_setup.service
sudo systemctl start radar_setup.service

echo "Completed"
echo "################################"
