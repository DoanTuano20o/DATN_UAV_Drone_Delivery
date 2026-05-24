# DATN UAV Drone Delivery

ROS2 UAV Drone Delivery system using ArduPilot/MAVLink, OpenCV ArUco,
Flask-SocketIO Web Dashboard, GPS mission and servo payload drop.

## Thanh phan chinh

- ROS2 Humble
- ArduPilot / MAVLink
- Orange Pi 5 Pro
- Flywoo Goku H743 / Pixhawk-class FC
- OpenCV ArUco
- Flask + Flask-SocketIO
- PCA9685 + MG996R servo

## Chay local

```bash
cd /media/orangepi/nvme_data/home/DATN_UAV
source .venv/bin/activate
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch uav_bringup full_system.launch.py
```

## Chay public web

```bash
./tools/start_public_uav_web_all.sh
```

## Admin demo

- username: `admin`
- password: `Drone111`

## Luu y an toan

- Thao canh khi test trong nha hoac khi kiem thu tren ban.
- Khong bay trong khu vuc cam bay hoac noi khong duoc phep.
- Luon co nguoi giam sat va san sang ngat he thong khi bay thu.
- Khong dua ngrok authtoken, GitHub token, secret key that vao README,
  source code hoac bat ky file nao trong repository.
