# Public web UAV bằng ngrok

## A. Mục tiêu

Public web UAV từ Orange Pi cho máy khác mạng truy cập landing page, không cần DNS riêng. Web local/LAN vẫn giữ nguyên:

```bash
http://192.168.1.27:5000/
http://localhost:5000
```

ngrok sẽ public local service:

```bash
http://localhost:5000
```

Link public cuối cùng sẽ do script in ra, thường có dạng:

```bash
https://xxxxx.ngrok-free.app
```

## B. Cài ngrok trên Linux ARM64

```bash
cd /media/orangepi/nvme_data/home/DATN_UAV
chmod +x tools/install_ngrok_arm64.sh
./tools/install_ngrok_arm64.sh
```

Script chỉ cài binary `ngrok` vào `/usr/local/bin/ngrok`, không ghi authtoken vào source project.

## C. Thêm authtoken

Vào ngrok dashboard để lấy authtoken, sau đó chạy:

```bash
ngrok config add-authtoken <YOUR_NGROK_AUTHTOKEN>
```

Không commit authtoken, file cấu hình ngrok thật, hoặc log runtime lên repository.

## D. Chạy tất cả bằng 1 terminal

Điều kiện trước khi chạy:

1. Đã cài ngrok:

```bash
./tools/install_ngrok_arm64.sh
```

2. Đã add authtoken thật:

```bash
ngrok config add-authtoken <YOUR_REAL_NGROK_AUTHTOKEN>
```

Không dùng chữ placeholder `YOUR_NGROK_AUTHTOKEN`. Không đưa authtoken vào ảnh, git, chat, script, hoặc tài liệu public.

Nếu lỡ lộ token:

- Vào ngrok dashboard reset/revoke token.
- Add token mới trên Orange Pi.

Chạy public web UAV chỉ bằng một terminal:

```bash
cd /media/orangepi/nvme_data/home/DATN_UAV
chmod +x tools/start_public_uav_web_all.sh
./tools/start_public_uav_web_all.sh
```

Script sẽ tự:

- Source `.venv`, ROS2 Humble, và `install/setup.bash`.
- Export tài khoản admin demo.
- Chạy `ros2 launch uav_bringup full_system.launch.py`.
- Đợi web local ở `http://127.0.0.1:5000`.
- Chạy ngrok tunnel.
- In `PUBLIC WEB URL`.
- Giữ terminal chạy cho tới khi nhấn `Ctrl+C`.

Log runtime:

```bash
/tmp/uav_full_system.launch.log
/tmp/uav_public_web.ngrok.log
```

## E. Cách chạy thủ công bằng 2 terminal

### Terminal 1: Chạy web UAV

```bash
cd /media/orangepi/nvme_data/home/DATN_UAV
source .venv/bin/activate
source /opt/ros/humble/setup.bash
source install/setup.bash

export UAV_ADMIN_USERNAME='admin'
export UAV_ADMIN_PASSWORD='Drone111'
export UAV_WEB_SECRET_KEY='drone-delivery-secret-key'

ros2 launch uav_bringup full_system.launch.py
```

### Terminal 2: Chạy public tunnel

```bash
cd /media/orangepi/nvme_data/home/DATN_UAV
chmod +x tools/start_ngrok_public_web.sh
./tools/start_ngrok_public_web.sh
```

Nếu dùng ngrok Free Dev Domain/reserved domain, có thể truyền domain trước khi chạy:

```bash
export NGROK_DOMAIN='your-dev-domain.ngrok-free.app'
./tools/start_ngrok_public_web.sh
```

## F. Link cuối cùng

Script sẽ tự đọc ngrok local API:

```bash
http://127.0.0.1:4040/api/tunnels
```

và in ra:

```bash
========================================
PUBLIC WEB URL:
https://xxxxx.ngrok-free.app
========================================
```

## G. Chia sẻ cho người khác

Landing public:

```bash
https://xxxxx.ngrok-free.app/
```

Tracking public:

```bash
https://xxxxx.ngrok-free.app/tracking
```

Dashboard admin:

```bash
https://xxxxx.ngrok-free.app/dashboard
```

Tài khoản demo:

```bash
Username: admin
Password: Drone111
```

## H. Test từ mạng khác

Dùng điện thoại 4G hoặc máy không cùng Wi-Fi mở public URL:

- Landing vào thẳng.
- Tracking vào thẳng.
- Dashboard chuyển sang trang đăng nhập nếu chưa login.
- Sau khi login đúng, dashboard hoạt động như local.

## I. Cảnh báo bảo mật

- Không public dashboard nếu chưa có login backend.
- Không dùng mật khẩu `Drone111` lâu dài khi demo thật.
- Nên đổi `UAV_ADMIN_PASSWORD` trước khi public cho nhiều người.
- Nên đặt `UAV_WEB_SECRET_KEY` dài và ngẫu nhiên.
- Không chia sẻ ngrok authtoken.
- Không commit file chứa secret thật.
