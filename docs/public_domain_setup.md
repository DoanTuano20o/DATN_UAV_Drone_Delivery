# Public Domain And Tunnel Setup

Tên tunnel/domain mong muốn:

```text
drone-delivery-hcmute
```

`drone-delivery-hcmute` có thể dùng làm tunnel name. Để người khác mạng truy cập qua internet, cần hostname đầy đủ theo domain bạn sở hữu:

```text
drone-delivery-hcmute.<your-domain>
```

Ví dụ:

```text
drone-delivery-hcmute.aiotsemi.com
```

Nếu chưa có domain riêng, có thể dùng URL do Cloudflare Tunnel quick tunnel hoặc ngrok cấp.

## Cloudflare Tunnel

Web Flask đang chạy local:

```text
http://localhost:5000
```

Cấu hình khuyến nghị:

```text
Tunnel name: drone-delivery-hcmute
Public hostname: drone-delivery-hcmute.<your-domain>
Service: http://localhost:5000
```

Cloudflare Tunnel route public hostname về local service thông qua tunnel, không cần mở port router.

## Ngrok Demo Nhanh

```bash
ngrok http 5000
```

Ngrok sẽ cấp một public URL tạm thời để demo nhanh.

## Admin Environment Variables

Trước khi public thật, nên đổi tài khoản/mật khẩu và secret key:

```bash
export UAV_ADMIN_USERNAME='admin'
export UAV_ADMIN_PASSWORD='Drone111'
export UAV_WEB_SECRET_KEY='random-long-secret-key'
```

Nếu chạy bằng systemd:

```ini
Environment=UAV_ADMIN_USERNAME=admin
Environment=UAV_ADMIN_PASSWORD=Drone111
Environment=UAV_WEB_SECRET_KEY=random-long-secret-key
```

Nếu chạy qua HTTPS/tunnel và muốn cookie chỉ gửi qua HTTPS:

```bash
export UAV_COOKIE_SECURE=1
```

## Cảnh Báo Public

- Không public dashboard nếu chưa có login backend.
- Không dùng mật khẩu mặc định lâu dài.
- Nên đổi `UAV_ADMIN_PASSWORD` và `UAV_WEB_SECRET_KEY` trước khi public thật.
- Không commit file chứa secret thật.

## Test Public Hostname

Landing public:

```text
https://drone-delivery-hcmute.<your-domain>/
```

Dashboard bắt đăng nhập:

```text
https://drone-delivery-hcmute.<your-domain>/dashboard
```
