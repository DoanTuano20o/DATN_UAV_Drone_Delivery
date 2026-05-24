#!/usr/bin/env python3
import time
from smbus2 import SMBus

# ===== Cấu hình đã chốt =====
BUS_ID = 5
I2C_ADDRESS = 0x40
SERVO_CHANNEL = 3
PWM_FREQUENCY = 50

# MG996R: chỉnh lại nếu thực tế mở/đóng chưa đúng
SERVO_MIN_US = 500
SERVO_MAX_US = 2500
SERVO_CLOSE_ANGLE = 0
SERVO_OPEN_ANGLE = 38
SERVO_OPEN_HOLD_S = 5.0

PCA9685_MODE1 = 0x00
PCA9685_PRESCALE = 0xFE
LED0_ON_L = 0x06


class PCA9685:
    def __init__(self, bus_id=BUS_ID, address=I2C_ADDRESS, frequency=PWM_FREQUENCY):
        self.bus_id = bus_id
        self.address = address
        self.frequency = frequency
        self.bus = SMBus(bus_id)

        # Reset mode
        self.write8(PCA9685_MODE1, 0x00)
        time.sleep(0.01)
        self.set_pwm_freq(frequency)

    def write8(self, reg, value):
        self.bus.write_byte_data(self.address, reg, value & 0xFF)

    def read8(self, reg):
        return self.bus.read_byte_data(self.address, reg)

    def set_pwm_freq(self, freq_hz):
        prescaleval = 25000000.0 / 4096.0 / float(freq_hz) - 1.0
        prescale = int(prescaleval + 0.5)

        oldmode = self.read8(PCA9685_MODE1)
        sleep_mode = (oldmode & 0x7F) | 0x10

        self.write8(PCA9685_MODE1, sleep_mode)
        self.write8(PCA9685_PRESCALE, prescale)
        self.write8(PCA9685_MODE1, oldmode)
        time.sleep(0.005)
        self.write8(PCA9685_MODE1, oldmode | 0xA1)

    def set_pwm(self, channel, on_tick, off_tick):
        reg = LED0_ON_L + 4 * channel
        self.write8(reg + 0, on_tick & 0xFF)
        self.write8(reg + 1, (on_tick >> 8) & 0xFF)
        self.write8(reg + 2, off_tick & 0xFF)
        self.write8(reg + 3, (off_tick >> 8) & 0xFF)

    def release_channel(self, channel):
        self.set_pwm(channel, 0, 0)

    def close(self):
        self.bus.close()


class ServoDropper:
    def __init__(
        self,
        bus_id=BUS_ID,
        address=I2C_ADDRESS,
        channel=SERVO_CHANNEL,
        min_us=SERVO_MIN_US,
        max_us=SERVO_MAX_US,
        close_angle=SERVO_CLOSE_ANGLE,
        open_angle=SERVO_OPEN_ANGLE,
    ):
        self.channel = channel
        self.min_us = min_us
        self.max_us = max_us
        self.close_angle = close_angle
        self.open_angle = open_angle
        self.pca = PCA9685(bus_id=bus_id, address=address, frequency=PWM_FREQUENCY)

    def angle_to_ticks(self, angle_deg):
        angle_deg = max(0.0, min(180.0, float(angle_deg)))
        pulse_us = self.min_us + (self.max_us - self.min_us) * (angle_deg / 180.0)
        period_us = 1_000_000.0 / PWM_FREQUENCY
        ticks = int((pulse_us / period_us) * 4096)
        return max(0, min(4095, ticks))

    def set_angle(self, angle_deg):
        ticks = self.angle_to_ticks(angle_deg)
        self.pca.set_pwm(self.channel, 0, ticks)
        print(f"[servo] ch={self.channel} angle={angle_deg:.1f} ticks={ticks}")

    def open(self):
        self.set_angle(self.open_angle)

    def close_servo(self):
        self.set_angle(self.close_angle)

    def drop(self, hold_s=SERVO_OPEN_HOLD_S):
        print(f"[dropper] OPEN {self.open_angle} deg")
        self.open()
        time.sleep(hold_s)
        print(f"[dropper] CLOSE {self.close_angle} deg")
        self.close_servo()

    def cleanup(self, release_pwm=False):
        try:
            if release_pwm:
                self.pca.release_channel(self.channel)
        finally:
            self.pca.close()


def main():
    print(f"[config] bus_id={BUS_ID}, address=0x{I2C_ADDRESS:02X}, channel={SERVO_CHANNEL}")
    dropper = None
    try:
        dropper = ServoDropper()
        dropper.close_servo()
        time.sleep(1.0)
        dropper.drop()
    except FileNotFoundError:
        print(f"[ERROR] Khong tim thay /dev/i2c-{BUS_ID}. Kiem tra bus I2C da duoc enable chua.")
        raise
    except PermissionError:
        print(f"[ERROR] Khong du quyen mo /dev/i2c-{BUS_ID}. Thu chay bang sudo hoac them user vao group i2c.")
        raise
    except OSError as e:
        print(f"[ERROR] Loi giao tiep I2C voi PCA9685 tai bus={BUS_ID}, address=0x{I2C_ADDRESS:02X}: {e}")
        print("[GOI Y] Kiem tra lai VCC 3.3V, SDA, SCL, GND chung va quet bang: sudo i2cdetect -y 5")
        raise
    finally:
        if dropper is not None:
            dropper.cleanup(release_pwm=False)


if __name__ == "__main__":
    main()
