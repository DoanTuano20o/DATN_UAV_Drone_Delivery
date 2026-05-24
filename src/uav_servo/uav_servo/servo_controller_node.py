#!/usr/bin/env python3
from __future__ import annotations

import threading
import time
from typing import Any

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from smbus2 import SMBus


BUS_ID = 5
I2C_ADDRESS = 0x40
SERVO_CHANNEL = 4
PWM_FREQUENCY = 50

SERVO_MIN_US = 500
SERVO_MAX_US = 2500
SERVO_CLOSE_ANGLE = 10.0
SERVO_OPEN_ANGLE = 55.0
SERVO_OPEN_HOLD_S = 3.0

PCA9685_MODE1 = 0x00
PCA9685_PRESCALE = 0xFE
LED0_ON_L = 0x06


def parse_int(value: Any, default: int) -> int:
    try:
        if isinstance(value, str):
            return int(value, 0)
        return int(value)
    except Exception:
        return int(default)


def parse_float(value: Any, default: float) -> float:
    try:
        return float(value)
    except Exception:
        return float(default)


class PCA9685:
    def __init__(self, bus_id: int = BUS_ID, address: int = I2C_ADDRESS, frequency: int = PWM_FREQUENCY):
        self.bus_id = int(bus_id)
        self.address = int(address)
        self.frequency = int(frequency)
        self.bus = SMBus(self.bus_id)

        self.write8(PCA9685_MODE1, 0x00)
        time.sleep(0.01)
        self.set_pwm_freq(self.frequency)

    def write8(self, reg: int, value: int) -> None:
        self.bus.write_byte_data(self.address, reg, value & 0xFF)

    def read8(self, reg: int) -> int:
        return self.bus.read_byte_data(self.address, reg)

    def set_pwm_freq(self, freq_hz: int) -> None:
        prescaleval = 25000000.0 / 4096.0 / float(freq_hz) - 1.0
        prescale = int(prescaleval + 0.5)

        oldmode = self.read8(PCA9685_MODE1)
        sleep_mode = (oldmode & 0x7F) | 0x10

        self.write8(PCA9685_MODE1, sleep_mode)
        self.write8(PCA9685_PRESCALE, prescale)
        self.write8(PCA9685_MODE1, oldmode)
        time.sleep(0.005)
        self.write8(PCA9685_MODE1, oldmode | 0xA1)

    def set_pwm(self, channel: int, on_tick: int, off_tick: int) -> None:
        reg = LED0_ON_L + 4 * int(channel)
        self.write8(reg + 0, on_tick & 0xFF)
        self.write8(reg + 1, (on_tick >> 8) & 0xFF)
        self.write8(reg + 2, off_tick & 0xFF)
        self.write8(reg + 3, (off_tick >> 8) & 0xFF)

    def release_channel(self, channel: int) -> None:
        self.set_pwm(channel, 0, 0)

    def close(self) -> None:
        self.bus.close()


class ServoDropper:
    def __init__(
        self,
        bus_id: int = BUS_ID,
        address: int = I2C_ADDRESS,
        channel: int = SERVO_CHANNEL,
        frequency: int = PWM_FREQUENCY,
        min_us: float = SERVO_MIN_US,
        max_us: float = SERVO_MAX_US,
        close_angle: float = SERVO_CLOSE_ANGLE,
        open_angle: float = SERVO_OPEN_ANGLE,
    ):
        self.channel = int(channel)
        self.frequency = int(frequency)
        self.min_us = float(min_us)
        self.max_us = float(max_us)
        self.close_angle = float(close_angle)
        self.open_angle = float(open_angle)
        self.pca = PCA9685(bus_id=bus_id, address=address, frequency=frequency)

    def angle_to_ticks(self, angle_deg: float) -> int:
        angle_deg = max(0.0, min(180.0, float(angle_deg)))
        pulse_us = self.min_us + (self.max_us - self.min_us) * (angle_deg / 180.0)
        period_us = 1_000_000.0 / float(self.frequency)
        ticks = int((pulse_us / period_us) * 4096)
        return max(0, min(4095, ticks))

    def set_angle(self, angle_deg: float) -> int:
        ticks = self.angle_to_ticks(angle_deg)
        self.pca.set_pwm(self.channel, 0, ticks)
        return ticks

    def open(self) -> int:
        return self.set_angle(self.open_angle)

    def close_servo(self) -> int:
        return self.set_angle(self.close_angle)

    def cleanup(self, release_pwm: bool = False) -> None:
        try:
            if release_pwm:
                self.pca.release_channel(self.channel)
        finally:
            self.pca.close()


class ServoControllerNode(Node):
    def __init__(self) -> None:
        super().__init__("servo_controller_node")

        self.declare_parameter("servo_cmd_topic", "/servo_cmd")
        self.declare_parameter("drop_done_topic", "/drop_done")
        self.declare_parameter("expected_drop_cmd", "DROP")
        self.declare_parameter("drop_done_payload", "DONE")

        self.declare_parameter("bus_id", BUS_ID)
        self.declare_parameter("i2c_address", I2C_ADDRESS)
        self.declare_parameter("servo_channel", SERVO_CHANNEL)
        self.declare_parameter("pwm_frequency", PWM_FREQUENCY)

        self.declare_parameter("servo_min_us", float(SERVO_MIN_US))
        self.declare_parameter("servo_max_us", float(SERVO_MAX_US))
        self.declare_parameter("servo_close_angle", float(SERVO_CLOSE_ANGLE))
        self.declare_parameter("servo_open_angle", float(SERVO_OPEN_ANGLE))
        self.declare_parameter("servo_open_hold_s", float(SERVO_OPEN_HOLD_S))

        # Khi node khởi động chỉ đưa servo về góc đóng, KHÔNG tự DROP.
        self.declare_parameter("init_close_on_start", True)
        self.declare_parameter("release_pwm_on_shutdown", False)

        self.servo_cmd_topic = str(self.get_parameter("servo_cmd_topic").value)
        self.drop_done_topic = str(self.get_parameter("drop_done_topic").value)
        self.expected_drop_cmd = str(self.get_parameter("expected_drop_cmd").value).strip().upper()
        self.drop_done_payload = str(self.get_parameter("drop_done_payload").value)

        self.bus_id = parse_int(self.get_parameter("bus_id").value, BUS_ID)
        self.i2c_address = parse_int(self.get_parameter("i2c_address").value, I2C_ADDRESS)
        self.servo_channel = parse_int(self.get_parameter("servo_channel").value, SERVO_CHANNEL)
        self.pwm_frequency = parse_int(self.get_parameter("pwm_frequency").value, PWM_FREQUENCY)

        self.servo_min_us = parse_float(self.get_parameter("servo_min_us").value, SERVO_MIN_US)
        self.servo_max_us = parse_float(self.get_parameter("servo_max_us").value, SERVO_MAX_US)
        self.servo_close_angle = parse_float(self.get_parameter("servo_close_angle").value, SERVO_CLOSE_ANGLE)
        self.servo_open_angle = parse_float(self.get_parameter("servo_open_angle").value, SERVO_OPEN_ANGLE)
        self.servo_open_hold_s = parse_float(self.get_parameter("servo_open_hold_s").value, SERVO_OPEN_HOLD_S)

        self.init_close_on_start = bool(self.get_parameter("init_close_on_start").value)
        self.release_pwm_on_shutdown = bool(self.get_parameter("release_pwm_on_shutdown").value)

        self.dropper: ServoDropper | None = None
        self.drop_busy = False
        self.drop_lock = threading.Lock()

        self.drop_done_pub = self.create_publisher(String, self.drop_done_topic, 10)
        self.cmd_sub = self.create_subscription(
            String,
            self.servo_cmd_topic,
            self.on_servo_cmd,
            10,
        )

        self.init_hardware()

        self.get_logger().info(
            "ServoControllerNode started | "
            f"cmd_topic={self.servo_cmd_topic} done_topic={self.drop_done_topic} "
            f"bus=/dev/i2c-{self.bus_id} addr=0x{self.i2c_address:02X} "
            f"ch={self.servo_channel} close={self.servo_close_angle:.1f} "
            f"open={self.servo_open_angle:.1f} hold={self.servo_open_hold_s:.1f}s"
        )

    def init_hardware(self) -> None:
        try:
            self.dropper = ServoDropper(
                bus_id=self.bus_id,
                address=self.i2c_address,
                channel=self.servo_channel,
                frequency=self.pwm_frequency,
                min_us=self.servo_min_us,
                max_us=self.servo_max_us,
                close_angle=self.servo_close_angle,
                open_angle=self.servo_open_angle,
            )

            if self.init_close_on_start:
                ticks = self.dropper.close_servo()
                self.get_logger().info(
                    f"Servo initialized CLOSED | angle={self.servo_close_angle:.1f} ticks={ticks}"
                )

        except FileNotFoundError as e:
            self.get_logger().error(
                f"Khong tim thay /dev/i2c-{self.bus_id}. Kiem tra bus I2C da enable chua. {e}"
            )
            raise
        except PermissionError as e:
            self.get_logger().error(
                f"Khong du quyen mo /dev/i2c-{self.bus_id}. Thu sudo hoac them user vao group i2c. {e}"
            )
            raise
        except OSError as e:
            self.get_logger().error(
                f"Loi I2C PCA9685 bus={self.bus_id}, addr=0x{self.i2c_address:02X}: {e}. "
                f"Thu quet: sudo i2cdetect -y {self.bus_id}"
            )
            raise

    def on_servo_cmd(self, msg: String) -> None:
        cmd = str(msg.data).strip().upper()

        if cmd == "":
            return

        if cmd == self.expected_drop_cmd:
            self.start_drop()
            return

        if cmd == "OPEN":
            self.manual_open()
            return

        if cmd == "CLOSE":
            self.manual_close()
            return

        self.get_logger().warn(
            f"Ignored servo command: {msg.data!r}. Expected {self.expected_drop_cmd!r}, OPEN, or CLOSE."
        )

    def manual_open(self) -> None:
        if self.dropper is None:
            self.get_logger().error("Servo hardware not initialized.")
            return

        with self.drop_lock:
            ticks = self.dropper.open()

        self.get_logger().info(
            f"Manual OPEN | angle={self.servo_open_angle:.1f} ticks={ticks}"
        )

    def manual_close(self) -> None:
        if self.dropper is None:
            self.get_logger().error("Servo hardware not initialized.")
            return

        with self.drop_lock:
            ticks = self.dropper.close_servo()

        self.get_logger().info(
            f"Manual CLOSE | angle={self.servo_close_angle:.1f} ticks={ticks}"
        )

    def start_drop(self) -> None:
        if self.dropper is None:
            self.get_logger().error("Servo hardware not initialized; cannot drop.")
            return

        with self.drop_lock:
            if self.drop_busy:
                self.get_logger().warn("DROP ignored: servo is already busy.")
                return
            self.drop_busy = True

        thread = threading.Thread(target=self.drop_worker, daemon=True)
        thread.start()

    def drop_worker(self) -> None:
        success = False

        try:
            self.get_logger().warn("DROP_PAYLOAD: OPEN servo")
            open_ticks = self.dropper.open() if self.dropper is not None else -1
            self.get_logger().info(
                f"Servo OPEN | angle={self.servo_open_angle:.1f} ticks={open_ticks}"
            )

            time.sleep(max(0.0, self.servo_open_hold_s))

            self.get_logger().warn("DROP_PAYLOAD: CLOSE servo")
            close_ticks = self.dropper.close_servo() if self.dropper is not None else -1
            self.get_logger().info(
                f"Servo CLOSE | angle={self.servo_close_angle:.1f} ticks={close_ticks}"
            )

            success = True

        except Exception as e:
            self.get_logger().error(f"DROP_PAYLOAD failed: {e}")

        finally:
            with self.drop_lock:
                self.drop_busy = False

            if success:
                done = String()
                done.data = self.drop_done_payload
                self.drop_done_pub.publish(done)
                self.get_logger().warn(
                    f"DROP_DONE published | topic={self.drop_done_topic} data={self.drop_done_payload}"
                )

    def destroy_node(self) -> bool:
        try:
            if self.dropper is not None:
                self.dropper.cleanup(release_pwm=self.release_pwm_on_shutdown)
                self.get_logger().info("Servo hardware closed.")
        except Exception as e:
            self.get_logger().warn(f"Servo cleanup error: {e}")

        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ServoControllerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass

        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
