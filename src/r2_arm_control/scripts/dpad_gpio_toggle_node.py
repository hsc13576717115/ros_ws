#!/usr/bin/env python3
from __future__ import annotations

import os
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool


class DpadGpioToggleNode(Node):
    def __init__(self) -> None:
        super().__init__("dpad_gpio_toggle_node")

        self.declare_parameter("joy_topic", "/joy")
        self.declare_parameter("state_topic", "/r2/manual/dpad_down_gpio36_state")
        self.declare_parameter("gpio_number", 36)
        self.declare_parameter("trigger_axis", 7)
        self.declare_parameter("trigger_threshold", -0.5)
        self.declare_parameter("initial_high", False)
        self.declare_parameter("active_high", True)
        self.declare_parameter("set_low_on_shutdown", False)
        self.declare_parameter("export_path", "/sys/class/gpio/export")
        self.declare_parameter("gpio_root", "/sys/class/gpio")

        joy_topic = str(self.get_parameter("joy_topic").value)
        state_topic = str(self.get_parameter("state_topic").value)
        self.gpio_number = int(self.get_parameter("gpio_number").value)
        self.trigger_axis = int(self.get_parameter("trigger_axis").value)
        self.trigger_threshold = float(self.get_parameter("trigger_threshold").value)
        self.logical_high = bool(self.get_parameter("initial_high").value)
        self.active_high = bool(self.get_parameter("active_high").value)
        self.set_low_on_shutdown = bool(self.get_parameter("set_low_on_shutdown").value)
        self.export_path = Path(str(self.get_parameter("export_path").value))
        self.gpio_root = Path(str(self.get_parameter("gpio_root").value))

        self.previous_trigger_pressed = False
        self.gpio_ready = False
        self.gpio_path = self.gpio_root / f"gpio{self.gpio_number}"
        self.direction_path = self.gpio_path / "direction"
        self.value_path = self.gpio_path / "value"

        self.state_pub = self.create_publisher(Bool, state_topic, 10)
        self.create_subscription(Joy, joy_topic, self.joy_callback, 10)

        self.gpio_ready = self.prepare_gpio()
        if self.gpio_ready:
            self.apply_output(self.logical_high)
        else:
            self.publish_state()

    def joy_callback(self, msg: Joy) -> None:
        trigger_pressed = self.read_axis_trigger(msg)
        if trigger_pressed and not self.previous_trigger_pressed:
            self.logical_high = not self.logical_high
            self.apply_output(self.logical_high)
        self.previous_trigger_pressed = trigger_pressed

    def read_axis_trigger(self, msg: Joy) -> bool:
        if self.trigger_axis < 0 or self.trigger_axis >= len(msg.axes):
            return False

        axis_value = float(msg.axes[self.trigger_axis])
        if self.trigger_threshold >= 0.0:
            return axis_value >= self.trigger_threshold
        return axis_value <= self.trigger_threshold

    def prepare_gpio(self) -> bool:
        if self.gpio_number < 0:
            self.get_logger().info("DPad GPIO toggle disabled because gpio_number < 0.")
            return False

        if not self.gpio_path.exists():
            if not os.access(self.export_path, os.W_OK):
                self.get_logger().error(
                    f"Cannot export GPIO {self.gpio_number}: {self.export_path} is not writable."
                )
                return False

            try:
                self.export_path.write_text(f"{self.gpio_number}\n", encoding="ascii")
            except OSError as exc:
                self.get_logger().error(f"Failed to export GPIO {self.gpio_number}: {exc}")
                return False

            for _ in range(20):
                if self.gpio_path.exists():
                    break
                time.sleep(0.05)

        if not self.gpio_path.exists():
            self.get_logger().error(f"GPIO path {self.gpio_path} did not appear after export.")
            return False

        try:
            self.direction_path.write_text("out\n", encoding="ascii")
        except OSError as exc:
            self.get_logger().error(
                f"Failed to set GPIO {self.gpio_number} direction to output: {exc}"
            )
            return False

        return True

    def apply_output(self, logical_high: bool) -> None:
        physical_high = logical_high if self.active_high else not logical_high
        value_text = "1\n" if physical_high else "0\n"

        if self.gpio_ready:
            try:
                self.value_path.write_text(value_text, encoding="ascii")
            except OSError as exc:
                self.get_logger().error(
                    f"Failed to write GPIO {self.gpio_number} output state: {exc}"
                )
                self.gpio_ready = False

        self.publish_state()

    def publish_state(self) -> None:
        self.state_pub.publish(Bool(data=bool(self.logical_high)))

    def destroy_node(self) -> bool:
        if self.set_low_on_shutdown and self.gpio_ready:
            self.logical_high = False
            self.apply_output(False)
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DpadGpioToggleNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
