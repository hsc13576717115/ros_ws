#!/usr/bin/env python3
from __future__ import annotations

import os
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Optional

import rclpy
from rcl_interfaces.srv import SetParameters
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool, String, UInt8MultiArray

from r2_arm_control.srv import MoveToXZ, SetArmState


@dataclass(frozen=True)
class ArmStateConfig:
    x: float
    z: float
    ee_linear_speed_limit: float
    ee_linear_acc_limit: float
    kp_joint0: float
    kp_joint1: float
    kd_joint0: float
    kd_joint1: float


@dataclass(frozen=True)
class GpioAction:
    before_move: Optional[bool]
    after_move: Optional[bool]


class ArmStateMachineNode(Node):
    def __init__(self) -> None:
        super().__init__("arm_state_machine_node")

        self.declare_parameter("joy_topic", "/joy")
        self.declare_parameter("move_service_name", "/r2/arm/move_to_xz")
        self.declare_parameter(
            "arm_set_parameters_service",
            "/arm_command_server_node/set_parameters",
        )
        self.declare_parameter("gpio_state_topic", "/r2/manual/dpad_down_gpio36_state")
        self.declare_parameter("state_name_topic", "/r2/arm/state_machine_state")
        self.declare_parameter("feedback_status_topic", "/r2/arm/feedback_status")
        self.declare_parameter("set_state_service_name", "/r2/arm/set_state")
        self.declare_parameter("gpio_number", 36)
        self.declare_parameter("trigger_axis", 7)
        self.declare_parameter("trigger_threshold", -0.5)
        self.declare_parameter("active_high", True)
        self.declare_parameter("set_low_on_shutdown", True)
        self.declare_parameter("auto_enter_idle_on_start", True)
        self.declare_parameter("startup_require_feedback_ok", True)
        self.declare_parameter("export_path", "/sys/class/gpio/export")
        self.declare_parameter("gpio_root", "/sys/class/gpio")

        self.declare_parameter("idle_x", 0.05)
        self.declare_parameter("idle_z", 0.30)
        self.declare_parameter("idle_ee_linear_speed_limit", 0.10)
        self.declare_parameter("idle_ee_linear_acc_limit", 0.25)
        self.declare_parameter("idle_kp_joint0", 60.0)
        self.declare_parameter("idle_kp_joint1", 60.0)
        self.declare_parameter("idle_kd_joint0", 2.0)
        self.declare_parameter("idle_kd_joint1", 2.0)
        self.declare_parameter("startup_idle_ee_linear_speed_limit", 0.04)
        self.declare_parameter("startup_idle_ee_linear_acc_limit", 0.08)
        self.declare_parameter("transition_via_x", 0.35)
        self.declare_parameter("transition_via_z", 0.30)

        self.declare_parameter("pick_x", 0.35)
        self.declare_parameter("pick_z", -0.05)
        self.declare_parameter("pick_ee_linear_speed_limit", 0.10)
        self.declare_parameter("pick_ee_linear_acc_limit", 0.25)
        self.declare_parameter("pick_kp_joint0", 60.0)
        self.declare_parameter("pick_kp_joint1", 60.0)
        self.declare_parameter("pick_kd_joint0", 2.0)
        self.declare_parameter("pick_kd_joint1", 2.0)

        self.declare_parameter("store_x", 0.05)
        self.declare_parameter("store_z", 0.30)
        self.declare_parameter("store_ee_linear_speed_limit", 0.10)
        self.declare_parameter("store_ee_linear_acc_limit", 0.25)
        self.declare_parameter("store_kp_joint0", 60.0)
        self.declare_parameter("store_kp_joint1", 60.0)
        self.declare_parameter("store_kd_joint0", 2.0)
        self.declare_parameter("store_kd_joint1", 2.0)

        self.declare_parameter("place_x", 0.35)
        self.declare_parameter("place_z", -0.05)
        self.declare_parameter("place_ee_linear_speed_limit", 0.10)
        self.declare_parameter("place_ee_linear_acc_limit", 0.25)
        self.declare_parameter("place_kp_joint0", 60.0)
        self.declare_parameter("place_kp_joint1", 60.0)
        self.declare_parameter("place_kd_joint0", 2.0)
        self.declare_parameter("place_kd_joint1", 2.0)

        self.joy_topic = str(self.get_parameter("joy_topic").value)
        self.move_service_name = str(self.get_parameter("move_service_name").value)
        self.arm_set_parameters_service = str(
            self.get_parameter("arm_set_parameters_service").value
        )
        self.gpio_state_topic = str(self.get_parameter("gpio_state_topic").value)
        self.state_name_topic = str(self.get_parameter("state_name_topic").value)
        self.feedback_status_topic = str(self.get_parameter("feedback_status_topic").value)
        self.set_state_service_name = str(
            self.get_parameter("set_state_service_name").value
        )
        self.gpio_number = int(self.get_parameter("gpio_number").value)
        self.trigger_axis = int(self.get_parameter("trigger_axis").value)
        self.trigger_threshold = float(self.get_parameter("trigger_threshold").value)
        self.active_high = bool(self.get_parameter("active_high").value)
        self.set_low_on_shutdown = bool(self.get_parameter("set_low_on_shutdown").value)
        self.auto_enter_idle_on_start = bool(
            self.get_parameter("auto_enter_idle_on_start").value
        )
        self.startup_require_feedback_ok = bool(
            self.get_parameter("startup_require_feedback_ok").value
        )
        self.export_path = Path(str(self.get_parameter("export_path").value))
        self.gpio_root = Path(str(self.get_parameter("gpio_root").value))

        self.state_sequence = ["idle", "pick", "store", "place"]

        self.declare_parameter("auto_chain_states", ["pick", "place"])
        self.declare_parameter("auto_chain_delay_sec", 0.5)
        raw_auto_chain = self.get_parameter("auto_chain_states").value
        self.auto_chain_states = [str(s).strip().lower() for s in raw_auto_chain]
        self.auto_chain_delay_sec = max(
            0.1, float(self.get_parameter("auto_chain_delay_sec").value)
        )

        self.state_configs = {
            state_name: self.load_state_config(state_name)
            for state_name in self.state_sequence
        }
        self.startup_idle_config = ArmStateConfig(
            x=self.state_configs["idle"].x,
            z=self.state_configs["idle"].z,
            ee_linear_speed_limit=float(
                self.get_parameter("startup_idle_ee_linear_speed_limit").value
            ),
            ee_linear_acc_limit=float(
                self.get_parameter("startup_idle_ee_linear_acc_limit").value
            ),
            kp_joint0=self.state_configs["idle"].kp_joint0,
            kp_joint1=self.state_configs["idle"].kp_joint1,
            kd_joint0=self.state_configs["idle"].kd_joint0,
            kd_joint1=self.state_configs["idle"].kd_joint1,
        )
        self.transition_via_x = float(self.get_parameter("transition_via_x").value)
        self.transition_via_z = float(self.get_parameter("transition_via_z").value)
        self.gpio_actions: Dict[str, GpioAction] = {
            "idle": GpioAction(before_move=False, after_move=None),
            "pick": GpioAction(before_move=True, after_move=None),
            "store": GpioAction(before_move=True, after_move=None),
            "place": GpioAction(before_move=None, after_move=False),
        }

        self.state_pub = self.create_publisher(Bool, self.gpio_state_topic, 10)
        self.state_name_pub = self.create_publisher(String, self.state_name_topic, 10)
        self.create_subscription(Joy, self.joy_topic, self.joy_callback, 10)
        self.create_subscription(
            UInt8MultiArray,
            self.feedback_status_topic,
            self.feedback_status_callback,
            10,
        )

        self.move_client = self.create_client(MoveToXZ, self.move_service_name)
        self.param_client = self.create_client(SetParameters, self.arm_set_parameters_service)
        self.set_state_service = self.create_service(
            SetArmState,
            self.set_state_service_name,
            self.set_state_callback,
        )

        self.current_state = "idle"
        self.previous_trigger_pressed = False
        self.logical_high = self.steady_gpio_for_state(self.current_state)
        self.busy = False
        self.startup_sequence_done = not self.auto_enter_idle_on_start
        self.pending_advance_requested = False
        self.transition_lock = threading.Lock()
        self.last_wait_log_time = 0.0
        self.last_feedback_wait_log_time = 0.0
        self.last_move_error = ""
        self.feedback_status_received = False
        self.feedback_ok = False
        self.stale_motor_count = 0
        self.hard_timeout_active = False
        self.startup_grace_active = False

        self.auto_chain_pending = False
        self.auto_chain_target = ""
        self.auto_chain_timer = None

        self.gpio_path = self.gpio_root / f"gpio{self.gpio_number}"
        self.direction_path = self.gpio_path / "direction"
        self.value_path = self.gpio_path / "value"
        self.gpio_ready = self.prepare_gpio()
        if self.gpio_ready:
            self.apply_output(self.logical_high)
        else:
            self.publish_gpio_state()

        self.publish_state_name()
        self.startup_timer = self.create_timer(0.5, self.ensure_idle_state_on_startup)
        self.get_logger().info(
            "Arm state machine ready. sequence=idle->pick->store->place->idle, "
            f"transition_via=({self.transition_via_x:.3f}, {self.transition_via_z:.3f}), "
            f"auto_enter_idle_on_start={self.auto_enter_idle_on_start}, "
            f"startup_require_feedback_ok={self.startup_require_feedback_ok}, "
            f"auto_chain_states={self.auto_chain_states}, "
            f"auto_chain_delay_sec={self.auto_chain_delay_sec:.1f}"
        )

    def load_state_config(self, state_name: str) -> ArmStateConfig:
        return ArmStateConfig(
            x=float(self.get_parameter(f"{state_name}_x").value),
            z=float(self.get_parameter(f"{state_name}_z").value),
            ee_linear_speed_limit=float(
                self.get_parameter(f"{state_name}_ee_linear_speed_limit").value
            ),
            ee_linear_acc_limit=float(
                self.get_parameter(f"{state_name}_ee_linear_acc_limit").value
            ),
            kp_joint0=float(self.get_parameter(f"{state_name}_kp_joint0").value),
            kp_joint1=float(self.get_parameter(f"{state_name}_kp_joint1").value),
            kd_joint0=float(self.get_parameter(f"{state_name}_kd_joint0").value),
            kd_joint1=float(self.get_parameter(f"{state_name}_kd_joint1").value),
        )

    def joy_callback(self, msg: Joy) -> None:
        trigger_pressed = self.read_axis_trigger(msg)
        if trigger_pressed and not self.previous_trigger_pressed:
            if self.auto_chain_pending:
                self.get_logger().warn(
                    f"Ignoring DPad-down press because auto-chain to '{self.auto_chain_target}' is pending."
                )
            elif self.busy:
                self.get_logger().warn(
                    "Ignoring DPad-down press because the arm state machine is busy."
                )
            elif not self.startup_sequence_done:
                self.get_logger().warn("Ignoring DPad-down press because the initial idle sequence is not ready yet.")
            else:
                next_state = self.next_state_name(self.current_state)
                self.get_logger().info(
                    f"DPad-down pressed: {self.current_state} -> {next_state}"
                )
                self.start_transition(next_state)
        self.previous_trigger_pressed = trigger_pressed

    def read_axis_trigger(self, msg: Joy) -> bool:
        if self.trigger_axis < 0 or self.trigger_axis >= len(msg.axes):
            return False

        axis_value = float(msg.axes[self.trigger_axis])
        if self.trigger_threshold >= 0.0:
            return axis_value >= self.trigger_threshold
        return axis_value <= self.trigger_threshold

    def next_state_name(self, state_name: str) -> str:
        state_index = self.state_sequence.index(state_name)
        return self.state_sequence[(state_index + 1) % len(self.state_sequence)]

    def steady_gpio_for_state(self, state_name: str) -> bool:
        if state_name in ("pick", "store"):
            return True
        return False

    def ensure_idle_state_on_startup(self) -> None:
        if (
            not self.auto_enter_idle_on_start
            or self.startup_sequence_done
            or self.busy
            or self.auto_chain_pending
        ):
            return
        if not self.startup_feedback_ready():
            self.log_startup_feedback_wait()
            return
        self.start_transition("idle", is_startup=True)

    def feedback_status_callback(self, msg: UInt8MultiArray) -> None:
        self.feedback_status_received = True
        self.feedback_ok = len(msg.data) > 0 and msg.data[0] != 0
        self.stale_motor_count = int(msg.data[1]) if len(msg.data) > 1 else 0
        self.hard_timeout_active = len(msg.data) > 2 and msg.data[2] != 0
        self.startup_grace_active = len(msg.data) > 3 and msg.data[3] != 0

    def startup_feedback_ready(self) -> bool:
        if not self.startup_require_feedback_ok:
            return True
        return (
            self.feedback_status_received
            and self.feedback_ok
            and self.stale_motor_count == 0
            and not self.hard_timeout_active
        )

    def log_startup_feedback_wait(self) -> None:
        now_sec = time.monotonic()
        if now_sec - self.last_feedback_wait_log_time < 2.0:
            return
        if not self.feedback_status_received:
            detail = f"waiting for {self.feedback_status_topic}"
        else:
            detail = (
                f"feedback_ok={self.feedback_ok}, stale_motor_count={self.stale_motor_count}, "
                f"hard_timeout={self.hard_timeout_active}, startup_grace={self.startup_grace_active}"
            )
        self.get_logger().warn(
            "Startup idle move is waiting for healthy arm feedback: " + detail
        )
        self.last_feedback_wait_log_time = now_sec

    def start_transition(self, target_state: str, is_startup: bool = False) -> None:
        worker = threading.Thread(
            target=self.run_transition,
            args=(target_state, is_startup),
            daemon=True,
        )
        worker.start()

    def set_state_callback(
        self,
        request: SetArmState.Request,
        response: SetArmState.Response,
    ) -> SetArmState.Response:
        target_state = str(request.state).strip().lower()
        if target_state not in self.state_sequence:
            message = (
                f"Unknown arm state '{target_state}'. "
                f"Valid states: {', '.join(self.state_sequence)}"
            )
            self.get_logger().error(message)
            response.accepted = False
            response.state = self.current_state
            response.message = message
            return response

        if self.auto_chain_pending:
            message = (
                f"Arm state machine is pending auto-chain to '{self.auto_chain_target}'. "
                f"Please wait until it finishes."
            )
            self.get_logger().warn(message)
            response.accepted = False
            response.state = self.current_state
            response.message = message
            return response

        if self.busy:
            message = "Arm state machine is busy."
            self.get_logger().warn(message)
            response.accepted = False
            response.state = self.current_state
            response.message = message
            return response

        self.start_transition(target_state)
        message = f"Arm state transition accepted: {target_state}"
        response.accepted = True
        response.state = self.current_state
        response.message = message
        return response

    def run_transition(self, target_state: str, is_startup: bool) -> tuple[bool, str]:
        if target_state not in self.state_sequence:
            message = (
                f"Unknown arm state '{target_state}'. "
                f"Valid states: {', '.join(self.state_sequence)}"
            )
            self.get_logger().error(message)
            return False, message

        with self.transition_lock:
            if self.busy:
                message = "Arm state machine is busy."
                self.get_logger().warn(message)
                return False, message
            self.busy = True

        success = False
        message = ""
        try:
            previous_gpio = self.logical_high
            action = self.gpio_actions[target_state]
            transition_config = self.transition_config(target_state, is_startup)

            if is_startup and not self.startup_feedback_ready():
                message = "Arm feedback is not healthy enough for startup idle move."
                self.get_logger().warn(message)
            elif not self.services_ready():
                now_sec = time.monotonic()
                if now_sec - self.last_wait_log_time >= 2.0:
                    move_ready = self.move_client.wait_for_service(timeout_sec=0.1)
                    param_ready = self.param_client.wait_for_service(timeout_sec=0.1)
                    missing = []
                    if not move_ready:
                        missing.append("/r2/arm/move_to_xz")
                    if not param_ready:
                        missing.append("/arm_command_server_node/set_parameters")
                    self.get_logger().warn(
                        f"Arm state machine waiting for services: {', '.join(missing)}. "
                        f"DPad-down is disabled until idle init succeeds."
                    )
                    self.last_wait_log_time = now_sec
                message = "Move service or parameter service is not ready."
            else:
                if is_startup and target_state == "idle":
                    self.get_logger().info(
                        "Startup safe-idle move: returning to idle coordinates with reduced speed."
                    )

                if action.before_move is not None:
                    self.apply_output(action.before_move)

                skip_transition_via = is_startup and target_state == "idle"

                if not self.apply_arm_motion_profile(transition_config):
                    self.restore_gpio(previous_gpio)
                    message = "Failed to apply arm motion profile."
                    self.get_logger().error(message)
                via_ok = skip_transition_via or self.move_to_transition_via_target(target_state)
                if not via_ok and (
                    self.is_best_effort_reached(self.last_move_error)
                    or self.is_stale_feedback_error(self.last_move_error)
                ):
                    self.get_logger().warn(
                        f"Transition via for '{target_state}' treated as reached despite warning: "
                        f"{self.last_move_error}"
                    )
                    via_ok = True

                if not via_ok:
                    self.restore_gpio(previous_gpio)
                    message = f"Failed to move through transition point for state '{target_state}'."
                    self.get_logger().error(message)
                elif not self.move_to_state_target(target_state, transition_config):
                    if self.is_best_effort_reached(self.last_move_error):
                        self.get_logger().warn(
                            f"Treating state '{target_state}' as reached despite final feedback warning: "
                            f"{self.last_move_error}"
                        )
                        success = True
                    elif is_startup and self.is_stale_feedback_error(self.last_move_error):
                        self.restore_gpio(previous_gpio)
                        message = (
                            f"Stale joint feedback during startup move to '{target_state}'; "
                            f"will retry on next timer tick."
                        )
                        self.get_logger().warn(message)
                    else:
                        self.restore_gpio(previous_gpio)
                        message = f"Failed to move to arm state '{target_state}'."
                        if self.last_move_error:
                            message += f" Reason: {self.last_move_error}"
                        self.get_logger().error(message)
                else:
                    success = True

                if success:
                    if action.after_move is not None:
                        self.apply_output(action.after_move)

                    self.current_state = target_state
                    self.startup_sequence_done = True
                    self.publish_state_name()
                    message = f"Arm state machine entered state '{target_state}'."
                    self.get_logger().info(message)

                    if is_startup and target_state == "idle":
                        self.get_logger().info("Arm state machine initialized to idle state.")

                    # Auto-chain: after certain states, automatically transition to the next
                    if (
                        not is_startup
                        and target_state in self.auto_chain_states
                    ):
                        next_auto = self.next_state_name(target_state)
                        self.auto_chain_target = next_auto
                        self.auto_chain_pending = True
                        self.get_logger().info(
                            f"Auto-chain scheduled: {target_state} -> {next_auto} "
                            f"after {self.auto_chain_delay_sec:.1f}s"
                        )
                        self.auto_chain_timer = self.create_timer(
                            self.auto_chain_delay_sec,
                            self._on_auto_chain_tick,
                        )
        finally:
            self.busy = False

        if success:
            self.pending_advance_requested = False
        return success, message

    def _on_auto_chain_tick(self) -> None:
        if self.auto_chain_timer is not None:
            self.auto_chain_timer.cancel()
            self.auto_chain_timer = None
        self.auto_chain_pending = False
        target = self.auto_chain_target
        self.auto_chain_target = ""
        if target and target in self.state_sequence:
            self.get_logger().info(f"Auto-chain executing: {self.current_state} -> {target}")
            self.start_transition(target)

    def services_ready(self) -> bool:
        move_ready = self.move_client.wait_for_service(timeout_sec=0.1)
        param_ready = self.param_client.wait_for_service(timeout_sec=0.1)
        return move_ready and param_ready

    def transition_config(self, target_state: str, is_startup: bool) -> ArmStateConfig:
        if is_startup and target_state == "idle":
            return self.startup_idle_config
        return self.state_configs[target_state]

    def transition_gpio_state(self, previous_gpio: bool, action: GpioAction) -> bool:
        if action.before_move is not None:
            return bool(action.before_move)
        return bool(previous_gpio)

    @staticmethod
    def is_best_effort_reached(message: str) -> bool:
        lowered = message.lower()
        return (
            "trajectory stream completed" in lowered
            or "reached the hold phase" in lowered
        )

    @staticmethod
    def is_stale_feedback_error(message: str) -> bool:
        lowered = message.lower()
        return (
            "measured joint feedback is stale" in lowered
            or "joint feedback was stale" in lowered
        )

    def apply_arm_motion_profile(self, config: ArmStateConfig) -> bool:
        request = SetParameters.Request()
        request.parameters = [
            Parameter("ee_linear_speed_limit", value=config.ee_linear_speed_limit).to_parameter_msg(),
            Parameter("ee_linear_acc_limit", value=config.ee_linear_acc_limit).to_parameter_msg(),
            Parameter("kp_joint0", value=config.kp_joint0).to_parameter_msg(),
            Parameter("kp_joint1", value=config.kp_joint1).to_parameter_msg(),
            Parameter("kd_joint0", value=config.kd_joint0).to_parameter_msg(),
            Parameter("kd_joint1", value=config.kd_joint1).to_parameter_msg(),
        ]

        try:
            response = self.param_client.call(request)
        except Exception as exc:
            self.get_logger().error(f"Failed to set arm motion profile parameters: {exc}")
            return False

        if response is None or len(response.results) != len(request.parameters):
            self.get_logger().error("Arm motion profile parameter update returned an invalid response.")
            return False

        failed_reasons = [
            result.reason
            for result in response.results
            if not result.successful
        ]
        if failed_reasons:
            self.get_logger().error(
                "Arm motion profile parameter update failed: " + " | ".join(failed_reasons)
            )
            return False

        return True

    def move_to_transition_via_target(self, state_name: str) -> bool:
        return self.move_to_cartesian_target(
            f"{state_name}:transition_via",
            self.transition_via_x,
            self.transition_via_z,
        )

    def move_to_cartesian_target(self, label: str, x: float, z: float) -> bool:
        self.last_move_error = ""
        request = MoveToXZ.Request()
        request.x = x
        request.z = z

        try:
            response = self.move_client.call(request)
        except Exception as exc:
            self.last_move_error = str(exc)
            self.get_logger().error(f"Move service call failed for '{label}': {exc}")
            return False

        if response is None:
            self.last_move_error = "empty response"
            self.get_logger().error(f"Move service returned no response for '{label}'.")
            return False

        if not response.accepted:
            self.last_move_error = response.message
            if self.is_stale_feedback_error(response.message):
                self.get_logger().warn(
                    f"Move service rejected '{label}' (transient): {response.message}"
                )
            elif self.is_best_effort_reached(response.message):
                self.get_logger().warn(
                    f"Move service rejected '{label}' (best-effort): {response.message}"
                )
            else:
                self.get_logger().error(
                    f"Move service rejected '{label}': {response.message}"
                )
            return False

        self.get_logger().info(
            f"Target '{label}' reached x={x:.3f}, z={z:.3f}. {response.message}"
        )
        return True

    def move_to_state_target(self, state_name: str, config: Optional[ArmStateConfig] = None) -> bool:
        if config is None:
            config = self.state_configs[state_name]
        return self.move_to_cartesian_target(state_name, config.x, config.z)

    def prepare_gpio(self) -> bool:
        if self.gpio_number < 0:
            self.get_logger().info("GPIO output disabled because gpio_number < 0.")
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

        for _ in range(20):
            if os.access(self.direction_path, os.W_OK):
                break
            time.sleep(0.05)

        for attempt in range(5):
            try:
                self.direction_path.write_text("out\n", encoding="ascii")
                return True
            except OSError as exc:
                if attempt == 4:
                    self.get_logger().error(
                        f"Failed to set GPIO {self.gpio_number} direction to output: {exc}"
                    )
                    return False
                time.sleep(0.05)

        return False

    def restore_gpio(self, logical_high: bool) -> None:
        self.apply_output(logical_high)
        self.publish_state_name()

    def apply_output(self, logical_high: bool) -> None:
        self.logical_high = bool(logical_high)
        physical_high = self.logical_high if self.active_high else not self.logical_high
        value_text = "1\n" if physical_high else "0\n"

        if not self.gpio_ready:
            self.gpio_ready = self.prepare_gpio()

        if self.gpio_ready:
            try:
                self.value_path.write_text(value_text, encoding="ascii")
            except OSError as exc:
                self.get_logger().error(
                    f"Failed to write GPIO {self.gpio_number} output state: {exc}"
                )
                self.gpio_ready = False

        self.publish_gpio_state()

    def publish_gpio_state(self) -> None:
        self.state_pub.publish(Bool(data=bool(self.logical_high)))

    def publish_state_name(self) -> None:
        self.state_name_pub.publish(String(data=self.current_state))

    def destroy_node(self) -> bool:
        if self.set_low_on_shutdown and self.gpio_ready and rclpy.ok():
            self.apply_output(False)
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ArmStateMachineNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
