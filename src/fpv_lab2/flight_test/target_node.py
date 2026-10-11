"""Вузол польоту до цільової точки (лабораторна робота №2, варіант 9).

Послідовність: перевірка системи -> GUIDED -> arm -> зліт -> стабілізація
-> політ до цілі (x0 - dx, y0 - dy) -> зупинка -> LAND -> завершення.
Базові стани (перевірка, GUIDED, arm, посадка) беруться з наданого прикладу.
"""
import math
import time
from functools import partial

from geometry_msgs.msg import TwistStamped
from ardupilot_msgs.srv import Takeoff

from .flight_state import FlightState
from .flight_test_node import FlightTestNode
from .handlers import HANDLERS
from .handlers.handler import FlightStateHandler
from .navigator import TargetNavigator


class TargetTakingOffHandler(FlightStateHandler):
    """Запам'ятовує стартову позицію, викликає зліт."""

    def handle(self, node):
        if node.request_in_progress:
            return
        if node.odometry is None:
            return  # чекаємо перших даних одометрії
        if node.start_xyz is None:
            p = node.odometry.pose.pose.position
            node.start_xyz = (p.x, p.y, p.z)
            node.target_xy = (
                p.x - node.delta_x,
                p.y - node.delta_y,
            )
            node.get_logger().info(
                f"Start position: x={p.x:.3f} y={p.y:.3f} z={p.z:.3f}"
            )
            node.get_logger().info(
                f"Offsets: dx={node.delta_x} dy={node.delta_y}; "
                f"target: x={node.target_xy[0]:.3f} y={node.target_xy[1]:.3f}"
            )
        node.request_in_progress = True
        request = Takeoff.Request()
        request.alt = float(node.TAKEOFF_ALTITUDE)
        future = node.takeoff_client.call_async(request)
        future.add_done_callback(partial(self.callback, node))
        node.get_logger().info(
            f"Requesting takeoff to {node.TAKEOFF_ALTITUDE:.1f} m"
        )

    def callback(self, node, future):
        node.request_in_progress = False
        try:
            response = future.result()
        except Exception as error:
            node.fail(f"Takeoff service failed: {error}")
            return
        if not response.status:
            node.fail("ArduPilot rejected the takeoff request")
            return
        node.get_logger().info("Takeoff command accepted, climbing...")
        node.takeoff_cmd_time = time.monotonic()
        node.hover_started_at = None
        node.state = FlightState.HOVERING


class TargetHoveringHandler(FlightStateHandler):
    """Чекає набору висоти, потім стабілізується в зависанні."""

    STABILIZE_TIME = 5.0

    def handle(self, node):
        if node.status is None or not node.status.armed:
            node.fail("Drone disarmed unexpectedly during flight")
            return
        z_rel = node.odometry.pose.pose.position.z - node.start_xyz[2]
        if node.hover_started_at is None:
            if z_rel >= node.TAKEOFF_ALTITUDE - 0.2:
                node.hover_started_at = time.monotonic()
                node.get_logger().info(
                    f"Altitude reached ({z_rel:.2f} m), stabilizing "
                    f"for {self.STABILIZE_TIME:.0f} s"
                )
            elif time.monotonic() - node.takeoff_cmd_time > 40.0:
                node.fail("Takeoff timeout: altitude not reached")
            return
        if time.monotonic() - node.hover_started_at >= self.STABILIZE_TIME:
            node.navigator = TargetNavigator(
                node.target_xy[0],
                node.target_xy[1],
                v_max=node.v_max,
                kp=node.kp,
                tolerance=node.tolerance,
            )
            node.get_logger().info("Hover stable, flying to target")
            node.state = FlightState.NAVIGATING


class NavigatingHandler(FlightStateHandler):
    def handle(self, node):
        if node.status is None or not node.status.armed:
            node.fail("Drone disarmed unexpectedly during flight")
            return
        p = node.odometry.pose.pose.position
        cmd = node.navigator.update(p.x, p.y, time.monotonic())

        if cmd.diverging:
            node.send_velocity(0.0, 0.0, 0.0)
            node.get_logger().error(
                "Distance to target is growing: check cmd_frame_id / "
                "velocity sign parameters. Landing for safety."
            )
            node.state = FlightState.LANDING
            return

        if cmd.reached:
            node.get_logger().info(
                f"Target reached (distance {cmd.distance:.3f} m)"
            )
            node.stop_started_at = time.monotonic()
            node.state = FlightState.STOPPING
            return

        z_target = node.start_xyz[2] + node.TAKEOFF_ALTITUDE
        vz = max(-0.3, min(0.3, 0.5 * (z_target - p.z)))
        node.send_velocity(cmd.vx, cmd.vy, vz)

        now = time.monotonic()
        if now - node.last_progress_log >= 1.0:
            node.last_progress_log = now
            node.get_logger().info(
                f"pos=({p.x:.2f}, {p.y:.2f}) dist={cmd.distance:.2f} m "
                f"v=({cmd.vx:.2f}, {cmd.vy:.2f})"
            )


class StoppingHandler(FlightStateHandler):
    """Зупиняє горизонтальний рух, фіксує підсумкову позицію і похибку."""

    STOP_TIME = 3.0

    def handle(self, node):
        node.send_velocity(0.0, 0.0, 0.0)
        if time.monotonic() - node.stop_started_at < self.STOP_TIME:
            return
        p = node.odometry.pose.pose.position
        tx, ty = node.target_xy
        error = math.hypot(p.x - tx, p.y - ty)
        sx, sy, _ = node.start_xyz
        log = node.get_logger()
        log.info("=== RESULT BEFORE LANDING ===")
        log.info(f"Start:  x={sx:.3f} y={sy:.3f}")
        log.info(f"Target: x={tx:.3f} y={ty:.3f}")
        log.info(f"Actual: x={p.x:.3f} y={p.y:.3f} z={p.z:.3f}")
        log.info(f"Position error e = {error:.3f} m")
        ok = error <= node.tolerance + 0.1
        log.info("Accuracy requirement (0.2-0.3 m): "
                 + ("OK" if ok else "NOT MET"))
        node.final_error = error
        node.state = FlightState.LANDING


class FlyToTargetNode(FlightTestNode):
    def __init__(self):
        super().__init__()
        # Варіант 9: dx = 5.2, dy = -0.1
        self.declare_parameter("delta_x", 5.2)
        self.declare_parameter("delta_y", -0.1)
        self.declare_parameter("takeoff_altitude", 2.0)
        self.declare_parameter("v_max", 1.0)
        self.declare_parameter("kp", 0.6)
        self.declare_parameter("tolerance", 0.2)
        self.declare_parameter("cmd_frame_id", "map")
        self.declare_parameter("vel_sign_x", 1.0)
        self.declare_parameter("vel_sign_y", 1.0)

        self.delta_x = self.get_parameter("delta_x").value
        self.delta_y = self.get_parameter("delta_y").value
        self.TAKEOFF_ALTITUDE = float(
            self.get_parameter("takeoff_altitude").value)
        self.v_max = self.get_parameter("v_max").value
        self.kp = self.get_parameter("kp").value
        self.tolerance = self.get_parameter("tolerance").value
        self.cmd_frame_id = self.get_parameter("cmd_frame_id").value
        self.sign_x = self.get_parameter("vel_sign_x").value
        self.sign_y = self.get_parameter("vel_sign_y").value

        self.start_xyz = None
        self.target_xy = None
        self.navigator = None
        self.stop_started_at = None
        self.takeoff_cmd_time = time.monotonic()
        self.last_progress_log = 0.0
        self.final_error = None

        self.cmd_vel_pub = self.create_publisher(
            TwistStamped, "/ap/v1/cmd_vel", 10)

        handlers = dict(HANDLERS)
        handlers[FlightState.TAKING_OFF] = TargetTakingOffHandler()
        handlers[FlightState.HOVERING] = TargetHoveringHandler()
        handlers[FlightState.NAVIGATING] = NavigatingHandler()
        handlers[FlightState.STOPPING] = StoppingHandler()
        self._handlers = handlers

    def send_velocity(self, vx, vy, vz):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.cmd_frame_id
        msg.twist.linear.x = float(vx * self.sign_x)
        msg.twist.linear.y = float(vy * self.sign_y)
        msg.twist.linear.z = float(vz)
        self.cmd_vel_pub.publish(msg)
