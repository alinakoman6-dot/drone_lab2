#!/usr/bin/env python3
import rclpy

from .flight_state import FlightState
from .target_node import FlyToTargetNode


def main(args=None):
    rclpy.init(args=args)
    node = FlyToTargetNode()
    code = 0
    try:
        while (
            rclpy.ok()
            and node.state != FlightState.FINISHED
            and node.state != FlightState.ERROR
        ):
            rclpy.spin_once(node, timeout_sec=0.1)
        if node.state == FlightState.ERROR:
            code = 1
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(code)


if __name__ == "__main__":
    main()
