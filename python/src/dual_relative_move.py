#!/usr/bin/env python3
"""Safely ramp two RS-05 motors by relative angles, then disable torque."""

from __future__ import annotations

import argparse
import math
import os
import signal
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from robstride_dynamics import Motor, ParameterType, RobstrideBus


MOTOR_MODEL = "rs-05"
CONTROL_HZ = 50.0
POSITION_LIMIT_RAD = 4.0 * math.pi


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Move RS-05 motors relative to their measured starting positions."
    )
    parser.add_argument("--channel", default="/dev/ttyUSB0")
    parser.add_argument(
        "--interface",
        choices=("robstride_serial", "socketcan"),
        default="robstride_serial",
        help="python-can backend used by the CAN adapter",
    )
    parser.add_argument("--motor-1-id", type=int, default=1)
    parser.add_argument("--motor-1-deg", type=float, default=20.0)
    parser.add_argument("--motor-2-id", type=int, default=2)
    parser.add_argument("--motor-2-deg", type=float, default=100.0)
    parser.add_argument(
        "--speed-deg-s",
        type=float,
        default=20.0,
        help="Maximum commanded ramp speed for either motor",
    )
    parser.add_argument(
        "--kp",
        type=float,
        default=5.0,
        help="MIT position gain",
    )
    parser.add_argument(
        "--kd",
        type=float,
        default=0.2,
        help="MIT damping gain",
    )
    parser.add_argument(
        "--torque-limit-nm",
        type=float,
        default=2.0,
        help="Motor torque limit",
    )
    parser.add_argument(
        "--hold-seconds",
        type=float,
        default=1.0,
        help="Time to hold the final targets before disabling torque",
    )
    parser.add_argument(
        "--probe-only",
        action="store_true",
        help="Verify both motor IDs without enabling torque or moving",
    )
    parser.add_argument(
        "--execute",
        action="store_true",
        help="Required acknowledgement that the mechanism is clear and supported",
    )
    return parser.parse_args()


def make_bus(args: argparse.Namespace) -> RobstrideBus:
    motors = {
        "motor_1": Motor(id=args.motor_1_id, model=MOTOR_MODEL),
        "motor_2": Motor(id=args.motor_2_id, model=MOTOR_MODEL),
    }
    calibration = {
        "motor_1": {"direction": 1, "homing_offset": 0.0},
        "motor_2": {"direction": 1, "homing_offset": 0.0},
    }
    interface_kwargs = {}
    if args.interface == "robstride_serial":
        interface_kwargs = {
            "baudrate": 921_600,
            "timeout": 0.1,
        }
    return RobstrideBus(
        args.channel,
        motors,
        calibration,
        bitrate=1_000_000,
        interface=args.interface,
        interface_kwargs=interface_kwargs,
    )


def require_motor(bus: RobstrideBus, name: str) -> None:
    motor_id = bus.motors[name].id
    response = bus.read_id(name, timeout=0.5)
    if response is None:
        raise RuntimeError(f"Motor ID {motor_id} did not answer the identity probe")
    print(f"Motor ID {motor_id} responded")


def command_and_read(
    bus: RobstrideBus, name: str, position: float, kp: float, kd: float
) -> tuple[float, float, float, float]:
    bus.write_operation_frame(name, position, kp, kd, 0.0, 0.0)
    return bus.read_operation_frame(name)


def main() -> int:
    args = parse_args()
    if not args.execute and not args.probe_only:
        print("Preflight only: no motor commands were sent.")
        print("Re-run with --execute after clearing and supporting the mechanism.")
        return 2
    if args.motor_1_id == args.motor_2_id:
        raise ValueError("Motor IDs must be different")
    if args.speed_deg_s <= 0:
        raise ValueError("--speed-deg-s must be positive")
    if not 0 <= args.kp <= 500:
        raise ValueError("--kp must be between 0 and 500")
    if not 0 <= args.kd <= 5:
        raise ValueError("--kd must be between 0 and 5")
    if not 0 < args.torque_limit_nm <= 17:
        raise ValueError("--torque-limit-nm must be between 0 and 17")

    bus = make_bus(args)
    enabled: list[str] = []
    stop_requested = False

    def request_stop(_signum: int, _frame: object) -> None:
        nonlocal stop_requested
        stop_requested = True

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)

    try:
        bus.connect(handshake=False)
        require_motor(bus, "motor_1")
        require_motor(bus, "motor_2")
        if args.probe_only:
            print("Probe complete; no motors were enabled")
            return 0

        starts: dict[str, float] = {}
        for name in ("motor_1", "motor_2"):
            # Configure while disabled, then capture the enable status as the
            # starting shaft position. Holding that exact position avoids a
            # jump to an assumed zero.
            bus.write(name, ParameterType.MODE, 0)
            bus.write(name, ParameterType.TORQUE_LIMIT, args.torque_limit_nm)
            starts[name], velocity, torque, temperature = bus.enable(name)
            enabled.append(name)
            command_and_read(bus, name, starts[name], args.kp, args.kd)
            print(
                f"{name}: start={math.degrees(starts[name]):.2f} deg, "
                f"velocity={velocity:.3f} rad/s, torque={torque:.3f} Nm, "
                f"temperature={temperature:.1f} C"
            )

        deltas = {
            "motor_1": math.radians(args.motor_1_deg),
            "motor_2": math.radians(args.motor_2_deg),
        }
        targets = {name: starts[name] + deltas[name] for name in starts}
        for name, target in targets.items():
            if not -POSITION_LIMIT_RAD <= target <= POSITION_LIMIT_RAD:
                raise RuntimeError(
                    f"{name} target {math.degrees(target):.1f} deg exceeds "
                    "the MIT frame range of +/-720 deg"
                )

        duration = max(abs(value) for value in deltas.values()) / math.radians(
            args.speed_deg_s
        )
        steps = max(1, math.ceil(duration * CONTROL_HZ))
        period = 1.0 / CONTROL_HZ
        print(
            f"Ramping motor {args.motor_1_id} by {args.motor_1_deg:+.1f} deg "
            f"and motor {args.motor_2_id} by {args.motor_2_deg:+.1f} deg "
            f"over {duration:.2f} s"
        )

        for step in range(1, steps + 1):
            if stop_requested:
                raise KeyboardInterrupt
            fraction = step / steps
            cycle_start = time.monotonic()
            for name in ("motor_1", "motor_2"):
                target = starts[name] + deltas[name] * fraction
                command_and_read(bus, name, target, args.kp, args.kd)
            remaining = period - (time.monotonic() - cycle_start)
            if remaining > 0:
                time.sleep(remaining)

        final_status = {}
        for name in ("motor_1", "motor_2"):
            final_status[name] = command_and_read(
                bus, name, targets[name], args.kp, args.kd
            )

        hold_until = time.monotonic() + max(0.0, args.hold_seconds)
        while time.monotonic() < hold_until and not stop_requested:
            cycle_start = time.monotonic()
            for name in ("motor_1", "motor_2"):
                final_status[name] = command_and_read(
                    bus, name, targets[name], args.kp, args.kd
                )
            remaining = period - (time.monotonic() - cycle_start)
            if remaining > 0:
                time.sleep(remaining)

        for name, status in final_status.items():
            position, velocity, torque, temperature = status
            print(
                f"{name}: final={math.degrees(position):.2f} deg, "
                f"velocity={velocity:.3f} rad/s, torque={torque:.3f} Nm, "
                f"temperature={temperature:.1f} C"
            )
        return 0
    except KeyboardInterrupt:
        print("Stop requested")
        return 130
    except Exception as exc:
        print(f"Aborted: {exc}", file=sys.stderr)
        return 1
    finally:
        for name in reversed(enabled):
            try:
                bus.disable(name)
                print(f"{name}: torque disabled")
            except Exception as exc:
                print(f"Warning: could not disable {name}: {exc}", file=sys.stderr)
        if bus.is_connected:
            try:
                bus.disconnect(disable_torque=False)
            except Exception as exc:
                print(f"Warning: could not close CAN bus: {exc}", file=sys.stderr)


if __name__ == "__main__":
    raise SystemExit(main())
