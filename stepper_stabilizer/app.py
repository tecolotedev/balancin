"""Command-line application for the Raspberry Pi stabilizer."""

import argparse
from contextlib import contextmanager
import signal
import sys
import threading
import time
from typing import Iterator, Optional, Sequence

from .controller import PidController
from .hardware import Motors, MpuSensor


CONTROL_LOOP_DELAY_SECONDS = 0.050
MAX_CONSECUTIVE_READ_ERRORS = 10


def stabilize() -> None:
    with _stop_event() as stop_requested:
        with Motors() as motors:
            with MpuSensor() as sensor:
                controller = PidController()
                previous_update = time.monotonic()
                consecutive_read_errors = 0

                print(
                    f"{sensor.model.value} ready at 0x{sensor.address:02X}; "
                    "starting stabilization (Ctrl-C to stop)"
                )

                while not stop_requested.is_set():
                    try:
                        data = sensor.read()
                    except OSError as error:
                        consecutive_read_errors += 1
                        print(
                            "IMU read failed "
                            f"({consecutive_read_errors}/"
                            f"{MAX_CONSECUTIVE_READ_ERRORS}): {error}",
                            file=sys.stderr,
                        )
                        if (
                            consecutive_read_errors
                            >= MAX_CONSECUTIVE_READ_ERRORS
                        ):
                            raise RuntimeError(
                                "stopping after 10 consecutive IMU "
                                "read failures"
                            ) from error
                    else:
                        consecutive_read_errors = 0
                        now = time.monotonic()
                        correction = controller.update(
                            data.acceleration_x_g,
                            now - previous_update,
                        )
                        previous_update = now
                        if correction is not None:
                            motors.rotate_both(correction)

                    stop_requested.wait(CONTROL_LOOP_DELAY_SECONDS)

    print("stopped; both motor drivers are disabled")


def diagnose_imu() -> None:
    print("Probing I2C bus 1 for an MPU6050/MPU6500...")
    with MpuSensor() as sensor:
        print(
            f"found {sensor.model.value} at address 0x{sensor.address:02X}"
        )
        data = sensor.read()
        print(
            "sample: "
            f"accel=({data.acceleration_x_g:.3f}, "
            f"{data.acceleration_y_g:.3f}, "
            f"{data.acceleration_z_g:.3f}) g, "
            f"temperature={data.temperature_c:.1f} °C"
        )


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="stepper-stabilizer",
        description=(
            "Control two DRV8825 stepper drivers from an "
            "MPU6050/MPU6500 on Raspberry Pi 5."
        ),
    )
    parser.add_argument(
        "--diagnose",
        action="store_true",
        help="detect the IMU and print one sample without claiming motor GPIO",
    )
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    arguments = build_parser().parse_args(argv)
    try:
        if arguments.diagnose:
            diagnose_imu()
        else:
            stabilize()
    except KeyboardInterrupt:
        # This is a fallback for environments that do not permit installing
        # the signal handler below.
        print("\nstopped by user", file=sys.stderr)
        return 130
    except Exception as error:
        print(f"error: {error}", file=sys.stderr)
        return 1
    return 0


@contextmanager
def _stop_event() -> Iterator[threading.Event]:
    stop_requested = threading.Event()
    previous_handlers = {}

    def request_stop(_signal_number: int, _frame: object) -> None:
        stop_requested.set()

    for signal_number in (signal.SIGINT, signal.SIGTERM):
        previous_handlers[signal_number] = signal.getsignal(signal_number)
        signal.signal(signal_number, request_stop)

    try:
        yield stop_requested
    finally:
        for signal_number, previous_handler in previous_handlers.items():
            signal.signal(signal_number, previous_handler)

