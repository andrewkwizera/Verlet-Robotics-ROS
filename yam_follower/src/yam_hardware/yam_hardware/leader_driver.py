#!/usr/bin/env python3

import time
import numpy as np

from i2rt.robots.get_robot import get_yam_robot
from i2rt.robots.utils import ArmType, GripperType


CAN_CHANNEL = "can0"
PUBLISH_RATE_HZ = 50.0


def main():
    robot = None

    try:
        print("Initializing YAM Leader on can0...")

        robot = get_yam_robot(
            channel=CAN_CHANNEL,
            arm_type=ArmType.YAM,
            gripper_type=GripperType.LINEAR_4310,
            zero_gravity_mode=True,
        )

        print("YAM Leader initialized.")
        print("Reading joint positions at 50 Hz.")
        print("Move the leader arm manually.")
        print("Press Ctrl+C to stop.\n")

        period = 1.0 / PUBLISH_RATE_HZ
        next_tick = time.monotonic()
        count = 0

        while True:
            positions = np.asarray(
                robot.get_joint_pos(),
                dtype=float
            )

            if positions.shape[0] < 7:
                raise RuntimeError(
                    "Expected six joints and one gripper position"
                )

            joints = positions[:6]

            if not np.all(np.isfinite(joints)):
                raise RuntimeError(
                    "Received invalid joint positions"
                )

            count += 1

            # Display positions approximately once per second.
            if count % 50 == 0:
                print(
                    "Joint positions (rad):",
                    np.round(joints, 4)
                )

            next_tick += period
            time.sleep(max(0.0, next_tick - time.monotonic()))

    except KeyboardInterrupt:
        print("\nStopping leader reader...")

    finally:
        if robot is not None:
            input(
                "\nPhysically support the Leader arm, "
                "then press Enter to close the CAN connection..."
            )
            robot.close()

        print("Leader connection closed.")


if __name__ == "__main__":
    main()