#!/usr/bin/env python3
"""Print the current PWM channel mapping.

Loads rov_config.CHANNEL_MAP (physical channels 1..16) and prints:
  - thruster name -> physical channel
  - horizontal / vertical groups
  - auxiliary outputs (e.g. lights)

Run:
  python -m tools.print_channel_map
"""

from __future__ import annotations

import rov_config as cfg
from motion.channel_map import ChannelMap


def main() -> None:
    """Load ``rov_config`` and print the validated physical channel map."""

    cm = ChannelMap.from_config(cfg)

    print("=== TritonOS channel map (physical) ===")
    print("\nThrusters:")
    for name in sorted(cm.thrusters.keys()):
        print(f"  {name:4s} -> PWM {cm.thrusters[name]}")

    print("\nHorizontal thrusters:", cm.horizontal_thrusters)
    print("Vertical thrusters:  ", cm.vertical_thrusters)

    if cm.aux:
        print("\nAux outputs:")
        for name in sorted(cm.aux.keys()):
            print(f"  {name:8s} -> PWM {cm.aux[name]}")
    else:
        print("\nAux outputs: (none)")

    print("\nMotor PWM channels:", cm.motor_channels)
    if cm.lights_channel is not None:
        print("Lights PWM channel:", cm.lights_channel)
    print("=== end ===")


if __name__ == "__main__":
    main()
