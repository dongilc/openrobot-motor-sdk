"""Shared command-line options for the examples.

    python 00_scan.py --sim                      # no hardware
    python 00_scan.py -i slcan -c COM5           # Windows
    python 00_scan.py -i slcan -c /dev/ttyACM0   # Linux
    python 00_scan.py -i pcan                    # first PCAN adapter (PCAN_USBBUS1)
    python 00_scan.py -i socketcan -c can0       # Linux
"""
import argparse
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from openrobot_motor import MotorBus  # noqa: E402


def parser(description: str, ids: bool = False) -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description=description)
    p.add_argument("-i", "--interface", default="slcan", choices=["slcan", "pcan", "socketcan"],
                   help="CAN adapter type (default slcan)")
    p.add_argument("-c", "--channel", help="COM5, /dev/ttyACM0, can0, ... (pcan: default PCAN_USBBUS1)")
    p.add_argument("-b", "--bitrate", type=int, default=1_000_000)
    p.add_argument("--sim", action="store_true", help="use the simulator, no hardware")
    if ids:
        p.add_argument("--ids", type=int, nargs="+", default=[1], help="motor IDs")
    else:
        p.add_argument("--id", type=int, default=1, help="motor ID (default 1)")
    return p


def open_bus(args) -> MotorBus:
    if args.sim:
        ids = getattr(args, "ids", None) or [args.id]
        return MotorBus.open("sim", sim_ids=ids)
    return MotorBus.open(args.interface, args.channel, args.bitrate)


def confirm_motion(args, what: str):
    """Motion examples ask before moving a real motor."""
    if args.sim:
        return
    print("This example will %s." % what)
    print("Make sure the shaft is free to turn and nothing is attached that can be hurt.")
    if input("Type yes to continue: ").strip().lower() != "yes":
        sys.exit("cancelled")
