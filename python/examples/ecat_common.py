"""Shared command-line options for the EtherCAT examples.

    python ecat_00_scan.py --list                 # network adapters
    python ecat_00_scan.py                        # first adapter with OpenRobot slaves
    python ecat_00_scan.py -a "Ethernet"          # adapter by part of its description
"""
import argparse
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from openrobot_motor.ethercat import EcatBus, list_adapters  # noqa: E402


def parser(description: str) -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description=description)
    p.add_argument("-a", "--adapter", help="adapter name or part of its description (default: search)")
    p.add_argument("--period", type=float, default=2.0, help="PDO cycle, ms (default 2)")
    p.add_argument("--slave", type=int, default=0, help="slave position on the bus, from 0")
    p.add_argument("--list", action="store_true", help="list network adapters and exit")
    return p


def open_bus(args) -> EcatBus:
    if args.list:
        for name, desc in list_adapters():
            print("%-60s %s" % (name, desc))
        sys.exit(0)
    return EcatBus.open(args.adapter, args.period / 1000.0)


def confirm_motion(what: str):
    print("This example will %s." % what)
    print("Make sure the shaft is free to turn and nothing is attached that can be hurt.")
    if input("Type yes to continue: ").strip().lower() != "yes":
        sys.exit("cancelled")
