"""Copy the shared C++ core into the Arduino library.

An Arduino library must be self-contained, so protocol.hpp and bus.hpp are
copied from cpp/include instead of referenced. Run after changing them:

    python sync_core.py
"""
import os
import shutil

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.join(HERE, "..", "cpp", "include", "openrobot_motor")
DST = os.path.join(HERE, "OpenRobotMotor", "src", "openrobot_motor")

os.makedirs(DST, exist_ok=True)
for name in ("protocol.hpp", "bus.hpp"):
    shutil.copyfile(os.path.join(SRC, name), os.path.join(DST, name))
    print("copied", name)
