#!/usr/bin/env python3
"""Copies the C++ core from the sixeyes-host repository into sixeyes_hardware, so this repo builds on its own.

    python scripts/sync-host-core.py            copy (overwrites)
    python scripts/sync-host-core.py --check    exit 1 if the copies are out of date

Needs sixeyes-host next to this repository. The sources of truth are in sixeyes-host: never edit the copies.
"""
import shutil
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
HOST = ROOT.parent / "sixeyes-host"
OUT = ROOT / "sixeyes_hardware"

# (file in sixeyes-host, where it goes here)
FILES = [
    ("core/include/sixeyes/codec.hpp", "include/sixeyes/codec.hpp"),
    ("core/include/sixeyes/serial.hpp", "include/sixeyes/serial.hpp"),
    ("core/include/sixeyes/stream.hpp", "include/sixeyes/stream.hpp"),
    ("core/include/sixeyes/session.hpp", "include/sixeyes/session.hpp"),
    ("protocol/generated/sixeyes_protocol.h", "include/sixeyes_protocol.h"),
    ("core/src/codec.cpp", "src/host/codec.cpp"),
    ("core/src/serial.cpp", "src/host/serial.cpp"),
    ("core/src/session.cpp", "src/host/session.cpp"),
]


def main():
    check = "--check" in sys.argv
    stale = 0
    for src_name, dst_name in FILES:
        src, dst = HOST / src_name, OUT / dst_name
        if not src.exists():
            print(f"Missing {src_name}: put sixeyes-host next to this repository.")
            return 2
        want = src.read_bytes().replace(b"\r\n", b"\n")
        have = dst.read_bytes().replace(b"\r\n", b"\n") if dst.exists() else None
        if want == have:
            continue
        stale += 1
        if check:
            print(f"Out of date: sixeyes_hardware/{dst_name}")
        else:
            dst.parent.mkdir(parents=True, exist_ok=True)
            dst.write_bytes(want)
            print(f"Copied {src_name} -> sixeyes_hardware/{dst_name}")
    if check and stale:
        return 1
    print("Up to date." if not stale or check else "Done.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
