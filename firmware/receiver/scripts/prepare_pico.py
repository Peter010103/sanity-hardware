#!/usr/bin/env python3
"""Fetch the pinned Pico source and apply the documented GCC 16 compatibility fixes."""

import argparse
from pathlib import Path
import subprocess

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("destination", type=Path)
args = parser.parse_args()
root = args.destination.resolve()
if not root.exists():
    subprocess.run(
        [
            "git",
            "clone",
            "--depth",
            "1",
            "--branch",
            "1.10.1",
            "https://github.com/eclipse-zenoh/zenoh-pico.git",
            str(root),
        ],
        check=True,
    )
revision = subprocess.check_output(
    ["git", "rev-parse", "HEAD"], cwd=root, text=True
).strip()
if revision != "e1ab223a28aaebb5dec1e70d98eab152332f777a":
    raise SystemExit("Destination must contain the pinned Zenoh-Pico 1.10.1 revision")
patch = Path(__file__).resolve().parents[1] / "patches/zenoh-pico-1.10.1-gcc16.patch"
if subprocess.run(
    ["git", "apply", "--reverse", "--check", str(patch)], cwd=root, capture_output=True
).returncode:
    subprocess.run(["git", "apply", "--check", str(patch)], cwd=root, check=True)
    subprocess.run(["git", "apply", str(patch)], cwd=root, check=True)
patch = Path(__file__).resolve().parents[1] / "patches/zenoh-pico-1.10.1-transport-lifetime.patch"
if subprocess.run(["git", "apply", "--reverse", "--check", str(patch)], cwd=root, capture_output=True).returncode:
    subprocess.run(["git", "apply", "--check", str(patch)], cwd=root, check=True)
    subprocess.run(["git", "apply", str(patch)], cwd=root, check=True)
print(root)
