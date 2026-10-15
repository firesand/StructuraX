#!/usr/bin/env python3
"""Inspect ELF GLIBC requirements without modifying symbol versions.

Build on the oldest supported distribution/sysroot to lower the requirement.
Editing version tags does not make newer ABI calls compatible with older libc.
"""
import argparse
from pathlib import Path
import re
import subprocess


def check_glibc_requirement(elf_path: Path, maximum: str = "2.35") -> str:
    result = subprocess.run(
        ["readelf", "--version-info", "--wide", str(elf_path)],
        check=True, capture_output=True, text=True,
    )
    versions = {tuple(map(int, v.split("."))) for v in re.findall(r"GLIBC_(\d+(?:\.\d+)+)", result.stdout)}
    if not versions:
        raise RuntimeError(f"No GLIBC requirement found in {elf_path}; expected a dynamically linked Linux ELF")
    required = max(versions)
    floor = ".".join(map(str, required))
    if required > tuple(map(int, maximum.split("."))):
        raise RuntimeError(
            f"{elf_path} requires GLIBC {floor}, above package baseline {maximum}. "
            "Rebuild on Ubuntu 22.04 or an equivalent baseline sysroot. "
            "The binary has not been modified."
        )
    print(f"[elf-check] {elf_path}: GLIBC {floor} <= {maximum}; symbol versions unchanged")
    return floor


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("binary", type=Path)
    parser.add_argument("--maximum", default="2.35")
    args = parser.parse_args()
    check_glibc_requirement(args.binary, args.maximum)
