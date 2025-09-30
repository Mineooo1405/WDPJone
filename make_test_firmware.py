#!/usr/bin/env python3

import os
import sys
import time
from pathlib import Path

"""
Generate a small dummy .bin file to test BE–FE–RasPi firmware streaming.

NOTE:
- This .bin is ONLY for testing the upload/streaming pipeline.
- It is NOT a real ESP32 firmware and must NOT be flashed to hardware.
"""

def generate_dummy_bin(out_path: Path, size_bytes: int = 256 * 1024) -> None:
    out_path.parent.mkdir(parents=True, exist_ok=True)

    # Simple header: 0xE9 (ESP image magic) + a small signature block for human inspection
    header = bytearray()
    header.append(0xE9)  # ESP32 image magic
    header += b"DUMMYTEST"  # readable tag
    header += int(time.time()).to_bytes(4, 'little')  # timestamp
    header += b"STREAM_ONLY"  # tag to indicate not real firmware

    # Build content: header + padding pattern to target size
    content = bytearray()
    content += header

    # Repeat a simple pattern to fill the rest of the file
    pattern = (b"RasPi-Gateway Firmware Stream Test\n" * 16)
    while len(content) < size_bytes:
        remaining = size_bytes - len(content)
        chunk = pattern[:remaining]
        content += chunk

    with open(out_path, 'wb') as f:
        f.write(content)

    print(f"Created dummy firmware: {out_path} ({len(content)} bytes)")


def main():
    # Default output
    default_out = Path('temp_firmware') / 'test_firmware_dummy.bin'
    out_path = Path(sys.argv[1]) if len(sys.argv) > 1 else default_out
    try:
        size_arg = int(sys.argv[2]) if len(sys.argv) > 2 else 256 * 1024
    except ValueError:
        size_arg = 256 * 1024
    generate_dummy_bin(out_path, size_arg)


if __name__ == '__main__':
    main()


