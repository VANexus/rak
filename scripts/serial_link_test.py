#!/usr/bin/env python3
"""Refuse to run the retired physical-port CRC-8 protocol utility."""

import sys


def main():
    print(
        "This legacy CRC-8 utility is disabled: it can transmit motion commands "
        "using a protocol incompatible with UART v1. Use scripts/serial_100hz.py "
        "for the PTY-only DISARMED test, or rak_base_driver after firmware review.",
        file=sys.stderr,
    )
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
