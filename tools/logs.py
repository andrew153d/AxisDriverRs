#!/usr/bin/env python3
"""Fetch the firmware's log over USB.

The firmware keeps its most recent log lines in a buffer. This reads them out
with the "Get Log" message (0x010A) and prints them. Reading empties the
buffer, so each line is shown once.

    tools/logs.py [--follow] [PORT]

Requires pyserial.
"""

import argparse
import sys
import time

from axis import DEFAULT_PORT, GET_LOG, Axis


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("port", nargs="?", default=DEFAULT_PORT)
    parser.add_argument(
        "-f", "--follow", action="store_true", help="keep polling for new lines"
    )
    args = parser.parse_args()

    with Axis(args.port) as axis:
        try:
            while True:
                chunk = axis.get(GET_LOG)
                if chunk:
                    sys.stdout.write(chunk.decode("utf-8", errors="replace"))
                    sys.stdout.flush()
                elif args.follow:
                    time.sleep(0.2)
                else:
                    return 0
        except KeyboardInterrupt:
            return 0


if __name__ == "__main__":
    sys.exit(main())
