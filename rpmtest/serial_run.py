"""Send commands to the rpmtest firmware and print/log everything it replies.

    python serial_run.py STEP [STEP ...]

Each STEP is either a number (wait that many seconds while printing output)
or a command line sent to the board, e.g.:

    python serial_run.py 1 "cpr 700" sweep 25
"""
import argparse
import sys
import time

import serial


def pump(ser, seconds, log):
    end = time.time() + seconds
    while time.time() < end:
        data = ser.read(ser.in_waiting or 1)
        if data:
            text = data.decode(errors="replace")
            sys.stdout.write(text)
            sys.stdout.flush()
            log.write(text)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="COM5")
    ap.add_argument("--log", default="serial_log.txt")
    ap.add_argument("steps", nargs="+")
    args = ap.parse_args()

    ser = serial.Serial()
    ser.port = args.port
    ser.baudrate = 115200
    ser.timeout = 0.05
    ser.dtr = False  # don't reset the board when the port opens
    ser.rts = False
    ser.open()

    with open(args.log, "a", encoding="utf-8") as log:
        for step in args.steps:
            try:
                pump(ser, float(step), log)
            except ValueError:
                line = f"\n>>> {step}\n"
                sys.stdout.write(line)
                log.write(line)
                ser.write((step + "\n").encode())
                pump(ser, 0.3, log)
        ser.write(b"s\n")  # leave the motors braked and the output quiet
        pump(ser, 0.3, log)
    ser.close()


if __name__ == "__main__":
    main()
