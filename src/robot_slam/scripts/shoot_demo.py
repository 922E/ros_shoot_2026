#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import argparse
import serial
import sys
import time

FIRE_COMMAND = b'\x55\x01\x12\x00\x00\x00\x01\x69'
STOP_COMMAND = b'\x55\x01\x11\x00\x00\x00\x01\x68'


def _write_command(ser, name, command):
    written = ser.write(command)
    ser.flush()
    print("%s: wrote %d/%d bytes" % (name, written, len(command)))
    if written != len(command):
        raise IOError("%s command was not fully written" % name)


def main():
    parser = argparse.ArgumentParser(description="Send one shooter test pulse")
    parser.add_argument("--port", default="/dev/shoot")
    parser.add_argument("--settle", type=float, default=1.0,
                        help="seconds to wait after opening the serial port")
    parser.add_argument("--pulse", type=float, default=0.10,
                        help="seconds between fire and stop commands")
    args = parser.parse_args()

    print("Opening %s at 9600 8N1" % args.port)
    ser = serial.Serial(port=args.port, baudrate=9600, parity="N",
                        bytesize=8, stopbits=1, timeout=1)
    try:
        print("Serial opened. Waiting %.2fs before the test pulse." %
              args.settle)
        time.sleep(args.settle)
        _write_command(ser, "fire", FIRE_COMMAND)
        time.sleep(args.pulse)
        _write_command(ser, "stop", STOP_COMMAND)
        print("Test pulse sent.")
    finally:
        ser.close()
        print("Serial closed.")


if __name__ == '__main__':
    try:
        main()
    except Exception as exc:
        print("Shoot test failed: %s" % exc)
        sys.exit(1)
