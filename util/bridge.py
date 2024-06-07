#!/usr/bin/env python

import struct
from socket import (AF_INET, MSG_DONTWAIT, SO_REUSEADDR, SO_SNDBUF,
                    SOCK_STREAM, SOL_SOCKET, socket)

import click
import serial

COMMAND_NAMES = [
    "reset",
    "tune_freq",
    "sample_rate",
    "manual_gain",
    "gain",
    "ppm_offset",
    "if_gain",
    "test_mode",
    "agc",
    "direct_sampling",
    "offset_tuning",
    "11",
    "12",
    "gain_index",
    "bias_tee",
]


def describe(cmd: int, arg: int):
    try:
        print("->", COMMAND_NAMES[cmd], arg)
    except IndexError:
        print("->", cmd, arg)


@click.command()
@click.option("-f", "--frequency", default=88200000, help="Frequency to tune to")
def bridge(frequency):
    sock = socket(AF_INET, SOCK_STREAM)
    sock.setsockopt(SOL_SOCKET, SO_REUSEADDR, 1)
    sock.setsockopt(SOL_SOCKET, SO_SNDBUF, 1024 * 100)

    print("Posing as rtl_tcp at tcp://127.0.0.1:1234")
    sock.bind(("127.0.0.1", 1234))
    sock.listen(3)

    while True:
        peer, addr = sock.accept()
        print("Client connected:", addr)

        with serial.Serial("/dev/ttyACM0", baudrate=10_000_000, timeout=0.1) as fp:
            print(f"Starting RX @ {frequency}")

            # Remove any leftovers.
            while fp.read(64):
                fp.write(b"\x00")
                fp.flush()

            fp.write(struct.pack(">BL", 1, int(frequency)))
            fp.flush()

            print("Begin")

            try:
                cmd = b""

                while True:
                    try:
                        cmd += peer.recv(1, MSG_DONTWAIT)
                    except BlockingIOError:
                        pass

                    while len(cmd) >= 5:
                        fp.write(cmd[:5])
                        info = struct.unpack(">BL", cmd[:5])
                        describe(*info)
                        cmd = cmd[5:]

                    data = fp.read(64)
                    if data:
                        peer.send(data)

            except ConnectionError:
                pass

            except KeyboardInterrupt:
                pass

            finally:
                fp.write(b"\x00")
                fp.flush()
                print("Bye.")


if __name__ == "__main__":
    bridge()
