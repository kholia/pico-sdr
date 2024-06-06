#!/usr/bin/env python

import binascii
import struct
from socket import (AF_INET, MSG_DONTWAIT, SO_REUSEADDR, SO_SNDBUF,
                    SOCK_STREAM, SOL_SOCKET, socket)

import click
import serial


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
            fp.write(struct.pack(">BBL", 0, 1, int(frequency)))
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
                        print("->", hex(info[0]), info[1])
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
                print("Bye.")


if __name__ == "__main__":
    bridge()
