#!/usr/bin/env python

import time
from codecs import encode
from socket import AF_INET, SO_SNDBUF, SOCK_STREAM, SOL_SOCKET, socket

import click
import serial


@click.command()
@click.option("-f", "--frequency", default=40680000, help="Frequency to tune to")
@click.option("--rx", default=10, help="Receive pin")
@click.option("--bias", default=11, help="Bias pin")
def bridge(frequency, rx, bias):
    sock = socket(AF_INET, SOCK_STREAM)
    sock.setsockopt(SOL_SOCKET, SO_SNDBUF, 1024 * 100)

    with serial.Serial("/dev/ttyACM0", baudrate=10_000_000) as fp:
        print("Resetting...")
        fp.write(b"\r\n")

        time.sleep(0.1)

        while fp.in_waiting:
            fp.read(fp.in_waiting)
            time.sleep(0.1)

        print("Connecting to localhost:1234...")
        sock.connect(("localhost", 1234))

        print(f"Starting RX {rx}/{bias} at {frequency}...")
        fp.write(f"brx {rx} {bias} {frequency}\r\n".encode("ascii"))
        fp.read_until(b"$")

        try:
            while True:
                bstr = fp.read(64)
                assert len(bstr) == 64

                try:
                    assert len(bstr) == sock.send(bstr)
                except ConnectionRefusedError:
                    pass

        except ConnectionError:
            pass

        except KeyboardInterrupt:
            pass

        finally:
            fp.write(b"\r\n")
            print("Bye.")


if __name__ == "__main__":
    bridge()
