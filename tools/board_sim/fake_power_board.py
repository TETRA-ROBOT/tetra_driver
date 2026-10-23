"""Simulated TETRA power board on a pseudo-terminal, speaking the same packet protocol as power_module.c."""
import os
import threading
import time

STX, ETX = 0x02, 0x03


def lrc(data):
    v = 0
    for b in data:
        v ^= b
    return v


class FakePowerBoard:
    def __init__(self):
        self.master, slave = os.openpty()
        self.path = os.ttyname(slave)
        self._slave = slave
        self.responding = True
        self.log = []
        threading.Thread(target=self._run, daemon=True).start()

    def _binary(self, data):
        body = bytes([STX]) + len(data).to_bytes(2, 'big') + data + bytes([ETX])
        os.write(self.master, body + bytes([lrc(data)]))

    def _run(self):
        while True:
            b = os.read(self.master, 1)
            if b[0] != STX:
                continue
            payload = b''
            while True:
                c = os.read(self.master, 1)
                if c[0] == ETX:
                    break
                payload += c
            os.read(self.master, 1)  # LRC
            cmd = payload[:2].decode('latin1')
            self.log.append((time.monotonic(), cmd))
            if not self.responding:
                continue
            if cmd == 'PQ':
                words = [85, 252, 15, 1, 0, 0, 500, 500, 500, 500]
                self._binary(b''.join(w.to_bytes(2, 'big') for w in words))
            elif cmd == 'PD':
                self._binary(b''.join(w.to_bytes(2, 'big') for w in [252] + [0] * 11))
            else:
                frame = bytes([STX, 0x30, ETX])
                os.write(self.master, frame + bytes([lrc(frame[1:])]))
