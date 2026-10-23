"""Simulated TETRA drive board on a pseudo-terminal, speaking the same packet protocol as drive_module.c."""
import math
import os
import threading
import time

STX, ETX = 0x02, 0x03
FLAG_OK = 0x30


def lrc(data):
    value = 0
    for b in data:
        value ^= b
    return value


class FakeDriveBoard:
    def __init__(self, wheel_distance=0.4330):
        self.master, slave = os.openpty()
        self.path = os.ttyname(slave)
        self._slave = slave  # keep open so the device stays alive
        self.wheel_distance = wheel_distance
        self.x_mm = 0.0
        self.y_mm = 0.0
        self.theta_rad = 0.0
        self.left_mm_s = 0
        self.right_mm_s = 0
        self.servo = False
        self.left_error = ord('0')
        self.right_error = ord('0')
        self.responding = True
        self.log = []  # (time, command, payload)
        self.lock = threading.Lock()
        self._last = time.monotonic()
        self._stop = False
        threading.Thread(target=self._run, daemon=True).start()

    # --- helpers -------------------------------------------------------------
    def commands(self, name):
        with self.lock:
            return [(t, p) for t, c, p in self.log if c == name]

    def _integrate(self):
        now = time.monotonic()
        dt = now - self._last
        self._last = now
        if not self.servo:
            return
        v = (self.left_mm_s + self.right_mm_s) / 2.0
        w = (self.right_mm_s - self.left_mm_s) / 1000.0 / self.wheel_distance
        self.x_mm += v * math.cos(self.theta_rad) * dt
        self.y_mm += v * math.sin(self.theta_rad) * dt
        self.theta_rad = (self.theta_rad + w * dt) % (2 * math.pi)

    def _send(self, body):
        frame = bytes([STX]) + body + bytes([ETX])
        frame += bytes([lrc(frame[1:])])
        os.write(self.master, frame)

    def _read_exact(self, n):
        data = b''
        while len(data) < n:
            data += os.read(self.master, n - len(data))
        return data

    # --- protocol ------------------------------------------------------------
    def _run(self):
        while not self._stop:
            b = os.read(self.master, 1)
            if b[0] != STX:
                continue
            cmd = self._read_exact(2).decode('latin1')
            if cmd == 'BV':
                payload = self._read_exact(6)[:4]  # 4 data bytes, ETX, LRC
            else:
                payload = b''
                while True:
                    c = self._read_exact(1)
                    if c[0] == ETX:
                        break
                    payload += c
                self._read_exact(1)  # LRC
            with self.lock:
                self.log.append((time.monotonic(), cmd, payload))
                self._integrate()
                if self.responding:
                    self._handle(cmd, payload)

    def _handle(self, cmd, payload):
        if cmd == 'BV':
            def speed(hi, lo):
                mag = ((hi & 0x7F) << 8) | lo
                return -mag if hi & 0x80 else mag
            self.left_mm_s = speed(payload[0], payload[1])
            self.right_mm_s = speed(payload[2], payload[3])

            def signed32(value):
                mag = int(abs(value)) & 0x7FFFFFFF
                raw = mag | (0x80000000 if value < 0 else 0)
                return raw.to_bytes(4, 'big')
            theta = int(round(math.degrees(self.theta_rad) * 10)) % 3600
            body = bytes([FLAG_OK]) + signed32(self.x_mm) + signed32(self.y_mm)
            body += theta.to_bytes(2, 'big') + b'\x00\x00\x00\x00' + bytes([0x00, 0x00])
            frame = bytes([STX]) + body + bytes([ETX])
            frame += bytes([lrc(frame[1:])])
            assert len(frame) == 20
            os.write(self.master, frame)
        elif cmd == 'AA':
            # [3] left error, [5] right error, [6],[7] bumper nibbles (servo bit 0x02 = off), [8] emg
            servo_bit = 0x00 if self.servo else 0x02
            body = bytes([FLAG_OK, ord('0'), self.left_error, ord('0'), self.right_error, 0x00, servo_bit, 0x00])
            self._send(body)
        elif cmd == 'AC' and payload == b'1':
            theta = int(round(math.degrees(self.theta_rad) * 10)) % 3600
            body = bytes([FLAG_OK]) + b'%10d%10d%10d' % (int(self.x_mm), int(self.y_mm), theta)
            self._send(body)
        elif cmd == 'XL':
            self._send(bytes([FLAG_OK]) + b'50')
        elif cmd == 'VR':
            frame = bytes([STX]) + b'12605130' + bytes([ETX])
            frame += bytes([lrc(frame[1:])])
            os.write(self.master, frame)
        else:
            if cmd == 'DB':
                self.servo = payload == b'11'
            elif cmd == 'CG':
                self.left_error = ord('0') if self.left_error == ord('6') and not self.persistent_fault else self.left_error
            elif cmd == 'CX':
                self.x_mm = self.y_mm = 0.0
                self.theta_rad = 0.0
            self._send(bytes([FLAG_OK]))

    persistent_fault = False
