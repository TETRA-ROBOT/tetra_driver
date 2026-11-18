"""Check ~/raw_command on the real wheel and power driver nodes against the simulated boards.

Raw exchanges run inside each driver's polling loop, so a reply must never be mixed with a polling frame
and polling must carry on afterwards.
"""
import os
import signal
import subprocess
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from interfaces.srv import BoardRawCommand
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32

sys.path.insert(0, os.path.dirname(__file__))
from fake_drive_board import FakeDriveBoard, lrc  # noqa: E402
from fake_power_board import FakePowerBoard  # noqa: E402

WS = os.environ['WS']
LOG_DIR = os.environ.get('LOG_DIR', '/tmp')
STX, ETX = 0x02, 0x03
results = []


def check(name, ok, detail=''):
    results.append((name, ok))
    print(f"[{'PASS' if ok else 'FAIL'}] {name} {detail}", flush=True)


def frame(body):
    return bytes([STX]) + body + bytes([ETX, lrc(body + bytes([ETX]))])


def length_frame(payload):
    return bytes([STX]) + len(payload).to_bytes(2, 'big') + payload + bytes([ETX, lrc(payload)])


drive = FakeDriveBoard()
power = FakePowerBoard()


def start(package, args, log_name):
    exe = os.path.join(WS, f'install/{package}/lib/{package}/{package}')
    return subprocess.Popen([exe, '--ros-args'] + args, stdout=open(os.path.join(LOG_DIR, log_name), 'w'),
                            stderr=subprocess.STDOUT)


wheel_proc = start('tetra_wheel_driver', ['-p', f'port:={drive.path}', '-p', 'robot_model:=m'], 'tetra_wheel_raw_test.log')
power_proc = start('tetra_power_driver', ['-p', f'port:={power.path}'], 'tetra_power_raw_test.log')

rclpy.init()
node = rclpy.create_node('raw_command_test')
odoms, battery = [], []
node.create_subscription(Odometry, 'odom', lambda m: odoms.append(time.monotonic()), 50)
node.create_subscription(Int32, 'tetra_battery', lambda m: battery.append(m.data), 50)
manual_cmd = node.create_publisher(Twist, 'cmd_vel_manual', 10)
wheel_cli = node.create_client(BoardRawCommand, '/tetra_wheel_driver/raw_command')
power_cli = node.create_client(BoardRawCommand, '/tetra_power_driver/raw_command')


def spin_for(seconds, publish=None):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        if publish:
            publish()
        rclpy.spin_once(node, timeout_sec=0.03)


def call(client, data, framing=0, read_mode=0, timeout_ms=0, max_reply_length=0):
    client.wait_for_service(timeout_sec=5.0)
    request = BoardRawCommand.Request(data=list(data), framing=framing, read_mode=read_mode,
                                      timeout_ms=timeout_ms, max_reply_length=max_reply_length)
    future = client.call_async(request)
    end = time.monotonic() + 5.0
    while not future.done() and time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.02)
    return future.result()


spin_for(2.0)
P = BoardRawCommand.Request

# --- power board -------------------------------------------------------------
res = call(power_cli, b'PD', framing=P.FRAMING_PROTOCOL, read_mode=P.READ_LENGTH_FRAME)
pd_reply = length_frame(b''.join(w.to_bytes(2, 'big') for w in [252] + [0] * 11))
check('power: protocol framing wraps STX..ETX LRC', bytes(res.sent) == frame(b'PD'), f'(sent={bytes(res.sent).hex()})')
check('power: length-prefixed reply returned exactly', res.success and bytes(res.reply) == pd_reply,
      f'(success={res.success}, msg={res.message}, reply={bytes(res.reply).hex()})')

res = call(power_cli, frame(b'LI1;50'), framing=P.FRAMING_RAW, read_mode=P.READ_ASCII_FRAME)
check('power: raw bytes sent as-is, ASCII frame reply', res.success and bytes(res.reply) == frame(b'\x30'),
      f'(msg={res.message}, reply={bytes(res.reply).hex()})')

mixed = 0
for _ in range(30):
    res = call(power_cli, b'PD', framing=P.FRAMING_PROTOCOL, read_mode=P.READ_UNTIL_IDLE, timeout_ms=200)
    if not res.success or bytes(res.reply) != pd_reply:
        mixed += 1
    rclpy.spin_once(node, timeout_sec=0.01)
check('power: 30 raw commands during polling never mix with poll frames', mixed == 0, f'({mixed} bad replies)')
mark = len(battery)
spin_for(1.0)
check('power: polling continues after raw commands', len(battery) - mark >= 15, f'({len(battery) - mark} battery msgs/s)')

power.responding = False
res = call(power_cli, b'PD', framing=P.FRAMING_PROTOCOL, read_mode=P.READ_LENGTH_FRAME, timeout_ms=200)
check('power: no reply reported as failure within timeout', not res.success and 'no reply' in res.message
      and 190 <= res.elapsed_ms < 700, f'(msg={res.message}, elapsed={res.elapsed_ms})')
power.responding = True
spin_for(3.0)

res = call(power_cli, b'')
check('power: empty data rejected', not res.success and res.message == 'data is empty', f'({res.message})')
res = call(power_cli, b'PD', timeout_ms=2001)
check('power: timeout above 2000 ms rejected', not res.success and 'timeout_ms' in res.message, f'({res.message})')
res = call(power_cli, b'x' * 257)
check('power: data above 256 bytes rejected', not res.success and '256' in res.message, f'({res.message})')

# --- drive board -------------------------------------------------------------
res = call(wheel_cli, b'VR', framing=P.FRAMING_PROTOCOL, read_mode=P.READ_ASCII_FRAME)
check('wheel: version command reply', res.success and bytes(res.reply) == frame(b'12605130'),
      f'(msg={res.message}, reply={bytes(res.reply)})')

mixed = 0
for _ in range(20):
    res = call(wheel_cli, b'XL', framing=P.FRAMING_PROTOCOL, read_mode=P.READ_ASCII_FRAME, timeout_ms=200)
    if not res.success or bytes(res.reply) != frame(b'\x3050'):
        mixed += 1
check('wheel: 20 raw commands during 30 Hz polling never mix with velocity frames', mixed == 0, f'({mixed} bad replies)')
mark = len(odoms)
spin_for(1.0)
check('wheel: odometry continues after raw commands', len(odoms) - mark >= 20, f'({len(odoms) - mark} odom msgs/s)')


def drive_forward():
    msg = Twist()
    msg.linear.x = 0.2
    manual_cmd.publish(msg)


spin_for(0.5, drive_forward)
drive_forward()
spin_for(0.05)
res = call(wheel_cli, b'VR', framing=P.FRAMING_PROTOCOL)
check('wheel: refused while wheels are driven', not res.success and 'driven' in res.message and len(res.sent) == 0,
      f'({res.message})')
spin_for(2.0)  # cmd_vel_timeout(0.5 s) 뒤 감속해 멈춘다
res = call(wheel_cli, b'VR', framing=P.FRAMING_PROTOCOL)
check('wheel: accepted again once stopped', res.success, f'({res.message})')

for proc in (wheel_proc, power_proc):
    proc.send_signal(signal.SIGINT)
    proc.wait(timeout=10)
log = open(os.path.join(LOG_DIR, 'tetra_power_raw_test.log')).read()
check('power: every raw command is logged by the driver', log.count('raw_command:') >= 35, f"({log.count('raw_command:')} lines)")

node.destroy_node()
rclpy.shutdown()
failed = [n for n, ok in results if not ok]
print(f'\n{len(results) - len(failed)}/{len(results)} passed', flush=True)
sys.exit(1 if failed else 0)
