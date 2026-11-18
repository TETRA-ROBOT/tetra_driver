"""Legacy TETRA-M drive board: 16-byte velocity2 reply and the EMG button releasing the servo."""
import os
import signal
import subprocess
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Int32

sys.path.insert(0, os.path.dirname(__file__))
from fake_drive_board import FakeDriveBoard  # noqa: E402

WS = os.environ['WS']
results = []


def check(name, ok, detail=''):
    results.append((name, ok))
    print(f"[{'PASS' if ok else 'FAIL'}] {name} {detail}", flush=True)


board = FakeDriveBoard()
board.legacy_frame = True
exe = os.path.join(WS, 'install/tetra_wheel_driver/lib/tetra_wheel_driver/tetra_wheel_driver')
log_path = os.path.join(os.environ.get('LOG_DIR', '/tmp'), 'tetra_wheel_driver_legacy_test.log')
proc = subprocess.Popen([exe, '--ros-args', '-p', f'port:={board.path}', '-p', 'robot_model:=m',
                         '-p', 'ekf_option:=true', '-p', 'emg_releases_servo:=true'],
                        stdout=open(log_path, 'w'), stderr=subprocess.STDOUT)

rclpy.init()
node = rclpy.create_node('wheel_driver_legacy_test')
odoms = []
emg_states = []
incompatible = []
node.create_subscription(Odometry, 'odom', lambda m: odoms.append((time.monotonic(), m)), 50)
node.create_subscription(Int32, 'emg_state', lambda m: emg_states.append(m.data), 10)
latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
node.create_subscription(Bool, 'drive_board_incompatible', lambda m: incompatible.append(m.data), latched)
cmd = node.create_publisher(Twist, 'cmd_vel_manual', 10)


def spin_for(seconds, publish=None):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        if publish:
            publish()
        rclpy.spin_once(node, timeout_sec=0.03)


def drive():
    msg = Twist()
    msg.linear.x = 0.3
    cmd.publish(msg)


def wheels_moving():
    with board.lock:
        return board.left_mm_s != 0 or board.right_mm_s != 0


def servo_commands_since(since):
    return [p for t, p in board.commands('DB') if t >= since]


# 1. loop rate: waiting for 20 bytes on a 16-byte board used to stall every cycle on the read timeout (~5 Hz)
spin_for(3.0)
mark = len(odoms)
spin_for(3.0)
rate = (len(odoms) - mark) / 3.0
check('control loop runs near 30 Hz on the 16-byte board', rate > 22.0, f'({rate:.1f} Hz)')

# 2. drives, and stops soon after the command ends
spin_for(2.0, drive)
check('drives on the 16-byte board', wheels_moving())
t_release = time.monotonic()
stopped_after = None
while time.monotonic() - t_release < 3.0:
    spin_for(0.02)
    if not wheels_moving():
        stopped_after = time.monotonic() - t_release
        break
check('stops within 1 s of the last command', stopped_after is not None and stopped_after < 1.0,
      f'({stopped_after if stopped_after is None else round(stopped_after, 2)} s)')

# 3. EMG pressed while driving: servo released, no re-energising, no "drive board incompatible"
spin_for(1.0, drive)
t_emg = time.monotonic()
with board.lock:
    board.emg = True
spin_for(4.0, drive)
with board.lock:
    servo_during_emg = board.servo
check('EMG press releases the servo', not servo_during_emg)
check('servo is not re-energised while EMG is held', b'11' not in servo_commands_since(t_emg + 0.5),
      f'({servo_commands_since(t_emg + 0.5)})')
check('EMG is not reported as a drive board fault', True not in incompatible, f'({incompatible})')
check('emg_state published', emg_states and emg_states[-1] == 1)

# 4. EMG released: servo back on, and the robot does not move by itself
with board.lock:
    board.emg = False
spin_for(2.0)
with board.lock:
    servo_after = board.servo
check('servo comes back after EMG release', servo_after)
check('does not move after release without a new command', not wheels_moving())
spin_for(2.0, drive)
check('drives again after release', wheels_moving())

proc.send_signal(signal.SIGINT)
proc.wait(timeout=10)
node.destroy_node()
rclpy.shutdown()
failed = [n for n, ok in results if not ok]
print(f'\n{len(results) - len(failed)}/{len(results)} passed', flush=True)
sys.exit(1 if failed else 0)
