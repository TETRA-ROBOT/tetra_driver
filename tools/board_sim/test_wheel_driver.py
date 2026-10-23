"""Drive the real tetra_wheel_driver node against FakeDriveBoard and check the fixed behaviours."""
import math
import os
import signal
import subprocess
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32

sys.path.insert(0, os.path.dirname(__file__))
from fake_drive_board import FakeDriveBoard  # noqa: E402

WS = os.environ['WS']
results = []


def check(name, ok, detail=''):
    results.append((name, ok))
    print(f"[{'PASS' if ok else 'FAIL'}] {name} {detail}", flush=True)


board = FakeDriveBoard()
exe = os.path.join(WS, 'install/tetra_wheel_driver/lib/tetra_wheel_driver/tetra_wheel_driver')
proc = subprocess.Popen([exe, '--ros-args', '-p', f'port:={board.path}', '-p', 'robot_model:=m',
                         '-p', 'm_bEKF_option:=true'],
                        stdout=open(os.path.join(os.environ.get('LOG_DIR', '/tmp'), 'tetra_wheel_driver_test.log'), 'w'), stderr=subprocess.STDOUT)

rclpy.init()
node = rclpy.create_node('wheel_driver_test')
odoms = []
node.create_subscription(Odometry, 'odom', lambda m: odoms.append((time.monotonic(), m)), 50)
cmd = node.create_publisher(Twist, 'cmd_vel_manual', 10)
servo_on = node.create_publisher(Int32, 'servo_on', 10)


def spin_for(seconds, publish=None):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        if publish:
            publish()
        rclpy.spin_once(node, timeout_sec=0.03)


def send(linear, angular):
    def publish():
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        cmd.publish(msg)
    return publish


spin_for(3.0)
check('odometry published at startup', len(odoms) > 20, f'({len(odoms)} msgs)')

# 1. 90° heading, drive forward: twist must be in base frame (x≈v, y≈0)
with board.lock:
    board.theta_rad = math.pi / 2
spin_for(1.0)
spin_for(3.0, send(0.2, 0.0))
recent = [m for t, m in odoms[-20:]]
vx = sum(m.twist.twist.linear.x for m in recent) / len(recent)
vy = sum(m.twist.twist.linear.y for m in recent) / len(recent)
check('twist is body frame at 90 deg heading', abs(vx - 0.2) < 0.03 and abs(vy) < 0.03, f'(vx={vx:.3f}, vy={vy:.3f})')

# 2. command timeout: stop publishing -> wheels stop within ~0.5 s (+ decel ramp)
spin_for(1.5)
with board.lock:
    stopped = board.left_mm_s == 0 and board.right_mm_s == 0
check('stops after cmd_vel timeout', stopped, f'(L={board.left_mm_s}, R={board.right_mm_s})')

# 3. speed clamp: 10 m/s request -> at most 2 m/s at the wheels
spin_for(4.0, send(10.0, 0.0))
with board.lock:
    peak = max(abs(board.left_mm_s), abs(board.right_mm_s))
check('linear speed clamped to 2.0 m/s', 0 < peak <= 2000, f'(wheel {peak} mm/s)')
spin_for(1.5)

# 4. angle wrap: turning through 0 deg must not spike angular velocity
with board.lock:
    board.theta_rad = math.radians(350)
spin_for(1.0)
start = len(odoms)
spin_for(3.0, send(0.0, 0.5))
wz = [m.twist.twist.angular.z for t, m in odoms[start + 5:]]
check('no angular velocity spike at 360->0 wrap', wz and max(abs(w) for w in wz) < 1.0, f'(max |wz|={max(abs(w) for w in wz):.2f})')
spin_for(1.5)

# 5. persistent overload: at most 3 resets about 1 s apart, then latched with servo off
t_fault = time.monotonic()
with board.lock:
    board.persistent_fault = True
    board.left_error = ord('6')
spin_for(7.0)
# 재시도는 '에러 리셋 + 서보 ON' 이다. 서보를 다시 켠 횟수를 센다 (서보가 꺼질 때 루프가 보내는 리셋 1번은 재시도가 아님)
resets = [(t, p) for t, p in board.commands('DB') if t >= t_fault and p == b'11']
check('overload: servo re-energised at most 3 times', len(resets) == 3, f'({len(resets)} in 7 s)')
gaps = [b[0] - a[0] for a, b in zip(resets, resets[1:])]
check('reset attempts at least 1 s apart', all(g >= 0.95 for g in gaps), f'(gaps {[round(g, 2) for g in gaps]})')
with board.lock:
    servo_now = board.servo
check('servo left off after latch', not servo_now)

# 6. operator servo_on clears the latch -> resets resume
board.persistent_fault = False
before = len(board.commands('CG'))
servo_on.publish(Int32(data=1))
spin_for(2.5)
check('servo_on clears the fault latch', len(board.commands('CG')) > before, f'({len(board.commands("CG")) - before} new resets)')
with board.lock:
    board.left_error = ord('0')
spin_for(1.0)

# 7. board stops answering -> odometry stops; answering again -> odometry resumes
with board.lock:
    board.responding = False
spin_for(1.0)
mark = len(odoms)
spin_for(3.0)
silent = len(odoms) - mark
with board.lock:
    board.responding = True
spin_for(4.0)
check('no stale odometry while board is silent', silent == 0, f'({silent} msgs while silent)')
check('odometry resumes after reconnect', len(odoms) - mark - silent > 20, f'({len(odoms) - mark - silent} msgs)')

# 8. shutdown: zero velocity before servo off
spin_for(1.0, send(0.2, 0.0))
t_sig = time.monotonic()
proc.send_signal(signal.SIGINT)
proc.wait(timeout=10)
after = [(t, c, p) for t, c, p in board.log if t >= t_sig]
order = [c for t, c, p in after if c in ('BV', 'DB')]
last_bv = [p for t, c, p in after if c == 'BV']
zero_first = order[-2:] == ['BV', 'DB'] and last_bv and last_bv[-1] == b'\x00\x00\x00\x00'
check('shutdown sends zero speed then servo off', zero_first, f'(tail {order[-3:]})')

node.destroy_node()
rclpy.shutdown()
failed = [n for n, ok in results if not ok]
print(f'\n{len(results) - len(failed)}/{len(results)} passed', flush=True)
sys.exit(1 if failed else 0)
