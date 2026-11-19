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
# 이 PC 의 ~/.config/tetra 보정 배율을 읽지 않게 시험용 파일을 쓴다
SCALE_FILE = os.path.join(os.environ.get('LOG_DIR', '/tmp'), f'wheel_odom_scale_test_{os.getpid()}.txt')
if os.path.exists(SCALE_FILE):
    os.remove(SCALE_FILE)
exe = os.path.join(WS, 'install/tetra_wheel_driver/lib/tetra_wheel_driver/tetra_wheel_driver')
proc = subprocess.Popen([exe, '--ros-args', '-p', f'port:={board.path}', '-p', 'robot_model:=m',
                         '-p', 'ekf_option:=true', '-p', f'odom_scale_file:={SCALE_FILE}'],
                        stdout=open(os.path.join(os.environ.get('LOG_DIR', '/tmp'), 'tetra_wheel_driver_test.log'), 'w'), stderr=subprocess.STDOUT)

rclpy.init()
node = rclpy.create_node('wheel_driver_test')
odoms = []
node.create_subscription(Odometry, 'odom', lambda m: odoms.append((time.monotonic(), m)), 50)
cmd = node.create_publisher(Twist, 'cmd_vel_manual', 10)
nav_final_cmd = node.create_publisher(Twist, 'cmd_vel_nav2', 10)
nav_raw_cmd = node.create_publisher(Twist, 'cmd_vel_nav', 10)
servo_on = node.create_publisher(Int32, 'servo_on', 10)


def spin_for(seconds, publish=None):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        if publish:
            publish()
        rclpy.spin_once(node, timeout_sec=0.03)


def send(linear, angular, publisher=None):
    def publish():
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        (publisher or cmd).publish(msg)
    return publish


def wheels_moving():
    with board.lock:
        return board.left_mm_s != 0 or board.right_mm_s != 0


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

# 9. Nav2 final output (collision_monitor -> cmd_vel_nav2) drives the wheels
spin_for(1.5)
spin_for(2.0, send(0.2, 0.0, nav_final_cmd))
check('drives on cmd_vel_nav2', wheels_moving())
spin_for(1.5)

# 10. raw controller output (before smoother/collision_monitor) is ignored by default
spin_for(2.0, send(0.2, 0.0, nav_raw_cmd))
check('ignores raw cmd_vel_nav (use_raw_nav_cmd=false)', not wheels_moving())

# 11. odometry scale (관리자 바퀴 오도메트리 보정): 파라미터를 바꾸면 그 뒤 이동량만 배율만큼 늘고 파일에 남는다
from rclpy.parameter import Parameter  # noqa: E402
from rclpy.parameter_client import AsyncParameterClient  # noqa: E402
params = AsyncParameterClient(node, 'tetra_wheel_driver')
params.wait_for_services(5.0)
future = params.set_parameters([Parameter('odom_linear_scale', Parameter.Type.DOUBLE, 1.1),
                                Parameter('odom_angular_scale', Parameter.Type.DOUBLE, 0.9)])
rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
spin_for(1.0)
board_start = (board.x_mm, board.y_mm)
odom_start = odoms[-1][1].pose.pose.position
spin_for(2.0, send(0.2, 0.0))
spin_for(1.0)
board_dist = math.hypot(board.x_mm - board_start[0], board.y_mm - board_start[1]) / 1000.0
odom_end = odoms[-1][1].pose.pose.position
odom_dist = math.hypot(odom_end.x - odom_start.x, odom_end.y - odom_start.y)
check('odom_linear_scale stretches odometry', board_dist > 0.2 and abs(odom_dist / board_dist - 1.1) < 0.01,
      f'(board {board_dist:.3f} m, odom {odom_dist:.3f} m)')
saved = open(SCALE_FILE).read().split() if os.path.exists(SCALE_FILE) else []
check('scale saved to odom_scale_file', saved == ['1.1', '0.9'], f'({saved})')
rejected = params.set_parameters([Parameter('odom_linear_scale', Parameter.Type.DOUBLE, 1.5)])
rclpy.spin_until_future_complete(node, rejected, timeout_sec=5.0)
check('out of range scale is rejected', rejected.result() is not None and not rejected.result().results[0].successful)

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
if os.path.exists(SCALE_FILE):
    os.remove(SCALE_FILE)
failed = [n for n, ok in results if not ok]
print(f'\n{len(results) - len(failed)}/{len(results)} passed', flush=True)
sys.exit(1 if failed else 0)
