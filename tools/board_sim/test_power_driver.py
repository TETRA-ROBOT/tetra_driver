"""Drive the real tetra_power_driver node against a simulated power board on a pseudo-terminal."""
import os
import signal
import subprocess
import sys
import time

import rclpy
from interfaces.srv import LedControl, PowerDataRead
from std_msgs.msg import Int32

sys.path.insert(0, os.path.dirname(__file__))
from fake_power_board import FakePowerBoard  # noqa: E402

WS = os.environ['WS']
HERE = os.path.dirname(__file__)
results = []


def check(name, ok, detail=''):
    results.append((name, ok))
    print(f"[{'PASS' if ok else 'FAIL'}] {name} {detail}", flush=True)


board = FakePowerBoard()
exe = os.path.join(WS, 'install/tetra_power_driver/lib/tetra_power_driver/tetra_power_driver')
node_log = os.path.join(os.environ.get('LOG_DIR', '/tmp'), 'tetra_power_driver_test.log')
proc = subprocess.Popen([exe, '--ros-args', '-p', f'port:={board.path}'],
                        stdout=open(node_log, 'w'), stderr=subprocess.STDOUT)

rclpy.init()
node = rclpy.create_node('power_driver_test')
battery, power_status, servo = [], [], []
node.create_subscription(Int32, 'tetra_battery', lambda m: battery.append(m.data), 50)
node.create_subscription(Int32, 'power_status', lambda m: power_status.append(m.data), 50)
node.create_subscription(Int32, 'servo_on', lambda m: servo.append(m.data), 50)
data_cli = node.create_client(PowerDataRead, 'Power_data_read_cmd')
led_cli = node.create_client(LedControl, 'led_cmd')


def spin_for(seconds):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.05)


def call(client, request):
    client.wait_for_service(timeout_sec=5.0)
    future = client.call_async(request)
    end = time.monotonic() + 5.0
    while not future.done() and time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.05)
    return future.result()


spin_for(3.0)
check('battery published from board', battery and battery[-1] == 85, f'(last={battery[-1] if battery else None})')

res = call(data_cli, PowerDataRead.Request())
check('data read reports success with board values', res.command_result and res.d0_battery_voltage == 252,
      f'(result={res.command_result}, d0={res.d0_battery_voltage})')

res = call(led_cli, LedControl.Request(id=1, led_brightness=50))
check('ASCII command (LED) still reports success', res.command_result)

board.responding = False
mark_status, mark_servo = len(power_status), len(servo)
spin_for(3.0)
errors = [v for v in power_status[mark_status:] if v < 0]
check('power error -10 published once on board loss', errors == [-10], f'({errors})')
check('servo off requested while board is silent', 2 in servo[mark_servo:])
res = call(data_cli, PowerDataRead.Request())
check('data read reports failure while board is silent', res is not None and not res.command_result,
      f'(result={res.command_result if res else None})')

board.responding = True
mark_battery = len(battery)
spin_for(4.0)
check('readings resume after reconnect', len(battery) - mark_battery > 10, f'({len(battery) - mark_battery} msgs)')

proc.send_signal(signal.SIGINT)
proc.wait(timeout=10)
log = open(node_log).read()
check('disconnect logged once, not every cycle', log.count('Power Board Disconnect') == 1 and log.count('data read Error') == 1,
      f"(disconnect x{log.count('Power Board Disconnect')}, read error x{log.count('data read Error')})")

node.destroy_node()
rclpy.shutdown()
failed = [n for n, ok in results if not ok]
print(f'\n{len(results) - len(failed)}/{len(results)} passed', flush=True)
sys.exit(1 if failed else 0)
