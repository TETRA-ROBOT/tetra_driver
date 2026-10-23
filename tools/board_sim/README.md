# board_sim — 하드웨어 없이 드라이버 확인

가상 구동 보드·전원 보드를 의사 터미널(pty)에 띄우고, **빌드한 실제 노드**를 그 포트에 붙여 동작을 확인합니다.
보드 패킷 형식은 `drive_module.c`, `power_module.c` 를 그대로 따릅니다.

| 파일 | 내용 |
|---|---|
| `fake_drive_board.py` | 가상 구동 보드. 속도 명령을 받아 위치를 적분하고, 에러 코드·무응답을 흉내 낼 수 있음 |
| `fake_power_board.py` | 가상 전원 보드. 배터리·데이터 읽기와 ASCII 명령에 응답, 무응답을 흉내 낼 수 있음 |
| `test_wheel_driver.py` | 바퀴 드라이버 시나리오 12개 (아래) |
| `test_power_driver.py` | 전원 드라이버 시나리오 8개 (아래) |

## 실행

```bash
# 워크스페이스에 interfaces, tetra_serial, tetra_wheel_driver, tetra_power_driver 를 빌드해 둔다
colcon build --packages-up-to tetra_wheel_driver tetra_power_driver
source install/setup.bash

export WS=$PWD               # 워크스페이스 루트 (install/ 이 있는 곳)
export ROS_DOMAIN_ID=77      # 다른 로봇·노드와 섞이지 않게
python3 -P <tetra_driver>/tools/board_sim/test_wheel_driver.py
python3 -P <tetra_driver>/tools/board_sim/test_power_driver.py
```

노드 로그는 `$LOG_DIR`(기본 `/tmp`)의 `tetra_*_driver_test.log` 에 남습니다.

## 확인하는 것

바퀴 드라이버: 시작 시 오도메트리 발행, 90° 방향 직진 시 twist 가 로봇 기준(vx≈v, vy≈0), 속도 명령이 끊기면 정지,
속도 상한(2 m/s), 359°→0° 회전 시 각속도 튐 없음, 과부하가 계속되면 1초 간격으로 3번만 다시 켜고 서보 OFF 로 멈춤,
`/servo_on` 1 로 해제, 보드 무응답 동안 오도메트리 발행 중단과 재연결, 종료 시 0 속도 → 서보 OFF.

전원 드라이버: 배터리 발행, 데이터 읽기 서비스 결과, ASCII 명령(LED) 성공 응답, 보드가 끊기면 `/power_status` -10 한 번과
`/servo_on` 2, 끊긴 동안 읽기 서비스는 실패로 응답, 재연결, 끊김 로그 한 번.

실제 보드와 다른 점: 가상 보드는 모든 명령에 즉시 응답하고 모터 물리(가감속, 미끄러짐)는 흉내 내지 않습니다.
보드 펌웨어의 실제 응답 시간·에러 동작은 로봇에서 확인해야 합니다.
