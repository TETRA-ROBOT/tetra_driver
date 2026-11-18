# tetra_driver

TETRA 로봇의 핵심 모터 드라이버 및 하드웨어 통신 인터페이스를 담당하는 서브모듈입니다.

---

## 1. 패키지 구성 (Packages Overview)

- **`tetra_wheel_driver`**: TETRA 차륜 모터 구동 및 주행 제어를 위한 ROS2 드라이버 패키지
- **`tetra_power_driver`**: 전원 제어, 배터리 모니터링, LED/GPIO/초음파 제어 등 전원 보드 하드웨어와의 시리얼(RS232) 통신 및 인터페이스 패키지
- **`tetra_serial`**: 두 드라이버가 같이 쓰는 RS232 포트 설정·패킷 읽기/쓰기 라이브러리

Pan/Tilt HerkuleX 서보는 `tetra_dependency/herkulex_driver` 를 씁니다. 하드웨어 없이 드라이버를 확인하려면 `tools/board_sim/README.md` 를 보세요.

---

## 2. 보드 명령 테스트 (`~/raw_command`)

관리자 화면(관리자 설정 → 보드 명령 테스트, API `POST /board/command`)이 보드에 임의 바이트를 보내고 응답을 그대로 보는 서비스입니다.
펌웨어 확인·새 명령 시험용이며, 모터를 움직이거나 보드 파라미터를 바꾸는 명령도 막지 않습니다.

| 서비스 | 보드 |
|---|---|
| `/tetra_wheel_driver/raw_command` | 구동 보드 (바퀴에 속도를 주는 중이면 거부) |
| `/tetra_power_driver/raw_command` | 전원 보드 |

타입은 `interfaces/srv/BoardRawCommand` (`tetra_dependency/interfaces/srv/BoardRawCommand.srv`):

- `data` 1~256 바이트, `framing` = `FRAMING_RAW`(그대로) / `FRAMING_PROTOCOL`(STX + data + ETX + LRC, LRC 는 data..ETX 의 XOR — 기존 명령과 같은 틀)
- `read_mode` = `READ_UNTIL_IDLE`(응답이 30ms 끊길 때까지) / `READ_ASCII_FRAME`(LRC 가 맞는 ETX 까지) / `READ_LENGTH_FRAME`(STX 길이 데이터 ETX LRC, 전원 보드 데이터 응답) / `READ_NONE`
- `timeout_ms` 0 이면 300, 최대 2000. `max_reply_length` 0 이면 256, 최대 256
- 응답: `success`, `message`, `sent`(실제로 보낸 바이트), `reply`(받은 바이트 그대로, 실패해도 받은 만큼), `elapsed_ms`

```bash
ros2 service call /tetra_power_driver/raw_command interfaces/srv/BoardRawCommand "{data: [80, 86], framing: 1, read_mode: 2, timeout_ms: 300}"
```

**폴링과 섞이지 않는 이유**: 두 드라이버는 30Hz 폴링 루프 안에서 `rclcpp::spin_some` 으로 서비스 콜백을 부릅니다 (스레드 하나).
그래서 raw 명령이 도는 동안에는 폴링 패킷이 나가지 않고, 응답도 폴링 쪽이 읽어 가지 않습니다. 보내기 전에는 `send_packet` 이 남은 수신 데이터를 버리고,
받은 뒤에는 늦게 오는 바이트를 조용해질 때까지(최대 0.2초) 버려 다음 폴링 응답으로 읽히지 않게 합니다 (`tetra_serial/rs232_raw.c`).
대신 그동안(최대 `timeout_ms` + 0.2초) 그 보드의 폴링이 멈춥니다: 구동 보드는 오도메트리·속도 명령, 전원 보드는 배터리·비상정지 상태가 그만큼 늦습니다.
그래서 구동 드라이버는 바퀴에 속도를 주는 중이면 거부하고, 웹 API 는 주행·도킹·태스크 중에 거부합니다.

공통 처리(검사·틀 씌우기·응답 문구)는 `tetra_serial/include/tetra_serial/raw_command_service.hpp` 한 곳에 있습니다.
가상 보드 확인: `tools/board_sim/test_raw_command.py`.
