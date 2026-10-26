# tetra_driver

TETRA 로봇의 핵심 모터 드라이버 및 하드웨어 통신 인터페이스를 담당하는 서브모듈입니다.

---

## 1. 패키지 구성 (Packages Overview)

- **`tetra_wheel_driver`**: TETRA 차륜 모터 구동 및 주행 제어를 위한 ROS2 드라이버 패키지
- **`tetra_power_driver`**: 전원 제어, 배터리 모니터링, LED/GPIO/초음파 제어 등 전원 보드 하드웨어와의 시리얼(RS232) 통신 및 인터페이스 패키지
- **`tetra_serial`**: 두 드라이버가 같이 쓰는 RS232 포트 설정·패킷 읽기/쓰기 라이브러리

Pan/Tilt HerkuleX 서보는 `tetra_dependency/herkulex_driver` 를 씁니다. 하드웨어 없이 드라이버를 확인하려면 `tools/board_sim/README.md` 를 보세요.
