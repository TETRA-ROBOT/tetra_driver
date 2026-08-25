# tetra_driver

TETRA 로봇의 핵심 모터 드라이버 및 하드웨어 통신 인터페이스를 담당하는 서브모듈입니다.

---

## 1. 패키지 구성 (Packages Overview)

- **`tetra`**: TETRA 차륜 모터 구동 및 메인 제어를 위한 ROS2 드라이버 패키지
- **`tetra_interface`**: 전원 제어, 배터리 모니터링, LED/GPIO 제어 등 보드 하드웨어와의 시리얼(RS232) 통신 및 인터페이스 패키지
- **`herkulex`**: Pan/Tilt 카메라 마운트 및 서보 모터 제어를 위한 Herkulex 드라이버 패키지
