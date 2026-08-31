# tetra_driver - AI Agent Context

## 개요
로봇의 저수준(Low-level) 하드웨어와 ROS2 사이를 연결하는 핵심 드라이버 패키지 그룹입니다. 
시리얼 통신(RS232)을 통해 모터 컨트롤러, 전원 관리 보드, 서보 모터를 직접 제어합니다.

## 패키지 맵

| 서브 패키지 | 주요 소스 | 역할 | 주요 통신 (Topic/Service) |
|---|---|---|---|
| `tetra_wheel_driver` | `tetra_wheel_driver.cpp`, `drive_module.c` | 좌/우 바퀴 모터 제어, Odometry 계산, TF 브로드캐스팅 | Sub: `/cmd_vel`<br>Pub: `/odom`, `/tf` |
| `tetra_power_driver` | `tetra_power_driver.cpp`, `power_module.c` | 배터리, 충전 전류, 전압, Sonar 초음파, LED 제어, 비상정지(EMG), 범퍼 | Pub: `/tetra_battery`, `/tetra_power`, `/gpio_status`<br>Srv: `Power_outport_cmd`, `led_control` |
| `herkulex` | `herkulex_node.cpp`, `herkulex_interface.c` | 스마트 서보 모터 기반 Pan/Tilt 제어 및 각도 이동 | Srv: `PanMove_cmd`, `TiltMove_cmd` |

## 아키텍처 다이어그램

```
  [ /cmd_vel ] ──> [ tetra_wheel_driver ] ──> RS232 ──> [ 모터 컨트롤러 (휠) ]
                          │
                          └──> [ /odom ], [ /tf ]
                          
  [ tetra_power_driver ] ──> RS232 ──> [ 전원 보드 (Power Board) ]
         │
         ├──> [ /tetra_battery ], [ /tetra_power ]
         ├──> [ /gpio_status ]
         └──> [ Sonar PointCloud2 ]
```

## AI 에이전트 주의사항
1. **시리얼 포트**: 기본 포트는 `/dev/ttyUSB0` (휠), `/dev/ttyUSB1` (전원) 등이며 udev rule (`/etc/udev/rules.d/`)에 의해 매핑됩니다.
2. **단위 표준**:
   - 선속도: `m/s`, 각속도: `rad/s`
   - 배터리: 퍼센트(`%`, 0~100)
   - 전압: `V`, 전류: `A`
