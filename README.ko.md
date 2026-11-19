# OpenRobot Motor SDK

[English](README.md) | 한국어

OpenRobot 모터 제어기를 CAN 과 EtherCAT 으로 구동하는 라이브러리와 예제. 첫 구동부터 다관절 로봇 적용까지 다룸.

| 구성 | 플랫폼 | 확인 환경 |
|---|---|---|
| [CAN 프로토콜](docs/can_protocol.ko.md) | 공통 | 펌웨어 261119 소스 |
| [EtherCAT 인터페이스](docs/ethercat_protocol.ko.md) | 공통 | 펌웨어 261119 소스, [ESI 파일](ethercat/OpenRobot_MC.xml) |
| [Python 패키지와 예제](python/) | Windows, Linux, macOS | Windows 에서 PCAN, EtherCAT |
| [C++ 라이브러리와 예제](cpp/) | Windows, Linux, macOS | Windows 에서 PCAN |
| [ROS 2 (ros2_control)](ros2/) | Linux (Jazzy) | SocketCAN 으로 PCAN |
| [Arduino 라이브러리](arduino/) | MCP2515 쉴드를 쓰는 보드 | Arduino Zero, Seeed CAN-BUS Shield |
| [Android 현장 점검 앱](android/) | Android 7.0 이상, USB OTG | CANable (slcan) |

CAN 어댑터: slcan (CANable 및 호환품), PEAK PCAN.

각 폴더의 README 는 영문만 있음.

## 제어기와 펌웨어

| 보드 | 펌웨어 |
|---|---|
| SPN-MC V2 / V2R2, SPN-MC D14, SPN-MC D17_D20, VESCAT HIGH V2R2 / V2R3 / V2R4 | 261119 이상 권장 |

펌웨어와 Motor Tool 은 [openrobot-motor-tool](https://github.com/dongilc/openrobot-motor-tool/releases) 에서 배포함.
이전 펌웨어도 이 SDK 로 쓸 수 있으나 통신 끊김 감시가 없어, 호스트가 명령을 멈춰도 모든 명령이 계속 실행됨
([CAN 프로토콜 8절](docs/can_protocol.ko.md#8-안전) 참고).

## 단계별 예제

0. 연결: 모터 찾기, 상태 읽기
1. 전류, 속도, 위치 명령
2. MIT 임피던스 제어
3. 여러 모터를 한 루프에서 구동
4. 안전: fault, 응답 누락, Ctrl+C

모터를 움직이는 예제는 시작 전에 확인을 받고, 종료 시 모터를 해제함.

## 안전

- 명령이 끊겨도 위치, 속도 명령은 계속 실행됨. 호스트 루프를 감시하고 오류 시 Motor off(0x80)를 보낼 것.
- 호스트와 별개로 모터 전원을 끊는 비상정지를 둘 것.
- 각도와 속도는 모터축 기준. MIT 모드만 출력축 기준.

## 라이선스

MIT, [LICENSE](LICENSE) 참고. SDK 는 CAN 또는 EtherCAT 으로 제어기와 통신만 하며, 제어기 펌웨어는 별도 라이선스로 배포함.
