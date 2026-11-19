# OpenRobot EtherCAT 인터페이스

[English](ethercat_protocol.md) | 한국어

EtherCAT 인터페이스(LAN9252, SOES 슬레이브 스택)가 있는 OpenRobot 모터 제어기를 EtherCAT 마스터로 구동하는
방법. 명령 코드는 [OpenRobot CAN 프로토콜](can_protocol.ko.md)과 같고, 값은 요청/응답 프레임 대신 고정된
프로세스 데이터(PDO)로 전달함.

펌웨어 261119 기준. ESI 파일: [ethercat/OpenRobot_MC.xml](../ethercat/OpenRobot_MC.xml).

## 1. 슬레이브

| 항목 | 값 |
|---|---|
| Vendor ID | 0x00524F42 |
| Product code | 0x00000001 |
| 메일박스 | CoE (SDO), 128바이트 |
| RxPDO (마스터 → 슬레이브) | 0x1600 → 오브젝트 0x7000, SM2, 17바이트, 고정 매핑 |
| TxPDO (슬레이브 → 마스터) | 0x1A00 → 오브젝트 0x6000, SM3, 20바이트, 고정 매핑 |
| 동기 | free run, 마스터 주기 2 ms(500 Hz) 이상 |
| 모터 ID | SDO 0x2070:03 (같은 보드의 CAN ID) |

모든 값은 리틀 엔디언.

### 기동 순서

1. 버스 스캔 (INIT).
2. PRE-OP 를 명시적으로 요청하고 기다림. 일부 마스터는 스캔 후 슬레이브를 INIT 에 둠. SDO 는 PRE-OP 부터 사용 가능.
3. 프로세스 데이터 매핑, SAFE-OP 요청, 주기 교환 시작.
4. OP 요청. 먼저 `control_mode = 0x80`(Motor off)을 보냄.

### 주기

**지원 주기: 2 ms(500 Hz) 이상.** 슬레이브는 free run 으로 모터 제어 인터럽트 사이에 ESC 를 폴링하므로,
모터가 도는 동안 남는 CPU 에 따라 처리하는 마스터 주기 수가 달라짐. SPN-MC V2, 최대 3600 deg/s 회전,
마스터 주기 2 ms 에서 측정:

| 펌웨어 | 프로세스 데이터 처리 |
|---|---|
| 261101 | TxPDO 갱신 약 40 Hz |
| 261102 이상 | 매 주기 |

더 짧은 주기는 지원하지 않음. 모터가 도는 동안 슬레이브가 주기를 건너뜀. 건너뛴 주기는 이전 TxPDO 값을 유지하고
다음 주기의 RxPDO 를 적용함. 버스 쪽에서 잃는 것은 없음(working counter 오류 없음).

## 2. RxPDO, 마스터 → 슬레이브 (17바이트)

| 바이트 | 필드 | 형식 | 스케일 |
|---|---|---|---|
| 0 | control_mode | u8 | 명령 코드, 4절 |
| 1..4 | target_position | i32 | × 10000 |
| 5..8 | target_velocity | i32 | × 10000 |
| 9..12 | target_torque | i32 | × 10000 |
| 13..14 | kp | u16 | × 100 |
| 15..16 | kd | u16 | × 1000 |

`control_mode` 가 0x80..0xC0 밖(0 제외)이면 손상된 프레임으로 보고 버림. `control_mode = 0` 은 받지만 아무것도
하지 않음. 이전 명령이 계속되고 PDO 타임아웃도 발생하지 않음(5절).

**PDO 주기당 변화 제한** (SPI 프레임 손상 대비): 수신한 PDO 하나당 `target_torque` 는 최대 5.0(× 10000 = 50000 raw),
`target_velocity` 는 최대 10.0(100000 raw)만 바뀜. 큰 스텝은 여러 주기에 걸쳐 도달함. 예: 360 deg/s 는 36 주기.

## 3. TxPDO, 슬레이브 → 마스터 (20바이트)

| 바이트 | 필드 | 형식 | 스케일, 단위 |
|---|---|---|---|
| 0 | control_mode | u8 | 동작 중인 명령 코드 (0 = 없음) |
| 1 | fault_code | u8 | fault, [CAN 프로토콜 7절](can_protocol.ko.md#7-fault) 참고 |
| 2..5 | actual_position | i32 | × 10000 deg, 모터 다회전 각도. **MIT 모드에서는 출력축, MIT 영점 기준** |
| 6..9 | actual_velocity | i32 | × 10000 deg/s, **모든 모드에서 모터축** |
| 10..13 | actual_torque | i32 | × 10000 **A** (모터 전류, Nm 아님) |
| 14..15 | bus_voltage | u16 | × 100 V |
| 16 | temp_motor | i8 | °C |
| 17 | temp_mosfet | i8 | °C |
| 18..19 | encoder_pos | u16 | 1회전 엔코더, 14비트 |

출력축 토크 ≈ 전류 × 출력축 Kt (SDO 0x2020:03 으로 읽음).

## 4. 명령 (`control_mode`)

| 코드 | 이름 | 사용 필드 | 의미 |
|---|---|---|---|
| 0x80 | Motor off | | 해제. Big Error fault 도 해제함 |
| 0x81 | Motor stop | | 속도(DPS) 0: 현재 위치 유지 |
| 0x88 | Motor start | | 듀티 0 |
| 0xA1 | 전류 | target_torque: A, kp: 댐핑 0..100 | 전류(토크) 제어, kp > 0 이면 댐핑 |
| 0xA2 | 속도 (DPS, 위치 추종) | target_velocity: deg/s, 모터축 | [CAN 0xA2](can_protocol.ko.md) 참고. EtherCAT 에서는 eRPM 사용 불가 |
| 0xA3 | 위치, 직접 | target_position: deg, 모터 다회전 | 속도 프로파일 없음, 큰 스텝은 [CAN 0xA3](can_protocol.ko.md) 과 같이 제한. Big Error 검사 |
| 0xA4 | 속도 제한 위치 | target_position: deg, target_velocity: 제한 1..25000 deg/s | 프로파일 이동. 범위 밖 제한값은 무시 |
| 0xA5 | 듀티 | target_torque: -1..1 | PWM 듀티 |
| 0xC0 | MIT | target_position rad, target_velocity rad/s, target_torque Nm, kp Nm/rad, kd Nm s/rad | 출력축. torque = kp (p - p_act) + kd (v - v_act) + t_ff |

값은 매 주기 반영됨. CAN 처럼 별도의 진입 단계는 없음. MIT 영점은 SDO 0x2050:05 = 1 로 설정(현재 출력축 위치가
0 rad 가 됨, 저장 안 함).

### Big Error

0xA3 과 0xC0(kp > 0)은 명령과 실제 위치를 출력축 기준으로 비교함. 차이가 기준(SDO 0x2030:08, 기본 180°)을
넘으면 fault 26 을 래치하고 0x80 외의 모든 명령을 무시함. 0x80 으로 해제됨. MIT 는 현재 위치에서 시작해
(MIT 영점 설정 후 p = 0 명령) 거기서부터 움직일 것.

## 5. 통신 끊김

| 상황 | 슬레이브 동작 |
|---|---|
| EtherCAT 이 명령 소스일 때 PDO 타임아웃(SDO 0x2030:07, 기본 1000 ms)보다 오래 PDO 가 없음 | **위치를 포함한 모든 모드에서** 모터를 멈춤 (아래 표의 동작) |
| 마스터가 OP 를 벗어남 | 출력이 0 이 되고 PDO 처리가 멈추므로 위 타임아웃이 발생함 |
| `control_mode = 0` 인 PDO 가 계속 들어옴 | 이전 명령이 계속되고 타임아웃은 발생하지 않음 |
| 0xA2 속도 (DPS) | 목표가 0.5 s 동안 갱신되지 않으면 스스로 멈춤 |

PDO 타임아웃 시 동작:

| 보드 | 동작 |
|---|---|
| SPN-MC V2 / V2R2, D14, D17_D20, VESCAT HIGH V2R2 / V2R3 / V2R4, 펌웨어 261119 이상 | `or_wd` 로 설정한 감시 동작 (기본 댐핑, [CAN 프로토콜 8절](can_protocol.ko.md#8-안전) 참고) |
| 다른 보드, 이전 펌웨어 | 모터 해제 (fault stop) |

반면 CAN 에서는 명령이 끊겨도 위치, 속도 명령이 계속 실행됨(CAN 프로토콜 8절). PDO 타임아웃 상태는 SDO 0x2030:02 에 표시됨.

타임아웃 후 PDO 가 다시 오면 자동으로 제어가 재개됨. 알려진 상태에서 시작하도록 먼저 0x80 을 보낼 것.

## 6. SDO 오브젝트

쓰기는 즉시 적용됨. 설정은 모터가 해제 상태일 때 나중에 플래시에 저장되므로, 쓰기가 제어 루프를 멈추지 않음.

| Index | Sub | 이름 | 형식 | 접근 |
|---|---|---|---|---|
| 0x2000 | 1, 2 | Current_Kp, Current_Ki | REAL32 | RW |
| 0x2001 | 1..5 | Speed_Kp, _Ki, _Kd, _Kd_filter, Speed_min_erpm | REAL32 | RW |
| 0x2002 | 1..4 | Position_Kp, _Ki, _Kd, _Kd_filter | REAL32 | RW |
| 0x2010 | 1 | Encoder_offset | REAL32 | RW |
| 0x2010 | 2 | Encoder_CPR | U32 | RO |
| 0x2020 | 1, 2 | Max_current, Abs_max_current (하드웨어 한계로 제한) | REAL32 | RW |
| 0x2020 | 3 | Torque_constant: 읽으면 출력축 Kt, 쓰면 모터축 Kt 설정 (0 = 모터 설정값) | REAL32 | RW |
| 0x2020 | 4, 5 | Motor_poles (U8), Flux_linkage | | RW |
| 0x2030 | 1 | Fault_code, 0 을 쓰면 해제 | U8 | RW |
| 0x2030 | 2 | PDO_timeout_flag | U8 | RO |
| 0x2030 | 3, 4 | Temp_FET, Temp_motor | REAL32 | RO |
| 0x2030 | 5, 6 | Temp_limit_FET, Temp_limit_motor | REAL32 | RW |
| 0x2030 | 7 | PDO_timeout_ms | U32 | RW |
| 0x2030 | 8 | Big_error_threshold, 출력축 deg | REAL32 | RW |
| 0x2040 | 1, 2 | Control_mode, Control_source | U8 | RO |
| 0x2050 | 1..4 | MIT_V_max, MIT_T_max, MIT_Kt_motor, MIT_Gear_ratio | REAL32 | RW |
| 0x2050 | 5 | MIT_set_zero: 1 을 씀 | U8 | RW |
| 0x2060 | 1, 2 | DPS_Vmax, DPS_Amax | REAL32 | RW |
| 0x2070 | 1, 2 | FW_version, HW_version | string | RO |
| 0x2070 | 3 | Motor_ID (모터 동작 중에는 거부) | U8 | RW |
| 0x2090 | 1..6 | 엔코더 상태 (일부 빌드) | | RO |
| 0x20A0 | 1..7 | 출력 엔코더 (일부 빌드) | | RO |
| 0x2100 | 1..4 | VESC 패킷 터널 (Motor Tool) | | |

## 7. 안전

- PDO 가 끊기면 슬레이브가 모터를 멈추지만(5절), 마스터가 오래된 명령을 계속 보내면 모터는 계속 움직임.
  마스터 루프를 감시하고 오류 시 0x80 을 보낼 것.
- 매 세션을 0x80 으로 시작하고, 주기 교환을 멈추기 전에 0x80 을 보낼 것.
- 모터 전원에 별도의 비상정지를 둘 것.
