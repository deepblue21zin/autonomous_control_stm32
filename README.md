# Autonomous Steering Control System

**STM32F429ZI 기반 자율주행 조향 서보모터 위치 제어 펌웨어**

| 항목 | 내용 |
|------|------|
| MCU | STM32F429ZI (ARM Cortex-M4, 180 MHz) |
| 서보 드라이버 | XDL-L7SA004BAA (브러시리스 서보) |
| 엔코더 | XML-FBL04AMK1 (12,000 PPR, 4x 쿼드러처) |
| 제어 주기 | 1 ms (SysTick 인터럽트 기반) |
| 조향 범위 | ±45° |
| 위치 분해능 | 0.0075°/count (48,000 count/rev) |
| IDE | STM32CubeIDE 1.19.0 |

---

## 시스템 개요

본 프로젝트는 자율주행 차량의 **조향 각도 하위 제어기(Low-Level Steering Controller)** 펌웨어이다. 상위 제어기(Jetson, 산업용 PC 등)로부터 목표 조향 각도를 수신하고, PID 폐루프 제어를 통해 브러시리스 서보모터를 정밀 위치 제어한다.

```
┌──────────────────────────────────┐
│       상위 제어기 (Main PC)       │
│  센서 융합 / 경로 계획 / 판단     │
│  → 목표 조향 각도 산출            │
└───────────────┬──────────────────┘
                │  UART / CAN / Ethernet
                ▼
┌──────────────────────────────────┐
│     STM32F429ZI (본 프로젝트)     │
│  명령 수신 → PID 제어 → 펄스 출력 │
│  → 현재 위치 피드백               │
└───────────────┬──────────────────┘
                │  PWM Pulse + Direction
                ▼
┌──────────────────────────────────┐
│  XDL-L7SA004BAA 서보 드라이버     │
│  + 브러시리스 모터 + 엔코더       │
│  → 조향 메커니즘 구동             │
└──────────────────────────────────┘
```

---

## 디렉토리 구조

```
autonomouse_control_stm32/
├── Core/
│   ├── Inc/                            # 헤더 파일 (14개)
│   │   ├── main.h                      # MCU 핀 정의, 시스템 매크로
│   │   ├── constants.h                 # 엔코더/펄스/안전 한계 상수
│   │   ├── position_control.h          # PID 제어 인터페이스
│   │   ├── pulse_control.h             # PWM 펄스 출력 인터페이스
│   │   ├── encoder_reader.h            # 엔코더 피드백 인터페이스
│   │   ├── relay_control.h             # 서보 ON/OFF, 비상정지
│   │   ├── adc_potentiometer.h         # 절대 위치 센서
│   │   ├── homing.h                    # 원점 복귀
│   │   ├── tim.h                       # 타이머 설정 (TIM1, TIM2)
│   │   ├── usart.h                     # UART 설정 (USART1, USART3)
│   │   ├── adc.h                       # ADC 설정
│   │   ├── gpio.h                      # GPIO 초기화
│   │   ├── stm32f4xx_it.h              # 인터럽트 핸들러
│   │   └── stm32f4xx_hal_conf.h        # HAL 설정
│   │
│   └── Src/                            # 소스 파일
│       ├── main.c                      # 엔트리 포인트 (데모 루프)
│       ├── main_edit.c                 # 엔트리 포인트 (PID 제어 루프 통합)
│       ├── position_control.c          # PID 제어 알고리즘
│       ├── pulse_control.c             # PWM 펄스 생성
│       ├── encoder_reader.c            # TIM2 쿼드러처 엔코더 읽기
│       ├── relay_control.c             # GPIO 릴레이 제어
│       ├── adc_potentiometer.c         # ADC 절대 위치 읽기
│       ├── homing.c                    # 원점 탐색
│       ├── tim.c                       # 타이머 초기화
│       ├── usart.c                     # UART 초기화
│       ├── adc.c                       # ADC 초기화
│       ├── gpio.c                      # GPIO 초기화
│       ├── stm32f4xx_it.c              # ISR (SysTick, Fault 등)
│       ├── stm32f4xx_hal_msp.c         # HAL MSP 콜백
│       ├── system_stm32f4xx.c          # 시스템 초기화
│       ├── ethernet_communication.c    # 이더넷 (미구현)
│       ├── syscalls.c                  # 표준 라이브러리 지원
│       └── sysmem.c                    # 힙 메모리 관리
│
├── Drivers/                            # STM32 HAL 라이브러리
├── Doc/                                # 설계 문서, 버그 리포트
├── Debug/                              # 빌드 산출물
├── servo_control_baqu.ioc              # STM32CubeMX 설정 파일
└── STM32F429ZITX_FLASH.ld              # 링커 스크립트
```

---

## 소프트웨어 아키텍처

계층화된 모듈 구조로 설계되어 각 모듈이 단일 책임을 갖는다.

```
┌─────────────────────────────────────────────────────┐
│                 Application Layer                    │
│  main_edit.c : 시스템 초기화 + 1ms 제어 루프 실행     │
├─────────────────────────────────────────────────────┤
│                  Control Layer                       │
│  position_control.c : PID 폐루프 위치 제어            │
│  homing.c           : 원점 복귀 시퀀스               │
├──────────┬──────────┬───────────┬───────────────────┤
│  Driver  │  Driver  │  Driver   │  Driver            │
│  Layer   │  Layer   │  Layer    │  Layer             │
│          │          │           │                    │
│ pulse_   │ encoder_ │ relay_    │ adc_               │
│ control  │ reader   │ control   │ potentiometer      │
│ .c       │ .c       │ .c        │ .c                 │
│          │          │           │                    │
│ TIM1 PWM │ TIM2 Enc │ GPIO     │ ADC1               │
│ PE9,PE11 │ PA0,PA1  │ PD14,PD15│ PA4                │
├──────────┴──────────┴───────────┴───────────────────┤
│              HAL Layer (STM32 HAL)                    │
│  tim.c / adc.c / usart.c / gpio.c                    │
├─────────────────────────────────────────────────────┤
│              Hardware (STM32F429ZI)                   │
└─────────────────────────────────────────────────────┘
```

### 모듈 설명

| 모듈 | 파일 | 역할 | 주요 API |
|------|------|------|----------|
| **PID 제어** | `position_control.c/h` | 목표 각도 추종 폐루프 제어 | `PositionControl_Init()`, `_Update()`, `_SetTarget()`, `_Enable()` |
| **펄스 출력** | `pulse_control.c/h` | 서보 드라이버로 PWM 펄스 생성 | `PulseControl_SetFrequency()`, `_SendSteps()`, `_Stop()` |
| **엔코더** | `encoder_reader.c/h` | 모터 회전 위치 피드백 | `EncoderReader_GetAngleDeg()`, `_GetCount()`, `_Reset()` |
| **릴레이** | `relay_control.c/h` | 서보 전원/비상정지 제어 | `Relay_ServoOn()`, `_ServoOff()`, `_Emergency()` |
| **포텐셔미터** | `adc_potentiometer.c/h` | 절대 위치 센서 읽기 | `ADC_Pot_GetAngle()`, `_GetRaw()`, `_Calibrate()` |
| **홈잉** | `homing.c/h` | 엔코더-포텐셔미터 동기화 | `Homing_FindZero()`, `_IsComplete()` |

---

## 하드웨어 인터페이스

### 핀 맵

| 주변장치 | 핀 | 기능 | 설정 |
|---------|-----|------|------|
| TIM1 CH1 | PE9 | 펄스 출력 (→ SN75176 → 서보 드라이버) | PWM, PSC=215, ARR=9 |
| TIM1 CH2 | PE11 | 방향 신호 (CW/CCW) | GPIO Output |
| TIM2 CH1 | PA0 | 엔코더 A상 | Encoder Mode TI12, Rising Edge |
| TIM2 CH2 | PA1 | 엔코더 B상 | Encoder Mode TI12, Rising Edge |
| ADC1 CH4 | PA4 | 포텐셔미터 (절대 위치) | 12-bit, 3 cycles |
| USART1 | PA9/PA10 | 외부 통신 (예약) | 115200 bps, 8N1 |
| USART3 | PD8/PD9 | 디버그 출력 (ST-Link) | 115200 bps, 8N1 |
| GPIO | PD14 | SVON 릴레이 (서보 전원) | Active LOW |
| GPIO | PD15 | EMG 릴레이 (비상정지) | Active LOW, B-Contact |
| GPIO | PB0, PB7, PB14 | LED (LD1, LD2, LD3) | Output |

### 클럭 구성

| 항목 | 값 |
|------|-----|
| HSE | 8 MHz |
| SYSCLK | 180 MHz (PLL: M=4, N=180, P=2) |
| APB1 | 45 MHz (DIV4) |
| APB2 | 90 MHz (DIV2) |
| SysTick | 1 ms |

---

## 제어 알고리즘

### PID 폐루프 제어

1ms 주기로 SysTick 인터럽트 플래그 기반 실행된다.

```
    목표 각도 ──→ ⊕ ──→ [  PID 제어기  ] ──→ [ 펄스 출력 ] ──→ [ 서보모터 ]
                  ↑ -    Kp·e + Ki·∫e + Kd·de/dt                     │
                  │                                                    │
                  └──────────── [ 엔코더 피드백 ] ←────────────────────┘
```

### PID 파라미터

| 파라미터 | 값 | 설명 |
|---------|-----|------|
| Kp | 500.0 | 비례 게인 - 위치 오차에 대한 즉각 반응 |
| Ki | 5.0 | 적분 게인 - 정상상태 오차 제거 |
| Kd | 20.0 | 미분 게인 - 오버슈트 감쇠 |
| Integral Limit | ±1,000 | 적분 와인드업 방지 |
| Output Limit | ±10,000 Hz | 최대 출력 주파수 |
| Position Tolerance | 0.5° | 안정 판단 허용 오차 |
| Stability Time | 100 ms | 오차 < 0.5° 지속 시 안정 판정 |

### 실행 구조 (ISR + Main Loop 패턴)

```c
// stm32f4xx_it.c - 1ms마다 플래그 설정
volatile uint8_t interrupt_flag = 0;
void SysTick_Handler(void) {
    HAL_IncTick();
    interrupt_flag = 1;
}

// main_edit.c - 플래그 확인 후 제어 실행
while (1) {
    if (interrupt_flag == 1) {
        interrupt_flag = 0;
        PositionControl_Update();  // PID 계산 → 펄스 출력
    }
}
```

> ISR 내에서 직접 제어 함수를 호출하지 않고 플래그 방식을 사용하는 이유:
> ISR 내부에서 printf, HAL_UART 등 블로킹 함수 실행을 방지하기 위함 (산업용 임베디드 표준 패턴)

### 제어 루프 상세 흐름

```
SysTick (1ms)
    │
    ▼
interrupt_flag = 1
    │
    ▼ (main loop에서 감지)
PositionControl_Update()
    │
    ├─ EncoderReader_GetAngleDeg()     // TIM2 카운터 → 현재 각도
    │      counter × 360 / 48000
    │
    ├─ error = target_angle - current_angle
    │
    ├─ PID_Calculate(error, dt=0.001)
    │      P = 500.0 × error
    │      I = 5.0 × ∫error (clamped ±1000)
    │      D = 20.0 × Δerror / 0.001
    │      output = P + I + D (clamped ±10000)
    │
    ├─ Safety Check
    │      |current| > 50° → Emergency Stop
    │      |error| > 60°   → Emergency Stop
    │
    ├─ PulseControl_SetFrequency(output)
    │      output > 0 → DIR = CW  (PE11 HIGH)
    │      output < 0 → DIR = CCW (PE11 LOW)
    │      |output| → TIM1 ARR 계산 → PWM 시작
    │
    └─ Stability Check
           |error| < 0.5° 연속 100ms → stable = true
```

---

## 안전 기능

| 기능 | 조건 | 동작 |
|------|------|------|
| 각도 제한 | \|현재 각도\| > 50° | PID 비활성화, 모터 정지 |
| 오차 제한 | \|오차\| > 60° | 비상정지 |
| 비상정지 릴레이 | EMG 핀 (PD15) LOW | 서보 드라이버 전원 차단 |
| 적분 와인드업 방지 | \|적분값\| > 1,000 | 적분항 클램핑 |
| 출력 제한 | \|출력\| > 10,000 Hz | 출력 주파수 클램핑 |
| ARR 범위 보호 | 계산값 < 1 또는 > 0xFFFF | 경계값으로 클램핑 |

---

## 변환 공식

| 변환 | 공식 | 분해능 |
|------|------|--------|
| 엔코더 → 각도 | `count × 360 / 48000` | 0.0075°/count |
| 각도 → 펄스 수 | `angle / 0.003` | 333.33 pulse/° |
| ADC → 전압 | `raw × 3.3 / 4095` | 0.806 mV/LSB |
| ADC → 각도 | 2점 보간 (캘리브레이션) | 가변 |
| 주파수 → ARR | `180MHz / ((PSC+1) × freq) - 1` | — |

---

## 빌드 및 플래시

### 요구 사항

- STM32CubeIDE 1.19.0 이상
- STM32F4 HAL 드라이버 (프로젝트 내 포함)
- ST-Link V2/V3 디버거

### 빌드

1. STM32CubeIDE에서 프로젝트 Import
2. `Project` → `Build All` (Ctrl+B)
3. Debug 폴더에 `.elf` 파일 생성 확인

### 플래시

1. ST-Link 연결
2. `Run` → `Debug As` → `STM32 C/C++ Application`
3. 또는 `Run` → `Run As` → `STM32 C/C++ Application`

---

## 개발 로드맵

### Phase 1: 하드웨어 구동 및 기본 제어 -- 완료

- [x] STM32F429 시스템 초기화 (클럭 180MHz, GPIO, Timer, UART, ADC)
- [x] 서보모터 펄스 구동 (`pulse_forward` / `pulse_reverse`)
- [x] TIM2 쿼드러처 엔코더 읽기
- [x] 릴레이 제어 (SVON, EMG)
- [x] PID 위치 제어 알고리즘 구현
- [x] 1ms 인터럽트 기반 제어 루프 (SysTick + 플래그)
- [x] 포텐셔미터 절대 위치 읽기
- [x] 안전 기능 (각도 제한, 비상정지, 와인드업 방지)
- [x] UART 디버그 출력 (`printf` → USART3)

### Phase 2: 통신 프로토콜 구현 -- 미착수

- [ ] UART 명령 수신부 구현 (USART1 인터럽트 RX)
- [ ] 명령 파서 (`TARGET:30.5\n` → `PositionControl_SetTarget(30.5)`)
- [ ] 상태 피드백 전송 (`POS:29.5\n`, `STATUS:OK,...\n`)
- [ ] 핸드셰이크 / ACK 프로토콜
- [ ] 에러 리포팅 (`ERROR:OVER_LIMIT\n`)

### Phase 3: 상위 시스템 연동 -- 미착수

- [ ] 상위 제어기 ↔ STM32 통신 테스트
- [ ] 프로토콜 정의 (명령/응답 포맷 확정)
- [ ] 에러 처리 및 통신 타임아웃
- [ ] 상태 머신 기반 모드 관리 (IDLE → HOMING → READY → RUNNING → ERROR)

### Phase 4: 고급 기능 -- 미착수

- [ ] CAN 통신 (자동차 산업 표준)
- [ ] Ethernet 통신
- [ ] 속도 제어 모드 추가
- [ ] 진단 기능 (알람 이력, 상태 모니터링)
- [ ] 미구현 API 함수 완성 (13개)

---

## 통신 프로토콜 설계 (Phase 2 예정)

### 명령 (상위 → STM32)

| 명령 | 포맷 | 설명 |
|------|------|------|
| 목표 설정 | `TARGET:30.5\n` | 목표 조향 각도 (°) |
| 서보 ON | `SVON:1\n` | 서보 드라이버 활성화 |
| 서보 OFF | `SVON:0\n` | 서보 드라이버 비활성화 |
| 비상정지 | `EMG:1\n` | 비상정지 |
| 비상정지 해제 | `EMG:0\n` | 비상정지 해제 |
| 상태 요청 | `STATUS?\n` | 현재 상태 조회 |

### 응답 (STM32 → 상위)

| 응답 | 포맷 | 설명 |
|------|------|------|
| 위치 | `POS:29.5\n` | 현재 각도 |
| 전체 상태 | `STATUS:OK,POS:29.5,TARGET:30.0,STABLE:1\n` | 복합 상태 |
| 에러 | `ERROR:OVER_LIMIT\n` | 에러 코드 |
| 수신 확인 | `ACK\n` | 명령 수신 확인 |

---

## 알려진 이슈

| ID | 설명 | 심각도 | 상태 |
|----|------|--------|------|
| BUG-001 | `SetFrequency()` 방향 핀 미설정 | Critical | **해결** |
| BUG-002 | `PositionControl_Update()` 주기적 호출 부재 | Critical | **해결** |
| BUG-003 | `SetFrequency()` PWM Start 누락 | Critical | **해결** |
| BUG-004 | `position_control.h` 선언 vs `.c` 구현 불일치 (13개 함수 미구현) | Major | 미해결 |
| BUG-005 | PID DEFAULT 매크로와 실제 초기값 불일치 | Minor | 미해결 |
| BUG-006 | 안정화 판단 시간 측정 `counter++` 방식 (HAL_GetTick 미사용) | Minor | 미해결 |

상세 내용은 [Doc/problem.md](Doc/problem.md) 참고.

---

## 참고 문서

| 문서 | 설명 |
|------|------|
| [Doc/최종 구조.md](Doc/최종%20구조.md) | 전체 코드 구조 상세 |
| [Doc/향후계획.md](Doc/향후계획.md) | 개발 로드맵 및 통신 프로토콜 설계 |
| [Doc/problem.md](Doc/problem.md) | 버그 리포트 및 해결 이력 |
| [Doc/수정내역_20260119.md](Doc/수정내역_20260119.md) | 코드 수정 내역 |
| [Doc/position_control_study.md](Doc/position_control_study.md) | PID 제어 학습 자료 |
| [Doc/change_code.md](Doc/change_code.md) | 코드 변경 로그 |
