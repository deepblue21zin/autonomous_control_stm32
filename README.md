# Autonomous Steering Servo Control System

> STM32F429ZI 기반 자율주행 차량 조향 서보모터 위치 제어 시스템

---

## 1. Project Overview

| 항목 | 내용 |
|------|------|
| **프로젝트명** | servo_control_baqu |
| **목적** | 자율주행 차량의 조향 장치를 정밀 제어하는 임베디드 하위 제어기 개발 |
| **MCU** | STM32F429ZIT6 (ARM Cortex-M4, 180 MHz, FPU) |
| **보드** | NUCLEO-F429ZI |
| **서보 드라이버** | XDL-L7SA004BAA |
| **제어 방식** | PID Closed-Loop Position Control |
| **통신** | UART (디버그: USART3, 상위 시스템: USART1) |
| **IDE** | STM32CubeIDE 1.19.0 |

### 시스템 개요

상위 제어기(Jetson, PC 등)로부터 목표 조향 각도를 수신받아, PID 제어 알고리즘을 통해 서보모터를 정확한 위치로 구동하는 하위 제어기(Sub-Controller)입니다.

```
┌──────────────────────────┐
│   Upper Controller       │   경로 계획 / 장애물 회피 / 조향 각도 계산
│   (Jetson, PC, etc.)     │
└──────────┬───────────────┘
           │  UART / CAN (TARGET: 30.0°)
           ▼
┌──────────────────────────┐
│   STM32F429ZI            │   PID Position Control (1ms cycle)
│   servo_control_baqu     │   Encoder Feedback → Error → PID → Pulse Output
└──────────┬───────────────┘
           │  Pulse + Direction (via SN75176)
           ▼
┌──────────────────────────┐
│   L7 Servo Driver        │   서보 드라이버 → 서보 모터 → 조향 기구
│   + Servo Motor          │   Operating Range: ±45°
└──────────────────────────┘
```

---

## 2. Hardware Specification

### 2.1 MCU

| Parameter | Value |
|-----------|-------|
| MCU | STM32F429ZIT6 |
| Core | ARM Cortex-M4 + FPU |
| Clock | 180 MHz (HSE 8 MHz + PLL) |
| Flash | 2 MB |
| SRAM | 256 KB |

### 2.2 Servo System

| Component | Model | Specification |
|-----------|-------|---------------|
| Servo Driver | XDL-L7SA004BAA | AC Servo, Pulse + Direction 입력 |
| Encoder | XML-FBL04AMK1 | 12,000 PPR (Quadrature × 4 = 48,000 counts/rev) |
| Resolution | - | 1 pulse = 0.003° (Electronic Gear Ratio) |
| Operating Range | - | ±45° |

### 2.3 Pin Map

| Function | Pin | Peripheral | Description |
|----------|-----|-----------|-------------|
| Pulse Output | PE9 | TIM1_CH1 (PWM) | SN75176 → L7 PF+ (Pin 9) |
| Direction Output | PE11 | GPIO_Output | SN75176 → L7 PR+ (Pin 11) |
| Encoder A | PA0 | TIM2_CH1 | Quadrature Encoder Input |
| Encoder B | PA1 | TIM2_CH2 | Quadrature Encoder Input |
| Debug UART TX | PD8 | USART3_TX | ST-Link Virtual COM |
| Debug UART RX | PD9 | USART3_RX | ST-Link Virtual COM |
| Host UART TX | PA9 | USART1_TX | Upper Controller 통신 |
| Host UART RX | PA10 | USART1_RX | Upper Controller 통신 |
| Servo ON Relay | - | GPIO_Output | Active LOW, SVON 제어 |
| Emergency Relay | - | GPIO_Output | Active LOW, EMG 제어 |
| ADC Potentiometer | - | ADC1 | Homing 절대위치 참조용 |

---

## 3. System Architecture

### 3.1 Software Layer Diagram

```
┌─────────────────────────────────────────────────────┐
│                  Application Layer                   │
│              main.c / main_edit.c                    │
│         (System Init, Main Loop, 1ms Scheduler)     │
├─────────────────────────────────────────────────────┤
│                  Control Layer                        │
│              position_control.c                      │
│         (PID Algorithm, Safety Check, State Mgmt)    │
├──────────┬──────────┬──────────┬────────────────────┤
│  Pulse   │ Encoder  │  Relay   │     Homing         │
│  Control │ Reader   │  Control │     Module          │
│  (TIM1)  │ (TIM2)   │  (GPIO)  │  (ADC + Encoder)   │
├──────────┴──────────┴──────────┴────────────────────┤
│               STM32 HAL Driver Layer                 │
│    (TIM, GPIO, UART, ADC, RCC, NVIC, SysTick)      │
├─────────────────────────────────────────────────────┤
│                  Hardware Layer                       │
│    STM32F429ZI → SN75176 → L7 Driver → Servo Motor  │
└─────────────────────────────────────────────────────┘
```

### 3.2 Control Loop Flow

```
SysTick ISR (1ms)          Main Loop
     │                        │
     │  interrupt_flag = 1    │
     │ ──────────────────→    │
     │                        ▼
     │                  ┌─────────────┐
     │                  │ flag == 1?  │──No──→ (idle)
     │                  └──────┬──────┘
     │                         │ Yes
     │                         ▼
     │                  ┌─────────────────────────────┐
     │                  │ PositionControl_Update()     │
     │                  │  1. Read Encoder Angle       │
     │                  │  2. Safety Check             │
     │                  │  3. Error = Target - Current │
     │                  │  4. PID Calculate            │
     │                  │  5. SetFrequency(output)     │
     │                  │  6. Stability Check          │
     │                  └─────────────────────────────┘
```

---

## 4. Module Description

### 4.1 position_control (PID Controller)

PID closed-loop 위치 제어의 핵심 모듈.

| Parameter | Value | Description |
|-----------|-------|-------------|
| Kp | 500.0 | 비례 게인 |
| Ki | 5.0 | 적분 게인 |
| Kd | 20.0 | 미분 게인 |
| Integral Limit | 1000.0 | Anti-Windup 한계 |
| Output Limit | 10,000 Hz | 최대 펄스 주파수 |
| Control Period | 1 ms | SysTick 기반 |
| Position Tolerance | 0.5° | 안정화 판정 오차 범위 |
| Stability Time | 100 ms | 안정화 판정 유지 시간 |

**주요 기능:**
- `PositionControl_Init()` — 초기화 (PID 상태 리셋)
- `PositionControl_Update()` — 1ms 주기 메인 제어 루프
- `PositionControl_SetTarget(float deg)` — 목표 각도 설정 (±45°)
- `PositionControl_Enable()` / `Disable()` — 제어 활성화/비활성화
- `PositionControl_EmergencyStop()` — 비상정지
- `PositionControl_CheckSafety()` — 각도 범위 및 오차 안전 검사

### 4.2 pulse_control (PWM Pulse Generator)

TIM1 PWM 인터럽트를 이용한 정밀 펄스 생성 모듈.

| Function | Description |
|----------|-------------|
| `PulseControl_SetFrequency(int32_t freq_hz)` | 부호 기반 방향 + 주파수 설정 (PID 출력용) |
| `PulseControl_SendSteps(uint32_t steps, MotorDirection dir)` | 지정 개수 펄스 전송 (Open-loop용) |
| `PulseControl_Stop()` | PWM 즉시 정지 |
| `PulseControl_IsBusy()` | 펄스 전송 중 상태 확인 |

**SetFrequency 동작:**
1. `freq_hz` 부호로 방향 핀(PE11) 설정 (양수: CW, 음수: CCW)
2. 절대값으로 ARR 계산: `ARR = TimerClock / ((PSC+1) × freq) - 1`
3. ARR 범위 보호 (1 ~ 0xFFFF)
4. PWM Start

### 4.3 encoder_reader (Position Feedback)

TIM2 Quadrature Encoder 모드를 이용한 위치 피드백 모듈.

| Parameter | Value |
|-----------|-------|
| PPR | 12,000 |
| Quadrature | ×4 (48,000 counts/rev) |
| Resolution | 0.0075°/count |

### 4.4 relay_control (Power Management)

서보 드라이버 전원 및 비상정지 릴레이 제어.

| Function | Description |
|----------|-------------|
| `Relay_ServoOn()` | 서보 드라이버 전원 ON (Active LOW) |
| `Relay_ServoOff()` | 서보 드라이버 전원 OFF |
| `Relay_Emergency()` | 비상정지 릴레이 활성화 |
| `Relay_EmergencyRelease()` | 비상정지 해제 |

### 4.5 homing (Zero Point Calibration)

ADC 포텐셔미터 기반 절대 위치 참조로 엔코더 원점을 설정하는 모듈.

---

## 5. Project Structure

```
servo_control_baqu/
├── Core/
│   ├── Inc/                          # Header files
│   │   ├── main.h                    # CubeMX generated pin definitions
│   │   ├── position_control.h        # PID controller API & data structures
│   │   ├── pulse_control.h           # Pulse generator API
│   │   ├── encoder_reader.h          # Encoder reader API
│   │   ├── relay_control.h           # Relay control API
│   │   ├── homing.h                  # Homing module API
│   │   ├── adc_potentiometer.h       # ADC potentiometer reader
│   │   ├── constants.h               # System constants & parameters
│   │   ├── adc.h                     # ADC peripheral config
│   │   ├── tim.h                     # Timer peripheral config
│   │   ├── usart.h                   # UART peripheral config
│   │   ├── gpio.h                    # GPIO peripheral config
│   │   ├── stm32f4xx_hal_conf.h      # HAL configuration
│   │   └── stm32f4xx_it.h            # Interrupt handler declarations
│   ├── Src/                          # Source files
│   │   ├── main.c                    # Original demo (open-loop test)
│   │   ├── main_edit.c               # Closed-loop control (working copy)
│   │   ├── position_control.c        # PID controller implementation
│   │   ├── pulse_control.c           # PWM pulse generation
│   │   ├── encoder_reader.c          # Encoder reading (TIM2)
│   │   ├── relay_control.c           # Relay GPIO control
│   │   ├── homing.c                  # Zero-point calibration
│   │   ├── adc_potentiometer.c       # ADC reading
│   │   ├── ethernet_communication.c  # Ethernet comm (placeholder)
│   │   ├── stm32f4xx_it.c            # Interrupt handlers (SysTick flag)
│   │   ├── adc.c                     # ADC init (CubeMX)
│   │   ├── tim.c                     # Timer init (CubeMX)
│   │   ├── usart.c                   # UART init (CubeMX)
│   │   ├── gpio.c                    # GPIO init (CubeMX)
│   │   └── system_stm32f4xx.c        # System clock config
│   └── Startup/
│       └── startup_stm32f429zitx.s   # Startup assembly
├── Drivers/
│   ├── CMSIS/                        # ARM CMSIS headers
│   └── STM32F4xx_HAL_Driver/         # ST HAL library
├── Doc/                              # Documentation
│   ├── problem.md                    # Bug reports & resolutions
│   ├── change_code.md                # Code change history
│   ├── team_request.md               # Team task assignments
│   ├── position_control_study.md     # Technical deep dive
│   ├── 향후계획.md                    # Future roadmap
│   ├── 최종 구조.md                   # Final architecture design
│   └── 수정내역_20260119.md           # Modification log
├── servo_control_baqu.ioc            # CubeMX project file
├── STM32F429ZITX_FLASH.ld           # Flash linker script
├── STM32F429ZITX_RAM.ld             # RAM linker script
└── README.md                         # This file
```

---

## 6. Build Environment

| Tool | Version |
|------|---------|
| IDE | STM32CubeIDE 1.19.0 |
| Toolchain | ARM GCC (arm-none-eabi-gcc) |
| HAL | STM32F4xx HAL Driver |
| Target | STM32F429ZITX |
| Debugger | ST-Link V2-1 (onboard NUCLEO) |
| Clock Config | HSE 8 MHz → PLL → 180 MHz SYSCLK |
| APB1 | 45 MHz (÷4) |
| APB2 | 90 MHz (÷2), TIM1 clock = 180 MHz |

### Build

1. STM32CubeIDE에서 프로젝트 Import
2. `servo_control_baqu.ioc`로 CubeMX 설정 확인
3. Build: `Project → Build All` (Ctrl+B)
4. Flash: `Run → Debug` (F11)

---

## 7. Execution Flow

### 7.1 Initialization Sequence

```
main()
  ├── HAL_Init()
  ├── SystemClock_Config()         // 180 MHz
  ├── MX_GPIO_Init()
  ├── MX_USART3_UART_Init()       // Debug (115200 baud)
  ├── MX_USART1_UART_Init()       // Host comm
  ├── MX_ADC1_Init()
  ├── MX_TIM1_Init()              // PWM pulse output
  ├── MX_TIM2_Init()              // Encoder input
  ├── HAL_TIM_PWM_Start()         // TIM1 CH1, CH2
  ├── HAL_TIM_Encoder_Start()     // TIM2 quadrature
  ├── Relay_Init()
  ├── PulseControl_Init()         // DIR pin init
  ├── EncoderReader_Init()
  └── PositionControl_Init()      // PID state reset
```

### 7.2 Main Control Loop

```
while (1) {
    if (interrupt_flag) {              // SysTick 1ms flag
        interrupt_flag = 0;
        PositionControl_Update();      // PID closed-loop
    }
}
```

---

## 8. Development Status

### Completed

| # | Task | Module | Status |
|---|------|--------|--------|
| 1 | Open-loop pulse control (forward/reverse) | pulse_control | Done |
| 2 | Encoder position reading (quadrature) | encoder_reader | Done |
| 3 | Relay control (ServoOn/Off, Emergency) | relay_control | Done |
| 4 | PID algorithm implementation | position_control | Done |
| 5 | Direction pin control in SetFrequency (BUG-001) | pulse_control | Fixed |
| 6 | PWM Start after ARR/CCR config (BUG-003) | pulse_control | Fixed |
| 7 | 1ms SysTick flag + main loop Update call (BUG-002) | stm32f4xx_it / main_edit | Fixed |

### In Progress

| # | Task | Module | Status |
|---|------|--------|--------|
| 8 | Demo code removal in main loop | main_edit | Pending |
| 9 | Unimplemented functions (GetTarget, GetError, etc.) | position_control | Pending |
| 10 | PID DEFAULT value alignment (.h ↔ .c) | position_control | Pending |
| 11 | Stability timer accuracy improvement | position_control | Pending |

### Planned (Future Phases)

| Phase | Task | Status |
|-------|------|--------|
| Phase 2 | UART command receiver & parser | Not started |
| Phase 2 | Status feedback to upper controller | Not started |
| Phase 3 | Upper controller integration test | Not started |
| Phase 3 | Communication protocol definition | Not started |
| Phase 4 | CAN bus communication | Not started |
| Phase 4 | Velocity control mode | Not started |
| Phase 4 | Diagnostics & alarm monitoring | Not started |

---

## 9. Known Issues

자세한 내용은 [Doc/problem.md](Doc/problem.md) 참조.

| ID | Severity | Description | Status |
|----|----------|-------------|--------|
| BUG-001 | Critical | SetFrequency()에서 방향 핀 미설정 → 단방향 회전 | **Resolved** |
| BUG-002 | Critical | PositionControl_Update() 미호출 → PID 미실행 | **Resolved** |
| BUG-003 | Critical | SetFrequency()에서 PWM Start 누락 → 펄스 미출력 | **Resolved** |
| BUG-004 | Major | position_control.h 선언 함수 13개 미구현 | Open |
| BUG-005 | Minor | PID DEFAULT 값 (.h: 2.0/0.1/0.5) ↔ (.c: 500/5/20) 불일치 | Open |
| BUG-006 | Minor | stable_time_ms++ 방식 → 실제 경과시간 미반영 | Open |

---

## 10. Communication Protocol (Planned)

### Command Format (Upper → STM32)

| Command | Format | Description |
|---------|--------|-------------|
| Set Target | `TARGET:30.5\n` | 목표 각도 30.5° |
| Servo ON | `SVON:1\n` | 서보 활성화 |
| Servo OFF | `SVON:0\n` | 서보 비활성화 |
| Emergency Stop | `EMG:1\n` | 비상정지 |
| Status Request | `STATUS?\n` | 상태 요청 |

### Response Format (STM32 → Upper)

| Response | Format | Description |
|----------|--------|-------------|
| Position | `POS:29.5\n` | 현재 각도 |
| Status | `STATUS:OK,POS:29.5,TARGET:30.0,STABLE:1\n` | 전체 상태 |
| Error | `ERROR:OVER_LIMIT\n` | 에러 발생 |
| Acknowledge | `ACK\n` | 명령 수신 확인 |

---

## 11. Safety Features

| Feature | Implementation | Threshold |
|---------|---------------|-----------|
| Angle Limit | `CheckSafety()` | ±50° (hard limit) |
| Error Limit | `CheckSafety()` | 60° max error |
| Emergency Stop | `EmergencyStop()` | 즉시 PWM 정지 + PID 리셋 |
| Servo Relay | `Relay_ServoOff()` | 전원 차단 |
| ARR Overflow Guard | `SetFrequency()` | Clamp 1 ~ 0xFFFF |
| Integral Windup | `PID_Calculate()` | ±1000.0 clamp |
| Output Limit | `PID_Calculate()` | ±10,000 Hz |

---

## 12. Documentation

| Document | Path | Description |
|----------|------|-------------|
| Bug Reports | [Doc/problem.md](Doc/problem.md) | 버그 발견·분석·해결 기록 |
| Change Log | [Doc/change_code.md](Doc/change_code.md) | 코드 변경 이력 (전/후 비교) |
| Team Tasks | [Doc/team_request.md](Doc/team_request.md) | 팀원별 수정 요청 목록 |
| Technical Study | [Doc/position_control_study.md](Doc/position_control_study.md) | PID 제어 기술 분석 |
| Architecture | [Doc/최종 구조.md](Doc/최종%20구조.md) | 최종 시스템 구조 설계 |
| Roadmap | [Doc/향후계획.md](Doc/향후계획.md) | 개발 로드맵 (Phase 1~4) |

---

## 13. License

This software is licensed under terms that can be found in the LICENSE file in the root directory of this software component. STM32 HAL drivers are provided by STMicroelectronics under their respective license terms.
