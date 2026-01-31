# 팀원별 수정 요청 리스트

> 자율주행 조향 시스템 — position_control 통합을 위한 모듈별 수정 사항
> 작성일: 2026-01-31
> 작성자: position_control 담당

---

## 전체 모듈 담당 현황

| 모듈 | 파일 | 현재 상태 | 담당 |
|------|------|----------|------|
| position_control | position_control.c/h | PID 작성 완료, 미사용 | **본인** |
| pulse_control | pulse_control.c/h | 기본 동작 완료, 버그 있음 | 팀원 |
| encoder_reader | encoder_reader.c/h | 동작 완료 | 팀원 |
| relay_control | relay_control.c/h | 동작 완료 | 팀원 |
| adc_potentiometer | adc_potentiometer.c/h | 동작 완료 | 팀원 |
| homing | homing.c/h | 동작 완료 | 팀원 |
| main (시스템 통합) | main.c, stm32f4xx_it.c | 데모 루프만 존재 | 협의 필요 |
| UART 통신 | (미생성) | Phase 2 미구현 | 미정 |
| Ethernet 통신 | ethernet_communication.c | 빈 파일 | 미정 |

---

# Part 1. 내일 테스트 동작을 위한 긴급 수정 사항

> closed-loop 위치 제어가 동작하려면 **반드시** 해결해야 하는 항목

| # | 대상 파일 | 수정 내용 | 담당 | 관련 버그 |
|---|----------|----------|------|----------|
| **T-1** | pulse_control.c | `SetFrequency()`에 방향 핀(PE11) 설정 추가 | pulse_control 담당 | BUG-001 |
| **T-2** | pulse_control.c | `SetFrequency()`에 PWM Start 호출 추가 | pulse_control 담당 | BUG-003 |
| **T-3** | main_edit.c 또는 stm32f4xx_it.c | `PositionControl_Update()` 1ms 주기 호출 구현 | 본인 + 협의 | BUG-002 |
| **T-4** | main_edit.c | 데모 루프 → closed-loop 제어 루프 교체 | 본인 | - |

### T-1 상세: pulse_control.c — SetFrequency() 방향 핀 설정

**현재 문제:**
```
PID output = -3000 (역방향 필요)
  → SetFrequency(-3000) 호출
    → 함수 내부에서 -3000을 3000으로 바꿈 (절대값)
    → 방향 핀(PE11) 변경 없음
    → 모터가 항상 같은 방향으로만 회전
```

**필요한 동작:**
```
PID output = -3000 (역방향 필요)
  → SetFrequency(-3000) 호출
    → freq_hz < 0 이므로 DIR 핀 = CCW 방향으로 설정
    → 절대값 3000으로 주파수 설정
    → 모터가 역방향으로 회전
```

**수정 위치:** `Core/Src/pulse_control.c` 57~72행
**참고:** 같은 파일의 `PulseControl_SendSteps()` (77~95행)에 방향 핀 설정 코드가 이미 있음

---

### T-2 상세: pulse_control.c — SetFrequency() PWM Start 누락

**현재 문제:**
```
SetFrequency(0)  → PWM Stop 호출 (정상)
SetFrequency(3000) → ARR/CCR만 변경, PWM Start 안 함
  → PWM이 멈춰있으면 펄스 안 나감
```

**필요한 동작:**
```
SetFrequency(0)  → PWM Stop (기존 유지)
SetFrequency(3000) → ARR/CCR 변경 + PWM Start 호출
```

**수정 위치:** `Core/Src/pulse_control.c` 57~72행

---

### T-3 상세: PositionControl_Update() 1ms 호출

**선택지 3가지 (팀 협의 필요):**

| 방법 | 수정 파일 | 장점 | 단점 |
|------|----------|------|------|
| A. SysTick | stm32f4xx_it.c | 간단, 이미 1ms 주기 | ISR에서 무거운 처리 |
| B. TIM3 인터럽트 | CubeMX 재설정 필요 | 깔끔한 분리 | CubeMX 변경 필요 |
| C. main loop 폴링 | main_edit.c | 가장 쉬움 | 제어 주기 불안정 가능 |

---

### T-4 상세: main_edit.c 루프 교체 (본인 작업)

**현재 (데모):**
```
서보ON → 5000펄스 정방향 → 대기 → 5000펄스 역방향 → 대기 → 서보OFF → 반복
```

**변경 목표 (closed-loop):**
```
서보ON → 제어 활성화 → 목표각도 설정 → [1ms마다 PID Update] → 안정화 대기
```

---

# Part 2. 모듈별 전체 수정/개발 리스트

> 향후계획.md의 Phase 2~4를 고려한 전체 작업 목록

---

## pulse_control 담당

| # | 내용 | 우선순위 | Phase |
|---|------|---------|-------|
| P-1 | `SetFrequency()` 방향 핀 설정 추가 (**긴급**) | **긴급** | 1 |
| P-2 | `SetFrequency()` PWM Start 호출 추가 (**긴급**) | **긴급** | 1 |
| P-3 | `SetFrequency()`와 `SendSteps()` 동시 사용 시 충돌 방지 | 높음 | 1 |
| P-4 | PWM 동작 상태 추적 변수 추가 (중복 Start/Stop 방지) | 중간 | 2 |
| P-5 | 주파수 범위 검증 (0~1MHz 제한, constants.h 참고) | 중간 | 2 |

**참고 사항:**
- `SetFrequency()`는 position_control의 PID 출력용 (연속 속도 제어)
- `SendSteps()`는 기존 데모용 (정해진 펄스 수 전송)
- 두 함수가 같은 TIM1을 사용하므로 동시 호출 시 충돌 가능

---

## encoder_reader 담당

| # | 내용 | 우선순위 | Phase |
|---|------|---------|-------|
| E-1 | TIM2 카운터 오버플로우 처리 (장시간 운전 시) | 중간 | 2 |
| E-2 | 엔코더 신호 이상 감지 (노이즈, 단선) | 중간 | 3 |
| E-3 | 각속도 계산 함수 추가 (position_control D항 개선용) | 낮음 | 3 |
| E-4 | 엔코더 분해능 런타임 설정 기능 | 낮음 | 4 |

**현재 상태:** 기본 동작 정상. 당장 수정 필요 없음.

---

## relay_control 담당

| # | 내용 | 우선순위 | Phase |
|---|------|---------|-------|
| R-1 | 릴레이 상태 읽기 함수 추가 (피드백용) | 낮음 | 2 |
| R-2 | UART 명령으로 SVON/EMG 제어 연동 | 중간 | 2 |

**현재 상태:** 기본 동작 정상. Active LOW 극성 수정 완료 (수정내역_20260119 참고).

---

## adc_potentiometer 담당

| # | 내용 | 우선순위 | Phase |
|---|------|---------|-------|
| A-1 | 캘리브레이션 값 Flash 저장/로드 | 낮음 | 3 |
| A-2 | ADC 노이즈 필터링 (이동평균 등) | 중간 | 2 |

**현재 상태:** 기본 동작 정상.

---

## homing 담당

| # | 내용 | 우선순위 | Phase |
|---|------|---------|-------|
| H-1 | 홈잉 완료 후 position_control 자동 활성화 연동 | 중간 | 2 |
| H-2 | 홈잉 실패 시 재시도 로직 | 낮음 | 3 |

**현재 상태:** 기본 동작 정상.

---

## UART 통신 담당 (신규 개발)

| # | 내용 | 우선순위 | Phase |
|---|------|---------|-------|
| U-1 | UART 수신 인터럽트 설정 (USART1 또는 USART3) | **높음** | 2 |
| U-2 | 수신 버퍼 및 줄바꿈 파싱 구현 | **높음** | 2 |
| U-3 | 명령 파서 구현 (TARGET, SVON, EMG, STATUS) | **높음** | 2 |
| U-4 | 응답 송신 구현 (POS, STATUS, ERROR, ACK) | 높음 | 2 |
| U-5 | 수신 타임아웃 처리 | 중간 | 3 |
| U-6 | 통신 에러 카운터 및 재연결 로직 | 낮음 | 3 |

**프로토콜 설계안 (향후계획.md 참고):**
```
상위→STM32: "TARGET:30.5\n", "SVON:1\n", "EMG:1\n", "STATUS?\n"
STM32→상위: "POS:29.5\n", "STATUS:OK,POS:29.5,TARGET:30.0,STABLE:1\n"
```

---

## Ethernet 통신 담당 (Phase 4)

| # | 내용 | 우선순위 | Phase |
|---|------|---------|-------|
| ET-1 | ethernet_communication.c 구현 | 낮음 | 4 |
| ET-2 | TCP/UDP 프로토콜 선택 및 구현 | 낮음 | 4 |

**현재 상태:** 빈 파일. Phase 4에서 진행.

---

## position_control 담당 (본인)

| # | 내용 | 우선순위 | Phase |
|---|------|---------|-------|
| PC-1 | main_edit.c에서 closed-loop 제어 루프 작성 (**긴급**) | **긴급** | 1 |
| PC-2 | 1ms 주기 호출 메커니즘 구현/연동 (**긴급**) | **긴급** | 1 |
| PC-3 | 미구현 함수 구현 (GetTarget, GetError 등 5개) | 높음 | 2 |
| PC-4 | PID DEFAULT 값 헤더/소스 동기화 | 중간 | 1 |
| PC-5 | 안정화 판단 HAL_GetTick() 기반으로 개선 | 중간 | 2 |
| PC-6 | 나머지 미구현 함수 8개 (콜백, 통계 등) | 낮음 | 3 |
| PC-7 | UART 명령 연동 (Phase 2 통신팀과 협업) | 높음 | 2 |

---

# Part 3. Phase별 진행 요약

## Phase 1 — 기본 동작 + closed-loop 테스트 (현재)

```
[긴급] T-1  pulse_control SetFrequency 방향 수정     → pulse 담당
[긴급] T-2  pulse_control SetFrequency PWM Start     → pulse 담당
[긴급] T-3  1ms 주기 Update 호출                     → 본인 + 협의
[긴급] T-4  main_edit.c closed-loop 루프             → 본인
[중간] PC-4 PID DEFAULT 값 정리                      → 본인
```

## Phase 2 — UART 통신 + 상위 시스템 연동

```
[높음] U-1~U-4  UART 통신 수신/파싱/응답             → UART 담당
[높음] PC-3     미구현 함수 5개 (상태 조회용)         → 본인
[높음] PC-7     UART 명령 → position_control 연동    → 본인 + UART 담당
[중간] R-2      릴레이 UART 제어                     → relay 담당
[중간] H-1      홈잉 후 자동 제어 활성화              → homing 담당
[중간] P-4~P-5  pulse 안정성 개선                    → pulse 담당
[중간] A-2      ADC 필터링                          → ADC 담당
[중간] PC-5     안정화 판단 개선                     → 본인
```

## Phase 3 — 안정화 + 에러 처리

```
[중간] E-2     엔코더 이상 감지                      → encoder 담당
[중간] U-5~U-6 통신 에러 처리                        → UART 담당
[낮음] PC-6    나머지 미구현 함수 (콜백, 통계 등)     → 본인
[낮음] E-3     각속도 계산                           → encoder 담당
[낮음] H-2     홈잉 재시도                           → homing 담당
[낮음] A-1     캘리브레이션 Flash 저장                → ADC 담당
```

## Phase 4 — 고급 기능

```
[낮음] ET-1~ET-2  Ethernet 통신                     → Ethernet 담당
[낮음] E-4        엔코더 런타임 설정                  → encoder 담당
       CAN 통신 (추가 개발 필요)
       속도 제어 (추가 개발 필요)
       진단 기능 (추가 개발 필요)
```

---

*마지막 업데이트: 2026-01-31*
