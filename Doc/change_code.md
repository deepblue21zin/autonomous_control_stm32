# 코드 변경 이력 (change_code.md)

> position_control 통합 작업 중 발생하는 모든 코드 변경을 기록합니다.
> 각 항목에 **변경 이유**, **변경 전/후 코드**, **영향 범위**를 명시합니다.

---

## 변경 예정 목록

| # | 파일 | 내용 | 상태 |
|---|------|------|------|
| 1 | pulse_control.c | SetFrequency() 방향 핀 버그 수정 | **완료** |
| 2 | stm32f4xx_it.c + main_edit.c | 1ms 주기 PositionControl_Update() 호출 추가 | **완료** |
| 3 | main.c | 데모 루프 → closed-loop 제어 루프 교체 | 대기 |
| 4 | position_control.c | 미구현 함수 구현 (GetTarget, GetError 등) | 대기 |
| 5 | position_control.h / .c | PID DEFAULT 값 불일치 정리 | 대기 |
| 6 | position_control.c | 안정화 판단 로직 개선 | 대기 |

---

## 변경 #1: SetFrequency() 방향 핀 설정 추가 (BUG-001)

**날짜:** 2026-01-31
**파일:** `Core/Src/pulse_control.c` (57~62행)
**관련 버그:** BUG-001

### 변경 이유

PID 제어 출력(`output`)은 양수(정방향)/음수(역방향)로 방향 정보를 포함한다.
`SetFrequency()`가 이 부호를 무시하고 절대값만 사용하여 모터가 항상 한 방향으로만 회전하는 문제.

### 변경 전

```c
void PulseControl_SetFrequency(int32_t freq_hz) {
    if (freq_hz < 0) freq_hz = -freq_hz; // 부호 정보 소실
    if (freq_hz == 0) {
        HAL_TIM_PWM_Stop_IT(p_htim1, TIM_CHANNEL_1);
        return;
    }
    // ... ARR/CCR 설정
}
```

### 변경 후

```c
void PulseControl_SetFrequency(int32_t freq_hz) {
    // [추가] 부호에 따라 방향 핀 설정
    if (freq_hz >= 0) {
        HAL_GPIO_WritePin(DIR_GPIO_PORT, DIR_PIN, GPIO_PIN_SET);   // CW
    } else {
        HAL_GPIO_WritePin(DIR_GPIO_PORT, DIR_PIN, GPIO_PIN_RESET); // CCW
    }
    if (freq_hz < 0) freq_hz = -freq_hz; // 음수 처리
    if (freq_hz == 0) {
        HAL_TIM_PWM_Stop_IT(p_htim1, TIM_CHANNEL_1);
        return;
    }
    // ... ARR/CCR 설정 (기존 유지)
}
```

### 영향 범위

- `position_control.c`의 `PositionControl_Update()`에서 호출하는 `PulseControl_SetFrequency(output)` 정상 동작 가능
- 기존 `pulse_forward()` / `pulse_reverse()`는 `SendSteps()`를 사용하므로 영향 없음

---

## 변경 #2: SetFrequency() PWM Start 추가 + ARR 범위 보호 (BUG-003)

**날짜:** 2026-01-31
**파일:** `Core/Src/pulse_control.c` (59~91행)
**관련 버그:** BUG-003

### 변경 이유

`SetFrequency()`가 ARR/CCR 레지스터만 변경하고 `HAL_TIM_PWM_Start_IT()`를 호출하지 않아서,
PWM이 멈춘 상태에서 호출하면 펄스가 출력되지 않는 문제.
또한 극단적인 주파수 입력 시 ARR 값이 16비트 범위를 벗어나거나 0이 되는 문제도 방지.

### 변경 전

```c
    __HAL_TIM_SET_AUTORELOAD(p_htim1, arr);
    __HAL_TIM_SET_COMPARE(p_htim1, TIM_CHANNEL_1, arr / 2);
    // ← PWM Start 없음, ARR 범위 체크 없음
}
```

### 변경 후

```c
    if (arr > 0xFFFF) arr = 0xFFFF; //16비트 한계
    if (arr < 1) arr = 1;           //최소값 보장 (0 방지)

    __HAL_TIM_SET_AUTORELOAD(p_htim1, arr);
    __HAL_TIM_SET_COMPARE(p_htim1, TIM_CHANNEL_1, arr / 2);

    // [추가] ARR/CCR 설정 후 PWM 시작
    if(freq_hz != 0) {HAL_TIM_PWM_Start_IT(p_htim1, TIM_CHANNEL_1);}
}
```

### 영향 범위

- PWM 정지 상태에서 `SetFrequency()`를 호출해도 정상적으로 펄스 출력 시작
- 목표 도달(output=0 → Stop) 후 새 목표 설정(output!=0) 시 모터 정상 재시작
- ARR 오버플로우/언더플로우 방지

---

## 변경 #3: SysTick 플래그 + main loop에서 PositionControl_Update() 1ms 호출 (BUG-002)

**날짜:** 2026-01-31
**파일:** `Core/Src/stm32f4xx_it.c` (44, 190행), `Core/Src/main_edit.c` (54, 123~127행)
**관련 버그:** BUG-002

### 변경 이유

`PositionControl_Update()`가 프로젝트 어디서도 호출되지 않아 PID 제어가 전혀 실행되지 않음.
1ms 주기로 호출해야 하며, ISR에서 직접 호출하면 내부 printf/HAL 블로킹 함수로 인해 위험하므로 플래그 방식 채택.

### 변경 전

```c
// stm32f4xx_it.c — SysTick_Handler
void SysTick_Handler(void) {
    HAL_IncTick();
    // ← 아무것도 없음
}

// main_edit.c — while(1) 루프
while (1) {
    // ← PositionControl_Update() 호출 없음
    pulse_forward(5000);  // 데모 코드만 존재
    ...
}
```

### 변경 후

```c
// stm32f4xx_it.c — [추가] 플래그 변수 + SysTick에서 설정
volatile uint8_t interrupt_flag = 0;

void SysTick_Handler(void) {
    HAL_IncTick();
    interrupt_flag = 1;  // 1ms마다 플래그 세움
}

// main_edit.c — [추가] extern 선언 + 플래그 확인
extern volatile uint8_t interrupt_flag;

while (1) {
    if (interrupt_flag == 1) {
        interrupt_flag = 0;
        PositionControl_Update();  // 1ms 주기 PID 제어
    }
}
```

### 영향 범위

- `PositionControl_Update()`가 1ms 주기로 실행되어 PID closed-loop 제어 가능
- SysTick ISR은 플래그만 세우므로 실행 시간 영향 최소 (수 ns)
- Update() 내부의 printf, HAL 호출은 main 컨텍스트에서 안전하게 실행
- **주의:** 기존 데모 코드(pulse_forward/reverse, HAL_Delay)가 아직 남아있어 제거 필요
