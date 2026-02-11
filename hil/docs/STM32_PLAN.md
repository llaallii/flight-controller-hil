# STM32F407 Flight Controller — Stage 3 Integration Plan

## 1. Overview

The STM32F407 Discovery/DK board replaces `dummy_fc.py` with **zero changes**
to the MuJoCo bridge or ArduPilot configuration.  The only difference is transport:

| | Stage 2 (Dummy FC) | Stage 3 (STM32) |
|---|---|---|
| **Transport** | UDP localhost | UART Serial (/dev/ttyUSB0) |
| **sysid / compid** | 3 / 1 | 3 / 1 |
| **Messages received** | SET_ATTITUDE_TARGET (82), SCALED_IMU (26) | Same |
| **Messages transmitted** | SERVO_OUTPUT_RAW (36), HEARTBEAT (0) | Same |
| **Control loop** | 1 kHz Python | 1 kHz TIM7 ISR (C) |

## 2. Hardware Configuration

### 2.1 UART (MAVLink transport)
- **Peripheral:** USART2 (PA2 = TX, PA3 = RX) — available on Discovery header.
- **Baud rate:** 921600 (8N1).
- **DMA RX:** DMA1 Stream 5 Channel 4, circular mode, 512-byte buffer.
- **DMA TX:** DMA1 Stream 6 Channel 4, normal mode, triggered per packet.
- **Why DMA:** At 921600 baud a byte arrives every ~11 µs.  Interrupt-per-byte
  would burn 90+ µs/ms of CPU; DMA circular RX costs zero CPU between checks.

### 2.2 Timer ISR (1 kHz control tick)
- **Peripheral:** TIM7 (basic timer, no output compare needed).
- **Prescaler:** (APB1 clock / 1 MHz) − 1  → 1 µs tick.
- **Period:** 999 → interrupt every 1000 µs = 1 kHz.
- **Priority:** High (preempts MAVLink parsing but not DMA).

### 2.3 GPIO (optional LEDs / debug)
- PD12–PD15 (Discovery LEDs): heartbeat blink, failsafe indicator.
- PA0 (user button): manual arm/disarm toggle.

## 3. Firmware Architecture

```
┌─────────────── main.c ───────────────┐
│  HAL_Init()                          │
│  SystemClock_Config()  → 168 MHz     │
│  MX_USART2_UART_Init() → 921600     │
│  MX_DMA_Init()                       │
│  MX_TIM7_Init()       → 1 kHz ISR   │
│  mavlink_init()                      │
│                                      │
│  while (1) {                         │
│      mavlink_poll_rx();  // parse DMA│
│      // low-priority background      │
│  }                                   │
└──────────────────────────────────────┘

┌──── TIM7_IRQHandler (1 kHz) ─────────┐
│  1. Read latest setpoint + gyro      │
│  2. Failsafe check (timestamp)       │
│  3. Rate PID  (roll, pitch, yaw)     │
│  4. Quad-X mixer                     │
│  5. Write motor output to TX buffer  │
│  6. Trigger DMA TX (SERVO_OUTPUT_RAW)│
│  7. Toggle debug LED                 │
└──────────────────────────────────────┘
```

## 4. MAVLink Integration (C)

### 4.1 Library
Use the official MAVLink C headers (header-only, no malloc):
```
git clone --recursive https://github.com/mavlink/c_library_v2.git
```
Include `common/mavlink.h`.  Define channel 0 for USART2.

### 4.2 Parsing (in main loop)
```c
// Pseudo-code — called from while(1)
void mavlink_poll_rx(void) {
    // Check how many new bytes DMA has written
    uint16_t head = UART_RX_BUF_SIZE - __HAL_DMA_GET_COUNTER(&hdma_usart2_rx);
    while (rx_tail != head) {
        uint8_t byte = rx_buf[rx_tail];
        rx_tail = (rx_tail + 1) % UART_RX_BUF_SIZE;

        mavlink_message_t msg;
        mavlink_status_t status;
        if (mavlink_parse_char(MAVLINK_COMM_0, byte, &msg, &status)) {
            handle_message(&msg);
        }
    }
}
```

### 4.3 Message Handlers
```c
void handle_message(const mavlink_message_t *msg) {
    switch (msg->msgid) {
    case MAVLINK_MSG_ID_SET_ATTITUDE_TARGET: {  // 82
        mavlink_set_attitude_target_t pkt;
        mavlink_msg_set_attitude_target_decode(msg, &pkt);
        // Store atomically (ISR reads these)
        __disable_irq();
        g_setpoint.p     = pkt.body_roll_rate;
        g_setpoint.q     = pkt.body_pitch_rate;
        g_setpoint.r     = pkt.body_yaw_rate;
        g_setpoint.thrust = pkt.thrust;
        g_setpoint.timestamp_ms = HAL_GetTick();
        __enable_irq();
        break;
    }
    case MAVLINK_MSG_ID_SCALED_IMU: {  // 26
        mavlink_scaled_imu_t pkt;
        mavlink_msg_scaled_imu_decode(msg, &pkt);
        __disable_irq();
        g_imu.gyro_p = pkt.xgyro * 0.001f;  // mrad/s → rad/s
        g_imu.gyro_q = pkt.ygyro * 0.001f;
        g_imu.gyro_r = pkt.zgyro * 0.001f;
        __enable_irq();
        break;
    }
    default:
        break;
    }
}
```

### 4.4 Transmitting Motor Outputs
```c
// Called from TIM7 ISR after mixer
void send_servo_output(const float motors[4]) {
    mavlink_message_t msg;
    uint16_t pwm[4];
    for (int i = 0; i < 4; i++)
        pwm[i] = 1000 + (uint16_t)(motors[i] * 1000.0f);

    mavlink_msg_servo_output_raw_pack(
        FC_SYSID, FC_COMPID, &msg,
        HAL_GetTick() * 1000,  // time_usec
        0,                     // port
        pwm[0], pwm[1], pwm[2], pwm[3],
        0, 0, 0, 0            // servo5-8
    );
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    HAL_UART_Transmit_DMA(&huart2, buf, len);
}
```

## 5. Rate PID (identical logic to dummy_fc.py)

```c
typedef struct {
    float kp, ki, kd, imax;
    float integral;
    float prev_meas;
} pid_t;

float pid_update(pid_t *pid, float sp, float meas, float dt) {
    float err = sp - meas;
    float p = pid->kp * err;

    pid->integral += err * dt;
    float i_lim = pid->imax / fmaxf(pid->ki, 1e-9f);
    pid->integral = fmaxf(-i_lim, fminf(i_lim, pid->integral));
    float i = pid->ki * pid->integral;

    float d_meas = (meas - pid->prev_meas) / dt;
    pid->prev_meas = meas;
    float d = -pid->kd * d_meas;

    return p + i + d;
}
```

## 6. Quad-X Mixer (identical to config.py MIXER matrix)

```c
void quad_x_mix(float thr, float roll, float pitch, float yaw, float out[4]) {
    out[0] = thr - roll + pitch - yaw;  // M1 FR CW
    out[1] = thr + roll - pitch - yaw;  // M2 BL CW
    out[2] = thr + roll + pitch + yaw;  // M3 FL CCW
    out[3] = thr - roll - pitch + yaw;  // M4 BR CCW
    for (int i = 0; i < 4; i++)
        out[i] = fmaxf(0.0f, fminf(1.0f, out[i]));
}
```

## 7. Failsafe

```c
// In TIM7 ISR:
uint32_t age_ms = HAL_GetTick() - g_setpoint.timestamp_ms;
if (age_ms > SETPOINT_TIMEOUT_MS) {
    // Motors off, set CRITICAL state in heartbeat
    for (int i = 0; i < 4; i++) motors[i] = 0.0f;
    failsafe_active = true;
}
```

## 8. Memory / Performance Budget (STM32F407 @ 168 MHz)

| Item | Estimate |
|------|----------|
| MAVLink parsing (per byte) | ~20 ns |
| MAVLink full packet parse | ~5 µs |
| 3× PID update | ~3 µs |
| Mixer | ~0.5 µs |
| DMA TX trigger | ~1 µs |
| **Total per 1 kHz tick** | **~15 µs of 1000 µs** |
| Flash usage (code + MAVLink) | ~40 KB of 1024 KB |
| RAM usage | ~8 KB of 192 KB |

## 9. Transport Swap Checklist

When moving from Stage 2 → Stage 3:

1. Stop `dummy_fc.py`.
2. Flash STM32 with firmware (same PID gains, same MAVLink messages).
3. Connect STM32 USART2 pins to USB-Serial adapter → Linux serial device.
4. Run bridge with: `python bridge.py --stage 3 --serial /dev/ttyUSB0`
5. Verify heartbeat appears in bridge logs.
6. Arm and fly — same ArduPilot mission, same MuJoCo model.

## 10. STM32CubeMX Peripheral Summary

| Peripheral | Configuration |
|------------|--------------|
| RCC | HSE 8 MHz → PLL → 168 MHz SYSCLK |
| USART2 | 921600 baud, 8N1, DMA RX circular + DMA TX normal |
| TIM7 | Prescaler for 1 µs, period 999, update IRQ enabled |
| GPIO PD12 | Output — heartbeat LED toggle at 1 Hz |
| GPIO PD13 | Output — failsafe indicator |
| NVIC | TIM7 priority 1, DMA priority 2, USART2 priority 3 |
