#ifndef CAR_CONFIG_H
#define CAR_CONFIG_H
#ifndef CAR_COMM_DIAGNOSTIC
#define CAR_COMM_DIAGNOSTIC 0
#endif
#define CAR_WIRELESS_UART UART_8
#define CAR_WIRELESS_TX UART8_TX_D16
#define CAR_WIRELESS_RX UART8_RX_D17
#define CAR_WIRELESS_BAUD 115200u
#define CAR_WIRELESS_RTS D26
#ifndef CAR_WIRELESS_AUTO_BAUD
#define CAR_WIRELESS_AUTO_BAUD 1
#endif
#ifndef CAR_WIRELESS_USE_RTS
#define CAR_WIRELESS_USE_RTS 1
#endif
/* School example pin mapping; verify wiring and physical polarity before driving. */
#ifndef CAR_SIMULATED
#define CAR_SIMULATED 0
#endif
#ifndef CAR_DISPLAY_ENABLED
#define CAR_DISPLAY_ENABLED 1
#endif
#define CAR_ENCODER_QUADRATURE 1 /* 0: pulse + direction; 1: quadrature */
#define CAR_LEFT_ENCODER QTIMER1_ENCODER1
#define CAR_LEFT_A QTIMER1_ENCODER1_CH1_C0
#define CAR_LEFT_B QTIMER1_ENCODER1_CH2_C1
#define CAR_RIGHT_ENCODER QTIMER1_ENCODER2
#define CAR_RIGHT_A QTIMER1_ENCODER2_CH1_C2
#define CAR_RIGHT_B QTIMER1_ENCODER2_CH2_C24
#define CAR_LEFT_ENCODER_SIGN (-1)
#define CAR_RIGHT_ENCODER_SIGN (1)
#define CAR_LEFT_PWM PWM2_MODULE0_CHA_C6
#define CAR_LEFT_DIR C7
#define CAR_RIGHT_PWM PWM2_MODULE1_CHA_C8
#define CAR_RIGHT_DIR C9
#define CAR_LEFT_FORWARD GPIO_HIGH
/* 2026-10-17 bench recheck: HIGH drove backward with negative encoder counts. */
#define CAR_RIGHT_FORWARD GPIO_LOW
#define CAR_MOTOR_HZ 17000
#define CAR_SERVO_PWM PWM4_MODULE2_CHA_C30
#define CAR_SERVO_HZ 50
/* Conservative starting pulse range, NOT measured BDS300 mechanical limits. */
#define CAR_SERVO_CENTER_US 1500.0f
#define CAR_SERVO_MIN_US 1350.0f
#define CAR_SERVO_MAX_US 1650.0f
#define CAR_STEER_SIGN (-1.0f)
#define CAR_STEER_KP 3.0f
#define CAR_STEER_KD 0.08f
#define CAR_SPEED_TARGET 20.0f /* encoder counts / 10 ms, not cm/s */
#define CAR_SPEED_KP 40.0f
#define CAR_SPEED_KI 100.0f
#define CAR_PWM_LIMIT 2000.0f /* SeekFree scale 0..10000: limit to 20% */
#define CAR_FRAME_TIMEOUT_MS 150u
#define CAR_FOREGROUND_TIMEOUT_MS 100u
#define CAR_SCREEN_PERIOD_MS 200u
#define CAR_TELEMETRY_PERIOD_MS 500u
#endif
