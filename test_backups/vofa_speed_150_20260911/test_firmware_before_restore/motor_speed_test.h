#ifndef MOTOR_SPEED_TEST_H
#define MOTOR_SPEED_TEST_H

// TEMPORARY bench test. Set to 0 and rebuild/reflash to restore normal driving.
#ifndef MOTOR_SPEED_TEST_ENABLE
#define MOTOR_SPEED_TEST_ENABLE (1)
#endif
#define MOTOR_SPEED_TEST_TARGET_CM_S (150.0f)
#define MOTOR_SPEED_TEST_TIMEOUT_MS  (10000U)
#define MOTOR_SPEED_TEST_SEND_MS     (20U)

#endif
