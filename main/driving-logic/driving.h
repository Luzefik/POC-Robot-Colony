#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MOTOR_MAX_DUTY 511 /* LEDC 9-bit */

void motor_init(void);

/* Raw channel access: motor_id 0..3, pwm clamped to [0, MOTOR_MAX_DUTY].
 * Channel pairs on the L298N: (0 fwd, 1 rev) = left side,
 * (2 fwd, 3 rev) = right side. */
void motor(int motor_id, int pwm);

/* Signed per-side commands: cmd in [-MOTOR_MAX_DUTY, MOTOR_MAX_DUTY],
 * positive = forward. The opposite channel of the pair is always zeroed. */
void drive_left(int cmd);
void drive_right(int cmd);
void drive_stop(void);

static inline int clampi(int x, int min, int max) {
    if (x < min)
        return min;
    if (x > max)
        return max;
    return x;
}

#ifdef __cplusplus
}
#endif
