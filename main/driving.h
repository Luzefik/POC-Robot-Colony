#ifdef __cplusplus
extern "C" {
#endif

#include "dots_algo.h"

void motor_init(void);
void motor(int motor_id, int pwm);
void mov(detection_data_t *data);

#ifdef __cplusplus
}
#endif