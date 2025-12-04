#ifdef __cplusplus
extern "C" {
#endif

void motor_init(void);
void motor(int motor_id, int pwm);

#define MOTOR_MAX_DUTY 1023

#ifdef __cplusplus
}
#endif