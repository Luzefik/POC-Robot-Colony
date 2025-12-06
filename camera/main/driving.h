#ifdef __cplusplus
extern "C" {
#endif

void motor_init(void);
void motor(int motor_id, int pwm);

#define MOTOR_MAX_DUTY 511

#ifdef __cplusplus
}
#endif