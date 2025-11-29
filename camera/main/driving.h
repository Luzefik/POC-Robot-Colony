#ifdef __cplusplus
extern "C" {
#endif

void motor_init(void);
void motor(int motor_id, int pwm);

#ifdef __cplusplus
}
#endif