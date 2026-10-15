
#include "rs232.h"

/*
*******************************************************************************
*                             FUNCTION PROTOTYPES
*******************************************************************************
*/

int drvm_init(const char *port, int time_out);
void drvm_deinit(int fd);
int drvm_set_servo(int fd, int mode);
int drvm_set_velocity_mode(int fd);
int drvm_set_position_mode(int fd);
int drvm_set_drive_err_reset(int fd);
int drvm_set_odometry(int fd, int x, int y, int theta);
int drvm_set_velocity(int fd, int left, int right);
int drvm_set_position(int fd, int type, int pass, int data);
int drvm_set_parameter(int fd, int param, int value);
int drvm_read_bumper_emg(int fd, int *bumper, int *emg_state, int *left_error_code, int *right_error_code);
int drvm_read_drive_err(int fd, unsigned char *left_wheel_err, unsigned char *right_wheel_err);
int drvm_read_encoder(int fd, int *left, int *right);
int drvm_read_parameter(int fd, int param, int *value);
int drvm_read_odometry(int fd, double *Xpos_mm, double *Ypos_mm, double *deg);
int drvm_set_Charge_On(int fd);
int drvm_set_Charge_Off(int fd);
int drvm_set_velocity2(int fd, int left, int right, double *x, double *y, double *theta, double *velocity_l, double *velocity_r, int *bumper, int *emg);

/* drvm_set_velocity2() 의 마지막 응답 분류.
 * 반환값(ret)은 정상 응답에서도 음수일 수 있어 보드 호환성 판별에 쓸 수 없으므로 별도로 노출한다. */
#define DRVM_RESP_OK          0  /* 신형 보드: odometry 포함 20바이트 응답 */
#define DRVM_RESP_LEGACY_ACK  1  /* 구형 보드: velocity2 미지원, 02 32 03 LRC 짧은 ACK 만 회신 */
#define DRVM_RESP_INVALID     2  /* 무응답(보드 미연결) 또는 깨진 패킷 */
int drvm_get_velocity2_response(void);
