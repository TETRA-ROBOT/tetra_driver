#ifndef TETRA_SERIAL__RS232_H_
#define TETRA_SERIAL__RS232_H_

#ifdef __cplusplus
extern "C" {
#endif

#define COM1	"/dev/ttyS0"
#define COM2	"/dev/ttyS1"
#define TTYUSB0	"/dev/ttyUSB0"
#define TTYUSB1	"/dev/ttyUSB1"
#define TTYUSB2	"/dev/ttyUSB2"
#define TTYUSB3	"/dev/ttyUSB3"

/* 포트를 열고 다른 프로세스가 같은 포트를 열지 못하게 잠근다. 실패하면 -1 */
int open_port(const char *device);
void close_port(int fd);
/* 8N1 raw 모드. time 은 read 타임아웃(0.1초 단위). 실패하면 -1 */
int set_terminal(int fd, int baudrate, int time);

#ifdef __cplusplus
}
#endif

#endif  // TETRA_SERIAL__RS232_H_
