#ifndef TETRA_SERIAL__RS232_COMMON_H_
#define TETRA_SERIAL__RS232_COMMON_H_

#include <stddef.h>

#include "tetra_serial/rs232.h"

#ifdef __cplusplus
extern "C" {
#endif

// Protocol Packet
#define STX				0x02
#define ETX				0x03
#define ACK				0x06
#define NAK				0x15
#define RST				0x12

#define FLAG_OK			0x30 // Processing Success
#define FLAG_ERROR		0x31 // Protocol Error
#define FLAG_FAIL		0x32 // Processing Fail
#define FLAG_NO			0x33 // No Processing
#define FLAG_END		0x34 // End of Packet
#define FLAG_PASS		0x35 // Pass through
#define FLAG_COMPLETE	0x36 // Motion Complete

unsigned char make_lrc(unsigned char data[], int num);
unsigned long mypow(int i, int j);
int int2str(unsigned char data[], int num);
void str2int(unsigned char data[], unsigned int dist[]);
void decimal2binary(int decimal, int binary[]);
void display_error_message(unsigned char flag);

/* 남아 있던 응답을 버리고 패킷 전체를 보낸다.
 * 늦게 도착한 이전 응답을 이번 요청의 응답으로 읽지 않도록 매 요청 전에 수신 버퍼를 비운다.
 * 성공하면 보낸 바이트 수, 실패하면 -1 */
int send_packet(int fd, const unsigned char *data, int length);

/* STX ~ ETX ~ LRC 한 프레임을 읽는다 (ASCII 응답).
 * 반환: 0 = FLAG_OK 이고 LRC 일치, -1 = 무응답·버퍼 초과·FLAG 실패·LRC 불일치 */
int get_response(int fd, unsigned char data[], size_t size);

/* get_response 와 같이 읽되 FLAG·LRC 를 검사하지 않고 읽은 길이(STX~LRC)를 돌려준다.
 * 전원 보드의 일부 응답은 FLAG 자리에 데이터가 와서 호출하는 쪽이 직접 해석한다. 실패하면 -1 */
int get_response_raw(int fd, unsigned char data[], size_t size);

/* STX 부터 frame_length 바이트 고정 길이 응답 (구동 보드 velocity2: 20바이트).
 * 반환: 0 = FLAG_OK 이고 LRC 일치, -1 = 그 밖의 경우. 실패해도 읽은 내용은 data 에 남는다 */
int get_response_fixed(int fd, unsigned char data[], size_t size, int frame_length);

/* STX, 길이 2바이트(상위·하위), 데이터, ETX, LRC 로 된 바이너리 응답 (전원 보드).
 * 반환: 0 = 정상, -1 = LRC 불일치, -10 = STX 없음, -11 = 읽기 실패, -12 = 길이가 버퍼보다 큼 */
int get_response_length_prefixed(int fd, unsigned char data[], size_t size);

#ifdef __cplusplus
}
#endif

#endif  // TETRA_SERIAL__RS232_COMMON_H_
