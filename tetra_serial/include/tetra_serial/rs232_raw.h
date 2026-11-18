#ifndef TETRA_SERIAL__RS232_RAW_H_
#define TETRA_SERIAL__RS232_RAW_H_

#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 관리자 화면의 "보드 명령 테스트"가 보드에 임의 바이트를 보내고 응답을 그대로 받을 때 쓴다.
 * 드라이버의 폴링 루프와 같은 스레드에서만 부른다 (다른 스레드에서 부르면 폴링 패킷과 섞인다). */

#define RAW_COMMAND_MAX_LENGTH	256	/* 보낼 데이터 최대 길이 (프로토콜 틀을 씌우기 전) */
#define RAW_FRAME_OVERHEAD		3	/* STX + ETX + LRC */
#define RAW_REPLY_MAX_LENGTH	256	/* 받을 응답 최대 길이 */
#define RAW_TIMEOUT_DEFAULT_MS	300
#define RAW_TIMEOUT_MAX_MS		2000
/* 응답이 이만큼 끊기면 끝난 것으로 본다. 115200bps 에서 한 프레임은 몇 ms 안에 이어서 오므로 넉넉하다 */
#define RAW_IDLE_GAP_MS			30

/* 응답을 어디까지 읽을지 (interfaces/srv/BoardRawCommand.srv 의 READ_* 와 같은 값) */
#define RAW_READ_UNTIL_IDLE		0	/* 응답이 RAW_IDLE_GAP_MS 끊기거나 시간·길이 한도까지 */
#define RAW_READ_ASCII_FRAME	1	/* STX ... ETX LRC (LRC 가 맞는 첫 ETX 에서 끝) */
#define RAW_READ_LENGTH_FRAME	2	/* STX 길이(2) 데이터 ETX LRC (전원 보드 바이너리 응답) */
#define RAW_READ_NONE			3	/* 응답을 기다리지 않는다 */

/* 프레임 판정 결과 */
#define RAW_FRAME_NOT_CHECKED	0	/* UNTIL_IDLE·NONE 은 프레임을 보지 않는다 */
#define RAW_FRAME_OK			1
#define RAW_FRAME_BAD_LRC		2
#define RAW_FRAME_INCOMPLETE	3	/* 시간·길이 한도까지 프레임이 끝나지 않음 */

/* STX + payload + ETX + LRC(payload..ETX 의 XOR) 로 감싼다. 보드 명령과 같은 틀이다 (drive_module.c, power_module.c).
 * 반환: 프레임 길이, 버퍼가 작거나 길이가 잘못되면 -1 */
int raw_build_protocol_frame(const unsigned char *payload, int payload_length, unsigned char frame[], size_t size);

/* data[0..length) 에서 mode 에 맞는 프레임이 끝났는지 본다.
 * 반환: RAW_FRAME_OK / RAW_FRAME_BAD_LRC (프레임 끝까지 받음), RAW_FRAME_INCOMPLETE (더 받아야 함) */
int raw_frame_status(const unsigned char data[], int length, int mode);

/* tx 를 보내고 응답을 reply 에 받는다. 보내기 전에 남은 수신 데이터를 버리고(send_packet),
 * 받은 뒤에는 늦게 오는 바이트가 다음 폴링 응답으로 읽히지 않게 조용해질 때까지 버린다.
 * frame_status 에는 RAW_FRAME_* 를 넣는다.
 * 반환: 받은 바이트 수(0 이상), 인자가 잘못되면 -2, 보내기 실패 -1 */
int raw_exchange(int fd, const unsigned char *tx, int tx_length, unsigned char reply[], size_t reply_size,
	int timeout_ms, int mode, int *frame_status);

#ifdef __cplusplus
}
#endif

#endif  // TETRA_SERIAL__RS232_RAW_H_
