#define _POSIX_C_SOURCE 200809L

#include <errno.h>
#include <poll.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include "tetra_serial/rs232_common.h"
#include "tetra_serial/rs232_raw.h"

/* 응답 뒤에 늦게 오는 바이트를 버릴 때 기다리는 최대 시간. 끝없이 데이터를 보내는 장치라도 폴링 루프를 오래 붙잡지 않는다 */
#define RAW_DRAIN_MAX_MS	200

static long now_ms(void)
{
	struct timespec ts;
	clock_gettime(CLOCK_MONOTONIC, &ts);
	return ts.tv_sec * 1000L + ts.tv_nsec / 1000000L;
}

/* timeout_ms 동안 읽을 데이터를 기다린다. 1 = 읽을 수 있음, 0 = 시간 초과, -1 = 오류 */
static int wait_readable(int fd, long timeout_ms)
{
	struct pollfd pfd;
	int ret;

	if(timeout_ms < 0) timeout_ms = 0;
	pfd.fd = fd;
	pfd.events = POLLIN;
	pfd.revents = 0;
	do
	{
		ret = poll(&pfd, 1, (int)timeout_ms);
	} while(ret < 0 && errno == EINTR);
	if(ret <= 0) return ret;
	return (pfd.revents & POLLIN) ? 1 : -1;
}

int raw_build_protocol_frame(const unsigned char *payload, int payload_length, unsigned char frame[], size_t size)
{
	int frame_length = payload_length + RAW_FRAME_OVERHEAD;

	if(payload == NULL || payload_length < 0 || (size_t)frame_length > size) return -1;
	frame[0] = STX;
	if(payload_length > 0) memcpy(&frame[1], payload, (size_t)payload_length);
	frame[payload_length + 1] = ETX;
	frame[payload_length + 2] = make_lrc(&frame[1], payload_length + 1);
	return frame_length;
}

static int find_stx(const unsigned char data[], int length)
{
	int i;

	for(i = 0; i < length; i++)
	{
		if(data[i] == STX) return i;
	}
	return -1;
}

/* 데이터 안에 0x03 이 섞인 응답(구동 보드 velocity2 같은 바이너리)이 있어서, LRC 까지 맞는 ETX 를 프레임 끝으로 본다 */
static int ascii_frame_status(const unsigned char data[], int length, int stx)
{
	int etx;
	int status = RAW_FRAME_INCOMPLETE;

	for(etx = stx + 1; etx + 1 < length; etx++)
	{
		if(data[etx] != ETX) continue;
		if(data[etx + 1] == make_lrc((unsigned char *)&data[stx + 1], etx - stx)) return RAW_FRAME_OK;
		status = RAW_FRAME_BAD_LRC;
	}
	return status;
}

/* get_response_length_prefixed 와 같은 틀: STX, 길이 상위·하위, 데이터, ETX, LRC(데이터만 XOR) */
static int length_frame_status(const unsigned char data[], int length, int stx)
{
	int data_length;
	int frame_length;
	int i;
	unsigned char lrc = 0;

	if(length < stx + 3) return RAW_FRAME_INCOMPLETE;
	data_length = (data[stx + 1] << 8) | data[stx + 2];
	frame_length = data_length + 5;
	if(length < stx + frame_length) return RAW_FRAME_INCOMPLETE;
	for(i = 0; i < data_length; i++) lrc ^= data[stx + 3 + i];
	if(data[stx + frame_length - 2] != ETX || data[stx + frame_length - 1] != lrc) return RAW_FRAME_BAD_LRC;
	return RAW_FRAME_OK;
}

int raw_frame_status(const unsigned char data[], int length, int mode)
{
	int stx;

	if(mode != RAW_READ_ASCII_FRAME && mode != RAW_READ_LENGTH_FRAME) return RAW_FRAME_NOT_CHECKED;
	stx = find_stx(data, length);
	if(stx < 0) return RAW_FRAME_INCOMPLETE;
	if(mode == RAW_READ_ASCII_FRAME) return ascii_frame_status(data, length, stx);
	return length_frame_status(data, length, stx);
}

/* 읽기를 멈춰도 되는지. ASCII 프레임은 LRC 가 틀린 ETX 뒤에도 진짜 끝이 올 수 있어 OK 일 때만 멈춘다 */
static int frame_finished(int status, int mode)
{
	if(status == RAW_FRAME_OK) return 1;
	return mode == RAW_READ_LENGTH_FRAME && status == RAW_FRAME_BAD_LRC;
}

static int read_reply(int fd, unsigned char reply[], size_t reply_size, int timeout_ms, int mode)
{
	const long deadline = now_ms() + timeout_ms;
	int length = 0;

	while((size_t)length < reply_size)
	{
		long wait_ms = deadline - now_ms();
		int ready;
		ssize_t got;

		if(wait_ms <= 0) break;
		if(mode == RAW_READ_UNTIL_IDLE && length > 0 && wait_ms > RAW_IDLE_GAP_MS) wait_ms = RAW_IDLE_GAP_MS;
		ready = wait_readable(fd, wait_ms);
		if(ready < 0) break;
		if(ready == 0)
		{
			if(mode == RAW_READ_UNTIL_IDLE && length > 0) break;
			continue;
		}
		got = read(fd, &reply[length], reply_size - (size_t)length);
		if(got < 0 && errno == EINTR) continue;
		if(got <= 0) break;
		length += (int)got;
		if(frame_finished(raw_frame_status(reply, length, mode), mode)) break;
	}
	return length;
}

static void drain_input(int fd)
{
	const long deadline = now_ms() + RAW_DRAIN_MAX_MS;
	unsigned char discard[64];

	while(now_ms() < deadline && wait_readable(fd, RAW_IDLE_GAP_MS) == 1)
	{
		if(read(fd, discard, sizeof(discard)) <= 0) break;
	}
}

int raw_exchange(int fd, const unsigned char *tx, int tx_length, unsigned char reply[], size_t reply_size,
	int timeout_ms, int mode, int *frame_status)
{
	int length = 0;

	if(frame_status != NULL) *frame_status = RAW_FRAME_NOT_CHECKED;
	if(fd < 0 || tx == NULL || tx_length <= 0 || tx_length > RAW_COMMAND_MAX_LENGTH + RAW_FRAME_OVERHEAD) return -2;
	if(timeout_ms < 0 || timeout_ms > RAW_TIMEOUT_MAX_MS || mode < RAW_READ_UNTIL_IDLE || mode > RAW_READ_NONE) return -2;
	if(mode != RAW_READ_NONE && (reply == NULL || reply_size == 0)) return -2;

	if(send_packet(fd, tx, tx_length) != tx_length) return -1;
	if(mode != RAW_READ_NONE)
	{
		length = read_reply(fd, reply, reply_size, timeout_ms, mode);
		if(frame_status != NULL) *frame_status = raw_frame_status(reply, length, mode);
	}
	drain_input(fd);
	return length;
}
