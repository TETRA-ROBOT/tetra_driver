#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <unistd.h>
#include <termios.h>
#include "drive_module.h"
#include "tetra_serial/rs232_common.h"


int m_check_X = 0;
int m_check_Y = 0;
int m_check_vl = 0;
int m_check_vr = 0;
unsigned int buf_x =0;
unsigned int buf_y =0;
int Left_Vel[8];
int Right_Vel[8];

unsigned int m_Befor_buf_x = 0;
unsigned int m_Befor_buf_y = 0;

unsigned char Left_data[2];
unsigned char Right_data[2];

int drvm_init(const char *device, int time_out)
{
	int fd;

	if((fd = open_port(device)) < 0)
	{
		return -1;
	}

	if(set_terminal(fd, 115200, (int)(time_out/100)) != 0)
	{
		close_port(fd);
		return -1;
	}

	return fd;
}

void drvm_deinit(int fd)
{
	if(fd >= 0) close(fd);
}


int drvm_set_servo(int fd, int mode)
{
	int ret;
	unsigned char packet_buf[256] = {STX, 'D', 'B'};
	
	if(fd < 0) return -1;

	if(mode == 1) {//on
		packet_buf[3] = '1';
		packet_buf[4] = '1';
	}
	else {
		packet_buf[3] = '0';
		packet_buf[4] = '0';
	}
	packet_buf[5] = ETX;
	packet_buf[6] = make_lrc(&packet_buf[1], 5);

	ret = send_packet(fd, packet_buf, 7);
	//fflush(fd);
	if(ret <= 0) return -2;

	memset(packet_buf, 0, sizeof(unsigned char)*255);
	ret = get_response(fd, packet_buf, sizeof(packet_buf));
	//fflush(fd);
	return ret;
}

int drvm_set_velocity_mode(int fd)
{
	int ret;
	unsigned char packet_buf[256] = {STX, 'C', 'Z', '1', '1', ETX};
	
	if(fd < 0) return -1;

	packet_buf[6] = make_lrc(&packet_buf[1], 5);

	ret = send_packet(fd, packet_buf, 7);
	//fflush(fd);
	if(ret <= 0) return -2;

	memset(packet_buf, 0, sizeof(unsigned char)*255);
	ret = get_response(fd, packet_buf, sizeof(packet_buf));
	//fflush(fd);
	return ret;
}

int drvm_set_position_mode(int fd)
{
	int ret;
	unsigned char packet_buf[256] = {STX, 'C', 'Z', '0', '0', ETX};
	
	if(fd < 0) return -1;

	packet_buf[6] = make_lrc(&packet_buf[1], 5);

	ret = send_packet(fd, packet_buf, 7);
	//fflush(fd);
	if(ret <= 0) return -2;

	memset(packet_buf, 0, sizeof(unsigned char)*255);
	ret = get_response(fd, packet_buf, sizeof(packet_buf));
	//fflush(fd);
	return ret;
}


int drvm_set_drive_err_reset(int fd)
{
	int ret;
	unsigned char packet_buf[256] = {STX, 'C', 'G', ETX};
	
	if(fd < 0) return -1;

	packet_buf[4] = make_lrc(&packet_buf[1], 3);

	ret = send_packet(fd, packet_buf, 5);
	//fflush(fd);
	if(ret <= 0) return -2;

	memset(packet_buf, 0, sizeof(unsigned char)*255);
	ret = get_response(fd, packet_buf, sizeof(packet_buf));
	//fflush(fd);
	return ret;
}

int drvm_set_odometry(int fd, int x, int y, int theta)
{
	int ret;
	int index;
	unsigned char packet_buf[256] = {STX, 'C', 'X'};

	if(fd < 0) return -1;

	index = 3;
	ret = int2str(&packet_buf[index], x);
	if(x == 0)
	{
		ret = 1;
	}
	index += ret;
	packet_buf[index] = ';';
	index++;

	ret = int2str(&packet_buf[index], y);
	if(y == 0)
	{
		ret = 1;
	}
	index += ret;
	packet_buf[index] = ';';
	index++;
	ret = int2str(&packet_buf[index], theta);
	if(theta == 0)
	{
		ret = 1;
	}
	index += ret;

	packet_buf[index] = ETX;
	index++;

	packet_buf[index] = make_lrc(&packet_buf[1], index-1);
	index++;

	ret = send_packet(fd, packet_buf, index);
	//fflush(fd);
	if(ret <= 0) return -2;
	
	memset(packet_buf, 0, sizeof(unsigned char)*255);
	ret = get_response(fd, packet_buf, sizeof(packet_buf));
	//fflush(fd);

	return ret;
}

int drvm_set_velocity(int fd, int left, int right)
{
	int ret;
	int index;
	unsigned char packet_buf[255] = {STX, 'B', 'E'};

	if(fd < 0) return -1;

	index = 3;
	ret = int2str(&packet_buf[index], left);
	if(left == 0)
	{
		ret = 1;
	}
	index += ret;
	packet_buf[index] = ';';
	index++;

	ret = int2str(&packet_buf[index], right);
	if(right == 0)
	{
		ret = 1;
	}
	index += ret;

	packet_buf[index] = ETX;
	index++;

	packet_buf[index] = make_lrc(&packet_buf[1], index-1);
	index++;

	ret = send_packet(fd, packet_buf, index);
	//fflush(fd);
	if(ret <= 0) return -2;

	memset(packet_buf, 0, sizeof(unsigned char)*255);
	ret = get_response(fd, packet_buf, sizeof(packet_buf));
	//fflush(fd);

	return ret;
}

int drvm_set_position(int fd, int type, int pass, int data)
{
	int ret;
	int index;
	unsigned char packet_buf[255] = {STX, 'B', 'D'};

	if(fd < 0) return -1;

	int2str(&packet_buf[3], type);
	int2str(&packet_buf[4], pass);
	index = 5;
	ret = int2str(&packet_buf[index], data);
	if(data == 0)
	{
		ret = 1;
	}
	index += ret;

	packet_buf[index] = ETX;
	index++;

	packet_buf[index] = make_lrc(&packet_buf[1], index-1);
	index++;

	ret = send_packet(fd, packet_buf, index);
	//fflush(fd);
	if(ret <= 0) return -2;

	memset(packet_buf, 0, sizeof(unsigned char)*255);
	ret = get_response(fd, packet_buf, sizeof(packet_buf));


	return ret;
}

int drvm_read_bumper_emg(int fd, int *bumper, int *emg_state, int *left_error_code, int *right_error_code)
{
	int ret;
	unsigned char packet_buf[255] = {STX, 'A', 'A', ETX};
	int bumper_val1, bumper_val2;
	int emg_val;
	int binary[16] = {0};

	if(fd < 0) return -1;

	packet_buf[4] = make_lrc(&packet_buf[1], 3);

	ret = send_packet(fd, packet_buf, 5);
	//fflush(fd);
	if(ret <= 0) return -2;

	memset(packet_buf, 0, sizeof(unsigned char)*255);
	ret = get_response(fd, packet_buf, sizeof(packet_buf));
	if(ret !=0 ) return ret;

	//Error Code add...
	memset(binary, 0, sizeof(int)*16);
	*left_error_code = packet_buf[3];
	memset(binary, 0, sizeof(int)*16);
	*right_error_code = packet_buf[5];

	memset(binary, 0, sizeof(int)*16);
	bumper_val1 = packet_buf[7] & 0x0f;
	
	memset(binary, 0, sizeof(int)*16);
	bumper_val2 = packet_buf[6] & 0x0f;

	*bumper = (bumper_val1) + (bumper_val2 << 4);

	memset(binary, 0, sizeof(int)*16);
	emg_val = packet_buf[8];
	decimal2binary(emg_val, binary);
	*emg_state = binary[0];

	return ret;
}

int drvm_read_drive_err(int fd, unsigned char *left_wheel_err, unsigned char *right_wheel_err)
{
	int ret;
	unsigned char packet_buf[256] = {STX, 'A', 'A', ETX};

	if(fd < 0) return -1;

	packet_buf[4] = make_lrc(&packet_buf[1], 3);

	ret = send_packet(fd, packet_buf, 5);
	//fflush(fd);
	if(ret <= 0) return -2;

	memset(packet_buf, 0, sizeof(unsigned char)*255);
	ret = get_response(fd, packet_buf, sizeof(packet_buf));
	if(ret != 0 ) return ret;

	*left_wheel_err = packet_buf[3];
	*right_wheel_err = packet_buf[5];
	return ret;
}

int drvm_read_encoder(int fd, int *left, int *right)
{
	unsigned char packet_buf[256] = {STX, 'A', 'C','0', ETX};
	int ret;
	char tmp[11];

	if(fd < 0) return -1;

	packet_buf[5] = make_lrc(&packet_buf[1], 4);

	ret = send_packet(fd, packet_buf, 6);
	//fflush(fd);
	if(ret <= 0) return -2;

	ret = get_response(fd, packet_buf, sizeof(packet_buf));
	if(ret != 0 ) return ret;

	memcpy(tmp, &packet_buf[2], 10);
	tmp[10] = '\0';
	*left = atoi(tmp);
	memcpy(tmp, &packet_buf[12], 10);
	tmp[10] = '\0';
	*right = atoi(tmp);

	return ret;
}

int drvm_read_odometry(int fd, double *Xpos_mm, double *Ypos_mm, double *deg)
{
	unsigned char packet_buf[256] = {STX, 'A','C','1', ETX};
	int ret;
	char tmp[11];

	if(fd < 0) return -1;

	packet_buf[5] = make_lrc(&packet_buf[1], 4);

	ret = send_packet(fd, packet_buf, 6);
	//fflush(fd);
	if(ret <= 0) return -2;

	ret = get_response(fd, packet_buf, sizeof(packet_buf));
	if(ret != 0 ) return ret;

	memcpy(tmp, &packet_buf[2], 10);
	tmp[10] = '\0';
	*Xpos_mm = atof(tmp);

	memcpy(tmp, &packet_buf[12], 10);
	tmp[10] = '\0';
	*Ypos_mm = atof(tmp);

	memcpy(tmp, &packet_buf[22], 10);
	tmp[10] = '\0';
	*deg = atof(tmp);

	// for(int i=0; i<35; i++)
	// {
	// 	printf("packet_buf[%d]: %x \n", i, packet_buf[i]);
	// }
	// printf("------------------- \n");

	return ret;
}

int drvm_read_parameter(int fd, int param, int *value)
{
	unsigned char packet_buf[256] = {STX, 'X', 'L'};
	int ret;
	int index = 3;
	unsigned int val[255] = {0};

	if(fd < 0) return -1;

	ret = int2str(&packet_buf[index], param);
	index += ret;
	
	packet_buf[index] = ETX;
	index++;
	packet_buf[index] = make_lrc(&packet_buf[1], index-1);
	index++;
	ret = send_packet(fd, packet_buf, index);
	if(ret <= 0) return -2;

	ret = get_response(fd, packet_buf, sizeof(packet_buf));
	if(ret != 0 ) return ret;

	str2int(packet_buf, val); 
	*value = val[0];

	return ret;
}


int drvm_set_parameter(int fd, int param, int value)
{
	unsigned char packet_buf[256] = {STX, 'X', 'B'};
	unsigned char packet_buf2[256] = {STX, 'X', 'D', ETX};
	int ret;
	int index = 3;

	if(fd < 0) return -1;

	if(param <= 9)
	{
		packet_buf[index] = '0';
		index ++;
	}

	ret = int2str(&packet_buf[index], param);
	index += ret;
	ret = int2str(&packet_buf[index], value);
	if(value == 0)
	{
		ret = 1;
	}
	index += ret;
	
	packet_buf[index] = ETX;
	index++;
	packet_buf[index] = make_lrc(&packet_buf[1], index-1);
	index++;
	ret = send_packet(fd, packet_buf, index);
	if(ret <= 0) return -2;
	ret = get_response(fd, packet_buf, sizeof(packet_buf));
	if(ret) return -1;
	
	packet_buf2[4] = make_lrc(&packet_buf2[1], 3);
	ret = send_packet(fd, packet_buf2, 5);
	if(ret <= 0) return -2;

	ret = get_response(fd, packet_buf2, sizeof(packet_buf2));
	if(packet_buf2[1] == 0x34) ret = 0;

	//fflush(fd);

	if(ret) return -1;

	return 0;
}

int drvm_set_Charge_On(int fd)
{
	int ret;
	unsigned char packet_buf[256] = {STX, 'P', 'E'};
	
	if(fd < 0) return -1;

	packet_buf[3] = '2';
	packet_buf[4] = '5';
	packet_buf[5] = '5';
	
	packet_buf[6] = ETX;
	packet_buf[7] = make_lrc(&packet_buf[1], 6);

	ret = send_packet(fd, packet_buf, 8);
	//fflush(fd);
	if(ret <= 0) return -2;

	memset(packet_buf, 0, sizeof(unsigned char)*255);
	ret = get_response(fd, packet_buf, sizeof(packet_buf));
	//fflush(fd);
	return ret;
}

int drvm_set_Charge_Off(int fd)
{
	int ret;
	unsigned char packet_buf[256] = {STX, 'P', 'E'};
	
	if(fd < 0) return -1;

	packet_buf[3] = '1';
	packet_buf[4] = '2';
	packet_buf[5] = '7';
	
	packet_buf[6] = ETX;
	packet_buf[7] = make_lrc(&packet_buf[1], 6);

	ret = send_packet(fd, packet_buf, 8);
	//fflush(fd);
	if(ret <= 0) return -2;

	memset(packet_buf, 0, sizeof(unsigned char)*255);
	ret = get_response(fd, packet_buf, sizeof(packet_buf));
	//fflush(fd);
	return ret;
}

static int s_velocity2_response = DRVM_RESP_INVALID;

/* 버전 응답은 다른 명령과 달리 FLAG 없이 STX + ASCII + ETX + LRC 로 오므로 ETX 까지 읽고 LRC 를 직접 검증한다 */
int drvm_read_version(int fd, char *version)
{
	unsigned char packet_buf[32] = {STX, 'V', 'R', ETX};
	unsigned char received = 0;
	unsigned char lrc = 0;
	int length = 0;

	if(fd < 0) return -1;
	packet_buf[4] = make_lrc(&packet_buf[1], 3);
	if(send_packet(fd, packet_buf, 5) <= 0) return -2;

	do {
		if(read(fd, &received, 1) <= 0) return -3;
	} while(received != STX);

	memset(packet_buf, 0, sizeof(packet_buf));
	while(length < (int)sizeof(packet_buf) - 1) {
		if(read(fd, &packet_buf[length], 1) <= 0) return -3;
		if(packet_buf[length] == ETX) break;
		length++;
	}
	if(read(fd, &lrc, 1) <= 0) return -3;
	if(lrc != make_lrc(packet_buf, length + 1)) return -4;
	if(length != DRVM_VERSION_LENGTH) return -5;

	memcpy(version, packet_buf, DRVM_VERSION_LENGTH);
	version[DRVM_VERSION_LENGTH] = '\0';
	return 0;
}

int drvm_get_velocity2_response(void)
{
	return s_velocity2_response;
}

static int is_legacy_velocity2_ack(const unsigned char *packet_buf)
{
	return packet_buf[0] == 0x02 && packet_buf[1] == 0x32 && packet_buf[2] == 0x03;
}

/* velocity2 응답 길이. 지금 보드는 20바이트(x, y, θ, 좌·우 속도, 범퍼, EMG)이고, 구형 M 구동보드는 속도 4바이트가 없는
 * 16바이트(x, y, θ, 2바이트)를 준다. 20바이트를 기다리면 구형 보드에서는 매 주기 read 타임아웃(0.2초)까지 멈춰
 * 제어 루프가 30Hz 가 아니라 약 5Hz 로 돌고, 조그에서 손을 떼도 한참 더 간다. 그래서 짧은 프레임이 연속으로 오면 길이를 바꾼다. */
#define DRVM_VELOCITY2_FRAME        20
#define DRVM_VELOCITY2_SHORT_FRAME  16
#define DRVM_FRAME_SWITCH_STREAK    3
#define DRVM_FRAME_REVERT_STREAK    10
static int s_velocity2_frame_length = DRVM_VELOCITY2_FRAME;
static int s_frame_mismatch_streak = 0;

static int is_velocity2_frame(const unsigned char *packet_buf, int length)
{
	return packet_buf[0] == 0x02 && (packet_buf[1] == 0x30 || packet_buf[1] == 0x32)
		&& packet_buf[length - 2] == 0x03 && packet_buf[length - 1] == make_lrc((unsigned char *)&packet_buf[1], length - 2);
}

/* 20바이트를 기다리다 타임아웃된 버퍼가 사실은 온전한 16바이트 프레임인지 (나머지는 memset 한 0 그대로) */
static int is_short_frame_in_long_read(const unsigned char *packet_buf)
{
	int i;
	for(i = DRVM_VELOCITY2_SHORT_FRAME; i < DRVM_VELOCITY2_FRAME; i++)
	{
		if(packet_buf[i] != 0) return 0;
	}
	return is_velocity2_frame(packet_buf, DRVM_VELOCITY2_SHORT_FRAME);
}

int drvm_velocity2_frame_length(void)
{
	return s_velocity2_frame_length;
}

static void parse_velocity2_pose(const unsigned char *packet_buf, double *x, double *y, double *theta)
{
	m_check_X = (packet_buf[2] >> 7);
	buf_x = (packet_buf[5] & 0xff) | ((packet_buf[4] << 8) & 0xff00) | ((packet_buf[3] << 16) & 0xff0000) | ((packet_buf[2] << 24) & 0x7f000000);
	*x = (m_check_X == 0) ? (double)buf_x : -1.0 * (double)buf_x;

	m_check_Y = (packet_buf[6] >> 7);
	buf_y = (packet_buf[9] & 0xff) | ((packet_buf[8] << 8) & 0xff00) | ((packet_buf[7] << 16) & 0xff0000) | ((packet_buf[6] << 24) & 0x7f000000);
	*y = (m_check_Y == 0) ? (double)buf_y : -1.0 * (double)buf_y;

	uint16_t raw_theta = (packet_buf[11] & 0xff) | ((packet_buf[10] << 8) & 0xff00);
	*theta = (double)raw_theta;
}

int drvm_set_velocity2(int fd, int left, int right, double *x, double *y, double *theta, double *velocity_l, double *velocity_r, int *bumper, int *emg)
{
	int ret;
	unsigned char packet_buf[255] = {STX, 'B','V'};

	//Left//
	int mag = (left < 0) ? -left : left;
	if (mag > 0x7FFF) mag = 0x7FFF;
	uint8_t sign = (left < 0) ? 0x80 : 0x00;
	Left_data[0] = sign | ((mag >> 8) & 0x7F);
	Left_data[1] = mag & 0xFF;

	//Right//
	int mag2 = (right < 0) ? -right : right;
	if (mag2 > 0x7FFF) mag2 = 0x7FFF;
	uint8_t sign2 = (right < 0) ? 0x80 : 0x00;
	Right_data[0] = sign2 | ((mag2 >> 8) & 0x7F);
	Right_data[1] = mag2 & 0xFF;

	s_velocity2_response = DRVM_RESP_INVALID;
	if(fd < 0) return -1;

	packet_buf[3] = Left_data[0];
	packet_buf[4] = Left_data[1];
	packet_buf[5] = Right_data[0];
	packet_buf[6] = Right_data[1];
	packet_buf[7] = ETX;
	packet_buf[8] = make_lrc(&packet_buf[1], 7);

	ret = send_packet(fd, packet_buf, 9);
	// printf("[write] %02X | %02X | %02X | %02X | %02X | %02X | %02X | %02X | %02X \n", 
	// 		packet_buf[0] ,packet_buf[1], packet_buf[2], packet_buf[3], packet_buf[4], packet_buf[5], packet_buf[6], packet_buf[7], packet_buf[8]);

	if(ret <= 0) return -2;
	memset(packet_buf, 0, sizeof(unsigned char)*255);
	ret = get_response_fixed(fd, packet_buf, sizeof(packet_buf), s_velocity2_frame_length);

	if(s_velocity2_frame_length == DRVM_VELOCITY2_SHORT_FRAME)
	{
		// 구형 프레임에는 속도가 없고 범퍼·EMG 는 호출한 쪽이 'AA' 로 읽으므로 위치만 쓴다
		if(is_velocity2_frame(packet_buf, DRVM_VELOCITY2_SHORT_FRAME))
		{
			s_frame_mismatch_streak = 0;
			s_velocity2_response = DRVM_RESP_OK;
			parse_velocity2_pose(packet_buf, x, y, theta);
			return 0;
		}
		// 보드를 새것으로 바꿨다면 20바이트 프레임의 앞 16바이트만 읽게 되어 계속 어긋난다
		if(!is_legacy_velocity2_ack(packet_buf) && ++s_frame_mismatch_streak >= DRVM_FRAME_REVERT_STREAK)
		{
			s_frame_mismatch_streak = 0;
			s_velocity2_frame_length = DRVM_VELOCITY2_FRAME;
			printf("[drive] velocity2 응답을 20바이트로 다시 읽습니다\n");
		}
	}
	else if(is_short_frame_in_long_read(packet_buf))
	{
		if(++s_frame_mismatch_streak >= DRVM_FRAME_SWITCH_STREAK)
		{
			s_frame_mismatch_streak = 0;
			s_velocity2_frame_length = DRVM_VELOCITY2_SHORT_FRAME;
			printf("[drive] 구형 구동보드의 16바이트 velocity2 응답을 감지했습니다. 이후 16바이트로 읽습니다\n");
		}
		s_velocity2_response = DRVM_RESP_OK;
		parse_velocity2_pose(packet_buf, x, y, theta);
		return 0;
	}

	// FLAG_FAIL(0x32) 프레임도 odometry 를 싣고 오므로 받아들이되, LRC 가 맞을 때만 쓴다.
	// 깨진 프레임으로 x·y·θ 를 덮어쓰면 오도메트리가 한 주기 튄다.
	if(s_velocity2_frame_length == DRVM_VELOCITY2_FRAME && is_velocity2_frame(packet_buf, DRVM_VELOCITY2_FRAME))
	{
		s_frame_mismatch_streak = 0;
		s_velocity2_response = DRVM_RESP_OK;
		parse_velocity2_pose(packet_buf, x, y, theta);

		// new Velocity add... 
		uint16_t raw_velocity_l = (packet_buf[13] & 0xff) | ((packet_buf[12] << 8) & 0x7f00);
		m_check_vl = (packet_buf[12] >> 7);
		if(m_check_vl == 0)
		{
			*velocity_l = (double)raw_velocity_l;
		}
		else
		{
			*velocity_l = (double)raw_velocity_l * -1.0;
		}
		

		uint16_t raw_velocity_r = (packet_buf[15] & 0xff) | ((packet_buf[14] << 8) & 0x7f00);
		m_check_vr = (packet_buf[14] >> 7);
		if(m_check_vr == 0)
		{
			*velocity_r = (double)raw_velocity_r;
		}
		else
		{
			*velocity_r = (double)raw_velocity_r * -1.0;
		}
		

		*bumper = packet_buf[16] & 0xff;
		*emg = packet_buf[17] & 0xff; 

		// printf("[OK] %02X | %02X | %02X | %02X | %02X | %02X | %02X | %02X | %02X | %02X | %02X | %02X | %02X | %02X | %02X | %02X | %02X | %02X | %02X | %02X \n", 
		// 	packet_buf[0] ,packet_buf[1], packet_buf[2], packet_buf[3], packet_buf[4], packet_buf[5], packet_buf[6], packet_buf[7], packet_buf[8],
		// 	packet_buf[9], packet_buf[10], packet_buf[11], packet_buf[12], packet_buf[13], packet_buf[14], packet_buf[15], packet_buf[16], 
		// 	packet_buf[17], packet_buf[18], packet_buf[19]);

	}
	else if(is_legacy_velocity2_ack(packet_buf))
	{
		// 구형 보드는 매 주기마다 이 응답을 주므로 여기서 printf 하면 로그가 넘친다. 경고는 노드에서 throttle 처리.
		s_velocity2_response = DRVM_RESP_LEGACY_ACK;
		return -2;
	}
	else
	{
		printf("Packet Error !!!!!!!, packet_buf[1] = 0x%02X \n", packet_buf[1]);
		printf("[NG] %02X | %02X | %02X | %02X | %02X | %02X | %02X | %02X | %02X | %02X | %02X | %02X | %02X | %02X | %02X | %02X | %02X | %02X | %02X | %02X \n", 
			packet_buf[0] ,packet_buf[1], packet_buf[2], packet_buf[3], packet_buf[4], packet_buf[5], packet_buf[6], packet_buf[7], packet_buf[8],
			packet_buf[9], packet_buf[10], packet_buf[11], packet_buf[12], packet_buf[13], packet_buf[14], packet_buf[15], packet_buf[16], 
			packet_buf[17], packet_buf[18], packet_buf[19]);
		return -2;
	}
	

	return ret;
}
