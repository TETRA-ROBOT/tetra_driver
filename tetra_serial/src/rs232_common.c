#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <termios.h>

#include "tetra_serial/rs232_common.h"

unsigned long mypow(int i, int j)
{
	unsigned long result = 1;
	int k;

	for(k = 0; k < j; k++)
		result *= i;

	return result;
}

/* "123;45;6" 처럼 ';' 로 나뉜 숫자를 ETX 까지 읽어 dist 에 더한다. data 에는 반드시 ETX 가 있어야 한다 */
void str2int(unsigned char data[], unsigned int dist[])
{
	int i, j, k, index;
	char tmp[10];

	i = j = k = index = 0;
	while(1)
	{
		if('0' <= data[i] && data[i] <= '9')
		{
			// unsigned int 를 넘는 자릿수는 버린다 (노이즈로 숫자가 길게 이어져도 tmp 를 넘지 않게)
			if(j < (int)sizeof(tmp)) tmp[j++] = data[i];
		}
		else if(data[i] == ';' || data[i] == ETX)
		{
			for(k = 0; k < j; k++)
			{
				dist[index] += (tmp[k] - '0') * mypow(10, j - k - 1);
			}
			j = 0;
			index++;
		}

		if(data[i] == ETX)
		{
			break;
		}
		i++;
	}
}

int int2str(unsigned char data[], int num)
{
	int index = 0, i, j;
	unsigned char flag = 0;

	if(num < 0)
	{
		data[index++] = '-';
		num = -1 * num;
	}
	data[index] = '0';

	for(i = 9; i >= 0; i--)
	{
		j = num / mypow(10, i);
		num -= j * mypow(10, i);
		if(j > 0 || flag == 1)
		{
			data[index++] = (unsigned char)(j + '0');
			flag = 1;
		}
	}

	return index;
}

void decimal2binary(int decimal, int binary[])
{
	int i = 0;

	while(decimal > 0)
	{
		binary[i] = decimal % 2;
		decimal = decimal / 2;
		i++;
	}
}

unsigned char make_lrc(unsigned char data[], int num)
{
	unsigned char tmp = 0;
	int i;

	for(i = 0; i < num; i++)
		tmp ^= data[i];

	return tmp;
}

void display_error_message(unsigned char flag)
{
	switch(flag)
	{
		case FLAG_ERROR :
			printf("Protocol Error\n");
			break;
		case FLAG_FAIL :
			printf("Proccessing Fail\n");
			break;
		case FLAG_NO :
			printf("NO Proccessing\n");
			break;
		case FLAG_END :
			printf("End of Packet\n");
			break;
		case FLAG_PASS :
			printf("Pass through\n");
			break;
		case FLAG_COMPLETE :
			printf("Motion Complete\n");
			break;
		default :
			printf("Unknown Error\n");
			break;
	}
}

int send_packet(int fd, const unsigned char *data, int length)
{
	int sent = 0;

	if(fd < 0) return -1;
	tcflush(fd, TCIFLUSH);
	while(sent < length)
	{
		int ret = write(fd, data + sent, length - sent);
		if(ret <= 0) return -1;
		sent += ret;
	}
	return sent;
}

static int wait_stx(int fd, unsigned char data[])
{
	do
	{
		if(read(fd, data, 1) <= 0) return -1;
	} while(data[0] != STX);
	return 0;
}

/* STX 부터 ETX 다음의 LRC 까지 읽는다. 끝까지 ETX 가 없으면 버퍼를 넘기 전에 포기한다 */
static int read_frame(int fd, unsigned char data[], size_t size)
{
	size_t index = 1;

	if(size < 3 || wait_stx(fd, data) != 0) return -1;
	do
	{
		if(index >= size) return -1;
		if(read(fd, &data[index], 1) <= 0) return -1;
		index++;
	} while(data[index - 2] != ETX);

	return (int)index;
}

int get_response(int fd, unsigned char data[], size_t size)
{
	int length = read_frame(fd, data, size);

	if(length < 0) return -1;
	if(data[1] != FLAG_OK) return -1;
	if(data[length - 1] != make_lrc(&data[1], length - 2)) return -1;
	return 0;
}

int get_response_raw(int fd, unsigned char data[], size_t size)
{
	return read_frame(fd, data, size);
}

int get_response_fixed(int fd, unsigned char data[], size_t size, int frame_length)
{
	int read_bytes = 0;

	if(frame_length < 3 || (size_t)frame_length > size) return -1;
	if(wait_stx(fd, data) != 0) return -1;

	// 고정 길이 응답은 ETX 위치로 끝을 찾으면 매 주기 read 타임아웃까지 기다리게 된다
	while(read_bytes < frame_length - 1)
	{
		int ret = read(fd, &data[1 + read_bytes], frame_length - 1 - read_bytes);
		if(ret <= 0) return -1;
		read_bytes += ret;
	}

	if(data[1] != FLAG_OK) return -1;
	if(data[frame_length - 1] != make_lrc(&data[1], frame_length - 2)) return -1;
	return 0;
}

int get_response_length_prefixed(int fd, unsigned char data[], size_t size)
{
	int index;
	int total_byte;
	int frame_length;
	unsigned char lrc = 0;

	if(size < 5 || wait_stx(fd, data) != 0) return -10;

	if(read(fd, &data[1], 1) <= 0 || read(fd, &data[2], 1) <= 0) return -11;
	total_byte = (data[2] & 0xff) | ((data[1] << 8) & 0xff00);

	// STX + 길이 2 + 데이터 + ETX + LRC. 데이터 안의 0x02 를 STX 로 잘못 잡으면 길이가 터무니없이 커진다
	frame_length = total_byte + 5;
	if((size_t)frame_length > size) return -12;

	for(index = 3; index < frame_length; index++)
	{
		if(read(fd, &data[index], 1) <= 0) return -11;
	}

	for(index = 0; index < total_byte; index++)
	{
		lrc ^= data[index + 3];
	}
	if(data[frame_length - 1] != lrc)
	{
		printf("lrc check error (length %d, received %02x, calculated %02x)\n", total_byte, data[frame_length - 1], lrc);
		return -1;
	}
	return 0;
}
