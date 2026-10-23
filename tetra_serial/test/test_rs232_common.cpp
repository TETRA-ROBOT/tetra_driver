#include <gtest/gtest.h>
#include <unistd.h>

#include <cstring>
#include <vector>

extern "C" {
#include "tetra_serial/rs232_common.h"
}

namespace
{

// 파이프에 보드 응답을 써 두고 읽는 쪽 fd 를 돌려준다. 쓰는 쪽을 닫아 두므로 데이터가 끝나면 read 가 0 (타임아웃과 같음)
int FeedBytes(const std::vector<unsigned char> & bytes)
{
  int fds[2];
  EXPECT_EQ(pipe(fds), 0);
  EXPECT_EQ(write(fds[1], bytes.data(), bytes.size()), static_cast<ssize_t>(bytes.size()));
  close(fds[1]);
  return fds[0];
}

std::vector<unsigned char> AsciiFrame(unsigned char flag, const char * payload)
{
  std::vector<unsigned char> frame = {STX, flag};
  frame.insert(frame.end(), payload, payload + std::strlen(payload));
  frame.push_back(ETX);
  frame.push_back(make_lrc(&frame[1], static_cast<int>(frame.size()) - 1));
  return frame;
}

}  // namespace

TEST(GetResponse, AcceptsValidFrameAfterNoise)
{
  std::vector<unsigned char> bytes = {0x41, 0x42};
  auto frame = AsciiFrame(FLAG_OK, "123");
  bytes.insert(bytes.end(), frame.begin(), frame.end());
  int fd = FeedBytes(bytes);
  unsigned char buf[32] = {0};
  EXPECT_EQ(get_response(fd, buf, sizeof(buf)), 0);
  EXPECT_EQ(buf[2], '1');
  close(fd);
}

TEST(GetResponse, RejectsFailFlagAndBadLrc)
{
  unsigned char buf[32];
  int fd = FeedBytes(AsciiFrame(FLAG_FAIL, "1"));
  EXPECT_EQ(get_response(fd, buf, sizeof(buf)), -1);
  close(fd);

  auto frame = AsciiFrame(FLAG_OK, "1");
  frame.back() ^= 0xFF;
  fd = FeedBytes(frame);
  EXPECT_EQ(get_response(fd, buf, sizeof(buf)), -1);
  close(fd);
}

TEST(GetResponse, StopsAtBufferEndWhenEtxNeverComes)
{
  std::vector<unsigned char> bytes = {STX};
  bytes.insert(bytes.end(), 64, 'A');
  int fd = FeedBytes(bytes);
  unsigned char buf[24];
  std::memset(buf, 0xEE, sizeof(buf));
  // 앞 8바이트만 버퍼로 넘긴다. 나머지가 그대로면 넘쳐 쓰지 않은 것
  EXPECT_EQ(get_response_raw(fd, buf, 8), -1);
  for (size_t i = 8; i < sizeof(buf); i++) {
    EXPECT_EQ(buf[i], 0xEE) << "overwrote byte " << i;
  }
  close(fd);
}

TEST(GetResponseRaw, ReturnsFrameLengthWithoutFlagCheck)
{
  int fd = FeedBytes(AsciiFrame('9', "12"));
  unsigned char buf[32];
  EXPECT_EQ(get_response_raw(fd, buf, sizeof(buf)), 6);  // STX 9 1 2 ETX LRC
  close(fd);
}

TEST(GetResponseFixed, ReadsFixedLengthFrame)
{
  std::vector<unsigned char> frame(20, 0x11);
  frame[0] = STX;
  frame[1] = FLAG_OK;
  frame[18] = ETX;
  frame[19] = make_lrc(&frame[1], 18);
  int fd = FeedBytes(frame);
  unsigned char buf[32];
  EXPECT_EQ(get_response_fixed(fd, buf, sizeof(buf), 20), 0);
  close(fd);

  fd = FeedBytes(frame);
  EXPECT_EQ(get_response_fixed(fd, buf, 10, 20), -1);  // 버퍼보다 긴 프레임은 읽지 않는다
  close(fd);
}

TEST(GetResponseLengthPrefixed, ChecksLengthAgainstBuffer)
{
  std::vector<unsigned char> frame = {STX, 0x00, 0x03, 'a', 'b', 'c', ETX, 0};
  frame[7] = 'a' ^ 'b' ^ 'c';
  int fd = FeedBytes(frame);
  unsigned char buf[32];
  EXPECT_EQ(get_response_length_prefixed(fd, buf, sizeof(buf)), 0);
  close(fd);

  // 데이터 안의 0x02 를 STX 로 잘못 잡아 길이가 0x7F00 처럼 읽히는 경우
  std::vector<unsigned char> bogus = {STX, 0x7F, 0x00, 'x', 'y'};
  fd = FeedBytes(bogus);
  EXPECT_EQ(get_response_length_prefixed(fd, buf, sizeof(buf)), -12);
  close(fd);
}

TEST(Str2Int, IgnoresDigitsBeyondTenWithoutOverflow)
{
  unsigned char data[] = "123456789012345;7\x03";
  unsigned int values[4] = {0};
  str2int(data, values);
  EXPECT_EQ(values[1], 7u);
}

TEST(SendPacket, WritesWholePacket)
{
  int fds[2];
  ASSERT_EQ(pipe(fds), 0);
  const unsigned char packet[] = {STX, 'A', 'B', ETX, 0x01};
  EXPECT_EQ(send_packet(fds[1], packet, sizeof(packet)), 5);
  EXPECT_EQ(send_packet(-1, packet, sizeof(packet)), -1);
  unsigned char got[5];
  EXPECT_EQ(read(fds[0], got, sizeof(got)), 5);
  EXPECT_EQ(std::memcmp(got, packet, sizeof(packet)), 0);
  close(fds[0]);
  close(fds[1]);
}
