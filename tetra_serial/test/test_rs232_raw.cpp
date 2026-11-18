#include <gtest/gtest.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <chrono>
#include <cstdint>
#include <string>
#include <thread>
#include <vector>

#include "tetra_serial/raw_command_service.hpp"

extern "C" {
#include "tetra_serial/rs232_common.h"
}

namespace
{

using Bytes = std::vector<unsigned char>;

// 보드 쪽 끝(board)에 응답을 미리 써 두면 드라이버 쪽 끝(driver)이 보낸 뒤 그 응답을 읽는다
struct FakeLink
{
  int driver = -1;
  int board = -1;
  FakeLink() { int fds[2]; EXPECT_EQ(socketpair(AF_UNIX, SOCK_STREAM, 0, fds), 0); driver = fds[0]; board = fds[1]; }
  ~FakeLink() { close(driver); if(board >= 0) close(board); }
  void BoardSends(const Bytes & bytes) { EXPECT_EQ(write(board, bytes.data(), bytes.size()), static_cast<ssize_t>(bytes.size())); }
  Bytes BoardReceived(size_t count)
  {
    Bytes bytes(count);
    EXPECT_EQ(read(board, bytes.data(), count), static_cast<ssize_t>(count));
    return bytes;
  }
};

Bytes AsciiFrame(const std::string & body)
{
  Bytes frame(body.size() + RAW_FRAME_OVERHEAD);
  raw_build_protocol_frame(reinterpret_cast<const unsigned char *>(body.data()), static_cast<int>(body.size()), frame.data(), frame.size());
  return frame;
}

Bytes LengthFrame(const Bytes & payload)
{
  Bytes frame = {STX, static_cast<unsigned char>(payload.size() >> 8), static_cast<unsigned char>(payload.size() & 0xff)};
  frame.insert(frame.end(), payload.begin(), payload.end());
  frame.push_back(ETX);
  frame.push_back(make_lrc(const_cast<unsigned char *>(payload.data()), static_cast<int>(payload.size())));
  return frame;
}

struct FakeRequest
{
  std::vector<uint8_t> data;
  uint8_t framing = 0;
  uint8_t read_mode = 0;
  uint16_t timeout_ms = 0;
  uint16_t max_reply_length = 0;
};

struct FakeResponse
{
  bool success = false;
  std::string message;
  std::vector<uint8_t> sent;
  std::vector<uint8_t> reply;
  uint32_t elapsed_ms = 0;
};

}  // namespace

TEST(RawFrame, BuildsSameFrameAsBoardCommands)
{
  // drive_module.c drvm_set_drive_err_reset: STX 'C' 'G' ETX LRC
  const Bytes frame = AsciiFrame("CG");
  ASSERT_EQ(frame.size(), 5u);
  EXPECT_EQ(frame[0], STX);
  EXPECT_EQ(frame[3], ETX);
  EXPECT_EQ(frame[4], 'C' ^ 'G' ^ ETX);
  unsigned char tiny[4];
  EXPECT_EQ(raw_build_protocol_frame(reinterpret_cast<const unsigned char *>("CG"), 2, tiny, sizeof(tiny)), -1);
}

TEST(RawFrame, AsciiFrameEndsAtEtxWithMatchingLrc)
{
  // 0x03 이 데이터에 섞여 있어도 LRC 가 맞는 ETX 까지 읽는다
  Bytes reply = AsciiFrame(std::string("\x30\x01\x03\x05", 4));
  EXPECT_EQ(raw_frame_status(reply.data(), static_cast<int>(reply.size()), RAW_READ_ASCII_FRAME), RAW_FRAME_OK);
  // 데이터 속 0x03 다음 바이트가 LRC 와 다르면 아직 끝이 아니다 (끝까지 못 받으면 BAD_LRC 로 알린다)
  EXPECT_EQ(raw_frame_status(reply.data(), 4, RAW_READ_ASCII_FRAME), RAW_FRAME_INCOMPLETE);
  EXPECT_EQ(raw_frame_status(reply.data(), 5, RAW_READ_ASCII_FRAME), RAW_FRAME_BAD_LRC);
  EXPECT_EQ(raw_frame_status(reply.data(), 2, RAW_READ_ASCII_FRAME), RAW_FRAME_INCOMPLETE);
  reply.back() ^= 0xff;
  EXPECT_EQ(raw_frame_status(reply.data(), static_cast<int>(reply.size()), RAW_READ_ASCII_FRAME), RAW_FRAME_BAD_LRC);
}

TEST(RawFrame, LengthFrameChecksLengthEtxAndLrc)
{
  Bytes reply = LengthFrame({0x00, 0x55, 0x03, 0x02});
  EXPECT_EQ(raw_frame_status(reply.data(), static_cast<int>(reply.size()), RAW_READ_LENGTH_FRAME), RAW_FRAME_OK);
  EXPECT_EQ(raw_frame_status(reply.data(), static_cast<int>(reply.size()) - 1, RAW_READ_LENGTH_FRAME), RAW_FRAME_INCOMPLETE);
  reply.back() ^= 0x01;
  EXPECT_EQ(raw_frame_status(reply.data(), static_cast<int>(reply.size()), RAW_READ_LENGTH_FRAME), RAW_FRAME_BAD_LRC);
}

TEST(RawExchange, SendsBytesAndStopsAtFrameEnd)
{
  FakeLink link;
  const Bytes reply = AsciiFrame("0OK");
  link.BoardSends(reply);
  // 프레임 뒤에 늦게 온 바이트는 다음 폴링 응답으로 읽히지 않게 버린다
  std::thread late([&link]() { std::this_thread::sleep_for(std::chrono::milliseconds(10)); link.BoardSends({0x77}); });
  const Bytes tx = AsciiFrame("VR");
  unsigned char rx[RAW_REPLY_MAX_LENGTH];
  int status = -1;
  const int got = raw_exchange(link.driver, tx.data(), static_cast<int>(tx.size()), rx, sizeof(rx), 500, RAW_READ_ASCII_FRAME, &status);
  late.join();
  ASSERT_EQ(got, static_cast<int>(reply.size()));
  EXPECT_EQ(Bytes(rx, rx + got), reply);
  EXPECT_EQ(status, RAW_FRAME_OK);
  EXPECT_EQ(link.BoardReceived(tx.size()), tx);
  struct pollfd pfd = {link.driver, POLLIN, 0};
  EXPECT_EQ(poll(&pfd, 1, 0), 0);
}

TEST(RawExchange, IdleModeEndsShortlyAfterLastByteAndTimesOutWithoutReply)
{
  FakeLink link;
  const Bytes tx = {'h', 'i'};
  unsigned char rx[16];
  int status = -1;
  link.BoardSends({'a', 'b', 'c'});
  auto started = std::chrono::steady_clock::now();
  int got = raw_exchange(link.driver, tx.data(), 2, rx, sizeof(rx), 1500, RAW_READ_UNTIL_IDLE, &status);
  auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - started).count();
  EXPECT_EQ(got, 3);
  EXPECT_EQ(status, RAW_FRAME_NOT_CHECKED);
  EXPECT_LT(elapsed, 300);

  started = std::chrono::steady_clock::now();
  got = raw_exchange(link.driver, tx.data(), 2, rx, sizeof(rx), 150, RAW_READ_UNTIL_IDLE, &status);
  elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - started).count();
  EXPECT_EQ(got, 0);
  EXPECT_GE(elapsed, 140);
  EXPECT_LT(elapsed, 600);
}

TEST(RawExchange, RejectsOutOfRangeArguments)
{
  FakeLink link;
  unsigned char tx[RAW_COMMAND_MAX_LENGTH + RAW_FRAME_OVERHEAD + 1] = {0};
  unsigned char rx[8];
  EXPECT_EQ(raw_exchange(-1, tx, 1, rx, sizeof(rx), 100, RAW_READ_UNTIL_IDLE, nullptr), -2);
  EXPECT_EQ(raw_exchange(link.driver, tx, sizeof(tx), rx, sizeof(rx), 100, RAW_READ_UNTIL_IDLE, nullptr), -2);
  EXPECT_EQ(raw_exchange(link.driver, tx, 1, rx, sizeof(rx), RAW_TIMEOUT_MAX_MS + 1, RAW_READ_UNTIL_IDLE, nullptr), -2);
  EXPECT_EQ(raw_exchange(link.driver, tx, 1, rx, sizeof(rx), 100, 9, nullptr), -2);
}

TEST(RawCommandService, ValidatesRequestBeforeSending)
{
  int calls = 0;
  auto exchange = [&calls](const unsigned char *, int, unsigned char *, size_t, int, int, int *) { calls++; return 0; };
  FakeResponse response;

  FakeRequest empty;
  tetra_serial::HandleRawCommand(empty, response, exchange);
  EXPECT_FALSE(response.success);
  EXPECT_EQ(response.message, "data is empty");

  FakeRequest too_long;
  too_long.data.assign(RAW_COMMAND_MAX_LENGTH + 1, 'a');
  tetra_serial::HandleRawCommand(too_long, response, exchange);
  EXPECT_FALSE(response.success);

  FakeRequest slow;
  slow.data = {'a'};
  slow.timeout_ms = RAW_TIMEOUT_MAX_MS + 1;
  tetra_serial::HandleRawCommand(slow, response, exchange);
  EXPECT_FALSE(response.success);

  FakeRequest refused;
  refused.data = {'a'};
  tetra_serial::HandleRawCommand(refused, response, exchange, false, "wheels are moving");
  EXPECT_EQ(response.message, "wheels are moving");
  EXPECT_EQ(calls, 0);
}

TEST(RawCommandService, WrapsProtocolFrameAndReportsFrameResult)
{
  FakeLink link;
  link.BoardSends(AsciiFrame("0"));
  FakeRequest request;
  request.data = {'C', 'G'};
  request.framing = tetra_serial::kRawFramingProtocol;
  request.read_mode = RAW_READ_ASCII_FRAME;
  FakeResponse response;
  auto exchange = [&link](const unsigned char * tx, int tx_length, unsigned char * rx, size_t rx_size, int timeout_ms, int mode, int * status) {
    return raw_exchange(link.driver, tx, tx_length, rx, rx_size, timeout_ms, mode, status);
  };
  tetra_serial::HandleRawCommand(request, response, exchange);
  EXPECT_TRUE(response.success) << response.message;
  const Bytes expected_frame = AsciiFrame("CG");
  EXPECT_EQ(response.sent, std::vector<uint8_t>(expected_frame.begin(), expected_frame.end()));
  EXPECT_EQ(response.reply.size(), 4u);
  EXPECT_NE(response.message.find("frame OK"), std::string::npos);

  request.timeout_ms = 100;
  tetra_serial::HandleRawCommand(request, response, exchange);
  EXPECT_FALSE(response.success);
  EXPECT_EQ(response.message, "no reply within 100 ms");
}
