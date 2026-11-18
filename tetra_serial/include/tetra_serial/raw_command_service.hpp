#ifndef TETRA_SERIAL__RAW_COMMAND_SERVICE_HPP_
#define TETRA_SERIAL__RAW_COMMAND_SERVICE_HPP_

// 구동·전원 드라이버의 ~/raw_command 서비스(interfaces/srv/BoardRawCommand) 처리.
// 두 드라이버가 같은 검사·응답 규칙을 쓰도록 여기 한 곳에 둔다. tetra_serial 은 interfaces 에 의존하지 않으므로
// 요청·응답 타입은 템플릿으로 받는다.
//
// 폴링과 섞이지 않는 이유: 두 드라이버 모두 폴링 루프 안에서 rclcpp::spin_some 으로 서비스 콜백을 부른다.
// 그래서 이 함수가 도는 동안에는 폴링 패킷이 나가지 않고, 응답도 폴링 쪽이 읽어 가지 않는다.
// 대신 timeout_ms 만큼 폴링 루프가 멈추므로 한도(RAW_TIMEOUT_MAX_MS)를 둔다.

#include <chrono>
#include <cstdint>
#include <string>
#include <vector>

extern "C" {
#include "tetra_serial/rs232_raw.h"
}

namespace tetra_serial
{

constexpr uint8_t kRawFramingRaw = 0;
constexpr uint8_t kRawFramingProtocol = 1;

/* 요청을 검사해 보낼 바이트를 만든다. 잘못된 요청이면 빈 벡터와 이유를 돌려준다 */
inline std::vector<unsigned char> BuildRawCommandBytes(
  const std::vector<uint8_t> & data, uint8_t framing, std::string & error)
{
  if(data.empty()) { error = "data is empty"; return {}; }
  if(data.size() > RAW_COMMAND_MAX_LENGTH)
  {
    error = "data is longer than " + std::to_string(RAW_COMMAND_MAX_LENGTH) + " bytes";
    return {};
  }
  if(framing == kRawFramingRaw) return std::vector<unsigned char>(data.begin(), data.end());
  if(framing != kRawFramingProtocol) { error = "unknown framing " + std::to_string(framing); return {}; }

  std::vector<unsigned char> frame(data.size() + RAW_FRAME_OVERHEAD);
  const int length = raw_build_protocol_frame(data.data(), static_cast<int>(data.size()), frame.data(), frame.size());
  if(length < 0) { error = "cannot build frame"; return {}; }
  frame.resize(static_cast<size_t>(length));
  return frame;
}

inline const char * RawFrameStatusText(int status)
{
  switch(status)
  {
    case RAW_FRAME_OK: return "frame OK";
    case RAW_FRAME_BAD_LRC: return "frame ETX/LRC mismatch";
    case RAW_FRAME_INCOMPLETE: return "frame incomplete";
    default: return "";
  }
}

/* exchange(tx, tx_length, reply, reply_size, timeout_ms, read_mode, &frame_status) 는 드라이버가 자기 포트로 raw_exchange 를 부르는 함수.
 * board_ready 가 false 면 보내지 않고 refuse_reason 을 돌려준다 (예: 바퀴가 움직이는 중) */
template<typename Request, typename Response, typename ExchangeFn>
void HandleRawCommand(const Request & request, Response & response, ExchangeFn exchange,
  bool board_ready = true, const std::string & refuse_reason = "")
{
  response.success = false;
  response.sent.clear();
  response.reply.clear();
  response.elapsed_ms = 0;

  if(!board_ready) { response.message = refuse_reason; return; }
  const int timeout_ms = request.timeout_ms == 0 ? RAW_TIMEOUT_DEFAULT_MS : static_cast<int>(request.timeout_ms);
  if(timeout_ms > RAW_TIMEOUT_MAX_MS)
  {
    response.message = "timeout_ms must be <= " + std::to_string(RAW_TIMEOUT_MAX_MS);
    return;
  }
  const size_t max_reply = request.max_reply_length == 0 ? RAW_REPLY_MAX_LENGTH : request.max_reply_length;
  if(max_reply > RAW_REPLY_MAX_LENGTH)
  {
    response.message = "max_reply_length must be <= " + std::to_string(RAW_REPLY_MAX_LENGTH);
    return;
  }
  if(request.read_mode > RAW_READ_NONE) { response.message = "unknown read_mode"; return; }

  std::string error;
  const std::vector<unsigned char> tx = BuildRawCommandBytes(request.data, request.framing, error);
  if(tx.empty()) { response.message = error; return; }

  std::vector<unsigned char> reply(max_reply);
  int frame_status = RAW_FRAME_NOT_CHECKED;
  const auto started = std::chrono::steady_clock::now();
  const int received = exchange(tx.data(), static_cast<int>(tx.size()), reply.data(), reply.size(),
    timeout_ms, static_cast<int>(request.read_mode), &frame_status);
  response.elapsed_ms = static_cast<uint32_t>(std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::steady_clock::now() - started).count());

  if(received < 0)
  {
    response.message = received == -2 ? "serial port is not open" : "write to serial port failed";
    return;
  }
  response.sent.assign(tx.begin(), tx.end());
  response.reply.assign(reply.begin(), reply.begin() + received);

  const std::string summary = "sent " + std::to_string(tx.size()) + " bytes, received " + std::to_string(received) + " bytes";
  if(request.read_mode == RAW_READ_NONE)
  {
    response.success = true;
    response.message = summary + " (reply not awaited)";
  }
  else if(received == 0)
  {
    response.message = "no reply within " + std::to_string(timeout_ms) + " ms";
  }
  else if(frame_status == RAW_FRAME_NOT_CHECKED)
  {
    response.success = true;
    response.message = summary;
  }
  else
  {
    response.success = (frame_status == RAW_FRAME_OK);
    response.message = summary + ", " + RawFrameStatusText(frame_status);
  }
}

}  // namespace tetra_serial

#endif  // TETRA_SERIAL__RAW_COMMAND_SERVICE_HPP_
