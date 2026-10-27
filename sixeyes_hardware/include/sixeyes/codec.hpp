// Frame encoding for the SixEyes link: COBS + CRC-16. Same rules as python/src/sixeyes_host/codec.py.
// A frame on the wire is COBS(type | seq | payload | crc16) followed by a 0x00 byte.
#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

namespace sixeyes {

uint16_t crc16(const uint8_t *data, size_t n);  // CRC-16/CCITT-FALSE

std::vector<uint8_t> cobs_encode(const uint8_t *data, size_t n);
// Returns false if the input is not valid COBS.
bool cobs_decode(const uint8_t *data, size_t n, std::vector<uint8_t> &out);

struct Frame {
  uint8_t type = 0;
  uint8_t seq = 0;
  std::vector<uint8_t> payload;
};

// One whole frame, including the trailing 0x00 delimiter.
std::vector<uint8_t> encode_frame(uint8_t type, uint8_t seq, const uint8_t *payload, size_t n);
// Decode one frame without its trailing 0x00. Returns false if it is damaged.
bool decode_frame(const uint8_t *encoded, size_t n, Frame &out);

// Feed it bytes as they arrive; complete, valid frames are appended to `out`.
// Damaged frames are dropped and counted in `errors`.
class FrameParser {
 public:
  void feed(const uint8_t *data, size_t n, std::vector<Frame> &out);
  size_t errors = 0;

 private:
  static constexpr size_t kMaxFrame = 512;
  std::vector<uint8_t> buf_;
};

}  // namespace sixeyes
