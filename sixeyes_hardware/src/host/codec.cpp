#include "sixeyes/codec.hpp"

#include <utility>

namespace sixeyes {

uint16_t crc16(const uint8_t *data, size_t n) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < n; i++) {
    crc ^= static_cast<uint16_t>(data[i]) << 8;
    for (int b = 0; b < 8; b++) {
      crc = (crc & 0x8000) ? static_cast<uint16_t>((crc << 1) ^ 0x1021) : static_cast<uint16_t>(crc << 1);
    }
  }
  return crc;
}

std::vector<uint8_t> cobs_encode(const uint8_t *data, size_t n) {
  std::vector<uint8_t> out(1, 0);  // placeholder for the first block's length byte
  size_t code_idx = 0;
  uint8_t code = 1;
  for (size_t i = 0; i < n; i++) {
    if (data[i] == 0) {
      out[code_idx] = code;
      code_idx = out.size();
      out.push_back(0);
      code = 1;
    } else {
      out.push_back(data[i]);
      code++;
      if (code == 0xFF) {
        out[code_idx] = code;
        code_idx = out.size();
        out.push_back(0);
        code = 1;
      }
    }
  }
  out[code_idx] = code;
  return out;
}

bool cobs_decode(const uint8_t *data, size_t n, std::vector<uint8_t> &out) {
  out.clear();
  size_t i = 0;
  while (i < n) {
    const uint8_t code = data[i];
    if (code == 0) return false;
    const size_t end = i + code;
    if (end > n) return false;
    out.insert(out.end(), data + i + 1, data + end);
    i = end;
    if (i < n && code != 0xFF) out.push_back(0);
  }
  return true;
}

std::vector<uint8_t> encode_frame(uint8_t type, uint8_t seq, const uint8_t *payload, size_t n) {
  std::vector<uint8_t> body;
  body.reserve(n + 4);
  body.push_back(type);
  body.push_back(seq);
  if (n) body.insert(body.end(), payload, payload + n);
  const uint16_t crc = crc16(body.data(), body.size());
  body.push_back(static_cast<uint8_t>(crc & 0xFF));
  body.push_back(static_cast<uint8_t>(crc >> 8));
  std::vector<uint8_t> out = cobs_encode(body.data(), body.size());
  out.push_back(0);
  return out;
}

bool decode_frame(const uint8_t *encoded, size_t n, Frame &out) {
  std::vector<uint8_t> body;
  if (!cobs_decode(encoded, n, body) || body.size() < 4) return false;
  const size_t len = body.size();
  const uint16_t got = static_cast<uint16_t>(body[len - 2] | (body[len - 1] << 8));
  if (crc16(body.data(), len - 2) != got) return false;
  out.type = body[0];
  out.seq = body[1];
  out.payload.assign(body.begin() + 2, body.end() - 2);
  return true;
}

void FrameParser::feed(const uint8_t *data, size_t n, std::vector<Frame> &out) {
  for (size_t i = 0; i < n; i++) {
    if (data[i] != 0) {
      if (buf_.size() < kMaxFrame) buf_.push_back(data[i]);
      continue;
    }
    if (buf_.empty()) continue;  // empty frame: ignore (also resyncs after noise)
    Frame f;
    if (decode_frame(buf_.data(), buf_.size(), f)) {
      out.push_back(std::move(f));
    } else {
      errors++;
    }
    buf_.clear();
  }
}

}  // namespace sixeyes
