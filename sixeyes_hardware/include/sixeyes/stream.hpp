// Something bytes can be written to and read from: a USB serial port, or a fake one in a test.
#pragma once

#include <cstddef>
#include <cstdint>

namespace sixeyes {

class ByteStream {
 public:
  virtual ~ByteStream() = default;
  // Returns the number of bytes written, or -1 on error.
  virtual long write(const uint8_t *data, size_t n) = 0;
  // Waits up to timeout_ms for data. Returns bytes read (0 = timeout), or -1 on error.
  virtual long read(uint8_t *buf, size_t cap, int timeout_ms) = 0;
};

}  // namespace sixeyes
