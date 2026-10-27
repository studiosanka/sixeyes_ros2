// A USB serial (CDC) port. Windows and POSIX (Linux, macOS) implementations are in serial.cpp.
#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>

#include "sixeyes/stream.hpp"

namespace sixeyes {

class SerialPort : public ByteStream {
 public:
  ~SerialPort() override = default;
  // Opens e.g. "COM3" or "/dev/ttyACM0". On failure returns nullptr and fills `error`.
  static std::unique_ptr<SerialPort> open(const std::string &name, std::string &error);
};

}  // namespace sixeyes
