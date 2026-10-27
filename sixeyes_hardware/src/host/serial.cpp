#include "sixeyes/serial.hpp"

#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#include <windows.h>

namespace sixeyes {
namespace {

class WinPort : public SerialPort {
 public:
  explicit WinPort(HANDLE h) : h_(h) {}
  ~WinPort() override { CloseHandle(h_); }

  long write(const uint8_t *data, size_t n) override {
    DWORD written = 0;
    if (!WriteFile(h_, data, static_cast<DWORD>(n), &written, nullptr)) return -1;
    return static_cast<long>(written);
  }

  long read(uint8_t *buf, size_t cap, int timeout_ms) override {
    // MAXDWORD / MAXDWORD / N: return at once if bytes are waiting, else wait up to N ms for the first one.
    COMMTIMEOUTS to = {};
    to.ReadIntervalTimeout = MAXDWORD;
    to.ReadTotalTimeoutMultiplier = MAXDWORD;
    to.ReadTotalTimeoutConstant = timeout_ms > 0 ? static_cast<DWORD>(timeout_ms) : 1;
    SetCommTimeouts(h_, &to);
    DWORD got = 0;
    if (!ReadFile(h_, buf, static_cast<DWORD>(cap), &got, nullptr)) return -1;
    return static_cast<long>(got);
  }

 private:
  HANDLE h_;
};

}  // namespace

std::unique_ptr<SerialPort> SerialPort::open(const std::string &name, std::string &error) {
  const std::string path = "\\\\.\\" + name;  // needed for COM10 and above
  HANDLE h = CreateFileA(path.c_str(), GENERIC_READ | GENERIC_WRITE, 0, nullptr, OPEN_EXISTING, 0, nullptr);
  if (h == INVALID_HANDLE_VALUE) {
    error = "could not open " + name + " (is it in use, or the wrong port?)";
    return nullptr;
  }
  DCB dcb = {};
  dcb.DCBlength = sizeof(dcb);
  if (!GetCommState(h, &dcb)) {
    CloseHandle(h);
    error = name + " is not a serial port";
    return nullptr;
  }
  dcb.BaudRate = CBR_115200;  // ignored by USB CDC, but must be valid
  dcb.ByteSize = 8;
  dcb.Parity = NOPARITY;
  dcb.StopBits = ONESTOPBIT;
  dcb.fBinary = TRUE;
  dcb.fDtrControl = DTR_CONTROL_ENABLE;  // some boards only send data when DTR is high
  if (!SetCommState(h, &dcb)) {
    CloseHandle(h);
    error = "could not configure " + name;
    return nullptr;
  }
  return std::make_unique<WinPort>(h);
}

}  // namespace sixeyes

#else  // POSIX
#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>

namespace sixeyes {
namespace {

class PosixPort : public SerialPort {
 public:
  explicit PosixPort(int fd) : fd_(fd) {}
  ~PosixPort() override { ::close(fd_); }

  long write(const uint8_t *data, size_t n) override {
    size_t sent = 0;
    while (sent < n) {
      const ssize_t r = ::write(fd_, data + sent, n - sent);
      if (r < 0) {
        if (errno == EAGAIN || errno == EINTR) {
          pollfd p = {fd_, POLLOUT, 0};
          ::poll(&p, 1, 100);
          continue;
        }
        return -1;
      }
      sent += static_cast<size_t>(r);
    }
    return static_cast<long>(sent);
  }

  long read(uint8_t *buf, size_t cap, int timeout_ms) override {
    pollfd p = {fd_, POLLIN, 0};
    const int ready = ::poll(&p, 1, timeout_ms);
    if (ready < 0) return errno == EINTR ? 0 : -1;
    if (ready == 0) return 0;
    const ssize_t r = ::read(fd_, buf, cap);
    if (r < 0) return (errno == EAGAIN || errno == EINTR) ? 0 : -1;
    return static_cast<long>(r);
  }

 private:
  int fd_;
};

}  // namespace

std::unique_ptr<SerialPort> SerialPort::open(const std::string &name, std::string &error) {
  const int fd = ::open(name.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd < 0) {
    error = "could not open " + name + " (wrong port, or no permission: on Linux add yourself to the 'dialout' group)";
    return nullptr;
  }
  termios tio = {};
  if (tcgetattr(fd, &tio) != 0) {
    ::close(fd);
    error = name + " is not a serial port";
    return nullptr;
  }
  cfmakeraw(&tio);
  cfsetispeed(&tio, B115200);  // ignored by USB CDC, but must be valid
  cfsetospeed(&tio, B115200);
  tio.c_cflag |= CLOCAL | CREAD;
  tio.c_cc[VMIN] = 0;
  tio.c_cc[VTIME] = 0;
  if (tcsetattr(fd, TCSANOW, &tio) != 0) {
    ::close(fd);
    error = "could not configure " + name;
    return nullptr;
  }
  return std::make_unique<PosixPort>(fd);
}

}  // namespace sixeyes
#endif
