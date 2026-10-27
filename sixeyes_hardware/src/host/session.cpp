#include "sixeyes/session.hpp"

#include <algorithm>
#include <cstring>
#include <limits>
#include <vector>

namespace sixeyes {

std::string statusText(uint8_t status) {
  switch (status) {
    case SIXEYES_STATUS_OK: return "";
    case SIXEYES_STATUS_BAD_VERSION: return "The arm and this software speak different protocol versions. Update the arm's firmware or this software.";
    case SIXEYES_STATUS_NOT_HOMED: return "The arm is not homed yet. Home it first.";
    case SIXEYES_STATUS_NOT_IN_CONTROL: return "Another source is in control of the arm. Disconnect the phone, or switch control to this computer while the arm is still.";
    case SIXEYES_STATUS_BAD_ARGS: return "The arm rejected the values. Check that the numbers are finite and in range.";
    case SIXEYES_STATUS_FAULTED: return "The arm has a fault. Check the e-stop button, then clear the fault and home again.";
    case SIXEYES_STATUS_UNSUPPORTED: return "This arm's firmware does not support that yet.";
    case SIXEYES_STATUS_BUSY: return "The arm is busy moving. Wait until it has stopped.";
    default: return "The arm refused the command (code " + std::to_string(status) + ").";
  }
}

Session::Session(std::unique_ptr<ByteStream> stream, int ack_timeout_ms, int heartbeat_ms)
    : stream_(std::move(stream)), ack_timeout_ms_(ack_timeout_ms), heartbeat_ms_(heartbeat_ms) {
  io_ = std::thread([this] { ioLoop(); });
}

Session::~Session() {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    running_ = false;
  }
  cv_.notify_all();
  if (io_.joinable()) io_.join();
}

// ---------------------------------------------------------------- the reader thread

void Session::ioLoop() {
  uint8_t buf[256];
  auto last_beat = Clock::now();
  for (;;) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!running_) return;
    }
    const long n = stream_->read(buf, sizeof(buf), 20);
    if (n > 0) {
      std::vector<Frame> frames;
      parser_.feed(buf, static_cast<size_t>(n), frames);
      for (const Frame &f : frames) handle(f);
    } else if (n < 0) {
      {
        std::lock_guard<std::mutex> lock(mutex_);
        io_error_ = true;
      }
      cv_.notify_all();
      std::this_thread::sleep_for(std::chrono::milliseconds(20));  // do not spin on a dead port
    }
    const auto now = Clock::now();
    if (now - last_beat >= std::chrono::milliseconds(heartbeat_ms_)) {
      last_beat = now;
      maybeHeartbeat();
    }
  }
}

void Session::handle(const Frame &f) {
  if (f.type == SIXEYES_MSG_STATE) {
    sixeyes_state_t st;
    if (!sixeyes_state_unpack(f.payload.data(), f.payload.size(), &st)) return;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      state_.t_ms = st.t_ms;
      state_.homing_state = st.homing_state;
      state_.control_source = st.control_source;
      state_.faults = st.faults;
      std::memcpy(state_.measured, st.measured, sizeof(state_.measured));
      std::memcpy(state_.target, st.target, sizeof(state_.target));
      have_state_ = true;
      state_count_++;
      state_time_ = Clock::now();
    }
    cv_.notify_all();
  } else if (f.type == SIXEYES_MSG_HELLO_ACK) {
    sixeyes_hello_ack_t h;
    if (!sixeyes_hello_ack_unpack(f.payload.data(), f.payload.size(), &h)) return;
    std::lock_guard<std::mutex> lock(mutex_);
    info_.protocol_version = h.protocol_version;
    info_.fw_major = h.fw_major;
    info_.fw_minor = h.fw_minor;
    info_.fw_patch = h.fw_patch;
    info_.capabilities = h.capabilities;
  } else if (f.type == SIXEYES_MSG_ACK) {
    sixeyes_ack_t a;
    if (!sixeyes_ack_unpack(f.payload.data(), f.payload.size(), &a)) return;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      const int key = (a.ref_type << 8) | a.ref_seq;
      if (a.ref_type == SIXEYES_MSG_HOME && a.status == SIXEYES_STATUS_OK) home_ack_count_ = state_count_;
      auto it = pending_.find(key);
      if (it != pending_.end()) {
        it->second = a.status;  // someone is waiting for this answer
      } else if (a.status != SIXEYES_STATUS_OK && a.ref_type == SIXEYES_MSG_TARGETS) {
        refusal_ = statusText(a.status);  // a streamed command nobody waits for
      }
    }
    cv_.notify_all();
  }
}

void Session::maybeHeartbeat() {
  // Keeps the arm's 200 ms watchdog fed while this computer is in control of a homed arm.
  bool needed = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    needed = have_state_ && state_.homing_state == SIXEYES_HOMING_STATE_READY &&
             state_.control_source == SIXEYES_CONTROL_SOURCE_USB;
  }
  if (needed) sendFrame(SIXEYES_MSG_HEARTBEAT, nextSeq(), nullptr, 0);
}

// ---------------------------------------------------------------- sending

uint8_t Session::nextSeq() {
  std::lock_guard<std::mutex> lock(mutex_);
  return ++seq_;
}

bool Session::sendFrame(uint8_t type, uint8_t seq, const uint8_t *payload, size_t n) {
  const std::vector<uint8_t> bytes = encode_frame(type, seq, payload, n);
  std::lock_guard<std::mutex> lock(write_mutex_);
  size_t sent = 0;
  int stalls = 0;
  while (sent < bytes.size()) {
    const long r = stream_->write(bytes.data() + sent, bytes.size() - sent);
    if (r < 0) return false;
    if (r == 0) {
      if (++stalls > 100) return false;
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      continue;
    }
    sent += static_cast<size_t>(r);
  }
  return true;
}

std::string Session::command(uint8_t type, const uint8_t *payload, size_t n) {
  const uint8_t seq = nextSeq();
  const int key = (type << 8) | seq;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    pending_[key] = -1;  // registered before sending, so an instant reply is not missed
  }
  if (!sendFrame(type, seq, payload, n)) {
    std::lock_guard<std::mutex> lock(mutex_);
    pending_.erase(key);
    return "Could not write to the arm. Check the USB cable.";
  }
  std::unique_lock<std::mutex> lock(mutex_);
  cv_.wait_for(lock, std::chrono::milliseconds(ack_timeout_ms_), [&] { return pending_[key] >= 0 || io_error_; });
  const int status = pending_[key];
  pending_.erase(key);
  if (status < 0) return "The arm did not answer. Check the USB cable and that the arm is powered.";
  return statusText(static_cast<uint8_t>(status));
}

// ---------------------------------------------------------------- commands

std::string Session::connect() {
  sixeyes_hello_t hello = {SIXEYES_PROTOCOL_VERSION};
  uint8_t payload[SIXEYES_SIZE_HELLO];
  sixeyes_hello_pack(&hello, payload);
  const std::string err = command(SIXEYES_MSG_HELLO, payload, sizeof(payload));
  if (!err.empty()) {
    const ArmInfo i = info();
    if (i.protocol_version != 0 && i.protocol_version != SIXEYES_PROTOCOL_VERSION) {
      return "Protocol mismatch: the arm speaks v" + std::to_string(i.protocol_version) + ", this software v" +
             std::to_string(SIXEYES_PROTOCOL_VERSION) +
             (i.protocol_version < SIXEYES_PROTOCOL_VERSION ? ". Update the arm's firmware." : ". Update this software.");
    }
    return err;
  }
  std::unique_lock<std::mutex> lock(mutex_);
  if (!cv_.wait_for(lock, std::chrono::milliseconds(3000), [&] { return have_state_; })) {
    return "The arm connected but sent no data. Reconnect, or restart the arm.";
  }
  return "";
}

std::string Session::setControlUsb() {
  sixeyes_set_control_source_t cmd = {SIXEYES_CONTROL_SOURCE_USB};
  uint8_t payload[SIXEYES_SIZE_SET_CONTROL_SOURCE];
  sixeyes_set_control_source_pack(&cmd, payload);
  return command(SIXEYES_MSG_SET_CONTROL_SOURCE, payload, sizeof(payload));
}

std::string Session::home() { return command(SIXEYES_MSG_HOME, nullptr, 0); }
std::string Session::stop() { return command(SIXEYES_MSG_STOP, nullptr, 0); }
std::string Session::clearFault() { return command(SIXEYES_MSG_CLEAR_FAULT, nullptr, 0); }

std::string Session::waitReady(int timeout_ms) {
  std::unique_lock<std::mutex> lock(mutex_);
  // A state from before the arm accepted the home command could still say "ready" from an earlier homing.
  const bool ok = cv_.wait_for(lock, std::chrono::milliseconds(timeout_ms), [&] {
    return state_count_ > home_ack_count_ && state_.homing_state == SIXEYES_HOMING_STATE_READY;
  });
  if (ok) return "";
  if (state_.faults != 0 || state_.homing_state == SIXEYES_HOMING_STATE_FAULT) {
    return "The arm reported a fault while homing. Check the e-stop button and the joints, then clear the fault.";
  }
  return "Timed out waiting for the arm to finish homing.";
}

std::string Session::sendTargets(const float joints[SIXEYES_JOINT_COUNT]) {
  sixeyes_targets_t cmd;
  std::memcpy(cmd.joints, joints, sizeof(cmd.joints));
  uint8_t payload[SIXEYES_SIZE_TARGETS];
  sixeyes_targets_pack(&cmd, payload);
  return command(SIXEYES_MSG_TARGETS, payload, sizeof(payload));
}

void Session::sendTargetsNoWait(const float joints[SIXEYES_JOINT_COUNT]) {
  sixeyes_targets_t cmd;
  std::memcpy(cmd.joints, joints, sizeof(cmd.joints));
  uint8_t payload[SIXEYES_SIZE_TARGETS];
  sixeyes_targets_pack(&cmd, payload);
  sendFrame(SIXEYES_MSG_TARGETS, nextSeq(), payload, sizeof(payload));
}

// ---------------------------------------------------------------- reading

bool Session::state(ArmState &out) const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!have_state_) return false;
  out = state_;
  return true;
}

double Session::stateAgeSeconds() const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!have_state_) return std::numeric_limits<double>::infinity();
  return std::chrono::duration<double>(Clock::now() - state_time_).count();
}

ArmInfo Session::info() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return info_;
}

std::string Session::takeRefusal() {
  std::lock_guard<std::mutex> lock(mutex_);
  std::string r;
  r.swap(refusal_);
  return r;
}

}  // namespace sixeyes
