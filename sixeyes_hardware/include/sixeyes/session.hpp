// A conversation with an arm: handshake, commands that wait for their answer, the state stream and the
// heartbeat that keeps the arm's 200 ms watchdog fed. The same rules as python/src/sixeyes_host/session.py.
//
// Every function that can fail returns a sentence in plain words, or an empty string for success.
// A background thread reads the stream and sends heartbeats; all public functions are safe to call from any thread.
#pragma once

#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "sixeyes/codec.hpp"
#include "sixeyes/stream.hpp"
#include "sixeyes_protocol.h"

namespace sixeyes {

struct ArmState {
  uint32_t t_ms = 0;
  uint8_t homing_state = SIXEYES_HOMING_STATE_UNHOMED;
  uint8_t control_source = SIXEYES_CONTROL_SOURCE_BLE;
  uint16_t faults = 0;
  float measured[SIXEYES_JOINT_COUNT] = {};  // five angles in radians, then the gripper 0..100
  float target[SIXEYES_JOINT_COUNT] = {};    // the last target the arm accepted
};

struct ArmInfo {
  uint16_t protocol_version = 0;
  uint8_t fw_major = 0, fw_minor = 0, fw_patch = 0;
  uint16_t capabilities = 0;  // SIXEYES_CAP_* bits
};

// What a refusal from the arm means, in plain words. Empty for SIXEYES_STATUS_OK.
std::string statusText(uint8_t status);

class Session {
 public:
  // Takes ownership of the stream and starts the reader thread.
  explicit Session(std::unique_ptr<ByteStream> stream, int ack_timeout_ms = 1500, int heartbeat_ms = 50);
  ~Session();
  Session(const Session &) = delete;
  Session &operator=(const Session &) = delete;

  // Says hello, checks the protocol versions match, and waits for the first state message.
  std::string connect();

  std::string setControlUsb();  // this computer takes control (refused while the arm is moving)
  std::string home();           // starts homing; use waitReady() to wait for it to finish
  // Waits until the arm reports Ready in a state that arrived after the last home command was accepted.
  std::string waitReady(int timeout_ms);
  std::string stop();
  std::string clearFault();
  // Joint targets: five angles in radians, then the gripper 0..100. Waits for the arm's answer.
  std::string sendTargets(const float joints[SIXEYES_JOINT_COUNT]);
  // For streaming at a steady rate: does not wait. A refusal shows up later in takeRefusal().
  void sendTargetsNoWait(const float joints[SIXEYES_JOINT_COUNT]);

  // The newest state. Returns false if none has arrived yet.
  bool state(ArmState &out) const;
  // Seconds since the newest state arrived; infinity if none has.
  double stateAgeSeconds() const;
  ArmInfo info() const;
  // The latest refusal of a streamed command (empty if none). Reading it clears it.
  std::string takeRefusal();

 private:
  using Clock = std::chrono::steady_clock;

  void ioLoop();
  void handle(const Frame &f);
  void maybeHeartbeat();
  bool sendFrame(uint8_t type, uint8_t seq, const uint8_t *payload, size_t n);
  uint8_t nextSeq();
  std::string command(uint8_t type, const uint8_t *payload, size_t n);

  std::unique_ptr<ByteStream> stream_;
  const int ack_timeout_ms_;
  const int heartbeat_ms_;

  FrameParser parser_;  // only the reader thread touches it
  std::thread io_;
  bool running_ = true;  // guarded by mutex_

  mutable std::mutex mutex_;
  std::condition_variable cv_;
  uint8_t seq_ = 0;
  std::map<int, int> pending_;  // (type << 8 | seq) -> status, or -1 while waiting
  ArmState state_;
  bool have_state_ = false;
  unsigned state_count_ = 0;     // how many states have arrived
  unsigned home_ack_count_ = 0;  // state_count_ when the arm last accepted a home command
  Clock::time_point state_time_;
  ArmInfo info_;
  std::string refusal_;
  bool io_error_ = false;

  std::mutex write_mutex_;
};

}  // namespace sixeyes
