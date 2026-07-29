#pragma once

#include <fcntl.h>
#include <stdlib.h>
#include <termios.h>
#include <unistd.h>

#include <array>
#include <atomic>
#include <map>
#include <mutex>
#include <chrono>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

// A Feetech STS bus on a pty, for testing MoteHardware against the real
// SCServo SDK without a robot.
//
// The bus is where the arm and the wheels actually meet, so the parts of this
// component worth proving — that the arm comes up limp, that a goal is seeded
// before torque is enabled, that a soft limit is enforced on the far side of
// every client, that an unchanged goal costs no packet — are all statements
// about the bytes on the wire. This responder speaks the protocol well enough
// to record them.
//
// Protocol (SCS/STS, protocol_end 0, little-endian words):
//   instruction:  FF FF ID LEN INST P... CHK      LEN = nparams + 2
//   status:       FF FF ID LEN ERR P... CHK       CHK = ~(sum of ID..params)
//   PING 0x01, READ 0x02, WRITE 0x03, SYNC_WRITE 0x83 (broadcast, no reply)

namespace mote_hardware
{
namespace test
{

constexpr int STS_MODE = 33;
constexpr int STS_TORQUE_ENABLE = 40;
constexpr int STS_GOAL_POSITION = 42;
constexpr int STS_LOCK = 55;
constexpr int STS_PRESENT_POSITION = 56;
constexpr int STS_PRESENT_SPEED = 58;
constexpr int STS_REGISTER_COUNT = 80;

class StsBusSim
{
public:
  // `ids` are the servos that answer. Anything else is silently absent, which
  // is how a missing servo behaves on a real bus.
  explicit StsBusSim(const std::vector<int> & ids)
  {
    // Non-blocking: the responder polls so it can notice the destructor asking
    // it to stop. A blocking read on a pty with an open slave never returns.
    master_fd_ = ::posix_openpt(O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (master_fd_ < 0 || ::grantpt(master_fd_) != 0 || ::unlockpt(master_fd_) != 0) {
      throw std::runtime_error("could not create a pty for the servo bus");
    }
    slave_path_ = ::ptsname(master_fd_);

    // Raw on both ends: the SDK sets its own termios on the slave, but the
    // master must not translate or echo or the framing breaks.
    termios opt{};
    ::tcgetattr(master_fd_, &opt);
    ::cfmakeraw(&opt);
    ::tcsetattr(master_fd_, TCSANOW, &opt);

    for (const int id : ids) {
      auto & regs = registers_[id];
      regs.fill(0);
      regs[STS_MODE] = 0;  // servo (position) mode
      set_word(regs, STS_PRESENT_POSITION, 2048);
      set_word(regs, STS_GOAL_POSITION, 2048);
    }

    running_ = true;
    thread_ = std::thread(&StsBusSim::serve, this);
  }

  ~StsBusSim()
  {
    running_ = false;
    if (thread_.joinable()) {
      thread_.join();
    }
    if (master_fd_ >= 0) {
      ::close(master_fd_);
    }
  }

  const std::string & port() const {return slave_path_;}

  // Every transaction the component performed, in order.
  std::vector<std::string> events()
  {
    settle();
    std::lock_guard<std::mutex> lock(mutex_);
    return events_;
  }

  // Wait until the bus has been quiet briefly.
  //
  // A sync write is a broadcast with no status packet, so the SDK returns as
  // soon as the bytes are handed to the kernel — the responder may not have
  // seen them yet. Every assertion about bus traffic therefore has to let the
  // wire drain first, or it races the very packet it is checking for.
  void settle(int quiet_ms = 20, int limit_ms = 1000)
  {
    const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::milliseconds(limit_ms);
    std::size_t last = 0;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      last = events_.size();
    }
    while (std::chrono::steady_clock::now() < deadline) {
      std::this_thread::sleep_for(std::chrono::milliseconds(quiet_ms));
      std::lock_guard<std::mutex> lock(mutex_);
      if (events_.size() == last) {
        return;
      }
      last = events_.size();
    }
  }

  void clear_events()
  {
    settle();
    std::lock_guard<std::mutex> lock(mutex_);
    events_.clear();
  }

  int present_position(int id)
  {
    settle();
    std::lock_guard<std::mutex> lock(mutex_);
    return word(registers_[id], STS_PRESENT_POSITION);
  }

  void set_present_position(int id, int counts)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    set_word(registers_[id], STS_PRESENT_POSITION, counts);
  }

  bool torque_enabled(int id)
  {
    settle();
    std::lock_guard<std::mutex> lock(mutex_);
    return registers_[id][STS_TORQUE_ENABLE] != 0;
  }

  void set_mode(int id, int mode)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    registers_[id][STS_MODE] = static_cast<unsigned char>(mode);
  }

  // Ignore mode writes for this servo, as one whose EEPROM write does not take
  // does: it answers, but can never be confirmed in position mode.
  void set_mode_write_ignored(int id, bool ignored)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (ignored) {
      mode_locked_.insert(id);
    } else {
      mode_locked_.erase(id);
    }
  }

  // Stop answering for this servo, as an unpowered or unplugged one does.
  void set_absent(int id, bool absent)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (absent) {
      absent_.insert(id);
    } else {
      absent_.erase(id);
    }
  }

private:
  using Registers = std::array<unsigned char, STS_REGISTER_COUNT>;

  static int word(const Registers & regs, int addr)
  {
    return regs[addr] | (regs[addr + 1] << 8);
  }

  static void set_word(Registers & regs, int addr, int value)
  {
    regs[addr] = static_cast<unsigned char>(value & 0xFF);
    regs[addr + 1] = static_cast<unsigned char>((value >> 8) & 0xFF);
  }

  void log(const std::string & event)
  {
    events_.push_back(event);
  }

  static unsigned char checksum(const std::vector<unsigned char> & body)
  {
    unsigned sum = 0;
    for (const unsigned char byte : body) {
      sum += byte;
    }
    return static_cast<unsigned char>(~sum & 0xFF);
  }

  void reply(int id, const std::vector<unsigned char> & params)
  {
    std::vector<unsigned char> body;
    body.push_back(static_cast<unsigned char>(id));
    body.push_back(static_cast<unsigned char>(params.size() + 2));
    body.push_back(0);  // error byte
    body.insert(body.end(), params.begin(), params.end());

    std::vector<unsigned char> packet{0xFF, 0xFF};
    packet.insert(packet.end(), body.begin(), body.end());
    packet.push_back(checksum(body));
    ssize_t written = ::write(master_fd_, packet.data(), packet.size());
    (void)written;
  }

  // Apply a register write, keeping the model of the servo consistent: a real
  // one moves to a goal it is given (this one does so instantly), so a test can
  // assert where the arm was told to go.
  //
  // A position command is not a write to GOAL_POSITION alone: both WritePosEx
  // and SyncWritePosEx start at ACC (41) and carry ACC, goal, time and speed in
  // one 7-byte block. So a goal is any write whose range *covers* registers
  // 42-43, wherever it starts.
  void apply_write(int id, int addr, const std::vector<unsigned char> & data)
  {
    if (addr == STS_MODE && mode_locked_.count(id)) {
      log("mode id=" + std::to_string(id) + " REFUSED");
      return;
    }
    auto & regs = registers_[id];
    for (std::size_t i = 0; i < data.size(); ++i) {
      const std::size_t target = static_cast<std::size_t>(addr) + i;
      if (target < regs.size()) {
        regs[target] = data[i];
      }
    }

    const int end = addr + static_cast<int>(data.size());
    std::ostringstream out;
    if (addr == STS_TORQUE_ENABLE) {
      out << "torque id=" << id << (data[0] ? " on" : " off");
    } else if (addr <= STS_GOAL_POSITION && end > STS_GOAL_POSITION + 1) {
      const int goal = word(regs, STS_GOAL_POSITION);
      set_word(regs, STS_PRESENT_POSITION, goal);
      out << "goal id=" << id << " counts=" << goal;
    } else if (addr == STS_MODE) {
      out << "mode id=" << id << " value=" << static_cast<int>(data[0]);
    } else if (addr == STS_LOCK) {
      out << "lock id=" << id << " value=" << static_cast<int>(data[0]);
    } else {
      out << "write id=" << id << " addr=" << addr << " len=" << data.size();
    }
    log(out.str());
  }

  void handle(const std::vector<unsigned char> & packet)
  {
    const int id = packet[2];
    const int inst = packet[4];
    const std::vector<unsigned char> params(packet.begin() + 5, packet.end() - 1);

    std::lock_guard<std::mutex> lock(mutex_);

    if (inst == 0x83) {  // SYNC_WRITE — broadcast, every servo in one packet
      const int addr = params[0];
      const int len = params[1];
      std::size_t offset = 2;
      int served = 0;
      while (offset + 1 + static_cast<std::size_t>(len) <= params.size()) {
        const int target = params[offset];
        const std::vector<unsigned char> data(
          params.begin() + static_cast<long>(offset) + 1,
          params.begin() + static_cast<long>(offset) + 1 + len);
        if (registers_.count(target) && !absent_.count(target)) {
          apply_write(target, addr, data);
          ++served;
        }
        offset += 1 + static_cast<std::size_t>(len);
      }
      // Logged after the goals so a test can count packets and goals apart:
      // "one packet carried two joints" is the claim worth checking.
      log("syncwrite servos=" + std::to_string(served));
      return;
    }

    if (!registers_.count(id) || absent_.count(id)) {
      return;  // nobody home: the SDK times out, as on a real bus
    }

    if (inst == 0x01) {  // PING
      log("ping id=" + std::to_string(id));
      reply(id, {});
    } else if (inst == 0x02) {  // READ
      const int addr = params[0];
      const int len = params[1];
      std::vector<unsigned char> data;
      for (int i = 0; i < len; ++i) {
        const std::size_t target = static_cast<std::size_t>(addr + i);
        data.push_back(target < registers_[id].size() ? registers_[id][target] : 0);
      }
      std::ostringstream out;
      out << "read id=" << id << " addr=" << addr << " len=" << len;
      log(out.str());
      reply(id, data);
    } else if (inst == 0x03) {  // WRITE
      const int addr = params[0];
      const std::vector<unsigned char> data(params.begin() + 1, params.end());
      apply_write(id, addr, data);
      reply(id, {});
    }
  }

  void serve()
  {
    std::vector<unsigned char> buffer;
    while (running_) {
      unsigned char chunk[256];
      const ssize_t got = ::read(master_fd_, chunk, sizeof(chunk));
      if (got > 0) {
        buffer.insert(buffer.end(), chunk, chunk + got);
      } else {
        std::this_thread::sleep_for(std::chrono::microseconds(200));
        continue;
      }

      // Resynchronise on FF FF rather than trusting the stream: it makes the
      // responder immune to any preamble the SDK or the pty adds.
      while (buffer.size() >= 6) {
        if (buffer[0] != 0xFF || buffer[1] != 0xFF) {
          buffer.erase(buffer.begin());
          continue;
        }
        const std::size_t length = buffer[3];
        const std::size_t total = 4 + length;  // FF FF ID LEN + (LEN bytes)
        if (buffer.size() < total) {
          break;  // wait for the rest
        }
        handle(std::vector<unsigned char>(buffer.begin(), buffer.begin() + total));
        buffer.erase(buffer.begin(), buffer.begin() + total);
      }
    }
  }

  int master_fd_ = -1;
  std::string slave_path_;
  std::thread thread_;
  std::atomic<bool> running_{false};

  std::mutex mutex_;
  std::map<int, Registers> registers_;
  std::set<int> absent_;
  std::set<int> mode_locked_;
  std::vector<std::string> events_;
};

}  // namespace test
}  // namespace mote_hardware
