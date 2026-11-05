#pragma once
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <mutex>
#include <sstream>
#include <string>

// No ROS time or executor dependency. All state is protected by one mutex;
// the ONLY caller of transmit() is the dedicated TX thread. The writer must
// have a bounded deadline; holding the mutex through that write prevents an
// older nonzero frame from being sent after a reset/fault acknowledgement.
namespace charging {
class CommandWatchdog {
 public:
  using Clock = std::chrono::steady_clock;
  using Time = Clock::time_point;
  using Frame = std::array<uint8_t, 11>;
  using Writer = std::function<bool(const Frame&)>;
  explicit CommandWatchdog(std::string session) : session_(std::move(session)) {}
  static Frame frame(double x=0, double y=0, double z=0) {
    Frame f{{0x7b,0,0,0,0,0,0,0,0,0,0x7d}};
    const double values[3]={x,y,z};
    for (unsigned i=0;i<3;++i) {
      const int16_t value=static_cast<int16_t>(values[i]*1000.0);
      f[3+2*i]=static_cast<uint16_t>(value)>>8;
      f[4+2*i]=static_cast<uint16_t>(value)&255;
    }
    for (unsigned i=0;i<9;++i) f[9]^=f[i];
    return f;
  }
  void command(double x,double y,double z,Time now=Clock::now()) {
    std::lock_guard<std::mutex> guard(mutex_);
    if (!std::isfinite(x)||!std::isfinite(y)||!std::isfinite(z)||
        std::abs(x)>32.767||std::abs(y)>32.767||std::abs(z)>32.767) {
      latch("invalid_command"); command_zero_=false; return;
    }
    // A late callback cannot erase a timeout which the TX thread has not
    // yet sampled. Recovery always requires the explicit reset service.
    expire(now);
    command_=frame(x,y,z); command_at_=now; have_command_=true;
    command_zero_=(x==0.0&&y==0.0&&z==0.0);
  }
  void feedback(double x,double y,double z,Time now=Clock::now()) {
    std::lock_guard<std::mutex> guard(mutex_);
    expire(now);
    const bool gap=!have_feedback_||age(now,feedback_at_)>0.25;
    const bool quiet=std::isfinite(x)&&std::isfinite(y)&&std::isfinite(z)&&
      std::abs(x)<=0.01&&std::abs(y)<=0.01&&std::abs(z)<=0.02;
    if (!quiet||gap) quiet_since_=now;
    have_feedback_=true; feedback_at_=now; feedback_quiet_=quiet;
  }
  std::string reset(bool &success,Time now=Clock::now()) {
    std::lock_guard<std::mutex> guard(mutex_);
    expire(now);
    success=have_command_&&command_zero_&&age(now,command_at_)<=0.25&&
      have_feedback_&&age(now,feedback_at_)<=0.25&&feedback_quiet_&&age(now,quiet_since_)>=2.0&&
      have_tx_&&age(now,tx_at_)<=0.1;
    if (success) { latched_=false; reason_="ready"; }
    return json(now,success?"reset_ack":"reset_rejected");
  }
  bool transmit(const Writer &writer,Time now=Clock::now()) {
    std::lock_guard<std::mutex> guard(mutex_);
    expire(now);
    const auto selected=latched_?frame():command_;
    bool ok=false;
    try { ok=writer(selected); } catch (...) { ok=false; }
    if (!ok) latch("serial_tx_failed");
    else { have_tx_=true; tx_at_=now; }
    return ok;
  }
  void fault(const std::string &reason) { std::lock_guard<std::mutex> guard(mutex_); latch(reason); }
  std::string state(Time now=Clock::now()) {
    std::lock_guard<std::mutex> guard(mutex_); expire(now); return json(now,"state");
  }
 private:
  static double age(Time now,Time then) { return std::chrono::duration<double>(now-then).count(); }
  void latch(const std::string &reason) { if(!latched_) { latched_=true; reason_=reason; } }
  void expire(Time now) {
    if (!latched_&&(!have_command_||age(now,command_at_)>0.25)) latch("command_timeout");
    if (!latched_&&(!have_feedback_||age(now,feedback_at_)>0.25)) latch("feedback_timeout");
  }
  std::string json(Time now,const char *event) {
    std::ostringstream out;
    out<<"{\"version\":2,\"session\":\""<<session_<<"\",\"state_seq\":"<<++seq_
      <<",\"monotonic_ns\":"<<std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count()
      <<",\"latched\":"<<(latched_?"true":"false")<<",\"reason\":\""<<reason_
      <<"\",\"event\":\""<<event<<"\",\"command_age_s\":"<<(have_command_?age(now,command_at_):-1)
      <<",\"feedback_age_s\":"<<(have_feedback_?age(now,feedback_at_):-1)<<"}";
    return out.str();
  }
  std::mutex mutex_;
  std::string session_;
  std::string reason_="startup_reset_required";
  uint64_t seq_=0;
  bool latched_=true,have_command_=false,have_feedback_=false,command_zero_=true,feedback_quiet_=false,have_tx_=false;
  Time command_at_{},feedback_at_{},quiet_since_{},tx_at_{};
  Frame command_=frame();
};
}
