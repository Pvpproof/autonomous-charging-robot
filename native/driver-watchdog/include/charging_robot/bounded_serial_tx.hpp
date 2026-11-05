#pragma once
#include "command_watchdog.hpp"
#include <cerrno>
#include <fcntl.h>
#include <poll.h>
#include <unistd.h>

namespace charging {
// Dedicated nonblocking TX descriptor for the same terminal already configured
// by serial::Serial. No shared library read/write lock, tcdrain or flush here.
// A successful write means accepted by the OS, not acknowledged by the MCU.
class BoundedSerialTx {
 public:
  bool open(const std::string &path) {
    fd_=::open(path.c_str(),O_WRONLY|O_NOCTTY|O_NONBLOCK|O_CLOEXEC);
    return fd_>=0;
  }
  ~BoundedSerialTx() { if(fd_>=0) ::close(fd_); }
  bool write(const CommandWatchdog::Frame &frame) {
    if(fd_<0) return false;
    const auto deadline=CommandWatchdog::Clock::now()+std::chrono::milliseconds(20);
    size_t sent=0;
    while(sent<frame.size()) {
      if(CommandWatchdog::Clock::now()>=deadline) return false;
      const auto n=::write(fd_,frame.data()+sent,frame.size()-sent);
      if(n>0) { sent+=static_cast<size_t>(n); continue; }
      if(n<0&&errno==EINTR) continue;
      if(n<0&&(errno==EAGAIN||errno==EWOULDBLOCK)) {
        pollfd descriptor{fd_,POLLOUT,0};
        // bounded polling; no waiting on the receive-side serial mutex.
        const int result=::poll(&descriptor,1,1);
        if(result<0&&errno!=EINTR) return false;
        if(result>0&&(descriptor.revents&(POLLERR|POLLHUP|POLLNVAL))) return false;
        continue;
      }
      return false;
    }
    return true;
  }
 private:
  int fd_=-1;
};
}
