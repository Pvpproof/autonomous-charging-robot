#include "charging_robot/command_watchdog.hpp"
#include "charging_robot/bounded_serial_tx.hpp"
#include <algorithm>
#include <atomic>
#include <cassert>
#include <iostream>
#include <pty.h>
#include <termios.h>
#include <thread>
#include <vector>
using W=charging::CommandWatchdog;
using namespace std::chrono;
static int tests=0;
static void require(bool value,const char *name) {
  if(!value) { std::cerr<<"FAIL "<<name<<"\n"; std::exit(1); }
  ++tests;
}
static bool contains(W &w,const char *v,W::Time t=W::Clock::now()) {
  return w.state(t).find(v)!=std::string::npos;
}
static void ready(W &w,W::Time t) {
  for(int i=0;i<=110;++i) {
    const auto now=t+milliseconds(i*20);
    w.command(0,0,0,now); w.feedback(0,0,0,now);
    w.transmit([](const W::Frame&){return true;},now);
  }
  bool ok=false;w.reset(ok,t+milliseconds(2200));
  require(ok,"explicit reset after fresh zero+continuous quiet+TX");
}
int main() {
  const auto t=W::Clock::now();
  W::Frame observed{};
  auto capture=[&](const W::Frame&f){observed=f;return true;};
  W startup("boot-a");
  startup.command(.15,0,0,t);startup.transmit(capture,t);
  require(observed==W::frame(),"startup blocks ordinary motion");
  bool ok=true;startup.reset(ok,t);
  require(!ok,"reset rejects missing quiet feedback");
  W w("boot-b");ready(w,t);
  const auto run=t+milliseconds(2200);
  w.command(-.15,0,-.05,run);w.feedback(0,0,0,run);w.transmit(capture,run);
  require(observed==W::frame(-.15,0,-.05),"active negative command bytes");
  unsigned checksum=0;for(unsigned i=0;i<9;++i)checksum^=observed[i];
  require(observed[9]==checksum&&observed[0]==0x7b&&observed[10]==0x7d,"complete frame checksum");
  w.feedback(0,0,0,run+milliseconds(240));
  w.transmit(capture,run+milliseconds(251));
  require(observed==W::frame()&&contains(w,"command_timeout",run+milliseconds(251)),"251ms command loss locks zero");
  w.command(.15,0,0,run+milliseconds(252));w.transmit(capture,run+milliseconds(252));
  require(observed==W::frame(),"recovered command cannot resume");
  W late("boot-late");ready(late,t);
  late.command(.1,0,0,run);late.feedback(0,0,0,run);
  late.command(.1,0,0,run+milliseconds(300));late.feedback(0,0,0,run+milliseconds(300));
  late.transmit(capture,run+milliseconds(300));
  require(observed==W::frame(),"late callback cannot erase unsampled timeout");
  W feedback("boot-feedback");ready(feedback,t);
  for(int i=0;i<16;++i) { feedback.command(.1,0,0,run+milliseconds(i*20));feedback.transmit(capture,run+milliseconds(i*20)); }
  require(observed==W::frame()&&contains(feedback,"feedback_timeout",run+milliseconds(300)),"feedback loss while command alive locks");
  W txfail("boot-tx");ready(txfail,t);
  txfail.transmit([](const W::Frame&){return false;},run);txfail.command(.1,0,0,run);txfail.transmit(capture,run);
  require(observed==W::frame()&&contains(txfail,"serial_tx_failed",run),"TX failure persistent lock");
  W invalid("boot-invalid");ready(invalid,t);invalid.command(NAN,0,0,run);invalid.transmit(capture,run);
  require(observed==W::frame()&&contains(invalid,"invalid_command",run),"nonfinite command rejected");
  W gap("boot-gap");gap.feedback(0,0,0,t);gap.feedback(0,0,0,t+seconds(3));gap.command(0,0,0,t+seconds(3));gap.transmit(capture,t+seconds(3));gap.reset(ok,t+seconds(3));
  require(!ok,"quiet interval cannot span feedback loss");
  W restart("boot-c");require(contains(restart,"boot-c")&&contains(restart,"startup_reset_required"),"new process session starts locked");

  int master=-1,slave=-1;char name[128]{};
  require(openpty(&master,&slave,name,nullptr,nullptr)==0,"allocate private PTY");
  termios attrs{};tcgetattr(slave,&attrs);cfmakeraw(&attrs);tcsetattr(slave,TCSANOW,&attrs);
  fcntl(master,F_SETFL,O_NONBLOCK);
  charging::BoundedSerialTx tx;require(tx.open(name),"open separate nonblocking TX descriptor");
  W integration("boot-pty");
  // Fake monotonic history sets only setup state; subsequent execution uses real steady clock.
  const auto history=W::Clock::now()-milliseconds(2200);ready(integration,history);
  std::atomic<bool> stop{false},rx_done{false};
  std::thread blocked_rx([&]{pollfd p{slave,POLLIN,0};poll(&p,1,2000);rx_done=true;});
  std::thread writer([&]{while(!stop) {integration.transmit([&](const W::Frame&f){return tx.write(f);});std::this_thread::sleep_for(milliseconds(20));}});
  const auto start=W::Clock::now();
  while(W::Clock::now()-start<milliseconds(160)) {
    integration.command(-.15,0,0);integration.feedback(0,0,0);std::this_thread::sleep_for(milliseconds(20));
  }
  const auto last=W::Clock::now()-milliseconds(20);
  std::vector<uint8_t> bytes;uint8_t buffer[512];W::Time first_zero{};bool saw_motion=false;
  const auto deadline=W::Clock::now()+milliseconds(500);
  while(W::Clock::now()<deadline) {
    auto n=read(master,buffer,sizeof(buffer));if(n>0)bytes.insert(bytes.end(),buffer,buffer+n);
    while(bytes.size()>=11) {
      W::Frame f{};std::copy(bytes.begin(),bytes.begin()+11,f.begin());bytes.erase(bytes.begin(),bytes.begin()+11);
      require(f[0]==0x7b&&f[10]==0x7d,"PTY no interleaved frames");
      if(f==W::frame(-.15,0,0))saw_motion=true;
      else if(saw_motion&&f==W::frame()&&first_zero==W::Time{})first_zero=W::Clock::now();
    }
    std::this_thread::sleep_for(milliseconds(2));
  }
  require(saw_motion,"PTY observes nonzero only after reset");
  require(first_zero!=W::Time{}&&duration_cast<milliseconds>(first_zero-last).count()<310,"TX zero within 310ms despite blocked 2s RX");
  require(!rx_done,"receive read still blocked when watchdog stopped");
  integration.command(-.15,0,0);integration.feedback(0,0,0);
  require(contains(integration,"\"latched\":true"),"PTY message recovery keeps latch");
  stop=true;writer.join();blocked_rx.join();
  std::cout<<"PTY timeout observed_ms="<<duration_cast<milliseconds>(first_zero-last).count()<<" RX_block_ms=2000\n";
  // Fill PTY output queue without consuming master. Bounded writer must return.
  bool full=false;auto before=W::Clock::now();
  for(int i=0;i<10000;++i)if(!tx.write(W::frame())){full=true;break;}
  require(full,"PTY backpressure produces bounded TX failure");
  require(W::Clock::now()-before<seconds(2),"bounded TX saturation cannot wait for serial RX");
  close(slave);close(master);
  std::cout<<"PASS assertions="<<tests<<"\n";
}
