#include "components/jsdrive2/jsdrive2.h"
#include <cassert>
#include <cmath>
#include <iostream>

using esphome::jsdrive2::JSDrive2;
using esphome::uart::UARTComponent;
using esphome::GPIOPin;
using esphome::text_sensor::TextSensor;
using esphome::binary_sensor::BinarySensor;

struct ProbeBridge : JSDrive2 {
  using JSDrive2::recover_;
  void simulate_failed_alignment() {
    armed_=false; sync_required_=true; align_attempts_=3; align_high_box_=0; align_wake_attempted_=true;
  }
  bool override_active() const {
    return override_align_ && !armed_;
  }
  bool recovered_alignment_cleared() const {
    return armed_ && !override_align_ && align_attempts_==0;
  }
  bool armed_for_test() const { return armed_; }
};

static constexpr uint8_t seg[] = {0x3F,0x06,0x5B,0x4F,0x66,0x6D,0x7D,0x07,0x7F,0x6F};
static void height(UARTComponent &u, int n) {
  const int whole=n/10;
  const uint8_t a=seg[(whole/10)%10], b=uint8_t(seg[whole%10]|0x80), c=seg[n%10];
  u.feed({0x5A,a,b,c,uint8_t(a+b+c)});
}
static void blank(UARTComponent &u) { u.feed({0x5A,0,0,0,0}); }
static void command(UARTComponent &u, uint8_t c) { u.feed({0xA5,0,c,uint8_t(0xFF-c),0xFF}); }
static int shown(const UARTComponent &u) {
  if (u.tx.empty()) return -1;
  const auto &f=u.tx.back();
  if (f[1]==0 && f[2]==0 && f[3]==0) return -1;
  auto digit=[](uint8_t v) {
    for (int i=0;i<10;i++) if (seg[i]==(v&0x7F)) return i;
    return -1;
  };
  return 100*digit(f[1])+10*digit(f[2])+digit(f[3]);
}
static bool shows_snc(const UARTComponent &u) {
  if (u.tx.empty()) return false;
  const auto &f=u.tx.back();
  return f[1]==0x6D && f[2]==0x54 && f[3]==0x39;
}
static bool sent_command(const UARTComponent &u, uint8_t command) {
  for (const auto &frame : u.tx) if (frame[2]==command) return true;
  return false;
}
static void tick(JSDrive2 &b, uint32_t n=1) { esphome::test_ms+=n; b.loop(); }

struct Fixture {
  ProbeBridge bridge;
  UARTComponent remote,a,b;
  GPIOPin remote_action, remote_awake, a_action,b_action,a_awake,b_awake;
  TextSensor activity;
  BinarySensor up;
  Fixture() {
    esphome::test_ms=1000;
    a_awake.level=b_awake.level=true;
    bridge.set_remote_uart(&remote); bridge.set_desk_uart_a(&a); bridge.set_desk_uart_b(&b);
    bridge.set_remote_action_pin(&remote_action); bridge.set_remote_awake_pin(&remote_awake);
    bridge.set_box_a_action_pin(&a_action); bridge.set_box_b_action_pin(&b_action);
    bridge.set_box_a_awake_pin(&a_awake); bridge.set_box_b_awake_pin(&b_awake);
    bridge.set_activity_sensor(&activity); bridge.set_up_sensor(&up);
    bridge.setup();
  }
  void pair(int n) { height(a,n); height(b,n); tick(bridge); }
};

int main() {
  {
    Fixture f;
    f.pair(285);
    f.remote_action.level=true; command(f.remote,0x20); tick(f.bridge,10);
    assert(f.activity.state=="UP" && f.up.state);
    assert(f.a.flush_count==1 && f.b.flush_count==1);
    f.remote_action.level=false; command(f.remote,0); tick(f.bridge,10);
    assert(f.activity.state=="IDLE" && !f.up.state);
  }
  {
    Fixture f;
    f.pair(285); blank(f.a); blank(f.b); tick(f.bridge,10);
    assert(f.bridge.request_height(30.0f));
    tick(f.bridge,10);
    assert(shown(f.remote)==285); // HA activity drives handset status without prior handset frames
  }
  {
    Fixture f;
    f.pair(285);
    assert(f.bridge.request_height(30.0f)); tick(f.bridge,10);
    assert(f.activity.state=="UP" && f.up.state); // same activity as handset UP
    assert(f.a.flush_count==1 && f.b.flush_count==1); // same clean pin-5 edge
  }
  {
    Fixture f;
    f.pair(285);
    f.b.flush_result=esphome::uart::UARTFlushResult::UART_FLUSH_RESULT_TIMEOUT;
    assert(f.bridge.request_height(30.0f)); tick(f.bridge,10);
    assert(!f.a_action.level && !f.b_action.level);
    assert(!f.bridge.armed_for_test()); // never raise pin 5 with queued idle bytes
  }
  {
    Fixture f;
    f.pair(285);
    assert(f.bridge.request_height(30.0f)); tick(f.bridge,10);
    assert(f.a_action.level && f.b_action.level);
    blank(f.a); blank(f.b); tick(f.bridge,10);
    assert(f.bridge.armed_for_test());
    assert(f.a_action.level && f.b_action.level); // one blank frame is tolerated
    for (int i=0;i<16;i++) {
      blank(f.a); blank(f.b); tick(f.bridge,10);
    }
    assert(!f.bridge.armed_for_test());
    assert(!f.a_action.level && !f.b_action.level); // sustained missing heights stop motion
  }
  {
    Fixture f;
    f.pair(285); command(f.remote,0); tick(f.bridge,10); tick(f.bridge,10);
    assert(shown(f.remote)==285);
    f.remote_action.level=true; command(f.remote,0x20); tick(f.bridge,10);
    assert(f.a_action.level && f.b_action.level);
    tick(f.bridge,119);
    assert(f.a_action.level && f.b_action.level); // three observed 35 ms gaps are tolerated
    tick(f.bridge,2);
    assert(!f.a_action.level && !f.b_action.level); // stale handset stream drops the action lines
  }
  {
    Fixture f;
    f.pair(285); command(f.remote,0); tick(f.bridge,10);
    f.remote_action.level=true; command(f.remote,0x20); tick(f.bridge,10);
    height(f.a,290); tick(f.bridge,10);
    assert(!f.a_action.level && !f.b_action.level); // 0.5 split stops both sides
    assert(shows_snc(f.remote));
    f.remote_action.level=false; command(f.remote,0); tick(f.bridge,10);
    f.remote_action.level=true; command(f.remote,0x40); tick(f.bridge,10);
    assert(f.a_action.level && !f.b_action.level); // held DOWN lowers only higher A
  }
  {
    Fixture f;
    f.pair(440); command(f.remote,0); tick(f.bridge,10);
    f.remote_action.level=true; command(f.remote,0x04); tick(f.bridge,10);
    f.pair(285); tick(f.bridge,10);
    assert(shown(f.remote)==285); // destination announcement is shown before travel
    f.pair(435); tick(f.bridge,10);
    assert(shown(f.remote)==435); // physical travel replaces the announcement
    f.remote_action.level=false; command(f.remote,0); tick(f.bridge,10);
  }
  {
    Fixture f;
    f.pair(285); command(f.remote,0); tick(f.bridge,10);
    f.remote_action.level=true; command(f.remote,0x20); tick(f.bridge,10);
    assert(f.a_action.level && f.b_action.level);
    f.remote_action.level=false; command(f.remote,0); tick(f.bridge,10);
    assert(!f.a_action.level && !f.b_action.level);
    f.remote_action.level=true; command(f.remote,0x20); tick(f.bridge,10);
    assert(f.a_action.level && f.b_action.level); // quick second press remains usable
    command(f.remote,0x60); tick(f.bridge,10);
    assert(!f.a_action.level && !f.b_action.level); // combined buttons never reach boxes
  }
  {
    Fixture f;
    f.pair(440); command(f.remote,0); tick(f.bridge,10);
    f.remote_action.level=true; command(f.remote,0x04); tick(f.bridge,10);
    height(f.a,285); height(f.b,285); command(f.remote,0x04); tick(f.bridge,10);
    height(f.a,439); command(f.remote,0x04); tick(f.bridge,10);
    height(f.a,438); height(f.b,438); command(f.remote,0x04); tick(f.bridge,10);
    assert(shown(f.remote)==438); // skipped intermediate tenth pairs at next common report
    height(f.a,430); command(f.remote,0x04); tick(f.bridge,10);
    assert(f.bridge.armed_for_test()); // transient preset report skew is tolerated
    assert(!shows_snc(f.remote));
    height(f.b,430); command(f.remote,0x04); tick(f.bridge,10);
    assert(shown(f.remote)==430);
  }
  {
    Fixture f;
    f.pair(285); command(f.remote,0); tick(f.bridge,10);
    blank(f.a); blank(f.b); tick(f.bridge,10); tick(f.bridge,10);
    assert(shown(f.remote)==-1); // display timeout follows the boxes
  }
  {
    Fixture f;
    f.a_awake.level=false; tick(f.bridge,1); tick(f.bridge,60);
    assert(!f.a_action.level && !f.b_action.level); // pulse completes synchronously
    assert(sent_command(f.a,0x60));
    assert(f.a.tx.back()[2]==0x00); // chord is followed by idle with pin 5 low
  }
  {
    Fixture f;
    f.a_awake.level=false; f.b_awake.level=false;
    f.remote_action.level=true; command(f.remote,0x20); tick(f.bridge,10);
    tick(f.bridge,10);
    assert(shown(f.remote)!=505); // first wake press is not a fault
    assert(!f.a_action.level && !f.b_action.level);
    assert(sent_command(f.a,0x60));
    assert(sent_command(f.b,0x60));
  }
  {
    Fixture f;
    height(f.a,299); height(f.b,297); tick(f.bridge);
    blank(f.a); blank(f.b); tick(f.bridge,400);
    assert(!f.a_action.level && !f.b_action.level); // bounded wakes have completed
    for (const auto *u : {&f.a,&f.b})
      for (const auto &frame : u->tx) assert(frame[2]!=0x40); // no blind DOWN
  }
  {
    Fixture f;
    height(f.a,299); height(f.b,297); tick(f.bridge);
    height(f.a,299); height(f.b,297); tick(f.bridge,400);
    assert(f.a_action.level && !f.b_action.level); // higher box only
    for (int i=0;i<18;i++) tick(f.bridge,10);
    assert(f.a_action.level); // long correction remains bounded at 190 ms
    tick(f.bridge,10);
    assert(!f.a_action.level && !f.b_action.level);
  }
  {
    Fixture f;
    f.pair(285); f.bridge.simulate_failed_alignment();
    height(f.a,285); height(f.b,285); tick(f.bridge,900);
    f.bridge.arm();
    assert(f.bridge.recovered_alignment_cleared()); // equal healthy boxes arm directly
  }
  {
    Fixture f;
    f.pair(285); tick(f.bridge,400); f.pair(285);
    assert(f.bridge.recall_preset(3));
    tick(f.bridge,10);
    assert(f.a_action.level && f.b_action.level);
    assert(f.a.tx.back()[2]==0x08 && f.b.tx.back()[2]==0x08);
    assert(f.activity.state=="PRESET 3");
    height(f.a,440); height(f.b,440); tick(f.bridge,10);
    assert(shown(f.remote)==440); // matching destination reaches handset
  }
  {
    Fixture f;
    f.pair(285); command(f.remote,0); tick(f.bridge,10);
    f.remote_action.level=true; command(f.remote,0x08); tick(f.bridge,10);
    assert(f.a_action.level && f.b_action.level);
    assert(f.a.tx.back()[2]==0x08 && f.b.tx.back()[2]==0x08);
  }
  {
    Fixture f;
    f.pair(285); command(f.remote,0); tick(f.bridge,10);
    f.remote_action.level=true; command(f.remote,0x08); tick(f.bridge,10);
    height(f.a,440); height(f.b,440); tick(f.bridge,10);
    f.remote_action.level=false; command(f.remote,0); tick(f.bridge,10);
    height(f.a,430); tick(f.bridge,10); // A starts travel without B
    tick(f.bridge,480);
    blank(f.a); blank(f.b); tick(f.bridge,30);
    assert(!f.bridge.armed_for_test());
    assert(f.a_action.level && f.b_action.level);
    assert(f.a.tx.back()[2]==0x40 && f.b.tx.back()[2]==0x40); // opposite arrow cancels
    tick(f.bridge,190);
    assert(!f.a_action.level && !f.b_action.level);
  }
  {
    Fixture f;
    f.pair(285); tick(f.bridge,400); f.pair(285);
    f.a_awake.level=false; tick(f.bridge,10);
    assert(f.bridge.recall_preset(3));
    tick(f.bridge,10);
    for (const auto *u : {&f.a,&f.b})
      for (const auto &frame : u->tx) assert(frame[2]!=0x08);
    assert(!f.b_action.level); // no preset sent to either box while A is asleep
  }
  {
    Fixture f;
    height(f.a,338); height(f.b,286); tick(f.bridge);
    f.bridge.simulate_failed_alignment();
    height(f.a,338); height(f.b,286); tick(f.bridge,900);
    // Held DOWN starts synchronization and drives only the higher box.
    f.remote_action.level=true;
    for (int i=0;i<70;i++) {
      height(f.a,338); height(f.b,286);
      if (i%10==0) command(f.remote,0x40);
      tick(f.bridge,10);
    }
    assert(f.bridge.override_active());
    assert(f.a_action.level && !f.b_action.level); // only the higher box gets DOWN
    assert(f.a.tx.back()[2]==0x40);
    height(f.a,338); height(f.b,286); tick(f.bridge,35);
    assert(f.a_action.level && !f.b_action.level); // observed 35 ms loop gap is tolerated
    for (int i=0;i<50;i++) {
      height(f.a,338); height(f.b,286); tick(f.bridge,10);
      assert(f.a_action.level && !f.b_action.level); // held continuously past 190 ms
      assert(f.a.tx.back()[2]==0x40);
    }
    height(f.a,337); height(f.b,286); tick(f.bridge,10);
    height(f.a,286); height(f.b,286); tick(f.bridge,10);
    assert(!f.a_action.level && !f.b_action.level); // release DOWN at the lower height
    for (int i=0;i<40;i++) { height(f.a,286); height(f.b,286); tick(f.bridge,10); }
    assert(f.bridge.recovered_alignment_cleared());
    command(f.remote,0x40); height(f.a,286); height(f.b,286); tick(f.bridge,10);
    assert(!f.a_action.level && !f.b_action.level); // held DOWN is consumed after recovery
    f.remote_action.level=false; command(f.remote,0); tick(f.bridge,10);
  }
  {
    Fixture f;
    height(f.a,299); height(f.b,290); tick(f.bridge);
    f.bridge.simulate_failed_alignment();
    f.remote_action.level=true;
    for (int i=0;i<70;i++) {
      height(f.a,299); height(f.b,290);
      if (i%10==0) command(f.remote,0x40);
      tick(f.bridge,10);
    }
    assert(f.a_action.level && !f.b_action.level);
    for (int i=0;i<151;i++) { height(f.a,299); height(f.b,290); tick(f.bridge,10); }
    assert(!f.a_action.level && !f.b_action.level); // no height progress stops continuous drive
    assert(!f.bridge.armed_for_test());
  }
  {
    Fixture f;
    height(f.a,290); height(f.b,298); tick(f.bridge);
    f.bridge.simulate_failed_alignment();
    f.remote_action.level=true;
    for (int i=0;i<60;i++) {
      height(f.a,290); height(f.b,298);
      if (i%10==0) command(f.remote,0x40);
      tick(f.bridge,10);
    }
    assert(!f.a_action.level && f.b_action.level); // B is higher, so only B gets DOWN
    assert(f.b.tx.back()[2]==0x40 && f.a.tx.back()[2]==0);
    height(f.a,291); height(f.b,298); tick(f.bridge,10);
    assert(!f.a_action.level && !f.b_action.level); // lower side changed: stop
    assert(!f.bridge.armed_for_test());
  }
  {
    Fixture f;
    height(f.a,305); height(f.b,290); tick(f.bridge);
    f.bridge.simulate_failed_alignment();
    f.remote_action.level=true; command(f.remote,0x40); tick(f.bridge,10);
    assert(f.a_action.level && !f.b_action.level);
    f.remote_action.level=false; command(f.remote,0); tick(f.bridge,10);
    assert(!f.a_action.level && !f.b_action.level); // releasing DOWN stops recovery
    assert(!f.bridge.armed_for_test());
  }
  std::cout << "jsdrive2 host bridge tests passed\n";
}
