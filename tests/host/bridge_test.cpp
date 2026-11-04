#include "components/jsdrive2/jsdrive2.h"
#include <cassert>
#include <cmath>
#include <iostream>

using esphome::jsdrive2::JSDrive2;
using esphome::uart::UARTComponent;
using esphome::GPIOPin;

struct ProbeBridge : JSDrive2 {
  using JSDrive2::recover_;
  void simulate_failed_alignment() {
    armed_=false; align_attempts_=3; align_high_box_=0; align_wake_attempted_=true;
  }
  bool override_active() const {
    return override_align_ && !armed_;
  }
  bool recovered_alignment_cleared() const {
    return armed_ && !override_align_ && align_attempts_==0;
  }
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
static void tick(JSDrive2 &b, uint32_t n=1) { esphome::test_ms+=n; b.loop(); }

struct Fixture {
  ProbeBridge bridge;
  UARTComponent remote,a,b;
  GPIOPin remote_action, remote_awake, a_action,b_action,a_awake,b_awake;
  Fixture() {
    esphome::test_ms=1000;
    a_awake.level=b_awake.level=true;
    bridge.set_remote_uart(&remote); bridge.set_desk_uart_a(&a); bridge.set_desk_uart_b(&b);
    bridge.set_remote_action_pin(&remote_action); bridge.set_remote_awake_pin(&remote_awake);
    bridge.set_box_a_action_pin(&a_action); bridge.set_box_b_action_pin(&b_action);
    bridge.set_box_a_awake_pin(&a_awake); bridge.set_box_b_awake_pin(&b_awake);
    bridge.setup();
  }
  void pair(int n) { height(a,n); height(b,n); tick(bridge); }
};

int main() {
  {
    Fixture f;
    f.pair(285); command(f.remote,0); tick(f.bridge,10); tick(f.bridge,10);
    assert(shown(f.remote)==285);
    f.remote_action.level=true; command(f.remote,0x20); tick(f.bridge,10);
    assert(f.a_action.level && f.b_action.level);
    tick(f.bridge,31);
    assert(!f.a_action.level && !f.b_action.level); // stale handset stream drops the action lines
  }
  {
    Fixture f;
    f.pair(285); command(f.remote,0); tick(f.bridge,10);
    f.remote_action.level=true; command(f.remote,0x20); tick(f.bridge,10);
    height(f.a,290); tick(f.bridge,10);
    assert(!f.a_action.level && !f.b_action.level); // 0.5 split stops both sides
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
    assert(f.a_action.level && !f.b_action.level); // wake only the sleeping box
    tick(f.bridge,22); assert(!f.a_action.level && !f.b_action.level);
  }
  {
    Fixture f;
    f.a_awake.level=false; f.b_awake.level=false;
    f.remote_action.level=true; command(f.remote,0x20); tick(f.bridge,10);
    tick(f.bridge,10);
    assert(shown(f.remote)!=505); // first wake press is not a fault
    assert(f.a_action.level && f.b_action.level);
  }
  {
    Fixture f;
    height(f.a,299); height(f.b,297); tick(f.bridge);
    blank(f.a); blank(f.b); tick(f.bridge,400);
    assert(f.a_action.level && f.b_action.level); // stale heights start wake, not correction
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
    assert(f.bridge.override_active());
    height(f.a,285); height(f.b,285); tick(f.bridge);
    assert(f.bridge.recovered_alignment_cleared());
  }
  {
    Fixture f;
    f.pair(285); blank(f.a); blank(f.b); tick(f.bridge,10);
    assert(!f.bridge.recall_preset(3));
    for (const auto *u : {&f.a,&f.b})
      for (const auto &frame : u->tx) assert(frame[2]!=0x08);
    assert(!f.a_action.level && !f.b_action.level);
  }
  {
    Fixture f;
    height(f.a,338); height(f.b,286); tick(f.bridge);
    f.bridge.simulate_failed_alignment();
    height(f.a,338); height(f.b,286); tick(f.bridge,900);
    // The recovery chord is valid even when the handset action line is low.
    for (int i=0;i<7;i++) {
      height(f.a,338); height(f.b,286); command(f.remote,0x60); tick(f.bridge,100);
    }
    assert(f.bridge.override_active());
    assert(!f.a_action.level && !f.b_action.level);
    command(f.remote,0); height(f.a,338); height(f.b,286); tick(f.bridge,10);
    assert(f.a_action.level && !f.b_action.level); // only the higher box gets DOWN
    assert(f.a.tx.back()[2]==0x40);
    for (int i=0;i<19;i++) tick(f.bridge,10);
    assert(!f.a_action.level && !f.b_action.level);
    height(f.a,337); height(f.b,286); tick(f.bridge,800);
    height(f.a,337); height(f.b,286); tick(f.bridge,400);
    assert(f.a_action.level && !f.b_action.level);
    for (int i=0;i<19;i++) tick(f.bridge,10);
    height(f.a,286); height(f.b,286); tick(f.bridge,800);
    assert(f.bridge.recovered_alignment_cleared());
  }
  std::cout << "jsdrive2 host bridge tests passed\n";
}
