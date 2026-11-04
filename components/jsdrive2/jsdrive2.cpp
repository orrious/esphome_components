#include "jsdrive2.h"
#include "esphome/core/log.h"
#include <algorithm>
#include <cstdio>
#include <cstring>

namespace esphome {
namespace jsdrive2 {

static const char *const TAG = "jsdrive2";
static const uint8_t SEG[10] = {0x3F,0x06,0x5B,0x4F,0x66,0x6D,0x7D,0x07,0x7F,0x6F};
const uint8_t JSDrive2::CMD_IDLE[5] = {0xA5,0x00,0x00,0xFF,0xFF};
const uint8_t JSDrive2::CMD_UP[5] = {0xA5,0x00,0x20,0xDF,0xFF};
const uint8_t JSDrive2::CMD_DOWN[5] = {0xA5,0x00,0x40,0xBF,0xFF};
const uint8_t JSDrive2::CMD_M[5] = {0xA5,0x00,0x01,0xFE,0xFF};
const uint8_t JSDrive2::CMD_PRESET[4][5] = {
  {0xA5,0x00,0x02,0xFD,0xFF},{0xA5,0x00,0x04,0xFB,0xFF},
  {0xA5,0x00,0x08,0xF7,0xFF},{0xA5,0x00,0x10,0xEF,0xFF}};

static int digit(uint8_t value) {
  value &= 0x7F;
  for (int i=0;i<10;i++) if (SEG[i] == value) return i;
  return -1;
}
static float decode_height(const uint8_t f[5]) {
  int a=digit(f[1]), b=digit(f[2]), c=digit(f[3]);
  if (a<0 || b<0 || c<0) return NAN;
  int n=(a*10+b)*10+c;
  return n>=259 && n<=515 ? n/10.0f : NAN;
}
static bool valid_display(const uint8_t f[5]) {
  return f[0]==0x5A && (uint8_t)(f[1]+f[2]+f[3])==f[4];
}
static bool valid_command(const uint8_t f[5]) {
  return f[0]==0xA5 && f[4]==0xFF && (uint8_t)(f[1]+f[2]+f[3])==0xFF;
}
static int tenths(float value) { return (int) lroundf(value*10.0f); }
static const char *command_name(uint8_t command) {
  switch (command) {
    case 0x00: return "IDLE";
    case 0x01: return "M";
    case 0x02: return "PRESET 1";
    case 0x04: return "PRESET 2";
    case 0x08: return "PRESET 3";
    case 0x10: return "PRESET 4";
    case 0x20: return "UP";
    case 0x40: return "DOWN";
    case 0x60: return "UP+DOWN";
    default: return "BUTTON COMBINATION";
  }
}

void JSDrive2::setup() {
  for (GPIOPin *pin : {remote_action_pin_, remote_awake_pin_, box_a_.action_pin, box_b_.action_pin,
                       box_a_.awake_pin, box_b_.awake_pin}) if (pin != nullptr) pin->setup();
  for (uart::UARTComponent *bus : {box_a_.uart,box_b_.uart})
    if (bus != nullptr) bus->set_flush_timeout(20);
  set_actions_(false,false);
  if (remote_awake_pin_ != nullptr) remote_awake_pin_->digital_write(true);
  if (activity_sensor_) activity_sensor_->publish_state("IDLE");
  ESP_LOGI(TAG, "Three-session WP-CB01 bridge ready; armed by default");
  publish_();
}

bool JSDrive2::parse_(Parser &p, uint8_t byte, uint8_t out[5]) {
  if (p.length == 0 && byte != p.start) return false;
  p.data[p.length++] = byte;
  if (p.length < 5) return false;
  memcpy(out,p.data,5); p.length=0; return true;
}

void JSDrive2::poll_uart_(uart::UARTComponent *uart, Parser &parser, bool remote, Box *box) {
  if (uart == nullptr) return;
  uint8_t c, frame[5]; uint8_t frames=0;
  while (uart->available() && frames < 8) {
    if (!uart->read_byte(&c)) break;
    if (!parse_(parser,c,frame)) continue;
    frames++;
    if (remote) {
      if (valid_command(frame)) handle_remote_(frame); else parser.invalid++;
    } else {
      if (valid_display(frame)) handle_box_(*box,frame); else parser.invalid++;
    }
  }
}

void JSDrive2::set_actions_(bool a, bool b) {
  set_box_action_(box_a_, a); set_box_action_(box_b_, b);
  if (!a && !b) set_bridge_command_(0);
}

void JSDrive2::set_box_action_(Box &box, bool active) {
  if (box.action_pin != nullptr) box.action_pin->digital_write(active);
  box.action=active;
}

void JSDrive2::send_box_(Box &box, const uint8_t frame[5]) {
  if (box.uart == nullptr) return;
  uint32_t now=millis();
  if (box.last_tx && now-box.last_tx>box.max_tx_gap) box.max_tx_gap=now-box.last_tx;
  box.last_tx=now; box.tx_count++; box.uart->write_array(frame,5);
  if (frame[2] != 0) box.last_nonidle_tx=now;
}

void JSDrive2::send_both_(const uint8_t frame[5], bool action) {
  const bool rising=action && (!box_a_.action || !box_b_.action);
  if (rising) {
    // Match the validated debug bridge: drain idle frames from both UARTs
    // before the pin-5 rising edge of a new press.
    if (box_a_.uart==nullptr || box_b_.uart==nullptr) {
      motion_=MotionState::IDLE; fail_("box UART unavailable before action"); return;
    }
    const auto a=box_a_.uart->flush(), b=box_b_.uart->flush();
    auto drained=[](uart::UARTFlushResult result) {
      return result==uart::UARTFlushResult::UART_FLUSH_RESULT_SUCCESS ||
             result==uart::UARTFlushResult::UART_FLUSH_RESULT_ASSUMED_SUCCESS;
    };
    if (!drained(a) || !drained(b)) {
      // A command behind queued idle bytes cannot be paired reliably with
      // pin 5. Stop without trying another UART command on a failed link.
      motion_=MotionState::IDLE; fail_("box UART did not drain before action"); return;
    }
  }
  set_actions_(action,action); send_box_(box_a_,frame); send_box_(box_b_,frame);
  set_bridge_command_(action?frame[2]:0);
  last_command_tx_=millis();
  if (rising) ESP_LOGI(TAG,"Both box action outputs asserted with command 0x%02X",frame[2]);
}

bool JSDrive2::box_ready_(const Box &box, uint32_t now) const {
  return box.seen && box.awake && box.wake==WakeState::IDLE && now-box.last_frame<=LINK_MS;
}

bool JSDrive2::fresh_height_(const Box &box, uint32_t now) const {
  return box_ready_(box,now) && !std::isnan(box.live_height) && now-box.last_height<=LIVE_MS;
}

bool JSDrive2::wake_ready_() const {
  return box_a_.wake==WakeState::IDLE && box_b_.wake==WakeState::IDLE;
}

void JSDrive2::start_wake_(Box &box) {
  if (box.wake!=WakeState::IDLE || box.action_pin==nullptr || box.uart==nullptr) return;
  box.wake=WakeState::ASSERT; box.wake_started=millis();
  set_box_action_(box,true);
  ESP_LOGI(TAG,"Wake %s: pin 5 asserted",box.name);
}

void JSDrive2::poll_wake_(Box &box, uint32_t now) {
  if (box.wake==WakeState::ASSERT && now-box.wake_started>=10) {
    send_box_(box,CMD_UP); box.wake=WakeState::HOLD;
  }
  if (box.wake==WakeState::HOLD && now-box.wake_started>=21) {
    set_box_action_(box,false); send_box_(box,CMD_IDLE);
    box.wake=WakeState::WAIT_HEIGHT;
  }
  if (box.wake==WakeState::WAIT_HEIGHT) {
    if (box.awake && box.seen && !std::isnan(box.live_height) &&
        box.last_height>=box.wake_started && now-box.last_height<=LIVE_MS) {
      box.wake=WakeState::IDLE;
      ESP_LOGI(TAG,"Wake %s verified at %.1f",box.name,box.live_height);
    } else if (now-box.wake_started>1000) {
      box.wake=WakeState::FAILED;
      ESP_LOGW(TAG,"Wake %s failed: no fresh height",box.name);
    }
  }
}

void JSDrive2::poll_automatic_wake_(uint32_t now) {
  for (Box *box : {&box_a_,&box_b_}) {
    if (box->awake) { box->sleep_low_since=0; box->wake_attempted=false; }
    else if (box->sleep_low_since==0) box->sleep_low_since=now;
  }
  if (!armed_ || motion_!=MotionState::IDLE || align_!=AlignState::IDLE ||
      remote_button_down_ || remote_action_ ||
      (last_auto_wake_!=0 && now-last_auto_wake_<WAKE_RETRY_MS)) return;
  bool any=false;
  for (Box *box : {&box_a_,&box_b_}) {
    if (box->wake==WakeState::FAILED) box->wake=WakeState::IDLE;
    if (box->wake==WakeState::IDLE && !box->awake && !box->wake_attempted &&
        box->sleep_low_since!=0 && now-box->sleep_low_since>=SLEEP_STABLE_MS &&
        (!box->seen || now-box->last_frame>LINK_MS)) {
      box->wake_attempted=true; start_wake_(*box); any=true;
    }
  }
  if (any) last_auto_wake_=now;
}

void JSDrive2::poll_box_idle_(Box &box, uint32_t now) {
  if (preset_cancel_until_!=0) return;
  if (box.wake==WakeState::ASSERT || box.wake==WakeState::HOLD) return;
  if (box.action && now-box.last_nonidle_tx<20) return;
  if (now-box.last_tx>=STREAM_MS) send_box_(box,CMD_IDLE);
}

void JSDrive2::poll_action_watchdog_(uint32_t now) {
  for (Box *box : {&box_a_,&box_b_}) {
    if (!box->action) continue;
    // The planned pulse ends at 21 ms in poll_wake_. Allow one delayed loop
    // before the watchdog intervenes; this board has observed 35 ms loop gaps.
    const bool wake_pulse=(box->wake==WakeState::ASSERT || box->wake==WakeState::HOLD) &&
                          now-box->wake_started<=60;
    const bool align_pulse=align_==AlignState::PULSE &&
                           now-align_started_<=uint32_t(align_duration_)+10 &&
                           now-box->last_nonidle_tx<=ACTION_WATCHDOG_MS;
    const bool override_run=override_align_ && align_==AlignState::OVERRIDE_RUN &&
                            box==((align_high_box_==0)?&box_a_:&box_b_) &&
                            now-box->last_nonidle_tx<=60; // observed loop gaps reach 35 ms
    const bool handset=armed_ && remote_action_ && remote_button_down_ &&
                       now-remote_last_frame_<=ACTION_WATCHDOG_MS &&
                       now-box->last_nonidle_tx<=ACTION_WATCHDOG_MS;
    const bool bridge=armed_ && motion_!=MotionState::IDLE &&
                      motion_!=MotionState::DISCOVER_WAIT &&
                      now-box->last_nonidle_tx<=ACTION_WATCHDOG_MS;
    const bool preset_cancel=preset_cancel_until_!=0 &&
                             now-box->last_nonidle_tx<=ACTION_WATCHDOG_MS;
    if (!box->awake && !wake_pulse) {
      set_box_action_(*box,false);
      fail_("box slept while action output was high");
      return;
    }
    if (!(wake_pulse || align_pulse || override_run || handset || bridge || preset_cancel)) {
      set_box_action_(*box,false);
      if (align_==AlignState::PULSE || align_==AlignState::OVERRIDE_RUN)
        fail_("alignment command watchdog expired");
      else if (motion_!=MotionState::IDLE) fail_("action watchdog expired");
      else ESP_LOGW(TAG,"Action watchdog released box %s",box->name);
    }
  }
}
bool JSDrive2::trusted_equal_() const {
  return !std::isnan(box_a_.last_good) && !std::isnan(box_b_.last_good) && tenths(box_a_.last_good)==tenths(box_b_.last_good);
}
bool JSDrive2::live_equal_() const {
  uint32_t now=millis();
  return box_ready_(box_a_,now) && box_ready_(box_b_,now) && !std::isnan(box_a_.live_height) &&
         !std::isnan(box_b_.live_height) && now-box_a_.last_height<=LIVE_MS && now-box_b_.last_height<=LIVE_MS &&
         tenths(box_a_.live_height)==tenths(box_b_.live_height);
}
bool JSDrive2::movement_ready_(bool allow_cached) const {
  uint32_t now=millis();
  return armed_ && box_ready_(box_a_,now) && box_ready_(box_b_,now) &&
         (live_equal_() || (allow_cached && trusted_equal_()));
}

const uint8_t *JSDrive2::command_frame_(uint8_t command) const {
  if (command==0x20) return CMD_UP;
  if (command==0x40) return CMD_DOWN;
  if (command==0x01) return CMD_M;
  for (uint8_t i=0;i<4;i++) if (command==(1U<<(i+1))) return CMD_PRESET[i];
  return CMD_IDLE;
}

void JSDrive2::set_handset_buttons_(uint8_t command) {
  remote_input_command_=command;
  publish_activity_();
}

void JSDrive2::set_bridge_command_(uint8_t command) {
  bridge_output_command_=command;
  publish_activity_();
}

void JSDrive2::publish_activity_() {
  const uint8_t command=remote_input_command_?remote_input_command_:bridge_output_command_;
  if (command==activity_command_) return;
  activity_command_=command;
  if (up_sensor_) up_sensor_->publish_state(command&0x20);
  if (down_sensor_) down_sensor_->publish_state(command&0x40);
  if (memory1_sensor_) memory1_sensor_->publish_state(command&0x02);
  if (memory2_sensor_) memory2_sensor_->publish_state(command&0x04);
  if (memory3_sensor_) memory3_sensor_->publish_state(command&0x08);
  if (memory4_sensor_) memory4_sensor_->publish_state(command&0x10);
  if (activity_sensor_) activity_sensor_->publish_state(command_name(command));
}

void JSDrive2::handle_remote_(const uint8_t f[5]) {
  uint32_t now=millis(); uint8_t command=f[2];
  remote_seen_=true; remote_last_frame_=now; set_handset_buttons_(command);
  bool physical=remote_action_pin_ != nullptr && remote_action_pin_->digital_read();
  remote_action_=physical;
  if (preset_cancel_until_!=0) return;
  if (command==0) {
    if (remote_button_down_ && !override_align_) {
      set_actions_(false,false); send_both_(CMD_IDLE,false);
    }
    remote_button_down_=false; remote_command_=0;
    remote_wake_consumed_=false;
    recovery_started_=0; recovery_last_attempt_=0; recovery_attempted_=false;
    return;
  }
  // Recovery is a command from the handset UART, not a motion command. Some
  // handsets drop the separate action line when two keys are held together.
  if (command==0x60) {
    if (motion_==MotionState::COARSE || motion_==MotionState::FINE_PULSE ||
        motion_==MotionState::FINE_SETTLE || motion_==MotionState::DISCOVER_WAIT)
      stop("simultaneous handset buttons took control");
    if (!override_align_) { set_actions_(false,false); send_both_(CMD_IDLE,false); }
    if (!armed_) {
      if (recovery_started_==0) recovery_started_=now;
      if (!recovery_attempted_ && now-recovery_started_>=500 &&
          (recovery_last_attempt_==0 || now-recovery_last_attempt_>=250)) {
        recovery_last_attempt_=now;
        recovery_attempted_=recover_();
      }
    }
    remote_button_down_=true; remote_command_=command;
    return;
  }
  if (override_align_) {
    fail_("UP+DOWN override interrupted by handset");
    remote_button_down_=true; remote_command_=command;
    return;
  }
  if (!physical) {
    if (remote_button_down_) set_actions_(false,false);
    remote_button_down_=false;
    remote_wake_consumed_=false;
    return;
  }
  const bool changed=command!=remote_command_;
  if ((command & (command-1)) != 0) {
    if (motion_==MotionState::COARSE || motion_==MotionState::FINE_PULSE ||
        motion_==MotionState::FINE_SETTLE || motion_==MotionState::DISCOVER_WAIT)
      stop("simultaneous handset buttons took control");
    set_actions_(false,false); send_both_(CMD_IDLE,false);
    remote_button_down_=true; remote_command_=command;
    recovery_started_=0; recovery_last_attempt_=0; recovery_attempted_=false;
    return;
  }
  recovery_started_=0; recovery_last_attempt_=0; recovery_attempted_=false;
  const bool preset=command==0x02 || command==0x04 || command==0x08 || command==0x10;
  const bool memory=command==0x01 || preset;
  const bool manual=command==0x20 || command==0x40;
  if (manual && remote_wake_consumed_) {
    for (Box *box : {&box_a_,&box_b_})
      if (box->wake!=WakeState::ASSERT && box->wake!=WakeState::HOLD)
        set_box_action_(*box,false);
    remote_button_down_=true; remote_command_=command;
    return;
  }
  if (changed && motion_!=MotionState::IDLE && motion_!=MotionState::PRESET &&
      motion_!=MotionState::SAVE_M && motion_!=MotionState::SAVE_SLOT) {
    stop("handset took control");
  }
  if (align_!=AlignState::IDLE) {
    if (fresh_height_(box_a_,now) && fresh_height_(box_b_,now) &&
        abs(tenths(box_a_.live_height)-tenths(box_b_.live_height))<=1) {
      align_=AlignState::IDLE; set_actions_(false,false);
    } else {
      set_actions_(false,false); remote_button_down_=true; remote_command_=command;
      return;
    }
  }
  if (motion_==MotionState::PRESET && changed && command==0x01) {
    stop("M canceled preset travel");
    remote_button_down_=true; remote_command_=command;
    return;
  }
  if (motion_==MotionState::PRESET && changed && preset) {
    if (fresh_height_(box_a_,now) && fresh_height_(box_b_,now) &&
        abs(tenths(box_a_.live_height)-tenths(box_b_.live_height))<tenths(hard_skew_)) {
      clear_preset_(); motion_=MotionState::IDLE;
    } else {
      set_actions_(false,false); remote_button_down_=true; remote_command_=command;
      return;
    }
  }
  const bool manual_ready=manual && armed_ && fresh_height_(box_a_,now) &&
      fresh_height_(box_b_,now) &&
      abs(tenths(box_a_.live_height)-tenths(box_b_.live_height))<tenths(hard_skew_) &&
      ((box_a_.action && box_b_.action) ||
       abs(tenths(box_a_.live_height)-tenths(box_b_.live_height))<=1);
  const bool ready=movement_ready_(memory) || manual_ready ||
      (motion_==MotionState::PRESET && manual &&
       fresh_height_(box_a_,now) && fresh_height_(box_b_,now) &&
       abs(tenths(box_a_.live_height)-tenths(box_b_.live_height))<tenths(hard_skew_));
  if (!ready) {
    if (manual) remote_wake_consumed_=true;
    // Do not cancel a bounded wake pulse when the handset repeats its held
    // button frame. The other box must remain low until both are ready.
    for (Box *box : {&box_a_,&box_b_})
      if (box->wake!=WakeState::ASSERT && box->wake!=WakeState::HOLD)
        set_box_action_(*box,false);
    if (manual && changed && armed_) {
      for (Box *box : {&box_a_,&box_b_}) {
        if (box->wake==WakeState::FAILED) box->wake=WakeState::IDLE;
        if (box->wake==WakeState::IDLE &&
            (!box->awake || box->display==DisplayState::BLANK || !fresh_height_(*box,now)))
          start_wake_(*box);
      }
    }
    if (changed) {
      ESP_LOGW(TAG,"Handset command 0x%02X waiting for synchronized heights",command);
      // A first arrow press may only wake sleeping boxes. Its missing live
      // height is expected until the bounded wake handshake completes.
      // Report SOS only for an unavailable link when no wake is in progress.
      const bool waking=manual && (box_a_.wake==WakeState::ASSERT || box_a_.wake==WakeState::HOLD ||
                                   box_a_.wake==WakeState::WAIT_HEIGHT || box_b_.wake==WakeState::ASSERT ||
                                   box_b_.wake==WakeState::HOLD || box_b_.wake==WakeState::WAIT_HEIGHT);
      if (!waking && (!box_ready_(box_a_,now) || !box_ready_(box_b_,now))) {
        sos_=true; sos_until_=now+3000; error_="SOS: box link unavailable";
      }
    }
    remote_button_down_=true; remote_command_=command;
    return;
  }
  if (preset && changed) start_preset_(command==0x02?1:command==0x04?2:command==0x08?3:4,true);
  // Both handset and HA preset requests use the same on-wire command frame.
  // The handset controls release timing through its action pin; HA synthesizes
  // that press in poll_preset_.
  send_both_(preset?command_frame_(command):f,true);
  remote_button_down_=true; remote_command_=command;
}

std::string JSDrive2::display_text_(const Box &b) const {
  char buf[40];
  switch (b.display) {
    case DisplayState::BLANK: return "display off";
    case DisplayState::HEIGHT: snprintf(buf,sizeof(buf),"height %.1f",b.last_good); return buf;
    case DisplayState::MEMORY: return "S- memory prompt";
    case DisplayState::ERROR: return "E04";
    case DisplayState::OTHER: snprintf(buf,sizeof(buf),"raw %02X %02X %02X",b.glyph[0],b.glyph[1],b.glyph[2]); return buf;
    default: return "unknown";
  }
}

void JSDrive2::handle_box_(Box &box, const uint8_t f[5]) {
  uint32_t now=millis(); float h=decode_height(f); DisplayState prior=box.display;
  const auto prior_glyph=box.glyph;
  box.seen=true; box.last_frame=now; box.glyph={{f[1],f[2],f[3]}};
  if (!std::isnan(h)) {
    box.display=DisplayState::HEIGHT;
    if (motion_==MotionState::PRESET) handle_preset_height_(box,h,now);
    else {
      box.live_height=h; box.last_height=now;
      if (std::isnan(box.last_good) || tenths(box.last_good)!=tenths(h)) box.last_change=now;
      box.last_good=h;
    }
  } else if (f[1]==0 && f[2]==0 && f[3]==0) {
    // A single blank frame can occur between numeric reports. Keep the last
    // numeric value until last_height exceeds LIVE_MS; do not stop motion on
    // one display frame.
    box.display=DisplayState::BLANK;
    if (preset_phase_==PresetPhase::WAIT_MOTION &&
        box_a_.display==DisplayState::BLANK && box_b_.display==DisplayState::BLANK)
      preset_announcement_visible_=false;
  } else if (f[1]==0x6D && (f[2]==0x40 || f[2]==0) && f[3]==0) {
    box.display=DisplayState::MEMORY;
  } else {
    // E04 is E,0,4. Keep unknown glyphs visible rather than discarding them.
    box.display=(digit(f[2])==0 && digit(f[3])==4) ? DisplayState::ERROR : DisplayState::OTHER;
    if (box.display==DisplayState::ERROR) error_="E04";
  }
  if (prior!=box.display || prior_glyph!=box.glyph)
    ESP_LOGD(TAG,"Box %s: %s",box.name,display_text_(box).c_str());
  if (box.display==DisplayState::ERROR &&
      (motion_!=MotionState::IDLE || box_a_.action || box_b_.action)) {
    fail_("control box reported E04"); return;
  }
  update_confirmed_();
  if ((motion_!=MotionState::IDLE || box_a_.action || box_b_.action) &&
      motion_!=MotionState::PRESET && !override_align_ && wake_ready_() &&
      !std::isnan(box_a_.live_height) && !std::isnan(box_b_.live_height) &&
      fabsf(box_a_.live_height-box_b_.live_height)+0.001f>=hard_skew_) fail_("box height difference reached hard limit");
}

void JSDrive2::start_preset_(uint8_t preset, bool from_handset) {
  const uint32_t now=millis();
  active_preset_=preset; preset_from_handset_=from_handset;
  preset_phase_=PresetPhase::ANNOUNCEMENT; preset_motion_seen_=false;
  preset_start_=trusted_equal_()?box_a_.last_good:NAN;
  preset_target_=NAN; preset_raw_a_=box_a_.last_good; preset_raw_b_=box_b_.last_good;
  preset_announcement_visible_=false;
  preset_queue_a_.count=0; preset_queue_b_.count=0; preset_pair_wait_=0;
  preset_last_change_=now; preset_first_motion_=0;
  motion_=MotionState::PRESET; motion_started_=phase_started_=now;
}

void JSDrive2::clear_preset_() {
  preset_phase_=PresetPhase::IDLE; preset_queue_a_.count=0; preset_queue_b_.count=0;
  preset_pair_wait_=0; preset_target_=NAN; preset_from_handset_=false;
  preset_announcement_visible_=false;
}

void JSDrive2::commit_preset_pair_(float height, uint32_t now) {
  if (preset_phase_==PresetPhase::ANNOUNCEMENT &&
      !std::isnan(preset_start_) && tenths(height)!=tenths(preset_start_)) {
    preset_target_=height; preset_phase_=PresetPhase::WAIT_MOTION;
    preset_announcement_visible_=true;
    ESP_LOGI(TAG,"Preset %u destination %.1f matched by both boxes",active_preset_,height);
    return;
  }
  if (preset_phase_==PresetPhase::WAIT_MOTION && !std::isnan(preset_target_) &&
      tenths(height)==tenths(preset_target_)) {
    preset_announcement_visible_=true; return;
  }
  if (preset_phase_==PresetPhase::WAIT_MOTION && !std::isnan(preset_start_) &&
      tenths(height)==tenths(preset_start_)) {
    preset_announcement_visible_=false; return;
  }
  if (preset_phase_==PresetPhase::ANNOUNCEMENT) return;
  for (Box *box : {&box_a_,&box_b_}) {
    if (std::isnan(box->last_good) || tenths(box->last_good)!=tenths(height)) box->last_change=now;
    box->live_height=height; box->last_height=now; box->last_good=height;
  }
  preset_phase_=PresetPhase::MOVING; preset_motion_seen_=true;
  preset_announcement_visible_=false;
  if (preset_first_motion_==0) preset_first_motion_=now;
  preset_last_change_=now;
  update_confirmed_();
}

void JSDrive2::handle_preset_height_(Box &box, float height, uint32_t now) {
  float &raw=(&box==&box_a_)?preset_raw_a_:preset_raw_b_;
  HeightQueue &queue=(&box==&box_a_)?preset_queue_a_:preset_queue_b_;
  if (!std::isnan(raw) && tenths(raw)==tenths(height)) return;
  raw=height;
  if (preset_phase_==PresetPhase::MOVING && !std::isnan(preset_raw_a_) &&
      !std::isnan(preset_raw_b_) &&
      abs(tenths(preset_raw_a_)-tenths(preset_raw_b_))>=tenths(hard_skew_)) {
    fail_("preset physical heights exceeded hard skew"); return;
  }
  if (queue.count==HeightQueue::CAPACITY) {
    for (uint8_t i=1;i<queue.count;i++) { queue.values[i-1]=queue.values[i]; queue.times[i-1]=queue.times[i]; }
    queue.count--;
  }
  queue.values[queue.count]=height; queue.times[queue.count]=now; queue.count++;
  while (preset_queue_a_.count && preset_queue_b_.count) {
    int match_a=-1, match_b=-1, best=2*HeightQueue::CAPACITY+1;
    for (uint8_t a=0;a<preset_queue_a_.count;a++) for (uint8_t b=0;b<preset_queue_b_.count;b++) {
      if (tenths(preset_queue_a_.values[a])==tenths(preset_queue_b_.values[b]) && a+b<best) {
        match_a=a; match_b=b; best=a+b;
      }
    }
    if (match_a<0) break;
    const float matched=preset_queue_a_.values[match_a];
    const uint32_t matched_at=std::max(preset_queue_a_.times[match_a],preset_queue_b_.times[match_b]);
    const uint8_t used_a=match_a+1, used_b=match_b+1;
    for (uint8_t i=used_a;i<preset_queue_a_.count;i++) {
      preset_queue_a_.values[i-used_a]=preset_queue_a_.values[i];
      preset_queue_a_.times[i-used_a]=preset_queue_a_.times[i];
    }
    for (uint8_t i=used_b;i<preset_queue_b_.count;i++) {
      preset_queue_b_.values[i-used_b]=preset_queue_b_.values[i];
      preset_queue_b_.times[i-used_b]=preset_queue_b_.times[i];
    }
    preset_queue_a_.count-=used_a; preset_queue_b_.count-=used_b;
    commit_preset_pair_(matched,matched_at);
    if (motion_!=MotionState::PRESET) return;
  }
  if (preset_queue_a_.count && preset_queue_b_.count)
    preset_pair_wait_=std::min(preset_queue_a_.times[0],preset_queue_b_.times[0]);
  else if (preset_queue_a_.count) preset_pair_wait_=preset_queue_a_.times[0];
  else if (preset_queue_b_.count) preset_pair_wait_=preset_queue_b_.times[0];
  else preset_pair_wait_=0;
}

void JSDrive2::poll_preset_(uint32_t now) {
  if (preset_from_handset_ && (!remote_seen_ || now-remote_last_frame_>LINK_MS)) {
    fail_("handset link lost during preset travel"); return;
  }
  if (box_a_.action || box_b_.action) {
    if (!preset_from_handset_) {
      if (now-phase_started_>=190) {
        set_actions_(false,false); send_both_(CMD_IDLE,false);
      } else if (now-last_command_tx_>=STREAM_MS) {
        send_both_(CMD_PRESET[active_preset_-1],true);
      }
    }
  }
  if (!box_ready_(box_a_,now) || !box_ready_(box_b_,now)) {
    fail_("box link or awake input lost during preset travel"); return;
  }
  if (preset_motion_seen_ && now-preset_last_change_>=1200 && !remote_action_) {
    // A box can skip the final tenth. Preserve each physical report for the
    // bounded post-stop alignment rather than treating the queue as a fault.
    if (!std::isnan(preset_raw_a_) && !std::isnan(preset_raw_b_)) {
      if (std::isnan(box_a_.last_good) || tenths(box_a_.last_good)!=tenths(preset_raw_a_)) box_a_.last_change=now;
      if (std::isnan(box_b_.last_good) || tenths(box_b_.last_good)!=tenths(preset_raw_b_)) box_b_.last_change=now;
      box_a_.last_good=preset_raw_a_; box_b_.last_good=preset_raw_b_;
      box_a_.live_height=preset_raw_a_; box_b_.live_height=preset_raw_b_;
      box_a_.last_height=now; box_b_.last_height=now;
      update_confirmed_();
    }
    clear_preset_(); motion_=MotionState::IDLE;
    set_actions_(false,false); return;
  }
  if (!preset_motion_seen_ && now-motion_started_>5000) {
    clear_preset_(); finish_motion_("preset completed without travel"); return;
  }
  if (preset_pair_wait_ && now-preset_pair_wait_>PRESET_PAIR_MS && !preset_motion_seen_) {
    fail_("preset announcement heights did not match"); return;
  }
  // During physical travel a box can skip an intermediate or final tenth.
  // The raw 0.5 split and link watchdogs protect motion; unmatched display
  // tenths are resolved by the final settlement and alignment path.
}

void JSDrive2::poll_preset_cancel_(uint32_t now) {
  if (preset_cancel_until_==0) return;
  if (!box_a_.awake || !box_b_.awake) {
    preset_cancel_until_=0; preset_cancel_frame_=nullptr;
    set_actions_(false,false);
    return;
  }
  if (int32_t(now-preset_cancel_until_)>=0) {
    preset_cancel_until_=0; preset_cancel_frame_=nullptr;
    set_actions_(false,false); send_box_(box_a_,CMD_IDLE); send_box_(box_b_,CMD_IDLE);
    return;
  }
  if (now-last_command_tx_>=STREAM_MS) send_both_(preset_cancel_frame_,true);
}

void JSDrive2::update_confirmed_() {
  if (trusted_equal_()) {
    float h=(box_a_.last_good+box_b_.last_good)/2.0f;
    if (std::isnan(confirmed_height_) || tenths(h)!=tenths(confirmed_height_)) {
      confirmed_height_=h;
      if (height_sensor_) height_sensor_->publish_state(h);
      if (cover_) { cover_->position=(h-min_height_)/(max_height_-min_height_); cover_->publish_state(); }
    }
  }
}

void JSDrive2::send_remote_status_() {
  if (remote_uart_==nullptr) return;
  uint8_t f[5]={0x5A,0,0,0,0}; uint32_t now=millis();
  if (sos_ && now<sos_until_) { f[1]=0x6D; f[2]=0x3F; f[3]=0x6D; }
  else if (box_a_.display==DisplayState::MEMORY && box_b_.display==DisplayState::MEMORY) {
    f[1]=0x6D; f[2]=((now/500)&1)?0:0x40;
  } else if (!std::isnan(confirmed_height_) &&
             (activity_command_!=0 || motion_!=MotionState::IDLE || align_!=AlignState::IDLE ||
              box_a_.display!=DisplayState::BLANK || box_b_.display!=DisplayState::BLANK)) {
    const float shown=(preset_phase_==PresetPhase::WAIT_MOTION &&
                       preset_announcement_visible_ && !std::isnan(preset_target_))
                          ? preset_target_ : confirmed_height_;
    int n=tenths(shown), whole=n/10;
    f[1]=SEG[(whole/10)%10]; f[2]=SEG[whole%10]|0x80; f[3]=SEG[n%10];
  }
  f[4]=(uint8_t)(f[1]+f[2]+f[3]);
  if (remote_last_tx_ && now-remote_last_tx_>remote_max_tx_gap_) remote_max_tx_gap_=now-remote_last_tx_;
  remote_last_tx_=now; remote_tx_count_++; remote_uart_->write_array(f,5);
}

void JSDrive2::fail_(const std::string &reason, bool disarm, bool show_sos) {
  // Idle frames alone do not cancel autonomous preset travel. A short press
  // of the opposite arrow is the observed handset cancel operation.
  const bool was_preset=motion_==MotionState::PRESET;
  float preset_direction=NAN;
  if (was_preset && !std::isnan(preset_start_)) {
    if (!std::isnan(preset_target_)) preset_direction=preset_target_-preset_start_;
    else {
      const float da=std::isnan(preset_raw_a_)?0:preset_raw_a_-preset_start_;
      const float db=std::isnan(preset_raw_b_)?0:preset_raw_b_-preset_start_;
      preset_direction=fabsf(da)>=fabsf(db)?da:db;
    }
  }
  preset_cancel_until_=0; preset_cancel_frame_=nullptr;
  set_actions_(false,false); send_box_(box_a_,CMD_IDLE); send_box_(box_b_,CMD_IDLE);
  for (Box *box : {&box_a_,&box_b_})
    if (box->wake==WakeState::ASSERT || box->wake==WakeState::HOLD) box->wake=WakeState::FAILED;
  motion_=MotionState::IDLE; align_=AlignState::IDLE; clear_preset_();
  align_wake_attempted_=false; override_align_=false;
  target_height_=NAN; direction_=0; pending_preset_=0;
  fault_=reason; if (disarm) armed_=false;
  if (!std::isnan(preset_direction) && fabsf(preset_direction)>=0.05f) {
    preset_cancel_frame_=preset_direction>0?CMD_DOWN:CMD_UP;
    preset_cancel_until_=millis()+190;
    send_both_(preset_cancel_frame_,true);
    ESP_LOGW(TAG,"Preset cancel: opposite arrow sent to both boxes for 190 ms");
  }
  if (show_sos) { sos_=true; sos_until_=millis()+3000; error_="SOS: "+reason; }
  ESP_LOGE(TAG,"Stopped: %s%s",reason.c_str(),disarm?"; bridge disarmed":""); publish_();
}

void JSDrive2::finish_motion_(const char *reason) {
  set_actions_(false,false); send_box_(box_a_,CMD_IDLE); send_box_(box_b_,CMD_IDLE);
  motion_=MotionState::IDLE; clear_preset_();
  target_height_=NAN; direction_=0; fine_taps_=0; pending_preset_=0;
  ESP_LOGI(TAG,"%s",reason); update_confirmed_(); publish_();
}

bool JSDrive2::discover_height() {
  if (!armed_ || motion_!=MotionState::IDLE || remote_button_down_ || align_!=AlignState::IDLE) return false;
  const uint32_t now=millis();
  if (fresh_height_(box_a_,now) && fresh_height_(box_b_,now) && trusted_equal_()) return true;
  pending_preset_=0; motion_=MotionState::DISCOVER_WAIT; motion_started_=phase_started_=now;
  for (Box *box : {&box_a_,&box_b_}) {
    if (box->wake==WakeState::FAILED) box->wake=WakeState::IDLE;
    if (!fresh_height_(*box,now)) start_wake_(*box);
  }
  ESP_LOGI(TAG,"Discovering each box height with its own wake session"); return true;
}

bool JSDrive2::request_height(float height) {
  height=roundf(height*10.0f)/10.0f;
  if (height<min_height_ || height>max_height_) { fault_="target outside configured range"; publish_(); return false; }
  if (!armed_ || motion_!=MotionState::IDLE || remote_button_down_ || align_!=AlignState::IDLE) return false;
  pending_preset_=0; target_height_=height; fine_taps_=0; motion_started_=millis();
  if (!movement_ready_(true) || !live_equal_()) {
    const uint32_t now=millis();
    motion_=MotionState::DISCOVER_WAIT; phase_started_=now;
    for (Box *box : {&box_a_,&box_b_}) {
      if (box->wake==WakeState::FAILED) box->wake=WakeState::IDLE;
      if (!fresh_height_(*box,now)) start_wake_(*box);
    }
    ESP_LOGI(TAG,"Target %.1f requested; discovering height first",height); return true;
  }
  int current=tenths(confirmed_height_), target=tenths(height);
  if (current==target) { finish_motion_("already at requested height"); return true; }
  direction_=target>current?1:-1; motion_=MotionState::COARSE; phase_started_=millis();
  ESP_LOGI(TAG,"Target move %.1f -> %.1f",confirmed_height_,height); return true;
}

bool JSDrive2::recall_preset(uint8_t preset) {
  if (preset<1 || preset>4 || !armed_ || motion_!=MotionState::IDLE ||
      remote_button_down_ || remote_action_ || align_!=AlignState::IDLE) return false;
  const uint32_t now=millis();
  pending_preset_=preset; motion_=MotionState::DISCOVER_WAIT;
  motion_started_=phase_started_=now;
  // A handset press can only reach the boxes once both box sessions are
  // awake. HA uses that same preset frame and action sequence after wake.
  for (Box *box : {&box_a_,&box_b_}) {
    if (box->wake==WakeState::FAILED) box->wake=WakeState::IDLE;
    if (!fresh_height_(*box,now) || box->display==DisplayState::BLANK)
      start_wake_(*box);
  }
  ESP_LOGI(TAG,"Recall preset %u: waiting for fresh settled box heights",preset);
  return true;
}

bool JSDrive2::save_preset(uint8_t preset) {
  if (preset<1 || preset>4 || motion_!=MotionState::IDLE || !movement_ready_(true)) return false;
  save_preset_=preset; motion_=MotionState::SAVE_M; motion_started_=phase_started_=millis();
  send_both_(CMD_M,true); ESP_LOGI(TAG,"Save preset %u: requesting memory prompt",preset); return true;
}

void JSDrive2::stop(const char *reason) { fail_(reason,false,false); }
void JSDrive2::arm() {
  if (armed_) return;
  if (!std::isnan(box_a_.last_good) && !std::isnan(box_b_.last_good) &&
      abs(tenths(box_a_.last_good)-tenths(box_b_.last_good))>=tenths(hard_skew_)) {
    ESP_LOGW(TAG,"Arm refused: large skew requires held UP+DOWN override");
    return;
  }
  if (!recover_()) ESP_LOGW(TAG,"Arm refused until both boxes are healthy and settled");
}
void JSDrive2::disarm() { stop("disarmed"); armed_=false; publish_(); }

bool JSDrive2::recover_() {
  const uint32_t now=millis();
  const bool linked_a=box_a_.seen && box_a_.awake && now-box_a_.last_frame<=LINK_MS;
  const bool linked_b=box_b_.seen && box_b_.awake && now-box_b_.last_frame<=LINK_MS;
  if (!linked_a || !linked_b ||
      std::isnan(box_a_.last_good) || std::isnan(box_b_.last_good) ||
      motion_!=MotionState::IDLE || align_!=AlignState::IDLE) {
    ESP_LOGW(TAG,"Recovery chord waiting for healthy, stopped boxes");
    return false;
  }
  set_actions_(false,false); sos_=false; sos_until_=0;
  if (box_a_.wake==WakeState::FAILED) box_a_.wake=WakeState::IDLE;
  if (box_b_.wake==WakeState::FAILED) box_b_.wake=WakeState::IDLE;
  // The chord explicitly overrides the ordinary 0.5 limit. Keep all normal
  // commands locked out until fresh reports show that the boxes match.
  armed_=false; override_align_=true; override_started_=now;
  fault_="override alignment in progress"; error_="none";
  align_attempts_=0; align_high_box_=0xFF; align_wake_attempted_=false;
  override_last_progress_=0; override_high_last_=0;
  align_lower_before_=0;
  ESP_LOGW(TAG,"UP+DOWN override: lowering higher box from %.1f / %.1f",
           box_a_.last_good,box_b_.last_good);
  publish_(); return true;
}

void JSDrive2::start_command_(uint8_t command, bool from_remote) {
  (void) from_remote; const uint8_t *f=command_frame_(command); send_both_(f,command!=0);
}

void JSDrive2::poll_motion_() {
  if (motion_==MotionState::IDLE) return;
  uint32_t now=millis();
  if (now-motion_started_>TARGET_TIMEOUT_MS) { fail_("45 second transaction timeout"); return; }
  if (motion_==MotionState::DISCOVER_WAIT) {
    if (pending_preset_!=0) {
      if (movement_ready_(false) && align_==AlignState::IDLE &&
          now-box_a_.last_change>=400 && now-box_b_.last_change>=400 &&
          !remote_action_) {
        const uint8_t preset=pending_preset_; pending_preset_=0;
        start_preset_(preset,false);
        send_both_(CMD_PRESET[preset-1],true);
        ESP_LOGI(TAG,"Recall preset %u: handset frame and pin 5 asserted together",preset);
      } else if (now-phase_started_>8000 && align_==AlignState::IDLE) {
        fail_("preset could not obtain settled synchronized heights");
      }
      return;
    }
    if (movement_ready_(false) && align_==AlignState::IDLE) {
      if (std::isnan(target_height_)) { finish_motion_("height discovery complete"); return; }
      int cur=tenths(confirmed_height_), target=tenths(target_height_);
      if (cur==target) { finish_motion_("already at requested height"); return; }
      direction_=target>cur?1:-1; motion_=MotionState::COARSE; phase_started_=now;
    } else if (now-phase_started_>3000 && align_==AlignState::IDLE)
      fail_("height discovery did not return synchronized box heights");
    return;
  }
  if (!box_ready_(box_a_,now) || !box_ready_(box_b_,now)) { fail_("box link or awake input lost"); return; }

  if (motion_==MotionState::SAVE_M) {
    if (now-phase_started_>=190) {
      set_actions_(false,false); send_both_(CMD_IDLE,false);
      motion_=MotionState::SAVE_SLOT; phase_started_=now;
    } else if (now-last_command_tx_>=STREAM_MS) send_both_(CMD_M,true);
    return;
  }
  if (motion_==MotionState::SAVE_SLOT) {
    if (box_a_.display==DisplayState::MEMORY && box_b_.display==DisplayState::MEMORY) {
      start_preset_(save_preset_,false); send_both_(CMD_PRESET[save_preset_-1],true);
      ESP_LOGI(TAG,"Both boxes accepted memory prompt; storing preset %u",save_preset_); return;
    }
    if (now-phase_started_>3000) fail_("both boxes did not enter memory prompt");
    return;
  }
  if (motion_==MotionState::PRESET) {
    poll_preset_(now);
    return;
  }

  if (std::isnan(box_a_.live_height) || std::isnan(box_b_.live_height) || now-box_a_.last_height>LIVE_MS || now-box_b_.last_height>LIVE_MS) {
    // During settling a blank display is expected; retain last-good positions.
    if (motion_!=MotionState::FINE_SETTLE || now-phase_started_>4000) { fail_("fresh height missing"); return; }
  }
  int a=tenths(box_a_.last_good), b=tenths(box_b_.last_good), target=tenths(target_height_);
  if (abs(a-b)>=tenths(hard_skew_)) { fail_("live box height difference reached hard limit"); return; }
  if (a==target && b==target) { finish_motion_("requested height reached by both boxes"); return; }
  bool crossed=direction_>0 ? (a>target || b>target) : (a<target || b<target);
  if (crossed) { fail_("target crossed before both boxes matched"); return; }
  int remaining=direction_>0 ? target-std::max(a,b) : std::min(a,b)-target;
  if (motion_==MotionState::COARSE && remaining<=tenths(COARSE_RELEASE)) {
    set_actions_(false,false); send_both_(CMD_IDLE,false); motion_=MotionState::FINE_SETTLE; phase_started_=now; return;
  }
  if (motion_==MotionState::FINE_PULSE && now-phase_started_>=FINE_PULSE_MS) {
    set_actions_(false,false); send_both_(CMD_IDLE,false); motion_=MotionState::FINE_SETTLE; phase_started_=now; return;
  }
  if (motion_==MotionState::FINE_SETTLE) {
    if (now-phase_started_<FINE_SETTLE_MS || now-box_a_.last_change<400 || now-box_b_.last_change<400) return;
    if (a!=b) return; // bounded one-sided alignment is started by poll_alignment_.
    if (fine_taps_>=15) { fail_("15 fine taps did not reach target"); return; }
    fine_taps_++; motion_=MotionState::FINE_PULSE; phase_started_=now; last_command_tx_=0;
  }
  if ((motion_==MotionState::COARSE || motion_==MotionState::FINE_PULSE) && now-last_command_tx_>=STREAM_MS) {
    send_both_(direction_>0?CMD_UP:CMD_DOWN,true);
  }
}

void JSDrive2::poll_alignment_() {
  uint32_t now=millis();
  if (!armed_ && !override_align_) {
    if (align_!=AlignState::IDLE) { set_actions_(false,false); align_=AlignState::IDLE; }
    return;
  }
  if (override_align_ && now-override_started_>90000) {
    fail_("UP+DOWN override timed out"); return;
  }
  if (override_align_) {
    if (!box_ready_(box_a_,now) || !box_ready_(box_b_,now)) {
      fail_("box link lost during UP+DOWN override"); return;
    }
    if (align_==AlignState::OVERRIDE_RUN || align_==AlignState::SETTLE) {
      Box &higher=align_high_box_==0?box_a_:box_b_;
      Box &lower=align_high_box_==0?box_b_:box_a_;
      if (!fresh_height_(higher,now)) {
        fail_("higher box height missing during UP+DOWN override"); return;
      }
      const int high=tenths(higher.live_height), low=tenths(lower.last_good);
      if (low!=align_lower_before_) {
        fail_("lower box changed during UP+DOWN override"); return;
      }
      if (high<low) {
        fail_("UP+DOWN override crossed below lower box"); return;
      }
      if (align_==AlignState::OVERRIDE_RUN) {
        if (high<override_high_last_) {
          override_high_last_=high; override_last_progress_=now;
        }
        if (high==low) {
          set_box_action_(higher,false); send_box_(higher,CMD_IDLE);
          align_=AlignState::SETTLE; align_started_=now;
          ESP_LOGI(TAG,"UP+DOWN override reached %.1f; verifying stop",higher.live_height);
          return;
        }
        if (now-override_last_progress_>1500) {
          fail_("higher box made no progress during UP+DOWN override"); return;
        }
        if (now-higher.last_nonidle_tx>=STREAM_MS) send_box_(higher,CMD_DOWN);
        return;
      }
      if (now-align_started_<400) return;
      if (high>low) {
        // Stopping can leave the higher box one step short. Continue the same
        // held DOWN command after verifying the lower box stayed fixed.
        align_=AlignState::OVERRIDE_RUN;
        override_high_last_=high; override_last_progress_=now;
        set_box_action_(higher,true); send_box_(higher,CMD_DOWN);
        return;
      }
      override_align_=false; armed_=true; fault_="none"; error_="none";
      align_=AlignState::IDLE; align_high_box_=0xFF;
      align_wake_attempted_=false; align_attempts_=0;
      update_confirmed_(); publish_();
      ESP_LOGI(TAG,"UP+DOWN override complete: both boxes at %.1f",higher.live_height);
      return;
    }
    if (!fresh_height_(box_a_,now) || !fresh_height_(box_b_,now)) {
      if (!align_wake_attempted_) {
        align_wake_attempted_=true; align_wake_started_=now;
        for (Box *box : {&box_a_,&box_b_}) {
          if (box->wake==WakeState::FAILED) box->wake=WakeState::IDLE;
          if (!fresh_height_(*box,now)) start_wake_(*box);
        }
      } else if (now-align_wake_started_>2000) {
        fail_("UP+DOWN override could not obtain fresh heights");
      }
      return;
    }
    const int a=tenths(box_a_.live_height), b=tenths(box_b_.live_height);
    if (a==b) {
      override_align_=false; armed_=true; fault_="none"; error_="none";
      align_=AlignState::IDLE; align_high_box_=0xFF;
      update_confirmed_(); publish_();
      return;
    }
    align_high_box_=a>b?0:1;
    Box &higher=align_high_box_==0?box_a_:box_b_;
    Box &lower=align_high_box_==0?box_b_:box_a_;
    align_lower_before_=tenths(lower.live_height);
    override_high_last_=tenths(higher.live_height);
    override_last_progress_=now;
    align_=AlignState::OVERRIDE_RUN;
    set_box_action_(lower,false); send_box_(lower,CMD_IDLE);
    set_box_action_(higher,true); send_box_(higher,CMD_DOWN);
    ESP_LOGW(TAG,"UP+DOWN override: holding box %s DOWN from %.1f to %.1f",
             higher.name,higher.live_height,lower.live_height);
    return;
  }
  if (align_==AlignState::PULSE) {
    Box &higher=align_high_box_==0?box_a_:box_b_;
    Box &lower=align_high_box_==0?box_b_:box_a_;
    if (!box_a_.awake || !box_b_.awake) { fail_("box slept during correction pulse"); return; }
    const bool reached_lower=fresh_height_(higher,now) && fresh_height_(lower,now) &&
                             tenths(higher.live_height)<=tenths(lower.live_height);
    if (remote_action_ || remote_button_down_ || reached_lower || now-align_started_>=align_duration_) {
      set_box_action_(higher,false); send_box_(higher,CMD_IDLE);
      align_=AlignState::SETTLE; align_started_=now;
    } else if (now-higher.last_nonidle_tx>=STREAM_MS) send_box_(higher,CMD_DOWN);
    return;
  }
  if (!(motion_==MotionState::IDLE || motion_==MotionState::DISCOVER_WAIT ||
        motion_==MotionState::FINE_SETTLE) || remote_button_down_ || remote_action_) return;
  if (!box_ready_(box_a_,now) || !box_ready_(box_b_,now) ||
      std::isnan(box_a_.last_good) || std::isnan(box_b_.last_good)) {
    if (align_==AlignState::SETTLE && now-align_started_>2000)
      fail_("box link lost after correction pulse");
    return;
  }
  if (align_==AlignState::SETTLE && now-align_started_<800) return;
  int d=tenths(box_a_.last_good)-tenths(box_b_.last_good);
  if (d!=0 && (!fresh_height_(box_a_,now) || !fresh_height_(box_b_,now))) {
    if (!align_wake_attempted_) {
      align_wake_attempted_=true; align_wake_started_=now;
      for (Box *box : {&box_a_,&box_b_}) {
        if (box->wake==WakeState::FAILED) box->wake=WakeState::IDLE;
        if (!fresh_height_(*box,now)) start_wake_(*box);
      }
      ESP_LOGI(TAG,"Alignment needs fresh heights; waking stale box sessions");
    } else if (now-align_wake_started_>2000) {
      fail_("alignment could not obtain fresh box heights");
    }
    return;
  }
  if (d==0) {
    const bool completed=align_!=AlignState::IDLE;
    if (completed) ESP_LOGI(TAG,"Boxes aligned at %.1f",box_a_.last_good);
    align_=AlignState::IDLE; align_attempts_=0; align_high_box_=0xFF;
    align_wake_attempted_=false;
    update_confirmed_(); if (completed) publish_(); return;
  }
  if (abs(d)>=tenths(hard_skew_)) {
    fail_("stopped boxes exceed hard skew"); return;
  }
  if (now-box_a_.last_change<400 || now-box_b_.last_change<400) return;
  const uint8_t high=d>0?0:1;
  if (align_high_box_!=0xFF && align_high_box_!=high) {
    fail_("correction crossed past the lower box"); return;
  }
  if (align_attempts_>=3) {
    fail_("alignment pulse limit reached"); return;
  }
  Box &higher=high==0?box_a_:box_b_;
  // A 190 ms hold is the shortest tap exercised by the bridge's target
  // positioning path. Stop sooner if the higher box reports the lower height.
  align_duration_=190;
  align_lower_before_=tenths(high==0?box_b_.last_good:box_a_.last_good);
  align_high_box_=high; align_attempts_++;
  align_=AlignState::PULSE; align_started_=now;
  set_box_action_(higher,true); send_box_(higher,CMD_DOWN);
  ESP_LOGW(TAG,"Aligning box %s down for %u ms",higher.name,(unsigned)align_duration_);
}

void JSDrive2::publish_() {
  uint32_t now=millis(); bool ready_a=box_ready_(box_a_,now), ready_b=box_ready_(box_b_,now), known=trusted_equal_();
  bool live=live_equal_(), sync=known && fabsf(box_a_.last_good-box_b_.last_good)<0.15f;
  bool moving=motion_!=MotionState::IDLE || box_a_.action || box_b_.action;
  if (height_a_sensor_) height_a_sensor_->publish_state(box_a_.last_good);
  if (height_b_sensor_) height_b_sensor_->publish_state(box_b_.last_good);
  if (height_difference_sensor_) height_difference_sensor_->publish_state((!std::isnan(box_a_.last_good)&&!std::isnan(box_b_.last_good))?fabsf(box_a_.last_good-box_b_.last_good):NAN);
  if (remote_frame_age_sensor_) remote_frame_age_sensor_->publish_state(remote_last_frame_?now-remote_last_frame_:NAN);
  if (box_a_frame_age_sensor_) box_a_frame_age_sensor_->publish_state(box_a_.last_frame?now-box_a_.last_frame:NAN);
  if (box_b_frame_age_sensor_) box_b_frame_age_sensor_->publish_state(box_b_.last_frame?now-box_b_.last_frame:NAN);
  if (remote_invalid_frames_sensor_) remote_invalid_frames_sensor_->publish_state(remote_parser_.invalid);
  if (box_a_invalid_frames_sensor_) box_a_invalid_frames_sensor_->publish_state(box_a_.parser.invalid);
  if (box_b_invalid_frames_sensor_) box_b_invalid_frames_sensor_->publish_state(box_b_.parser.invalid);
  if (remote_tx_max_gap_sensor_) remote_tx_max_gap_sensor_->publish_state(remote_max_tx_gap_);
  if (box_a_tx_max_gap_sensor_) box_a_tx_max_gap_sensor_->publish_state(box_a_.max_tx_gap);
  if (box_b_tx_max_gap_sensor_) box_b_tx_max_gap_sensor_->publish_state(box_b_.max_tx_gap);
  if (alignment_attempts_sensor_) alignment_attempts_sensor_->publish_state(align_attempts_);
#define PB(ptr,val) if (ptr) ptr->publish_state(val)
  PB(in_sync_sensor_,sync); PB(height_known_sensor_,known); PB(height_live_sensor_,live);
  PB(healthy_sensor_,armed_&&ready_a&&ready_b&&known); PB(remote_connected_sensor_,remote_seen_&&now-remote_last_frame_<=LINK_MS);
  PB(box_a_connected_sensor_,ready_a); PB(box_b_connected_sensor_,ready_b); PB(box_a_awake_sensor_,box_a_.awake);
  PB(box_b_awake_sensor_,box_b_.awake); PB(armed_sensor_,armed_); PB(moving_sensor_,moving);
#undef PB
  const char *op=align_!=AlignState::IDLE?"aligning":motion_==MotionState::IDLE?(activity_command_?command_name(activity_command_):"idle"):motion_==MotionState::COARSE?"target coarse":motion_==MotionState::FINE_PULSE?"target fine pulse":motion_==MotionState::FINE_SETTLE?"target settling":motion_==MotionState::PRESET?"preset":motion_==MotionState::SAVE_M||motion_==MotionState::SAVE_SLOT?"saving preset":"discovering height";
  if (operation_sensor_) operation_sensor_->publish_state(op);
  if (last_fault_sensor_) last_fault_sensor_->publish_state(fault_);
  if (box_a_display_sensor_) box_a_display_sensor_->publish_state(display_text_(box_a_));
  if (box_b_display_sensor_) box_b_display_sensor_->publish_state(display_text_(box_b_));
  if (error_code_sensor_) error_code_sensor_->publish_state(error_);
  if (cover_) {
    cover_->current_operation = moving ? (direction_>0?cover::COVER_OPERATION_OPENING:cover::COVER_OPERATION_CLOSING) : cover::COVER_OPERATION_IDLE;
    cover_->publish_state();
  }
}

void JSDrive2::loop() {
  uint32_t now=millis();
  box_a_.awake=box_a_.awake_pin!=nullptr && box_a_.awake_pin->digital_read();
  box_b_.awake=box_b_.awake_pin!=nullptr && box_b_.awake_pin->digital_read();
  remote_action_=remote_action_pin_!=nullptr && remote_action_pin_->digital_read();
  if (remote_awake_pin_) remote_awake_pin_->digital_write(true);
  poll_action_watchdog_(now);
  poll_uart_(remote_uart_,remote_parser_,true); poll_uart_(box_a_.uart,box_a_.parser,false,&box_a_); poll_uart_(box_b_.uart,box_b_.parser,false,&box_b_);
  now=millis();
  if (box_a_.seen && now-box_a_.last_frame>LINK_MS) box_a_.seen=false;
  if (box_b_.seen && now-box_b_.last_frame>LINK_MS) box_b_.seen=false;
  if (remote_seen_ && now-remote_last_frame_>LINK_MS) {
    remote_seen_=false; remote_button_down_=false; remote_wake_consumed_=false; remote_command_=0;
    if (!override_align_) set_actions_(false,false);
    set_handset_buttons_(0);
    if (motion_==MotionState::PRESET && preset_from_handset_)
      fail_("handset link lost during preset travel");
  }
  poll_wake_(box_a_,now); poll_wake_(box_b_,now);
  poll_automatic_wake_(now);
  poll_motion_();
  poll_alignment_();
  poll_preset_cancel_(now);
  poll_action_watchdog_(millis());
  poll_box_idle_(box_a_,now); poll_box_idle_(box_b_,now);
  if (now-remote_last_tx_>=STREAM_MS) send_remote_status_();
  if (now-last_publish_>=1000) { last_publish_=now; publish_(); }
}

void JSDrive2::dump_config() {
  ESP_LOGCONFIG(TAG,"WP-CB01 dual-box bridge");
  ESP_LOGCONFIG(TAG,"  Range %.1f..%.1f; hard skew %.1f",min_height_,max_height_,hard_skew_);
  LOG_PIN("  Handset action: ",remote_action_pin_); LOG_PIN("  Handset awake: ",remote_awake_pin_);
  LOG_PIN("  Box A action: ",box_a_.action_pin); LOG_PIN("  Box A awake: ",box_a_.awake_pin);
  LOG_PIN("  Box B action: ",box_b_.action_pin); LOG_PIN("  Box B awake: ",box_b_.awake_pin);
}

void JSDrive2::run_button(uint8_t mode) {
  if (mode==0) stop(); else if (mode==1) discover_height(); else if (mode==2) arm(); else if (mode==3) disarm();
  else if (mode>=11&&mode<=14) recall_preset(mode-10); else if (mode>=21&&mode<=24) save_preset(mode-20);
}
void JSDrive2TargetNumber::control(float value) { if (parent_&&parent_->request_height(value)) publish_state(value); }
cover::CoverTraits JSDrive2Cover::get_traits() { cover::CoverTraits t; t.set_supports_position(true); t.set_supports_stop(true); return t; }
void JSDrive2Cover::control(const cover::CoverCall &call) {
  if (!parent_) return;
  if (call.get_stop()) parent_->stop("cover stop");
  if (call.get_position().has_value()) parent_->request_height(parent_->min_height()+*call.get_position()*(parent_->max_height()-parent_->min_height()));
}
void JSDrive2CommandButton::press_action() { if (parent_) parent_->run_button(mode_); }

}  // namespace jsdrive2
}  // namespace esphome
