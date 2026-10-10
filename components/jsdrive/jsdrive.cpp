#include "jsdrive.h"
#include "esphome/core/log.h"
#include <algorithm>
#include <cstdio>

namespace esphome {
namespace jsdrive {

static const char *const TAG = "jsdrive";
static const uint8_t SEG[10] = {0x3F,0x06,0x5B,0x4F,0x66,0x6D,0x7D,0x07,0x7F,0x6F};
const uint8_t JSDrive::CMD_IDLE[5] = {0xA5,0x00,0x00,0xFF,0xFF};
const uint8_t JSDrive::CMD_UP[5] = {0xA5,0x00,0x20,0xDF,0xFF};
const uint8_t JSDrive::CMD_DOWN[5] = {0xA5,0x00,0x40,0xBF,0xFF};
const uint8_t JSDrive::CMD_M[5] = {0xA5,0x00,0x01,0xFE,0xFF};
const uint8_t JSDrive::CMD_PRESET[4][5] = {
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

void JSDrive::setup() {
  for (GPIOPin *pin : {remote_action_pin_, remote_awake_pin_, box_a_.action_pin, box_b_.action_pin,
                       box_a_.awake_pin, box_b_.awake_pin}) if (pin != nullptr) pin->setup();
  set_actions_(false,false);
  if (remote_awake_pin_ != nullptr) remote_awake_pin_->digital_write(false);
  ESP_LOGI(TAG, "Three-session WP-CB01 bridge ready; armed by default");
  publish_();
}

bool JSDrive::parse_(Parser &p, uint8_t byte, uint8_t out[5]) {
  if (p.length == 0 && byte != p.start) return false;
  p.data[p.length++] = byte;
  if (p.length < 5) return false;
  memcpy(out,p.data,5); p.length=0; return true;
}

void JSDrive::poll_uart_(uart::UARTComponent *uart, Parser &parser, bool remote, Box *box) {
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

void JSDrive::set_actions_(bool a, bool b) {
  if (box_a_.action_pin != nullptr) box_a_.action_pin->digital_write(a);
  if (box_b_.action_pin != nullptr) box_b_.action_pin->digital_write(b);
  box_a_.action=a; box_b_.action=b;
}

void JSDrive::send_box_(Box &box, const uint8_t frame[5]) {
  if (box.uart == nullptr) return;
  uint32_t now=millis();
  if (box.last_tx && now-box.last_tx>box.max_tx_gap) box.max_tx_gap=now-box.last_tx;
  box.last_tx=now; box.tx_count++; box.uart->write_array(frame,5);
}

void JSDrive::send_both_(const uint8_t frame[5], bool action) {
  set_actions_(action,action); send_box_(box_a_,frame); send_box_(box_b_,frame);
  last_command_tx_=millis();
}

bool JSDrive::box_ready_(const Box &box, uint32_t now) const {
  return box.seen && box.awake && now-box.last_frame<=LINK_MS;
}
bool JSDrive::trusted_equal_() const {
  return !std::isnan(box_a_.last_good) && !std::isnan(box_b_.last_good) && tenths(box_a_.last_good)==tenths(box_b_.last_good);
}
bool JSDrive::live_equal_() const {
  uint32_t now=millis();
  return box_ready_(box_a_,now) && box_ready_(box_b_,now) && !std::isnan(box_a_.live_height) &&
         !std::isnan(box_b_.live_height) && now-box_a_.last_height<=LIVE_MS && now-box_b_.last_height<=LIVE_MS &&
         tenths(box_a_.live_height)==tenths(box_b_.live_height);
}
bool JSDrive::movement_ready_(bool allow_cached) const {
  uint32_t now=millis();
  return armed_ && box_ready_(box_a_,now) && box_ready_(box_b_,now) &&
         (live_equal_() || (allow_cached && trusted_equal_()));
}

const uint8_t *JSDrive::command_frame_(uint8_t command) const {
  if (command==0x20) return CMD_UP;
  if (command==0x40) return CMD_DOWN;
  if (command==0x01) return CMD_M;
  for (uint8_t i=0;i<4;i++) if (command==(1U<<(i+1))) return CMD_PRESET[i];
  return CMD_IDLE;
}

void JSDrive::set_handset_buttons_(uint8_t command) {
  if (up_sensor_) up_sensor_->publish_state(command==0x20);
  if (down_sensor_) down_sensor_->publish_state(command==0x40);
  if (memory1_sensor_) memory1_sensor_->publish_state(command==0x02);
  if (memory2_sensor_) memory2_sensor_->publish_state(command==0x04);
  if (memory3_sensor_) memory3_sensor_->publish_state(command==0x08);
  if (memory4_sensor_) memory4_sensor_->publish_state(command==0x10);
}

void JSDrive::handle_remote_(const uint8_t f[5]) {
  uint32_t now=millis(); uint8_t command=f[2];
  remote_seen_=true; remote_last_frame_=now; set_handset_buttons_(command);
  bool physical=remote_action_pin_ != nullptr && remote_action_pin_->digital_read();
  if (command!=0 && command!=remote_command_ && physical && motion_!=MotionState::IDLE) stop("handset took control");
  remote_action_=physical;
  if (command==0) {
    if (remote_button_down_) { set_actions_(false,false); send_both_(CMD_IDLE,false); }
    remote_button_down_=false; remote_command_=0; return;
  }
  if (!physical) return;
  bool memory=command==0x01 || (command>=0x02 && command<=0x10 && (command&(command-1))==0);
  if (!movement_ready_(memory)) {
    set_actions_(false,false);
    if ((command==0x20 || command==0x40) && command!=remote_command_ && trusted_equal_() &&
        box_ready_(box_a_,now) && box_ready_(box_b_,now)) {
      // A deep-sleep handset press wakes but does not move the original desk.
      // Reproduce the captured handshake; the next press can move after both
      // boxes report matching live heights.
      discover_height();
      remote_command_=command;
      remote_button_down_=true;
    }
    if (command!=remote_command_) ESP_LOGW(TAG,"Handset command 0x%02X blocked: boxes are not ready and synchronized",command);
    return;
  }
  if ((command==0x02 || command==0x04 || command==0x08 || command==0x10) && command!=remote_command_) {
    motion_=MotionState::PRESET; motion_started_=now; phase_started_=now; preset_motion_seen_=false;
    active_preset_=(command==0x02?1:command==0x04?2:command==0x08?3:4);
  }
  send_both_(f,true); remote_button_down_=true; remote_command_=command;
}

std::string JSDrive::display_text_(const Box &b) const {
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

void JSDrive::handle_box_(Box &box, const uint8_t f[5]) {
  uint32_t now=millis(); float h=decode_height(f); DisplayState prior=box.display;
  box.seen=true; box.last_frame=now; box.glyph={{f[1],f[2],f[3]}};
  if (!std::isnan(h)) {
    box.display=DisplayState::HEIGHT; box.live_height=h; box.last_height=now;
    if (std::isnan(box.last_good) || tenths(box.last_good)!=tenths(h)) box.last_change=now;
    box.last_good=h;
  } else if (f[1]==0 && f[2]==0 && f[3]==0) {
    box.display=DisplayState::BLANK; box.live_height=NAN;
  } else if (f[1]==0x6D && (f[2]==0x40 || f[2]==0) && f[3]==0) {
    box.display=DisplayState::MEMORY; box.live_height=NAN;
  } else {
    // E04 is E,0,4. Keep unknown glyphs visible rather than discarding them.
    box.display=(digit(f[2])==0 && digit(f[3])==4) ? DisplayState::ERROR : DisplayState::OTHER;
    box.live_height=NAN;
    if (box.display==DisplayState::ERROR) error_="E04";
  }
  if (prior!=box.display || !std::isnan(h)) ESP_LOGD(TAG,"Box %s: %s",box.name,display_text_(box).c_str());
  update_confirmed_();
  if (motion_!=MotionState::IDLE && !std::isnan(box_a_.live_height) && !std::isnan(box_b_.live_height) &&
      fabsf(box_a_.live_height-box_b_.live_height)+0.001f>=hard_skew_) fail_("box height difference reached hard limit");
}

void JSDrive::update_confirmed_() {
  if (trusted_equal_()) {
    float h=(box_a_.last_good+box_b_.last_good)/2.0f;
    if (std::isnan(confirmed_height_) || tenths(h)!=tenths(confirmed_height_)) {
      confirmed_height_=h;
      if (height_sensor_) height_sensor_->publish_state(h);
      if (cover_) { cover_->position=(h-min_height_)/(max_height_-min_height_); cover_->publish_state(); }
    }
  }
}

void JSDrive::send_remote_status_() {
  if (remote_uart_==nullptr) return;
  uint8_t f[5]={0x5A,0,0,0,0}; uint32_t now=millis();
  if (sos_ && now<sos_until_) { f[1]=0x6D; f[2]=0x3F; f[3]=0x6D; }
  else if (box_a_.display==DisplayState::MEMORY && box_b_.display==DisplayState::MEMORY) {
    f[1]=0x6D; f[2]=((now/500)&1)?0:0x40;
  } else if (!std::isnan(confirmed_height_)) {
    int n=tenths(confirmed_height_), whole=n/10;
    f[1]=SEG[(whole/10)%10]; f[2]=SEG[whole%10]|0x80; f[3]=SEG[n%10];
  }
  f[4]=(uint8_t)(f[1]+f[2]+f[3]);
  if (remote_last_tx_ && now-remote_last_tx_>remote_max_tx_gap_) remote_max_tx_gap_=now-remote_last_tx_;
  remote_last_tx_=now; remote_tx_count_++; remote_uart_->write_array(f,5);
}

void JSDrive::fail_(const std::string &reason, bool disarm, bool show_sos) {
  set_actions_(false,false); send_box_(box_a_,CMD_IDLE); send_box_(box_b_,CMD_IDLE);
  motion_=MotionState::IDLE; target_height_=NAN; direction_=0;
  fault_=reason; if (disarm) armed_=false;
  if (show_sos) { sos_=true; sos_until_=millis()+3000; error_="SOS: "+reason; }
  ESP_LOGE(TAG,"Stopped: %s%s",reason.c_str(),disarm?"; bridge disarmed":""); publish_();
}

void JSDrive::finish_motion_(const char *reason) {
  set_actions_(false,false); send_box_(box_a_,CMD_IDLE); send_box_(box_b_,CMD_IDLE);
  motion_=MotionState::IDLE; target_height_=NAN; direction_=0; fine_taps_=0;
  ESP_LOGI(TAG,"%s",reason); update_confirmed_(); publish_();
}

bool JSDrive::discover_height() {
  if (motion_!=MotionState::IDLE || remote_button_down_) return false;
  motion_=MotionState::DISCOVER_ASSERT; motion_started_=phase_started_=millis(); set_actions_(true,true);
  ESP_LOGI(TAG,"Starting captured wake discovery handshake"); return true;
}

bool JSDrive::request_height(float height) {
  height=roundf(height*10.0f)/10.0f;
  if (height<min_height_ || height>max_height_) { fault_="target outside configured range"; publish_(); return false; }
  if (motion_!=MotionState::IDLE || remote_button_down_) return false;
  target_height_=height; fine_taps_=0; motion_started_=millis();
  if (!movement_ready_(true) || !live_equal_()) {
    motion_=MotionState::DISCOVER_ASSERT; phase_started_=millis(); set_actions_(true,true);
    ESP_LOGI(TAG,"Target %.1f requested; discovering height first",height); return true;
  }
  int current=tenths(confirmed_height_), target=tenths(height);
  if (current==target) { finish_motion_("already at requested height"); return true; }
  direction_=target>current?1:-1; motion_=MotionState::COARSE; phase_started_=millis();
  ESP_LOGI(TAG,"Target move %.1f -> %.1f",confirmed_height_,height); return true;
}

bool JSDrive::recall_preset(uint8_t preset) {
  if (preset<1 || preset>4 || motion_!=MotionState::IDLE || !movement_ready_(true)) return false;
  active_preset_=preset; preset_motion_seen_=false; motion_=MotionState::PRESET; motion_started_=phase_started_=millis();
  send_both_(CMD_PRESET[preset-1],true); ESP_LOGI(TAG,"Recall preset %u",preset); return true;
}

bool JSDrive::save_preset(uint8_t preset) {
  if (preset<1 || preset>4 || motion_!=MotionState::IDLE || !movement_ready_(true)) return false;
  save_preset_=preset; motion_=MotionState::SAVE_M; motion_started_=phase_started_=millis();
  send_both_(CMD_M,true); ESP_LOGI(TAG,"Save preset %u: requesting memory prompt",preset); return true;
}

void JSDrive::stop(const char *reason) { fail_(reason,false,false); }
void JSDrive::arm() { armed_=true; fault_="none"; publish_(); }
void JSDrive::disarm() { stop("disarmed"); armed_=false; publish_(); }

void JSDrive::start_command_(uint8_t command, bool from_remote) {
  (void) from_remote; const uint8_t *f=command_frame_(command); send_both_(f,command!=0);
}

void JSDrive::poll_motion_() {
  if (motion_==MotionState::IDLE) return;
  uint32_t now=millis();
  if (now-motion_started_>TARGET_TIMEOUT_MS) { fail_("45 second transaction timeout"); return; }
  if (motion_==MotionState::DISCOVER_ASSERT) {
    if (now-phase_started_>=10) { send_box_(box_a_,CMD_UP); send_box_(box_b_,CMD_UP); motion_=MotionState::DISCOVER_WAIT; phase_started_=now; }
    return;
  }
  if (motion_==MotionState::DISCOVER_WAIT) {
    if (now-phase_started_>=10 && (box_a_.action || box_b_.action)) { set_actions_(false,false); send_box_(box_a_,CMD_IDLE); send_box_(box_b_,CMD_IDLE); }
    if (movement_ready_(false)) {
      if (std::isnan(target_height_)) { finish_motion_("height discovery complete"); return; }
      int cur=tenths(confirmed_height_), target=tenths(target_height_);
      if (cur==target) { finish_motion_("already at requested height"); return; }
      direction_=target>cur?1:-1; motion_=MotionState::COARSE; phase_started_=now;
    } else if (now-phase_started_>3000) fail_("height discovery did not return matching box heights");
    return;
  }
  if (!box_ready_(box_a_,now) || !box_ready_(box_b_,now)) { fail_("box link or awake input lost"); return; }

  if (motion_==MotionState::SAVE_M) {
    if (now-phase_started_>=190) { set_actions_(false,false); send_both_(CMD_IDLE,false); motion_=MotionState::SAVE_SLOT; phase_started_=now; }
    return;
  }
  if (motion_==MotionState::SAVE_SLOT) {
    if (box_a_.display==DisplayState::MEMORY && box_b_.display==DisplayState::MEMORY) {
      send_both_(CMD_PRESET[save_preset_-1],true); motion_=MotionState::PRESET; active_preset_=save_preset_; phase_started_=now;
      ESP_LOGI(TAG,"Both boxes accepted memory prompt; storing preset %u",save_preset_); return;
    }
    if (now-phase_started_>3000) fail_("both boxes did not enter memory prompt");
    return;
  }
  if (motion_==MotionState::PRESET) {
    if (box_a_.action || box_b_.action) {
      if (now-phase_started_>=190 && !remote_button_down_) { set_actions_(false,false); send_both_(CMD_IDLE,false); phase_started_=now; }
    }
    if (!std::isnan(box_a_.live_height) && !std::isnan(box_b_.live_height)) {
      int delta=abs(tenths(box_a_.live_height)-tenths(box_b_.live_height));
      if (delta>=tenths(hard_skew_)) { fail_("preset height pair exceeded hard skew"); return; }
      if (delta==0) { preset_motion_seen_=true; update_confirmed_(); phase_started_=now; }
    }
    if (preset_motion_seen_ && !std::isnan(box_a_.live_height) && !std::isnan(box_b_.live_height) &&
        now-box_a_.last_change>700 && now-box_b_.last_change>700) finish_motion_("preset settled");
    else if (!preset_motion_seen_ && now-motion_started_>5000) finish_motion_("preset completed without travel");
    return;
  }

  if (std::isnan(box_a_.live_height) || std::isnan(box_b_.live_height) || now-box_a_.last_height>LIVE_MS || now-box_b_.last_height>LIVE_MS) {
    // During settling a blank display is expected; retain last-good positions.
    if (motion_!=MotionState::FINE_SETTLE || now-phase_started_>2000) { fail_("fresh height missing"); return; }
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

void JSDrive::poll_alignment_() {
  if (motion_!=MotionState::IDLE || remote_button_down_ || !armed_) return;
  uint32_t now=millis();
  if (!box_ready_(box_a_,now) || !box_ready_(box_b_,now) || std::isnan(box_a_.last_good) || std::isnan(box_b_.last_good)) return;
  int d=tenths(box_a_.last_good)-tenths(box_b_.last_good);
  if (d==0) { align_attempts_=0; return; }
  if (abs(d)>=tenths(hard_skew_)) { fail_("stopped boxes exceed hard skew"); return; }
  if (now-box_a_.last_change<800 || now-box_b_.last_change<800) return;
  if (align_attempts_>=3) { fail_("three alignment pulses did not synchronize boxes"); return; }
  Box &higher=d>0?box_a_:box_b_; uint32_t duration=align_attempts_==0?20:align_attempts_==1?35:50;
  higher.action_pin->digital_write(true); higher.action=true; send_box_(higher,CMD_DOWN);
  motion_=MotionState::ALIGN_PULSE; motion_started_=phase_started_=now; target_height_=duration; align_attempts_++;
  ESP_LOGW(TAG,"Aligning box %s down for %u ms",higher.name,(unsigned)duration);
}

void JSDrive::publish_() {
  uint32_t now=millis(); bool ready_a=box_ready_(box_a_,now), ready_b=box_ready_(box_b_,now), known=trusted_equal_();
  bool live=live_equal_(), sync=known && fabsf(box_a_.last_good-box_b_.last_good)<0.15f;
  bool moving=motion_!=MotionState::IDLE;
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
  const char *op=motion_==MotionState::IDLE?"idle":motion_==MotionState::COARSE?"target coarse":motion_==MotionState::FINE_PULSE?"target fine pulse":motion_==MotionState::FINE_SETTLE?"target settling":motion_==MotionState::PRESET?"preset":motion_==MotionState::SAVE_M||motion_==MotionState::SAVE_SLOT?"saving preset":motion_==MotionState::DISCOVER_ASSERT||motion_==MotionState::DISCOVER_WAIT?"discovering height":"aligning";
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

void JSDrive::loop() {
  uint32_t now=millis();
  box_a_.awake=box_a_.awake_pin!=nullptr && box_a_.awake_pin->digital_read();
  box_b_.awake=box_b_.awake_pin!=nullptr && box_b_.awake_pin->digital_read();
  if (remote_awake_pin_) remote_awake_pin_->digital_write(box_a_.awake&&box_b_.awake);
  poll_uart_(remote_uart_,remote_parser_,true); poll_uart_(box_a_.uart,box_a_.parser,false,&box_a_); poll_uart_(box_b_.uart,box_b_.parser,false,&box_b_);
  if (box_a_.seen && now-box_a_.last_frame>LINK_MS) box_a_.seen=false;
  if (box_b_.seen && now-box_b_.last_frame>LINK_MS) box_b_.seen=false;
  if (remote_seen_ && now-remote_last_frame_>LINK_MS) { remote_seen_=false; remote_button_down_=false; set_handset_buttons_(0); }
  if (motion_==MotionState::ALIGN_PULSE) {
    if (now-phase_started_>=(uint32_t)target_height_) { set_actions_(false,false); send_box_(box_a_,CMD_IDLE); send_box_(box_b_,CMD_IDLE); motion_=MotionState::ALIGN_SETTLE; phase_started_=now; }
  } else if (motion_==MotionState::ALIGN_SETTLE) {
    if (now-phase_started_>=800) { motion_=MotionState::IDLE; target_height_=NAN; }
  } else poll_motion_();
  poll_alignment_();
  if (motion_==MotionState::IDLE && now-box_a_.last_tx>=STREAM_MS) send_box_(box_a_,CMD_IDLE);
  if (motion_==MotionState::IDLE && now-box_b_.last_tx>=STREAM_MS) send_box_(box_b_,CMD_IDLE);
  if (now-remote_last_tx_>=STREAM_MS) send_remote_status_();
  if (now-last_publish_>=1000) { last_publish_=now; publish_(); }
}

void JSDrive::dump_config() {
  ESP_LOGCONFIG(TAG,"WP-CB01 dual-box bridge");
  ESP_LOGCONFIG(TAG,"  Range %.1f..%.1f; hard skew %.1f",min_height_,max_height_,hard_skew_);
  LOG_PIN("  Handset action: ",remote_action_pin_); LOG_PIN("  Handset awake: ",remote_awake_pin_);
  LOG_PIN("  Box A action: ",box_a_.action_pin); LOG_PIN("  Box A awake: ",box_a_.awake_pin);
  LOG_PIN("  Box B action: ",box_b_.action_pin); LOG_PIN("  Box B awake: ",box_b_.awake_pin);
}

void JSDrive::run_button(uint8_t mode) {
  if (mode==0) stop(); else if (mode==1) discover_height(); else if (mode==2) arm(); else if (mode==3) disarm();
  else if (mode>=11&&mode<=14) recall_preset(mode-10); else if (mode>=21&&mode<=24) save_preset(mode-20);
}
void JSDriveTargetNumber::control(float value) { if (parent_&&parent_->request_height(value)) publish_state(value); }
cover::CoverTraits JSDriveCover::get_traits() { cover::CoverTraits t; t.set_supports_position(true); t.set_supports_stop(true); return t; }
void JSDriveCover::control(const cover::CoverCall &call) {
  if (!parent_) return;
  if (call.get_stop()) parent_->stop("cover stop");
  if (call.get_position().has_value()) parent_->request_height(parent_->min_height()+*call.get_position()*(parent_->max_height()-parent_->min_height()));
}
void JSDriveCommandButton::press_action() { if (parent_) parent_->run_button(mode_); }

}  // namespace jsdrive
}  // namespace esphome
