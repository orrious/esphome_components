#include <Arduino.h>
#include <cstring>

// ESP32 GPIO assignments from the breadboard wiring.
static constexpr int PIN_REMOTE_RX = 12;
static constexpr int PIN_REMOTE_TX = 13;
static constexpr int PIN_BOX_A_RX = 4;
static constexpr int PIN_BOX_A_TX = 5;
static constexpr int PIN_BOX_B_RX = 8;
static constexpr int PIN_BOX_B_TX = 9;

static constexpr int PIN_REMOTE_ACTION = 14;
static constexpr int PIN_BOX_A_ACTION = 6;
static constexpr int PIN_BOX_B_ACTION = 10;
static constexpr int PIN_BOX_A_AWAKE = 7;
static constexpr int PIN_BOX_B_AWAKE = 11;
static constexpr int PIN_REMOTE_AWAKE = 15;

static constexpr uint32_t UART_BAUD = 9600;
static constexpr uint32_t BOX_FALLBACK_MS = 100;
static constexpr uint32_t DEFAULT_ACTION_MS = 350;

HardwareSerial remote_uart(2);  // UART2
HardwareSerial box_a_uart(1);   // UART1
HardwareSerial box_b_uart(0);   // UART0

struct FrameParser {
  uint8_t start;
  uint8_t bytes[5]{};
  uint8_t length = 0;

  explicit FrameParser(uint8_t start_byte) : start(start_byte) {}
};

FrameParser remote_parser{0xA5};
FrameParser box_a_parser{0x5A};
FrameParser box_b_parser{0x5A};

bool pass_through = true;
bool raw_logging = true;
uint32_t action_until = 0;
uint32_t default_action_ms = DEFAULT_ACTION_MS;
uint32_t last_box_a_frame = 0;
uint32_t last_box_b_frame = 0;
float box_a_height = NAN;
float box_b_height = NAN;
uint8_t console_line[128]{};
size_t console_length = 0;

const uint8_t CMD_IDLE[5] = {0xA5, 0x00, 0x00, 0xFF, 0xFF};
const uint8_t CMD_UP[5] = {0xA5, 0x00, 0x20, 0xDF, 0xFF};
const uint8_t CMD_DOWN[5] = {0xA5, 0x00, 0x40, 0xBF, 0xFF};
const uint8_t CMD_M[5] = {0xA5, 0x00, 0x01, 0xFE, 0xFF};
const uint8_t CMD_MEMORY[4][5] = {
    {0xA5, 0x00, 0x02, 0xFD, 0xFF},
    {0xA5, 0x00, 0x04, 0xFB, 0xFF},
    {0xA5, 0x00, 0x08, 0xF7, 0xFF},
    {0xA5, 0x00, 0x10, 0xEF, 0xFF},
};

bool elapsed(uint32_t now, uint32_t deadline) {
  return static_cast<int32_t>(now - deadline) >= 0;
}

bool frame_complete(FrameParser &parser, uint8_t byte, uint8_t *frame) {
  if (parser.length == 0) {
    if (byte != parser.start) return false;
  }

  parser.bytes[parser.length++] = byte;
  if (parser.length != 5) return false;

  memcpy(frame, parser.bytes, 5);
  parser.length = 0;
  return true;
}

bool valid_command(const uint8_t *frame) {
  return frame[0] == 0xA5 && frame[4] == 0xFF &&
         static_cast<uint8_t>(frame[1] + frame[2] + frame[3]) == 0xFF;
}

bool valid_display(const uint8_t *frame) {
  return frame[0] == 0x5A &&
         static_cast<uint8_t>(frame[0] + frame[1] + frame[2] + frame[3]) ==
             frame[4];
}

void print_frame(const char *label, const uint8_t *frame) {
  Serial.printf("%s %02X %02X %02X %02X %02X\n", label, frame[0], frame[1],
                frame[2], frame[3], frame[4]);
}

int decode_digit(uint8_t segment) {
  static const uint8_t masks[10] = {0x3F, 0x06, 0x5B, 0x4F, 0x66,
                                    0x6D, 0x7D, 0x07, 0x7F, 0x6F};
  const uint8_t glyph = segment & 0x7F;
  for (int digit = 0; digit < 10; digit++) {
    if (masks[digit] == glyph) return digit;
  }
  return -1;
}

float decode_height(const uint8_t *frame) {
  if (frame[1] == 0 && frame[2] == 0 && frame[3] == 0) return NAN;
  const int d0 = decode_digit(frame[1]);
  const int d1 = decode_digit(frame[2]);
  const int d2 = decode_digit(frame[3]);
  if (d0 < 0 || d1 < 0 || d2 < 0) return NAN;
  // The WP-CB01 displays heights as two digits, decimal point, one digit.
  return static_cast<float>(d0 * 10 + d1) + static_cast<float>(d2) / 10.0f;
}

void begin_action(uint32_t duration_ms) {
  action_until = millis() + duration_ms;
  digitalWrite(PIN_BOX_A_ACTION, HIGH);
  digitalWrite(PIN_BOX_B_ACTION, HIGH);
}

void update_action_lines() {
  const bool pulse_active = !elapsed(millis(), action_until);
  const bool remote_active = digitalRead(PIN_REMOTE_ACTION) == HIGH;
  const bool active = pulse_active || remote_active;
  digitalWrite(PIN_BOX_A_ACTION, active ? HIGH : LOW);
  digitalWrite(PIN_BOX_B_ACTION, active ? HIGH : LOW);
}

void update_remote_awake() {
  // Pin 8 is open-drain on the desk side. Pull the remote side low if either
  // control box reports sleep; otherwise release the ESP32 output to high-Z.
  const bool box_a_awake = digitalRead(PIN_BOX_A_AWAKE) == HIGH;
  const bool box_b_awake = digitalRead(PIN_BOX_B_AWAKE) == HIGH;
  if (box_a_awake && box_b_awake) {
    pinMode(PIN_REMOTE_AWAKE, INPUT);
  } else {
    pinMode(PIN_REMOTE_AWAKE, OUTPUT);
    digitalWrite(PIN_REMOTE_AWAKE, LOW);
  }
}

void send_to_boxes(const uint8_t *frame, uint32_t action_ms) {
  box_a_uart.write(frame, 5);
  box_b_uart.write(frame, 5);
  if (action_ms != 0) begin_action(action_ms);
  print_frame("TX BOTH", frame);
}

void handle_remote_frame(const uint8_t *frame) {
  if (!valid_command(frame)) {
    print_frame("BAD REMOTE", frame);
    return;
  }
  if (raw_logging) print_frame("RX REMOTE", frame);
  if (pass_through) send_to_boxes(frame, 0);
}

void handle_box_frame(const char *label, const uint8_t *frame, bool box_a) {
  if (!valid_display(frame)) {
    print_frame("BAD BOX", frame);
    return;
  }

  const uint32_t now = millis();
  if (box_a) {
    last_box_a_frame = now;
    box_a_height = decode_height(frame);
  } else {
    last_box_b_frame = now;
    box_b_height = decode_height(frame);
  }

  if (raw_logging) print_frame(label, frame);
  const float height = decode_height(frame);
  if (!isnan(height)) {
    Serial.printf("  decoded %.1f in; A=%.1f B=%.1f sync=%s\n", height,
                  box_a_height, box_b_height,
                  (!isnan(box_a_height) && !isnan(box_b_height) &&
                   fabsf(box_a_height - box_b_height) <= 0.1f)
                      ? "yes"
                      : "no");
  }

  // Only one response may be placed on the remote's RX line. Prefer box A;
  // use box B only when box A has gone quiet.
  if (box_a || elapsed(now, last_box_a_frame + BOX_FALLBACK_MS)) {
    remote_uart.write(frame, 5);
  }
}

void poll_uart(HardwareSerial &uart, FrameParser &parser,
               void (*handler)(const uint8_t *)) {
  uint8_t frame[5];
  while (uart.available()) {
    const uint8_t byte = static_cast<uint8_t>(uart.read());
    if (frame_complete(parser, byte, frame)) handler(frame);
  }
}

void handle_box_a(const uint8_t *frame) { handle_box_frame("RX BOX A", frame, true); }
void handle_box_b(const uint8_t *frame) { handle_box_frame("RX BOX B", frame, false); }

void poll_remote() { poll_uart(remote_uart, remote_parser, handle_remote_frame); }
void poll_box_a() { poll_uart(box_a_uart, box_a_parser, handle_box_a); }
void poll_box_b() { poll_uart(box_b_uart, box_b_parser, handle_box_b); }

void print_status() {
  Serial.printf("pass=%s raw=%s action=%s remote_action=%d awakeA=%d awakeB=%d\n",
                pass_through ? "on" : "off", raw_logging ? "on" : "off",
                !elapsed(millis(), action_until) ? "on" : "off",
                digitalRead(PIN_REMOTE_ACTION), digitalRead(PIN_BOX_A_AWAKE),
                digitalRead(PIN_BOX_B_AWAKE));
  Serial.printf("heightA=%.1f heightB=%.1f lastA=%lu ms lastB=%lu ms\n",
                box_a_height, box_b_height,
                static_cast<unsigned long>(millis() - last_box_a_frame),
                static_cast<unsigned long>(millis() - last_box_b_frame));
}

void send_named_command(const uint8_t *frame) {
  send_to_boxes(frame, default_action_ms);
}

void process_command(char *line) {
  while (*line == ' ' || *line == '\t') line++;
  if (*line == '\0') return;

  if (strcmp(line, "help") == 0) {
    Serial.println("Commands: status, up, down, stop, m, preset 1..4");
    Serial.println("          raw a5 00 20 df ff, pass on|off, raw on|off");
    Serial.println("          pulse <milliseconds>");
    return;
  }
  if (strcmp(line, "status") == 0) {
    print_status();
    return;
  }
  if (strcmp(line, "up") == 0) {
    send_named_command(CMD_UP);
    return;
  }
  if (strcmp(line, "down") == 0) {
    send_named_command(CMD_DOWN);
    return;
  }
  if (strcmp(line, "stop") == 0) {
    send_named_command(CMD_IDLE);
    return;
  }
  if (strcmp(line, "m") == 0) {
    send_named_command(CMD_M);
    return;
  }

  int preset = 0;
  if (sscanf(line, "preset %d", &preset) == 1 && preset >= 1 && preset <= 4) {
    send_named_command(CMD_MEMORY[preset - 1]);
    return;
  }

  int pulse_ms = 0;
  if (sscanf(line, "pulse %d", &pulse_ms) == 1 && pulse_ms >= 0 && pulse_ms <= 5000) {
    default_action_ms = static_cast<uint32_t>(pulse_ms);
    Serial.printf("default action pulse = %lu ms\n",
                  static_cast<unsigned long>(default_action_ms));
    return;
  }

  if (strcmp(line, "pass on") == 0) {
    pass_through = true;
    Serial.println("remote pass-through enabled");
    return;
  }
  if (strcmp(line, "pass off") == 0) {
    pass_through = false;
    Serial.println("remote pass-through disabled");
    return;
  }
  if (strcmp(line, "raw on") == 0) {
    raw_logging = true;
    Serial.println("raw frame logging enabled");
    return;
  }
  if (strcmp(line, "raw off") == 0) {
    raw_logging = false;
    Serial.println("raw frame logging disabled");
    return;
  }

  unsigned int values[5];
  if (sscanf(line, "raw %x %x %x %x %x", &values[0], &values[1], &values[2],
             &values[3], &values[4]) == 5) {
    uint8_t frame[5];
    for (int i = 0; i < 5; i++) frame[i] = static_cast<uint8_t>(values[i]);
    if (!valid_command(frame)) {
      Serial.println("raw command rejected: invalid A5 checksum/frame");
      return;
    }
    send_named_command(frame);
    return;
  }

  Serial.println("unknown command; type help");
}

void poll_console() {
  while (Serial.available()) {
    const char c = static_cast<char>(Serial.read());
    if (c == '\r' || c == '\n') {
      console_line[console_length] = '\0';
      process_command(reinterpret_cast<char *>(console_line));
      console_length = 0;
    } else if (console_length + 1 < sizeof(console_line)) {
      console_line[console_length++] = static_cast<uint8_t>(c);
    }
  }
}

void setup() {
  pinMode(PIN_BOX_A_ACTION, OUTPUT);
  pinMode(PIN_BOX_B_ACTION, OUTPUT);
  digitalWrite(PIN_BOX_A_ACTION, LOW);
  digitalWrite(PIN_BOX_B_ACTION, LOW);

  // Keep the action outputs low when the handset is unplugged or the input
  // level shifter is floating.
  pinMode(PIN_REMOTE_ACTION, INPUT_PULLDOWN);
  pinMode(PIN_BOX_A_AWAKE, INPUT);
  pinMode(PIN_BOX_B_AWAKE, INPUT);
  pinMode(PIN_REMOTE_AWAKE, INPUT);

  Serial.begin(115200);
  delay(250);
  Serial.println();
  Serial.println("WP-CB01 dual-box diagnostic bridge");

  remote_uart.begin(UART_BAUD, SERIAL_8N1, PIN_REMOTE_RX, PIN_REMOTE_TX);
  box_a_uart.begin(UART_BAUD, SERIAL_8N1, PIN_BOX_A_RX, PIN_BOX_A_TX);
  box_b_uart.begin(UART_BAUD, SERIAL_8N1, PIN_BOX_B_RX, PIN_BOX_B_TX);

  Serial.println("UARTs ready: remote=UART2, boxA=UART1, boxB=UART0");
  Serial.println("Action outputs are low; remote pass-through is ON");
  Serial.println("Type help for commands");
}

void loop() {
  poll_console();
  poll_remote();
  poll_box_a();
  poll_box_b();
  update_action_lines();
  update_remote_awake();
  delay(1);
}
