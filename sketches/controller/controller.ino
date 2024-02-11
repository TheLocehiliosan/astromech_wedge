/*
  Components:
    * Dome Motor: SyRen motor controller
    * Leg Motors: Sabertooth 2x25  / Dip switches DOWN: [1, 2] | UP: [3, 4, 5, 6]
    * Sound System: MP3 Trigger
    * Remote: PS2 Controller
    * Periscope: Two stepper motors driven by two ULN2003

  Hardware Notes:
    * When using a wired-controller, it appears to be necessary to use a
      pull-up resistor of 4.7k between the PS2 data line (Line 1/PIN 12) and
      the 5VCC of the Arduino.
*/

#include "Sound.h"
#include <PS2X_lib.h>
#include <Sabertooth.h>
#include <SoftwareSerial.h>
#include <AccelStepper.h>

// PINS (avoid 0 & 1 to preserve serial communication)
// Remaining free pins: 13, A0
#define PIN_NON_READ     A1
#define PIN_SCOPE_TURN_4 A2
#define PIN_SCOPE_TURN_3 A3
#define PIN_SCOPE_TURN_2 A4
#define PIN_SCOPE_TURN_1 A5
#define PIN_SCOPE_LIFT_1  2
#define PIN_SCOPE_LIFT_2  3
#define PIN_SCOPE_LIFT_3  4
#define PIN_SCOPE_LIFT_4  5
#define PIN_DOME_MOTOR    6
#define PIN_DRIVE_MOTOR   7
#define PIN_REMOTE_DAT    8
#define PIN_REMOTE_CMD    9
#define PIN_REMOTE_ATT   10
#define PIN_REMOTE_CLK   11
#define PIN_SOUND        12

// Calibration
#define DOME_DEADBAND  10
#define DRIVE_DEADBAND 10

#define DOME_SPEED      127
#define DRIVE_SPEED     127
#define DRIVE_RAMP      5
#define DRIVE_TURN_MAX  75
#define DRIVE_TURN_PCT  0.2  // This percentage of the turning controls...
#define DRIVE_TURN_RAMP 0.3  // ...will get this much of the turning power

#define SCOPE_LIFT_STEPS_PER_REV 2038
#define SCOPE_TURN_STEPS_PER_REV 2038

// Constants
#define SERIAL_BAUD 9600

#define DEFAULT_VOLUME 80
#define VOLUME_CHANGE  1

#define REMOTE_CONNECTION_ATTEMPTS 30

#define SABERTOOTH_BAUD      9600 // This is the factory set baud rate
#define SABERTOOTH_BYTE_ADDR 128  // This is the address when DIP switches are DDUUUU
#define SABERTOOTH_TIMEOUT   950  // Motors will shut off if communications stops for this many milliseconds

#define MASK_GREEN 0b00000001
#define MASK_PINK  0b00000010
#define MASK_RED   0b00000100
#define MASK_BLUE  0b00001000
#define MASK_L1    0b00010000
#define MASK_L2    0b00100000
#define MASK_R1    0b01000000
#define MASK_R2    0b10000000

struct MaskValues {
  byte mask;
  byte sound_id_start;
  byte sound_id_end;
};
const MaskValues sound_table[] = {
  // TODO: Fill out sound table
  {MASK_GREEN,           1, 0},
  {MASK_GREEN | MASK_L1, 2, 0},
  {MASK_RED,             3, 0},
  {MASK_BLUE | MASK_L1,  4, 9},
};

struct Gear {
  byte num;
  byte percent;
  byte sound;
};
#define DEFAULT_GEAR 2
const Gear gears[] = {
  // TODO: Select notification sounds
  {1,  50, 31},
  {2,  80, 32},
  {3, 100, 33},
};

// ALL USER-SERVICABLE PARTS ARE ABOVE HERE

// dome motor object
SoftwareSerial dome_serial(PIN_NON_READ, PIN_DOME_MOTOR);
Sabertooth dome_motor(SABERTOOTH_BYTE_ADDR, dome_serial);
int8_t dome_speed = 0;

// drive motors object
SoftwareSerial drive_serial(PIN_NON_READ, PIN_DRIVE_MOTOR);
Sabertooth drive_motor(SABERTOOTH_BYTE_ADDR, drive_serial);
const int gear_count = sizeof(gears) / sizeof(gears[0]);
bool drive_enabled = false;
byte gear = 0;
int8_t drive_max = 0;
int8_t drive_speed = 0;
int8_t drive_turn = 0;
uint8_t turn_pct_lower = DRIVE_TURN_PCT * 255;
uint8_t turn_pct_upper = 255 - turn_pct_lower;
uint8_t turn_ramp = DRIVE_TURN_RAMP * DRIVE_TURN_MAX;

// periscope motors
AccelStepper scope_lift(
  AccelStepper::FULL4WIRE,
  PIN_SCOPE_LIFT_1,
  PIN_SCOPE_LIFT_3,
  PIN_SCOPE_LIFT_2,
  PIN_SCOPE_LIFT_4
);
AccelStepper scope_turn(
  AccelStepper::FULL4WIRE,
  PIN_SCOPE_TURN_1,
  PIN_SCOPE_TURN_3,
  PIN_SCOPE_TURN_2,
  PIN_SCOPE_TURN_4
);

// sound object
Sound sound(PIN_NON_READ, PIN_SOUND);
const int sound_table_count = sizeof(sound_table) / sizeof(sound_table[0]);
byte volume = DEFAULT_VOLUME;

// remote object
PS2X remote;

void setup() {
  // configure debug serial messages
  Serial.begin(SERIAL_BAUD);
  Serial.println("Astromech booting up");

  init_motors();
  init_sound();
  init_remote();

  Serial.println("Astromech online");
  report_stick_drift();
}

void init_motors() {
  dome_serial.begin(SABERTOOTH_BAUD);
  dome_motor.autobaud();
  dome_motor.setTimeout(SABERTOOTH_TIMEOUT);
  dome_motor.setDeadband(DOME_DEADBAND);
  dome_motor.motor(1, 0);
  Serial.println("Dome motor initialized");
  drive_serial.begin(SABERTOOTH_BAUD);
  drive_motor.setTimeout(SABERTOOTH_TIMEOUT);
  drive_motor.setDeadband(DRIVE_DEADBAND);
  drive_motor.drive(0); drive_motor.turn(0);
  Serial.println("Drive motors initialized");
  shift_gear(DEFAULT_GEAR);
}

void init_sound() {
  sound.volume(volume);
  Serial.println("Sound system initialized");
}

void init_remote() {
  Serial.print("Connecting to remote ");
  bool remote_connected = false;
  for (byte i = 0; i < REMOTE_CONNECTION_ATTEMPTS; i++) {
    Serial.print(".");
    byte result = remote.config_gamepad(
      PIN_REMOTE_CLK,
      PIN_REMOTE_CMD,
      PIN_REMOTE_ATT,
      PIN_REMOTE_DAT,
      false, // disable pressures
      false  // disable rumble
    );
    if (result != 1) {
      remote_connected = true;
      break;
    }
  }
  Serial.println();
  remote_connected || abort("Remote failed to connect");
  Serial.println("Remote connected");
}

void loop() {
  remote.read_gamepad();

  handle_remote_disconnection();

  handle_drive();

  handle_dome();

  handle_periscope();

  handle_volume();

  handle_speech();

  // show_button_state();

  /* delay(50); */
  scope_lift.run();
  scope_turn.run();
}

bool abort(char msg[]) {
  Serial.print("Abort: ");
  Serial.println(msg);
  Serial.println("Shutting down all motors");
  drive_motor.drive(0); drive_motor.turn(0);
  dome_motor.motor(1, 0);
  while(true);
}

bool modifiers() {
  return
    remote.Button(PSB_L1) ||
    remote.Button(PSB_L2) ||
    remote.Button(PSB_R1) ||
    remote.Button(PSB_R2)
  ;
}

// This detects if the controller is PHYSICALLY disconnected from the Arduino.
// (either a hardware failure, or loose connection). This is NOT related to a
// wireless remote being connected or not.
void handle_remote_disconnection() {
  if (remote.Analog(PSS_RX) == 255 &&
      remote.Analog(PSS_RY) == 255 &&
      remote.Analog(PSS_LX) == 255 &&
      remote.Analog(PSS_LY) == 255
  ) abort("The remote is physically disconnected");
}

void handle_drive() {
  toggle_drive();
  select_gear();
  drive();
}

void toggle_drive() {
  if(remote.ButtonPressed(PSB_START)) {
    drive_enabled = !drive_enabled;
    Serial.print("Toggled drive. Enabled:");
    Serial.println(drive_enabled);
    if (drive_enabled) {
      sound.play(999); // TODO: Choose sound
    }
    else {
      sound.play(999); // TODO: Choose sound
    }
  }
}

void select_gear() {
  if (!remote.Button(PSB_R1)) return;
  byte new_gear;
  if (remote.ButtonPressed(PSB_PAD_UP)) {
    new_gear = min(gear + 1, gear_count);
  }
  if (remote.ButtonPressed(PSB_PAD_DOWN)) {
    new_gear = max(gear - 1, 1);
  }
  shift_gear(new_gear);
}

void shift_gear(byte new_gear) {
  if (new_gear == gear) return;
  bool notify = (gear != 0);
  for (int i = 0; i < gear_count; i++) {
    Gear g = gears[i];
    if (g.num == new_gear) {
      gear = new_gear;
      drive_max = int(DRIVE_SPEED * g.percent / 100);
      Serial.print("Shifting gears - Gear:");
      Serial.print(gear);
      Serial.print(" Max speed:");
      Serial.println(drive_max);
      if (notify) sound.play(g.sound);
      return;
    }
  }
}

void drive() {
  int8_t old_speed = drive_speed;
  int8_t old_turn = drive_turn;
  if (drive_enabled) {
    int8_t target_speed = map(remote.Analog(PSS_RY), 0, 255, -drive_max, drive_max);
    if (target_speed < drive_speed) {
      drive_speed = max(drive_speed - DRIVE_RAMP, target_speed);
    }
    if (target_speed > drive_speed) {
      drive_speed = min(drive_speed + DRIVE_RAMP, target_speed);
    }
  }
  else {
    drive_speed = 0;
  }

  // The bottom and top DRIVE_TURN_PCT% of the stick values will map to
  // DRIVE_TURN_RAMP of the full range of turning power. The remaining turning
  // power will map the to the stick values in the middle. This will result in
  // a different slope of the turning power at the extremes of the stick
  // values.
  if (drive_enabled) {
    uint8_t stick = remote.Analog(PSS_RX);
    if (stick <= turn_pct_lower) {
      drive_turn = map(stick, 0, turn_pct_lower, -DRIVE_TURN_MAX, -turn_ramp);
    }
    else if (turn_pct_lower < stick && stick < turn_pct_upper) {
      drive_turn = map(stick, turn_pct_lower, turn_pct_upper, -turn_ramp, turn_ramp);
    }
    else if (stick >= turn_pct_upper) {
      drive_turn = map(stick, turn_pct_upper, 255, turn_ramp, DRIVE_TURN_MAX);
    }
  }
  else {
    drive_turn = 0;
  }

  if (drive_speed != old_speed) {
    Serial.print("Drive speed updated:");
    Serial.println(drive_speed);
  }
  if (drive_turn != old_turn) {
    Serial.print("Drive turn updated:");
    Serial.println(drive_turn);
  }
  drive_motor.drive(drive_speed);
  drive_motor.turn(drive_turn);
}

void handle_dome() {
  int8_t old_speed = dome_speed;
  dome_speed = map(remote.Analog(PSS_LX), 0, 255, -DOME_SPEED, DOME_SPEED);
  // SyRen doesn't seem to honor the deadband() method
  // Manually handling the deadband
  if (-DOME_DEADBAND <= dome_speed && dome_speed <= DOME_DEADBAND) {
    dome_speed = 0;
  }
  if (dome_speed != old_speed) {
    Serial.print("Dome speed updated:");
    Serial.println(dome_speed);
  }
  dome_motor.motor(1, dome_speed);
}

void handle_periscope() {
  // TODO: Control the periscope
}

void handle_volume() {
  if (modifiers()) return;
  byte old_volume = volume;
  if(remote.Button(PSB_PAD_UP)) {
    volume = min(volume + VOLUME_CHANGE, 100);
  }
  if(remote.Button(PSB_PAD_DOWN)) {
    volume = max(volume - VOLUME_CHANGE, 0);
  }
  if (old_volume != volume) {
    Serial.print("Volume updated:");
    Serial.println(volume);
    sound.volume(volume);
  }
}

void handle_speech() {
  byte button_mask = 0;
  if (remote.ButtonPressed(PSB_GREEN)) {
    button_mask |= MASK_GREEN;
  }
  if (remote.ButtonPressed(PSB_PINK)) {
    button_mask |= MASK_PINK;
  }
  if (remote.ButtonPressed(PSB_RED)) {
    button_mask |= MASK_RED;
  }
  if (remote.ButtonPressed(PSB_BLUE)) {
    button_mask |= MASK_BLUE;
  }
  if (button_mask == 0) return;
  if (remote.Button(PSB_L1)) {
    button_mask |= MASK_L1;
  }
  if (remote.Button(PSB_L2)) {
    button_mask |= MASK_L2;
  }
  if (remote.Button(PSB_R1)) {
    button_mask |= MASK_R1;
  }
  if (remote.Button(PSB_R2)) {
    button_mask |= MASK_R2;
  }
  for (int i = 0; i < sound_table_count; i++) {
    if (sound_table[i].mask == button_mask) {
      MaskValues s = sound_table[i];
      byte selected_sound = s.sound_id_start;
      if (s.sound_id_end > 0) {
        char buffer[60];
        sprintf(buffer, "Selecting a random sound from %d thru %d", s.sound_id_start, s.sound_id_end);
        Serial.println(buffer);
        selected_sound = random(s.sound_id_start, s.sound_id_end+1);
      }
      Serial.print("Playing sound:");
      Serial.println(selected_sound);
      sound.play(selected_sound);
      return;
    }
  }
}

void report_stick_drift() {
  uint8_t center = 128;
  int8_t lx_drift = remote.Analog(PSS_LX) - center;
  int8_t ly_drift = remote.Analog(PSS_LY) - center;
  int8_t rx_drift = remote.Analog(PSS_RX) - center;
  int8_t ry_drift = remote.Analog(PSS_RY) - center;
  if ((lx_drift | ly_drift | rx_drift | ry_drift) == 0) return;
  char buffer[80];
  sprintf(
    buffer,
    "Controller stick drift detected: Left [X:%d Y:%d] Right [X:%d Y:%d]",
    lx_drift, ly_drift, rx_drift, ry_drift
  );
  Serial.println(buffer);
}

void show_button_state() {
  char buffer[80];
  sprintf(
    buffer,
    "LX:%3d LY:%3d RX:%3d RY:%3d L:%d/%d/%d R:%d/%d/%d GPRB:%d%d%d%d UDLR:%d%d%d%d SlSt:%d%d",
    remote.Analog(PSS_LX),     remote.Analog(PSS_LY),
    remote.Analog(PSS_RX),     remote.Analog(PSS_RY),
    remote.Button(PSB_L1),     remote.Button(PSB_L2),       remote.Button(PSB_L3),
    remote.Button(PSB_R1),     remote.Button(PSB_R2),       remote.Button(PSB_R3),
    remote.Button(PSB_GREEN),  remote.Button(PSB_PINK),     remote.Button(PSB_RED),      remote.Button(PSB_BLUE),
    remote.Button(PSB_PAD_UP), remote.Button(PSB_PAD_DOWN), remote.Button(PSB_PAD_LEFT), remote.Button(PSB_PAD_RIGHT),
    remote.Button(PSB_SELECT), remote.Button(PSB_START)
  );
  Serial.println(buffer);
}
