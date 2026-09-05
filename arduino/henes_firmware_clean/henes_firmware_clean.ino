// =============================================================================
// henes_firmware.ino  —  HENES T870 autonomous low-level controller
// Pipeline:  [PID] -> PWM -> [safety guard] -> analogWrite(motor)
//   drive : dual DC motor, encoder velocity PID + FF + anti-windup + soft-start
//   steer : DC motor + potentiometer (A15), position PID, calibrated mapping
//   plus  : 3x sonar, serial (serial_bridge format), watchdog
// Bench first: wheels off ground, start MAX_*_PWM low, raise gradually.
// =============================================================================
#include <SPI.h>
#include <NewPing.h>

// ------------------------------ 1. Pins --------------------------------------
#define MOTOR1_PWM 5    // front drive
#define MOTOR1_ENA 6
#define MOTOR1_ENB 7
#define MOTOR2_PWM 2    // rear drive
#define MOTOR2_ENA 3
#define MOTOR2_ENB 4
#define MOTOR3_PWM 8    // steer
#define MOTOR3_ENA 9
#define MOTOR3_ENB 10

#define Steering_Sensor A15   // steering potentiometer
#define ENC1_ADD 22           // encoder SPI CS (LS7366R)
#define ENC2_ADD 23
#define SONAR_NUM 3
#define MAX_DISTANCE 200

// ------------------------------ 2. Steering calibration ----------------------
// Measured on this board (motor OFF): left rail 936 / center 412 / right rail 0.
// Sign: left(+) -> ADC up, right(-) -> ADC down. Per-direction slope (linkage
// asymmetry): left 26.2, right 20.6 counts/deg. Conservative (smaller value per
// side) to avoid oversteer.
#define STEER_CENTER    412
#define STEER_LEFT_MAX    0
#define STEER_RIGHT_MAX 936
#define STEER_MAX_ANGLE  20.0        // measured max lock (not the assumed 30)
#define STEER_AD_MIN  (STEER_LEFT_MAX  + 10)
#define STEER_AD_MAX  (STEER_RIGHT_MAX - 10)
#define STEER_CPD_LEFT   26.2        // +angle (left)  counts/deg
#define STEER_CPD_RIGHT  20.6        // -angle (right) counts/deg
#define STEER_COUNTS_PER_DEG  23.4   // legacy average reference

int steerAngleToADC(float ang) {
  if (ang >  STEER_MAX_ANGLE) ang =  STEER_MAX_ANGLE;
  if (ang < -STEER_MAX_ANGLE) ang = -STEER_MAX_ANGLE;
  float adc = STEER_CENTER + ang * (ang >= 0 ? STEER_CPD_LEFT : STEER_CPD_RIGHT);
  return constrain((int)adc, STEER_LEFT_MAX, STEER_RIGHT_MAX);
}
float steerADCToAngle(int adc) {     // actual wheel angle from ADC (tracking error monitor)
  int d = adc - STEER_CENTER;
  return (d >= 0) ? (d / STEER_CPD_LEFT) : (d / STEER_CPD_RIGHT);
}

// ------------------------------ 3. Safety parameters -------------------------
#define MAX_DRIVE_PWM   160          // drive PWM cap (raise only while watching VMIN)
#define MAX_STEER_PWM   130
#define STEER_MOTOR_TEST 0           // 1 = steer motor OFF (turn by hand to read ADC)
#define NO_ENCODER 0                 // 1 = open-loop FF drive, drive stall guard off
#define STEER_SOFT_MS   2500UL       // boot steer PWM ramp 0->MAX (inrush limit)
#define STEER_SOFT_MIN  25

// steering position-control tuning
#define STEER_DEADBAND    16   // reached target within this -> stop
#define STEER_RESUME      24   // must exceed this to restart (hysteresis)
#define STEER_MIN_MOVE    34   // stiction: below this DC the motor does not move
#define STEER_BOOST_STEP   6   // stiction-escape ramp per 10ms cycle
#define STEER_MOVING_DELTA 2   // ADC change >= this counts as "moving"

// drive stall: high PWM but not moving, while commanded to move
#define DRIVE_STALL_PWM    55
#define DRIVE_STALL_SPEED  0.05
#define DRIVE_STALL_MS     700
#define DRIVE_STALL_COOLDOWN_MS 2000

// steer stall: high PWM but ADC not changing and error still large
#define STEER_STALL_PWM    30
#define STEER_STALL_DELTA  3
#define STEER_STALL_ERR    15
#define STEER_STALL_MS     250
#define STEER_STALL_COOLDOWN_MS 1500

#define SERIAL_TIMEOUT_MS  500       // no command -> watchdog stop
#define CONTROL_DT_MS      10        // 100 Hz
#define TEL_INTERVAL_MS    50        // 20 Hz telemetry

// accel/brake slew limits (m/s^2)
const float ACCEL_LIMIT = 0.8;
const float BRAKE_LIMIT = 0.8;
const float WATCHDOG_BRAKE_LIMIT = 2.5;

// ------------------------------ 4. Global state ------------------------------
float target_velocity = 0.0;     // ROS target speed
float commanded_velocity = 0.0;  // soft-start slewed target
float current_velocity = 0.0;
float target_steer_angle = 0.0;  // ROS target steer angle (deg)

// open-loop mode: "PWM:x" bypasses velocity PID for FF identification
bool openloop_active = false;
int openloop_target_pwm = 0;
int openloop_pwm = 0;
#define OPENLOOP_RATE     3
#define MAX_OPENLOOP_PWM 230

signed long encoder1count = 0, encoder2count = 0, prev_encoder1 = 0;
unsigned long prev_time = 0, last_rx_time = 0, last_tel_time = 0;
bool watchdog_tripped = true;    // start tripped: motors off until first command

unsigned long drive_stall_ms = 0, steer_stall_ms = 0, steer_cut_ms = 0;
unsigned long drive_cut_ms = 0;
bool drive_stalled = false, steer_stalled = false;
int prev_sensorValue = STEER_CENTER;

// wheel / encoder. Forward = encoder1count DECREASES (cpr is negative).
const float wheel_radius = 0.1327;
const int counts_per_revolution = -290;
const float wheel_circumference = 2 * 3.14159 * wheel_radius;

// drive velocity PID (FF carries the load, PID trims the residual)
float velocity_kp = 30.0, velocity_ki = 15.0, velocity_kd = 0.5;
float velocity_error = 0.0, velocity_error_old = 0.0, velocity_error_sum = 0.0;
int velocity_pwm_output = 0;
const float VELOCITY_DT = CONTROL_DT_MS / 1000.0;
const float STATIC_FF = 80.0, VELOCITY_FF_GAIN = 95.0;   // PWM = 80 + 95*v

// steering position PID
float steering_kp = 1.0, steering_ki = 0.0, steering_kd = 0.2;
float steering_error = 0.0, steering_error_old = 0.0, steering_error_sum = 0.0;
int steering_pwm_output = 0;
int sensorValue = STEER_CENTER;

NewPing sonar[SONAR_NUM] = {
  NewPing(11, 11, MAX_DISTANCE),
  NewPing(12, 12, MAX_DISTANCE),
  NewPing(13, 13, MAX_DISTANCE)
};

// ------------------------------ 5. Low-level motor control -------------------
void front_motor_control(int pwm) {
  int lim = openloop_active ? MAX_OPENLOOP_PWM : MAX_DRIVE_PWM;
  pwm = constrain(pwm, -lim, lim);
  if (pwm > 0)      { digitalWrite(MOTOR1_ENA, HIGH); digitalWrite(MOTOR1_ENB, LOW);  analogWrite(MOTOR1_PWM, pwm); }
  else if (pwm < 0) { digitalWrite(MOTOR1_ENA, LOW);  digitalWrite(MOTOR1_ENB, HIGH); analogWrite(MOTOR1_PWM, -pwm); }
  else              { digitalWrite(MOTOR1_ENA, LOW);  digitalWrite(MOTOR1_ENB, LOW);  analogWrite(MOTOR1_PWM, 0); }
}
void rear_motor_control(int pwm) {
  int lim = openloop_active ? MAX_OPENLOOP_PWM : MAX_DRIVE_PWM;
  pwm = constrain(pwm, -lim, lim);
  if (pwm > 0)      { digitalWrite(MOTOR2_ENA, HIGH); digitalWrite(MOTOR2_ENB, LOW);  analogWrite(MOTOR2_PWM, pwm); }
  else if (pwm < 0) { digitalWrite(MOTOR2_ENA, LOW);  digitalWrite(MOTOR2_ENB, HIGH); analogWrite(MOTOR2_PWM, -pwm); }
  else              { digitalWrite(MOTOR2_ENA, LOW);  digitalWrite(MOTOR2_ENB, LOW);  analogWrite(MOTOR2_PWM, 0); }
}
// steer motor with end-stop hard cut (pwm>0 pushes ADC up = physical left)
void steer_motor_control(int pwm) {
  if (STEER_MOTOR_TEST) pwm = 0;
  pwm = constrain(pwm, -MAX_STEER_PWM, MAX_STEER_PWM);
  bool at_right_limit = (sensorValue >= STEER_AD_MAX);
  bool at_left_limit  = (sensorValue <= STEER_AD_MIN);
  if ((pwm > 0 && at_right_limit) || (pwm < 0 && at_left_limit)) pwm = 0;
  if (pwm > 0)      { digitalWrite(MOTOR3_ENA, LOW);  digitalWrite(MOTOR3_ENB, HIGH); analogWrite(MOTOR3_PWM, pwm); }
  else if (pwm < 0) { digitalWrite(MOTOR3_ENA, HIGH); digitalWrite(MOTOR3_ENB, LOW);  analogWrite(MOTOR3_PWM, -pwm); }
  else              { digitalWrite(MOTOR3_ENA, LOW);  digitalWrite(MOTOR3_ENB, LOW);  analogWrite(MOTOR3_PWM, 0); }
}
void all_motors_off() { front_motor_control(0); rear_motor_control(0); steer_motor_control(0); }

// ------------------------------ 5.5 Supply voltage (VCC) monitor -------------
// Read internal 1.1V bandgap to back out VCC; sample every cycle, report min.
int vcc_mv = 0;
int vcc_min_mv = 9999;

int readVccMv() {
  ADCSRB &= ~_BV(MUX5);
  ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  ADCSRA |= _BV(ADSC); while (bit_is_set(ADCSRA, ADSC));
  ADCSRA |= _BV(ADSC); while (bit_is_set(ADCSRA, ADSC));
  uint8_t low = ADCL, high = ADCH;
  long raw = ((long)high << 8) | low;
  if (raw == 0) return 0;
  return (int)(1125300L / raw);
}

// ------------------------------ 6. Encoder (LS7366R over SPI) ----------------
void initEncoders() {
  pinMode(ENC1_ADD, OUTPUT); pinMode(ENC2_ADD, OUTPUT);
  digitalWrite(ENC1_ADD, HIGH); digitalWrite(ENC2_ADD, HIGH);
  SPI.begin();
  digitalWrite(ENC1_ADD, LOW); SPI.transfer(0x88); SPI.transfer(0x03); digitalWrite(ENC1_ADD, HIGH);
  digitalWrite(ENC2_ADD, LOW); SPI.transfer(0x88); SPI.transfer(0x03); digitalWrite(ENC2_ADD, HIGH);
}
long readEncoder(int no) {
  unsigned int c1, c2, c3, c4;
  digitalWrite(ENC1_ADD + no - 1, LOW);
  SPI.transfer(0x60);
  c1 = SPI.transfer(0x00); c2 = SPI.transfer(0x00); c3 = SPI.transfer(0x00); c4 = SPI.transfer(0x00);
  digitalWrite(ENC1_ADD + no - 1, HIGH);
  return ((long)c1 << 24) + ((long)c2 << 16) + ((long)c3 << 8) + (long)c4;
}
void clearEncoderCount(int no) {
  digitalWrite(ENC1_ADD + no - 1, LOW);
  SPI.transfer(0x98); SPI.transfer(0x00); SPI.transfer(0x00); SPI.transfer(0x00); SPI.transfer(0x00);
  digitalWrite(ENC1_ADD + no - 1, HIGH);
  delayMicroseconds(100);
  digitalWrite(ENC1_ADD + no - 1, LOW); SPI.transfer(0xE0); digitalWrite(ENC1_ADD + no - 1, HIGH);
}

// ------------------------------ 7. Velocity (10ms moving average) ------------
void calculate_velocity() {
  long d = encoder1count - prev_encoder1;
  float rev = (float)d / counts_per_revolution;
  float raw = (rev * wheel_circumference) / VELOCITY_DT;
  static float buf[5] = {0,0,0,0,0}; static int bi = 0;
  buf[bi] = raw; bi = (bi + 1) % 5;
  float s = 0; for (int i = 0; i < 5; i++) s += buf[i];
  current_velocity = s / 5.0;
  prev_encoder1 = encoder1count;
}

// ------------------------------ 8. Soft-start + drive velocity PID -----------
void apply_acceleration_limit() {
  float limit = watchdog_tripped ? WATCHDOG_BRAKE_LIMIT
                : ((fabs(target_velocity) > fabs(commanded_velocity)) ? ACCEL_LIMIT : BRAKE_LIMIT);
  float md = limit * VELOCITY_DT;
  if (target_velocity > commanded_velocity) commanded_velocity = min(target_velocity, commanded_velocity + md);
  else if (target_velocity < commanded_velocity) commanded_velocity = max(target_velocity, commanded_velocity - md);
}

// ===== (ramp anti-rollback) — position feedback =============
//
// PROBLEM: at a stop the velocity setpoint is 0, so FF(0)=0 PWM and the car has
//   no holding torque. On a slope it coasts backward; the velocity loop cannot
//   arrest it because its tiny correction PWM stays below the stiction threshold
//   (measured 2026-09-02: it just slid until caught by hand).
//
// FIX: when stopped, LATCH the current encoder count (hold_target) and run a
//   position PI loop that drives the wheel back to that latched point.
//     perr = encoder1count - hold_target
//   Forward motion DECREASES the count (counts_per_revolution < 0), so rolling
//   BACKWARD INCREASES it -> perr > 0 -> +PWM -> forward push (arrests rollback).
//   The integral keeps raising PWM past stiction until the car is actually held;
//   the proportional term sets how quickly it reacts.
//
// STALL GUARD: no special handling needed. The drive stall guard only fires when
//   target_velocity > 0.05 (see safety_guard). During hold target_velocity ~= 0,
//   so the guard is inactive and will NOT cut the sustained holding PWM.
//
// SIGN CHECK (do this on the bench, wheels off ground, before any ramp):
//   with VEL:0, roll a wheel backward by hand -> the motor must push it FORWARD.
//   If it pushes further backward, flip the sign of HOLD_KP (or swap perr terms).
//
// TUNING:
//   HOLD_KP  ~10 PWM/cm at 3.0 (1 count ~= 2.87mm). If holding a slope needs
//            ~100 PWM, P alone settles at ~10cm of rollback before it holds, so
//            raising HOLD_KP (8~10) reduces the initial slip; watch for buzz/
//            oscillation. Integral then trims steady-state to ~0.
//   Optional v2: add a gravity feed-forward (IUPHILL STOP-HOLD MU pitch -> g*sin(theta)) to kill
//            the initial slip, and blend hold PWM into FF on release.
bool  hold_active = false;   // true while position-hold is engaged
long  hold_target = 0;       // encoder count latched at the moment of stopping
float hold_isum   = 0.0;     // hold integral accumulator
#define HOLD_KP        3.0     // PWM per encoder count (restoring strength)
#define HOLD_KI        0.5     // integral (removes residual creep)
#define HOLD_KD        3.0 
#define HOLD_ISUM_MAX  300.0   // integral authority clamp (anti-windup)
#define HOLD_MAX_PWM   160     // hold PWM cap

void velocity_pid_control() {
  // open-loop: bypass PID, rate-limit the requested PWM
  if (openloop_active) {
    int d = openloop_target_pwm - openloop_pwm;
    if (d >  OPENLOOP_RATE) d =  OPENLOOP_RATE;
    if (d < -OPENLOOP_RATE) d = -OPENLOOP_RATE;
    openloop_pwm += d;
    velocity_pwm_output = constrain(openloop_pwm,
                                    -MAX_OPENLOOP_PWM, MAX_OPENLOOP_PWM);
    velocity_error_sum = 0.0;
    velocity_error_old = 0.0;
    commanded_velocity = 0.0;
    return;
  }
#if NO_ENCODER
  // no encoder feedback -> open-loop FF only (PWM = 80 + 95*v)
  {
    float ffo = 0.0;
    if (commanded_velocity > 0.02)       ffo =  STATIC_FF + VELOCITY_FF_GAIN * commanded_velocity;
    else if (commanded_velocity < -0.02) ffo = -STATIC_FF + VELOCITY_FF_GAIN * commanded_velocity;
    velocity_pwm_output = constrain((int)ffo, -MAX_DRIVE_PWM, MAX_DRIVE_PWM);
    if (fabs(target_velocity) < 0.05 && fabs(commanded_velocity) < 0.05)
      velocity_pwm_output = 0;
    velocity_error = 0.0; velocity_error_sum = 0.0; velocity_error_old = 0.0;
    return;
  }
#endif
  velocity_error = commanded_velocity - current_velocity;
  float ed = (velocity_error - velocity_error_old) / VELOCITY_DT;
  float ff = 0.0;
  if (commanded_velocity > 0.02)  ff =  STATIC_FF + VELOCITY_FF_GAIN * commanded_velocity;
  else if (commanded_velocity < -0.02) ff = -STATIC_FF + VELOCITY_FF_GAIN * commanded_velocity;
  // conditional anti-windup
  float ts = velocity_error_sum + velocity_error * VELOCITY_DT;
  ts = constrain(ts, -8.0, 8.0);
  float tout = ff + velocity_kp * velocity_error + velocity_ki * ts + velocity_kd * ed;
  bool sat_p = (tout > MAX_DRIVE_PWM && velocity_error > 0);
  bool sat_n = (tout < -MAX_DRIVE_PWM && velocity_error < 0);
  if (!sat_p && !sat_n) velocity_error_sum = ts;
  float out = ff + velocity_kp * velocity_error + velocity_ki * velocity_error_sum + velocity_kd * ed;
  velocity_pwm_output = constrain((int)out, -MAX_DRIVE_PWM, MAX_DRIVE_PWM);

  // ---- stop-hold: overrides the PWM above when stopped (see block at top) ----
  if (fabs(target_velocity) < 0.05 && fabs(commanded_velocity) < 0.05) {
    if (!hold_active) {                 // engage: latch current position
      hold_active = true;
      hold_target = encoder1count;
      hold_isum   = 0.0;
    }
    long perr = encoder1count - hold_target;      // rolled back -> +
    hold_isum += (float)perr * VELOCITY_DT;
    hold_isum = constrain(hold_isum, -HOLD_ISUM_MAX, HOLD_ISUM_MAX);
    float hout = HOLD_KP * (float)perr + HOLD_KI * hold_isum;
    velocity_pwm_output = constrain((int)hout, -HOLD_MAX_PWM, HOLD_MAX_PWM);
    velocity_error_sum = 0.0;           // keep velocity integral clean for resume
  } else {
    hold_active = false;                // moving again -> release hold
  }
  velocity_error_old = velocity_error;
}

// ------------------------------ 9. Steering position PID ---------------------
void steering_pid_control() {
  if (watchdog_tripped) {              // comms lost: freeze steer (no low-PWM stall)
    steering_pwm_output = 0;
    steering_error_old = 0;
    return;
  }
  int target_adc = steerAngleToADC(target_steer_angle);
  steering_error = target_adc - sensorValue;
  float ed = steering_error - steering_error_old;
  // hysteresis deadband: stop within DEADBAND, restart only past RESUME (no buzz)
  static bool settled = true;
  int ae = abs(steering_error);
  if (settled) { if (ae > STEER_RESUME)  settled = false; }
  else         { if (ae <= STEER_DEADBAND) settled = true;  }

  static int breakaway_boost = 0;
  int pwm;
  if (settled) {
    pwm = 0;
    breakaway_boost = 0;
  } else {
    pwm = (int)(steering_kp * steering_error + steering_kd * ed);
    if (abs(pwm) < STEER_MIN_MOVE)
      pwm = (steering_error > 0) ? STEER_MIN_MOVE : -STEER_MIN_MOVE;
    // stiction escape: ramp PWM while commanded but ADC not changing; reset on move
    bool moving = abs(sensorValue - prev_sensorValue) >= STEER_MOVING_DELTA;
    if (moving) {
      breakaway_boost = 0;
    } else {
      int room = MAX_STEER_PWM - abs(pwm);
      if (room < 0) room = 0;
      breakaway_boost = min(breakaway_boost + STEER_BOOST_STEP, room);
    }
    pwm += (pwm > 0) ? breakaway_boost : -breakaway_boost;
  }
  // boot steer soft-start: ramp PWM cap 0->MAX over STEER_SOFT_MS (inrush limit)
  int lim = MAX_STEER_PWM;
  unsigned long up = millis();
  if (up < STEER_SOFT_MS) {
    lim = (int)((long)MAX_STEER_PWM * up / STEER_SOFT_MS);
    if (lim < STEER_SOFT_MIN) lim = STEER_SOFT_MIN;
  }
  steering_pwm_output = constrain(pwm, -lim, lim);
  steering_error_old = steering_error;
}

// ------------------------------ 10. Safety guard (stall detection) -----------
void safety_guard() {
  // drive stall: high PWM, not moving, and commanded to move (skipped if NO_ENCODER)
  if (!NO_ENCODER &&
      abs(velocity_pwm_output) > DRIVE_STALL_PWM &&
      fabs(current_velocity) < DRIVE_STALL_SPEED &&
      (fabs(target_velocity) > 0.05 || openloop_target_pwm != 0)) {
    drive_stall_ms += CONTROL_DT_MS;
    if (drive_stall_ms >= DRIVE_STALL_MS) drive_stalled = true;
  } else {
    drive_stall_ms = 0;
  }
  if (drive_stalled) {
    velocity_pwm_output = 0;             // cut
    velocity_error_sum = 0;
    drive_cut_ms += CONTROL_DT_MS;
    if ((fabs(target_velocity) < 0.05 && openloop_target_pwm == 0) ||
        drive_cut_ms >= DRIVE_STALL_COOLDOWN_MS) {
      drive_stalled = false;
      drive_cut_ms = 0;
      drive_stall_ms = 0;
    }
  } else {
    drive_cut_ms = 0;
  }

  // steer stall: high PWM, ADC not changing, error still large
  if (abs(steering_pwm_output) > STEER_STALL_PWM &&
      abs(sensorValue - prev_sensorValue) < STEER_STALL_DELTA &&
      abs(steering_error) > STEER_STALL_ERR) {
    steer_stall_ms += CONTROL_DT_MS;
    if (steer_stall_ms >= STEER_STALL_MS) steer_stalled = true;
  } else {
    steer_stall_ms = 0;
  }
  if (steer_stalled) {
    steering_pwm_output = 0;             // cut
    steering_error_sum = 0;
    steer_cut_ms += CONTROL_DT_MS;
    if (abs(steering_error) < STEER_STALL_ERR ||
        steer_cut_ms >= STEER_STALL_COOLDOWN_MS) {
      steer_stalled = false;
      steer_cut_ms = 0;
      steer_stall_ms = 0;
    }
  } else {
    steer_cut_ms = 0;
  }
  prev_sensorValue = sensorValue;
}

// ------------------------------ 11. Serial parser + watchdog -----------------
void parseCommand(String line) {
  line.trim();
  if (line.length() == 0) return;
  int vi = line.indexOf("VEL:");
  if (vi >= 0) {
    int st = vi + 4, en = line.indexOf(",", st);
    float v = (en >= 0 ? line.substring(st, en) : line.substring(st)).toFloat();
    target_velocity = constrain(v, -3.0, 3.0);
    if (openloop_active) {           // VEL command -> back to closed loop
      openloop_active = false;
      openloop_target_pwm = 0;
      openloop_pwm = 0;
    }
  }
  int pi = line.indexOf("PWM:");        // open-loop identification command
  if (pi >= 0) {
    int st = pi + 4, en = line.indexOf(",", st);
    int v = (en >= 0 ? line.substring(st, en) : line.substring(st)).toInt();
    openloop_active = true;
    openloop_target_pwm = constrain(v, -MAX_OPENLOOP_PWM, MAX_OPENLOOP_PWM);
    target_velocity = 0.0;
    commanded_velocity = 0.0;
  }
  int si = line.indexOf("STEER:");
  if (si >= 0) {
    int st = si + 6, en = line.indexOf(",", st);
    float s = (en >= 0 ? line.substring(st, en) : line.substring(st)).toFloat();
    target_steer_angle = constrain(s, -STEER_MAX_ANGLE, STEER_MAX_ANGLE);
  }
  last_rx_time = millis();
  watchdog_tripped = false;
}
void check_watchdog() {
  if (millis() - last_rx_time > SERIAL_TIMEOUT_MS) {
    if (!watchdog_tripped) Serial.println("WATCHDOG: serial timeout - stop");
    watchdog_tripped = true;
    target_velocity = 0.0;
    openloop_active = false;
    openloop_target_pwm = 0;
    openloop_pwm = 0;
  }
}

// ------------------------------ 12. Telemetry (serial_bridge format) ---------
void send_telemetry() {
  unsigned long now = millis();
  if (now - last_tel_time < TEL_INTERVAL_MS) return;
  last_tel_time = now;
  const char* slope = "FLAT";
  Serial.print("STATUS_10ms: ENC1="); Serial.print(encoder1count);
  Serial.print(" VEL="); Serial.print(current_velocity, 3);
  Serial.print(" TARGET="); Serial.print(target_velocity, 3);
  Serial.print(" PWM="); Serial.print(velocity_pwm_output);
  Serial.print(" SLOPE="); Serial.print(slope);
  Serial.print(" MODE="); Serial.print(openloop_active ? "OPENLOOP" : "PID");
  static float sonar_m[SONAR_NUM] = {0, 0, 0};   // ping one sonar per cycle (non-blocking)
  static uint8_t si = 0;
  sonar_m[si] = sonar[si].ping_cm() / 100.0;
  si = (si + 1) % SONAR_NUM;
  Serial.print(" SONAR1="); Serial.print(sonar_m[0], 2);
  Serial.print(" SONAR2="); Serial.print(sonar_m[1], 2);
  Serial.print(" SONAR3="); Serial.println(sonar_m[2], 2);
  Serial.print("STEER: ADC="); Serial.print(sensorValue);
  Serial.print(" TGT="); Serial.print(steerAngleToADC(target_steer_angle));
  Serial.print(" PWM="); Serial.print(steering_pwm_output);
  Serial.print(" ANG="); Serial.print(target_steer_angle, 1);
  Serial.print(" ANGACT="); Serial.print(steerADCToAngle(sensorValue), 1);
  Serial.print(" VCC="); Serial.print(vcc_mv);
  Serial.print(" VMIN="); Serial.println(vcc_min_mv);
  vcc_min_mv = 9999;
  if (drive_stalled || steer_stalled) {
    Serial.print("STALL: drive="); Serial.print(drive_stalled);
    Serial.print(" steer="); Serial.println(steer_stalled);
  }
}

// ------------------------------ 13. setup ------------------------------------
void setup() {
  Serial.begin(57600);
  // PWM frequency: drive stays 488Hz (driver limit), steer 3.9kHz (noise).
  // Timer0 (pins 4,13) is millis()/delay() — do not touch.
  TCCR3B = (TCCR3B & 0b11111000) | 0x03;   // drive 488Hz
  TCCR4B = (TCCR4B & 0b11111000) | 0x02;   // steer 3.9kHz

  int mp[] = {MOTOR1_PWM,MOTOR1_ENA,MOTOR1_ENB,MOTOR2_PWM,MOTOR2_ENA,MOTOR2_ENB,MOTOR3_PWM,MOTOR3_ENA,MOTOR3_ENB};
  for (int i = 0; i < 9; i++) pinMode(mp[i], OUTPUT);
  all_motors_off();
  initEncoders(); clearEncoderCount(1); clearEncoderCount(2);
  sensorValue = analogRead(Steering_Sensor);
  prev_sensorValue = sensorValue;
  prev_time = last_rx_time = last_tel_time = millis();
  Serial.println("HENES firmware ready: drive PID + steer PID + STALL guard + watchdog");
  Serial.print("  STEER cal: center="); Serial.print(STEER_CENTER);
  Serial.print(" L="); Serial.print(STEER_LEFT_MAX);
  Serial.print(" R="); Serial.print(STEER_RIGHT_MAX);
  Serial.print("  MAX_PWM drive="); Serial.print(MAX_DRIVE_PWM);
  Serial.print(" steer="); Serial.println(MAX_STEER_PWM);
}

// ------------------------------ 14. loop (100Hz control) ---------------------
void loop() {
  while (Serial.available() > 0) {
    String cmd = Serial.readStringUntil('\n');
    parseCommand(cmd);
  }
  check_watchdog();

  unsigned long now = millis();
  if (now - prev_time >= CONTROL_DT_MS) {
    prev_time = now;

    encoder1count = readEncoder(1);
    encoder2count = readEncoder(2);
    sensorValue = analogRead(Steering_Sensor);
    vcc_mv = readVccMv();
    if (vcc_mv > 0 && vcc_mv < vcc_min_mv) vcc_min_mv = vcc_mv;
    calculate_velocity();

    apply_acceleration_limit();
    velocity_pid_control();
    steering_pid_control();

    safety_guard();                   // overrides PID results

    front_motor_control(velocity_pwm_output);
    rear_motor_control(velocity_pwm_output);
    steer_motor_control(steering_pwm_output);
  }

  send_telemetry();
}
