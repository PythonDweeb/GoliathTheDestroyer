
#include <ECE3.h>
#include <math.h>

// ========================================
// TRACK PROFILE
// ========================================

// false = regular COTA track
// true = extra-credit long straight
const bool EXTRA_CREDIT_MODE = false;

// ========================================
// SPEED SETTINGS
// ========================================

// Previous working normal speed = 95.
const int NORMAL_STRAIGHT_SPEED = 102;

// Only reached on a confirmed long straight.
const int FAST_STRAIGHT_SPEED =
    EXTRA_CREDIT_MODE ? 125 : 114;

// Preserve working corner speeds.
const int CORNER_SPEED = 48;
const int CORNER_HOLD_SPEED = 62;
const int LOST_SPEED = 75;

// Keep the distance requirement that
// helped the robot survive T4 -> T5.
const unsigned long FAST_DISTANCE_TICKS = 450;

// Enter faster after stable readings.
const unsigned long FAST_STABLE_MS = 70;

// Preserve original corner stability hold.
const unsigned long CORNER_STABLE_MS = 450;

// Fast but controlled acceleration.
const float ACCEL_PWM_PER_MS = 0.35f;

// ========================================
// MOTOR PINS
// ========================================

const int left_nslp_pin = 31;
const int left_dir_pin = 29;
const int left_pwm_pin = 40;

const int right_nslp_pin = 11;
const int right_dir_pin = 30;
const int right_pwm_pin = 39;

const int LEFT_ENCODER_PIN = 12;
const int RIGHT_ENCODER_PIN = 13;

const int mx_spd = 254;

uint16_t sensorValues[8];

int left_spd = 0;
int right_spd = 0;

// ========================================
// U-TURN SETTINGS
// ========================================

const unsigned long UTURN_TICKS = 360;
const bool TURN_TEST_ONLY = false;

const int UTURN_FAST_PWM = 85;
const int UTURN_MED_PWM = 75;
const int UTURN_SLOW_PWM = 65;

const unsigned long BRAKE_MS = 100;
const unsigned long UTURN_MAX_MS = 4500;
const unsigned long MARKER_IGNORE_MS = 1500;

// ========================================
// ENCODERS
// ========================================

volatile unsigned long left_ticks = 0;
volatile unsigned long right_ticks = 0;

void left_encoder_ISR() {
  left_ticks++;
}

void right_encoder_ISR() {
  right_ticks++;
}

void read_encoder_counts(
    unsigned long &l,
    unsigned long &r
) {
  noInterrupts();

  l = left_ticks;
  r = right_ticks;

  interrupts();
}

void reset_encoders() {
  noInterrupts();

  left_ticks = 0;
  right_ticks = 0;

  interrupts();
}

// ========================================
// ROBOT STATE
// ========================================

enum RobotState {
  FOLLOWING,
  BRAKING,
  TURNING,
  AFTER_TURN,
  STOPPED
};

RobotState state = FOLLOWING;

bool returning = false;
bool marker_armed = false;

int clear_count = 0;
int black_count = 0;

int last_direction = 1;
int turn_direction = 1;

unsigned long brake_start = 0;
unsigned long turn_start = 0;
unsigned long turn_end = 0;

int stop_reason = 0;

// ========================================
// ORIGINAL WORKING PD CONTROLLER
// ========================================

const float Kp = 45.0f;
const float Ksharp = 30.0f;
const float Kd = 25.0f;

float prev_error = 0.0f;
float filtered_derivative = 0.0f;

// ========================================
// CORNER STATE
// ========================================

bool corner_mode = false;
unsigned long centered_since = 0;

// ========================================
// SPEED STATE
// ========================================

float current_base_speed = 60.0f;
unsigned long last_speed_update = 0;

// ========================================
// FAST MODE STATE
// ========================================

// Encoder reference from the most recent
// corner or line loss.
unsigned long corner_left_ticks = 0;
unsigned long corner_right_ticks = 0;

// Stable reading timer.
unsigned long fast_stable_since = 0;

// Fast mode stays active through small
// fluctuations, unlike the last version.
bool fast_mode = false;

// LED indicates stable straight tracking.
bool straight_led = false;
unsigned long led_stable_since = 0;

// ========================================
// MOTOR FUNCTIONS
// ========================================

void stop_wheels() {
  left_spd = 0;
  right_spd = 0;
}

void stop_with_reason(int reason) {
  state = STOPPED;
  stop_reason = reason;

  fast_mode = false;
  straight_led = false;

  stop_wheels();
}

void assign_wheels() {
  digitalWrite(
      left_dir_pin,
      left_spd < 0 ? HIGH : LOW
  );

  digitalWrite(
      right_dir_pin,
      right_spd < 0 ? HIGH : LOW
  );

  analogWrite(
      left_pwm_pin,
      constrain(abs(left_spd), 0, mx_spd)
  );

  analogWrite(
      right_pwm_pin,
      constrain(abs(right_spd), 0, mx_spd)
  );
}

// ========================================
// LED DIAGNOSTICS
// ========================================

void update_led() {
  if (state == FOLLOWING) {
    digitalWrite(
        LED_BUILTIN,
        straight_led ? HIGH : LOW
    );
  }
  else if (state == STOPPED &&
           stop_reason == 1) {
    digitalWrite(LED_BUILTIN, HIGH);
  }
  else if (state == STOPPED &&
           stop_reason == 2) {
    digitalWrite(
        LED_BUILTIN,
        (millis() / 150) % 2
    );
  }
  else if (state == STOPPED &&
           (stop_reason == 3 ||
            stop_reason == 4)) {
    digitalWrite(
        LED_BUILTIN,
        (millis() / 600) % 2
    );
  }
  else {
    digitalWrite(LED_BUILTIN, LOW);
  }
}

// ========================================
// FAST MODE RESET
// ========================================

void reset_fast_tracking(
    unsigned long l,
    unsigned long r
) {
  corner_left_ticks = l;
  corner_right_ticks = r;

  fast_stable_since = 0;
  led_stable_since = 0;

  fast_mode = false;
  straight_led = false;
}

// ========================================
// ENCODER U-TURN
// ========================================

void execute_uturn(unsigned long now) {
  unsigned long l, r;
  read_encoder_counts(l, r);

  unsigned long ticks = (l + r) / 2;
  unsigned long elapsed = now - turn_start;

  // ======================================
  // U-TURN COMPLETE
  // ======================================

  if (ticks >= UTURN_TICKS) {
    stop_wheels();

    if (TURN_TEST_ONLY) {
      stop_with_reason(1);
      return;
    }

    state = AFTER_TURN;
    returning = true;
    turn_end = now;

    marker_armed = false;
    clear_count = 0;
    black_count = 0;

    last_direction = -last_direction;

    prev_error = 0;
    filtered_derivative = 0;

    corner_mode = true;
    centered_since = 0;

    current_base_speed = 48;
    last_speed_update = now;

    reset_fast_tracking(l, r);

    return;
  }

  // ======================================
  // ENCODER FAILURE
  // ======================================

  if (elapsed > 750 &&
      (l < 5 || r < 5)) {
    stop_with_reason(2);
    return;
  }

  // ======================================
  // U-TURN TIMEOUT
  // ======================================

  if (elapsed > UTURN_MAX_MS) {
    stop_with_reason(3);
    return;
  }

  // ======================================
  // U-TURN PWM
  // ======================================

  unsigned long remaining =
      UTURN_TICKS - ticks;

  int turn_pwm;

  if (remaining > 80) {
    turn_pwm = UTURN_FAST_PWM;
  }
  else if (remaining > 25) {
    turn_pwm = UTURN_MED_PWM;
  }
  else {
    turn_pwm = UTURN_SLOW_PWM;
  }

  long difference = (long)l - (long)r;

  int sync = constrain(
      (int)(difference / 4),
      -12,
      12
  );

  int lp = constrain(
      turn_pwm - sync,
      65,
      100
  );

  int rp = constrain(
      turn_pwm + sync,
      65,
      100
  );

  left_spd = turn_direction * lp;
  right_spd = -turn_direction * rp;
}

// ========================================
// ACCELERATION / DECELERATION
// ========================================

void update_speed(
    int target,
    unsigned long now
) {
  unsigned long dt =
      now - last_speed_update;

  last_speed_update = now;

  if (dt > 80) {
    dt = 80;
  }

  // Reduce commanded PWM immediately.
  if (current_base_speed > target) {
    current_base_speed = target;
  }
  else {
    current_base_speed +=
        ACCEL_PWM_PER_MS * (float)dt;

    if (current_base_speed > target) {
      current_base_speed = target;
    }
  }
}

// ========================================
// MAIN CONTROLLER
// ========================================

void set_new_wheel_spd() {
  unsigned long now = millis();

  if (state == STOPPED) {
    stop_wheels();
    return;
  }

  // ======================================
  // BRAKING BEFORE U-TURN
  // ======================================

  if (state == BRAKING) {
    stop_wheels();

    if (now - brake_start >= BRAKE_MS) {
      reset_encoders();

      turn_start = now;
      state = TURNING;
    }

    return;
  }

  if (state == TURNING) {
    execute_uturn(now);
    return;
  }

  // ======================================
  // SENSOR POSITION
  // ======================================

  int min_value = sensorValues[0];
  int max_value = sensorValues[0];

  for (int i = 1; i < 8; i++) {
    min_value = min(
        min_value,
        (int)sensorValues[i]
    );

    max_value = max(
        max_value,
        (int)sensorValues[i]
    );
  }

  int contrast = max_value - min_value;

  long weighted_sum = 0;
  long total = 0;

  for (int i = 0; i < 8; i++) {
    int value =
        (int)sensorValues[i] - min_value;

    weighted_sum += (long)value * i;
    total += value;
  }

  float position = 3.5f;

  if (total > 0) {
    position =
        (float)weighted_sum / total;
  }

  float error = position - 3.5f;
  float abs_error = fabsf(error);

  unsigned long l, r;
  read_encoder_counts(l, r);

  // ======================================
  // POST-U-TURN REACQUISITION
  // ======================================

  if (state == AFTER_TURN) {
    unsigned long elapsed = now - turn_end;

    if (elapsed < 120) {
      stop_wheels();
      return;
    }

    bool usable_line =
        contrast > 350 &&
        min_value < 1800;

    if (!usable_line) {
      if (elapsed < 600) {
        left_spd = 45;
        right_spd = 45;
      }
      else {
        stop_with_reason(4);
      }

      return;
    }

    state = FOLLOWING;

    prev_error = 0;
    filtered_derivative = 0;

    reset_fast_tracking(l, r);
  }

  // ======================================
  // BLACK MARKER DETECTION
  // ======================================

  bool cooldown =
      returning &&
      now - turn_end < MARKER_IGNORE_MS;

  if (cooldown) {
    marker_armed = false;
    clear_count = 0;
    black_count = 0;
  }
  else {
    if (min_value <= 1800) {
      clear_count++;

      if (clear_count >= 10) {
        marker_armed = true;
      }
    }
    else {
      clear_count = 0;
    }

    if (min_value > 1800 &&
        marker_armed) {
      black_count++;

      if (black_count >= 4) {
        black_count = 0;
        clear_count = 0;

        marker_armed = false;

        if (!returning) {
          state = BRAKING;
          brake_start = now;

          turn_direction =
              last_direction > 0 ? -1 : 1;

          stop_wheels();
        }
        else {
          stop_with_reason(0);
        }

        return;
      }
    }
    else {
      black_count = 0;
    }
  }

  // ======================================
  // ORIGINAL LOST-LINE RECOVERY
  // ======================================

  if (contrast < 250) {
    reset_fast_tracking(l, r);

    if (max_value > 1500) {
      if (returning &&
          now - turn_end < 300) {
        left_spd = 48;
        right_spd = 48;
      }
      else if (corner_mode) {
        left_spd =
            -last_direction * 60;

        right_spd =
            last_direction * 60;
      }
      else {
        left_spd = 50;
        right_spd = 50;
      }
    }
    else {
      left_spd =
          -last_direction * LOST_SPEED;

      right_spd =
          last_direction * LOST_SPEED;
    }

    prev_error = 0;
    filtered_derivative = 0;

    return;
  }

  // ======================================
  // LAST KNOWN LINE DIRECTION
  // ======================================

  if (error > 0.2f) {
    last_direction = 1;
  }

  if (error < -0.2f) {
    last_direction = -1;
  }

  // ======================================
  // ORIGINAL DERIVATIVE FILTER
  // ======================================

  float derivative =
      error - prev_error;

  filtered_derivative =
      0.6f * filtered_derivative +
      0.4f * derivative;

  // ======================================
  // GENTLER CENTER CORRECTION
  // ======================================

  // Reduce proportional steering only
  // near the center of the sensor array.
  //
  // Sharp-turn proportional steering
  // remains exactly the original strength.

  float gentle_scale;

  if (abs_error <= 0.35f) {
    gentle_scale = 0.65f;
  }
  else if (abs_error >= 0.90f) {
    gentle_scale = 1.0f;
  }
  else {
    gentle_scale =
        0.65f +
        0.35f *
        (abs_error - 0.35f) / 0.55f;
  }

  // Original controller's proportional
  // terms, softened only near the center.
  float proportional =
      Kp * error +
      Ksharp * error * abs_error;

  // Keep the derivative term unscaled.
  // It helps damp crossing the centerline.
  float correction =
      gentle_scale * proportional +
      Kd * filtered_derivative;

  // Preserve original correction limits.
  float cap = 110.0f;

  if (abs_error > 1.2f) {
    cap = 160.0f;
  }

  if (abs_error > 2.1f) {
    cap = 190.0f;
  }

  correction = constrain(
      correction,
      -cap,
      cap
  );

  // ======================================
  // ORIGINAL CORNER DETECTION
  // ======================================

  bool entering_corner =
      abs_error > 0.9f ||
      (
        abs_error > 0.5f &&
        fabsf(filtered_derivative) > 0.28f
      );

  if (entering_corner) {
    corner_mode = true;
    centered_since = 0;
  }
  else if (corner_mode) {
    bool stable =
        abs_error < 0.4f &&
        fabsf(filtered_derivative) < 0.15f;

    if (stable) {
      if (centered_since == 0) {
        centered_since = now;
      }

      if (now - centered_since >=
          CORNER_STABLE_MS) {
        corner_mode = false;
        centered_since = 0;
      }
    }
    else {
      centered_since = 0;
    }
  }

  // ======================================
  // FAST MODE WITH HYSTERESIS
  // ======================================

  if (corner_mode || entering_corner) {
    // Reset distance reference through turns.
    reset_fast_tracking(l, r);
  }
  else {
    // Conditions for ENTERING fast mode.
    bool steady_for_entry =
        contrast > 350 &&
        abs_error < 0.48f &&
        fabsf(filtered_derivative) < 0.18f;

    if (steady_for_entry) {
      if (fast_stable_since == 0) {
        fast_stable_since = now;
      }
    }
    else {
      fast_stable_since = 0;
    }

    unsigned long distance_l =
        l - corner_left_ticks;

    unsigned long distance_r =
        r - corner_right_ticks;

    unsigned long ticks_since_corner =
        (distance_l + distance_r) / 2;

    // ENTER fast mode.
    if (!fast_mode &&
        steady_for_entry &&
        fast_stable_since != 0 &&
        now - fast_stable_since >=
            FAST_STABLE_MS &&
        ticks_since_corner >=
            FAST_DISTANCE_TICKS) {
      fast_mode = true;
    }

    // EXIT only for a substantial deviation.
    // Don't exit for every tiny fluctuation.
    if (fast_mode &&
        (
          contrast < 300 ||
          abs_error > 0.75f ||
          fabsf(filtered_derivative) > 0.32f
        )) {
      fast_mode = false;
    }

    // LED: centered and relatively steady.
    bool led_centered =
        contrast > 350 &&
        abs_error < 0.55f &&
        fabsf(filtered_derivative) < 0.20f;

    if (led_centered) {
      if (led_stable_since == 0) {
        led_stable_since = now;
      }

      if (now - led_stable_since >= 60) {
        straight_led = true;
      }
    }
    else {
      led_stable_since = 0;
      straight_led = false;
    }
  }

  // ======================================
  // ORIGINAL ADAPTIVE SPEED
  // ======================================

  int target;

  if (abs_error <= 0.4f) {
    target = NORMAL_STRAIGHT_SPEED;
  }
  else if (abs_error >= 1.3f) {
    target = CORNER_SPEED;
  }
  else {
    float t =
        (abs_error - 0.4f) / 0.9f;

    target =
        NORMAL_STRAIGHT_SPEED -
        (int)(
          (NORMAL_STRAIGHT_SPEED -
           CORNER_SPEED) * t
        );
  }

  // Preserve the corner speed restriction.
  if (corner_mode) {
    target = min(
        target,
        CORNER_HOLD_SPEED
    );
  }

  // ======================================
  // FAST STRAIGHT SPEED
  // ======================================

  // Fast mode may remain armed during tiny
  // fluctuations, but actual fast PWM
  // requires reasonable current alignment.
  if (fast_mode &&
      !corner_mode &&
      abs_error < 0.60f &&
      fabsf(filtered_derivative) < 0.24f) {

    target = FAST_STRAIGHT_SPEED;
  }

  // ======================================
  // ACCELERATION / DECELERATION
  // ======================================

  update_speed(target, now);

  // ======================================
  // FINAL WHEEL SPEEDS
  // ======================================

  left_spd = constrain(
      (int)(current_base_speed - correction),
      -mx_spd,
      mx_spd
  );

  right_spd = constrain(
      (int)(current_base_speed + correction),
      -mx_spd,
      mx_spd
  );

  prev_error = error;
}

// ========================================
// SETUP
// ========================================

void setup() {
  pinMode(left_nslp_pin, OUTPUT);
  pinMode(left_dir_pin, OUTPUT);
  pinMode(left_pwm_pin, OUTPUT);

  pinMode(right_nslp_pin, OUTPUT);
  pinMode(right_dir_pin, OUTPUT);
  pinMode(right_pwm_pin, OUTPUT);

  digitalWrite(left_dir_pin, LOW);
  digitalWrite(right_dir_pin, LOW);

  digitalWrite(left_nslp_pin, HIGH);
  digitalWrite(right_nslp_pin, HIGH);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  // Preserve the working encoder setup order.
  ECE3_Init();

  pinMode(LEFT_ENCODER_PIN, INPUT);
  pinMode(RIGHT_ENCODER_PIN, INPUT);

  attachInterrupt(
      digitalPinToInterrupt(LEFT_ENCODER_PIN),
      left_encoder_ISR,
      RISING
  );

  attachInterrupt(
      digitalPinToInterrupt(RIGHT_ENCODER_PIN),
      right_encoder_ISR,
      RISING
  );

  Serial.begin(9600);

  delay(2000);

  left_spd = 20;
  right_spd = 20;

  last_speed_update = millis();

  assign_wheels();
}

// ========================================
// MAIN LOOP
// ========================================

void loop() {
  if (state == FOLLOWING ||
      state == AFTER_TURN) {
    ECE3_read_IR(sensorValues);
  }

  set_new_wheel_spd();
  assign_wheels();
  update_led();
}
