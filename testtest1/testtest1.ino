#include <ECE3.h>

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

uint16_t sensorValues[8];

int left_spd = 0;
int right_spd = 0;

const int mx_spd = 254;

// ========================================
// SPEED SETTINGS - PRESERVED
// ========================================

const int STRAIGHT_SPEED = 95;
const int CORNER_SPEED = 48;
const int CORNER_HOLD_SPEED = 62;
const int LOST_SPEED = 75;

int smooth_base_spd = 60;
unsigned long last_speed_step = 0;

// ========================================
// CORNER SETTINGS - PRESERVED
// ========================================

const unsigned long CORNER_STABLE_MS = 450;

bool corner_mode = false;
unsigned long centered_since = 0;

// ========================================
// U-TURN TEST SETTINGS
// ========================================

// FIRST TEST: stop after only 10 encoder ticks.
// This should produce a very small rotation.
const unsigned long UTURN_TICKS = 360;

// Keep true while verifying the U-turn.
// Once calibrated, change to false.
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

// Diagnostic result:
// 0 = none
// 1 = encoder target reached
// 2 = encoder pulse failure
// 3 = timeout
// 4 = line not found after turn
int stop_reason = 0;

// ========================================
// PD CONTROLLER - PRESERVED
// ========================================

float prev_error = 0;
float filtered_derivative = 0;

const float Kp = 45.0;
const float Ksharp = 30.0;
const float Kd = 25.0;

// ========================================
// MOTOR FUNCTIONS
// ========================================

int abso(int x) {
  return x < 0 ? -x : x;
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
      constrain(abso(left_spd), 0, mx_spd)
  );

  analogWrite(
      right_pwm_pin,
      constrain(abso(right_spd), 0, mx_spd)
  );
}

void stop_wheels() {
  left_spd = 0;
  right_spd = 0;
}

void stop_with_reason(int reason) {
  state = STOPPED;
  stop_reason = reason;
  stop_wheels();
}

// ========================================
// LED DIAGNOSTICS
// ========================================

void update_diagnostic_LED() {
  if (state != STOPPED) {
    digitalWrite(LED_BUILTIN, LOW);
    return;
  }

  unsigned long now = millis();

  if (stop_reason == 1) {
    // SUCCESS: LED stays on.
    digitalWrite(LED_BUILTIN, HIGH);
  }
  else if (stop_reason == 2) {
    // Encoder failure: fast blinking.
    digitalWrite(
        LED_BUILTIN,
        (now / 150) % 2
    );
  }
  else if (stop_reason == 3 ||
           stop_reason == 4) {
    // Timeout or missing line: slow blinking.
    digitalWrite(
        LED_BUILTIN,
        (now / 600) % 2
    );
  }
  else {
    digitalWrite(LED_BUILTIN, LOW);
  }
}

// ========================================
// ENCODER U-TURN
// ========================================

void execute_uturn(unsigned long now) {
  unsigned long elapsed = now - turn_start;

  unsigned long l, r;
  read_encoder_counts(l, r);

  unsigned long average_ticks = (l + r) / 2;

  // ======================================
  // TARGET REACHED
  // ======================================

  if (average_ticks >= UTURN_TICKS) {
    stop_wheels();

    Serial.print("Encoder target reached L=");
    Serial.print(l);
    Serial.print(" R=");
    Serial.println(r);

    if (TURN_TEST_ONLY) {
      // Diagnostic mode: stop here.
      stop_with_reason(1);
      return;
    }

    // Normal operation: begin reacquisition.
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
    smooth_base_spd = 48;

    return;
  }

  // ======================================
  // ENCODER FAILURE
  // ======================================

  if (elapsed > 750 && (l < 5 || r < 5)) {
    stop_with_reason(2);

    Serial.print("ENCODER FAILURE L=");
    Serial.print(l);
    Serial.print(" R=");
    Serial.println(r);

    return;
  }

  // ======================================
  // TIMEOUT
  // ======================================

  if (elapsed > UTURN_MAX_MS) {
    stop_with_reason(3);

    Serial.print("UTURN TIMEOUT L=");
    Serial.print(l);
    Serial.print(" R=");
    Serial.println(r);

    return;
  }

  // ======================================
  // WHEEL SPEED SYNCHRONIZATION
  // ======================================

  unsigned long remaining =
      UTURN_TICKS - average_ticks;

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

  int sync_correction = constrain(
      (int)(difference / 4),
      -12,
      12
  );

  int left_pwm = constrain(
      turn_pwm - sync_correction,
      65,
      100
  );

  int right_pwm = constrain(
      turn_pwm + sync_correction,
      65,
      100
  );

  left_spd = turn_direction * left_pwm;
  right_spd = -turn_direction * right_pwm;
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
  // BRAKING BEFORE TURN
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
    int value = sensorValues[i] - min_value;

    weighted_sum += (long)value * i;
    total += value;
  }

  float position = 3.5;

  if (total > 0) {
    position = (float)weighted_sum / total;
  }

  float error = position - 3.5;
  float abs_error = fabsf(error);

  // ======================================
  // POST-TURN REACQUISITION
  // ======================================

  if (state == AFTER_TURN) {
    unsigned long elapsed = now - turn_end;

    // Let the robot settle briefly.
    if (elapsed < 120) {
      stop_wheels();
      return;
    }

    bool usable_line =
        contrast > 350 &&
        min_value < 1800;

    if (!usable_line) {
      // Move slowly away from the black marker.
      if (elapsed < 600) {
        left_spd = 45;
        right_spd = 45;
      }
      else {
        stop_with_reason(4);
      }

      return;
    }

    // Line detected: resume ordinary PD control.
    state = FOLLOWING;

    prev_error = 0;
    filtered_derivative = 0;
  }

  // ======================================
  // BLACK MARKER DETECTION
  // ======================================

  bool cooldown =
      returning &&
      (now - turn_end < MARKER_IGNORE_MS);

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

    if (min_value > 1800 && marker_armed) {
      black_count++;

      if (black_count >= 4) {
        black_count = 0;
        clear_count = 0;
        marker_armed = false;

        if (!returning) {
          state = BRAKING;
          brake_start = now;

          turn_direction =
              (last_direction > 0) ? -1 : 1;

          stop_wheels();
        }
        else {
          state = STOPPED;
          stop_wheels();
        }

        return;
      }
    }
    else {
      black_count = 0;
    }
  }

  // ======================================
  // LOST LINE RECOVERY
  // ======================================

  if (contrast < 250) {
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

  if (error > 0.2) {
    last_direction = 1;
  }

  if (error < -0.2) {
    last_direction = -1;
  }

  // ======================================
  // PD STEERING
  // ======================================

  float derivative = error - prev_error;

  filtered_derivative =
      0.6 * filtered_derivative +
      0.4 * derivative;

  float correction =
      Kp * error +
      Ksharp * error * abs_error +
      Kd * filtered_derivative;

  float max_correction = 110.0;

  if (abs_error > 1.2) {
    max_correction = 160.0;
  }

  if (abs_error > 2.1) {
    max_correction = 190.0;
  }

  correction = constrain(
      correction,
      -max_correction,
      max_correction
  );

  // ======================================
  // CORNER MODE
  // ======================================

  bool entering_corner =
      abs_error > 0.9 ||
      (abs_error > 0.5 &&
       fabsf(filtered_derivative) > 0.28);

  if (entering_corner) {
    corner_mode = true;
    centered_since = 0;
  }
  else if (corner_mode) {
    bool stable =
        abs_error < 0.4 &&
        fabsf(filtered_derivative) < 0.15;

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
  // ADAPTIVE SPEED
  // ======================================

  int target_base;

  if (abs_error <= 0.4) {
    target_base = STRAIGHT_SPEED;
  }
  else if (abs_error >= 1.3) {
    target_base = CORNER_SPEED;
  }
  else {
    float t = (abs_error - 0.4) / 0.9;

    target_base =
        STRAIGHT_SPEED -
        (int)((STRAIGHT_SPEED - CORNER_SPEED) * t);
  }

  if (corner_mode) {
    target_base = min(
        target_base,
        CORNER_HOLD_SPEED
    );
  }

  // ======================================
  // GRADUAL ACCELERATION
  // ======================================

  if (target_base < smooth_base_spd) {
    smooth_base_spd = target_base;
  }

  if (now - last_speed_step >= 10) {
    if (smooth_base_spd < target_base) {
      smooth_base_spd = min(
          target_base,
          smooth_base_spd + 3
      );
    }

    last_speed_step = now;
  }

  int base_spd = smooth_base_spd;

  left_spd = constrain(
      (int)(base_spd - correction),
      -mx_spd,
      mx_spd
  );

  right_spd = constrain(
      (int)(base_spd + correction),
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

  digitalWrite(left_dir_pin, LOW);
  digitalWrite(left_nslp_pin, HIGH);

  pinMode(right_nslp_pin, OUTPUT);
  pinMode(right_dir_pin, OUTPUT);
  pinMode(right_pwm_pin, OUTPUT);

  digitalWrite(right_dir_pin, LOW);
  digitalWrite(right_nslp_pin, HIGH);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  // Initialize line sensors first.
  ECE3_Init();

  // Then initialize encoder interrupts.
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

  update_diagnostic_LED();
}