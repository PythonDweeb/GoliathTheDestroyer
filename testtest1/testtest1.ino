#include <ECE3.h>

// ========================================
// VERSION 9
// ========================================
//
// Check this line is in the file you flash.
// The first version had STRAIGHT_SPEED 112;
// this one has 170.
//
// Search "FIX 1" ... "FIX 12" for each change.
//
// MAIN KNOBS
//
// Faster / slower overall:
//   CRUISE_SPEED, BEND_SPEED, CORNER_SPEED
// Straight-line boost:
//   STRAIGHT_SPEED
// LED never lights on straights:
//   raise STRAIGHT_ENTER_ERROR (0.40 -> 0.50)
//   and STRAIGHT_MAX_ERROR (0.55 -> 0.70)
// Brakes too much:
//   raise ENTRY_BRAKE_MIN_EXCESS (30 -> 45)
//   raise FAST_SHARP_ERROR (0.65 -> 0.85)
// Runs past sharp turns:
//   lower STRAIGHT_SPEED, or make
//   ENTRY_BRAKE_BASE more negative
// Flies off coming OUT of a turn:
//   raise STRAIGHT_AFTER_SHARP_MS
// Slides past the black block:
//   raise MARKER_BRAKE_PWM
// Backs up at the black block:
//   lower MARKER_BRAKE_PWM

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
// SPEED SETTINGS -- DISCRETE MODES
// ========================================

// false = regular track
// true = extra-credit long straight
const bool EXTRA_CREDIT_MODE = false;

const int STRAIGHT_SPEED =
    EXTRA_CREDIT_MODE ? 190 : 170;

const int CRUISE_SPEED =
    EXTRA_CREDIT_MODE ? 125 : 120;

const int BEND_SPEED = 95;
const int EARLY_BRAKE_SPEED = 78;
const int CORNER_SPEED = 58;
const int HARD_EXIT_SPEED = 100;
const int LOST_SPEED = 75;

// Accelerate by 0.50 PWM per millisecond.
// Reduce commanded PWM immediately.
const float ACCEL_PWM_PER_MS = 0.50f;

float smooth_base_spd = 60.0f;
unsigned long last_speed_step = 0;

// ========================================
// DISCRETE TRACK MODES
// ========================================

enum TrackMode {
  TRACK_STRAIGHT,
  TRACK_BEND,
  TRACK_SHARP
};

TrackMode track_mode = TRACK_STRAIGHT;

bool corner_mode = false;
bool straight_mode = false;

bool seen_hard_corner = false;

unsigned long last_hard_corner = 0;
unsigned long straight_since = 0;

// Prevent acceleration between sharp turns.
const unsigned long HARD_CORNER_GUARD_MS = 175;

// Confirm centered and stable tracking.
const unsigned long STRAIGHT_CONFIRM_MS = 75;

// ========================================
// FIX 1: BRAKE INTO SHARP TURNS
// ========================================
//
// Setting the PWM to CORNER_SPEED does not
// slow the car at once -- it coasts into the
// turn at straight speed. On entry to a sharp
// turn, push the base PWM below CORNER_SPEED
// for a moment so the wheels actually brake.
//
// Brake time = (how many PWM too fast) *
// ENTRY_BRAKE_MS_PER_PWM, up to the maximum.
// Set ENTRY_BRAKE_MS_PER_PWM to 0 to disable.

// Negative = reverse thrust while braking.
const int ENTRY_BRAKE_BASE = -60;
const float ENTRY_BRAKE_MS_PER_PWM = 1.5f;
const unsigned long ENTRY_BRAKE_MAX_MS = 150;

// FIX 4: brake less often.
// Only brake when the car is at least this
// many PWM faster than CORNER_SPEED, so it
// fires coming off a straight but not on
// every wobble inside a turn.
const float ENTRY_BRAKE_MIN_EXCESS = 30.0f;

// After one brake, wait this long before
// another is allowed.
const unsigned long ENTRY_BRAKE_REARM_MS = 400;

unsigned long last_entry_brake = 0;

// FIX 5: recognize straights more easily.
// The car counts as "on a straight" while
// the error stays under STRAIGHT_MAX_ERROR.
// A blip shorter than STRAIGHT_GRACE_MS no
// longer restarts the confirmation timer.
const float STRAIGHT_MAX_ERROR = 0.55f;

// FIX 8: to ENTER straight mode the error
// must be under this (tighter) limit...
const float STRAIGHT_ENTER_ERROR = 0.40f;

// ...and this long must have passed since
// the last sharp-turn reading.
const unsigned long STRAIGHT_AFTER_SHARP_MS = 400;
const unsigned long STRAIGHT_GRACE_MS = 15;

unsigned long last_stable = 0;

// How slowly the real speed follows the PWM.
const float SPEED_LAG_MS = 120.0f;

// FIX 9: reverse thrust added while the line
// is lost and the car is still moving fast.
// Set to 0 to disable.
const int LOST_BRAKE_PWM = 130;

// FIX 10: early sharp-turn trigger at speed.
// "Fast" = this many PWM above CORNER_SPEED.
const float FAST_SHARP_EXCESS = 60.0f;
const float FAST_SHARP_ERROR = 0.65f;
const int FAST_SHARP_LOOPS = 3;

int fast_count = 0;

// FIX 11: straight mode is judged on the
// AVERAGED error (slow_error), compared with
// STRAIGHT_ENTER_ERROR / STRAIGHT_MAX_ERROR.
// A wiggle around the line no longer blocks
// it. One reading may still be this far off.
const float SLOW_ERROR_S = 0.06f;
const float STRAIGHT_SPIKE_ERROR = 1.00f;

float slow_error = 0.0f;
bool slow_valid = false;

float lagged_base = 60.0f;
unsigned long entry_brake_until = 0;

// ========================================
// FIX 2: DAMPING THAT ACTUALLY ACTS
// ========================================
//
// The loop runs every few milliseconds, so
// (error - prev_error) is only a few
// hundredths per loop. Kd = 25 times that is
// a couple of PWM against a P term of 100+.
// That is no damping, so the car overshoots
// when it swings back and weaves wider each
// time it crosses the line.
//
// error_rate is the same derivative measured
// in sensor-widths per SECOND, so it does not
// depend on how fast the loop runs.

// PWM per (sensor-width per second).
const float KD_RATE_SHARP = 6.0f;
const float KD_RATE_BEND = 4.5f;
const float KD_RATE_STRAIGHT = 4.0f;

// Largest steering correction on a straight.
const float STRAIGHT_CORRECTION_CAP = 60.0f;

// Largest PWM the damping term may add.
const float DAMPING_CAP = 110.0f;

// ========================================
// FIX 6: MOTOR TRIM
// ========================================
//
// The two motors are never equally strong.
// To go straight the car needs a constant
// steering correction, and with P-only
// steering that means riding a constant
// error. At higher speed the error grows,
// sits right at the STRAIGHT/BEND boundary,
// and the car flickers between the two
// modes instead of accelerating.
//
// steer_trim slowly learns that constant
// correction on straights, as a fraction of
// the base speed, so the error returns to
// zero.

// Fraction of base PWM learned per second,
// per sensor-width of error.
const float TRIM_GAIN = 0.30f;

// Largest trim: 0.25 = one motor up to about
// 40% weaker than the other.
const float TRIM_MAX = 0.25f;

float steer_trim = 0.0f;

// Low-pass time constant for error_rate.
const float RATE_FILTER_S = 0.012f;

float error_rate = 0.0f;
float rate_prev_error = 0.0f;
unsigned long last_rate_us = 0;
bool rate_valid = false;

// ========================================
// FIX 3: SENSOR NOISE FLOOR
// ========================================
//
// Readings within contrast / NOISE_FLOOR_DIV
// of the whitest sensor count as white.
// Set to 0 to disable.

const int NOISE_FLOOR_DIV = 4;

// ========================================
// U-TURN SETTINGS
// ========================================

// Keep the calibrated 360-tick turn.
const unsigned long UTURN_TICKS = 360;
const bool TURN_TEST_ONLY = false;

const int UTURN_FAST_PWM = 85;
const int UTURN_MED_PWM = 75;
const int UTURN_SLOW_PWM = 65;

const unsigned long BRAKE_MS = 100;

// ========================================
// FIX 12: BRAKE AT THE BLACK BLOCK
// ========================================
//
// The old code only cut the motors at the
// block and let the car coast, which is fine
// at 100 PWM but not when it arrives in
// straight mode. Now both wheels get reverse
// thrust until the encoders show the car has
// stopped (or the time limit runs out). Then
// the usual pause and U-turn follow.

const int MARKER_BRAKE_PWM = 150;
const unsigned long MARKER_BRAKE_MAX_MS = 250;

// Only brake if arriving faster than this.
const float MARKER_BRAKE_MIN_SPEED = 70.0f;

// Encoder check: every WINDOW ms, the car is
// "stopped" if both wheels together moved
// this many ticks or fewer.
const unsigned long MARKER_BRAKE_WINDOW_MS = 15;
const unsigned long MARKER_BRAKE_STOP_TICKS = 2;

bool marker_braking = false;
bool stop_after_brake = false;
unsigned long marker_brake_start = 0;
unsigned long brake_window_start = 0;
unsigned long brake_window_ticks = 0;
unsigned long brake_last_moved = 0;
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

// 0 = normal finish
// 1 = encoder target reached
// 2 = encoder failure
// 3 = U-turn timeout
// 4 = post-turn line not found
int stop_reason = 0;

// ========================================
// ORIGINAL WORKING PD GAINS
// ========================================

float prev_error = 0;
float filtered_derivative = 0;

const float Kp = 45.0f;
const float Ksharp = 30.0f;
const float Kd = 25.0f;

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
  if (state == FOLLOWING) {
    digitalWrite(
        LED_BUILTIN,
        straight_mode ? HIGH : LOW
    );

    return;
  }

  if (state != STOPPED) {
    digitalWrite(LED_BUILTIN, LOW);
    return;
  }

  unsigned long now = millis();

  if (stop_reason == 1) {
    digitalWrite(LED_BUILTIN, HIGH);
  }
  else if (stop_reason == 2) {
    // Fast blinking = encoder failure.
    digitalWrite(
        LED_BUILTIN,
        (now / 150) % 2
    );
  }
  else if (stop_reason == 3 ||
           stop_reason == 4) {
    // Slow blinking = timeout/error.
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

    // Reacquire slowly after U-turn.
    corner_mode = true;
    track_mode = TRACK_SHARP;

    straight_mode = false;
    straight_since = 0;

    seen_hard_corner = true;
    last_hard_corner = now;

    rate_valid = false;
    slow_valid = false;

    // The car has been turning in place.
    lagged_base = 48;
    entry_brake_until = 0;

    smooth_base_spd = 48;
    last_speed_step = now;

    return;
  }

  // ======================================
  // ENCODER FAILURE
  // ======================================

  if (elapsed > 750 &&
      (l < 5 || r < 5)) {
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
  // WHEEL SYNCHRONIZATION
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
  // BRAKING BEFORE U-TURN
  // ======================================

  if (state == BRAKING) {
    // FIX 12: reverse thrust until stopped.
    if (marker_braking) {
      left_spd = -MARKER_BRAKE_PWM;
      right_spd = -MARKER_BRAKE_PWM;

      if (now - brake_window_start >=
          MARKER_BRAKE_WINDOW_MS) {
        unsigned long l, r;
        read_encoder_counts(l, r);

        unsigned long moved =
            (l + r) - brake_window_ticks;

        brake_window_ticks = l + r;
        brake_window_start = now;

        // Stopped, or already speeding up
        // again in reverse.
        if (moved <= MARKER_BRAKE_STOP_TICKS ||
            moved > brake_last_moved + 1) {
          marker_braking = false;
        }

        brake_last_moved = moved;
      }

      if (now - marker_brake_start >=
          MARKER_BRAKE_MAX_MS) {
        marker_braking = false;
      }

      if (!marker_braking) {
        // Start the normal pause from here.
        brake_start = now;
        stop_wheels();
      }

      return;
    }

    stop_wheels();

    if (now - brake_start >= BRAKE_MS) {
      if (stop_after_brake) {
        state = STOPPED;
        return;
      }

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

  // FIX 3: ignore the white background.
  // The white sensors never read exactly
  // the same, so their leftovers pull the
  // average back toward the middle -- most
  // strongly when the line is at the edge
  // of the array and about to be lost.
  int floor_value =
      NOISE_FLOOR_DIV > 0
          ? contrast / NOISE_FLOOR_DIV
          : 0;

  for (int i = 0; i < 8; i++) {
    int value =
        sensorValues[i] - min_value - floor_value;

    if (value < 0) {
      value = 0;
    }

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

  // ======================================
  // POST-TURN REACQUISITION
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
    rate_valid = false;
    slow_valid = false;

    smooth_base_spd = 48;
    last_speed_step = now;
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

    if (min_value > 1800 &&
        marker_armed) {
      black_count++;

      if (black_count >= 4) {
        black_count = 0;
        clear_count = 0;
        marker_armed = false;

        // FIX 12: arm the reverse-thrust brake
        // if the car is arriving fast.
        marker_braking =
            lagged_base > MARKER_BRAKE_MIN_SPEED;

        if (marker_braking) {
          unsigned long l, r;
          read_encoder_counts(l, r);

          marker_brake_start = now;
          brake_window_start = now;
          brake_window_ticks = l + r;
          brake_last_moved = 0xFFFFFFF0UL;
        }

        if (!returning) {
          state = BRAKING;
          brake_start = now;
          stop_after_brake = false;

          turn_direction =
              (last_direction > 0) ? -1 : 1;

          stop_wheels();
        }
        else if (marker_braking) {
          // Brake first, then stop for good.
          state = BRAKING;
          brake_start = now;
          stop_after_brake = true;

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

  // Preserve working recovery behavior.
  if (contrast < 250) {
    straight_mode = false;
    straight_since = 0;

    unsigned long lost_dt =
        now - last_speed_step;

    if (lost_dt > 80) {
      lost_dt = 80;
    }

    smooth_base_spd = 55;
    last_speed_step = now;

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

    // FIX 9: the line was lost while the car
    // was still moving fast. Add reverse
    // thrust to the recovery until the speed
    // estimate has come down, so the car
    // does not coast far off the track.
    if (lagged_base > (float)CORNER_SPEED) {
      left_spd -= LOST_BRAKE_PWM;
      right_spd -= LOST_BRAKE_PWM;
    }

    prev_error = 0;
    filtered_derivative = 0;

    // The line will reappear at a new place.
    // Do not treat that jump as a slope.
    rate_valid = false;
    slow_valid = false;

    // FIX 1: recovery is slow or a spin in
    // place. Let the speed estimate fall so
    // the brake does not fire again when the
    // line is found.
    lagged_base +=
        (0.5f * (left_spd + right_spd) -
         lagged_base) *
        ((float)lost_dt /
         (SPEED_LAG_MS + (float)lost_dt));

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

  // Is the measured line moving farther
  // away from the sensor array center?
  bool outward =
      error * filtered_derivative > 0.0f;

  // ======================================
  // FIX 2: TIME-BASED ERROR RATE
  // ======================================

  unsigned long now_us = micros();

  float dt_s =
      (now_us - last_rate_us) * 1.0e-6f;

  last_rate_us = now_us;

  if (!rate_valid ||
      dt_s <= 0.0f ||
      dt_s > 0.05f) {
    // First sample after a reset or a gap.
    error_rate = 0.0f;
    rate_valid = true;
  }
  else {
    float raw_rate =
        (error - rate_prev_error) / dt_s;

    float alpha =
        dt_s / (RATE_FILTER_S + dt_s);

    error_rate +=
        alpha * (raw_rate - error_rate);
  }

  rate_prev_error = error;

  // FIX 11: a slow (averaged) copy of the
  // error. Fast wiggle and sensor noise
  // average out of it; a real bend does not.
  if (!slow_valid) {
    slow_error = error;
    slow_valid = true;
  }
  else if (dt_s > 0.0f && dt_s < 0.05f) {
    slow_error +=
        (error - slow_error) *
        (dt_s / (SLOW_ERROR_S + dt_s));
  }

  float abs_slow = fabsf(slow_error);

  // ======================================
  // DISCRETE TRACK MODE DETECTION
  // ======================================

  // FIX 10: at straight-line speed there is
  // no time to wait for a big error. If the
  // car is moving fast and the line moves
  // this far off center, treat it as a
  // sharp turn right away.
  bool fast_now =
      lagged_base >
      (float)CORNER_SPEED + FAST_SHARP_EXCESS;

  // FIX 11: it must hold for a few loops in
  // a row, so one noisy reading on a
  // straight cannot fire the brake.
  if (fast_now &&
      abs_error >= FAST_SHARP_ERROR) {
    if (fast_count < 100) {
      fast_count++;
    }
  }
  else {
    fast_count = 0;
  }

  bool sharp_now =
      abs_error >= 1.20f ||
      fast_count >= FAST_SHARP_LOOPS ||
      (
        abs_error >= 0.85f &&
        outward &&
        fabsf(filtered_derivative) > 0.25f
      );

  bool bend_now =
      abs_error >= 0.55f ||
      (
        abs_error >= 0.32f &&
        outward &&
        fabsf(filtered_derivative) > 0.12f
      );

  // FIX 1: arriving fast at a sharp turn.
  // Brake for a time that grows with how
  // much too fast the car is.
  if (sharp_now &&
      track_mode != TRACK_SHARP) {
    float excess =
        lagged_base - (float)CORNER_SPEED;

    // FIX 4: only when really too fast, and
    // not again right after the last brake.
    bool rearmed =
        last_entry_brake == 0 ||
        now - last_entry_brake >=
        ENTRY_BRAKE_REARM_MS;

    if (excess > ENTRY_BRAKE_MIN_EXCESS &&
        rearmed) {
      last_entry_brake = now;

      unsigned long brake_ms =
          (unsigned long)(
              excess * ENTRY_BRAKE_MS_PER_PWM
          );

      if (brake_ms > ENTRY_BRAKE_MAX_MS) {
        brake_ms = ENTRY_BRAKE_MAX_MS;
      }

      entry_brake_until = now + brake_ms;
    }
  }

  if (sharp_now) {
    track_mode = TRACK_SHARP;

    seen_hard_corner = true;
    last_hard_corner = now;
  }
  else if (bend_now) {
    track_mode = TRACK_BEND;
  }
  else {
    track_mode = TRACK_STRAIGHT;
  }

  // Used by the original recovery behavior.
  corner_mode =
      track_mode != TRACK_STRAIGHT;

  // ======================================
  // THREE STEERING STRENGTHS
  // ======================================

  float correction;
  float correction_cap;
  float kd_rate;

  if (track_mode == TRACK_STRAIGHT) {
    // Gentle steering.
    // Avoid large changes for tiny errors.
    correction =
        23.0f * error +
        14.0f * filtered_derivative;

    correction_cap = STRAIGHT_CORRECTION_CAP;
    kd_rate = KD_RATE_STRAIGHT;
  }
  else if (track_mode == TRACK_BEND) {
    // Moderate steering for gradual bends.
    correction =
        35.0f * error +
        18.0f * error * abs_error +
        20.0f * filtered_derivative;

    correction_cap = 105.0f;
    kd_rate = KD_RATE_BEND;
  }
  else {
    // Original successful sharp-turn gains.
    correction =
        Kp * error +
        Ksharp * error * abs_error +
        Kd * filtered_derivative;

    kd_rate = KD_RATE_SHARP;

    correction_cap =
        abs_error > 2.1f ? 190.0f : 160.0f;
  }

  // FIX 2: damping. Opposes the swing of the
  // line across the sensor, so the car stops
  // rotating as it comes back to center
  // instead of overshooting to the other side.
  float damping = kd_rate * error_rate;

  damping = constrain(
      damping,
      -DAMPING_CAP,
      DAMPING_CAP
  );

  correction += damping;

  correction = constrain(
      correction,
      -correction_cap,
      correction_cap
  );

  // FIX 6: learn the motor trim, but only
  // on straights and only from small errors,
  // so turns do not disturb it.
  if (track_mode == TRACK_STRAIGHT &&
      dt_s > 0.0f &&
      dt_s < 0.05f) {
    steer_trim += TRIM_GAIN * error * dt_s;

    steer_trim = constrain(
        steer_trim,
        -TRIM_MAX,
        TRIM_MAX
    );
  }

  // ======================================
  // CONFIRM GENUINE STRAIGHT
  // ======================================

  // FIX 7: anything short of a sharp turn
  // counts, as long as the error is inside
  // STRAIGHT_MAX_ERROR. A one-loop flicker
  // into BEND no longer blocks it.
  // FIX 8: getting INTO straight mode is
  // harder than staying in it, and it is
  // not allowed right after a sharp turn,
  // when the car is still settling.
  float straight_limit =
      straight_mode
          ? STRAIGHT_MAX_ERROR
          : STRAIGHT_ENTER_ERROR;

  bool settled_after_sharp =
      !seen_hard_corner ||
      now - last_hard_corner >=
      STRAIGHT_AFTER_SHARP_MS;

  bool stable =
      track_mode != TRACK_SHARP &&
      settled_after_sharp &&
      contrast > 350 &&
      abs_slow < straight_limit &&
      abs_error < STRAIGHT_SPIKE_ERROR;

  if (stable) {
    last_stable = now;

    if (straight_since == 0) {
      straight_since = now;
    }

    straight_mode =
        now - straight_since >=
        STRAIGHT_CONFIRM_MS;
  }
  else if (straight_since != 0 &&
           track_mode != TRACK_SHARP &&
           now - last_stable <
           STRAIGHT_GRACE_MS) {
    // FIX 5: a short blip. Keep the timer
    // and the current straight_mode.
  }
  else {
    straight_since = 0;
    straight_mode = false;
  }

  // ======================================
  // DISCRETE SPEED CONTROL
  // ======================================

  int target_base;

  if (track_mode == TRACK_SHARP) {
    // Immediate slowdown for sharp corners.
    target_base = CORNER_SPEED;
  }
  else if (straight_mode) {
    // FIX 7: confirmed straight wins over a
    // momentary BEND, so one noisy reading
    // cannot knock the speed down.
    target_base = STRAIGHT_SPEED;
  }
  else if (track_mode == TRACK_BEND) {
    target_base = BEND_SPEED;

    // Brake earlier if error is increasing.
    if (outward && abs_error > 0.42f) {
      target_base = EARLY_BRAKE_SPEED;
    }
  }
  else {
    // Centered but not fully stabilized.
    target_base = CRUISE_SPEED;
  }

  // ======================================
  // PROTECT CONSECUTIVE SHARP TURNS
  // ======================================

  bool hard_guard =
      seen_hard_corner &&
      now - last_hard_corner <
      HARD_CORNER_GUARD_MS;

  if (hard_guard &&
      track_mode != TRACK_SHARP) {
    target_base = min(
        target_base,
        HARD_EXIT_SPEED
    );
  }

  // A rapid change in error means the
  // robot may be crossing the centerline.
  // Avoid full acceleration at that instant.
  if (fabsf(filtered_derivative) > 0.22f) {
    target_base = min(
        target_base,
        CRUISE_SPEED
    );
  }

  // FIX 1: while the entry brake is on,
  // nothing may speed the car back up.
  bool entry_braking =
      now < entry_brake_until;

  if (entry_braking) {
    target_base = min(
        target_base,
        CORNER_SPEED
    );
  }

  // ======================================
  // TIME-BASED ACCELERATION
  // ======================================

  unsigned long dt =
      now - last_speed_step;

  last_speed_step = now;

  // Ignore very large time gaps.
  if (dt > 80) {
    dt = 80;
  }

  // Immediate commanded deceleration.
  if (smooth_base_spd > target_base) {
    smooth_base_spd =
        (float)target_base;
  }
  else {
    // Accelerate proportionally to time.
    smooth_base_spd = min(
        (float)target_base,
        smooth_base_spd +
        ACCEL_PWM_PER_MS * dt
    );
  }

  // FIX 1: the brake overrides the base PWM
  // for its short duration only. When it
  // ends, the base goes straight back to
  // smooth_base_spd -- no slow ramp up from
  // a negative value.
  float base_cmd = smooth_base_spd;

  if (entry_braking) {
    base_cmd = (float)ENTRY_BRAKE_BASE;
  }

  // The car's real speed lags the command.
  // lagged_base follows the command slowly,
  // as a rough stand-in for how fast the car
  // is actually moving.
  lagged_base +=
      (base_cmd - lagged_base) *
      ((float)dt / (SPEED_LAG_MS + (float)dt));

  // ======================================
  // FINAL MOTOR COMMANDS
  // ======================================

  // FIX 6: the trim scales with speed and is
  // added after the correction cap.
  float trim_pwm = 0.0f;

  if (base_cmd > 0.0f) {
    trim_pwm = steer_trim * base_cmd;
  }

  left_spd = constrain(
      (int)(base_cmd - correction - trim_pwm),
      -mx_spd,
      mx_spd
  );

  right_spd = constrain(
      (int)(base_cmd + correction + trim_pwm),
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

  // Important: preserve working order.
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

  last_speed_step = millis();

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