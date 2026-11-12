
#include <ECE3.h>

// Pin Assignments
const int left_nslp_pin = 31;
const int left_dir_pin = 29;
const int left_pwm_pin = 40;
const int right_nslp_pin = 11;
const int right_dir_pin = 30;
const int right_pwm_pin = 39;

uint16_t sensorValues[8];

int left_spd;
int right_spd;
int mid_loc = 4;
int mx_spd = 254;

long enc_bin_cnt;
const unsigned long enc_bin_len = 50;

char new_duty_cycle[4];

// U-turn settings (NEW)
const int UTURN_PWM = 80;
const unsigned long UTURN_MS = 700;

int abso(int x) {
  return (x < 0 ? -x : x);
}

void assign_wheels() {
  if (left_spd < 0) {
    digitalWrite(left_dir_pin, HIGH);
  } else {
    digitalWrite(left_dir_pin, LOW);
  }

  if (right_spd < 0) {
    digitalWrite(right_dir_pin, HIGH);
  } else {
    digitalWrite(right_dir_pin, LOW);
  }

  analogWrite(left_pwm_pin, abso(left_spd));
  analogWrite(right_pwm_pin, abso(right_spd));
}

void inc_spd(bool right, bool inc) {
  if ((right && inc) || ((!right) && (!inc))) {
    if (abso(right_spd) < mx_spd) {
      right_spd++;
    } else if (abso(left_spd) < mx_spd) {
      left_spd--;
    }
  }

  if ((right && (!inc)) || ((!right) && inc)) {
    if (abso(left_spd) < mx_spd) {
      left_spd++;
    } else if (abso(right_spd) < mx_spd) {
      right_spd--;
    }
  }
}

void set_new_wheel_spd() {
  static float prev_error = 0;
  static float filtered_derivative = 0;
  static int last_direction = 1;

  // Finish line detection
  static bool finished = false;
  static int black_count = 0;

  // U-turn state (NEW)
  static bool turning = false;
  static bool returning = false;
  static bool marker_armed = false;
  static int clear_count = 0;
  static unsigned long turn_start = 0;

  if (finished) {
    left_spd = 0;
    right_spd = 0;
    return;
  }

  // Execute U-turn (NEW)
  if (turning) {
    if (millis() - turn_start < UTURN_MS) {
      left_spd = UTURN_PWM;
      right_spd = -UTURN_PWM;
    } else {
      turning = false;
      returning = true;
      prev_error = 0;
      filtered_derivative = 0;
      left_spd = 0;
      right_spd = 0;
    }
    return;
  }

  int min_sensor_value = sensorValues[0];
  int max_sensor_value = sensorValues[0];

  for (int i = 1; i < 8; i++) {
    min_sensor_value = min(min_sensor_value, (int)sensorValues[i]);
    max_sensor_value = max(max_sensor_value, (int)sensorValues[i]);
  }

  // Arm marker detection only after leaving a black marker (NEW)
  if (min_sensor_value <= 1800) {
    clear_count++;

    if (clear_count >= 10) {
      marker_armed = true;
    }
  } else {
    clear_count = 0;
  }

  // First black marker: U-turn
  // Second black marker: stop permanently (NEW)
  if (min_sensor_value > 1800 && marker_armed) {
    black_count++;

    if (black_count >= 4) {
      black_count = 0;
      marker_armed = false;
      clear_count = 0;

      if (!returning) {
        turning = true;
        turn_start = millis();
        left_spd = UTURN_PWM;
        right_spd = -UTURN_PWM;
      } else {
        finished = true;
        left_spd = 0;
        right_spd = 0;
      }
      return;
    }
  } else {
    black_count = 0;
  }

  // Line lost or uniform sensor readings
  if (max_sensor_value - min_sensor_value < 250) {
    if (max_sensor_value > 1500) {
      left_spd = 55;
      right_spd = 55;
    } else {
      left_spd = -last_direction * 55;
      right_spd = last_direction * 55;
    }

    prev_error = 0;
    filtered_derivative = 0;
    return;
  }

  long weighted_sum = 0;
  long total = 0;

  for (int i = 0; i < 8; i++) {
    int value = sensorValues[i] - min_sensor_value;

    weighted_sum += (long)value * i;
    total += value;
  }

  if (total == 0) {
    left_spd = 0;
    right_spd = 0;
    return;
  }

  float position = (float)weighted_sum / total;
  float error = position - 3.5;

  if (error > 0.2) last_direction = 1;
  if (error < -0.2) last_direction = -1;

  // PD controller - original working gains
  const float Kp = 90.0;
  const float Ksharp = 15.0;
  const float Kd = 18.0;

  // Damping
  float derivative = error - prev_error;

  filtered_derivative =
      0.6 * filtered_derivative + 0.4 * derivative;

  float correction =
      Kp * error +
      Ksharp * error * fabsf(error) +
      Kd * filtered_derivative;

  // Stronger correction on sharp turns
  float max_correction = 140.0;

  if (fabsf(error) > 2.0) {
    max_correction = 180.0;
  }

  correction = constrain(
      correction, -max_correction, max_correction
  );

  // Lower overall forward speed
  int base_spd = constrain(
      (int)(55 - 3 * fabsf(error)), 45, 55
  );

  left_spd = constrain(
      (int)(base_spd - correction), -mx_spd, mx_spd
  );

  right_spd = constrain(
      (int)(base_spd + correction), -mx_spd, mx_spd
  );

  prev_error = error;
}

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

  left_spd = 20;
  right_spd = 20;

  ECE3_Init();

  Serial.begin(9600);

  delay(2000);

  analogWrite(left_pwm_pin, left_spd);
  analogWrite(right_pwm_pin, right_spd);
}

void loop() {
  ECE3_read_IR(sensorValues);
  set_new_wheel_spd();
  assign_wheels();
}
