// PWM Duty Cycle Experiment

#include <ECE3.h>

// Pin Assignments
const int left_nslp_pin = 31;
const int left_dir_pin = 29;
const int left_pwm_pin = 40;
const int right_nslp_pin = 31;
const int right_dir_pin = 30;
const int right_pwm_pin = 39;

uint16_t sensorValues[8];

// Motor Speed
int left_spd;
int right_spd;
int mid_loc = 4;
int mx_spd = 254;

float prev_error = 0;

// Encoder Variables
long enc_bin_cnt;
const unsigned long enc_bin_len = 50;

// Serial Communication
char new_duty_cycle[4];

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
  int min_sensor_value = sensorValues[0];

  for (int i = 1; i < 8; i++) {
    min_sensor_value = min(min_sensor_value, (int)sensorValues[i]);
  }

  long weighted_sum = 0;
  long total = 0;

  for (int i = 0; i < 8; i++) {
    int value = sensorValues[i] - min_sensor_value;
    weighted_sum += (long)value;
    total += value;
  }

  if (total == 0) {
    left_spd = 0;
    right_spd = 0;
    prev_error = 0;
    return;
  }

  float position = (float)weighted_sum / total;
  float error = position - 3.5;

  const int base_spd = 45;
  const float Kp = 20.0;
  const float Kd = 10.0;

  float correction = Kp * error + Kd * (error - prev_error);
  prev_error = error;

  left_spd = constrain((int)(base_spd - correction), -mx_spd, mx_spd);
  right_spd = constrain((int)(base_spd + correction), -mx_spd, mx_spd);
}

void setup() {
  // Left motor
  pinMode(left_nslp_pin, OUTPUT);
  pinMode(left_dir_pin, OUTPUT);
  pinMode(left_pwm_pin, OUTPUT);

  digitalWrite(left_dir_pin, LOW);
  digitalWrite(left_nslp_pin, HIGH);

  // Right motor
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
  static unsigned long last_update = 0;

  if (millis() - last_update < 10) return;
  last_update = millis();

  ECE3_read_IR(sensorValues);
  set_new_wheel_spd();
  assign_wheels();
}