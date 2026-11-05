// PWM Duty Cycle Experiment
// 
// This code does the following:
//  - create a PWM signal that runs the left motor
//    - this is set by the variable enc_bin_cnt (set by Serial Input)
//  - calculate the speed of the left motor using the encoder
//    - this measurement is in arbitrary units (details below)
// 

#include <ECE3.h> // Used for encoder functionality

// NOTE: Function descriptions
//       - getEncoderCount_left()
//           returns how many encoder increments the left wheel has turned
//       - resetEncoderCount_left()
//           sets encoder increments to 0

// Pin Assignments
const int left_nslp_pin = 31;
const int left_dir_pin = 29;
const int left_pwm_pin = 40;
const int right_nslp_pin = 11;
const int right_dir_pin = 30;
const int right_pwm_pin = 39;

uint16_t sensorValues[8];

// Motor Speed
int left_spd;
int right_spd;
int mid_loc = 4;
int mx_spd = 254;

// Encoder Variables
long enc_bin_cnt;
const unsigned long enc_bin_len = 50; // 50 ms bins
    // Encoder Speed Calculation Explanation:
    // We wait for a set amount of time (enc_bin_len), and find how many
    // times the encoder has incremented in that period. We call 
    // this period a bin when refering to the encoder. The number 
    // encoder counts per bin is a proportional to speed.

// Serial Communication
char new_duty_cycle[4];

// // Prototypes
// int get_spd(int prev_spd);

int abso(int x) {return (x<0?(-x):x);}

void assign_wheels() {
  if (left_spd<0) {digitalWrite(left_dir_pin,HIGH);}
  else {digitalWrite(left_dir_pin,LOW);}
  if (right_spd<0) {digitalWrite(right_dir_pin,HIGH);}
  else {digitalWrite(right_dir_pin,LOW);}
  analogWrite(left_pwm_pin, abso(left_spd));
  analogWrite(right_pwm_pin, abso(right_spd));
}

void inc_spd(bool right, bool inc) {
  if ((right&&inc)||((!right)&&(!inc))) {
    if (abso(right_spd)<mx_spd) {right_spd++;}
    else if (abso(left_spd)<mx_spd) {left_spd--;}
  }
  if ((right&&(!inc))||((!right)&&inc)) {
    if (abso(left_spd)<mx_spd) {left_spd++;}
    else if (abso(right_spd)<mx_spd) {right_spd--;}
  }
}

// void set_new_wheel_spd() {
//   int max_sensor_location = 0;
//   int max_sensor_value = 0;
//   for (int i = 0; i < 8; i++) {
//     if (sensorValues[i]>max_sensor_value) {max_sensor_value=sensorValues[i]; max_sensor_location=i;}
//   }
//   if (max_sensor_location<mid_loc) {inc_spd(true,true);}
//   if (max_sensor_location>mid_loc) {inc_spd(false,true);}

//   int correction = 5.0 * (max_sensor_location - 3.5);
//   left_spd = constrain(25 - correction, -mx_spd, mx_spd);
//   right_spd = constrain(25 + correction, -mx_spd, mx_spd);

//   // Serial.print(left_spd); Serial.print(" "); Serial.print(right_spd); Serial.println();
// }

//OLD SET NEW WHEEL SPD

// void set_new_wheel_spd() {
//   int min_sensor_value = sensorValues[0];

//   for (int i = 1; i < 8; i++) {
//     min_sensor_value = min(min_sensor_value, (int)sensorValues[i]);
//   }

//   long weighted_sum = 0;
//   long total = 0;

//   for (int i = 0; i < 8; i++) {
//     int value = sensorValues[i] - min_sensor_value;
//     weighted_sum += (long)value * i;
//     total += value;
//   }

//   if (total == 0) {
//     left_spd = 0;
//     right_spd = 0;
//     return;
//   }

//   float position = (float)weighted_sum / total;
//   float error = position - 3.5;

//   const int base_spd = 90;
//   const float Kp = 80.0;

//   float correction = Kp * error;

//   left_spd = constrain((int)(base_spd - correction), -mx_spd, mx_spd);
//   right_spd = constrain((int)(base_spd + correction), -mx_spd, mx_spd);
// }

void set_new_wheel_spd() {
  static float prev_error = 0;
  static float filtered_derivative = 0;
  static int last_direction = 1;

  int min_sensor_value = sensorValues[0];
  int max_sensor_value = sensorValues[0];

  for (int i = 1; i < 8; i++) {
    min_sensor_value = min(min_sensor_value, (int)sensorValues[i]);
    max_sensor_value = max(max_sensor_value, (int)sensorValues[i]);
  }

  // Line lost or uniform sensor readings
  if (max_sensor_value - min_sensor_value < 250) {
    if (max_sensor_value > 1500) {
      // Likely all-black region or intersection
      left_spd = 55;
      right_spd = 55;
    } else {
      // Search toward the last known line position
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

  const float Kp = 18.0;
  const float Ksharp = 15.0;
  const float Kd = 12.0;

  // Damping
  float derivative = error - prev_error;
  filtered_derivative =
      0.6 * filtered_derivative + 0.4 * derivative;

  // Small correction on straights, large on sharp turns
  float correction =
      Kp * error +
      Ksharp * error * abs(error) +
      Kd * filtered_derivative;

  correction = constrain(correction, -140.0, 140.0);

  // Slow down on sharp turns
  int base_spd = constrain(
      (int)(75 - 10 * abs(error)), 45, 75
  );

  left_spd = constrain(
      (int)(base_spd - correction), -mx_spd, mx_spd
  );

  right_spd = constrain(
      (int)(base_spd + correction), -mx_spd, mx_spd
  );

  prev_error = error;
}

///////////////////////////////////
void setup() {
// This function runs once

  // Pin Settings
  pinMode(left_nslp_pin,OUTPUT);
  pinMode(left_dir_pin,OUTPUT);
  pinMode(left_pwm_pin,OUTPUT);

  // Setting Initial Values
  digitalWrite(left_dir_pin,LOW);
  digitalWrite(left_nslp_pin,HIGH);


  pinMode(right_nslp_pin,OUTPUT);
  pinMode(right_dir_pin,OUTPUT);
  pinMode(right_pwm_pin,OUTPUT);

  // Setting Initial Values
  digitalWrite(right_dir_pin,LOW);
  digitalWrite(right_nslp_pin,HIGH);
  left_spd = 20;
  right_spd = 20;
  ECE3_Init(); // Used for encoder functionality

  Serial.begin(9600); // data rate for serial data transmission

  delay(2000); //Wait 2 seconds before starting 

  analogWrite(left_pwm_pin, left_spd);
  analogWrite(right_pwm_pin, right_spd);
}

void loop() {
  // Get the new speed from Serial input
  ECE3_read_IR(sensorValues);
  set_new_wheel_spd();
  assign_wheels();
  // print the sensor values as numbers from 0 to 2500, where 0 means maximum reflectance and
  // 2500 means minimum reflectance
  // for (unsigned char i = 0; i < 8; i++)
  // {
  //   Serial.print(sensorValues[i]);
  //   Serial.print('\t'); // tab to format the raw data into columns in the Serial monitor
  // }
  // Serial.println();

  // delay(500);
  // left_spd = get_spd(left_spd);

  // Set the new speed
  // delay(20);

  // Reset the encoder and count how many ticks
  // occur in enc_bin_len milliseconds.
  // resetEncoderCount_left();
  // delay(enc_bin_len);
  // enc_bin_cnt = getEncoderCount_left();

  // Report encoder counts and speed
  // Serial.print("Encoder ticks: "); Serial.print(enc_bin_cnt);
  // Serial.print(" @ left_spd: "); Serial.print(left_spd);
  // Serial.print("\n");
}

// int get_spd(int prev_spd) {

//   int bytes_avail = Serial.available();
//   if(bytes_avail == 0) // no new serial inputs
//     return prev_spd;

//   else if (bytes_avail > 4) { // invalid serial input

//     do { // eat invalid buffered input
//       delay(100);
//       Serial.read();
//     } while(Serial.available() > 0);  
//     Serial.println("INVALID"); 
//     //Serial.println(bytes_avail);  x
//     return prev_spd;
//   }

//   else {
//     for (int i = 0; i < bytes_avail; i++) // read out buffer
//       new_duty_cycle[i] = Serial.read();
//     int sum = 0;
//     for (int i = 0; i < bytes_avail; i++) { // calculate new speed

//       int num = new_duty_cycle[i] - '0';
//       if (num == -38)
//         break;
//       else if (num > 9 || num < 0) { // invalid character
//         Serial.println("INVALID"); 
//         return prev_spd;
//       }
//       sum += pow(10, bytes_avail - 2 - i) * num;
//     }

//     if (sum >= 255)
//       return 255;
//     else
//       return sum;
//   }
// }