#include <Wire.h>
#include <MPU6050.h>

// Define the two MPU6050 objects with their respective I2C addresses
MPU6050 imu1(0x68); // Replace 0x68 with the I2C address of the first MPU6050
MPU6050 imu2(0x69); // Replace 0x69 with the I2C address of the second MPU6050

// Pins for the motor control
int in3 = 4;
int in4 = 5;
const int motorIn1 = 6;
const int motorIn2 = 7;
int in1 = 9;
int in2 = 10;
const int motorIn3 = 11;
const int motorIn4 = 12;

// Variables for PID control
float kp = 10;
float ki = 0;
float kd = 0;
unsigned long currentTime, previousTime;
float elapsedTime;
float error;
float lastError;
float input, output, setPoint;
float cumpError, rateError;

void motor(float PWM) {
  if (PWM >= 0) {
    analogWrite(motorIn1, abs(PWM));
    analogWrite(motorIn2, 0);
  } 
}
void motor2(float PWM) {
  if (PWM >= 0) {
    analogWrite(motorIn3, abs(PWM));
    analogWrite(motorIn4, 0);
  } 
}

void setup() {
  setPoint = 30; // Set the setpoint at zero degrees (you may adjust this value)
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  digitalWrite(4, LOW);
  digitalWrite(5, HIGH);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  digitalWrite(9, LOW);
  digitalWrite(10, HIGH);

  Wire.begin();
  imu1.initialize();
  imu2.initialize();

  Serial.begin(115200);
  pinMode(motorIn1, OUTPUT);
  pinMode(motorIn3, OUTPUT);
}
float calculateAngle(int16_t ax, int16_t ay, int16_t az) {
  // Compute the angle from accelerometer data
  // Replace these formulas with the appropriate calculation for your specific orientation
  float angleX = atan2(ay, az) * -RAD_TO_DEG;
  float angleY = atan2(ax, az) * -RAD_TO_DEG;

  // Assuming the IMU is oriented correctly, you may choose to return only one of the angles
  // For example, if you want to measure the pitch angle, return 'angleX'
  return angleX;
}

void loop() {
  // Read data from the first IMU (imu1)
  int16_t ax1, ay1, az1;
  imu1.getAcceleration(&ax1, &ay1, &az1);

  // Read data from the second IMU (imu2)
  int16_t ax2, ay2, az2;
  imu2.getAcceleration(&ax2, &ay2, &az2);

  // Calculate angles from accelerometer data for both IMUs
  float angle1 = calculateAngle(ax1, ay1, az1);
  float angle2 = calculateAngle(ax2, ay2, az2);



  // Calculate the difference between angles (input) and apply PID control
  input = angle2 - angle1;
  output = computePID(input);

  // Cap the output to avoid excessive motor control
  if (output > 255) {
    output = 255;
  } else if (output < 80) {
    output = 80;
  }

  // Control the motor based on the PID output
  motor(output);
  motor2(output);
  
  // Print the angles (you can process or use the angles as needed)
  Serial.print("Angle 1:");
  Serial.print(angle1);
  Serial.print(",");
  Serial.print("Angle 2:");
  Serial.println(angle2);
  Serial.print(0); // To freeze the lower limit
  Serial.print(" ");
  Serial.print(1000); // To freeze the upper limit
  Serial.print(" ");
// Print the PID output to the serial monitor
//  Serial.print("\tPID Output:");
 // Serial.print(output);
//  Serial.print("\terror:");
//  Serial.println(error);

  delay(10);
}



float computePID(float inp) {
  currentTime = millis(); // Get current time
  elapsedTime = (float)(currentTime - previousTime); // Compute time elapsed from previous computation

  error = setPoint - inp; // Determine error
  cumpError += error * elapsedTime; // Compute integral
  rateError = (error - lastError) / elapsedTime; // Compute derivative

  float out = kp * error + ki * cumpError + kd * rateError; // PID output

  lastError = error; // Remember current error
  previousTime = currentTime; // Remember current time

  // Cap the output to avoid excessive control
  if (out > 255) {
    out = 255;
  } else if (out < -255) {
    out = -255;
  }

  if (error == 0) {
    out = 0;
  }

  return out; // Return the PID output
}
