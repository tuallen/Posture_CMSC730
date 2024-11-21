#include <Arduino.h>
#include <Wire.h>

// IMU Addresses
const int MPU9250_ADDRESS_1 = 0x68;
const int MPU9250_ADDRESS_2 = 0x69;
const int sda = 21;
const int scl = 22;

// Sensor variables
int16_t accelX_1, accelY_1, accelZ_1;
int16_t gyroX_1, gyroY_1, gyroZ_1;
int16_t accelX_2, accelY_2, accelZ_2;
int16_t gyroX_2, gyroY_2, gyroZ_2;

// Madgwick filter variables
float beta = 0.1f;    // Filter gain
float sampleFreq = 100.0f; // Sample frequency in Hz
float deltaT = 1.0f / sampleFreq;

// Quaternions for each sensor
float q0_1 = 1.0f, q1_1 = 0.0f, q2_1 = 0.0f, q3_1 = 0.0f; // Sensor 1
float q0_2 = 1.0f, q1_2 = 0.0f, q2_2 = 0.0f, q3_2 = 0.0f; // Sensor 2

// Function prototypes
void initMPU9250(int address);
void readSensorData(int address, int16_t &accelX, int16_t &accelY, int16_t &accelZ, 
                    int16_t &gyroX, int16_t &gyroY, int16_t &gyroZ);
void madgwickUpdate(float gx, float gy, float gz, float ax, float ay, float az,
                    float &q0, float &q1, float &q2, float &q3);

void setup() {
  Wire.begin(sda, scl);
  Serial.begin(115200);
  
  // Initialize both sensors
  initMPU9250(MPU9250_ADDRESS_1);
  initMPU9250(MPU9250_ADDRESS_2);
}

void loop() {
  // Read data from both sensors
  readSensorData(MPU9250_ADDRESS_1, accelX_1, accelY_1, accelZ_1, gyroX_1, gyroY_1, gyroZ_1);
  readSensorData(MPU9250_ADDRESS_2, accelX_2, accelY_2, accelZ_2, gyroX_2, gyroY_2, gyroZ_2);
  
  // Convert to real values (adjust these based on your sensor's full-scale settings)
  float gyroScale = 1.0f / 131.0f; // For ±250 deg/s
  float accelScale = 1.0f / 16384.0f; // For ±2g

  // Process sensor 1 data
  float gx1 = gyroX_1 * gyroScale;
  float gy1 = gyroY_1 * gyroScale;
  float gz1 = gyroZ_1 * gyroScale;
  float ax1 = accelX_1 * accelScale;
  float ay1 = accelY_1 * accelScale;
  float az1 = accelZ_1 * accelScale;
  
  // Process sensor 2 data
  float gx2 = gyroX_2 * gyroScale;
  float gy2 = gyroY_2 * gyroScale;
  float gz2 = gyroZ_2 * gyroScale;
  float ax2 = accelX_2 * accelScale;
  float ay2 = accelY_2 * accelScale;
  float az2 = accelZ_2 * accelScale;

  // Update orientation estimates using Madgwick filter
  madgwickUpdate(gx1, gy1, gz1, ax1, ay1, az1, q0_1, q1_1, q2_1, q3_1);
  madgwickUpdate(gx2, gy2, gz2, ax2, ay2, az2, q0_2, q1_2, q2_2, q3_2);

  // Print quaternions with 6 decimal places for precision
  Serial.print("S1,");
  Serial.print(q0_1, 6); Serial.print(",");
  Serial.print(q1_1, 6); Serial.print(",");
  Serial.print(q2_1, 6); Serial.print(",");
  Serial.print(q3_1, 6); Serial.print(",");
  
  Serial.print("S2,");
  Serial.print(q0_2, 6); Serial.print(",");
  Serial.print(q1_2, 6); Serial.print(",");
  Serial.print(q2_2, 6); Serial.print(",");
  Serial.println(q3_2, 6);
  
  delay(int(deltaT * 1000)); // Maintain sample frequency
}

void initMPU9250(int address) {
  Wire.beginTransmission(address);
  Wire.write(0x6B);  // Power management register
  Wire.write(0);     // Wake up the sensor
  Wire.endTransmission();
  
  // Configure gyroscope range (±250 deg/s)
  Wire.beginTransmission(address);
  Wire.write(0x1B);
  Wire.write(0x00);
  Wire.endTransmission();
  
  // Configure accelerometer range (±2g)
  Wire.beginTransmission(address);
  Wire.write(0x1C);
  Wire.write(0x00);
  Wire.endTransmission();
}

void readSensorData(int address, int16_t &accelX, int16_t &accelY, int16_t &accelZ, 
                    int16_t &gyroX, int16_t &gyroY, int16_t &gyroZ) {
  // Request accelerometer data
  Wire.beginTransmission(address);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(address, 6);
  
  if (Wire.available() == 6) {
    accelX = Wire.read() << 8 | Wire.read();
    accelY = Wire.read() << 8 | Wire.read();
    accelZ = Wire.read() << 8 | Wire.read();
  }
  
  // Request gyroscope data
  Wire.beginTransmission(address);
  Wire.write(0x43);
  Wire.endTransmission(false);
  Wire.requestFrom(address, 6);
  
  if (Wire.available() == 6) {
    gyroX = Wire.read() << 8 | Wire.read();
    gyroY = Wire.read() << 8 | Wire.read();
    gyroZ = Wire.read() << 8 | Wire.read();
  }
}

void madgwickUpdate(float gx, float gy, float gz, float ax, float ay, float az,
                    float &q0, float &q1, float &q2, float &q3) {
  float recipNorm;
  float s0, s1, s2, s3;
  float qDot1, qDot2, qDot3, qDot4;
  float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2, _8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

  // Convert gyroscope degrees/sec to radians/sec
  gx *= 0.0174533f;
  gy *= 0.0174533f;
  gz *= 0.0174533f;

  // Rate of change of quaternion from gyroscope
  qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
  qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
  qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
  qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

  // Compute feedback only if accelerometer measurement valid
  if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
    // Normalise accelerometer measurement
    recipNorm = 1.0f / sqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    // Auxiliary variables to avoid repeated arithmetic
    _2q0 = 2.0f * q0;
    _2q1 = 2.0f * q1;
    _2q2 = 2.0f * q2;
    _2q3 = 2.0f * q3;
    _4q0 = 4.0f * q0;
    _4q1 = 4.0f * q1;
    _4q2 = 4.0f * q2;
    _8q1 = 8.0f * q1;
    _8q2 = 8.0f * q2;
    q0q0 = q0 * q0;
    q1q1 = q1 * q1;
    q2q2 = q2 * q2;
    q3q3 = q3 * q3;

    // Gradient decent algorithm corrective step
    s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
    s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
    s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
    s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;

    // Normalise step magnitude
    recipNorm = 1.0f / sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
    s0 *= recipNorm;
    s1 *= recipNorm;
    s2 *= recipNorm;
    s3 *= recipNorm;

    // Apply feedback step
    qDot1 -= beta * s0;
    qDot2 -= beta * s1;
    qDot3 -= beta * s2;
    qDot4 -= beta * s3;
  }

  // Integrate rate of change of quaternion to yield quaternion
  q0 += qDot1 * deltaT;
  q1 += qDot2 * deltaT;
  q2 += qDot3 * deltaT;
  q3 += qDot4 * deltaT;

  // Normalise quaternion
  recipNorm = 1.0f / sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 *= recipNorm;
  q1 *= recipNorm;
  q2 *= recipNorm;
  q3 *= recipNorm;
}