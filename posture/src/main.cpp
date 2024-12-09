#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <ESP32Servo.h>

// Desktop Guy Pins & Servo
int servoPin = 13;
Servo myServo;

// Stretch Sensor Pins
const int stretchPin = 39;
const int relayPin = 19;
int stretchCount = 0;
const int stretchThreshold = 1755;
float x1 = 0, x2 = 0, y1m = 0, y2 = 0, z1 = 0, z2 = 0;
void updateEulerAngles(float w, float x, float y, float z, float &xrot, float &yrot, float &zrot);

// MPU9250 I2C Addresses
const int MPU9250_ADDRESS_1 = 0x68; // Sensor 1 (AD0 = GND)
const int MPU9250_ADDRESS_2 = 0x69; // Sensor 2 (AD0 = VCC)

// I2C Pins
const int SDA_PIN = 21;
const int SCL_PIN = 22;

// Register Addresses
const uint8_t PWR_MGMT_1 = 0x6B;
const uint8_t GYRO_CONFIG = 0x1B;
const uint8_t ACCEL_CONFIG = 0x1C;
const uint8_t ACCEL_XOUT_H = 0x3B;
const uint8_t GYRO_XOUT_H = 0x43;

// Sensor Scale Factors
const float GYRO_SCALE = 1.0f / 131.0f;    // ±250 deg/s
const float ACCEL_SCALE = 1.0f / 16384.0f; // ±2g

// Calibration Offsets
float accelOffsetX_1 = 0, accelOffsetY_1 = 0, accelOffsetZ_1 = 0;
float gyroOffsetX_1 = 0, gyroOffsetY_1 = 0, gyroOffsetZ_1 = 0;
float accelOffsetX_2 = 0, accelOffsetY_2 = 0, accelOffsetZ_2 = 0;
float gyroOffsetX_2 = 0, gyroOffsetY_2 = 0, gyroOffsetZ_2 = 0;

// Quaternion Variables
float q0_1 = 1.0f, q1_1 = 0.0f, q2_1 = 0.0f, q3_1 = 0.0f; // Sensor 1
float q0_2 = 1.0f, q1_2 = 0.0f, q2_2 = 0.0f, q3_2 = 0.0f; // Sensor 2

// Filter Parameters
const float beta = 0.1f; // Madgwick filter gain
const float sampleFreq = 100.0f; // Sampling frequency in Hz

// Function Prototypes
void initializeMPU9250(uint8_t address);
void calibrateMPU9250(uint8_t address, float &accelOffsetX, float &accelOffsetY, float &accelOffsetZ,
                      float &gyroOffsetX, float &gyroOffsetY, float &gyroOffsetZ);
void readMPU9250(uint8_t address, int16_t &accelX, int16_t &accelY, int16_t &accelZ,
                 int16_t &gyroX, int16_t &gyroY, int16_t &gyroZ);
void madgwickUpdate(float gx, float gy, float gz, float ax, float ay, float az,
                    float &q0, float &q1, float &q2, float &q3);

void setup() {
  // Initialize Serial Monitor
  Serial.begin(115200);
  while (!Serial) {
    ; // Wait for Serial to be ready
  }

  // Initialize I2C Communication
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000); // Fast I2C communication

  // Initialize MPU9250 Sensors
  initializeMPU9250(MPU9250_ADDRESS_1);
  initializeMPU9250(MPU9250_ADDRESS_2);

  // Calibrate Sensors
  Serial.println("Calibrating sensors, please wait...");
  calibrateMPU9250(MPU9250_ADDRESS_1, accelOffsetX_1, accelOffsetY_1, accelOffsetZ_1,
                   gyroOffsetX_1, gyroOffsetY_1, gyroOffsetZ_1);
  calibrateMPU9250(MPU9250_ADDRESS_2, accelOffsetX_2, accelOffsetY_2, accelOffsetZ_2,
                   gyroOffsetX_2, gyroOffsetY_2, gyroOffsetZ_2);
  Serial.println("Calibration complete!");
  
  // Initialize stretch sensor
  pinMode(stretchPin, INPUT);
  pinMode(relayPin, INPUT);

  // Initialize Desktop Servo Motor
  pinMode(servoPin, OUTPUT);
  myServo.attach(servoPin, 500, 2600);
}

void loop() {
  // Variables to hold raw sensor data
  int16_t accelX, accelY, accelZ, gyroX, gyroY, gyroZ;

  // Read Sensor 1 Data
  readMPU9250(MPU9250_ADDRESS_1, accelX, accelY, accelZ, gyroX, gyroY, gyroZ);
  float ax1 = accelX * ACCEL_SCALE - accelOffsetX_1;
  float ay1 = accelY * ACCEL_SCALE - accelOffsetY_1;
  float az1 = accelZ * ACCEL_SCALE - accelOffsetZ_1;
  float gx1 = gyroX * GYRO_SCALE - gyroOffsetX_1;
  float gy1 = gyroY * GYRO_SCALE - gyroOffsetY_1;
  float gz1 = gyroZ * GYRO_SCALE - gyroOffsetZ_1;

  madgwickUpdate(gx1, gy1, gz1, ax1, ay1, az1, q0_1, q1_1, q2_1, q3_1);

  // Read Sensor 2 Data
  readMPU9250(MPU9250_ADDRESS_2, accelX, accelY, accelZ, gyroX, gyroY, gyroZ);
  float ax2 = accelX * ACCEL_SCALE - accelOffsetX_2;
  float ay2 = accelY * ACCEL_SCALE - accelOffsetY_2;
  float az2 = accelZ * ACCEL_SCALE - accelOffsetZ_2;
  float gx2 = gyroX * GYRO_SCALE - gyroOffsetX_2;
  float gy2 = gyroY * GYRO_SCALE - gyroOffsetY_2;
  float gz2 = gyroZ * GYRO_SCALE - gyroOffsetZ_2;

  madgwickUpdate(gx2, gy2, gz2, ax2, ay2, az2, q0_2, q1_2, q2_2, q3_2);

  // Print Quaternion Data
  // Serial.print("Sensor 1 Quaternion: ");
  // Serial.print(q0_1, 6); Serial.print(", ");
  // Serial.print(q1_1, 6); Serial.print(", ");
  // Serial.print(q2_1, 6); Serial.print(", ");
  // Serial.println(q3_1, 6);

  // Serial.print("Sensor 2 Quaternion: ");
  // Serial.print(q0_2, 6); Serial.print(", ");
  // Serial.print(q1_2, 6); Serial.print(", ");
  // Serial.print(q2_2, 6); Serial.print(", ");
  // Serial.println(q3_2, 6);

  // Calculate Euler Angles
  updateEulerAngles(q0_1, q1_1, q2_1, q3_1, x1, y1m, z1);
  updateEulerAngles(q0_2, q1_2, q2_2, q3_2, x2, y2, z2);

  // Serial.printf("Sensor 1 Euler Angles X Y Z: %.2f, %.2f, %.2f\n", x1, y1m, z1);
  // Serial.printf("Sensor 2 Euler Angles X Y Z: %.2f, %.2f, %.2f\n", x2, y2, z2);

  // Print stretch sensor data
  int stretchValue = analogRead(stretchPin);
  if (stretchValue < stretchThreshold) {
    stretchCount += 1;
  } else if (stretchCount > 0) {
    stretchCount -= 1;
  }

  Serial.printf("Stretch: %d Stretch count: %d\n", stretchValue, stretchCount);

  delay(10); // Maintain ~100Hz sampling rate
  float angle = ((gx1 + gx2) / 2) * 3;
  // Serial.printf(" %.2f ", angle);
  // Serial.printf("Angle: %f\n", angle);
  myServo.write(map(long(angle), -90, 90, 1, 120)); //  REPLACE 0 WITH ANGLE

  if (stretchCount > 20) {
    Serial.println("Stretch detected!");
    digitalWrite(relayPin, HIGH);
    delay(1000);
    digitalWrite(relayPin, LOW);
    stretchCount = 0;
  }
}

void initializeMPU9250(uint8_t address) {
  Wire.beginTransmission(address);
  Wire.write(PWR_MGMT_1);
  Wire.write(0x00); // Wake up the sensor
  Wire.endTransmission();
  delay(100);
}

void calibrateMPU9250(uint8_t address, float &accelOffsetX, float &accelOffsetY, float &accelOffsetZ,
                      float &gyroOffsetX, float &gyroOffsetY, float &gyroOffsetZ) {
  const int numSamples = 500;
  int32_t accelX = 0, accelY = 0, accelZ = 0;
  int32_t gyroX = 0, gyroY = 0, gyroZ = 0;

  for (int i = 0; i < numSamples; i++) {
    int16_t ax, ay, az, gx, gy, gz;
    readMPU9250(address, ax, ay, az, gx, gy, gz);
    accelX += ax; accelY += ay; accelZ += az;
    gyroX += gx; gyroY += gy; gyroZ += gz;
    delay(2);
  }

  accelOffsetX = (float)accelX / numSamples * ACCEL_SCALE;
  accelOffsetY = (float)accelY / numSamples * ACCEL_SCALE;
  accelOffsetZ = (float)accelZ / numSamples * ACCEL_SCALE;
  gyroOffsetX = (float)gyroX / numSamples * GYRO_SCALE;
  gyroOffsetY = (float)gyroY / numSamples * GYRO_SCALE;
  gyroOffsetZ = (float)gyroZ / numSamples * GYRO_SCALE;
}

void readMPU9250(uint8_t address, int16_t &accelX, int16_t &accelY, int16_t &accelZ,
                 int16_t &gyroX, int16_t &gyroY, int16_t &gyroZ) {
    Wire.beginTransmission(address);
    Wire.write(ACCEL_XOUT_H);
    Wire.endTransmission(false);

    Wire.requestFrom((int)address, (int)14, 1); // Explicit casting fixes ambiguity
    if (Wire.available() == 14) {
        accelX = (Wire.read() << 8) | Wire.read();
        accelY = (Wire.read() << 8) | Wire.read();
        accelZ = (Wire.read() << 8) | Wire.read();
        Wire.read(); Wire.read(); // Skip temperature data
        gyroX = (Wire.read() << 8) | Wire.read();
        gyroY = (Wire.read() << 8) | Wire.read();
        gyroZ = (Wire.read() << 8) | Wire.read();
    }
}


void madgwickUpdate(float gx, float gy, float gz, float ax, float ay, float az,
                    float &q0, float &q1, float &q2, float &q3) {
    float _2q0 = 2.0f * q0;
    float _2q1 = 2.0f * q1;
    float _2q2 = 2.0f * q2;
    float _2q3 = 2.0f * q3;
    float _4q0 = 4.0f * q0;
    float _4q1 = 4.0f * q1;
    float _4q2 = 4.0f * q2;
    float _4q3 = 4.0f * q3;
    float _8q1 = 8.0f * q1;
    float _8q2 = 8.0f * q2;
    float q0q0 = q0 * q0;
    float q1q1 = q1 * q1;
    float q2q2 = q2 * q2;
    float q3q3 = q3 * q3;

    // Normalize accelerometer measurement
    float norm = sqrt(ax * ax + ay * ay + az * az);
    if (norm == 0.0f) return; // Avoid division by zero
    norm = 1.0f / norm;
    ax *= norm;
    ay *= norm;
    az *= norm;

    // Auxiliary variables for gradient descent algorithm
    float s0, s1, s2, s3;
    s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
    s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2;
    s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2;
    s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;

    norm = sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
    norm = 1.0f / norm;
    s0 *= norm;
    s1 *= norm;
    s2 *= norm;
    s3 *= norm;

    // Apply feedback step
    float qDot0 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz) - beta * s0;
    float qDot1 = 0.5f * (q0 * gx + q2 * gz - q3 * gy) - beta * s1;
    float qDot2 = 0.5f * (q0 * gy - q1 * gz + q3 * gx) - beta * s2;
    float qDot3 = 0.5f * (q0 * gz + q1 * gy - q2 * gx) - beta * s3;

    // Integrate rate of change to yield quaternion
    q0 += qDot0 * (1.0f / sampleFreq);
    q1 += qDot1 * (1.0f / sampleFreq);
    q2 += qDot2 * (1.0f / sampleFreq);
    q3 += qDot3 * (1.0f / sampleFreq);

    // Normalize quaternion
    norm = sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    norm = 1.0f / norm;
    q0 *= norm;
    q1 *= norm;
    q2 *= norm;
    q3 *= norm;
}


void updateEulerAngles(float w, float x, float y, float z, float &xrot, float &yrot, float &zrot) {
  xrot = atan2(2.0f * (w * x + y * z), w * w - x * x - y * y + z * z);
  yrot = -asin(2.0f * (x * z - w * y));
  zrot = atan2(2.0f * (x * y + w * z), w * w + x * x - y * y - z * z);
}