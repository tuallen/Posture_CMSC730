#include <Arduino.h>
#include <Wire.h>
#include <MadgwickAHRS.h>  // Madgwick filter library

const int MPU9250_ADDRESS_1 = 0x68;  // First MPU9250 I2C address (AD0 = LOW)
const int MPU9250_ADDRESS_2 = 0x69;  // Second MPU9250 I2C address (AD0 = HIGH)
const int sda = 21;
const int scl = 22;

Madgwick filter_1;  // Madgwick filter instance for the first sensor
Madgwick filter_2;  // Madgwick filter instance for the second sensor

int16_t accelX, accelY, accelZ, gyroX, gyroY, gyroZ;
float accelXOffset_1 = 0, accelYOffset_1 = 0, accelZOffset_1 = 0;
float gyroXOffset_1 = 0, gyroYOffset_1 = 0, gyroZOffset_1 = 0;
float accelXOffset_2 = 0, accelYOffset_2 = 0, accelZOffset_2 = 0;
float gyroXOffset_2 = 0, gyroYOffset_2 = 0, gyroZOffset_2 = 0;

void readAccelData(uint8_t address) {
  Wire.beginTransmission(address);
  Wire.write(0x3B);  // Starting register for accelerometer data
  Wire.endTransmission(false);
  Wire.requestFrom(address, 6);  // Request 6 bytes for accelerometer

  if (Wire.available() == 6) {
    accelX = Wire.read() << 8 | Wire.read();
    accelY = Wire.read() << 8 | Wire.read();
    accelZ = Wire.read() << 8 | Wire.read();
  }
}

void readGyroData(uint8_t address) {
  Wire.beginTransmission(address);
  Wire.write(0x43);  // Starting register for gyroscope data
  Wire.endTransmission(false);
  Wire.requestFrom(address, 6);  // Request 6 bytes for gyroscope

  if (Wire.available() == 6) {
    gyroX = Wire.read() << 8 | Wire.read();
    gyroY = Wire.read() << 8 | Wire.read();
    gyroZ = Wire.read() << 8 | Wire.read();
  }
}

void wakeUpMPU(uint8_t address) {
  Wire.beginTransmission(address);
  Wire.write(0x6B);  // Power management register
  Wire.write(0);      // Set it to 0 to wake up the sensor
  Wire.endTransmission();
  delay(100);
}

// Apply calibration offsets
void applyAccelCalibration(float &accelX, float &accelY, float &accelZ, float accelXOffset, float accelYOffset, float accelZOffset) {
  accelX -= accelXOffset;
  accelY -= accelYOffset;
  accelZ -= accelZOffset;
}

void applyGyroCalibration(float &gyroX, float &gyroY, float &gyroZ, float gyroXOffset, float gyroYOffset, float gyroZOffset) {
  gyroX -= gyroXOffset;
  gyroY -= gyroYOffset;
  gyroZ -= gyroZOffset;
}

// Function to update the Madgwick filter
void updateFilter(Madgwick &filter, float gx, float gy, float gz, float ax, float ay, float az, float dt) {
  // Gyroscope data must be converted to rad/s (from deg/s) before passing to the filter
  const float gyroScale = 131.0;  // 1 LSB = 131 deg/s
  gx = gx / gyroScale * PI / 180.0;
  gy = gy / gyroScale * PI / 180.0;
  gz = gz / gyroScale * PI / 180.0;

  // Accelerometer data should be converted to 'g' units
  const float accelScale = 16384.0;  // 1 LSB = 1g
  ax = ax / accelScale;
  ay = ay / accelScale;
  az = az / accelScale;

  // Update the Madgwick filter with new sensor data
  filter.update(gx, gy, gz, ax, ay, az);
}

void printQuaternions(Madgwick &filter, const char* sensorName) {
  // Get the quaternion from the filter
  float qw = filter.getQuaternionW();
  float qx = filter.getQuaternionX();
  float qy = filter.getQuaternionY();
  float qz = filter.getQuaternionZ();

  // Print the quaternion values
  Serial.println(String("*Quaternion from ") + sensorName + "*");
  Serial.print(">qw:");
  Serial.print(qw, 6);
  Serial.print("\n");

  Serial.print(">qx:");
  Serial.print(qx, 6);
  Serial.print("\n");

  Serial.print(">qy:");
  Serial.print(qy, 6);
  Serial.print("\n");

  Serial.print(">qz:");
  Serial.print(qz, 6);
  Serial.print("\n");
}

void setup() {
  Wire.begin(sda, scl);
  Serial.begin(9600);  // Ensure this matches your Teleplot settings

  // Initialize both filters
  filter_1.begin(256);  // Initialize Madgwick filter for 256 Hz
  filter_2.begin(256);

  // Wake up both MPU9250 sensors
  wakeUpMPU(MPU9250_ADDRESS_1);
  wakeUpMPU(MPU9250_ADDRESS_2);
}

void loop() {
  static unsigned long lastUpdate = 0;
  unsigned long now = millis();
  float dt = (now - lastUpdate) / 1000.0;  // Time between updates in seconds
  lastUpdate = now;

  // Read and process data from the first sensor
  readAccelData(MPU9250_ADDRESS_1);
  readGyroData(MPU9250_ADDRESS_1);
  applyAccelCalibration(accelX, accelY, accelZ, accelXOffset_1, accelYOffset_1, accelZOffset_1);
  applyGyroCalibration(gyroX, gyroY, gyroZ, gyroXOffset_1, gyroYOffset_1, gyroZOffset_1);
  updateFilter(filter_1, gyroX, gyroY, gyroZ, accelX, accelY, accelZ, dt);
  printQuaternions(filter_1, "MPU9250_1");

  // Read and process data from the second sensor
  readAccelData(MPU9250_ADDRESS_2);
  readGyroData(MPU9250_ADDRESS_2);
  applyAccelCalibration(accelX, accelY, accelZ, accelXOffset_2, accelYOffset_2, accelZOffset_2);
  applyGyroCalibration(gyroX, gyroY, gyroZ, gyroXOffset_2, gyroYOffset_2, gyroZOffset_2);
  updateFilter(filter_2, gyroX, gyroY, gyroZ, accelX, accelY, accelZ, dt);
  printQuaternions(filter_2, "MPU9250_2");

  // Add a small delay to control the data rate
  delay(10);  // Adjust the delay as needed
}
