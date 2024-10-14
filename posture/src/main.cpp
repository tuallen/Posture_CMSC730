#include <Arduino.h>
#include <Wire.h>

const int MPU9250_ADDRESS = 0x68;  // MPU9250 I2C address
const int sda = 21;
const int scl = 22;

int16_t accelX, accelY, accelZ, gyroX, gyroY, gyroZ;

void readAccelData() {
  Wire.beginTransmission(MPU9250_ADDRESS);
  Wire.write(0x3B);  // Starting register for accelerometer data
  Wire.endTransmission(false);
  Wire.requestFrom(MPU9250_ADDRESS, 6);  // Request 6 bytes for accelerometer

  if (Wire.available() == 6) {
    accelX = Wire.read() << 8 | Wire.read();
    accelY = Wire.read() << 8 | Wire.read();
    accelZ = Wire.read() << 8 | Wire.read();
  }
}

void readGyroData() {
  Wire.beginTransmission(MPU9250_ADDRESS);
  Wire.write(0x43);  // Starting register for gyroscope data
  Wire.endTransmission(false);
  Wire.requestFrom(MPU9250_ADDRESS, 6);  // Request 6 bytes for gyroscope

  if (Wire.available() == 6) {
    gyroX = Wire.read() << 8 | Wire.read();
    gyroY = Wire.read() << 8 | Wire.read();
    gyroZ = Wire.read() << 8 | Wire.read();
  }
}


void setup() {
  Wire.begin(sda, scl);
  Serial.begin(9600);  // Ensure this matches your Teleplot settings

  // Wake up the MPU9250
  Wire.beginTransmission(MPU9250_ADDRESS);
  Wire.write(0x6B);  // Power management register
  Wire.write(0);      // Set it to 0 to wake up the sensor
  Wire.endTransmission();
  delay(100);
}

void loop() {
  // Read accelerometer and gyroscope data
  readAccelData();
  readGyroData();

  // Send the data in Teleplot-compatible format
  Serial.println("*Acceleration is in g*");
  Serial.print(">AccelX:");
  Serial.print(accelX / 16384.0, 2);  // Convert to 'g' and print with 2 decimal places
  Serial.print("\n");

  Serial.print(">AccelY:");
  Serial.print(accelY / 16384.0, 2);
  Serial.print("\n");

  Serial.print(">AccelZ:");
  Serial.print(accelZ / 16384.0, 2);
  Serial.print("\n");

  Serial.print(">GyroX:");
  Serial.print(gyroX);
  Serial.print("\n");

  Serial.print(">GyroY:");
  Serial.print(gyroY);
  Serial.print("\n");

  Serial.print(">GyroZ:");
  Serial.print(gyroZ);
  Serial.print("\n");

  // Add a small delay to control the data rate
  delay(100);  // Adjust the delay as needed
}