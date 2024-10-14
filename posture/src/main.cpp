#include <Arduino.h>
#include <Wire.h>

const int MPU9250_ADDRESS_1 = 0x68;  // First MPU9250 I2C address (AD0 = LOW)
const int MPU9250_ADDRESS_2 = 0x69;  // Second MPU9250 I2C address (AD0 = HIGH)
const int sda = 21;
const int scl = 22;

int16_t accelX, accelY, accelZ, gyroX, gyroY, gyroZ;

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

void setup() {
  Wire.begin(sda, scl);
  Serial.begin(9600);  // Ensure this matches your Teleplot settings

  // Wake up both MPU9250 sensors
  wakeUpMPU(MPU9250_ADDRESS_1);
  wakeUpMPU(MPU9250_ADDRESS_2);
}

void printSensorData(uint8_t address, const char* sensorName) {
  // Read accelerometer and gyroscope data for the given sensor
  readAccelData(address);
  readGyroData(address);

  // Send the data in Teleplot-compatible format
  Serial.println(String("*Data from ") + sensorName + "*");
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
}

void loop() {
  // Read and print data from the first sensor
  printSensorData(MPU9250_ADDRESS_1, "MPU9250_1");

  // Read and print data from the second sensor
  printSensorData(MPU9250_ADDRESS_2, "MPU9250_2");

  // Add a small delay to control the data rate
  delay(500);  // Adjust the delay as needed
}
