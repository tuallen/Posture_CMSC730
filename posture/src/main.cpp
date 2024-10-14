#include <Arduino.h>
#include <Wire.h>

const int MPU9250_ADDRESS_1 = 0x68;  // First MPU9250 I2C address (AD0 = LOW)
const int MPU9250_ADDRESS_2 = 0x69;  // Second MPU9250 I2C address (AD0 = HIGH)
const int sda = 21;
const int scl = 22;

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

// Accelerometer and Gyroscope calibration for a given sensor
void calibrateIMU(uint8_t address, float &accelXOffset, float &accelYOffset, float &accelZOffset,
                  float &gyroXOffset, float &gyroYOffset, float &gyroZOffset) {
  long accelSumX = 0, accelSumY = 0, accelSumZ = 0;
  long gyroSumX = 0, gyroSumY = 0, gyroSumZ = 0;
  const int samples = 1000;  // Number of samples for averaging

  for (int i = 0; i < samples; i++) {
    readAccelData(address);
    readGyroData(address);

    accelSumX += accelX;
    accelSumY += accelY;
    accelSumZ += accelZ;
    gyroSumX += gyroX;
    gyroSumY += gyroY;
    gyroSumZ += gyroZ;

    delay(10);  // Delay between readings
  }

  // Calculate accelerometer offsets
  accelXOffset = accelSumX / (float)samples;
  accelYOffset = accelSumY / (float)samples;
  accelZOffset = (accelSumZ / (float)samples) - 16384;  // 1g = 16384 in raw values

  // Calculate gyroscope offsets
  gyroXOffset = gyroSumX / (float)samples;
  gyroYOffset = gyroSumY / (float)samples;
  gyroZOffset = gyroSumZ / (float)samples;

  Serial.println("Calibration complete.");
}

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

void printSensorData(uint8_t address, const char* sensorName,
                     float accelXOffset, float accelYOffset, float accelZOffset,
                     float gyroXOffset, float gyroYOffset, float gyroZOffset) {
  // Read accelerometer and gyroscope data
  readAccelData(address);
  readGyroData(address);

  // Apply calibration
  applyAccelCalibration(accelX, accelY, accelZ, accelXOffset, accelYOffset, accelZOffset);
  applyGyroCalibration(gyroX, gyroY, gyroZ, gyroXOffset, gyroYOffset, gyroZOffset);

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

void setup() {
  Wire.begin(sda, scl);
  Serial.begin(9600);  // Ensure this matches your Teleplot settings

  // Wake up both MPU9250 sensors
  wakeUpMPU(MPU9250_ADDRESS_1);
  wakeUpMPU(MPU9250_ADDRESS_2);

  // Calibrate both sensors
  Serial.println("Calibrating MPU9250_1...");
  calibrateIMU(MPU9250_ADDRESS_1, accelXOffset_1, accelYOffset_1, accelZOffset_1,
               gyroXOffset_1, gyroYOffset_1, gyroZOffset_1);

  Serial.println("Calibrating MPU9250_2...");
  calibrateIMU(MPU9250_ADDRESS_2, accelXOffset_2, accelYOffset_2, accelZOffset_2,
               gyroXOffset_2, gyroYOffset_2, gyroZOffset_2);
}

void loop() {
  // Read and print data from the first sensor
  printSensorData(MPU9250_ADDRESS_1, "MPU9250_1", accelXOffset_1, accelYOffset_1, accelZOffset_1,
                  gyroXOffset_1, gyroYOffset_1, gyroZOffset_1);

  // Read and print data from the second sensor
  printSensorData(MPU9250_ADDRESS_2, "MPU9250_2", accelXOffset_2, accelYOffset_2, accelZOffset_2,
                  gyroXOffset_2, gyroYOffset_2, gyroZOffset_2);

  // Add a small delay to control the data rate
  delay(500);  // Adjust the delay as needed
}
