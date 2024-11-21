#include <Arduino.h>
#include <Wire.h>

const int MPU9250_ADDRESS_1 = 0x68; // I2C address of first MPU9250 (AD0 = 0)
const int MPU9250_ADDRESS_2 = 0x69; // I2C address of second MPU9250 (AD0 = 1)

const int sda = 21;
const int scl = 22;

int16_t accelX_1, accelY_1, accelZ_1;
int16_t gyroX_1, gyroY_1, gyroZ_1;
int16_t accelX_2, accelY_2, accelZ_2;
int16_t gyroX_2, gyroY_2, gyroZ_2;

// Function prototypes
void initMPU9250(int address);
void readSensorData(int address, int16_t &accelX, int16_t &accelY, int16_t &accelZ, 
                    int16_t &gyroX, int16_t &gyroY, int16_t &gyroZ);

void setup() {
  Wire.begin(sda, scl);
  Serial.begin(9600);
  
  // Initialize both sensors
  initMPU9250(MPU9250_ADDRESS_1);
  initMPU9250(MPU9250_ADDRESS_2);
}

void loop() {
  // Read data from the first MPU9250 sensor
  readSensorData(MPU9250_ADDRESS_1, accelX_1, accelY_1, accelZ_1, gyroX_1, gyroY_1, gyroZ_1);
  // Read data from the second MPU9250 sensor
  readSensorData(MPU9250_ADDRESS_2, accelX_2, accelY_2, accelZ_2, gyroX_2, gyroY_2, gyroZ_2);
  
  // Print accelerometer and gyroscope data from both sensors
  Serial.print("Sensor 1 - Accel: ");
  Serial.print("X: "); Serial.print(accelX_1); Serial.print(" ");
  Serial.print("Y: "); Serial.print(accelY_1); Serial.print(" ");
  Serial.print("Z: "); Serial.println(accelZ_1);
  Serial.print("Sensor 1 - Gyro: ");
  Serial.print("X: "); Serial.print(gyroX_1); Serial.print(" ");
  Serial.print("Y: "); Serial.print(gyroY_1); Serial.print(" ");
  Serial.print("Z: "); Serial.println(gyroZ_1);
  
  Serial.print("Sensor 2 - Accel: ");
  Serial.print("X: "); Serial.print(accelX_2); Serial.print(" ");
  Serial.print("Y: "); Serial.print(accelY_2); Serial.print(" ");
  Serial.print("Z: "); Serial.println(accelZ_2);
  Serial.print("Sensor 2 - Gyro: ");
  Serial.print("X: "); Serial.print(gyroX_2); Serial.print(" ");
  Serial.print("Y: "); Serial.print(gyroY_2); Serial.print(" ");
  Serial.print("Z: "); Serial.println(gyroZ_2);
  
  delay(500); // Delay between reads
}

// Function to initialize MPU9250 sensor
void initMPU9250(int address) {
  Wire.beginTransmission(address);
  Wire.write(0x6B);  // Power management register
  Wire.write(0);     // Wake up the sensor
  Wire.endTransmission();
}

// Function to read accelerometer and gyro data from the sensor
void readSensorData(int address, int16_t &accelX, int16_t &accelY, int16_t &accelZ, 
                    int16_t &gyroX, int16_t &gyroY, int16_t &gyroZ) {
  // Request accelerometer data (starting from 0x3B)
  Wire.beginTransmission(address);
  Wire.write(0x3B);  // Accelerometer data start register
  Wire.endTransmission(false);
  Wire.requestFrom(address, 6); // Read 6 bytes (accelerometer data)
  
  if (Wire.available() == 6) {
    accelX = Wire.read() << 8 | Wire.read(); // Read X axis data
    accelY = Wire.read() << 8 | Wire.read(); // Read Y axis data
    accelZ = Wire.read() << 8 | Wire.read(); // Read Z axis data
  }
  
  // Request gyroscope data (starting from 0x43)
  Wire.beginTransmission(address);
  Wire.write(0x43);  // Gyroscope data start register
  Wire.endTransmission(false);
  Wire.requestFrom(address, 6); // Read 6 bytes (gyroscope data)
  
  if (Wire.available() == 6) {
    gyroX = Wire.read() << 8 | Wire.read(); // Read X axis data
    gyroY = Wire.read() << 8 | Wire.read(); // Read Y axis data
    gyroZ = Wire.read() << 8 | Wire.read(); // Read Z axis data
  }
}
