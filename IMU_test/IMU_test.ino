#include <Wire.h>

// MPU9250 I2C addresses
#define MPU9250_ADDRESS 0x68
#define MAG_ADDRESS 0x0C

// MPU9250 registers
#define ACCEL_XOUT_H 0x3B
#define GYRO_XOUT_H 0x43
#define PWR_MGMT_1 0x6B
#define INT_PIN_CFG 0x37
#define WHO_AM_I_MPU9250 0x75

// Magnetometer registers
#define MAG_CNTL1 0x0A
#define MAG_XOUT_L 0x03
#define MAG_WHO_AM_I 0x00
#define MAG_STATUS2 0x02

// Scale factors
#define ACCEL_SCALE 16384.0  // ±2g
#define GYRO_SCALE 131.0     // ±250°/s

void setup() {
  Serial.begin(115200);
  while(!Serial) { delay(10); }
  delay(1000);
  
  Wire.begin();
  
  Serial.println("=== MPU9250 DIAGNOSTICS ===");
  
  // Test MPU9250 communication
  uint8_t mpu_id = readRegister(MPU9250_ADDRESS, WHO_AM_I_MPU9250);
  Serial.print("MPU9250 WHO_AM_I: 0x");
  Serial.println(mpu_id, HEX);
  if(mpu_id != 0x71) {
    Serial.println("ERROR: Expected 0x71 for MPU9250!");
  } else {
    Serial.println("MPU9250 communication OK");
  }
  
  // Initialize MPU9250
  initMPU9250();
  Serial.println("MPU9250 initialized");
  
  // Test magnetometer communication
  Serial.println("Scanning for magnetometer...");
  
  // Try common magnetometer addresses
  uint8_t addresses[] = {0x0C, 0x0D, 0x0E, 0x1E, 0x30};
  String names[] = {"AK8963(0x0C)", "AK8975(0x0D)", "AK09916(0x0E)", "HMC5883L(0x1E)", "QMC5883L(0x30)"};
  
  bool found_mag = false;
  for(int i = 0; i < 5; i++) {
    Serial.print("Trying ");
    Serial.print(names[i]);
    Serial.print(": ");
    
    Wire.beginTransmission(addresses[i]);
    if(Wire.endTransmission() == 0) {
      // Device responded, check WHO_AM_I
      uint8_t whoami = readRegister(addresses[i], 0x00);
      Serial.print("WHO_AM_I=0x");
      Serial.print(whoami, HEX);
      
      if(whoami != 0xFF && whoami != 0x00) {
        Serial.println(" - FOUND!");
        found_mag = true;
      } else {
        Serial.println(" - responds but no valid ID");
      }
    } else {
      Serial.println("no response");
    }
  }
  
  if(!found_mag) {
    Serial.println("No magnetometer found on any common address");
    Serial.println("Your module may not have a magnetometer");
  }
  
  // Original AK8963 test
  uint8_t mag_id = readRegister(MAG_ADDRESS, MAG_WHO_AM_I);
  Serial.print("AK8963 WHO_AM_I: 0x");
  Serial.println(mag_id, HEX);
  
  // Initialize magnetometer
  initMagnetometer();
  
  // Check magnetometer status
  delay(100);
  uint8_t mag_status = readRegister(MAG_ADDRESS, MAG_STATUS2);
  Serial.print("Magnetometer Status: 0x");
  Serial.println(mag_status, HEX);
  
  Serial.println("=== END DIAGNOSTICS ===");
  Serial.println();
  Serial.println("Starting sensor readings...");
  Serial.println("Format: Ax,Ay,Az,Gx,Gy,Gz,Mx,My,Mz");
  delay(1000);
}

void loop() {
  // Read sensors
  int16_t accel[3], gyro[3], mag[3];
  
  readAccelGyro(accel, gyro);
  readMagnetometer(mag);
  
  // Convert to physical units
  float ax = accel[0] / ACCEL_SCALE;
  float ay = accel[1] / ACCEL_SCALE;
  float az = accel[2] / ACCEL_SCALE;
  
  float gx = gyro[0] / GYRO_SCALE;
  float gy = gyro[1] / GYRO_SCALE;
  float gz = gyro[2] / GYRO_SCALE;
  
  float mx = mag[0];
  float my = mag[1];
  float mz = mag[2];
  
  // Output data
  Serial.print(ax, 3); Serial.print(",");
  Serial.print(ay, 3); Serial.print(",");
  Serial.print(az, 3); Serial.print(",");
  Serial.print(gx, 2); Serial.print(",");
  Serial.print(gy, 2); Serial.print(",");
  Serial.print(gz, 2); Serial.print(",");
  Serial.print(mx, 1); Serial.print(",");
  Serial.print(my, 1); Serial.print(",");
  Serial.println(mz, 1);
  
  delay(50);
}

void initMPU9250() {
  Serial.println("Configuring MPU...");
  
  // Wake up MPU9250
  writeRegister(MPU9250_ADDRESS, PWR_MGMT_1, 0x00);
  delay(100);
  
  // Check power management
  uint8_t pwr = readRegister(MPU9250_ADDRESS, PWR_MGMT_1);
  Serial.print("Power management: 0x");
  Serial.println(pwr, HEX);
  
  // Try multiple bypass configurations
  Serial.println("Enabling I2C bypass...");
  
  // Method 1: Standard bypass
  writeRegister(MPU9250_ADDRESS, INT_PIN_CFG, 0x02);
  delay(10);
  
  // Method 2: Also try master mode disable
  writeRegister(MPU9250_ADDRESS, 0x6A, 0x00);  // USER_CTRL - disable I2C master
  delay(10);
  
  // Method 3: Enhanced bypass
  writeRegister(MPU9250_ADDRESS, INT_PIN_CFG, 0x22);  // More flags
  delay(10);
  
  // Check bypass status
  uint8_t bypass = readRegister(MPU9250_ADDRESS, INT_PIN_CFG);
  Serial.print("Bypass config: 0x");
  Serial.println(bypass, HEX);
}

void initMagnetometer() {
  // Set magnetometer to continuous measurement mode
  writeRegister(MAG_ADDRESS, MAG_CNTL1, 0x16);  // 16-bit, 100Hz
  delay(10);
}

void readAccelGyro(int16_t* accel, int16_t* gyro) {
  uint8_t buffer[14];
  
  Wire.beginTransmission(MPU9250_ADDRESS);
  Wire.write(ACCEL_XOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU9250_ADDRESS, 14);
  
  for(int i = 0; i < 14; i++) {
    buffer[i] = Wire.read();
  }
  
  accel[0] = (buffer[0] << 8) | buffer[1];
  accel[1] = (buffer[2] << 8) | buffer[3];
  accel[2] = (buffer[4] << 8) | buffer[5];
  
  gyro[0] = (buffer[8] << 8) | buffer[9];
  gyro[1] = (buffer[10] << 8) | buffer[11];
  gyro[2] = (buffer[12] << 8) | buffer[13];
}

void readMagnetometer(int16_t* mag) {
  uint8_t buffer[7];
  
  Wire.beginTransmission(MAG_ADDRESS);
  Wire.write(MAG_XOUT_L);
  Wire.endTransmission(false);
  Wire.requestFrom(MAG_ADDRESS, 7);
  
  for(int i = 0; i < 7; i++) {
    buffer[i] = Wire.read();
  }
  
  // Check overflow
  if(buffer[6] & 0x08) {
    mag[0] = mag[1] = mag[2] = 0;
    return;
  }
  
  mag[0] = (buffer[1] << 8) | buffer[0];
  mag[1] = (buffer[3] << 8) | buffer[2];  
  mag[2] = (buffer[5] << 8) | buffer[4];
}

uint8_t readRegister(uint8_t address, uint8_t reg) {
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(address, (uint8_t)1);
  return Wire.read();
}

void writeRegister(uint8_t address, uint8_t reg, uint8_t value) {
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}