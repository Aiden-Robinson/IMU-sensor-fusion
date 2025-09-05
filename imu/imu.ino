#include <Wire.h>
#include <math.h>

// MPU9250 I2C address and registers
#define MPU9250_ADDRESS 0x68
#define ACCEL_XOUT_H 0x3B
#define GYRO_XOUT_H 0x43
#define PWR_MGMT_1 0x6B

// Scale factors
#define ACCEL_SCALE 16384.0  // ±2g
#define GYRO_SCALE 131.0     // ±250°/s, convert to deg/s

// Kalman filter matrices (4x4 state: roll, pitch, roll_bias, pitch_bias)
float x[4] = {0, 0, 0, 0};     // State vector [roll, pitch, roll_bias, pitch_bias]
float P[4][4];                  // Error covariance matrix
float Q[4][4];                  // Process noise covariance
float R[2][2];                  // Measurement noise covariance
float K[4][2];                  // Kalman gain
float F[4][4];                  // State transition matrix
float H[2][4];                  // Measurement matrix


float dt = 0.02;  // 50Hz update rate
unsigned long lastTime = 0;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  
  // Initialize MPU9250
  initMPU9250();
  
  // Initialize Kalman filter matrices
  initKalmanFilter();
  
  Serial.println("Kalman Filter AHRS Ready");
  Serial.println("Format: Roll,Pitch,AccelX,AccelY,AccelZ,GyroZ");
  
  lastTime = millis();
  delay(1000);
}

void loop() {
  unsigned long currentTime = millis();
  dt = (currentTime - lastTime) / 1000.0;  // Convert to seconds
  lastTime = currentTime;
  
  // Read sensor data
  int16_t accel[3], gyro[3];
  readAccelGyro(accel, gyro);
  
  // Convert to physical units
  float ax = accel[0] / ACCEL_SCALE;  // g
  float ay = accel[1] / ACCEL_SCALE;  // g  
  float az = accel[2] / ACCEL_SCALE;  // g
  
  float gx = gyro[0] / GYRO_SCALE;    // deg/s
  float gy = gyro[1] / GYRO_SCALE;    // deg/s
  float gz = gyro[2] / GYRO_SCALE;    // deg/s
  
  // Calculate accelerometer roll and pitch
  float roll_accel = atan2(ay, sqrt(ax*ax + az*az)) * 180.0/PI;
  float pitch_accel = atan2(-ax, sqrt(ay*ay + az*az)) * 180.0/PI;
  
  // Kalman filter predict step
  kalmanPredict(gx, gy);
  
  // Kalman filter update step
  kalmanUpdate(roll_accel, pitch_accel);
  
  // Output results
  Serial.print(x[0], 2);  Serial.print(",");  // Filtered roll
  Serial.print(x[1], 2);  Serial.print(",");  // Filtered pitch
  Serial.print(ax, 3);    Serial.print(",");  // Raw accel X
  Serial.print(ay, 3);    Serial.print(",");  // Raw accel Y
  Serial.print(az, 3);    Serial.print(",");  // Raw accel Z
  Serial.println(gz, 3);                      // Raw gyro Z for yaw rate
  
  delay(20);  // 50Hz
}

void initMPU9250() {
  // Wake up MPU9250
  writeRegister(MPU9250_ADDRESS, PWR_MGMT_1, 0x00);
  delay(100);
}

void initKalmanFilter() {
  // Initialize state vector to zero
  for(int i = 0; i < 4; i++) {
    x[i] = 0.0;
  }
  
  // Initialize error covariance matrix P (diagonal)
  for(int i = 0; i < 4; i++) {
    for(int j = 0; j < 4; j++) {
      P[i][j] = (i == j) ? 1.0 : 0.0;  // Identity matrix
    }
  }
  
  // Process noise covariance Q
  for(int i = 0; i < 4; i++) {
    for(int j = 0; j < 4; j++) {
      Q[i][j] = 0.0;
    }
  }
  Q[0][0] = 0.001;  // Roll angle process noise
  Q[1][1] = 0.001;  // Pitch angle process noise  
  Q[2][2] = 0.003;  // Roll bias process noise
  Q[3][3] = 0.003;  // Pitch bias process noise
  
  // Measurement noise covariance R
  R[0][0] = 0.3;    // Roll measurement noise (accelerometer)
  R[0][1] = 0.0;
  R[1][0] = 0.0;
  R[1][1] = 0.3;    // Pitch measurement noise (accelerometer)
  
  // Measurement matrix H (we measure roll and pitch directly)
  for(int i = 0; i < 2; i++) {
    for(int j = 0; j < 4; j++) {
      H[i][j] = 0.0;
    }
  }
  H[0][0] = 1.0;  // Measure roll (state 0)
  H[1][1] = 1.0;  // Measure pitch (state 1)
}

void kalmanPredict(float gyro_x, float gyro_y) {
  // State transition matrix F
  F[0][0] = 1.0;  F[0][1] = 0.0;  F[0][2] = -dt;  F[0][3] = 0.0;
  F[1][0] = 0.0;  F[1][1] = 1.0;  F[1][2] = 0.0;   F[1][3] = -dt;
  F[2][0] = 0.0;  F[2][1] = 0.0;  F[2][2] = 1.0;   F[2][3] = 0.0;
  F[3][0] = 0.0;  F[3][1] = 0.0;  F[3][2] = 0.0;   F[3][3] = 1.0;
  
  // Control input (gyroscope measurements)
  float u[2] = {gyro_x * dt, gyro_y * dt};
  
  // Predict state: x = F*x + B*u
  float x_new[4];
  x_new[0] = x[0] + (gyro_x - x[2]) * dt;  // roll = roll + (gyro_x - bias) * dt
  x_new[1] = x[1] + (gyro_y - x[3]) * dt;  // pitch = pitch + (gyro_y - bias) * dt  
  x_new[2] = x[2];                         // roll_bias stays same
  x_new[3] = x[3];                         // pitch_bias stays same
  
  // Update state
  for(int i = 0; i < 4; i++) {
    x[i] = x_new[i];
  }
  
  // Predict covariance: P = F*P*F' + Q
  float P_temp[4][4];
  matrixMultiply4x4(F, P, P_temp);
  matrixMultiplyTranspose4x4(P_temp, F, P);
  matrixAdd4x4(P, Q, P);
}

void kalmanUpdate(float roll_meas, float pitch_meas) {
  // Innovation (measurement residual)
  float y[2];
  y[0] = roll_meas - x[0];   // Roll innovation
  y[1] = pitch_meas - x[1];  // Pitch innovation
  
  // Innovation covariance: S = H*P*H' + R
  float S[2][2];
  S[0][0] = P[0][0] + R[0][0];  // Roll variance
  S[0][1] = P[0][1];
  S[1][0] = P[1][0];  
  S[1][1] = P[1][1] + R[1][1];  // Pitch variance
  
  // Kalman gain: K = P*H'*S^-1
  float S_inv[2][2];
  matrixInverse2x2(S, S_inv);
  
  // K = P*H'*S_inv (simplified since H is simple)
  K[0][0] = P[0][0] * S_inv[0][0] + P[0][1] * S_inv[1][0];
  K[0][1] = P[0][0] * S_inv[0][1] + P[0][1] * S_inv[1][1];
  K[1][0] = P[1][0] * S_inv[0][0] + P[1][1] * S_inv[1][0];
  K[1][1] = P[1][0] * S_inv[0][1] + P[1][1] * S_inv[1][1];
  K[2][0] = P[2][0] * S_inv[0][0] + P[2][1] * S_inv[1][0];
  K[2][1] = P[2][0] * S_inv[0][1] + P[2][1] * S_inv[1][1];
  K[3][0] = P[3][0] * S_inv[0][0] + P[3][1] * S_inv[1][0];
  K[3][1] = P[3][0] * S_inv[0][1] + P[3][1] * S_inv[1][1];
  
  // Update state: x = x + K*y
  x[0] = x[0] + K[0][0] * y[0] + K[0][1] * y[1];
  x[1] = x[1] + K[1][0] * y[0] + K[1][1] * y[1];
  x[2] = x[2] + K[2][0] * y[0] + K[2][1] * y[1];
  x[3] = x[3] + K[3][0] * y[0] + K[3][1] * y[1];
  
  // Update covariance: P = (I - K*H)*P
  float I_KH[4][4];
  // I - K*H (simplified since H is simple)
  I_KH[0][0] = 1.0 - K[0][0];  I_KH[0][1] = -K[0][1];      I_KH[0][2] = 0.0;  I_KH[0][3] = 0.0;
  I_KH[1][0] = -K[1][0];       I_KH[1][1] = 1.0 - K[1][1]; I_KH[1][2] = 0.0;  I_KH[1][3] = 0.0;
  I_KH[2][0] = -K[2][0];       I_KH[2][1] = -K[2][1];      I_KH[2][2] = 1.0;  I_KH[2][3] = 0.0;
  I_KH[3][0] = -K[3][0];       I_KH[3][1] = -K[3][1];      I_KH[3][2] = 0.0;  I_KH[3][3] = 1.0;
  
  float P_new[4][4];
  matrixMultiply4x4(I_KH, P, P_new);
  
  // Copy back to P
  for(int i = 0; i < 4; i++) {
    for(int j = 0; j < 4; j++) {
      P[i][j] = P_new[i][j];
    }
  }
}

// Matrix operations
void matrixMultiply4x4(float A[4][4], float B[4][4], float C[4][4]) {
  for(int i = 0; i < 4; i++) {
    for(int j = 0; j < 4; j++) {
      C[i][j] = 0.0;
      for(int k = 0; k < 4; k++) {
        C[i][j] += A[i][k] * B[k][j];
      }
    }
  }
}

void matrixMultiplyTranspose4x4(float A[4][4], float B[4][4], float C[4][4]) {
  for(int i = 0; i < 4; i++) {
    for(int j = 0; j < 4; j++) {
      C[i][j] = 0.0;
      for(int k = 0; k < 4; k++) {
        C[i][j] += A[i][k] * B[j][k];  // B transposed
      }
    }
  }
}

void matrixAdd4x4(float A[4][4], float B[4][4], float C[4][4]) {
  for(int i = 0; i < 4; i++) {
    for(int j = 0; j < 4; j++) {
      C[i][j] = A[i][j] + B[i][j];
    }
  }
}

void matrixInverse2x2(float A[2][2], float A_inv[2][2]) {
  float det = A[0][0] * A[1][1] - A[0][1] * A[1][0];
  if(abs(det) < 1e-6) det = 1e-6;  // Avoid division by zero
  
  A_inv[0][0] = A[1][1] / det;
  A_inv[0][1] = -A[0][1] / det;
  A_inv[1][0] = -A[1][0] / det;
  A_inv[1][1] = A[0][0] / det;
}

// Hardware functions
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

void writeRegister(uint8_t address, uint8_t reg, uint8_t value) {
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}