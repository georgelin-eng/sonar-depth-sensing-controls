#ifndef IMU_functions.h
#define IMU_functions.h

const int SAMPLE_INTERVAL = 20;       // Time between sensor readings


const int MPU = 0x68;               // MPU6050 I2C address
const int PWR_MGMT_1 = 0x6B;        // Power management register
const int RESET_VALUE = 0x00;       // Reset value for all registers on the MPU

const int ACCEL_XOUT_H = 0x3B;      // Register of X-acceleration
const int GYRO_XOUT_H = 0x43;      // Register of X-acceleration

const int NUM_VALUES = 10;         // Number of values that are read from the IMU to initialize acceleration
const int GRAVITY = 9.81;           // Acceleration due to gravity
const int PRINT_DELAY = 500;        // Time bewteen printing new delays for the sake of printing

extern float SMA_posX = 0.0;
extern float SMA_posY = 0.0;
extern float SMA_posZ = 0.0;
extern float SMA_speedX = 0;
extern float SMA_speedY = 0;
extern float SMA_speedZ = 0;

#endif