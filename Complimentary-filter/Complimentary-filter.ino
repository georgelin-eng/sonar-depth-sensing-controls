#include <Wire.h>                // For I2C communications
#include <TinyGPS++.h>           // Parses GPS NMEA strings for easy to use outputs
#include <SoftwareSerial.h>      // Allows for TX and RX pins on the IC to be connected to digital pins on the Arduino
#include <Adafruit_HMC5883_U.h>  // Use with the magnetometer. See http://adafruit.github.io/Adafruit_HMC5883_Unified/html/class_adafruit___h_m_c5883___unified.html#a7676ad9bfb73590ef1089e5829734c91
#include <math.h>

#include "accelerometer_functions.h"  // functions that I'm using for the accelerometer
#include "system_state_variables.h"   // variables which descibe system state

const int RXPin = 3;
const int TXPin = 4;

int movingAverageCounter = 0;
const int windowSize = 15;

float* fMagX;  // Declare as pointer for dynamic memory allocation
float* fMagY;
float* fMagZ;

// float accelScalingFactor = 1.02310299429; // Callibration value for scaling
float accelScalingFactor = 1.02420299429; // Value used for testing sensor drift in the Z component

float* fAccX;
float* fAccY;
float* fAccZ;

float posX = 0.0;
float posY = 0.0;
float posZ = 0.0;
float speedX = 0;
float speedY = 0;
float speedZ = 0;
float AccX = 0;
float AccY = 0;
float AccZ = 0;

float prevAccX = 0;
float prevAccY = 0;
float prevAccZ = 0;
long prevTime = 0;
long printTime = 0;

int counter = 0;                     // These values are for initializing the initial value of acceleration so that gravity is accounted for
float initialAccX = 0;
float initialAccY = 0;
float initialAccZ = 0;

// Assign a unique ID to this sensor at the same time
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);

// Setup GPS and software serial
TinyGPSPlus gps;
SoftwareSerial ss(RXPin, TXPin);

// Sets up I2C communications on the I2C
void setup() {
  Serial.begin(9600);

  allocateArrays(); // Array used for the moving average filter

  /* Initialise the sensor */
  if (!mag.begin()) {
    Serial.println("Ooops, no HMC5883 detected ... Check your wiring!");
    while (1)
      ;
  }

  // SETUP ACCELEROMETER COMMUNICATIONS
  Wire.begin();                 // Enables use of the wire library
  Wire.beginTransmission(MPU);  // Begins  communication with MPU6050 which has address at 0x68
  Wire.write(PWR_MGMT_1);       // Talk to the register that controls power management
  Wire.write(RESET_VALUE);      // Make reset - place a 0 into the 6B register
  Wire.endTransmission(true);   //end the transmission
}

void loop() {
  // =======================  MAGNETOMETER ======================= //
  sensors_event_t event;          // creates a structure vairable, event, based off the sensors_event_t structure
  mag.getEvent(&event);           // Gets magnetic info
  float MagX = event.magnetic.x;  // Obtains the strength of the magnetic field vector in x through the member x
  float MagY = event.magnetic.y;  // This is a member of magnetic which is a member of the sensors_event_t struct.
  float MagZ = event.magnetic.z;  // Additionally, the values are stored in a float structure so I used the same data type for consistency

  /* Display the results (magnetic vector values are in micro-Tesla (uT)) */
  // printVector (MagX, MagY, MagZ);

  // =======================  GPS ======================= //



  // =======================  ACCELEROMETER ======================= //
  Wire.beginTransmission(MPU);
  Wire.write(ACCEL_XOUT_H);
  Wire.requestFrom(MPU, 6);

  if (counter <= NUM_VALUES) {
    initialAccX += (Wire.read() << 8 | Wire.read()) / 16384.0 * GRAVITY;
    initialAccY += (Wire.read() << 8 | Wire.read()) / 16384.0 * GRAVITY;
    initialAccZ += (Wire.read() << 8 | Wire.read()) / 16384.0 * GRAVITY;

    // Aferwards, take the average after some time so that the acceleration component due to gravity and tilt can be determined
    // This only works at a constant tilt angle and is best used if the IMU is level
    if (counter == NUM_VALUES) {
      initialAccX /= counter;
      initialAccY /= counter;
      initialAccZ /= counter;
    }

    counter++;

    // Serial.println ("Calibrating accelerometer...");
  }

  else {
    AccX = (Wire.read() << 8 | Wire.read()) / 16384.0 * GRAVITY * accelScalingFactor - 0;
    AccY = (Wire.read() << 8 | Wire.read()) / 16384.0 * GRAVITY * accelScalingFactor - 0;
    AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0 * GRAVITY * accelScalingFactor - 9.81;
  }

  Wire.endTransmission();

  // =======================  FILTERING RAW DATA ======================= //
  //            Currently Using: Moving average filter 
  //            Aims for the filtered output are: smoothness, remove sharp peaks, filtered matches closely with raw output

  // Initialize the filling in of the averages, then assign new values to the filter
  if (movingAverageCounter <= windowSize) {    
    fMagX[movingAverageCounter] = MagX;
    fMagY[movingAverageCounter] = MagY;
    fMagZ[movingAverageCounter] = MagZ;

    fAccX[movingAverageCounter] = AccX;
    fAccY[movingAverageCounter] = AccY;
    fAccZ[movingAverageCounter] = AccZ;

    movingAverageCounter++;

    // if (movingAverageCounter == windowSize) {
    //   float restingAcceleration = sqrt (pow(AccX,2) + pow(AccY,2) + pow(AccZ,2));

    //   accelScalingFactor = GRAVITY /restingAcceleration;

    //   alpha = calcEulerAngle (AccX ) * (180.0 / M_PI);
    //   beta = calcEulerAngle (AccY ) * (180.0 / M_PI) ;
    //   gamma = calcEulerAngle (AccZ )* (180.0 / M_PI);
    // }
  }

  else {
    float SMA_MagX = arrayAverage(fMagX, windowSize);
    float SMA_MagY = arrayAverage(fMagY, windowSize);
    float SMA_MagZ = arrayAverage(fMagZ, windowSize);

    float SMA_AccX = arrayAverage(fAccX, windowSize);
    float SMA_AccY = arrayAverage(fAccY, windowSize);
    float SMA_AccZ = arrayAverage(fAccZ, windowSize);

    shiftArrayLeft(fMagX, windowSize);
    shiftArrayLeft(fMagY, windowSize);
    shiftArrayLeft(fMagZ, windowSize);

    shiftArrayLeft(fAccX, windowSize);
    shiftArrayLeft(fAccY, windowSize);
    shiftArrayLeft(fAccZ, windowSize);

    // Add new value to the end of the averaging array;
    fMagX[windowSize - 1] = MagX;
    fMagY[windowSize - 1] = MagY;
    fMagZ[windowSize - 1] = MagZ;

    fAccX[windowSize - 1] = AccX;
    fAccY[windowSize - 1] = AccY;
    fAccZ[windowSize - 1] = AccZ;

    // ======================= CALCULATING POSITION

    long currentTime = millis();                          
    float delTime = (currentTime - prevTime) / 1000.0;  

    speedX += AccX * delTime;
    speedY += AccY * delTime;
    speedZ += AccZ * delTime;

    posX += 0.5 * AccX * delTime * delTime + speedX * delTime ;
    posY += 0.5 * AccY * delTime * delTime + speedY * delTime ;
    posZ += 0.5 * AccZ * delTime * delTime + speedZ * delTime ;

    SMA_speedX += SMA_AccX * delTime;
    SMA_speedY += SMA_AccY * delTime;
    SMA_speedZ += SMA_AccZ * delTime;

    SMA_posX += 0.5 * SMA_AccX * delTime * delTime + SMA_speedX * delTime ;
    SMA_posY += 0.5 * SMA_AccY * delTime * delTime + SMA_speedY * delTime ;
    SMA_posZ += 0.5 * SMA_AccZ * delTime * delTime + SMA_speedZ * delTime ;

    prevTime = currentTime; 

    // ======================= LOGGING DATA =======================

    // printVector (SMA_MagX, SMA_MagY, SMA_MagZ);
    // Serial.print ("Euler angles: "); printVector (alpha, beta, gamma);
    // printVector (AccX, AccY, AccZ);
    // printVector (SMA_AccX, SMA_AccY, SMA_AccZ);

    // --- Noise Comparisons
    // Serial.print(MagX);
    // Serial.print(",");
    // Serial.print(SMA_MagX);
    // Serial.println("");

    // Serial.print(AccZ);
    // Serial.print(",");
    // Serial.print(SMA_AccZ);
    // Serial.println("");

    // ---- Sensor drift comparisons 
    Serial.print (posZ); Serial.print (",");
    Serial.print (SMA_posZ); Serial.print (",");
    Serial.print (speedZ); Serial.print (",");
    Serial.print (SMA_speedZ); Serial.print (",");
    Serial.print(AccZ); Serial.print(",");
    Serial.println(SMA_AccZ);
   }

  delay (SAMPLE_INTERVAL);
}

// --------------- FUNCTIONS BEGIN --------------------------

void printVector(float x, float y, float z) {
  Serial.print(x);
  Serial.print(",");
  Serial.print(y);
  Serial.print(",");
  Serial.print(z);
  Serial.println("");
}

float arrayAverage(float data[], int size) {
  float sum = 0;
  for (int i = 0; i < size; i++) {
    sum += data[i];
  }

  return sum / size;
}

void shiftArrayLeft(float data[], int size) {
  for (int i = 0; i < size - 1; i++) {
    data[i] = data[i + 1];
  }
}

void allocateArrays() {
  fMagX = new float[windowSize];
  fMagY = new float[windowSize];
  fMagZ = new float[windowSize];

  fAccX = new float[windowSize];
  fAccY = new float[windowSize];
  fAccZ = new float[windowSize];
}

float calcEulerAngle (float acceleration) {
  return acos (acceleration / GRAVITY);
}
