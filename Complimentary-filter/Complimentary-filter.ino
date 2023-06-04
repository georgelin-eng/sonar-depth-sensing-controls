/*
 * This code is broken into specific sections, each which is responsible for one particular part of the overall function of the system 
 * Data aquisition: Magnometer -> GPS -> Accelerometer -> Gyroscope -> Sonar. Afterwards: filtering -> calculations -> writing to SD cards -> serial print
 * 
 */

#include <Wire.h>                // For I2C communications
#include <TinyGPS++.h>           // Parses GPS NMEA strings for easy to use outputs
#include <SoftwareSerial.h>      // Allows for TX and RX pins on the IC to be connected to digital pins on the Arduino
#include <Adafruit_HMC5883_U.h>  // Use with the magnetometer. See http://adafruit.github.io/Adafruit_HMC5883_Unified/html/class_adafruit___h_m_c5883___unified.html#a7676ad9bfb73590ef1089e5829734c91
#include <math.h>
// #include <MPU6050.h>

#include "IMU_functions.h"  // functions that I'm using for the accelerometer
#include "system_state_variables.h"   // variables which descibe system state

// MPU6050 mpu;

const int RXPin = 3;                  // Connects to the TXPin on GPS 
const int TXPin = 4;                  // Connects to the RXPin on GPS

int movingAverageCounter = 0;
const int windowSize = 11;

float* fMagX;  // Declare as pointer for dynamic memory allocation
float* fMagY;
float* fMagZ;

float* fAccX;
float* fAccY;
float* fAccZ;

float* fGyroX;
float* fGyroY;
float* fGyroZ;

float refLat, refLng;               //  New x-y positions are calculated with these longitude and lattitude values as reference
float xDistance;
float yDistance;

float accelScalingFactor = 4.164788518; // Callibration value for scaling
// float accelScalingFactor = 1.02310299429; // Callibration value for scaling
// float accelScalingFactor = 1.02420299429; // Value used for testing sensor drift in the Z component

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

float initialRoll = -1;
float initialPitch = -1;
float Accel_Roll;
float Accel_Pitch;
float Gyro_Roll = 0;
float Gyro_Pitch = 0;

float GyroX;
float GyroY;
float GyroZ;


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

  // SETUP IMU COMMUNICATIONS
  Wire.begin();                 // Enables use of the wire library
  Wire.beginTransmission(MPU);  // Begins  communication with MPU6050 which has address at 0x68
  Wire.write(PWR_MGMT_1);       // Talk to the register that controls power management
  Wire.write(RESET_VALUE);      // Make reset - place a 0 into the 6B register
  Wire.endTransmission(true);   //end the transmission

  // Configure Accelerometer Sensitivity - Full Scale Range (default +/- 2g)
  Wire.beginTransmission(MPU);
  Wire.write(0x1C);                  //Talk to the ACCEL_CONFIG register (1C hex)
  Wire.write(0x10);                  //Set the register bits as 00010000 (+/- 8g full scale range)
  Wire.endTransmission(true);
  // Configure Gyro Sensitivity - Full Scale Range (default +/- 250deg/s)
  Wire.beginTransmission(MPU);
  Wire.write(0x1B);                   // Talk to the GYRO_CONFIG register (1B hex)
  Wire.write(0x10);                   // Set the register bits as 00010000 (1000deg/s full scale)
  Wire.endTransmission(true);
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


  // =======================  GPS ================================ //
  // ============================================================= 

  // Converts every GPS value to a new location on a map using the intial GPS locations as a constant reference. 

  while (ss.available() > 0) {
    char c = ss.read();
    Serial.print(c); // Print raw NMEA sentences

    if (gps.encode(c)) {
      double lat = gps.location.lat(), lng = gps.location.lng();
      float speed = gps.speed.mps();
      int heading = gps.course.deg();
      int alt = gps.altitude.meters();

      // If the reference coordinates have not been set, use the first GPS reading as the reference
      if (isnan(refLat) || isnan(refLng) ) {
        refLat = lat;
        refLng = lng;
      }

      float course = gps.courseTo(refLat, refLng, lat, lng);
      float distance = gps.distanceBetween(refLat, refLng, lat, lng);

      // calculating differences
      xDistance = distance * sin(course * PI / 180.0) ;
      yDistance = distance * cos (course * PI / 180.0);

      delay (20);
    }
  }

  // =======================  ACCELEROMETER ================================== //
  // =========================================================================

  Wire.beginTransmission(MPU);
  Wire.write(ACCEL_XOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true);

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
  }

  else {
    AccX = (Wire.read() << 8 | Wire.read()) / 16384.0 * GRAVITY * accelScalingFactor - 0;
    AccY = (Wire.read() << 8 | Wire.read()) / 16384.0 * GRAVITY * accelScalingFactor - 0;
    AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0 * GRAVITY * accelScalingFactor - 0;
  }

  Wire.endTransmission();


  // =======================  GYROSCOPE ================================== //
  // =========================================================================

  Wire.beginTransmission(MPU);
  Wire.write (GYRO_XOUT_H); // !!!!!! These two lines seem to break the values for sum reason. 
  Wire.endTransmission(false);
  Wire.requestFrom(MPU,6, true);

  GyroX = (Wire.read() << 8 | Wire.read()) / 32.8;
  GyroY = (Wire.read() << 8 | Wire.read()) / 32.8;
  GyroZ = (Wire.read() << 8 | Wire.read()) / 32.8;

  Wire.endTransmission();

  // =======================  FILTERING RAW DATA ============================= //
  // =========================================================================
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

    // fGyroX[movingAverageCounter] = GyroX;
    // fGyroY[movingAverageCounter] = GyroY;
    // fGyroZ[movingAverageCounter] = GyroZ;

    movingAverageCounter++;
  }

  else {
    float SMA_MagX = arrayAverage(fMagX, windowSize);
    float SMA_MagY = arrayAverage(fMagY, windowSize);
    float SMA_MagZ = arrayAverage(fMagZ, windowSize);

    float SMA_AccX = arrayAverage(fAccX, windowSize);
    float SMA_AccY = arrayAverage(fAccY, windowSize);
    float SMA_AccZ = arrayAverage(fAccZ, windowSize);

    // float SMA_GyroX = arrayAverage(fGyroX, windowSize);
    // float SMA_GyroY = arrayAverage(fGyroY, windowSize);
    // float SMA_GyroZ = arrayAverage(fGyroZ, windowSize);

    shiftArrayLeft(fMagX, windowSize);
    shiftArrayLeft(fMagY, windowSize);
    shiftArrayLeft(fMagZ, windowSize);

    shiftArrayLeft(fAccX, windowSize);
    shiftArrayLeft(fAccY, windowSize);
    shiftArrayLeft(fAccZ, windowSize);

    // shiftArrayLeft(fGyroX, windowSize);
    // shiftArrayLeft(fGyroY, windowSize);
    // shiftArrayLeft(fGyroZ, windowSize);

    // Add new value to the end of the averaging array;
    fMagX[windowSize - 1] = MagX;
    fMagY[windowSize - 1] = MagY;
    fMagZ[windowSize - 1] = MagZ;

    fAccX[windowSize - 1] = AccX;
    fAccY[windowSize - 1] = AccY;
    fAccZ[windowSize - 1] = AccZ;

    // fGyroX[windowSize - 1] = GyroX;
    // fGyroY[windowSize - 1] = GyroY;
    // fGyroZ[windowSize - 1] = GyroZ;

    // =================== CALCULATING POSITION AND ORIENTATION =======================
    // ================================================================================


    long currentTime = millis();
    float delTime = (currentTime - prevTime) / 1000.0;  

    SMA_speedX += SMA_AccX * delTime;
    SMA_speedY += SMA_AccY * delTime;
    SMA_speedZ += SMA_AccZ * delTime;

    SMA_posX += 0.5 * SMA_AccX * delTime * delTime + SMA_speedX * delTime ;
    SMA_posY += 0.5 * SMA_AccY * delTime * delTime + SMA_speedY * delTime ;
    SMA_posZ += 0.5 * SMA_AccZ * delTime * delTime + SMA_speedZ * delTime ;

    if (initialRoll == -1 || initialPitch == -1 ) {
      initialRoll = atan2(SMA_AccY, SMA_AccZ) * 180/M_PI;
      initialPitch = atan2(-SMA_AccX, sqrt(SMA_AccY*SMA_AccY + SMA_AccZ*SMA_AccZ)) * 180/M_PI;          
    }
    else {
      Accel_Roll = atan2(SMA_AccY, SMA_AccZ) * 180/M_PI;
      Accel_Pitch = atan2(-SMA_AccX, sqrt(SMA_AccY*SMA_AccY + SMA_AccZ*SMA_AccZ)) * 180/M_PI;

      Gyro_Roll += GyroX * delTime;
      Gyro_Pitch += GyroY * delTime;
    }
    
    prevTime = currentTime; 


    // ======================= LOGGING DATA ==============================
    // ===================================================================

    // Function: This is the location of all values that will be printed to the console

    // printVector (SMA_AccX, SMA_AccY, SMA_AccZ);
    // printVector (AccX, AccY, AccZ);
    // printVector (GyroX, GyroY, GyroZ);
    // Serial.print (Accel_Roll); Serial.print (","); Serial.println (Accel_Pitch);
    Serial.print (Gyro_Roll); Serial.print (","); Serial.println (Gyro_Pitch);
    // Serial.print (initialRoll); Serial.print (","); Serial.println (initialPitch);

    // printVector (xDistance, yDistance, 0);     

    // --- Noise Comparisons
    // Serial.print(MagX);
    // Serial.print(",");
    // Serial.print(SMA_MagX);
    // Serial.println("");

    // Serial.print(AccZ);
    // Serial.print(",");
    // Serial.print(SMA_AccZ);
    // Serial.println("");

    //---- Sensor drift comparisons 
    // Serial.print (posZ); Serial.print (",");
    // Serial.print (SMA_posZ); Serial.print (",");
    // Serial.print (speedZ); Serial.print (",");
    // Serial.print (SMA_speedZ); Serial.print (",");
    // Serial.print(AccZ); Serial.print(",");
    // Serial.println(SMA_AccZ);
  }

  delay (SAMPLE_INTERVAL);
}


// ======================= FUNCTIONS BEGIN =========================
// =================================================================

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
