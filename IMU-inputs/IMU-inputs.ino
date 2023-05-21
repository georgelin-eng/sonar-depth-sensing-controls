#include <Wire.h>
const int MPU = 0x68;                 // MPU6050 I2C address
const int PWR_MGMT_1 = 0x6B;          // Power management register
const int RESET_VALUE = 0x00;         // Reset value for all registers on the MPU
const int ACCEL_XOUT_H = 0x3B;        // Register of X-acceleration
const int NUM_VALUES = 500;            // Number of values that are read from the IMU to initialize acceleration
const int GRAVITY = 9.81;             // Acceleration due to gravity
const int TESTING_DELAY  = 5;        // Time between reading initial acceleration data 
const int PRINT_DELAY = 500;          // Time bewteen printing new delays for the sake of printing

float posX = 0.0;
float posY = 0.0;
float posZ = 0.0;
float speedX = 0;
float speedY = 0;
float speedZ = 0;
long prevTime = 0;
long printTime = 0;

int counter = 0;                     // These values are for initializing the initial value of acceleration so that gravity is accounted for
float initialAccX = 0;
float initialAccY = 0;
float initialAccZ = 0;

void setup() {
  Serial.begin(9600);
  Wire.begin();                       // Enables use of the wire library
  Wire.beginTransmission(MPU);        // Begins  communication with MPU6050 which has address at 0x68
  Wire.write(PWR_MGMT_1);             // Talk to the register that controls power management
  Wire.write(RESET_VALUE);            // Make reset - place a 0 into the 6B register
  Wire.endTransmission(true);         //end the transmission
}

void loop() {  
  Wire.beginTransmission(MPU);
  Wire.write (ACCEL_XOUT_H);
  Wire.requestFrom(MPU, 6);

  if (counter <= NUM_VALUES) {
    if (counter == 0) {
      Serial.println();
      Serial.println ("Checking initial acceleration: ");
      Serial.println();
    }
    initialAccX += (Wire.read() << 8 | Wire.read()) / 16384.0 * GRAVITY;
    initialAccY += (Wire.read() << 8 | Wire.read()) / 16384.0 * GRAVITY;
    initialAccZ += (Wire.read() << 8 | Wire.read()) / 16384.0 * GRAVITY;

    // Aferwards, take the average after some time so that the acceleration component due to gravity and tilt can be determined
    // This only works at a constant tilt angle and is best used if the IMU is level
    if (counter == NUM_VALUES) {
      initialAccX /= counter;
      initialAccY /= counter;
      initialAccZ /= counter;

      printInitial (initialAccX, initialAccY, initialAccZ);
    }

    counter++;
  }
  
  else {
    float AccX = (Wire.read() << 8 | Wire.read()) / 16384.0 * GRAVITY - initialAccX;
    float AccY = (Wire.read() << 8 | Wire.read()) / 16384.0 * GRAVITY - initialAccY;
    float AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0 * GRAVITY - initialAccZ;

    long currentTime = millis();                          
    float delTime = (currentTime - prevTime) / 1000.0;  

    // position data is updated by integrating instantaneous acceleration twice. This gives the formula del_x  = 1/2 * a * t^2 + v*t which is added to the position
    speedX += AccX * delTime;
    speedY += AccY * delTime;
    speedZ += AccZ * delTime;

    posX += 0.5 * AccX * delTime * delTime + speedX * delTime ;
    posY += 0.5 * AccY * delTime * delTime + speedY * delTime ;
    posZ += 0.5 * AccZ * delTime * delTime + speedZ * delTime ;

    prevTime = currentTime; 

    // PRINT STATEMENTS
    if (millis() - printTime > PRINT_DELAY) {
      // printAcc (AccX, AccY, AccZ);
      // printInitial (initialAccX, initialAccY, initialAccZ);
      printPos (posX, posY, posZ);

      printTime = millis();
    }

    delay (10);
  }

  Wire.endTransmission();
  
  delay (TESTING_DELAY);
}

void printAcc (float AccX, float AccY, float AccZ) {
  Serial.print("Acceleration (X, Y, Z): ");
  Serial.print(AccX);
  Serial.print(" ");
  Serial.print(AccY);
  Serial.print(" ");
  Serial.println(AccZ);
}

void printInitial (float initialAccX, float initialAccY, float initialAccZ) {
  Serial.print ("Initial X: ");
  Serial.println (initialAccX);
  Serial.print ("Initial Y: ");
  Serial.println (initialAccY);
  Serial.print ("Initial Z: ");
  Serial.println (initialAccZ);
  Serial.println();
}

void printPos (float posX, float poxY, float posZ) {
  Serial.print("Position (X, Y, Z): ");
  Serial.print(posX);
  Serial.print(", ");
  Serial.print(posY);
  Serial.print(", ");
  Serial.println(posZ);
}

