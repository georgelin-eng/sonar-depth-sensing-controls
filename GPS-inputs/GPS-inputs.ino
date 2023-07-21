#include <TinyGPS++.h>
#include <SoftwareSerial.h> 
#include <Wire.h>
#include <Adafruit_HMC5883_U.h>
#include <math.h>

#define pi 3.14159265358979323846

// software based serial communication. 
// pins 0 and 1 on the UNO are for serial communications but this allows for other digital pins to be used
// Since the serial pins are used to communicate via USB to the computer, using SoftwareSerial helps avoid this potential interference
// AVOID using the RX and TX (0 and 1 ) pins since they give the following error >>>>>   avrdude: stk500_getsync() attempt 1 of 10: not in sync: resp=0x00

const int RXPin = 3; // Rx (Recive) pin on Arduino Uno connected to TX pin of GPS module
const int TXPin = 4; // Tx (Transmit) pin on Arduino Uno connected to RX pin of GPS module
float refLat, refLng;
float heading;

float* fxDist;
float* fyDist;
float xDist;
float yDist;

int movingAverageCounter = 0;
const int windowSize = 15;

TinyGPSPlus gps; // Creates a TinyGPS+ object
SoftwareSerial ss(RXPin, TXPin); // creates SoftwareSerial class

Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345); // Replace with your sensor's I2C address


void setup() {
  Serial.begin(9600);
  ss.begin(9600);

  allocateArrays(); // Array used for the moving average filter

  if (!mag.begin()) {
    Serial.println("Could not find a valid HMC5883L sensor, check wiring!");
    while (1);
  }
}

void loop() {
  // --------- MAGNETORMETER SECTION -----------  // 
  
  heading = getHeading();
  // Serial.print("Heading: "); Serial.print(heading); Serial.println(" degrees");
  // delay(200); // Adjust the delay as needed

  // --------- GPS MODULE SECTION ------------   //
  
  while (ss.available() > 0) {
    char c = ss.read();
    // Serial.print(c); // Print raw NMEA sentences

    while (gps.encode(c)) {
      double lat = gps.location.lat();
      double lng = gps.location.lng();
      float speed = gps.speed.mps();
      int alt = gps.altitude.meters();

      printGPSInfo (lat, lng, speed, heading, alt);  
      
      // If the reference coordinates have not been set, use the first GPS reading as the reference
      if (isnan(refLat) || isnan(refLng) ) {
            refLat = lat;
            refLng = lng;
      }

    float dist = distance (lat, lng, refLat, refLng, 'K') / 1000.0;
    xDist = dist * cos (heading);
    yDist = dist * sin (heading);

    // Serial.print (dist); Serial.println ("m");
    // Serial.print (heading); Serial.println ("degrees");

    // // Serial.print ("Position: "); 
    // Serial.print (xDist); Serial.print (","); Serial.println (yDist);

    }
  
  }

  if (movingAverageCounter <= windowSize) {    
    fxDist[movingAverageCounter] = xDist;
    fyDist[movingAverageCounter] = yDist;

    movingAverageCounter++;
  }

  else {
    float SMA_xDist = arrayAverage(fxDist, windowSize);
    float SMA_yDist = arrayAverage(fyDist, windowSize);

    shiftArrayLeft(fxDist, windowSize);
    shiftArrayLeft(fyDist, windowSize);

    fxDist[windowSize - 1] = xDist;
    fyDist[windowSize - 1] = yDist;

    // Serial.print (SMA_xDist); Serial.print (","); Serial.println (SMA_yDist);
  }

  delay (100);
}

void allocateArrays() {
  fxDist = new float[windowSize];
  fyDist = new float[windowSize];
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

float getHeading () {
  sensors_event_t event;
  mag.getEvent(&event);

  float heading = atan2(event.magnetic.y, event.magnetic.x) * 180.0 / PI;
  if (heading < 0) {
    heading += 360.0;
  }

  return heading;
}

void printGPSInfo (double lat, double lng, float speed, int heading, int alt) {
  Serial.print("Latitude: ");
  Serial.println(lat, 6);
  Serial.print("Longitude: ");
  Serial.println(lng, 6);
  Serial.print("Alt: ");
  Serial.println(alt);
  Serial.print("Speed: ");
  Serial.println(speed);
  Serial.print("heading: ");
  Serial.println(heading);
  Serial.println();
}

double distance(double lat1, double lon1, double lat2, double lon2, char unit) {
  double theta, dist;
  if ((lat1 == lat2) && (lon1 == lon2)) {
    return 0;
  }
  else {
    theta = lon1 - lon2;
    dist = sin(deg2rad(lat1)) * sin(deg2rad(lat2)) + cos(deg2rad(lat1)) * cos(deg2rad(lat2)) * cos(deg2rad(theta));
    dist = acos(dist);
    dist = rad2deg(dist);
    dist = dist * 60 * 1.1515;
    switch(unit) {
      case 'M':
        break;
      case 'K':
        dist = dist * 1.609344;
        break;
      case 'N':
        dist = dist * 0.8684;
        break;
    }
    return (dist);
  }
}

double deg2rad(double deg) {
  return (deg * pi / 180);
}

double rad2deg(double rad) {
  return (rad * 180 / pi);
}