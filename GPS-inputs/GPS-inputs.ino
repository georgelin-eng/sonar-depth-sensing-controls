#include <TinyGPS++.h>
#include <SoftwareSerial.h> 
#include <Wire.h>
#include <Adafruit_HMC5883_U.h>
#include <math.h>

// software based serial communication. 
// pins 0 and 1 on the UNO are for serial communications but this allows for other digital pins to be used
// Since the serial pins are used to communicate via USB to the computer, using SoftwareSerial helps avoid this potential interference
// AVOID using the RX and TX (0 and 1 ) pins since they give the following error >>>>>   avrdude: stk500_getsync() attempt 1 of 10: not in sync: resp=0x00

const int RXPin = 3; // Rx (Recive) pin on Arduino Uno connected to TX pin of GPS module
const int TXPin = 4; // Tx (Transmit) pin on Arduino Uno connected to RX pin of GPS module
// float refLat, refLng;

TinyGPSPlus gps; // Creates a TinyGPS+ object
SoftwareSerial ss(RXPin, TXPin); // creates SoftwareSerial class

Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345); // Replace with your sensor's I2C address


void setup() {
  Serial.begin(9600);
  ss.begin(9600);

  if (!mag.begin()) {
    Serial.println("Could not find a valid HMC5883L sensor, check wiring!");
    while (1);
  }
}

void loop() {
  // --------- MAGNETORMETER SECTION -----------  // 
  
  sensors_event_t event;
  mag.getEvent(&event);

  float heading = atan2(event.magnetic.y, event.magnetic.x) * 180.0 / PI;
  if (heading < 0) {
    heading += 360.0;
  }
  
  Serial.print("Heading: ");
  Serial.print(heading);
  Serial.println(" degrees");
  
  delay(1000); // Adjust the delay as needed


  // --------- GPS MODULE SECTION ------------   //
  
  // while (ss.available() > 0) {
  //   char c = ss.read();
  //   // Serial.print(c); // Print raw NMEA sentences

  //   if (gps.encode(c)) {
  //     double lat = gps.location.lat();
  //     double lng = gps.location.lng();
  //     float speed = gps.speed.mps();
  //     int heading = gps.course.deg();
  //     int alt = gps.altitude.meters();

  //     printGPSInfo (lat, lng, speed, heading, alt);  
      
  //     If the reference coordinates have not been set, use the first GPS reading as the reference
  //     if (isnan(refLat) || isnan(refLng) ) {
  //           refLat = lat;
  //           refLng = lng;
  //     }

  //     float course = gps.courseTo(refLat, refLng, lat, lng);
  //     float distance = gps.distanceBetween(refLat, refLng, lat, lng);

  //     // calculating differences
  //     float xDistance = distance * sin(course * PI / 180.0) ;
  //     float yDistance = distance * cos (course * PI / 180.0);


  //     delay (500);
  //   }
  // }
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

 