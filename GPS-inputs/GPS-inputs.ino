#include <TinyGPS++.h>
#include <SoftwareSerial.h> 
#include <math.h>

// software based serial communication. 
// pins 0 and 1 on the UNO are for serial communications but this allows for other digital pins to be used
// Since the serial pins are used to communicate via USB to the computer, using SoftwareSerial helps avoid this potential interference
// AVOID using the RX and TX (0 and 1 ) pins since they give the following error >>>>>   avrdude: stk500_getsync() attempt 1 of 10: not in sync: resp=0x00

const int RXPin = 3; // Rx (Recive) pin on Arduino Uno connected to TX pin of GPS module
const int TXPin = 4; // Tx (Transmit) pin on Arduino Uno connected to RX pin of GPS module
float refLat, refLng;

TinyGPSPlus gps; // Creates a TinyGPS+ object

// Creates a class called 'ss' which has some methods. The ones used are .available() which checks how many bytes
SoftwareSerial ss(RXPin, TXPin); 

void setup() {
  Serial.begin(9600);
  ss.begin(9600);

  Serial.println(F("BN-880 GPS Module Example"));
}

void loop() {
  while (ss.available() > 0) {
    char c = ss.read();
    // Serial.print(c); // Print raw NMEA sentences

    if (gps.encode(c)) {
      double lat = gps.location.lat();
      double lng = gps.location.lng();
      float alt = gps.altitude.meters();

      printGPSInfo (lat, lng, alt);  
      
      // If the reference coordinates have not been set, use the first GPS reading as the reference
      if (isnan(refLat) || isnan(refLng) ) {
            refLat = lat;
            refLng = lng;
      }

      float course = gps.courseTo(refLat, refLng, lat, lng);
      float distance = gps.distanceBetween(refLat, refLng, lat, lng);

      // calculating differences
      float xDistance = distance * sin(course * PI / 180.0) ;
      float yDistance = distance * cos (course * PI / 180.0);


      delay (500);
    }
  }
}

void printGPSInfo (long lat, long lng, long alt) {
  Serial.print("Latitude: ");
  Serial.println(lat);
  Serial.print("Longitude: ");
  Serial.println(lng);
  Serial.println();
  Serial.println(alt);
  Serial.println();
  Serial.println();
}

