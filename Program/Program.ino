#include <TinyGPS++.h>              // Parses GPS NMEA strings for easy to use outputs
#include <SoftwareSerial.h>         // Allows for TX and RX pins on the IC to be connected to digital pins on the Arduino
#include <Adafruit_HMC5883_U.h>     // Use with the magnetometer. See http://adafruit.github.io/Adafruit_HMC5883_Unified/html/class_adafruit___h_m_c5883___unified.html#a7676ad9bfb73590ef1089e5829734c91
#include <Wire.h>
#include <math.h>
#include <SD.h>
#include <SPI.h>
#include <LiquidCrystal.h>

#define MAX_SONAR_VAL 675

// Display initialization
const int rs = 1, en = 2, d4 = 4, d5 = 5, d6 = 6, d7 = 7;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

// Sonar pin numbers
const int trigPin = 8;
const int echoPin = 9;

// defines sonar variables
long duration;
float depth;
float prevDepth;

const int RXPin = 3;  // Connects to the TXPin on GPS 
const int TXPin = 4;  // Connects to the RXPin on GPS

//  New x-y positions are calculated with these longitude and lattitude values as reference
float refLat, refLng; 
float xDistance;
float yDistance;

double lat;  
double lng;

// Setup GPS and software serial
TinyGPSPlus gps;
SoftwareSerial ss(RXPin, TXPin);

// SD card
File myFile;
const int chipSelect = 10;

void setup() {
  Serial.begin(9600);
  ss.begin (9600); // Start serial communications with the GPS

  lcd.begin(16, 2);

  pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin, INPUT); // Sets the echoPin as an Input

  pinMode (10, OUTPUT); // Reserves pin ten as an output for use with SD card
  
  // // Initialize the SD card
  // if (SD.begin(chipSelect)){
  //   Serial.println("SD card is present & ready");
  // } 
  // else {
  //   Serial.println("SD card missing or failure");
  //   while(1); //halt program
  // }
  // //clear out old data file
  // if (SD.exists("csv.txt")) {
  //   Serial.println("Removing simple.txt");
  //   SD.remove("csv.txt");
  //   Serial.println("Done");
  // } 

  // //write csv headers to file:
  //  myFile = SD.open("csv.txt", FILE_WRITE);  
  //  if (myFile) // it opened OK
  //   {
  //   Serial.println("Writing headers to csv.txt");
  //   myFile.println("lattitude, longitude, depth");
  //   myFile.close(); 
  //   Serial.println("Headers written");
  //   }
  // else 
  //   Serial.println("Error opening csv.txt");  
}

void loop() {

  // =======================  SONAR ================================ //

  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);   // Reads the echoPin, returns the sound wave travel time in microseconds
  depth = duration * 0.0341 / 2;   // Calculating the distance

  if (depth == 0 || depth > MAX_SONAR_VAL) {
    depth = prevDepth;
  }

  else {
    prevDepth = depth;
  }

  delay (100);

  // =======================  GPS ================================ //
  // Converts every GPS value to a new location on a map using the intial GPS locations as a constant reference. 

  while (ss.available() > 0) {
    char c = ss.read();
    // Serial.print(c); // Print raw NMEA sentences

    while (gps.encode(c)) {
      lat = gps.location.lat(); 
      lng = gps.location.lng();
      float speed = gps.speed.mps();
      int alt = gps.altitude.meters();
      
      // If the reference coordinates have not been set, use the first GPS reading as the reference
      if (isnan(refLat) || isnan(refLng) ) {
            refLat = lat;
            refLng = lng;
      }
    }
  
  }

  // =======================  LOGGING DATA  ================================ //
  printData (lat,lng,depth);
  
  // open the file. note that only one file can be open at a time,
  myFile = SD.open("csv.txt", FILE_WRITE);     
  // if the file opened okay, write to it:
  if (myFile) 
  {
    Serial.println("Writing to csv.txt");
    myFile.print (lat, 6); myFile.print (",");  
    myFile.print (lng, 6); myFile.print (","); 
    myFile.println (depth);
    myFile.close();
  } 
  else 
  {
    // Serial.println("error opening csv.txt");
  }
  delay(200);  
}

void printData (double lat,double lng, double depth) {
  Serial.print (lat, 6); Serial.print (","); 
  Serial.print (lng, 6); Serial.print (","); 
  Serial.println(depth);
  
  lcd.setCursor(0,0);
  // lcd.print("Coordinates: "); lcd.print(lat); lcd.print(", "); lcd.print(lng);
  // lcd.setCursor(0,1);
  lcd.print("Depth: ");
}
