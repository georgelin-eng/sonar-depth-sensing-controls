#include <SD.h>

const int chipSelectPin = 53; // Change this to the actual CS pin you used

void setup() {
  Serial.begin(9600);

  // Initialize the SD card
  if (!SD.begin(chipSelectPin)) {
    Serial.println("SD card initialization failed!");
    return;
  }

  Serial.println("SD card initialized.");
}

void loop() {
  // Read data from the SD card
  File dataFile = SD.open("data.txt", FILE_WRITE | O_CREAT);
  
  if (dataFile) {
    Serial.println("Data found in 'data.txt':");
    
    while (dataFile.available()) {
      Serial.write(dataFile.read());
    }
    
    dataFile.close();
  } else {
    Serial.println("Error opening 'data.txt'");
  }
  
  delay(1000);
}