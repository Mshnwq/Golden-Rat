#include <Arduino.h>

// Pin connected to the photodiode
const int photodiodePin = A0;

void setup() {
  pinMode(photodiodePin, INPUT);
  Serial.begin(9600);
  Serial.println("Obstacle Detection Ready!");
}

void loop() {
  int sensorValue = analogRead(photodiodePin);
    Serial.println(sensorValue);

  // Adjust the threshold value to determine the obstacle detection sensitivity
  if (sensorValue > 500) {
    // Add your desired actions when an obstacle is detected
  }

  delay(100); // Adjust delay as needed
}