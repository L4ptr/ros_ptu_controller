// This script allows for serial communication over the usb port
#include <Adafruit_NeoPixel.h>

#define LED_PIN 0
#define TRIGGER_PIN 10

// Setting up the LEDs
Adafruit_NeoPixel strip = Adafruit_NeoPixel(1, LED_PIN, NEO_GRB + NEO_KHZ800);

bool triggerSignal = false; // Variable to store the received signal

void setup() {
  pinMode(TRIGGER_PIN, OUTPUT);
  digitalWrite(TRIGGER_PIN, LOW);
  
  strip.begin(); // Initialize LEDs
  strip.show(); // Initialize all LEDs to 'off'
  
  Serial.begin(9600); // Begin serial communication

  // Light the LED strip blue when powered
  strip.setPixelColor(0, strip.Color(0, 0, 255));
  strip.show();
}

void loop() {
  if (Serial.available() > 0) {
    triggerSignal = Serial.parseInt(); // Read the signal sent from ROS node

    if (triggerSignal) {
      // Set pin 10 high for 1ms
      digitalWrite(TRIGGER_PIN, HIGH);
      delay(1);
      digitalWrite(TRIGGER_PIN, LOW);
      
      // Change LED strip color to red
      strip.setPixelColor(0, strip.Color(255, 0, 0));
      strip.show();
      
      delay(1000);

      // Revert LED strip color back to blue
      strip.setPixelColor(0, strip.Color(0, 0, 255));
      strip.show();
    }
  }
}
