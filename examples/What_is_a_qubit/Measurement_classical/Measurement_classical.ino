#include <Arduino.h>
#include "Qbead.h"

Qbead bead;


void setup() {
    Serial.begin(9600);
    while (!Serial);

    Serial.println("Starting Qbead...");

    bead.begin();

    // Light up all LEDs briefly to confirm they work
    bead.clear();
    for (int i = 0; i < bead.pixels.numPixels(); i++) {
        bead.pixels.setPixelColor(i, color(255, 255, 255)); // White color
    }
    bead.show();
    //delay(50);
    bead.clear();
    bead.show();
}


//In this example the states of the QBead are restricted to 0 and 1
//The goal is to give an understanding of measurement in these states and the result
void loop() {
    bead.readIMU();
    bead.clear();

    float theta = bead.getState().getTheta();
    float phi = bead.getState().getPhi();
    if (theta <= 90) {
      bead.setBloch_deg_smooth(0, 90, color(255, 0, 255));
      Serial.println(0);
    }
    else {
      bead.setBloch_deg_smooth(180, 90, color(255, 0, 255));
      Serial.println(1);
    }
    
    if ((bead.getState().getX() * bead.getState().getX() + 
          bead.getState().getY() * bead.getState().getY() + 
          bead.getState().getZ() * bead.getState().getZ()) > 2) {
        Serial.println("Shaked");
        displayMeasurement();
    }

    bead.show();
    delay(50); 
}

void displayMeasurement() {
  //waiting for working QBead
  delay(5000);
}