#include <Arduino.h>
#include "Qbead.h"
#include "Helpers.h"

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

void loop() {
    bead.readIMU();
    bead.clear();
    bead.setBloch_deg_smooth(bead.getState().getTheta(), bead.getState().getPhi(), color(255, 0, 255));
    bead.show();
    delay(50); 
}
