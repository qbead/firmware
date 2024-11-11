#include <Arduino.h>
#include "Qbead.h"

Qbead bead;
int part = 0;

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


//In this example the goal is to learn about the basis states
//The QBead will display a state and another needs to be found
void loop() {
    bead.readIMU();
    bead.clear();

    switch(part) {
      case 0: findKetOne(); break;
      case 1: findHalf(); break;
      case 2: findPlusI(); break;
      case 3: findMinusI(); break;
      default: reset();
    }

    
    bead.show();
    delay(50); 
}

//|0> is lighted up, find |1>
void findKetOne() {
  float theta = bead.getState().getTheta();
    float phi = bead.getState().getPhi();

    if (theta >= 170 && (phi > 340 || phi < 20)) {
      for (int i = 0; i < 4; i++) {
        for (int i = 0; i < bead.pixels.numPixels(); i++) {
            bead.pixels.setPixelColor(i, color(255, 255, 255)); // White color
        }
        bead.show();
        delay(150);
        bead.clear();
        bead.show();
        delay(150);
      }
      part++;
      delay(1000);
    } else {
      bead.setBloch_deg_smooth(bead.getState().getTheta(), bead.getState().getPhi(), color(255, 0, 255));
      bead.setBloch_deg_smooth(0, 0, color(0, 0, 255));
    }
}

//|0> and |+> lighted up, find |->
void findHalf() {
    float theta = bead.getState().getTheta();
    float phi = bead.getState().getPhi();

    if ((theta >= 85 && theta <= 95) && (phi >= 170 && phi <= 190)) {
      for (int i = 0; i < 4; i++) {
        for (int i = 0; i < bead.pixels.numPixels(); i++) {
            bead.pixels.setPixelColor(i, color(255, 255, 255)); // White color
        }
        bead.show();
        delay(150);
        bead.clear();
        bead.show();
        delay(150);
      }
      part++;
      delay(1000);
    } else {
      bead.setBloch_deg_smooth(bead.getState().getTheta(), bead.getState().getPhi(), color(255, 0, 255));
      bead.setBloch_deg_smooth(0, 0, color(0, 0, 255));
      bead.setBloch_deg_smooth(90, 0, color(255, 0, 0));
    }
}

//|-> and |+> lighted up, find |+i>
void findPlusI() {
    float theta = bead.getState().getTheta();
    float phi = bead.getState().getPhi();

    if ((theta >= 85 && theta <= 95) && (phi >= 80 && phi <= 100)) {
      for (int i = 0; i < 4; i++) {
        for (int i = 0; i < bead.pixels.numPixels(); i++) {
            bead.pixels.setPixelColor(i, color(255, 255, 255)); // White color
        }
        bead.show();
        delay(150);
        bead.clear();
        bead.show();
        delay(150);
      }
      part++;
      delay(1000);
    } else {
      bead.setBloch_deg_smooth(bead.getState().getTheta(), bead.getState().getPhi(), color(255, 0, 255));
      bead.setBloch_deg_smooth(90, 0, color(0, 0, 255));
      bead.setBloch_deg_smooth(90, 180, color(255, 0, 0));
    }
}

//|-> and |+> lighted up, find |-i>
void findMinusI() {
    float theta = bead.getState().getTheta();
    float phi = bead.getState().getPhi();

    if ((theta >= 85 && theta <= 95) && (phi >= 260 && phi <= 280)) {
      for (int i = 0; i < 4; i++) {
        for (int i = 0; i < bead.pixels.numPixels(); i++) {
            bead.pixels.setPixelColor(i, color(255, 255, 255)); // White color
        }
        bead.show();
        delay(150);
        bead.clear();
        bead.show();
        delay(150);
      }
      part++;
      delay(1000);
    } else {
      bead.setBloch_deg_smooth(bead.getState().getTheta(), bead.getState().getPhi(), color(255, 0, 255));
      bead.setBloch_deg_smooth(90, 0, color(0, 0, 255));
      bead.setBloch_deg_smooth(90, 180, color(255, 0, 0));
    }
}


void reset() {
  part = 0;
}