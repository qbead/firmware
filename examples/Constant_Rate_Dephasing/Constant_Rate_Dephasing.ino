/*
  Constant Rate Dephasing

  This example simulates dephasing at a constant rate on the Qbead.
  Here, we have represented a mixed state as the average of states on the surface
  of the Bloch Sphere.  

  For an initial theta and phi, as time passes, the spread increases, i.e., the
  neighbouring states with the same theta light up. This is meant to show that
  we have an ensemble of the lit up states. Once the spread reaches 360 degrees,
  it remains there.

*/


#include "Qbead.h"

Qbead::Qbead qbead; // Create an instance of Qbead

float spreadRate = 10.0; // Rate at which spread(in degrees) increases
float currentSpread = 0.0; // Start with zero spread
float maximum_spread = 360.0; // Maximum spread (in degrees)
float quant_phi = 2.5; // Smallest step in degrees
int count = 0;  // Variable for blinking
void setup() {
    qbead.begin();
    qbead.state.setTheta(90.0); // Set initial theta
    qbead.state.setPhi(180.0); // Set initial phi
}


void loop() {
    count+=250;
    qbead.clear();
    qbead.setBloch_deg_smooth(qbead.state.getTheta(), qbead.state.getPhi(), color(255,255,255));  // Display initial state

    
    if (count%500==0){  // For blinking, if not true then only show the initial state
      if (count%1000==0){  // If true then update the spread
        count = 0; // Reset to avoid worry about overflow
        currentSpread += spreadRate;

        // Check if we exceed maxSpread
        if (currentSpread >= maximum_spread) {
            currentSpread = maximum_spread; // Cap at maximum spread
        }
        updateSpreadAndLightLEDs(qbead.state.getTheta(), qbead.state.getPhi(), currentSpread);

      }
      else{  // Display the current state and spread
        updateSpreadAndLightLEDs(qbead.state.getTheta(), qbead.state.getPhi(), currentSpread);
      }

    }
    qbead.show();  

    // Delay to control update rate
    delay(500);
}

void updateSpreadAndLightLEDs(float theta, float init_phi, float spread) {  // Function to light up all led's from (initial phi - spread/2) to (initial phi + spread/2)

    if (abs(spread-360) <= 1e-2) { // Light up all LEDs in the plane if spread = 360 degrees
    for(float phi = 0; phi<= 360; phi+=quant_phi) {
      if(abs(phi-init_phi)<1e-2){
        continue;
      }
        qbead.setBloch_deg_smooth(theta, phi, colorWheel_deg(phi));
    }
    }

    else if (abs(spread-0.0)>=1e-2){ // If spread is not zero
    
      float minphi = init_phi - spread / 2.0;
      float maxphi = init_phi + spread / 2.0;

      // Adjust for wraparound for cases like when minphi = -30 degrees and maxphi = 30 degrees or minphi = 270 degrees and maxphi = 390 degrees
      if (minphi < 0) {
          minphi += 360;
      }
      if (maxphi >= 360) {
          maxphi -= 360;
      }

      if(minphi<maxphi) {
        for(float phi = minphi; phi<= maxphi; phi+=quant_phi) {
          if(abs(phi-init_phi)<16){
            continue;
          }
          qbead.setBloch_deg_smooth(theta, phi, colorWheel_deg(phi));
        }
      }

      else {
        for(float phi = minphi; phi<= 360; phi+=quant_phi) {
          if(abs(phi-init_phi)<16){
            continue;
          }
          qbead.setBloch_deg_smooth(theta, phi, colorWheel_deg(phi));
        }
        for(float phi = 0; phi<= maxphi; phi+=quant_phi) {
          if(abs(phi-init_phi)<16){
            continue;
          }
          qbead.setBloch_deg_smooth(theta, phi, colorWheel_deg(phi));
        }
      }
    }
}

