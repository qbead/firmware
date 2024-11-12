/*
  Dynamic Decoupling

  This example simulates dynamic decoupling for dephasing at a constant rate on the Qbead.
  Here, we have represented a mixed state as the average of states on the surface
  of the Bloch Sphere.  

  For an initial theta and phi, as time passes, the spread increases, i.e., the
  neighbouring states with the same theta light up. This is meant to show that
  we have an ensemble of the lit up states. Once the spread reaches 90 degrees,
  we apply an X gate. From this point on, the spread decreases unti it reaches 0.
  Then the spread increases again and it repeats the process.

*/



#include "Qbead.h"

Qbead::Qbead qbead; // Create an instance of Qbead

float spreadRate = 10.0; // Rate at which spread(in degrees) increases
float currentSpread = 0.0; // Start with zero spread
float maximum_spread = 270; // Maximum spread (in degrees) Never reaches this since it stops at spread_flip
float quant_phi = 2.5; // Smallest step in degrees
int isX = 0; // Helps decide whether to increase or decrease the spread
float spread_flip = 90.0; // Perform X gate when reach this angle
const int number_of_states = 37; // Number of states calculated using: (int)((spread_flip/quant_phi) + 1);
Qbead::Qbead arr[number_of_states]; //Created states to show an animation of X gate being applied
float initial_theta = 90.0;  // Set initial theta
float initial_phi = 180.0; // Set initial phi
int count = 0; // Variable for blinking

void setup() {
    qbead.begin();
    qbead.state.setTheta(initial_theta); // Set initial theta for main state
    qbead.state.setPhi(initial_phi); // Set initial phi for main state

    for(int i=0;i<number_of_states;i++) // Initialize array of states for animation
    {
      arr[i].state.setTheta(initial_theta);
      float minphi = initial_phi - spread_flip / 2;
      float maxphi = initial_phi + spread_flip / 2; 
      if (minphi < 0) {
          minphi += 360;
      }
      if (maxphi > 360) {
          maxphi -= 360;
      }
      if(minphi<maxphi){
        arr[i].state.setPhi(minphi+i*quant_phi);
      } 
      else if(minphi==maxphi){

      }
      else{
        float angle = minphi+i*quant_phi;
        if(angle>=360){
          angle-=360;
        }
        arr[i].state.setPhi(angle);
      }
    }
}

void groupAni() {  // Function to animate states undergoing pi rotation about X axis
    for (int i=0; i<=6; i++){
        qbead.clear();
        
        for(int j=0;j<number_of_states;j++)
        {
          if(i>0){
          arr[j].state.RXgate(180.0/6.0);
          }
          qbead.setBloch_deg_smooth(arr[j].state.getTheta(), arr[j].state.getPhi(), color(255,255,255));
          
        }
        qbead.show();
        delay(750);
    }
}

void loop() {

  count+=250;
  qbead.clear();
  qbead.setBloch_deg_smooth(qbead.state.getTheta(), qbead.state.getPhi(), color(255,255,255)); // Display initial state

  if(count%500==0){ // For blinking, if not true then only show the initial state
    if(count%1000==0){ // If true then update the spread
      count=0; // Reset to avoid worry about overflow

      if(currentSpread >= spread_flip && isX == 0) {  // Check if X gate needs to be applied
        isX = 1;
        for(int i=0;i<number_of_states;i++){
          qbead.setBloch_deg_smooth(arr[i].state.getTheta(), arr[i].state.getPhi(), colorWheel_deg(arr[i].state.getPhi()));
        }
        qbead.show();
        groupAni();
        qbead.state.Xgate();
        updateSpreadAndLightLEDs(qbead.state.getTheta(), qbead.state.getPhi(), currentSpread);
        delay(1000);
    }
    if (isX == 0) { // Increase Spread
      currentSpread += spreadRate;
    }
    else { // Decreae spread
      currentSpread -= spreadRate;
      if(currentSpread == 0) {
        isX = 0;
      }
    }
    
    if (currentSpread > maximum_spread) {
        currentSpread = maximum_spread; // Cap at maximum spread
    }
    updateSpreadAndLightLEDs(qbead.state.getTheta(), qbead.state.getPhi(), currentSpread);

    }
    else{
      updateSpreadAndLightLEDs(qbead.state.getTheta(), qbead.state.getPhi(), currentSpread);
    }
  }
  
    qbead.show();
    // Delay to control update rate
    delay(500);
}

void updateSpreadAndLightLEDs(float theta, float init_phi, float spread) { // Function to light up all led's from (initial phi - spread/2) to (initial phi + spread/2)

    if (abs(spread-360.0)<=1e-2) { // Light up all LEDs in the plane if spread = 360 degrees
    for(float phi = 0; phi<= 360; phi+=quant_phi) {
       if(abs(phi-init_phi)<16){
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
      if (maxphi > 360) {
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

