/*
  Dephasing

  This example simulates dephasing on the Qbead.
  Here, we have represented a mixed state as the average of states on the surface
  of the Bloch Sphere.  

  For an initial theta and phi, as time passes, the spread increases, i.e., the
  neighbouring states with the same theta light up. This is meant to show that
  we have an ensemble of the lit up states. Once the spread reaches 360 degrees,
  it remains there.

  The spread is calculated using the formula, sin(phi_s/2)/(phi_s/2) = e^(-gamma*t)
  where phi_s is the spread and gamma is the dephasing rate. In this case, we have used 
  gamma = 0.25. After 180 degrees(or pi radians), we keep a constant rate so that we don't
  have to store a lot of values.

  The python notebook to calculate the values is included in the library.

*/




#include "Qbead.h"

Qbead::Qbead qbead; // Create an instance of Qbead

float currentSpread = 0.0; // Start with zero spread
float maximum_spread = 360.0; // Maximum spread (in degrees)
float quant_phi = 2.5; // Smallest step in degrees

int timeIndex = 0; // To store current index

// Time values in seconds
float timings[] = {0.0, 0.0, 0.06060606060606061, 0.12121212121212122, 0.18181818181818182, 0.24242424242424243, 0.30303030303030304, 0.36363636363636365, 
0.42424242424242425, 0.48484848484848486, 0.5454545454545454, 0.6060606060606061, 0.6666666666666667, 0.7272727272727273, 0.7878787878787878, 0.8484848484848485, 
0.9090909090909092, 0.9696969696969697, 1.0303030303030303, 1.0909090909090908, 1.1515151515151516, 1.2121212121212122, 1.2727272727272727, 1.3333333333333335, 
1.393939393939394, 1.4545454545454546, 1.5151515151515151, 1.5757575757575757, 1.6363636363636365, 1.696969696969697, 1.7575757575757576, 1.8181818181818183, 
1.878787878787879, 1.9393939393939394, 2.0, 2.0606060606060606, 2.121212121212121, 2.1818181818181817, 2.2424242424242427, 2.303030303030303, 2.3636363636363638, 
2.4242424242424243, 2.484848484848485, 2.5454545454545454, 2.606060606060606, 2.666666666666667, 2.7272727272727275, 2.787878787878788, 2.8484848484848486, 
2.909090909090909, 2.9696969696969697, 3.0303030303030303, 3.090909090909091, 3.1515151515151514, 3.2121212121212124, 3.272727272727273, 3.3333333333333335, 
3.393939393939394, 3.4545454545454546, 3.515151515151515, 3.5757575757575757, 3.6363636363636367, 3.6969696969696972, 3.757575757575758, 3.8181818181818183, 
3.878787878787879, 3.9393939393939394, 4.0, 4.0606060606060606, 4.121212121212121, 4.181818181818182, 4.242424242424242, 4.303030303030303, 4.363636363636363, 
4.424242424242425, 4.484848484848485, 4.545454545454546, 4.606060606060606, 4.666666666666667, 4.7272727272727275, 4.787878787878788, 4.848484848484849, 4.909090909090909, 
4.96969696969697, 5.03030303030303, 5.090909090909091, 5.151515151515151, 5.212121212121212, 5.2727272727272725, 5.333333333333334, 5.3939393939393945, 5.454545454545455, 
5.515151515151516, 5.575757575757576, 5.636363636363637, 5.696969696969697, 5.757575757575758, 5.818181818181818, 5.878787878787879, 5.9393939393939394, 6.0}; 

// Spread values in radians
float spreads[] = {3.793041647370262e-08, 3.793041647370262e-08, 0.6021085912808255, 0.8502162058331857, 1.0397117478874056, 1.1987224770335831, 1.33816100925298, 
1.4636326807198032, 1.5784723712841007, 1.6848561942790796, 1.784297927173516, 1.8779013676774756, 1.9665010167184558, 2.050746146241948, 2.1311538407377633, 
2.208143960153603, 2.2820630319599533, 2.3532010790234077, 2.421803780947228, 2.4880814600898487, 2.5522158508927784, 2.614365286709785, 2.674668734376798, 
2.733248974952371, 2.7902151417679977, 2.845664767840813, 2.8996854539144565, 2.952356239735994, 3.003748740714382, 3.0539280972812586, 3.1029537733847414, 
3.1508802324553873, 3.197757513093957, 3.2436317221057207, 3.2886621607965836, 3.333574254988394, 3.3784863491802044, 3.423398443372015, 3.4683105375638257, 
3.5132226317556365, 3.558134725947447, 3.6030468201392574, 3.647958914331068, 3.6928710085228786, 3.737783102714689, 3.7826951969065, 3.8276072910983103, 
3.8725193852901207, 3.917431479481931, 3.962343573673742, 4.007255667865552, 4.052167762057363, 4.097079856249174, 4.141991950440984, 4.1869040446327945, 
4.231816138824605, 4.276728233016415, 4.321640327208226, 4.366552421400037, 4.411464515591847, 4.456376609783658, 4.501288703975469, 4.54620079816728, 4.59111289235909, 
4.6360249865509005, 4.68093708074271, 4.725849174934521, 4.770761269126332, 4.815673363318142, 4.860585457509953, 4.905497551701764, 4.950409645893574, 4.995321740085385, 
5.0402338342771955, 5.085145928469006, 5.130058022660817, 5.174970116852627, 5.219882211044438, 5.264794305236249, 5.309706399428059, 5.354618493619869, 5.399530587811681, 
5.444442682003491, 5.4893547761953005, 5.534266870387111, 5.579178964578922, 5.624091058770732, 5.669003152962543, 5.713915247154354, 5.758827341346165, 5.803739435537976, 
5.848651529729786, 5.8935636239215965, 5.938475718113407, 5.983387812305217, 6.028299906497027, 6.073212000688839, 6.118124094880649, 6.163036189072459, 6.20794828326427, 6.252860377456081}; 

int csvSize = 101; // Number of entries in the arrays
int count = 0; // Variable for blinking

void setup() {
    qbead.begin();
    // Light up LEDs for test
    qbead.state.setTheta(90.0); // Set initial theta
    qbead.state.setPhi(180.0); // Set initial phi
    qbead.setBrightness(25); // way too bright
    for (int i = 0; i < qbead.pixels.numPixels(); i++) {
      qbead.pixels.setPixelColor(i, color(255, 255, 255));
      qbead.pixels.show();
      delay(5);
    }
    for (int phi = 0; phi < 360; phi += 30) {
      for (int theta = 0; theta < 180; theta += 3) {
        qbead.clear();
        qbead.setBloch_deg(theta, phi, colorWheel(phi));
        qbead.show();
      }
    }
    qbead.clear();
}

void loop() {
    int delayTime = 0;
    if (timeIndex>=csvSize){ // Start Blinking after spread is 360 degrees
      count+=250;
      delayTime = 500;
    }
    else{
      count = 1000;
    }

    qbead.clear();
    qbead.setBloch_deg_smooth(qbead.state.getTheta(), qbead.state.getPhi(), color(255,255,255)); // Display initial state
    
    if (count%500==0){ // For blinking, if not true then only show the initial state
      if (count%1000==0){ // If true then update the spread
        count=0; // Reset to avoid worry about overflow

        if (timeIndex < csvSize) { // If in array
            // Get the current spread and timing
            currentSpread = spreads[timeIndex] * 180.0/PI; //Convert to degrees
            
            if (timeIndex>=1){
              delayTime = (int)((timings[timeIndex] - timings[timeIndex-1]) * 1000 * 10); // Convert to milliseconds(*1000) and slow down 10 times(*10)
            }
            else{
              delayTime = (int)(timings[timeIndex] * 1000 * 10); // Convert to milliseconds(*1000) and slow down 10 times(*10)
            }
            updateSpreadAndLightLEDs(qbead.state.getTheta(), qbead.state.getPhi(), currentSpread);

            timeIndex++; // Move to the next entry in the array
        } 
        else {
            // If we've reached the end of the data, keep same spread(360 degrees)
            currentSpread = 360.0;
            updateSpreadAndLightLEDs(qbead.state.getTheta(), qbead.state.getPhi(), currentSpread);


        }
      }
      else{
        updateSpreadAndLightLEDs(qbead.state.getTheta(), qbead.state.getPhi(), currentSpread);
      }
    }

    qbead.show();
    // Delay to control update rate
    delay(delayTime);
}

void updateSpreadAndLightLEDs(float theta, float init_phi, float spread) { // Function to light up all led's from (initial phi - spread/2) to (initial phi + spread/2)

    if (abs(spread -360.0)<=1.0) { // Light up all LEDs in the plane if spread = 360 degrees
    for(float phi = 0; phi<= 360; phi+=quant_phi) {
      if(abs(phi-init_phi)<16){
            continue;
          }
        qbead.setBloch_deg_smooth(theta, phi, colorWheel_deg(phi));
    }
    }

    else if(abs(spread-0.0)>=1e-2) { // If spread is not zero
      
      float minphi = init_phi - (spread / 2.0);
      float maxphi = init_phi + (spread / 2.0);

      // Adjust for wraparound for cases like when minphi = -30 degrees and maxphi = 30 degrees or minphi = 270 degrees and maxphi = 390 degrees
      if (minphi < 0.0) {
          minphi += 360.0;
      }
      if (maxphi > 360.0) {
          maxphi -= 360.0;
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