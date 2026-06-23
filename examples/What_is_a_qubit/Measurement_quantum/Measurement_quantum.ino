#include <QBead.h>

Qbead::Qbead bead;


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



  float phi = bead.p_acc;
  bead.setBloch_deg(90, phi, color(255, 0, 255));
    
  bead.show();
  float t_past = bead.t_acc;
  float p_past = bead.p_acc;
  delay(200);

  bead.readIMU();
  
  if (abs(sqrt(bead.x*bead.x + bead.y*bead.y + bead.z*bead.z)) > 2){ // magnitude of accel vector above 2G
    bead.clear();
    int meas = random(0,2);
    DisplayMeasurement(meas);
    if (meas == 1){ // random number 0 or 100
      bead.setBloch_deg(0, 90, color(0,255,0)); // not smooth for single pixel measurement
    } else {
      bead.setBloch_deg(180, 90, color(255,0,0)); // invert the angles. theta is 0-180 and phi 0-360
    }
    bead.show();
    bead.sendMeasurement(meas);
    delay(2000);
  }

}

void DisplayMeasurement(int state){
    for (int i=2; i<5; i++){
      bead.setLegPixelColor(1, i, color(155,155,155));
    }
    bead.setLegPixelColor(5, 2, color(155,155,155));
    bead.setLegPixelColor(5, 4, color(155,155,155));
    bead.setLegPixelColor(6, 3, color(155,155,155));
    if (state ==0){
      for (int i=2; i<5; i++){
      bead.setLegPixelColor(2, i, color(155,155,0));
      bead.setLegPixelColor(4, i, color(155,155,0));
      }
      bead.setLegPixelColor(3, 2, color(155,155,0));
      bead.setLegPixelColor(3, 4, color(155,155,0));
    } else {
      for (int i=2; i<5; i++){
      bead.setLegPixelColor(3, i, color(155,155,0));
      }
      bead.setLegPixelColor(2, 2, color(155,155,0));
      bead.setLegPixelColor(2, 4, color(155,155,0));
      bead.setLegPixelColor(4, 4, color(155,155,0));
    }
  }

