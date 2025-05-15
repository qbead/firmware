#include <Qbead.h>

Qbead::Qbead bead;

void setup() {
  bead.begin();
  bead.setBrightness(25);  // way too bright
  Serial.println("testing all pixels discretely");
  for (int i = 0; i < bead.pixels.numPixels(); i++) {
    bead.pixels.setPixelColor(i, color(255, 255, 255));
    bead.pixels.show();
    delay(5);
  }
  Serial.println("testing smooth transition between pixels");
  for (int phi = 0; phi < 360; phi += 30) {
    for (int theta = 0; theta < 180; theta += 3) {
      bead.clear();
      bead.setBloch_deg(theta, phi, colorWheel_deg(phi));
      bead.show();
    }
  }
  Serial.println("starting inertial tracking");
}

void loop() {
  bead.readIMU();
  bead.clear();
  bead.showAxis();
  bead.setBloch_deg(120, 120, color(255, 0, 255));
  bead.show();
}