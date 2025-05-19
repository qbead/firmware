#include <Qbead.h>

Qbead::Qbead bead;

void setup() {
  bead.begin();
  bead.startAccelerometer();
  bead.setBrightness(25); // way too bright
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
  bead.setLed(Qbead::Coordinates(bead.gravity), color(255, 0, 255), true);
  bead.show();
}