#include <Arduino.h>
#include <Qbead_init.h>

Qbead::Qbead bead;

void setup() {
  Serial.println("start");
  bead.begin();
  Serial.println("1");
  bead.setBrightness(25); // way too bright
  for (int i = 0; i < bead.pixels.numPixels(); i++) {
    bead.pixels.setPixelColor(i, color(255, 255, 255));
    bead.pixels.show();
    delay(5);
  }
  Serial.println("2");
  for (int phi = 0; phi < 360; phi += 30) {
    for (int theta = 0; theta < 180; theta += 3) {
      bead.clear();
      bead.setBloch_deg(theta, phi, colorWheel(phi));
      bead.show();
    }
  }
  Serial.println("3");
}

void loop() {
  bead.readIMU();

  bead.clear();
  bead.setBloch_deg_smooth(bead.t, bead.p, color(255, 0, 255));
  bead.show();
  delay(10);
}