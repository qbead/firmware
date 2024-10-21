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
      bead.setBloch_deg(theta, phi, colorWheel(phi));
      bead.show();
    }
  }
  Serial.println("starting BLE tracking");
}

void loop() {
  bead.readIMU();
  bead.clear();
  bead.setBloch_deg_smooth(bead.t_acc, bead.p_acc, color(255, 255, 255));
  bead.setBloch_deg_smooth(90, 0, color(255, 0, 0));
  bead.setBloch_deg_smooth(90, 45, color(255, 0, 0));
  bead.setBloch_deg_smooth(90, 90, color(255, 0, 0));
  bead.setBloch_deg_smooth(90, 135, color(255, 0, 0));
  bead.setBloch_deg_smooth(90, 180, color(0, 255, 0));
  bead.setBloch_deg_smooth(90, 225, color(0, 255, 0));
  bead.setBloch_deg_smooth(90, 270, color(0, 255, 0));
  bead.setBloch_deg_smooth(90, 315, color(0, 255, 0));
  bead.show();
}
