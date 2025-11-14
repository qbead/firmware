// #include <Qbead.h>
#include <Lesson1.h>

BlochVector axisVertical, axisHorizontal;

Gyro bias;

void setup() {
  /*
  Test LEDs and initialise the vectors.
  */

  bead.begin();
  bead.setBrightness(25); // way too bright

  Serial.println("testing all pixels discretely");
  for (int i = 0; i < bead.pixels.numPixels(); i++) {
    bead.pixels.setPixelColor(i, color(255, 255, 255));
    bead.pixels.show();
    delay(5);
  }
  
  bias = calibrateGyro();

  Serial.println("starting inertial tracking");
  bead.clear();

  axisVertical.setAngles(0, 0);
  axisHorizontal.setAngles(M_PI/2, M_PI/2);

}
void loop() {

  bead.clear();
  bead.readIMU(false);

  axisHorizontal = updateHorizontalAxis(axisHorizontal, axisVertical, bead.angle_acc, bias, 2);
  axisVertical = bead.angle_acc;

  bead.setBloch_deg_smooth(axisVertical, color(0, 0, 255));
  bead.setBloch_deg_smooth(axisHorizontal, color(255, 255, 255));

  bead.show();
}