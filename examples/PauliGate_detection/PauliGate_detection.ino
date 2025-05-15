#include <Qbead.h>

Qbead::Qbead bead;
Qbead::QuantumState state = Qbead::QuantumState(Qbead::Coordinates(1, 2.3));
bool freeze = 0;
float timeIMU = 0;
float counter = 0;
uint32_t cooldownColor = color(255, 255, 255);


void setup() {
  bead.begin();
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
  bead.showAxis();
  if (freeze)
  {
    cooldownColor = color(255, 255, 0);
    float newTime = micros();
    counter += newTime - timeIMU;
    timeIMU = newTime;
    if (counter > 3000000)
    {
      counter = 0;
      freeze = 0;
    }
  } else
  {
    cooldownColor = color(255, 255, 255);
    freeze = bead.checkRotation(state);
    timeIMU = micros();
  }
  bead.setLed(state.getCoordinates(), cooldownColor);
  bead.show();
}