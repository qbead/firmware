#include <Qbead.h>

Qbead::Qbead bead;
Qbead::QuantumState state = Qbead::QuantumState(Qbead::Coordinates(1, 0));
int rotationState;
uint32_t stateColor = color(255, 255, 255);
const bool toggleAnimationOn = 1;


void animationGate(int gateType, int steps, int animationLength)
{
  if (gateType == 5)
  {
    state.collapse();
    return;
  }
  if (!toggleAnimationOn)
  {
    steps = 1;
  }
  float stepLength = M_PI / (float) steps;
  stateColor = color(255, 0, 255);
  for (int i = 0; i < steps; i++)
  {
    bead.clear();
    bead.showAxis();
    switch (gateType)
    {
      case 1:
        state.gateX(stepLength);
        break;
      case 2:
        state.gateY(stepLength);
        break;
      case 3:
        state.gateZ(stepLength);
        break;
      case 4:
        state.gateH(stepLength);
    }
    bead.setLed(state.getCoordinates(), stateColor);
    bead.show();
    delay(animationLength/steps);
  }
}

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
  bead.readIMU(false);
  bead.clear();
  bead.showAxis();
  stateColor = color(255, 255, 255);
  rotationState = bead.checkMotion();
  if (rotationState != 0) 
  {
    animationGate(rotationState, 10, 4000);
  }
  bead.setLed(state.getCoordinates(), stateColor);
  bead.show();
}