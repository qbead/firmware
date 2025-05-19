#include <Qbead.h>

Qbead::Qbead bead;
Qbead::QuantumState state = Qbead::QuantumState(Qbead::Coordinates(1, 2.3));
bool freeze = 0;
float timeIMU = 0;
float counter = 0;
uint32_t cooldownColor = color(255, 255, 255);
const bool toggleAnimationOn = 1;


void animationGate(Qbead::Coordinates oldPoint, int steps, int animationLength)
{
  for (int i = 0; i < steps; i++)
  {
    bead.clear();
    float q = i/ (float) steps;
    float newTheta = q*state.getCoordinates().theta() + (1-q)*oldPoint.theta();
    float newPhi = q*state.getCoordinates().phi() + (1-q)*oldPoint.phi();
    Qbead::Coordinates animationPoint = Qbead::Coordinates(newTheta, newPhi);
    bead.setLed(animationPoint, color(255, 255, 0), true);
    bead.readIMU(false);
    bead.showAxis();
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
  if (freeze)
  {
    cooldownColor = color(255, 255, 0);
    float newTime = micros();
    counter += newTime - timeIMU;
    timeIMU = newTime;
    prevInterruptCount = interruptCount;
    if (counter > 3000000)
    {
      counter = 0;
      freeze = 0;
    }
  } else
  {
    Qbead::QuantumState oldState = state;
    cooldownColor = color(255, 255, 255);
    freeze = bead.checkMotion(state);
    timeIMU = micros();
    if (freeze && toggleAnimationOn)
    {
      animationGate(oldState.getCoordinates(), 30, 4000);
    }
  }
  bead.setLed(state.getCoordinates(), cooldownColor);
  bead.show();
}