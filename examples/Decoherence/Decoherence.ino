#include <Qbead.h>

Qbead::Qbead bead;
int rotationState = 0;
uint32_t stateColor = color(255, 255, 255);
const bool toggleAnimationOn = 1;
int lastCollapseTime = 0;

void setup()
{
    bead.begin();
    bead.setBrightness(25);
    Serial.println("testing all pixels discretely");
    for (int i = 0; i < bead.pixels.numPixels(); i++)
    {
        bead.pixels.setPixelColor(i, color(255, 255, 255));
        bead.pixels.show();
        delay(5);
    }
    Serial.println("testing smooth transition between pixels");
    for (int phi = 0; phi < 360; phi += 30)
    {
        for (int theta = 0; theta < 180; theta += 3)
        {
            bead.clear();
            bead.setBloch_deg(theta, phi, colorWheel_deg(phi));
            bead.show();
        }
    }
    Serial.println("starting inertial tracking");
    lastCollapseTime = millis();
}

void loop()
{
    bead.readIMU(false);
    bead.clear();
    bead.showAxis();
    stateColor = color(255, 255, 255);
    Serial.print("rotationState: ");
    Serial.println(rotationState);
    if (bead.frozen)
    {
        stateColor = color(122, 122, 0);
    }
    else
    {
        rotationState = bead.checkMotion();
        if (rotationState != 0)
        {
            bead.frozen = true;
            bead.T_freeze = millis();
        }
        if (rotationState == 8)
        {
            lastCollapseTime = millis();
        }
    }
    int currentTime = millis();
    // Increment the amount of decoherence based on the time since the last collapse
    // This is to show decoherence
    float decoherence = (currentTime - lastCollapseTime) / 2000;
    bead.animateTo(rotationState, 2000);
    bead.setLed(bead.visualState, stateColor, decoherence);
    bead.show();
}