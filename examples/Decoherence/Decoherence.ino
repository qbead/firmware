#include <Qbead.h>

Qbead::Qbead bead;
int rotationState = 0;
uint32_t stateColor = color(255, 255, 255);
uint32_t decoherenceColor = color(122, 0, 122);
const bool toggleAnimationOn = 1;
Qbead::Coordinates oldCoordinates(0, 0, 1);
int t = 0;
bool wasFrozen = false;

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
    oldCoordinates = bead.state.getCoordinates();
    t = millis();
}

void loop()
{
    bead.readIMU(true);
    bead.clear();
    bead.showAxis();
    stateColor = color(255, 255, 255);
    Serial.print("rotationState: ");
    Serial.println(rotationState);
    if (bead.frozen)
    {
        stateColor = color(122, 122, 0);
        wasFrozen = true;
    }
    else
    {
        if (wasFrozen)
        {
            wasFrozen = false;
            oldCoordinates = bead.state.getCoordinates();
        }
        rotationState = bead.checkMotion();
        if (rotationState != 0)
        {
            bead.frozen = true;
            bead.T_freeze = millis();
        }
        float phi = bead.state.getCoordinates().phi();
        int dt = millis() - t;
        float randInt = random(200, 10000);
        phi += dt / randInt;
        if (phi > 2 * PI)
        {
            phi -= 2 * PI;
        }
        Qbead::Coordinates newCoordinates(bead.state.getCoordinates().theta(), phi);
        bead.state.setCoordinates(newCoordinates);
        bead.visualState = bead.state.getCoordinates();
        bead.setLed(oldCoordinates, decoherenceColor);
    }
    t = millis();
    bead.animateTo(rotationState, 2000);
    bead.setLed(bead.visualState, stateColor, 1);
    bead.show();
}