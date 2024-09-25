#include <Arduino.h>
#include <Qbead.h>


Qbead bead;

void setup() {
    Serial.begin(9600);
    while (!Serial);

    Serial.println("Starting Qbead...");

    bead.begin();

    // Initialize the state
    bead.getState().setPhi(0);
    bead.getState().setTheta(0);
    bead.setBloch_deg_smooth(bead.getState().getTheta(), bead.getState().getPhi(), color(255, 0, 255));
    bead.getState().printState();
    bead.show();
}

void loop() {
    bead.clear();

    // Apply the X gate to the internal state
    bead.getState().Xgate();

    // Print the updated state
    bead.getState().printState();

    // Visualize the new state
    bead.setBloch_deg_smooth(bead.getState().getTheta(), bead.getState().getPhi(), color(255, 0, 255));
    bead.show();

    delay(5000);
}
