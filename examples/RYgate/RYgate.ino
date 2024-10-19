#include <Qbead.h>

Qbead::Qbead bead;

void setup() {
    Serial.begin(9600);
    while (!Serial);

    Serial.println("Starting Qbead...");

    bead.begin();

    // Initialize the state
    bead.state.setPhi(0);
    bead.state.setTheta(90);
    bead.setBloch_deg_smooth(bead.state.getTheta(), bead.state.getPhi(), color(255, 0, 255));
    bead.state.printState();
    bead.show();
}

void loop() {
    bead.clear();

    // Apply the X gate to the internal state
    bead.state.RYgate(90);

    // Print the updated state
    bead.state.printState();

    // Visualize the new state
    bead.setBloch_deg_smooth(bead.state.getTheta(), bead.state.getPhi(), color(255, 0, 255));
    bead.show();

    delay(5000);
}
