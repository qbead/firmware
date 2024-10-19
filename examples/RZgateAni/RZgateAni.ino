#include <Qbead.h>

Qbead::Qbead bead;

void setup() {
    Serial.begin(9600);
    while (!Serial);

    Serial.println("Starting Qbead...");

    bead.begin();

    // Initialize the state
    bead.state.setPhi(90);
    bead.state.setTheta(90);
    bead.setBloch_deg_smooth(bead.state.getTheta(), bead.state.getPhi(), color(255, 0, 255));
    bead.state.printState();
    bead.show();
}

void loop() {
    bead.clear();

    // Apply the X gate to the internal state
    bead.state.RZgateAni(180, bead, 10, 2000);

    // Print the updated state
//     bead.getState().printState();

//     // Visualize the new state
//     bead.setBloch_deg_smooth(bead.getState().getTheta(), bead.getState().getPhi(), color(255, 0, 255));
//     bead.show();

     delay(2000);
 }