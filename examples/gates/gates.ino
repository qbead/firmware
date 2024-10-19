#include <Qbead.h>

Qbead::Qbead bead;

void setup() {
    Serial.begin(9600);
    while (!Serial);

    Serial.println("Starting Qbead...");

    bead.begin();

    // Initialize the state
    bead.state.setPhi(0); //CHANGE FOR EXAMPLE TO EXAMPLE IF YOU LIKE
    bead.state.setTheta(0); //CHANGE FOR EXAMPLE TO EXAMPLE IF YOU LIKE
    bead.setBloch_deg_smooth(bead.state.getTheta(), bead.state.getPhi(), color(255, 0, 255));
    bead.state.printState();
    bead.show();
}

void loop() {
    bead.clear();

    //uncomment whichever gate you like to apply

    // Apply the RX gate to the internal state
    bead.state.RXgate(50);

    // Apply the RY gate to the internal state
    // bead.state.RYgate(50);

    // // Apply the RZ gate to the internal state
    // bead.state.RZgate(50);

    // // Apply the X gate to the internal state
    // bead.state.Xgate();

    // // Apply the Y gate to the internal state
    // bead.state.Ygate();

    // // Apply the Z gate to the internal state/Users/bhoomikatanikonda/Desktop/RXgate
    // bead.state.Zgate();

    // // Apply the H gate to the internal state
    // bead.state.Hgate();

    // Print the updated state
    bead.state.printState();

    // Visualize the new state
    bead.setBloch_deg_smooth(bead.state.getTheta(), bead.state.getPhi(), color(255, 0, 255));
    bead.show();

    delay(5000);
}
