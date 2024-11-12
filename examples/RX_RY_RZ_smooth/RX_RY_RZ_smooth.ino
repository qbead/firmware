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

void RXgateAni(float angle, int steps, float time_in_ms) {
      for (int i=1; i<=steps; i++){
          bead.state.RXgate(angle/steps);
          bead.clear();
          //bead.state.printState(); //for debugging
          bead.setBloch_deg_smooth(bead.state.getTheta(), bead.state.getPhi(), color(255, 0, 255));
          bead.show();
          delay(time_in_ms);
      }
  }
  void RYgateAni(float angle, int steps, float time_in_ms) {
      for (int i=1; i<=steps; i++){
          bead.state.RYgate(angle/steps);
          bead.clear();
          //bead.state.printState(); //for debugging
          bead.setBloch_deg_smooth(bead.state.getTheta(), bead.state.getPhi(), color(255, 0, 255));
          bead.show();
          delay(time_in_ms);
      }
  }
  void RZgateAni(float angle, int steps, float time_in_ms) {
      for (int i=1; i<=steps; i++){
          bead.state.RZgate(angle/steps);
          bead.clear();
          //bead.state.printState(); //for debugging
          bead.setBloch_deg_smooth(bead.state.getTheta(), bead.state.getPhi(), color(255, 0, 255));
          bead.show();
          delay(time_in_ms);
      }
  }

void loop() { 
  //RXgateAni (180, 6, 1000);
  //RYgateAni (180, 6, 1000);
  RXgateAni (180, 6, 1000);
  delay(5000);
 }