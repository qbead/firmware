// # Tap to Measure
//

// First, let's include the Qbead library and set up a few useful data structures.
#include <Qbead.h>

Qbead::Qbead bead;

BlochVector current_state(90, 0);

// Prepare some colors for the visualization during the game.
uint32_t white = color(255, 255, 255);
uint32_t red = color(255, 0, 0);
uint32_t blue = color(0, 0, 255);
bool display_ready = false;
// ## Setup
//
// The setup function is called once when the Qbead is powered on and it is used to initialize the Qbead and set up the game.
void setup() {
  bead.begin();
  bead.setBrightness(25);
  // Test the pixels by flashing a colorful pattern to make sure they are working.
  bead.testPixels();
}

// ## Event loop
//
// The loop function is called repeatedly until the Qbead is powered off.
// It is used to read the IMU and update the current state of the game.
void loop() {
  // Read the IMU and compute the direction of gravity and whether a tap has been detected.
  // bead.readIMU(true);

  // Clear the display.
    bead.clear();

    bead.setBloch_deg_smooth(current_state, white);

    // Show the result.
    bead.show();

  if (bead.wasTapped()){
    bead.clear();
    BlochVector acc_vector(bead.xWhenTapped, bead.yWhenTapped, bead.zWhenTapped);
    
    float probability = pow(innerProductAbs(current_state, acc_vector),2);
    float threshold = random(0, 100)/100.0f;
    float identity_threshold = 1;
    // Serial.print("probability: ");
    // Serial.println(probability);
    // Serial.print("threshold: ");
    // Serial.println(threshold);
    if (probability > identity_threshold) {
      bead.setBloch_deg_smooth(current_state, red);
      // Serial.println("red identity");
    } else if (probability < 1 - identity_threshold) {
      bead.setBloch_deg_smooth(current_state, blue);
      // Serial.println("blue identity");
    } else if (probability > threshold) {
      current_state = acc_vector;
      bead.setBloch_deg_smooth(current_state, red);
      // Serial.println("red random");
    } else {
      current_state = -acc_vector;
      bead.setBloch_deg_smooth(current_state, blue);
      // Serial.println("blue random");
    }
    bead.show();
    delay(3000);
    bead.tapped = false;
    display_ready = false;
  }
}

// ToDo: clip to nearest pixel on qbead

