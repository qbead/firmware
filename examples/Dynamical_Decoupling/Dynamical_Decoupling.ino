#include <Qbead.h>

Qbead::Qbead bead;

void setup() {
  bead.begin();
  bead.setBrightness(25);
  bead.testPixels();
}

BlochVector current_state(90, 0);
BlochVector target_state(90, 0);

// Prepare some colors for the visualization during the game.
uint32_t purple = color(255, 0, 255);
uint32_t white = color(255, 255, 255);

void loop() {
  bead.readIMU();
  bead.clear();

  // ### Draw the "reference" state -- the one corresponding to no decoherence.
  bead.setBloch_deg_smooth(target_state, white);

  // ### Draw the current state, as it evolves over time under the influence of decoherence.

  // To make the game more realistic (and more difficult), we will periodically
  // lower the brightness of the current state.
  // If you wish, you can set the brightness to 255 to practice.
  uint8_t brightness = parabolaWave(millis()/100);

  // Draw the state with the appropriate brightness.
  bead.setBloch_deg_smooth(current_state, scaleColor_8bit(brightness, purple));

  // Show the result.
  bead.show();

  // ### Simulate the decoherence
  //
  // The decoherence is simulated by rotating the current state around a fixed laboratory frame.
  // We happen to use the direction of gravity as reported by the IMU in this example.
  current_state.rotateAround(BlochVector(bead.x, bead.y, bead.z), 0.1);

  // ### Check for taps
  //
  // If the user taps the Qbead, we will reset the game.
  if (bead.tapped) {
    current_state = target_state;
  }
}
