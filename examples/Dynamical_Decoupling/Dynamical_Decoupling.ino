// # Dynamical Decoupling
//
// This example demonstrates the dynamical decoupling of a quantum state.
// The user is presented with a target state and a current state.
// The current state is a quantum state that is subject to decoherence.
// The target state is fixed.
// The goal of the "game" is to not let the current state deviate too much from the target state.
//
// The current state experiences decoherence (i.e. it is rotated around a fixed laboratory frame).
// The method of "dynamical decoupling" is used to combat the decoherence by frequently changing the axis of decoherence.
//
// The user can tap the Qbead to reset the current state to the target state.


// First, let's include the Qbead library and set up a few useful data structures.
#include <Qbead.h>

Qbead::Qbead bead;

BlochVector current_state(90, 0);
BlochVector target_state(90, 0);

// Prepare some colors for the visualization during the game.
uint32_t purple = color(255, 0, 255);
uint32_t white = color(255, 255, 255);

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
  bead.readIMU();

  // Clear the display.
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
