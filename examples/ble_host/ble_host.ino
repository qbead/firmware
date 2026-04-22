// B van Ommen 22042026
// example program for a host qbead
// the host qbead will connect to a client qbead running
// tap_to_measure.io, and will light an LED in the orientation of the client qbead
#include <hostQbead.h>

HostQbead::HostQbead hostBead;
BlochVector current_state(90, 0);

// Prepare some colors for the visualization during the game.
uint32_t white = color(255, 255, 255);
uint32_t red = color(255, 0, 0);
uint32_t blue = color(0, 0, 255);

void setup(){
 hostBead.begin();
  hostBead.setBrightness(25);
  // Test the pixels by flashing a colorful pattern to make sure they are working.
  hostBead.testPixels();
}

void loop() {
  static long last_tap = 0;
  
  static uint32_t tap_color = white;
  // Clear the display.
  hostBead.clear();
  if (hostBead.wasTapped()){
    Serial.print("They were tapped!");
      last_tap = millis();

    BlochVector acc_vector(hostBead.x_whentapped, hostBead.y_whentapped, hostBead.z_whentapped);
    current_state = acc_vector;
  }
  float delta = max(min(millis() - last_tap, 10000), 0) / 10000.0;
  uint32_t color = addColor(scaleColor(1-delta, tap_color), scaleColor(delta, white));
  hostBead.setBloch_deg(current_state, color);
  // Show the result.
  hostBead.show();
  }

