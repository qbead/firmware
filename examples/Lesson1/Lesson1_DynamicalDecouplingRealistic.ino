/// # Experiment 3 - Realistic Dynamical Decoupling
///
/// TODO: Some LEDs blinks white periodically - why????
///
/// This is the REALISTIC version of the Dynamical Decoupling experiment, where the vertical magnetic field axis is NOT fixed
/// to 90-degree increments of the QBead. This means that slight variations in the QBead's orientation will not interrupt the
/// DD procedure.

/// Configurable Parameters:
/// - omega: rotation per loop iteration of the centre of the ensemble
/// - front_factor and back_factor: scaling factors of omega for the front- and back-boundaries of the ensemble packet
/// - reset_time and reset_delay: the length (number of loop iterations) of each sequence and the inter-sequence delay

// ## Libraries
// -- expand text --
#include <Lesson1.h>


// ## Parameters and Initialisation
// Reset sequence parameter
int t;
// Number of vectors in array
const int N = 31;
// Length of each reset sequence
const int resetTime = 500;//800;
// Length of reset delay - how long it blinks for to indicate reset to new sequence
const int resetDelay = 600;
// Base rotation per loop iteration of the decohering states
double omega = 0.1;
// The angular distance to the furthest vector away from the centre; used to determine colour of centre LED
float furthestVector = 0;
float f = 0;
ColourTuple vectorColour;

// Initialise the vectors -- they need to be declared here, and initialised to (0,0) in setup().
BlochVector vectors[N];
BlochVector angles, axisVertical;

// ### Setup Function
// This function tests the LEDs of the QBead and initialises the vectors
// used for the decoherent states.
void setup() {
  /*
  Test LEDs and initialise the vectors.
  */

  bead.begin();
  bead.setBrightness(25);
  Serial.println("testing all pixels discretely");
  for (int i = 0; i < bead.pixels.numPixels(); i++) {
    bead.pixels.setPixelColor(i, color(255, 255, 255));
    bead.pixels.show();
    delay(5);
  }
  Serial.println("testing smooth transition between pixels");
  for (int phi = 0; phi < 360; phi += 30) {
    for (int theta = 0; theta < 180; theta += 3) {
      BlochVector test;
      test.setAngles(theta, phi);
      bead.clear();
      bead.setBloch_deg_smooth(test, colorWheel_deg(phi));
      bead.show();
    }
  }
  Serial.println("starting inertial tracking");
  bead.clear();

  t=0;

  // Initialise vectors to zero
  for (int i = 0; i < N; i++){
    vectors[i].setAngles(0,0);
  }
  axisVertical.setAngles(0, 0);

}

// ### Loop Function
// This function is continually executed after the Setup function is complete.
// --
// It reads the Qbead's IMU and updates the position of the global vertical axis (blue)
// resulting from the motion of the Qbead.
//--
// Decoherence is modelled by vectors that move ('decohere') away from a central,
// unmoving vector. These vectors represent a quantum ensemble, and their 'spreading-out'
// represents the progressive decoherence of the ensemble. The colour of the three vectors
// is given on a green->red scale, and is calculated by the relative spread of the vectors.
// Green = coherent, red = decoherent.
//--
// Every 'reset_time' number of loop iterations, measure by the parameter t, the sequence
// restarts after a blinking delay.
void loop() {

  bead.clear();
  bead.readIMU(false);

  Serial.println(t);

  axisVertical = bead.angle_acc;

  if (t<resetTime) {
    
    /*
    Standard sequence - update position of vectors and axis.
    */

    // Show global vertical axis (relative to ground) in blue
    bead.setBloch_deg_smooth(axisVertical, color(0, 0, 255));

    // To colour the vectors based on furthestVector in previous loop() iteration
    vectorColour = calcColours(furthestVector);
    f = 0;

    for (int i = N-1; i > -1; i--){
      // Calculate how much to rotate each vector[i] by
      float rotationFactor = ((i+1)/2) * omega * pow(-1, (i%2)-1);
      // Rotate vectors and show them
      vectors[i].rotateAround(axisVertical, rotationFactor);
      bead.setBloch_deg_smooth(vectors[i], color(vectorColour.red, vectorColour.green, vectorColour.blue));
      // Update f to keep track of new furthestVector
      if (colourFunction(vectors[i], vectors[0]) > f) {f = colourFunction(vectors[i], vectors[0]);}
    }

    t++;
    furthestVector = f;

  } else {
    /*
    Reset sequence - indicates sim will restart by showing global vertical axis (relative to ground) blinking in green.
    */

    if (t%150 < 75) { // so that the vertical pointer blinks every 50t for duration of reset delay.
      bead.setBloch_deg_smooth(axisVertical, color(0, 0, 255));
    }
    
    t++;

  }

  if (t==(resetTime+resetDelay)) {
    /*
    Restart sequence - reset all vector angle values to zero.
    */

    for (int i = 0; i < N; i++){
      vectors[i].setAngles(0,0);
    }

    t=0; // resets counter

  }

  bead.show();
}