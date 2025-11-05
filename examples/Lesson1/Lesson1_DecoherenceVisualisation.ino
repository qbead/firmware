/// # Experiment 1 - A Visualisation of Decoherence
///
/// TODO: Some LEDs blinks white periodically - why????
/// Also need to fix speed of simulation - possible some optimisations in setBloch_deg_smooth, etc.

/// Configurable Parameters:
/// - omega: rotation per loop iteration of the centre of the ensemble

// ## Libraries
// -- expand text --
#include <Lesson1.h>


// ## Parameters and Initialisation
// Reset sequence parameter
int t;
// Number of vectors in outer array
const int N = 12; // 4n+1
// Number of vectors in inner array
const int K = 5; // 2; // 2D needs to be slower since array is bigger! Otherwise updateHorizontalAxis can't keep up
// Length of each reset sequence
const int resetTime=120;
// Length of reset delay - how long it blinks for to indicate reset to new sequence
const int resetDelay=600;
// Base rotation per loop iteration of the decohering states
double omega = 1;
// The colour-function's distance to the furthest vector away from the centre; used to determine colour of centre LED
float furthestVector = 0;
ColourTuple vectorColour;

// Initialise the vectors -- they need to be declared here, and initialised to (0,0) in setup().
BlochVector vectors[N][K], centre;
BlochVector angles, axisVertical, axisHorizontal;

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
    for (int j = 0; j < K; j++){
      vectors[i][j].setAngles(0,0);
    }
  }
  axisVertical.setAngles(90, 180);
  axisHorizontal.setAngles(90, 270);

}

// ### Loop Function
// This function is continually executed after the Setup function is complete.
//--
// Every 'reset_time' number of loop iterations, measure by the parameter t, the sequence
// restarts after a blinking delay.
void loop() {

  Serial.println(t);

  bead.clear();
  bead.readIMU(false);

  if (t<resetTime) {
    
    /*
    Standard sequence - update position of vectors and axis.
    */

    for (int i = N-1; i > -1; i--){

      float rotationVerticalAxis = omega*sin(2*M_PI*(i/float(N))); // rotationFactor about y/vertical axis
      float rotationHorizontalAxis = omega*cos(2*M_PI*(i/float(N))); // rotationFactor about x/horizontal axis

      for (int j = K-1; j > -1; j--){
        
          float rotationFactor = j / float(K-1); // ((i+1)/2) * omega * pow(-1, (i%2)-1);

          vectors[i][j].rotateAround(axisVertical, rotationFactor*rotationVerticalAxis); // calculate rotation about 1st axis
          vectors[i][j].rotateAround(axisHorizontal, rotationFactor*rotationHorizontalAxis); // rotate again about 2nd axis

          float f = colourFunction(vectors[i][j], centre);
          if (f > furthestVector) {furthestVector = f;}
        }
    }

    vectorColour = calcColours(furthestVector);

    for (int i = N-1; i > -1; i--) {
      for (int j = K-1; j > -1; j--) {
        bead.setBloch_deg_smooth(vectors[i][j], color(vectorColour.red, vectorColour.green, vectorColour.blue));
      }
    }

    furthestVector = 0;
    bead.setBloch_deg_smooth(centre, color(vectorColour.red, vectorColour.green, vectorColour.blue));

    t++;

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
      for (int j = 0; j < K; j++){
        vectors[i][j].setAngles(0,0);
      }
    }

    t=0; // resets counter

  }

  bead.show();
}