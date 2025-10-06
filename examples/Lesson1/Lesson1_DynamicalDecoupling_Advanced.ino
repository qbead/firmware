#include <Qbead.h>

/*

This is the ADVANCED version of the Dynamical Decoupling experiment (no angle rounding/increments). This means that slight
variations in the QBead's orientation WILL interrupt the DD procedure -- ensure that you keep the QBead perfectly still
while the ensemble precesses, and when you perform the procedure, the QBead flip/rotation should be as close to 180° as
possible.

TODO: 
- 'Fill in' the wavepacket between the front/back vectors and the central one with a colour gradient/cloud. Note sure how - 
  should check in with Stephan! He's worked on this.

Configurable Parameters:
- omega: rotation per loop iteration of the centre of the ensemble
- front_factor and back_factor: scaling factors of omega for the front- and back-boundaries of the ensemble packet
- reset_time and reset_delay: the length (number of loop iterations) of each sequence and the inter-sequence delay

*/

// Handy structs for using (theta, phi) and (x,y,z) vectors
struct AngleVec {
    float theta;
    float phi;
};
struct CoordVec {
    float x;
    float y;
    float z;
};

int t;                      // Reset sequence parameter
int reset_time=20000;       // Length of each reset sequence
int reset_delay=800;        // Length of reset delay - how long it blinks for to indicate reset to new sequence

float omega = -0.001;       // Base rotation (precession) per loop iteration of the ensemble states
float front_factor = 1.1;   // Scaling factor (of omega) for the front boundary of the fastest-precessing states
float back_factor = 0.9;    // Scaling factor for the back boundary of the slowest-precessing states
// For omega:
// +ve: anticlockwise, -ve: clockwise. Sign should inversely correspond to charge of particle being represented (i.e. electron --> +ve).
// Is this correct?

// Initialise the vectors -- need to be declared here, and initialised to (0,0) in setup().
AngleVec centre_vector;
AngleVec front_vector;
AngleVec back_vector;

Qbead::Qbead bead;

void setup() {
  /*
  Test LEDs and initialise the vectors.
  */
  bead.begin();
  bead.setBrightness(25); // way too bright
  Serial.println("testing all pixels discretely");
  for (int i = 0; i < bead.pixels.numPixels(); i++) {
    bead.pixels.setPixelColor(i, color(255, 255, 255));
    bead.pixels.show();
    delay(5);
  }
  Serial.println("testing smooth transition between pixels");
  for (int phi = 0; phi < 360; phi += 30) {
    for (int theta = 0; theta < 180; theta += 3) {
      bead.clear();
      bead.setBloch_deg(theta, phi, colorWheel_deg(phi));
      bead.show();
    }
  }
  Serial.println("starting inertial tracking");
  bead.clear();

  t=0;
  
  // Initialise spin vectors to zero
  centre_vector.theta=0;
  centre_vector.phi=0;
  // -----
  front_vector.theta=0;
  front_vector.phi=0;
  // -----
  back_vector.theta=0;
  back_vector.phi=0;
}

void loop() {
  /*
  Reads the Qbead's IMU and updates the position of the global vertical axis (green) and central state vector (white)
  resulting from the motion of the Qbead and the calculated spin precession, respectively. Decoherence is modelled by
  a front (blue) and back (red) vector that accelerate and decelerate away from the white vector, respectively, to
  represent the spreading-out of the ensemble's states as it decoheres. Every 'reset_time' number of loop() iterations,
  the sequence restarts after a blinking delay.
  */

  bead.clear();
  bead.readIMU();

  if (t<reset_time) {
    
    /*
    Standard sequence - update position of vectors and axis.
    */

    // Initialise global vertical axis as AngleVec (needed for precession calculations)
    AngleVec axis;
    axis.theta=bead.t_acc;
    axis.phi=bead.p_acc;

    if (axis.theta<0) {axis.theta+=180;}
    else if (axis.theta>180) {axis.theta-=180;}
    if (axis.phi<0) {axis.phi+=360;}
    else if (axis.phi>360) {axis.phi-=360;} 

    // Show global vertical axis (magnetic field) in green
    bead.setBloch_deg_smooth(axis.theta, axis.phi, color(0, 255, 0));

    // Calculate precession of spin vectors about axis
    // ----- Back vector -----
    AngleVec angles2 = calculate_rotation(axis, back_vector, back_factor*omega);
    back_vector.theta = angles2.theta;
    back_vector.phi = angles2.phi;
    bead.setBloch_deg_smooth(back_vector.theta, back_vector.phi, color(255, 0, 0)); // red
    // ----- Front vector -----
    AngleVec angles3 = calculate_rotation(axis, front_vector, front_factor*omega);
    front_vector.theta = angles3.theta;
    front_vector.phi = angles3.phi;
    bead.setBloch_deg_smooth(front_vector.theta, front_vector.phi, color(0, 0, 255)); // blue
    // ----- Centre vector -----
    AngleVec angles = calculate_rotation(axis, centre_vector, omega);
    centre_vector.theta = angles.theta;
    centre_vector.phi = angles.phi;
    bead.setBloch_deg_smooth(centre_vector.theta, centre_vector.phi, color(255, 255, 255)); // white

    t++;

  } else {
    /*
    Reset sequence - indicates sim will restart by showing global vertical axis (relative to ground) blinking in green.
    */

    if (t%200 < 100) { // so that the vertical pointer blinks every 100t for duration of reset delay.
      bead.setBloch_deg_smooth(bead.t_acc, bead.p_acc, color(0, 255, 0));
    }
    
    t++;

  }

  if (t==(reset_time+reset_delay)) {
    /*
    Restart sequence - reset all vector angle values to zero.
    */

    centre_vector.theta=0;
    centre_vector.phi=0;

    front_vector.theta=0;
    front_vector.phi=0;

    back_vector.theta=0;
    back_vector.phi=0;

    t=0; // resets counter

  }

  bead.show();
}

AngleVec coords_to_angles(CoordVec coords) {
  /*
  Calculates the angles theta and phi of a vector given its xyz coordinates, relative to the Qbead's internal axes.
  Uses Qbead.h's theta and phi functions.
  */
  AngleVec angles;

  angles.theta = theta(coords.x, coords.y, coords.z)*180/3.14159; // Factor of 180/3.14159 converts radians->degrees
  angles.phi = phi(coords.x, coords.y, coords.z)*180/3.14159;
  if (angles.phi<0) {angles.phi+=360;} // Ensures phi remains in [0,360] range

  return angles;
}

CoordVec angles_to_coords(AngleVec angles) {
  /*
  Calculates the xyz coordinates of a vector given its angles theta and phi, relative to the Qbead's internal axes.
  */
  CoordVec coords;

  // Convert degrees->radians
  angles.theta = angles.theta*3.14159/180;
  angles.phi = angles.phi*3.14159/180;

  coords.x = sin(angles.theta) * cos(angles.phi);
  coords.y = sin(angles.theta) * sin(angles.phi);
  coords.z = cos(angles.theta);

  return coords;
}

AngleVec calculate_rotation(AngleVec axis_angles, AngleVec vector_angles, float rot_angle) {
  /* Calculates the rotation of a vector v (given by vector_angles theta and phi) about an axis denoted by another vector k
  (given by axis_angles), using Rodrigues' Formula. axis_angles should be (bead.t_acc, bead.p_acc) for precession about 
  global vertical axis.
  */

  // Convert axis of rotation (k) and vector to be rotated (v) to xyz-coordinate form from angle-form.
  CoordVec k = angles_to_coords(axis_angles);
  CoordVec v = angles_to_coords(vector_angles);
  CoordVec v_rot;

  // Calculate rotation using component-wise Rodrigues' formula.
  float dot_prod = k.x*v.x + k.y*v.y + k.z*v.z;
  v_rot.x = v.x*cos(rot_angle) + (k.y*v.z - k.z*v.y)*sin(rot_angle) + k.x*dot_prod*(1-cos(rot_angle));
  v_rot.y = v.y*cos(rot_angle) + (k.z*v.x - k.x*v.z)*sin(rot_angle) + k.y*dot_prod*(1-cos(rot_angle));
  v_rot.z = v.z*cos(rot_angle) + (k.x*v.y - k.y*v.x)*sin(rot_angle) + k.z*dot_prod*(1-cos(rot_angle));

  return coords_to_angles(v_rot);
}