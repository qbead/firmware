#include <Qbead.h>

/*
Shows the precession of a wave packet of spin vectors around the (global) vertical axis. The wave packet is represented by
three vectors: a central one (white), a front one (blue) that precesses faster than the central one, and a back one (red)
that precesses slower than the central one. As time evolves, the three vectors spread out from one another as they each
precess as different rates, representing decoherence. The axis of precession is the (global) vertical which represents an 
external magnetic field, and this axis is shown in green.

-- How to use: --
1) Begin by uploading sketch to the Qbead. After a start-up sequence the wave packet (white) and vertical axis (green) will 
appear, and then the wave packet will begin precessing. Initially the packet will only be represented by the central (white)
vector, but as time continues, the blue and red vectors will appear as they decohere further from the centre.
2) After some length of time (I've seen it denoted 'tau' in the literature) holding the Qbead still, the three vectors will
have spread out from one another. If you now *flip* the Qbead by 180 degrees along the equatorial axis, the vectors will now
be precessing *towards* each other.
3) After another period 'tau', the wave packet decoherence/'spreading out' will have been undone, and the three vectors
should overlap again representing a re-cohered state. The 180 degree flip at time tau is the dynamical decoupling procedure
(I think), resulting in the re-cohered state at time 2*tau.
4) A 'reset' sequence is automatically triggered every 10,000 loop iterations, where the green vertical vector will blink
briefly. When the blinking stops the simulation restarts with a fully-cohered wave packet.

Important: make sure you hold the Qbead very still before and after the flip, as any motion will knock the three vectors
off-axis from each other and prevent re-coherence. The flip has to be (almost) perfect and instant for the same reason.
----

Handy dynamical decoupling visualisation: figure 3 in
https://aws.amazon.com/blogs/quantum-computing/suppressing-errors-with-dynamical-decoupling-using-pulse-control-on-amazon-braket/

TODO: 
- 'Fill in' the wavepacket between the front/back vectors and the central one with a colour gradient. Note sure how.
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

// Parameters to control Qbead reset sequence
int t;
const int N = 10;
const int reset_time=15000; // length of each sequence
const int reset_delay=800; // length of reset delay - how long it blinks for to indicate reset to new sequence

// omega is the 'rotation' per loop iteration of the magnetic moment vectors
// +ve: anticlockwise, -ve: clockwise. Sign should inversely correspond to charge of particle being represented (i.e. electron --> +ve).
// Is this correct?
const float omega = -0.005; // -0.002; // -0.001;

// Initialise the spin angular momentum vectors -- need to be declared here, and initialised to (0,0) in setup().
// Three vectors to show basic decoherence: a 'central' one to indicate the centre of the wave packet, and two
// others (front and back) to show how the wave packet spreads out over time. The front one precesses quicker than
// the centre one (1.5x), and the back one precesses slower (0.5x).
// TODO: 'fill in' the areas between the central vector and the front/back ones to fully model wave packet
AngleVec vectors[N];
AngleVec angles, axis;

Qbead::Qbead bead;


void setup() {
  bead.begin();
  bead.setBrightness(25); // way too bright
  Serial.println("testing all pixels discretely");
  for (int i = 0; i < bead.pixels.numPixels(); i++) {
    bead.pixels.setPixelColor(i, color(255, 255, 255));
    Serial.println(bead.pixels.getPixelColor(i));
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
  for (int i = 0; i < N; i++){
    vectors[i].theta = 0;
    vectors[i].phi = 0;
  }
}

void loop() {
  /*
  Reads the Qbead's IMU (Inertial Motion/Movement Unit?) and updates the position of the global vertical axis (green) and 
  spin vector (white) resulting from the motion of the Qbead and the calculated spin precession, respectively. Decoherence
  is modelled by a forward (blue) and backward (red) vector that accelerate and decelerate away from the white vector,
  respectively, to represent the spreading-out of the wave packet as it decoheres.
  */

  bead.clear();
  bead.readIMU();

  if (t<reset_time) { // Standard sequence

    // Initialise global vertical axis as AngleVec (needed for precession calculations)
    axis.theta=bead.t_acc;
    axis.phi=bead.p_acc;

    // Calculate precession of spin vectors about axis

    for (int i = 0; i < N; i++){
      angles = calculate_rotation(axis, vectors[i], (1 + (2. * i / N - 1) / 10) * omega);
      vectors[i].theta = angles.theta;
      vectors[i].phi = angles.phi;
      bead.setBloch_deg_smooth(vectors[i].theta, vectors[i].phi, color(127, 127, 127));
    }
    
    // Show global vertical axis (relative to ground) in green
    bead.setBloch_deg_smooth(bead.t_acc, bead.p_acc, color(0, 255, 0));

    t++;

  } else {
    /*
    Reset sequence - indicates sim will restart by showing global vertical axis (relative to ground) blinking in green.
    */

    if (t%200 < 100) { // so that the vertical pointer blinks for duration of reset delay.
      bead.setBloch_deg_smooth(bead.t_acc, bead.p_acc, color(0, 255, 0));
    }
    
    t++;

  }

  if (t==(reset_time+reset_delay)) {
    /*
    Restart - reset all vector angle values to zero.
    */
    for (int i = 0; i < N; i++){
      vectors[i].theta = 0;
      vectors[i].phi = 0;
    }

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
