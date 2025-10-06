#include <Qbead.h>

/*
This experiment visualises decoherence.

TODO: 
- 'Fill in' the surface of the Qbead between the decohering vectors and the central one with a colour gradient/cloud. Note sure how - 
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

int t;                      // Reset sequence parameter
int reset_time=5000;       // Length of each reset sequence
int reset_delay=800;        // Length of reset delay - how long it blinks for to indicate reset to new sequence

float omega = -0.1;//-0.001;       // Base rotation (precession) per loop iteration of the ensemble states

// Initialise the spin angular momentum vectors -- need to be declared here, and initialised to (0,0) in setup().
// Three vectors to show basic decoherence: a 'central' one to indicate the centre of the wave packet, and two
// others (front and back) to show how the wave packet spreads out over time. The front one precesses quicker than
// the centre one (1.5x), and the back one precesses slower (0.5x).
// TODO: 'fill in' the areas between the central vector and the front/back ones to fully model wave packet
AngleVec starting_vector = {90,180};
AngleVec centre_vector;
AngleVec front_vector;
AngleVec back_vector;
AngleVec up_vector;
AngleVec down_vector;
AngleVec ne_vector;
AngleVec nw_vector;
AngleVec se_vector;
AngleVec sw_vector;
int sign_up;
int sign_down;
int sign_ne;
int sign_nw;
int sign_se;
int sign_sw;

Qbead::Qbead bead;

void setup() {
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
  
  // Initialise spin vectors to starting points
  centre_vector.theta=starting_vector.theta;
  centre_vector.phi=starting_vector.phi;
  // -----
  front_vector.theta=starting_vector.theta;
  front_vector.phi=starting_vector.phi;
  // -----
  back_vector.theta=starting_vector.theta;
  back_vector.phi=starting_vector.phi;
  // -----
  up_vector.theta=starting_vector.theta;
  up_vector.phi=starting_vector.phi;
  sign_up = 1;
  // -----
  down_vector.theta=starting_vector.theta;
  down_vector.phi=starting_vector.phi;
  sign_down = 1;
  // -----
  ne_vector.theta=starting_vector.theta;
  ne_vector.phi=starting_vector.phi;
  sign_ne = 1;
  // -----
  nw_vector.theta=starting_vector.theta;
  nw_vector.phi=starting_vector.phi;
  sign_nw = 1;
  // -----
  se_vector.theta=starting_vector.theta;
  se_vector.phi=starting_vector.phi;
  sign_se = 1;
  // -----
  sw_vector.theta=starting_vector.theta;
  sw_vector.phi=starting_vector.phi;
  sign_sw = 1;
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

    // Calculate precession of spin vectors about axis
    // ----- Back vector -----
    float back_angle = back_vector.phi + omega;
    if (back_angle<0) {back_angle+=360;}
    bead.setBloch_deg_smooth(90, back_angle, color(255, 0, 0)); // red
    back_vector.phi = back_angle;
    // ----- Front vector -----
    float front_angle = front_vector.phi - omega;
    if (front_angle>360) {front_angle-=360;}
    bead.setBloch_deg_smooth(90, front_angle, color(255, 0, 0)); // red
    front_vector.phi = front_angle;
    // ----- Down vector -----
    float down_angle = down_vector.theta + sign_down*omega;
    if (down_angle<0) {
      sign_down = -1;
      down_vector.phi = 0;
    }
    else if (down_angle>180) {
      sign_down = 1;
      down_vector.phi = starting_vector.phi;
    }
    bead.setBloch_deg_smooth(down_angle, down_vector.phi, color(255, 0, 0)); // red
    down_vector.theta = down_angle;
    // ----- Up vector -----
    float up_angle = up_vector.theta - sign_up*omega;
    if (up_angle>180) {
      sign_up = -1;
      up_vector.phi = 0;
    }
    else if (up_angle<0) {
      sign_up = 1;
      up_vector.phi = starting_vector.phi;
    }
    bead.setBloch_deg_smooth(up_angle, up_vector.phi, color(255, 0, 0)); // red
    up_vector.theta = up_angle;
    // ----- NE vector -----
    float ne_theta = ne_vector.theta + sign_ne*omega;
    float ne_phi = ne_vector.phi - omega;
    if (ne_phi>360) {ne_phi-=360;}
    if (ne_theta<0) {sign_ne = -1;}
    else if (ne_theta>180) {sign_ne = 1;}
    bead.setBloch_deg_smooth(ne_theta, ne_phi, color(255, 0, 0)); // red
    ne_vector.theta = ne_theta;
    ne_vector.phi = ne_phi;
    // ----- NW vector -----
    float nw_theta = nw_vector.theta + sign_nw*omega;
    float nw_phi = nw_vector.phi + omega;
    if (nw_phi<0) {nw_phi+=360;}
    if (nw_theta<0) {sign_nw = -1;}
    else if (nw_theta>180) {sign_nw = 1;}
    bead.setBloch_deg_smooth(nw_theta, nw_phi, color(255, 0, 0)); // red
    nw_vector.theta = nw_theta;
    nw_vector.phi = nw_phi;
    // ----- SE vector -----
    float se_theta = se_vector.theta - sign_se*omega;
    float se_phi = se_vector.phi - omega;
    if (se_phi>360) {se_phi-=360;}
    if (se_theta>180) {sign_se = -1;}
    else if (se_theta<0) {sign_se = 1;}
    bead.setBloch_deg_smooth(se_theta, se_phi, color(255, 0, 0)); // red
    se_vector.theta = se_theta;
    se_vector.phi = se_phi;
    // ----- SW vector -----
    float sw_theta = sw_vector.theta - sign_sw*omega;
    float sw_phi = sw_vector.phi + omega;
    if (sw_phi<0) {sw_phi+=360;}
    if (sw_theta>180) {sign_sw = -1;}
    else if (sw_theta<0) {sign_sw = 1;}
    bead.setBloch_deg_smooth(sw_theta, sw_phi, color(255, 0, 0)); // red
    sw_vector.theta = sw_theta;
    sw_vector.phi = sw_phi;
    // ----- Centre vector -----
    bead.setBloch_deg_smooth(centre_vector.theta, centre_vector.phi, color(255, 255, 255)); // white

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

    centre_vector.theta=starting_vector.theta;
    centre_vector.phi=starting_vector.theta;

    front_vector.theta=starting_vector.theta;
    front_vector.phi=starting_vector.theta;

    back_vector.theta=starting_vector.theta;
    back_vector.phi=starting_vector.theta;

    up_vector.theta=starting_vector.theta;
    up_vector.phi=starting_vector.theta;
    sign_up=1;

    down_vector.theta=starting_vector.theta;
    down_vector.phi=starting_vector.theta;
    sign_down=1;

    ne_vector.theta=starting_vector.theta;
    ne_vector.phi=starting_vector.theta;
    sign_ne=1;

    nw_vector.theta=starting_vector.theta;
    nw_vector.phi=starting_vector.theta;
    sign_nw=1;

    se_vector.theta=starting_vector.theta;
    se_vector.phi=starting_vector.theta;
    sign_se=1;

    sw_vector.theta=starting_vector.theta;
    sw_vector.phi=starting_vector.theta;
    sign_sw=1;

    t=0; // resets counter

  }

  bead.show();
}
