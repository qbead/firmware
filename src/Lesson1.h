// #include <Qbead.h>
#include <Qbead.h>


Qbead::Qbead bead;

struct Gyro {
  float gx;
  float gy;
  float gz;
};

struct ColourTuple {
    float red;
    float green;
    float blue;
};

BlochVector angleRound(BlochVector u, int angleIncrement) {
  /*
  Returns the BlochVector u with its angles rounded to angleIncrements.
  */
  BlochVector v; // BlochVector to return
  float theta = angleIncrement*round(u.getTheta()/angleIncrement);
  float phi = angleIncrement*round(u.getPhi()/angleIncrement);
  v.setAngles(theta, phi);
  return v;
}

BlochVector crossProduct (BlochVector u, BlochVector v) {
  /*
  Returns u x v
  */
  BlochVector cross;
  float x = u.getY()*v.getZ() - u.getZ()*v.getY();
  float y = u.getZ()*v.getX() - u.getX()*v.getZ();
  float z = u.getX()*v.getY() - u.getY()*v.getX();
  cross.setXYZ(x, y, z);
  return cross;
}

float dotProduct (BlochVector u, BlochVector v) {
  /*
  Returns u.v
  */
  return u.getX()*v.getX() + u.getY()*v.getY() + u.getZ()*v.getZ();
}

float dotProductGyro (Gyro u, BlochVector v) {
  /*
  Returns u.v for a Gyro object u and BlochVector v (needed for horizontal axis calculations)
  */
  return u.gx*v.getX() + u.gy*v.getY() + u.gz*v.getZ();
}

float vectorLength (BlochVector u) {
  /*
  returns ||u||
  */
  return sqrt(dotProduct(u,u));
}

Gyro readGyroscope () {
  /*
  Returns the omega vector of instantaneous gyroscope rotations (measured in deg/sec) in rad/sec.
  Needs a coordinate reframing from gyroscope's XYZ to BlochVector's definitions!
  Gyro X Y Z -> BlochVector Y Z X
  */
  Gyro omega;
  omega.gy = (bead.imu.readFloatGyroX() * M_PI/180);
  omega.gz = (bead.imu.readFloatGyroY() * M_PI/180);
  omega.gx = (bead.imu.readFloatGyroZ() * M_PI/180);

  return omega;
}

Gyro calibrateGyro () {
  /*
  Calibrates the gyroscope - Qbead needs to be held still during calibration.
  */

  Serial.println("Calibrating Gyroscope: keep Qbead still!");

  Gyro bias { 0.f, 0.f, 0.f };
  delay(50);
  int numCalibrations = 200;
  for (int i = 0; i < numCalibrations; i++) {
    Gyro omega = readGyroscope();
    bias.gx += omega.gx;
    bias.gy += omega.gy;
    bias.gz += omega.gz;
    delay(5);
  }

  Serial.println("Calibration Complete! Bias:");
  Serial.print(bias.gx);
  Serial.print("\t");
  Serial.print(bias.gy);
  Serial.print("\t");
  Serial.print(bias.gz);
  Serial.println();

  bias.gx /= numCalibrations;
  bias.gy /= numCalibrations;
  bias.gz /= numCalibrations;

  return bias;
}

float colourFunction(BlochVector vector, BlochVector centre) {
  /*
  Calculates the value of the colour parameter f (0 <= f <= 2) of the decohering vector due
  to its distance from the centre vector. The further the distance, the higher the value of
  f, and thus the more red the calcColours(f) function will return. f is used to both track
  the colour of the furthest vector (to then assign that colour to all other)
  */

  float v_theta = vector.getTheta_rad();
  float v_phi = vector.getPhi_rad();

  float c_theta = centre.getTheta_rad();
  float c_phi = centre.getPhi_rad();
  
  // Great-circle angular distance between vector and the centre 
  float alpha = cos(v_theta)*cos(c_theta) + sin(v_theta)*sin(c_theta)*cos(v_phi - c_phi);

  if (alpha > 1.0) {alpha = 1.0;}
  if (alpha < -1.0) {alpha = -1.0;}

  alpha = acos(alpha); // radians

  if (abs(alpha) > (M_PI/2)) {alpha = (M_PI/2);} // cap vector to have peak red-ness at pi/2 and beyond

  float f = 4 * alpha / M_PI; // 0 < f < 2, since 0 < alpha < pi/2

  return f;
}

ColourTuple calcColours (float f) {
  /*
  The colour function of the decohering vector - calculates a tuple of (red, green, blue)
  colour values in a 0-1 range. When f = 0, colours = (0,1), giving green: when f = 1,
  colours = (1,1), giving yellow: and when f = 2, colours = (1,0), giving red. The colour
  tuple is then normalised and returned in a [0,255] range. The blue value of the tuple is
  included for completeness but kept at 0.
  */

  if (f < 0.4) {f = 0;}

  ColourTuple colours;
  colours.red = f; // 'more red' as alpha increases
  colours.green = 2-f; // 'less green' as alpha increases
  colours.blue = 0; // no blue

  // Clamp colours to be in 0-1 range
  if (colours.red > 1) {colours.red = 1;}
  if (colours.red < 0) {colours.red = 0;}
  if (colours.green > 1) {colours.green = 1;}
  if (colours.green < 0) {colours.green = 0;}

  // Bring back to 255 range
  colours.red = 255 * colours.red;
  colours.green = 255 * colours.green;

  return colours;

}

BlochVector updateHorizontalAxis (BlochVector horiz, BlochVector vert, BlochVector actualVert, Gyro bias, float angleFactor) {
  /*
  Updates the position of the Horizontal Axis based on movement of the QBead
  (defined from angles.acc/'actualVert' and gyroscope motion)
  */

  BlochVector rotationAxis = crossProduct(vert, actualVert);
  float rotationAngle = acos(dotProduct(vert,actualVert));
  if (rotationAngle > 0) {horiz.rotateAround(rotationAxis, rotationAngle);}

  Gyro omega = readGyroscope();
  omega.gx -= bias.gx;
  omega.gy -= bias.gy;
  omega.gz -= bias.gz;

  // float yawAngle = dotProductGyro(omega, actualVert) * bead.d / angleFactor; // Why divide by a factor??? No clue, just works
  // horiz.rotateAround(actualVert, -yawAngle);

  return horiz;
}