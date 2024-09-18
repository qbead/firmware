#ifndef STATE_H
#define STATE_H

#include <Arduino.h>
#include <math.h>

class State {
private:
    float x, y, z;    // Cartesian coordinates
    float theta, phi; // Spherical coordinates
    void cartesianToSpherical();
    void sphericalToCartesian();

public:
    // Constructors
    State();
    State(float x_init, float y_init, float z_init);      // Initialize with XYZ
    State(float theta_init, float phi_init);              // Initialize with theta and phi

    // Setters for Cartesian coordinates
    void setX(float new_x);
    void setY(float new_y);
    void setZ(float new_z);
    void setXYZ(float new_x, float new_y, float new_z);

    // Setters for spherical coordinates
    void setTheta(float new_theta);
    void setPhi(float new_phi);
    void setThetaPhi(float new_theta, float new_phi);

    // Getters for Cartesian coordinates
    float getX() const;
    float getY() const;
    float getZ() const;

    // Getters for spherical coordinates
    float getTheta() const;
    float getPhi() const;

    // Utility function
    void printState();
};

#endif // STATE_H
