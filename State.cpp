#include "State.h"

State::State() : x(0), y(0), z(1) {
    cartesianToSpherical();
}

State::State(float x_init, float y_init, float z_init) : x(x_init), y(y_init), z(z_init) {
    cartesianToSpherical();
}

State::State(float theta_init, float phi_init) : theta(theta_init), phi(phi_init) {
    sphericalToCartesian();
}

// Private method to convert Cartesian to spherical coordinates
void State::cartesianToSpherical() {
    float l = sqrt(x * x + y * y + z * z);
    if (l == 0) l = 1; // Avoid division by zero
    theta = acos(z / l) * 180 / PI; // Convert to degrees
    phi = atan2(y, x) * 180 / PI;
    if (phi < 0) phi += 360;
}

// Private method to convert spherical to Cartesian coordinates
void State::sphericalToCartesian() {
    float r = 1; // Assume unit radius for direction
    x = r * sin(theta * PI / 180.0) * cos(phi * PI / 180.0);
    y = r * sin(theta * PI / 180.0) * sin(phi * PI / 180.0);
    z = r * cos(theta * PI / 180.0);
}

// Setters for Cartesian coordinates
void State::setX(float new_x) {
    x = new_x;
    cartesianToSpherical();
}

void State::setY(float new_y) {
    y = new_y;
    cartesianToSpherical();
}

void State::setZ(float new_z) {
    z = new_z;
    cartesianToSpherical();
}

void State::setXYZ(float new_x, float new_y, float new_z) {
    x = new_x;
    y = new_y;
    z = new_z;
    cartesianToSpherical();
}

// Setters for spherical coordinates
void State::setTheta(float new_theta) {
    theta = new_theta;
    sphericalToCartesian();
}

void State::setPhi(float new_phi) {
    phi = new_phi;
    sphericalToCartesian();
}

void State::setThetaPhi(float new_theta, float new_phi) {
    theta = new_theta;
    phi = new_phi;
    sphericalToCartesian();
}

// Getters for Cartesian coordinates
float State::getX() const {
    return x;
}

float State::getY() const {
    return y;
}

float State::getZ() const {
    return z;
}

// Getters for spherical coordinates
float State::getTheta() const {
    return theta;
}

float State::getPhi() const {
    return phi;
}

// Utility function to print state
void State::printState() {
    Serial.print("Cartesian: x = "); Serial.print(x);
    Serial.print(", y = "); Serial.print(y);
    Serial.print(", z = "); Serial.println(z);

    Serial.print("Spherical: theta = "); Serial.print(theta);
    Serial.print(", phi = "); Serial.println(phi);
}
