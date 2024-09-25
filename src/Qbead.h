#ifndef QBEAD_H
#define QBEAD_H

/**
 * This header file contains the entire code for the Qbead project, including helper functions,
 * the State class, and the Qbead class, along with their method implementations.
 */

#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <LSM6DS3.h>
#include <math.h>

/*--------------------------------------------
  Helper Functions and Definitions
---------------------------------------------*/

/**
 * Combines RGB color values into a single 32-bit color value.
 *
 * @param r Red channel (0-255)
 * @param g Green channel (0-255)
 * @param b Blue channel (0-255)
 * @return 32-bit color value
 */
static uint32_t color(uint8_t r, uint8_t g, uint8_t b) {
    return ((uint32_t)r << 16) | ((uint32_t)g << 8) | b;
}

/**
 * Extracts the red channel from a 32-bit color value.
 *
 * @param rgb 32-bit color value
 * @return Red channel (0-255)
 */
static uint8_t redch(uint32_t rgb) {
    return (rgb >> 16) & 0xFF;
}

/**
 * Extracts the green channel from a 32-bit color value.
 *
 * @param rgb 32-bit color value
 * @return Green channel (0-255)
 */
static uint8_t greench(uint32_t rgb) {
    return (rgb >> 8) & 0xFF;
}

/**
 * Extracts the blue channel from a 32-bit color value.
 *
 * @param rgb 32-bit color value
 * @return Blue channel (0-255)
 */
static uint8_t bluech(uint32_t rgb) {
    return rgb & 0xFF;
}

/**
 * Generates a color based on a position around a color wheel.
 *
 * @param wheelPos Position on the color wheel (0-255)
 * @return 32-bit color value
 */
uint32_t colorWheel(uint8_t wheelPos) {
    wheelPos = 255 - wheelPos;
    if (wheelPos < 85) {
        return color(255 - wheelPos * 3, 0, wheelPos * 3);
    }
    if (wheelPos < 170) {
        wheelPos -= 85;
        return color(0, wheelPos * 3, 255 - wheelPos * 3);
    }
    wheelPos -= 170;
    return color(wheelPos * 3, 255 - wheelPos * 3, 0);
}

/**
 * Generates a color based on degrees around a color wheel.
 *
 * @param wheelPos Degrees (0-360)
 * @return 32-bit color value
 */
uint32_t colorWheel_deg(float wheelPos) {
    return colorWheel((uint8_t)(wheelPos * 255 / 360));
}

/**
 * Returns the sign of a float.
 *
 * @param x Input value
 * @return +1 if x > 0, -1 if x < 0, 0 if x == 0
 */
float sign(float x) {
    return (x > 0) - (x < 0);
}

/*--------------------------------------------
  State Class Definition and Implementation
---------------------------------------------*/

/**
 * Represents the state of the Qbead, including position in Cartesian and spherical coordinates.
 *
 * The State class handles conversions between coordinate systems and provides methods for manipulating the state, such as applying quantum gates.
 */
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

    // Quantum gates
    void Xgate();
};

/* State Class Implementation */

// Default constructor initializes to point along the z-axis
State::State() : x(0), y(0), z(1) {
    cartesianToSpherical();
}

// Initialize with Cartesian coordinates
State::State(float x_init, float y_init, float z_init) : x(x_init), y(y_init), z(z_init) {
    cartesianToSpherical();
}

// Initialize with spherical coordinates
State::State(float theta_init, float phi_init) : theta(theta_init), phi(phi_init) {
    sphericalToCartesian();
}

// Convert Cartesian to spherical coordinates
void State::cartesianToSpherical() {
    float l = sqrt(x * x + y * y + z * z);
    if (l == 0) l = 1; // Avoid division by zero
    theta = acos(z / l) * 180 / PI; // Convert to degrees
    phi = atan2(y, x) * 180 / PI;
    if (phi < 0) phi += 360;
}

// Convert spherical to Cartesian coordinates
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

// Print the current state for debugging
void State::printState() {
    Serial.print("Cartesian: x = "); Serial.print(x);
    Serial.print(", y = "); Serial.print(y);
    Serial.print(", z = "); Serial.println(z);

    Serial.print("Spherical: theta = "); Serial.print(theta);
    Serial.print(", phi = "); Serial.println(phi);
}

// Apply the X gate 
void State::Xgate() {
    // Flip theta over the x-axis
    theta = fmod(theta + 180.0, 360.0); // Ensure theta stays within 0-360 degrees
    sphericalToCartesian();
}

/*--------------------------------------------
  Qbead Class Definition and Implementation
---------------------------------------------*/

/**
 * Represents the Qbead device, handling LED control, IMU readings, and state management.
 *
 * The Qbead class integrates the State class, manages the LED visualization,
 * reads data from the IMU, and provides methods to manipulate and display the state.
 */
class Qbead {
public:
    // Constructor
    Qbead();

    // Initialization method
    void begin();

    // LED control methods
    void clear();
    void show();
    void setBrightness(uint8_t b);

    // State getters and setters
    State& getState();             // Non-const version
    const State& getState() const; // Const version
    void setState(const State& newState);

    // IMU reading
    void readIMU();

    // Visualization methods
    void setBloch_deg(float theta, float phi, uint32_t color);
    void setBloch_deg_smooth(float theta, float phi, uint32_t c);

    // LED pixel color setting
    void setLegPixelColor(int leg, int pixel, uint32_t color);

    // Public members
    Adafruit_NeoPixel pixels; 

private:
    State state;                 // State object
    LSM6DS3 imu;

    // Configuration constants
    static const uint8_t nsections = 6;
    static const uint8_t nlegs = 12;
    static const float theta_quant;
    static const float phi_quant;
    static const uint8_t ix = 0, iy = 2, iz = 1;
    static const bool sx = 0, sy = 0, sz = 1;

    // Internal variables
    float rbuffer[3];
    float rx, ry, rz;         // Raw accelerometer data
    unsigned long t_imu;      // Last IMU update time
};

/* Initialize static constants */
const float Qbead::theta_quant = 180.0 / Qbead::nsections;
const float Qbead::phi_quant = 360.0 / Qbead::nlegs;

/* Qbead Class Implementation */

// Constructor
Qbead::Qbead()
    : state(),
      imu(LSM6DS3(I2C_MODE, 0x6A)), // Default IMU address
      pixels(Qbead::nlegs * (Qbead::nsections - 1) + 2, 0, NEO_BRG + NEO_KHZ800), // Adjust LED pin and config as needed
      t_imu(0) {}

// Initialize the Qbead device
void Qbead::begin() {
    pixels.begin();
    clear();
    setBrightness(25); // Adjust brightness as needed

    Serial.println("Qbead initialized");

    // Initialize the IMU and check for errors
    uint16_t imuResult = imu.begin();
    if (imuResult != 0) {
        Serial.print("IMU error: ");
        Serial.println(imuResult);
    } else {
        Serial.println("IMU OK");
    }
}

// Clear all LEDs
void Qbead::clear() {
    pixels.clear();
}

// Show the updated LED colors
void Qbead::show() {
    pixels.show();
}

// Set the brightness of the LEDs
void Qbead::setBrightness(uint8_t b) {
    pixels.setBrightness(b);
}

// Get the current state (modifiable)
State& Qbead::getState() {
    return state;
}

// Get the current state (read-only)
const State& Qbead::getState() const {
    return state;
}

// Set the state with a new State object
void Qbead::setState(const State& newState) {
    state = newState;
}


// Read data from the IMU and update the state
void Qbead::readIMU() {
    rbuffer[0] = imu.readFloatAccelX();
    rbuffer[1] = imu.readFloatAccelY();
    rbuffer[2] = imu.readFloatAccelZ();
    rx = (1 - 2 * sx) * rbuffer[ix];
    ry = (1 - 2 * sy) * rbuffer[iy];
    rz = (1 - 2 * sz) * rbuffer[iz];

    unsigned long t_new = micros();
    float delta = t_new - t_imu;
    t_imu = t_new;
    const float T = 100000; // 100 ms
    if (delta > T) {
        delta = T;
    }
    float d = delta / T;

    float newX = d * rx + (1 - d) * state.getX();
    float newY = d * ry + (1 - d) * state.getY();
    float newZ = d * rz + (1 - d) * state.getZ();
    state.setXYZ(newX, newY, newZ);

    // Print the state for debugging
    state.printState();
}

// Visualize the state on the LED sphere
void Qbead::setBloch_deg(float theta, float phi, uint32_t color) {
    if (theta < 0 || theta > 180 || phi < 0 || phi > 360) {
        return;
    }
    float theta_section = theta / theta_quant;
    if (theta_section < 0.5) {
        setLegPixelColor(0, 0, color);
    } else if (theta_section > nsections - 0.5) {
        setLegPixelColor(0, nsections, color);
    } else {
        float phi_leg = phi / phi_quant;
        int theta_int = int(theta_section + 0.5);
        theta_int = theta_int > nsections - 1 ? nsections - 1 : theta_int;
        int phi_int = int(phi_leg + 0.5);
        phi_int = phi_int % nlegs;
        setLegPixelColor(phi_int, theta_int, color);
    }
}

// Visualize the state with smoothing
void Qbead::setBloch_deg_smooth(float theta, float phi, uint32_t c) {
    if (theta < 0 || theta > 180 || phi < 0 || phi > 360) {
        return;
    }
    float theta_section = theta / theta_quant;
    float phi_leg = phi / phi_quant;
    int theta_int = int(theta_section + 0.5);
    theta_int = theta_int > nsections - 1 ? nsections - 1 : theta_int;
    int phi_int = int(phi_leg + 0.5);
    phi_int = phi_int % nlegs;

    float p = (theta_section - theta_int);
    int theta_direction = sign(p);
    p = abs(p);
    float q = 1 - p;
    p = p * p;
    q = q * q;

    uint8_t rc = redch(c);
    uint8_t gc = greench(c);
    uint8_t bc = bluech(c);

    setLegPixelColor(phi_int, theta_int, color(q * rc, q * gc, q * bc));
    setLegPixelColor(phi_int, theta_int + theta_direction, color(p * rc, p * gc, p * bc));
}

// Set the color of a specific LED
void Qbead::setLegPixelColor(int leg, int pixel, uint32_t color) {
    leg = nlegs - leg; // Invert direction for the phi angle
    leg = leg % nlegs;
    int pixel_index;
    if (leg == 0) {
        pixel_index = pixel;
    } else if (pixel == 0) {
        pixel_index = 0;
    } else if (pixel == nsections) {
        pixel_index = 6; // Adjust if necessary
    } else {
        pixel_index = 7 + (leg - 1) * (nsections - 1) + (pixel - 1);
    }
    pixels.setPixelColor(pixel_index, color);
}

#endif // QBEAD_H
