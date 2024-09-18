#ifndef QBEAD_H
#define QBEAD_H

#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <LSM6DS3.h>
#include "State.h"

// Default configurations
#define QB_LEDPIN 0
#define QB_PIXELCONFIG NEO_BRG + NEO_KHZ800
#define QB_NSECTIONS 6
#define QB_NLEGS 12
#define QB_IMU_ADDR 0x6A
#define QB_IX 0
#define QB_IY 2
#define QB_IZ 1
#define QB_SX 0
#define QB_SY 0
#define QB_SZ 1

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
    State getState() const;
    void setState(const State& newState);

    // Orientation setters and getters
    void setOrientation(float theta, float phi);
    void getOrientation(float& theta, float& phi) const;

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

    const uint8_t nsections;
    const uint8_t nlegs;
    const float theta_quant;
    const float phi_quant;
    const uint8_t ix, iy, iz;
    const bool sx, sy, sz;
    float rbuffer[3];
    float rx, ry, rz;         // Raw accelerometer data
    unsigned long t_imu;      // Last IMU update time
};

#endif // QBEAD_H
