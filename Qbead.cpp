#include "Qbead.h"
#include "Helpers.h"

Qbead::Qbead()
    : state(),
      imu(LSM6DS3(I2C_MODE, QB_IMU_ADDR)),
      pixels(QB_NLEGS * (QB_NSECTIONS - 1) + 2, QB_LEDPIN, QB_PIXELCONFIG),
      nsections(QB_NSECTIONS),
      nlegs(QB_NLEGS),
      theta_quant(180.0 / QB_NSECTIONS),
      phi_quant(360.0 / QB_NLEGS),
      ix(QB_IX), iy(QB_IY), iz(QB_IZ),
      sx(QB_SX), sy(QB_SY), sz(QB_SZ),
      t_imu(0) {}

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

void Qbead::clear() {
    pixels.clear();
}

void Qbead::show() {
    pixels.show();
}

void Qbead::setBrightness(uint8_t b) {
    pixels.setBrightness(b);
}

// Get the current state
State Qbead::getState() const {
    return state;
}

// Set the state with a new State object
void Qbead::setState(const State& newState) {
    state = newState;
}

// Set the orientation using theta and phi
void Qbead::setOrientation(float theta, float phi) {
    state.setThetaPhi(theta, phi);
}

// Get the orientation
void Qbead::getOrientation(float& theta, float& phi) const {
    theta = state.getTheta();
    phi = state.getPhi();
}

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
