#ifndef QBEAD_H
#define QBEAD_H


#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <LSM6DS3.h>
#include <math.h>
#include <ArduinoEigen.h>

#include <bluefruit.h>

using namespace Eigen;

// default configs
#define QB_LEDPIN 10
#define QB_PIXELCONFIG NEO_BRG + NEO_KHZ800
#define QB_IMU_ADDR 0x6A
#define QB_IX 0
#define QB_IY 2
#define QB_IZ 1
#define QB_SX 0
#define QB_SY 0
#define QB_SZ 1
#define GYRO_MEASUREMENT_THRESHOLD 0.0001
#define GYRO_GATE_THRESHOLD 500
#define SHAKING_THRESHOLD 0.09
#define QB_PIXEL_COUNT 62

#define QB_MAX_PRPH_CONNECTION 2

const uint8_t QB_UUID_SERVICE[] =
{0x45,0x8d,0x08,0xaa,0xd6,0x63,0x44,0x25,0xbe,0x12,0x9c,0x35,0xc6,0x1f,0x0c,0xe3};
const uint8_t QB_UUID_COL_CHAR[] =
{0x45,0x8d,0x08,0xaa,0xd6,0x63,0x44,0x25,0xbe,0x12,0x9c,0x35,0xc6+1,0x1f,0x0c,0xe3};
const uint8_t QB_UUID_SPH_CHAR[] =
{0x45,0x8d,0x08,0xaa,0xd6,0x63,0x44,0x25,0xbe,0x12,0x9c,0x35,0xc6+2,0x1f,0x0c,0xe3};
const uint8_t QB_UUID_ACC_CHAR[] =
{0x45,0x8d,0x08,0xaa,0xd6,0x63,0x44,0x25,0xbe,0x12,0x9c,0x35,0xc6+3,0x1f,0x0c,0xe3};
const uint8_t QB_UUID_GYR_CHAR[] =
{0x45,0x8d,0x08,0xaa,0xd6,0x63,0x44,0x25,0xbe,0x12,0x9c,0x35,0xc6+4,0x1f,0x0c,0xe3};

const uint8_t zerobuffer20[] = {0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0};

// TODO manage namespaces better
static uint32_t color(uint8_t r, uint8_t g, uint8_t b) {
  return ((uint32_t)r << 16) | ((uint32_t)g << 8) | b;
}

static uint8_t redch(uint32_t rgb) {
  return rgb >> 16;
}

static uint8_t greench(uint32_t rgb) {
  return (0x00ff00 & rgb) >> 8;
}

static uint8_t bluech(uint32_t rgb) {
  return 0x0000ff & rgb;
}

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

uint32_t colorWheel_deg(float wheelPos) {
  return colorWheel(wheelPos * 255 / 360);
}

float sign(float x) {
  if (x > 0) return +1;
  else return -1;
}

// z = cos(t)
// x = cos(p)sin(t)
// y = sin(p)sin(t)
// Return the angle in radians between the x-axis and the line to the point (x, y)
float phi(float x, float y) {
  return atan2(y, x);
}

float phi(float x, float y, float z) {
  return phi(x, y);
}

float theta(float x, float y, float z) {
  float ll = x * x + y * y + z * z;
  float l = sqrt(ll);
  float theta = acos(z / l);
  return theta;
}

bool checkThetaAndPhi(float theta, float phi) {
  return theta >= 0 && theta <= 180 && phi >= 0 && phi <= 360;
}

void connect_callback(uint16_t conn_handle)
{
  // Get the reference to current connection
  BLEConnection* connection = Bluefruit.Connection(conn_handle);

  char central_name[32] = { 0 };
  connection->getPeerName(central_name, sizeof(central_name));

  Serial.print("[INFO]{BLE} Connected to "); // TODO take care of cases where Serial is not available
  Serial.println(central_name);
}

// In rads
void sphericalToCartesian(float theta, float phi, float& x, float& y, float& z)
{
  // Normalize yaw to be between 0 and 2*PI
  phi = fmod(phi, 2 * PI);
  if (phi < 0)
  {
    phi += 2 * PI;
  }
  if (!checkThetaAndPhi(theta * 180 / PI, phi * 180 / PI))
  {
    Serial.print("Theta or Phi out of range when creating coordinates class, initializing as 1");
    Serial.print("Theta: ");
    Serial.print(theta);
    Serial.print("Phi: ");
    Serial.println(phi);
    x = 0;
    y = 0;
    z = 1;
    return;
  }

  x = sin(theta) * cos(phi);
  y = sin(theta) * sin(phi);
  z = cos(theta);
}

namespace Qbead {

class Coordinates
{
public:
  Vector3d v;

  Coordinates(float argx, float argy, float argz)
  {
    v = Vector3d(argx, argy, argz);
    v.normalize();
  }

  // In rads
  Coordinates(float theta, float phi)
  {
    float x, y, z = 0;
    sphericalToCartesian(theta, phi, x, y, z);
    v = Vector3d(x, y, z);
  }

  Coordinates(Vector3d vector)
  {
    v = vector;
    v.normalize();
  }

  float x()
  {
    return v(0);
  }

  float y()
  {
    return v(1);
  }

  float z()
  {
    return v(2);
  }

  // In rads
  float theta()
  {
    return acos(z());
  }

  // In rads
  float phi()
  {
    return atan2(y(), x());
  }

  float dist(Vector3d other) const
  {
    Vector3d diff = v - other;
    return diff.norm();
  }

  void set(float argx, float argy, float argz)
  {
    v = Vector3d(argx, argy, argz);
    v.normalize();
  }

  // in rads
  void set(float theta, float phi)
  {
    float x, y, z = 0;
    sphericalToCartesian(theta, phi, x, y, z);
    v = Vector3d(x, y, z);
  }

  void set(Vector3d vector) {
    v = vector;
    v.normalize();
  }

  // in rads
  void setTheta(float theta)
  {
    set(theta, phi());
  }

  // in rads
  void setPhi(float phi)
  {
    set(theta(), phi);
  }
};

class QuantumState
{
private:
  Coordinates stateCoordinates;

public:
  QuantumState(Coordinates argStateCoordinates) : stateCoordinates(argStateCoordinates) {}
  QuantumState() : stateCoordinates(0, 0, 1) {}

  void setCoordinates(Coordinates argStateCoordinates)
  {
    stateCoordinates.set(argStateCoordinates.v);
  }

  Coordinates getCoordinates()
  {
    return stateCoordinates;
  }

  void collapse()
  {
    const float theta = stateCoordinates.theta();
    const float a = cos(theta / 2);
    const bool is1 = random(0, 100) < a * a * 100;
    this->stateCoordinates.set(0, 0, is1 ? 1 : -1);
  }

  // Rotate PI around the x axis
  void gateX()
  {
    stateCoordinates.set(stateCoordinates.x(), -stateCoordinates.y(), -stateCoordinates.z());
  }

  // Rotate PI around the y axis
  void gateZ()
  {
    stateCoordinates.set(-stateCoordinates.x(), -stateCoordinates.y(), stateCoordinates.z());
  }

  // Rotate PI around the z axis
  void gateY()
  {
    stateCoordinates.set(-stateCoordinates.x(), stateCoordinates.y(), -stateCoordinates.z());
  }

  // Rotate PI around the xz axis
  void gateH()
  {
    stateCoordinates.set(stateCoordinates.z(), stateCoordinates.y(), stateCoordinates.x()); //flip x and z axis
  }
};

class Qbead {
public:
  Qbead(const uint16_t pin00 = QB_LEDPIN,
        const uint16_t pixelconfig = QB_PIXELCONFIG,
        const uint8_t imu_addr = QB_IMU_ADDR,
        const uint8_t ix = QB_IX,
        const uint8_t iy = QB_IY,
        const uint8_t iz = QB_IZ,
        const bool sx = QB_SX,
        const bool sy = QB_SY,
        const bool sz = QB_SZ)
      : imu(LSM6DS3(I2C_MODE, imu_addr)),
        pixels(Adafruit_NeoPixel(QB_PIXEL_COUNT, pin00, pixelconfig)),
        ix(ix), iy(iy), iz(iz),
        sx(sx), sy(sy), sz(sz),
        bleservice(QB_UUID_SERVICE),
        blecharcol(QB_UUID_COL_CHAR),
        blecharsph(QB_UUID_SPH_CHAR),
        blecharacc(QB_UUID_ACC_CHAR),
        blechargyr(QB_UUID_GYR_CHAR)
        {}

  static Qbead *singletoninstance; // we need a global singleton static instance because bluefruit callbacks do not support context variables -- thankfully this is fine because there is indeed only one Qbead in existence at any time

  LSM6DS3 imu;
  Adafruit_NeoPixel pixels;

  BLEService bleservice;
  BLECharacteristic blecharcol;
  BLECharacteristic blecharsph;
  BLECharacteristic blecharacc;
  BLECharacteristic blechargyr;
  uint8_t connection_count = 0;

  const uint8_t ix, iy, iz;
  const bool sx, sy, sz;
  float rbuffer[3], rgyrobuffer[3];
  float x, y, z, rx, ry, rz; // filtered and raw acc, in units of g
  float xGyro, yGyro, zGyro, rxGyro, ryGyro, rzGyro; // filtered and raw gyro measurements, in units of deg/s
  float T_imu;             // last update from the IMU
  Vector3d gravity = Vector3d(0, 0, 1); // gravity vector
  Vector3d gyroVector = Vector3d(0, 0, 1); // gyro vector
  float yaw;

  float t_ble, p_ble; // theta and phi as sent over BLE connection
  uint32_t c_ble = 0xffffff; // color as sent over BLE connection

  // led map index to Coordinates
  // This map is for the first version of the pcb
  Coordinates led_map_v1[62] = {
    Coordinates(PI, 0),
    Coordinates(5 * PI / 6, 9 * PI / 6),
    Coordinates(4 * PI / 6, 9 * PI / 6),
    Coordinates(3 * PI / 6, 9 * PI / 6),
    Coordinates(2 * PI / 6, 9 * PI / 6),
    Coordinates(PI / 6, 9 * PI / 6),
    Coordinates(0, 0),
    Coordinates(5 * PI / 6, 10 * PI / 6),
    Coordinates(4 * PI / 6, 10 * PI / 6),
    Coordinates(3 * PI / 6, 10 * PI / 6),
    Coordinates(2 * PI / 6, 10 * PI / 6),
    Coordinates(PI / 6, 10 * PI / 6),
    Coordinates(5 * PI / 6, 11 * PI / 6),
    Coordinates(4 * PI / 6, 11 * PI / 6),
    Coordinates(3 * PI / 6, 11 * PI / 6),
    Coordinates(2 * PI / 6, 11 * PI / 6),
    Coordinates(PI / 6, 11 * PI / 6),
    Coordinates(5 * PI / 6, 0),
    Coordinates(4 * PI / 6, 0),
    Coordinates(3 * PI / 6, 0),
    Coordinates(2 * PI / 6, 0),
    Coordinates(PI / 6, 0),
    Coordinates(5 * PI / 6, PI / 6),
    Coordinates(4 * PI / 6, PI / 6),
    Coordinates(3 * PI / 6, PI / 6),
    Coordinates(2 * PI / 6, PI / 6),
    Coordinates(PI / 6, PI / 6),
    Coordinates(5 * PI / 6, 2 * PI / 6),
    Coordinates(4 * PI / 6, 2 * PI / 6),
    Coordinates(3 * PI / 6, 2 * PI / 6),
    Coordinates(2 * PI / 6, 2 * PI / 6),
    Coordinates(PI / 6, 2 * PI / 6),
    Coordinates(5 * PI / 6, 3 * PI / 6),
    Coordinates(4 * PI / 6, 3 * PI / 6),
    Coordinates(3 * PI / 6, 3 * PI / 6),
    Coordinates(2 * PI / 6, 3 * PI / 6),
    Coordinates(PI / 6, 3 * PI / 6),
    Coordinates(5 * PI / 6, 4 * PI / 6),
    Coordinates(4 * PI / 6, 4 * PI / 6),
    Coordinates(3 * PI / 6, 4 * PI / 6),
    Coordinates(2 * PI / 6, 4 * PI / 6),
    Coordinates(PI / 6, 4 * PI / 6),
    Coordinates(5 * PI / 6, 5 * PI / 6),
    Coordinates(4 * PI / 6, 5 * PI / 6),
    Coordinates(3 * PI / 6, 5 * PI / 6),
    Coordinates(2 * PI / 6, 5 * PI / 6),
    Coordinates(PI / 6, 5 * PI / 6),
    Coordinates(5 * PI / 6, 6 * PI / 6),
    Coordinates(4 * PI / 6, 6 * PI / 6),
    Coordinates(3 * PI / 6, 6 * PI / 6),
    Coordinates(2 * PI / 6, 6 * PI / 6),
    Coordinates(PI / 6, 6 * PI / 6),
    Coordinates(5 * PI / 6, 7 * PI / 6),
    Coordinates(4 * PI / 6, 7 * PI / 6),
    Coordinates(3 * PI / 6, 7 * PI / 6),
    Coordinates(2 * PI / 6, 7 * PI / 6),
    Coordinates(PI / 6, 7 * PI / 6),
    Coordinates(5 * PI / 6, 8 * PI / 6),
    Coordinates(4 * PI / 6, 8 * PI / 6),
    Coordinates(3 * PI / 6, 8 * PI / 6),
    Coordinates(2 * PI / 6, 8 * PI / 6),
    Coordinates(PI / 6, 8 * PI / 6),
  };

  static void ble_callback_color(uint16_t conn_hdl, BLECharacteristic* chr, uint8_t* data, uint16_t len) {
    Serial.println("[INFO]{BLE} Received a write on the color characteristic");
    singletoninstance->c_ble =  (data[2] << 16) | (data[1] << 8) | data[0];
    Serial.print("[DEBUG]{BLE} Received");
    Serial.println(singletoninstance->c_ble, HEX);
  }

  static void ble_callback_theta_phi(uint16_t conn_hdl, BLECharacteristic* chr, uint8_t* data, uint16_t len){
    Serial.println("[INFO]{BLE} Received a write on the spherical coordinates characteristic");
    singletoninstance->t_ble = ((uint32_t)data[0])*180/255;
    singletoninstance->p_ble = ((uint32_t)data[1])*360/255;
    Serial.print("[DEBUG]{BLE} Received t=");
    Serial.print(singletoninstance->t_ble);
    Serial.print(" p=");
    Serial.println(singletoninstance->p_ble);
  }

  void begin() {
    singletoninstance = this;
    Serial.begin(9600);
    while (!Serial); // TODO some form of warning or a way to give up if Serial never becomes available

    pixels.begin();
    clear();
    setBrightness(10);

    Serial.println("[INFO] Booting... Qbead on XIAO BLE Sense + LSM6DS3 compiled on " __DATE__ " at " __TIME__);
    if (!imu.begin()) {
      Serial.println("[DEBUG]{IMU} IMU initialized correctly");
    } else {
      Serial.println("[ERROR]{IMU} IMU failed to initialize");
    }

    // BLE Peripheral service setup
    Bluefruit.begin(QB_MAX_PRPH_CONNECTION, 0);
    Bluefruit.setName("qbead | " __DATE__ " " __TIME__);
    Bluefruit.Periph.setConnectCallback(connect_callback);
    bleservice.begin();
    // BLE Characteristic Bloch Sphere Visualizer color setup
    blecharcol.setProperties(CHR_PROPS_READ | CHR_PROPS_WRITE);
    blecharcol.setPermission(SECMODE_OPEN, SECMODE_OPEN);
    blecharcol.setUserDescriptor("BSV rgb color");
    blecharcol.setFixedLen(3);
    blecharcol.setWriteCallback(ble_callback_color);
    blecharcol.begin();
    blecharcol.write(zerobuffer20, 3);
    // BLE Characteristic Bloch Sphere Visualizer spherical coordinate setup
    blecharsph.setProperties(CHR_PROPS_READ | CHR_PROPS_WRITE);
    blecharsph.setPermission(SECMODE_OPEN, SECMODE_OPEN);
    blecharsph.setUserDescriptor("BSV spherical coordinates");
    blecharsph.setFixedLen(2);
    blecharsph.setWriteCallback(ble_callback_theta_phi);
    blecharsph.begin();
    blecharsph.write(zerobuffer20, 2);
    // BLE Characteristic IMU xyz accelerometer readout
    blecharacc.setProperties(CHR_PROPS_READ | CHR_PROPS_NOTIFY);
    blecharacc.setPermission(SECMODE_OPEN, SECMODE_OPEN);
    blecharacc.setUserDescriptor("xyz acceleration");
    blecharacc.setFixedLen(3*sizeof(float));
    blecharacc.begin();
    blecharacc.write(zerobuffer20, 3*sizeof(float));    
    // BLE Characteristic IMU xyz gyroscope readout
    blechargyr.setProperties(CHR_PROPS_READ | CHR_PROPS_NOTIFY);
    blechargyr.setPermission(SECMODE_OPEN, SECMODE_OPEN);
    blechargyr.setUserDescriptor("xyz gyroscope");
    blechargyr.setFixedLen(3*sizeof(float));
    blechargyr.begin();
    blechargyr.write(zerobuffer20, 3*sizeof(float));
    startBLEadv();
  }

  void clear() {
    pixels.clear();
  }

  void show() {
    pixels.show();
  }

  void setBrightness(uint8_t b) {
    pixels.setBrightness(b);
  }

  void setLed(Coordinates coordinates, uint32_t color, bool smooth = false) {
    float theta = coordinates.theta() * 180 / PI;
    float phi = coordinates.phi() * 180 / PI;
    if (phi < 0) {
      phi += 360;
    }
    setBloch_deg(theta, phi, color, smooth);
  }

  void showAxis() {
    setLed(Coordinates(0, 0, 1), color(255, 0, 0));
    setLed(Coordinates(0, 0, -1), color(255, 0, 0));
    setLed(Coordinates(0, 1, 0), color(0, 255, 0));
    setLed(Coordinates(0, -1, 0), color(0, 255, 0));
    setLed(Coordinates(1, 0, 0), color(0, 0, 255));
    setLed(Coordinates(-1, 0, 0), color(0, 0, 255));
  }

  // in rads
  float getDistToLed(float theta, float phi, int index) {
    const Coordinates led = led_map_v1[index];
    const Coordinates reference(theta, phi);
    return led.dist(reference.v);
  }

  // Single bit is lit up on the Bloch sphere  
  void setBloch_deg(float theta, float phi, uint32_t c, bool smooth = false) {
    int closest_index = -1;
    float closest_dist = 1000;
    int second_closest_index = -1;
    float second_closest_dist = 1000;
    for (int i = 0; i < 62; i++) {
      float dist = getDistToLed(theta * PI / 180, phi * PI / 180, i);
      if (dist < closest_dist) {
        second_closest_index = closest_index;
        second_closest_dist = closest_dist;
        closest_index = i;
        closest_dist = dist;
      } else if (dist < second_closest_dist) {
        second_closest_index = i;
        second_closest_dist = dist;
      }
    }
    if (smooth) {
      float dist1 = getDistToLed(theta * PI / 180, phi * PI / 180, closest_index);
      float dist2 = getDistToLed(theta * PI / 180, phi * PI / 180, second_closest_index);
      float ratio1 = dist1 / (dist1 + dist2);
      float ratio2 = dist2 / (dist1 + dist2);
      uint8_t r = redch(c);
      uint8_t g = greench(c);
      uint8_t b = bluech(c);
      float p1 = ratio1 * ratio1;
      float p2 = ratio2 * ratio2;
      pixels.setPixelColor(closest_index, color(p2 * r, p2 * g, p2 * b));
      pixels.setPixelColor(second_closest_index, color(p1 * r, p1 * g, p1 * b));
    } else {
      pixels.setPixelColor(closest_index, c);
    }
  }

  void setBloch_deg_smooth(float theta, float phi, uint32_t c) {
    setBloch_deg(theta, phi, c, true);
  }

  Vector3d getGravity()
  {
    // substract gravity from the gravity vector
    Vector3d g = gravity;
    g.normalize();
    return gravity - g;
  }

  bool checkMotion(QuantumState &toBeRotated)
  {
    Vector3d gravity = getGravity();
    u_int8_t n = 0;
    // store the function in a variable (default is do nothing)
    void (QuantumState::*func)();
    if (abs(gyroVector[0]) > GYRO_GATE_THRESHOLD)
    {
      Serial.println("Executing X gate");
      func = &QuantumState::gateX;
      n++;
    }
    if (abs(gyroVector[1]) > GYRO_GATE_THRESHOLD)
    {
      Serial.println("Executing Y gate");
      func = &QuantumState::gateY;
      n++;
    }
    if (abs(gyroVector[2]) > GYRO_GATE_THRESHOLD)
    {
      Serial.println("Executing Z gate");
      func = &QuantumState::gateZ;
      n++;
    }
    if (gravity(2) * gravity(2) > SHAKING_THRESHOLD)
    {
      Serial.println("Collapsing");
      toBeRotated.collapse();
      func = &QuantumState::collapse;
      n++;
    }
    if (gravity(0) * gravity(0) + gravity(1) * gravity(1) > SHAKING_THRESHOLD)
    {
      Serial.println("Hadamard gate");
      func = &QuantumState::gateH;
      n++;
    }
    if (n == 1)
    {
      // execute the function
      (toBeRotated.*func)();
      return true;
    }
    else if (n > 1)
    {
      Serial.println("Multiple gates detected, ignoring");
    }
    return false;
  }

  void readIMU(bool print=true) {
    rbuffer[0] = imu.readFloatAccelX();
    rbuffer[1] = imu.readFloatAccelY();
    rbuffer[2] = imu.readFloatAccelZ();    
    rgyrobuffer[0] = imu.readFloatGyroX();
    rgyrobuffer[1] = imu.readFloatGyroY();
    rgyrobuffer[2] = imu.readFloatGyroZ();

    // calibration of imu because imu is not aligned with bloch sphere
    rx = (1-2*sx)*rbuffer[ix];
    ry = (1-2*sy)*rbuffer[iy];
    rz = (1-2*sz)*rbuffer[iz];

    // calibration of imu because imu is not aligned with bloch sphere
    rxGyro = (1-2*sx)*rgyrobuffer[ix];
    ryGyro = (1-2*sy)*rgyrobuffer[iy];
    rzGyro = (1-2*sz)*rgyrobuffer[iz];
    
    float T_new = micros();
    float delta = T_new - T_imu;
    T_imu = T_new;
    // timefilter filter
    const float T_acc = 100000; // 100 ms // TODO make the filter timeconstant configurable
    const float T_gyr = 10000; // 10 ms
    if (delta > 10000)
    {
      gyroVector = Vector3d(rxGyro, ryGyro, rzGyro);
    }
    else
    {
      float d = delta / T_gyr;
      gyroVector = d * Vector3d(rxGyro, ryGyro, rzGyro) + (1 - d) * gyroVector;
    }
    if (delta > 100000) {
      x = rx;
      y = ry;
      z = rz;
    } else {
      float d = delta/T_acc;
      x = d*rx+(1-d)*x;
      y = d*ry+(1-d)*y;
      z = d*rz+(1-d)*z;
    }

    // Gyroscope
    xGyro = rxGyro * delta * PI / (1000000 * 180);
    yGyro = ryGyro * delta * PI / (1000000 * 180);
    // The zGyro has an offset of 0.0006 rad/s
    zGyro = rzGyro * delta * PI / (1000000 * 180) - 0.0006;
    if (xGyro < GYRO_MEASUREMENT_THRESHOLD && xGyro > -GYRO_MEASUREMENT_THRESHOLD) xGyro = 0;
    if (yGyro < GYRO_MEASUREMENT_THRESHOLD && yGyro > -GYRO_MEASUREMENT_THRESHOLD) yGyro = 0;
    if (zGyro < GYRO_MEASUREMENT_THRESHOLD && zGyro > -GYRO_MEASUREMENT_THRESHOLD) zGyro = 0;

    gravity = Vector3d(x, y, z);
    Vector3d gyro = Vector3d(xGyro, yGyro, zGyro);
    yaw += gravity.dot(gyro);
    yaw = fmod(yaw, 2 * PI);

    if (print) {
      Serial.print("raw gyro: ");
      Serial.print(rxGyro);
      Serial.print("\t");
      Serial.print(ryGyro);
      Serial.print("\t");
      Serial.println(rzGyro);
      Serial.print("gyro: ");
      Serial.print(gyroVector(0));
      Serial.print("\t");
      Serial.print(gyroVector(1));
      Serial.print("\t");
      Serial.println(gyroVector(2));
      Serial.print(x);
      Serial.print(" - ");
      Serial.print(imu.readFloatAccelX());
      Serial.print("\t");
      Serial.print(y);
      Serial.print(" - ");
      Serial.print(imu.readFloatAccelY());
      Serial.print("\t");
      Serial.print(z);
      Serial.print(" - ");
      Serial.print(imu.readFloatAccelZ());
      Serial.print("\t-1\t1\t");
      Serial.print(xGyro * 1000);
      Serial.print("\t");
      Serial.print(yGyro * 1000);
      Serial.print("\t");
      Serial.print(zGyro * 1000);
      Serial.print("\t");
      Serial.println(yaw * 180 / PI);
    }

    rbuffer[0] = x;
    rbuffer[1] = y;
    rbuffer[2] = z;
    blecharacc.write(rbuffer, 3*sizeof(float));
    for (uint16_t conn_hdl=0; conn_hdl < QB_MAX_PRPH_CONNECTION; conn_hdl++)
    {
      if ( Bluefruit.connected(conn_hdl) && blecharacc.notifyEnabled(conn_hdl) )
      {
        blecharacc.notify(rbuffer, 3*sizeof(float));
      }
    }
    rgyrobuffer[0] = xGyro;
    rgyrobuffer[1] = yGyro;
    rgyrobuffer[2] = zGyro;
    blechargyr.write(rgyrobuffer, 3*sizeof(float));
    for (uint16_t conn_hdl=0; conn_hdl < QB_MAX_PRPH_CONNECTION; conn_hdl++)
    {
      if ( Bluefruit.connected(conn_hdl) && blechargyr.notifyEnabled(conn_hdl) )
      {
        blechargyr.notify(rbuffer, 3*sizeof(float));
      }
    }
  }

  void startBLEadv(void)
  {
    Serial.println("[INFO]{BLE} Start advertising...");
    // Advertising packet
    Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
    Bluefruit.Advertising.addTxPower();

    // Include HRM Service UUID
    Bluefruit.Advertising.addService(bleservice);

    // Secondary Scan Response packet (optional)
    // Since there is no room for 'Name' in Advertising packet
    Bluefruit.ScanResponse.addName();

    /* Start Advertising
    * - Enable auto advertising if disconnected
    * - Interval:  fast mode = 20 ms, slow mode = 152.5 ms
    * - Timeout for fast mode is 30 seconds
    * - Start(timeout) with timeout = 0 will advertise forever (until connected)
    *
    * For recommended advertising interval
    * https://developer.apple.com/library/content/qa/qa1931/_index.html
    */
    Bluefruit.Advertising.restartOnDisconnect(true);
    Bluefruit.Advertising.setInterval(32, 244);    // in unit of 0.625 ms
    Bluefruit.Advertising.setFastTimeout(30);      // number of seconds in fast mode
    Bluefruit.Advertising.start(0);                // 0 = Don't stop advertising after n seconds
  }

}; // end class

Qbead *Qbead::singletoninstance = nullptr;

} // end namespace

#endif // QBEAD_H