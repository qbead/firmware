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
#define QB_IX 1
#define QB_IY 0
#define QB_IZ 2
#define QB_SX 0
#define QB_SY 0
#define QB_SZ 1
#define GYRO_GATE_THRESHOLD 8
#define QB_PIXEL_COUNT 62
#define QB_MAX_PRPH_CONNECTION 2
#define T_ACC 100000
#define T_GYRO 10000

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
const std::complex<float>i(0, 1);

// TODO manage namespaces better
// The setPixelColor switches blue and green
static uint32_t color(uint8_t r, uint8_t g, uint8_t b) {
  return ((uint32_t)r << 16) | ((uint16_t)b << 8) | g;
}

static uint8_t redch(uint32_t rgb) {
  return rgb >> 16;
}

static uint8_t greench(uint32_t rgb) {
  return 0x0000ff & rgb;
}

static uint8_t bluech(uint32_t rgb) {
  return (0x00ff00 & rgb) >> 8;
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

// Tap detection
LSM6DS3 myIMU(I2C_MODE, QB_IMU_ADDR); // Create an instance of the IMU
uint8_t interruptCount = 0; // Amount of received interrupts
uint8_t prevInterruptCount = 0; // Interrupt Counter from last loop

void setupTapInterrupt() {
  uint8_t error = 0;
  uint8_t dataToWrite = 0;

  // Double Tap Config
  myIMU.writeRegister(LSM6DS3_ACC_GYRO_CTRL1_XL, 0x60);
  myIMU.writeRegister(LSM6DS3_ACC_GYRO_TAP_CFG1, 0x8E);// INTERRUPTS_ENABLE, SLOPE_FDS
  myIMU.writeRegister(LSM6DS3_ACC_GYRO_TAP_THS_6D, 0x6C);
  myIMU.writeRegister(LSM6DS3_ACC_GYRO_INT_DUR2, 0x7F);
  myIMU.writeRegister(LSM6DS3_ACC_GYRO_WAKE_UP_THS, 0x80);
  myIMU.writeRegister(LSM6DS3_ACC_GYRO_MD1_CFG, 0x08);
}

void int1ISR()
{
  interruptCount++;
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

  // In rads
  float theta()
  {
    return acos(v(2));
  }

  // In rads
  float phi()
  {
    return atan2(v(1), v(0));
  }

  Vector2cf stateVector2D()
  {
    std::complex<float> alpha = cos(theta()/2);
    std::complex<float> beta = exp(i*phi()) * sin(theta()/2);
    return {alpha, beta};
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
    const float a = (cos(theta) + 1) / 2; // probability of measuring |0>
    if (a < 0.0001) {
      stateCoordinates.set(0, 0, -1);
      return;
    } else if (a > 0.9999) {
      stateCoordinates.set(0, 0, 1);
      return;
    }
    const bool is1 = random(0, 100) <= a * a * 100;
    this->stateCoordinates.set(0, 0, is1 ? 1 : -1);
  }

  void applyGate(Matrix2cf gate)
  {
    Vector2cf stateVector = stateCoordinates.stateVector2D();
    stateVector = gate * stateVector;
    stateVector.normalize();
    stateCoordinates.set(2*acos(abs(stateVector.x())), arg(stateVector.y()) - arg(stateVector.x()));
  }

  void applyGateType(uint16_t gateType, float rotationDegree = PI)
  {
    switch (gateType)
    {
    case 1:
      gateX(-rotationDegree);
      break;
    case 2:
      gateY(-rotationDegree);
      break;
    case 3:
      gateZ(rotationDegree);
      break;
    case 4:
      gateX(rotationDegree);
      break;
    case 5:
      gateY(rotationDegree);
      break;
    case 6:
      gateZ(-rotationDegree);
      break;
    case 7:
      gateH(rotationDegree);
      break;
    default:
      break;
    }
  }

  // Rotate PI around the x axis
  void gateX(float rotationDegree = PI)
  {
    Matrix2cf gateMatrix;
    gateMatrix << cos(rotationDegree / 2.0f), -sin(rotationDegree / 2.0f) * i,
        -sin(rotationDegree / 2.0f) * i, cos(rotationDegree / 2.0f); // global phase differs from pauli gates but this doesn't matter for bloch sphere
    applyGate(gateMatrix);
  }

  // Rotate PI around the y axis
  void gateZ(float rotationDegree = PI)
  {
    Matrix2cf gateMatrix;
    gateMatrix << exp(-i * rotationDegree / 2.0f), 0,
        0, exp(i * rotationDegree / 2.0f);
    applyGate(gateMatrix);
  }

  // Rotate PI around the z axis
  void gateY(float rotationDegree = PI)
  {
    Matrix2cf gateMatrix;
    gateMatrix << cos(rotationDegree / 2.0f), -sin(rotationDegree / 2.0f),
        sin(rotationDegree / 2.0f), cos(rotationDegree / 2.0f);
    applyGate(gateMatrix);
  }

  // Rotate PI around the xz axis
  void gateH(float rotationDegree = PI)
  {
    Matrix2cf gateMatrix;
    gateMatrix << (cos(rotationDegree / 2.0f) - i * sin(rotationDegree / 2.0f) / sqrt(2.0f)), -i * sin(rotationDegree / 2.0f) / sqrt(2.0f),
        -i * sin(rotationDegree / 2.0f) / sqrt(2.0f), (cos(rotationDegree / 2.0f) + i * sin(rotationDegree / 2.0f) / sqrt(2.0f));
    applyGate(gateMatrix);
  }
};

class Qbead {
public:
  Qbead(const uint16_t pin00 = QB_LEDPIN,
        const uint16_t pixelconfig = QB_PIXELCONFIG,
        const uint8_t imu_addr = QB_IMU_ADDR)
      : imu(LSM6DS3(I2C_MODE, imu_addr)),
        pixels(Adafruit_NeoPixel(QB_PIXEL_COUNT, pin00, pixelconfig)),
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

  float rbuffer[3], rgyrobuffer[3];
  float T_imu;             // last update from the IMU
  float T_freeze = 0;
  float T_shaking = 0;
  float shakingCounter = 0;
  bool frozen = false; // frozen means that there is an animation in progress
  bool shakingState = false; // if ShakingState is 1 detected shaking and if shaking keeps happening randomising state
  QuantumState state = QuantumState(Coordinates(-0.866, 0.25, -0.433));
  Coordinates visualState = Coordinates(-0.866, 0.25, -0.433);
  Vector3d gravityVector = Vector3d(0, 0, 1);
  Vector3d gyroVector = Vector3d(0, 0, 1);
  float yaw = 0;

  float t_ble, p_ble; // theta and phi as sent over BLE connection
  uint32_t c_ble = 0xffffff; // color as sent over BLE connection

  // led map index to Coordinates
  // This map is for the first version of the flex-pcb
  Coordinates led_map_v1[62] = {
    Coordinates(-1, -0, -0),
    Coordinates(-0.866, 0, -0.5),
    Coordinates(-0.5, 0, -0.866),
    Coordinates(-0, 0, -1),
    Coordinates(0.5, 0, -0.866),
    Coordinates(0.866, 0, -0.5),
    Coordinates(1, 0, 0),
    Coordinates(-0.866, 0.25, -0.433),
    Coordinates(-0.5, 0.433, -0.75),
    Coordinates(-0, 0.5, -0.866),
    Coordinates(0.5, 0.433, -0.75),
    Coordinates(0.866, 0.25, -0.433),
    Coordinates(-0.866, 0.433, -0.25),
    Coordinates(-0.5, 0.75, -0.433),
    Coordinates(-0, 0.866, -0.5),
    Coordinates(0.5, 0.75, -0.433),
    Coordinates(0.866, 0.433, -0.25),
    Coordinates(-0.866, 0.5, 0),
    Coordinates(-0.5, 0.866, 0),
    Coordinates(-0, 1, 0),
    Coordinates(0.5, 0.866, 0),
    Coordinates(0.866, 0.5, 0),
    Coordinates(-0.866, 0.433, 0.25),
    Coordinates(-0.5, 0.75, 0.433),
    Coordinates(-0, 0.866, 0.5),
    Coordinates(0.5, 0.75, 0.433),
    Coordinates(0.866, 0.433, 0.25),
    Coordinates(-0.866, 0.25, 0.433),
    Coordinates(-0.5, 0.433, 0.75),
    Coordinates(-0, 0.5, 0.866),
    Coordinates(0.5, 0.433, 0.75),
    Coordinates(0.866, 0.25, 0.433),
    Coordinates(-0.866, -0, 0.5),
    Coordinates(-0.5, -0, 0.866),
    Coordinates(-0, -0, 1),
    Coordinates(0.5, -0, 0.866),
    Coordinates(0.866, -0, 0.5),
    Coordinates(-0.866, -0.25, 0.433),
    Coordinates(-0.5, -0.433, 0.75),
    Coordinates(-0, -0.5, 0.866),
    Coordinates(0.5, -0.433, 0.75),
    Coordinates(0.866, -0.25, 0.433),
    Coordinates(-0.866, -0.433, 0.25),
    Coordinates(-0.5, -0.75, 0.433),
    Coordinates(-0, -0.866, 0.5),
    Coordinates(0.5, -0.75, 0.433),
    Coordinates(0.866, -0.433, 0.25),
    Coordinates(-0.866, -0.5, -0),
    Coordinates(-0.5, -0.866, -0),
    Coordinates(-0, -1, -0),
    Coordinates(0.5, -0.866, -0),
    Coordinates(0.866, -0.5, -0),
    Coordinates(-0.866, -0.433, -0.25),
    Coordinates(-0.5, -0.75, -0.433),
    Coordinates(-0, -0.866, -0.5),
    Coordinates(0.5, -0.75, -0.433),
    Coordinates(0.866, -0.433, -0.25),
    Coordinates(-0.866, -0.25, -0.433),
    Coordinates(-0.5, -0.433, -0.75),
    Coordinates(-0, -0.5, -0.866),
    Coordinates(0.5, -0.433, -0.75),
    Coordinates(0.866, -0.25, -0.433),
  };

  // TODO: check when the flex-pcb is in production
  Coordinates led_map_v2[107] = {
    Coordinates(0.0, 0.0),
    Coordinates(0.39, 4.97),
    Coordinates(0.78, 4.97),
    Coordinates(1.18, 5.08),
    Coordinates(1.18, 4.87),
    Coordinates(1.57, 4.89),
    Coordinates(1.57, 5.06),
    Coordinates(1.95, 5.04),
    Coordinates(1.95, 4.91),
    Coordinates(2.34, 4.97),
    Coordinates(2.73, 4.97),
    Coordinates(0.78, 4.45),
    Coordinates(1.18, 4.55),
    Coordinates(1.18, 4.35),
    Coordinates(1.57, 4.37),
    Coordinates(1.57, 4.53),
    Coordinates(1.95, 4.51),
    Coordinates(1.95, 4.39),
    Coordinates(2.34, 4.45),
    Coordinates(0.39, 3.93),
    Coordinates(0.78, 3.93),
    Coordinates(1.18, 4.03),
    Coordinates(1.18, 3.82),
    Coordinates(1.57, 3.84),
    Coordinates(1.57, 4.01),
    Coordinates(1.95, 3.99),
    Coordinates(1.95, 3.87),
    Coordinates(2.34, 3.93),
    Coordinates(2.73, 3.93),
    Coordinates(0.78, 3.4),
    Coordinates(1.18, 3.3),
    Coordinates(1.57, 3.32),
    Coordinates(1.95, 3.34),
    Coordinates(2.34, 3.4),
    Coordinates(0.39, 2.88),
    Coordinates(0.78, 2.88),
    Coordinates(1.18, 2.98),
    Coordinates(1.18, 2.78),
    Coordinates(1.57, 2.8),
    Coordinates(1.57, 2.96),
    Coordinates(1.95, 2.94),
    Coordinates(1.95, 2.82),
    Coordinates(2.34, 2.88),
    Coordinates(2.73, 2.88),
    Coordinates(0.78, 2.36),
    Coordinates(1.18, 2.46),
    Coordinates(1.18, 2.25),
    Coordinates(1.57, 2.27),
    Coordinates(1.57, 2.44),
    Coordinates(1.95, 2.42),
    Coordinates(1.95, 2.29),
    Coordinates(2.34, 2.36),
    Coordinates(0.39, 1.83),
    Coordinates(0.78, 1.83),
    Coordinates(1.18, 1.93),
    Coordinates(1.18, 1.73),
    Coordinates(1.57, 1.75),
    Coordinates(1.57, 1.92),
    Coordinates(1.95, 1.89),
    Coordinates(1.95, 1.77),
    Coordinates(2.34, 1.83),
    Coordinates(2.73, 1.83),
    Coordinates(0.78, 1.31),
    Coordinates(1.18, 1.41),
    Coordinates(1.18, 1.21),
    Coordinates(1.57, 1.23),
    Coordinates(1.57, 1.39),
    Coordinates(1.95, 1.37),
    Coordinates(1.95, 1.25),
    Coordinates(2.34, 1.31),
    Coordinates(0.39, 0.79),
    Coordinates(0.78, 0.79),
    Coordinates(1.18, 0.89),
    Coordinates(1.18, 0.68),
    Coordinates(1.57, 0.7),
    Coordinates(1.57, 0.87),
    Coordinates(1.95, 0.85),
    Coordinates(1.95, 0.72),
    Coordinates(2.34, 0.79),
    Coordinates(2.73, 0.79),
    Coordinates(0.78, 0.26),
    Coordinates(1.18, 0.36),
    Coordinates(1.18, 0.16),
    Coordinates(1.57, 0.18),
    Coordinates(1.57, 0.35),
    Coordinates(1.95, 0.32),
    Coordinates(1.95, 0.2),
    Coordinates(2.34, 0.26),
    Coordinates(0.39, 6.02),
    Coordinates(0.78, 6.02),
    Coordinates(1.18, 6.12),
    Coordinates(1.18, 5.92),
    Coordinates(1.57, 5.94),
    Coordinates(1.57, 6.1),
    Coordinates(1.95, 6.08),
    Coordinates(1.95, 5.96),
    Coordinates(2.34, 6.02),
    Coordinates(2.73, 6.02),
    Coordinates(0.78, 5.5),
    Coordinates(1.18, 5.6),
    Coordinates(1.18, 5.4),
    Coordinates(1.57, 5.41),
    Coordinates(1.57, 5.58),
    Coordinates(1.95, 5.56),
    Coordinates(1.95, 5.44),
    Coordinates(2.34, 5.5),
    Coordinates(3.14, 4.97),
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

  void startAccelerometer() {
    // BLE Characteristic IMU xyz accelerometer readout
    blecharacc.setProperties(CHR_PROPS_READ | CHR_PROPS_NOTIFY);
    blecharacc.setPermission(SECMODE_OPEN, SECMODE_OPEN);
    blecharacc.setUserDescriptor("xyz acceleration");
    blecharacc.setFixedLen(3*sizeof(float));
    blecharacc.begin();
    blecharacc.write(zerobuffer20, 3*sizeof(float));    
  }

  void begin() {
    singletoninstance = this;
    Serial.begin(9600);
    for (int waitCount = 0; waitCount < 50; waitCount++)
    {
      if (Serial) {break;}
      delay(100);
    }

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
    // BLE Characteristic IMU xyz gyroscope readout
    blechargyr.setProperties(CHR_PROPS_READ | CHR_PROPS_NOTIFY);
    blechargyr.setPermission(SECMODE_OPEN, SECMODE_OPEN);
    blechargyr.setUserDescriptor("xyz gyroscope");
    blechargyr.setFixedLen(3*sizeof(float));
    blechargyr.begin();
    blechargyr.write(zerobuffer20, 3*sizeof(float));
    startBLEadv();

    // Tap detection
    setupTapInterrupt();
    pinMode(PIN_LSM6DS3TR_C_INT1, INPUT);
    attachInterrupt(digitalPinToInterrupt(PIN_LSM6DS3TR_C_INT1), int1ISR, RISING);
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

  void setLed(Coordinates coordinates, uint32_t color, int leds = 1) {
    float theta = coordinates.theta() * 180 / PI;
    float phi = coordinates.phi() * 180 / PI;
    if (phi < 0) {
      phi += 360;
    }
    setBloch_deg(theta, phi, color, leds);
  }

  void showAxis() {
    setLed(Coordinates(1, 0, 0), color(0, 0, 122));
    setLed(Coordinates(-1, 0, 0), color(0, 0, 122));
    setLed(Coordinates(0, 1, 0), color(0, 0, 122));
    setLed(Coordinates(0, -1, 0), color(0, 0, 122));
    setLed(Coordinates(0, 0, 1), color(0, 255, 0));
    setLed(Coordinates(0, 0, -1), color(255, 0, 0));
  }

  // in rads
  float getDistToLed(float theta, float phi, int index) {
    const Coordinates led = led_map_v1[index];
    const Coordinates reference(theta, phi);
    return led.dist(reference.v);
  }

  // Single bit is lit up on the Bloch sphere  
  void setBloch_deg(float theta, float phi, uint32_t c, int leds = 1) {
    int index[leds];
    float dist[leds];
    for (int i = 0; i < leds; i++) {
      index[i] = -1;
      dist[i] = 1000;
    }
    for (int i = 0; i < QB_PIXEL_COUNT; i++) {
      float d = getDistToLed(theta * PI / 180, phi * PI / 180, i);
      for (int j = 0; j < leds; j++) {
        if (d < dist[j]) {
          for (int k = leds - 1; k > j; k--) {
            index[k] = index[k - 1];
            dist[k] = dist[k - 1];
          }
          index[j] = i;
          dist[j] = d;
          break;
        }
      }
    }
    for (int i = 0; i < leds; i++) {
      if (index[i] != -1) {
        uint8_t r = redch(c);
        uint8_t g = greench(c);
        uint8_t b = bluech(c);
        float p2 =  pow(200, -dist[i]);
        pixels.setPixelColor(index[i], color(p2 * r, p2 * g, p2 * b));
      }
    }
  }

  void setBloch_deg_smooth(float theta, float phi, uint32_t c) {
    setBloch_deg(theta, phi, c, 2);
  }

  void animateTo(uint8_t gate, uint16_t animationLength = 2000)
  {
    if (frozen)
    {
      prevInterruptCount = interruptCount;
    }
    else if (gate == 0)
    {
      return;
    }
    if (gate == 9)
    {
      visualState.set(state.getCoordinates().v);
    }
    if (gate == 8)
    {
      state.collapse();
      visualState.set(state.getCoordinates().v);
    }
    float T_new = millis();
    float delta = T_new - T_freeze;
    if (delta > animationLength)
    {
      frozen = false;
      state.applyGateType(gate);
      Serial.println("Animation finished");
      return;
    }
    float d = delta * PI / float(animationLength);
    QuantumState from = state;
    from.applyGateType(gate, d);
    visualState.set(from.getCoordinates().v);
  }

  bool detectShaking()
  {
    float totalAcceleration = gravityVector.norm();
    if (shakingState)
    {
      float newTime = millis();
      shakingCounter += newTime - T_shaking;
      T_shaking = newTime;
      if (shakingCounter < 300)
      {
        return false;
      }
      if (totalAcceleration > 11)
      {
        Serial.println("RANDOMISING");
        float randomTheta = (random(0, 1000)/1000.0f) * PI;
        float randomPhi = (random(0, 1000)/500.0f) * PI;
        state.setCoordinates(Coordinates(randomTheta, randomPhi));
        setLed(state.getCoordinates(), color(255, 0, 255));
        show();
        shakingState = false;
        prevInterruptCount = interruptCount;
        return true;
      }
      if (shakingCounter > 800)
      {
        shakingState = false;
      }
      return false;
    }
    if (totalAcceleration > 11)
    {
      Serial.print("Detected shaking turning on shakingState, acc length: ");
      Serial.println(totalAcceleration);
      shakingState = true;
      T_shaking = millis();
      shakingCounter = 0;
    } 
    return false;
  }

  int checkMotion()
  {
    if (frozen)
    {
      return 0;
    }
    frozen = true;
    T_freeze = micros();
    if (detectShaking())
    {
      return 9;
    }
    if (shakingState)
    {
      frozen = false;
      return 0;
    }
    // Handle tap interrupt
    if (interruptCount > prevInterruptCount)
    {
      uint8_t tapStatus = 0;
      myIMU.readRegister(&tapStatus, LSM6DS3_ACC_GYRO_TAP_SRC);
      prevInterruptCount = interruptCount;

      if (tapStatus & 0x01)
      {
        Serial.println("Collapsing");
        return 8;
      }
      else
      {
        Serial.println("Executing H gate");
        return 7;
      }
    }
    // Handle shaking
    for (int i = 0; i < 3; i++)
    {
      if (gyroVector[i] > GYRO_GATE_THRESHOLD)
      {
        return i + 1; // 1 = -x, 2 = -y, 3 = z
      }
    }
    for (int i = 0; i < 3; i++)
    {
      if (gyroVector[i] < - GYRO_GATE_THRESHOLD)
      {
        return i + 4; // 4 = x, 5 = y, 6 = -z
      }
    }
    frozen = false;
    return 0;
  }

  void writeToBLE(BLECharacteristic& destination, Vector3d vector) {
    float buffer[3] = {(float)vector(0), (float)vector(1), (float)vector(2)};
    destination.write(buffer, 3 * sizeof(float));
    for (uint16_t conn_hdl = 0; conn_hdl < QB_MAX_PRPH_CONNECTION; conn_hdl++)
    {
      if (Bluefruit.connected(conn_hdl) && destination.notifyEnabled(conn_hdl))
      {
        destination.notify(buffer, 3 * sizeof(float));
      }
    }
  }

  Vector3d getVectorFromBuffer(float *buffer) {
    // calibration of imu because imu is not aligned with bloch sphere
    float rx = (1 - 2 * QB_SX) * buffer[QB_IX];
    float ry = (1 - 2 * QB_SY) * buffer[QB_IY];
    float rz = (1 - 2 * QB_SZ) * buffer[QB_IZ];
    return Vector3d(rx, ry, rz);
  }

  void readIMU(bool print=true) {
    rbuffer[0] = imu.readFloatAccelX();
    rbuffer[1] = imu.readFloatAccelY();
    rbuffer[2] = imu.readFloatAccelZ();    
    rgyrobuffer[0] = imu.readFloatGyroX();
    rgyrobuffer[1] = imu.readFloatGyroY();
    rgyrobuffer[2] = imu.readFloatGyroZ();

    float T_new = micros();
    float delta = T_new - T_imu;
    T_imu = T_new;

    Vector3d newGyro = getVectorFromBuffer(rgyrobuffer) * PI / 180;
    float d = min(delta / float(T_GYRO), 1.0f);
    gyroVector = d * newGyro + (1 - d) * gyroVector; // low pass filter

    Vector3d newGravity = getVectorFromBuffer(rbuffer);
    d = min(delta / float(T_ACC), 1.0f);
    gravityVector = d * newGravity + (1 - d) * gravityVector;

    yaw += gravityVector.dot(gyroVector);
    yaw = fmod(yaw, 2 * PI);

    if (print) {
      Serial.print(gravityVector(0));
      Serial.print("\t");
      Serial.print(gravityVector(1));
      Serial.print("\t");
      Serial.print(gravityVector(2));
      Serial.print("\t-1\t1\t");
      Serial.print(gyroVector(0));
      Serial.print("\t");
      Serial.print(gyroVector(1));
      Serial.print("\t");
      Serial.println(gyroVector(2));
    }

    writeToBLE(blecharacc, gravityVector);
    writeToBLE(blechargyr, gyroVector);
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