#ifndef QBEAD_H
#define QBEAD_H


#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <LSM6DS3.h>
#include <math.h>
#include <ArduinoEigen.h>
#include <bluefruit.h>

// default configs
#define QB_LEDPIN 10
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

namespace Qbead {

class Qbead {
public:
  Qbead(const uint16_t pin00 = QB_LEDPIN,
        const uint16_t pixelconfig = QB_PIXELCONFIG,
        const uint16_t nsections = QB_NSECTIONS,
        const uint16_t nlegs = QB_NLEGS,
        const uint8_t imu_addr = QB_IMU_ADDR,
        const uint8_t ix = QB_IX,
        const uint8_t iy = QB_IY,
        const uint8_t iz = QB_IZ,
        const bool sx = QB_SX,
        const bool sy = QB_SY,
        const bool sz = QB_SZ)
      : imu(LSM6DS3(I2C_MODE, imu_addr)),
        pixels(Adafruit_NeoPixel(nlegs * (nsections - 1) + 2, pin00, pixelconfig)),
        nsections(nsections),
        nlegs(nlegs),
        theta_quant(180 / nsections),
        phi_quant(360 / nlegs),
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

  const uint8_t nsections;
  const uint8_t nlegs;
  const uint8_t theta_quant;
  const uint8_t phi_quant;
  const uint8_t ix, iy, iz;
  const bool sx, sy, sz;
  float rbuffer[3], rgyrobuffer[3];
  float t_acc, p_acc;        // theta and phi according to gravity
  float T_imu;             // last update from the IMU  
  Vector3d gravityVector = Vector3d(0, 0, 1);
  Vector3d gyroVector = Vector3d(0, 0, 1);
  float yaw = 0;

  float t_ble, p_ble; // theta and phi as sent over BLE connection
  uint32_t c_ble = 0xffffff; // color as sent over BLE connection

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

  void setLegPixelColor(int leg, int pixel, uint32_t color) {
    leg = nlegs - leg; // invert direction for the phi angle, because the PCB is set up as a left-handed coordinate system
    leg = leg % nlegs;
    if (leg == 0) {
      pixels.setPixelColor(pixel, color);
    } else if (pixel == 0) {
      pixels.setPixelColor(0, color);
    } else if (pixel == 6) {
      pixels.setPixelColor(6, color);
    } else {
      pixels.setPixelColor(7 + (leg - 1) * (nsections - 1) + pixel - 1, color);
    }
  }

  void setBrightness(uint8_t b) {
    pixels.setBrightness(b);
  }

  void setBloch_deg(float theta, float phi, uint32_t color) {
    if (!checkThetaAndPhi(theta, phi)) return;
    float theta_section = theta / theta_quant;
    if (theta_section < 0.5) {
      setLegPixelColor(0, 0, color);
    } else if (theta_section > nsections - 0.5) {
      setLegPixelColor(0, nsections, color);
    } else {
      int theta_int = min(nsections - 1, round(theta_section)); // to avoid precision issues near the end of the range
      int phi_int = round(phi / phi_quant);
      phi_int = phi_int > nlegs - 1 ? 0 : phi_int;
      setLegPixelColor(phi_int, theta_int, color);
    }
  }

  void setBloch_deg_smooth(float theta, float phi, uint32_t c) {
    if (!checkThetaAndPhi(theta, phi)) return;
    float theta_section = theta / theta_quant;
    int theta_int = min(nsections - 1, round(theta_section)); // to avoid precision issues near the end of the range
    int phi_int = round(phi / phi_quant);
    phi_int = phi_int > nlegs - 1 ? 0 : phi_int;

    float p = (theta_section - theta_int);
    int theta_direction = sign(p);
    p = abs(p);
    float q = 1 - p;
    p = p * p;
    q = q * q;

    uint8_t rc = redch(c);
    uint8_t bc = bluech(c);
    uint8_t gc = greench(c);

    setLegPixelColor(phi_int, theta_int, color(q * rc, q * bc, q * gc));
    setLegPixelColor(phi_int, theta_int + theta_direction, color(p * rc, p * bc, p * gc));
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
