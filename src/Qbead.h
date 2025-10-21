#ifndef QBEAD_H
#define QBEAD_H


#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <LSM6DS3.h>
#include <math.h>

#include <bluefruit.h>

// default configs
// TODO make them ifndef
#define QB_LEDPIN 10
#define QB_PIXELCONFIG NEO_GRB + NEO_KHZ800
#define QB_NSECTIONS 6
#define QB_NLEGS 12
#define QB_IMU_ADDR 0x6A
#define QB_IX 2
#define QB_IY 0
#define QB_IZ 1
#define QB_SX 0
#define QB_SY 0
#define QB_SZ 0

#define QB_MAX_PRPH_CONNECTION 2

const uint8_t QB_UUID_SERVICE[] =
{0x45,0x8d,0x08,0xaa,0xd6,0x63,0x44,0x25,0xbe,0x12,0x9c,0x35,0xc6,0x1f,0x0c,0xe3};
const uint8_t QB_UUID_COL_CHAR[] =
{0x45,0x8d,0x08,0xaa,0xd6,0x63,0x44,0x25,0xbe,0x12,0x9c,0x35,0xc6+1,0x1f,0x0c,0xe3};
const uint8_t QB_UUID_SPH_CHAR[] =
{0x45,0x8d,0x08,0xaa,0xd6,0x63,0x44,0x25,0xbe,0x12,0x9c,0x35,0xc6+2,0x1f,0x0c,0xe3};
const uint8_t QB_UUID_ACC_CHAR[] =
{0x45,0x8d,0x08,0xaa,0xd6,0x63,0x44,0x25,0xbe,0x12,0x9c,0x35,0xc6+3,0x1f,0x0c,0xe3};
const uint8_t QB_UUID_TAP_CHAR[] =
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

static uint32_t addColor(uint32_t c0, uint32_t c1) {
  uint8_t r = min(0xff, (int)redch(c0) + redch(c1));
  uint8_t g = min(0xff, (int)greench(c0) + greench(c1));
  uint8_t b = min(0xff, (int)bluech(c0) + bluech(c1));
  
  return color(r, g, b);
}

static uint32_t mulColor(float a, uint32_t c) {
  uint8_t r = min(0xff, a * redch(c));
  uint8_t g = min(0xff, a * greench(c));
  uint8_t b = min(0xff, a * bluech(c));
  
  return color(r, g, b);
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

static float toRadians(const float angle) {
  return angle / 180 * M_PI;
}

static float toDegrees(const float angle) {
  return angle * 180 / M_PI;
}

static float sin_deg(const float angle) {
  return sin(toRadians(angle));
}

static float cos_deg(const float angle) {
  return cos(toRadians(angle));
}

// Mod function that only returns positive numbers
static float modulo(const float a, const float b) {
  return fmod(a, b) + (a < 0) * b;
}

class BlochVector
{
  float theta; //Degrees
  float phi; //Degrees
  float x;
  float y;
  float z;

public:
  BlochVector() : theta(0), phi(0), x(0), y(0), z(1) {}

  BlochVector(const float theta_in, const float phi_in) {
    const float theta_mod360 = modulo(theta_in, 360);
    theta = (theta_mod360 < 180) ? theta_mod360 : 360 - theta_mod360;
    phi = modulo(phi_in + (theta_mod360 > 180) * 180, 360);
    x = cos_deg(phi_in) * sin_deg(theta_in); 
    y = sin_deg(phi_in) * sin_deg(theta_in); 
    z = cos_deg(theta_in);
  }

  BlochVector(const float x_in, const float y_in, const float z_in) {
    theta = toDegrees(atan2(sqrt(x_in*x_in + y_in*y_in), z_in));
    phi = modulo(toDegrees(atan2(y_in, x_in)), 360);
    const float r = sqrt(x_in*x_in + y_in*y_in + z_in*z_in);
    x = x_in / r; 
    y = y_in / r; 
    z = z_in / r;
  }
  
  BlochVector& operator=(const BlochVector& other) {
    theta = other.getTheta();
    phi = other.getPhi();
    x = other.getX();
    y = other.getY();
    z = other.getZ();
    return *this;
  }

  BlochVector rotatedAround(const BlochVector& axis, const float angle) const {
    const float axis_x = axis.getX();
    const float axis_y = axis.getY();
    const float axis_z = axis.getZ();
    const float dot_product = x * axis_x + y * axis_y + z * axis_z;
    const float new_x = x*cos_deg(angle) + (axis_y*z - axis_z*y)*sin_deg(angle) + axis_x*dot_product*(1-cos_deg(angle));
    const float new_y = y*cos_deg(angle) + (axis_z*x - axis_x*z)*sin_deg(angle) + axis_y*dot_product*(1-cos_deg(angle));
    const float new_z = z*cos_deg(angle) + (axis_x*y - axis_y*x)*sin_deg(angle) + axis_z*dot_product*(1-cos_deg(angle));

    BlochVector result(new_x, new_y, new_z);
    return result;
  }
  
  BlochVector& rotateAround(const BlochVector& axis, const float angle) {
    *this = this->rotatedAround(axis, angle);
    return *this;
  } 

  float centralAngle(const BlochVector& other) const {
    return toDegrees(acos(cos_deg(theta) * cos_deg(other.getTheta()) 
                        + sin_deg(theta) * sin_deg(other.getTheta()) * cos_deg(phi - other.getPhi())));
  }

  float getTheta() const {return theta;}
  float getTheta_rad() const {return theta * M_PI / 180;}
  float getPhi() const {return phi;}
  float getPhi_rad() const {return phi * M_PI / 180;}
  float getX() const {return x;}
  float getY() const {return y;}
  float getZ() const {return z;}

  void setXYZ(const float x, const float y, const float z){
    BlochVector new_vector(x, y, z);
    *this = new_vector;
  }

  void setAngles(const float theta, const float phi){
    BlochVector new_vector(theta, phi);
    *this = new_vector;
  }
};

float centralAngle(const BlochVector& v, const BlochVector& u) {
  return v.centralAngle(u);
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
        blechartap(QB_UUID_TAP_CHAR)
        {}

  static Qbead *singletoninstance; // we need a global singleton static instance because bluefruit callbacks do not support context variables -- thankfully this is fine because there is indeed only one Qbead in existence at any time

  LSM6DS3 imu;
  Adafruit_NeoPixel pixels;

  BLEService bleservice;
  BLECharacteristic blecharcol;
  BLECharacteristic blecharsph;
  BLECharacteristic blecharacc;
  BLECharacteristic blechartap;
  uint8_t connection_count = 0;

  const uint8_t nsections;
  const uint8_t nlegs;
  const uint8_t theta_quant;
  const uint8_t phi_quant;
  const uint8_t ix, iy, iz;
  const bool sx, sy, sz;
  float rbuffer[3];
  float x, y, z, rx, ry, rz; // filtered and raw acc, in units of g
  BlochVector angle_acc;     // theta and phi according to gravity
  float T_imu;               // last update from the IMU

  unsigned long last_tap = 0;
  const unsigned long tap_debounce = 1000;
  const float tap_threshold = 0.9;

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
    // BLE Characteristic IMU xyz readout
    blecharacc.setProperties(CHR_PROPS_READ | CHR_PROPS_NOTIFY);
    blecharacc.setPermission(SECMODE_OPEN, SECMODE_OPEN);
    blecharacc.setUserDescriptor("xyz acceleration");
    blecharacc.setFixedLen(3*sizeof(float));
    blecharacc.begin();
    blecharacc.write(zerobuffer20, 3*sizeof(float));
    // BLE Characteristic IMU xyz tap detection
    blechartap.setProperties(CHR_PROPS_READ | CHR_PROPS_NOTIFY);
    blechartap.setPermission(SECMODE_OPEN, SECMODE_OPEN);
    blechartap.setUserDescriptor("xyz tap detection");
    blechartap.setFixedLen(3*sizeof(float));
    blechartap.begin();
    blechartap.write(zerobuffer20, 3*sizeof(float));
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

  uint32_t getLegPixelColor(int leg, int pixel) {
    leg = nlegs - leg; // invert direction for the phi angle, because the PCB is set up as a left-handed coordinate system
    leg = leg % nlegs;
    if (leg == 0) {
      return pixels.getPixelColor(pixel);
    } 
    if (pixel == 0) {
      return pixels.getPixelColor(0);
    } 
    if (pixel == 6) {
      return pixels.getPixelColor(6);
    } 
    return pixels.getPixelColor(7 + (leg - 1) * (nsections - 1) + pixel - 1);
  }

  void addLegPixelColor(int leg, int pixel, uint32_t c0) {
    uint32_t c1 = getLegPixelColor(leg, pixel);
    setLegPixelColor(leg, pixel, addColor(c0, c1));
  }

  void setBrightness(uint8_t b) {
    pixels.setBrightness(b);
  }

  void setBloch_deg(BlochVector& state, uint32_t color) {
    float theta = state.getTheta();
    float phi = state.getPhi();
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

  void setBloch_deg_smooth(BlochVector& state, uint32_t color) {
    float width = 20;

    for (int phi_pixel = 0; phi_pixel <= nlegs; ++phi_pixel) {
      for (int theta_pixel = 0; theta_pixel <= nsections; ++theta_pixel) {
        float internal_angle = state.centralAngle(BlochVector(theta_quant * theta_pixel, phi_quant * phi_pixel));
        if (internal_angle > width) {
          continue;
        }

        float brightness = 1 - internal_angle * internal_angle / (width * width);

        addLegPixelColor(phi_pixel, theta_pixel, mulColor(brightness, color));
      }
    }
  }

  void readIMU(bool print=true) {
    rbuffer[0] = imu.readFloatAccelX();
    rbuffer[1] = imu.readFloatAccelY();
    rbuffer[2] = imu.readFloatAccelZ();
    rx = (1-2*sx)*rbuffer[ix];
    ry = (1-2*sy)*rbuffer[iy];
    rz = (1-2*sz)*rbuffer[iz];

    float rawmag2 = rx*rx+ry*ry+rz*rz;

    float T_new = micros();
    float delta = T_new - T_imu;
    T_imu = T_new;
    const float T = 100000; // 100 ms // TODO make the filter timeconstant configurable
    if (delta > 100000) {
      x = rx;
      y = ry;
      z = rz;
    } else {
      float d = delta/T;
      x = d*rx+(1-d)*x;
      y = d*ry+(1-d)*y;
      z = d*rz+(1-d)*z;
    }
    float mag2 = x*x+y*y+z*z;
    
    angle_acc.setXYZ(x, y, z); 

    if (print) {
      Serial.print(x);
      Serial.print("\t");
      Serial.print(y);
      Serial.print("\t");
      Serial.print(z);
      Serial.print("\t");
      Serial.print(mag2);
      Serial.print("\t");
      Serial.print(rawmag2);
      Serial.print("\t-1\t1\t");
      Serial.print(angle_acc.getTheta());
      Serial.print("\t");
      Serial.print(angle_acc.getPhi());
      Serial.print("\t-360\t360\t");
      Serial.println();
    }

    rbuffer[0] = x;
    rbuffer[1] = y;
    rbuffer[2] = z;
    blecharacc.write(rbuffer, 3*sizeof(float));

    bool a_tap = abs(rawmag2-1) > tap_threshold && millis()-last_tap>tap_debounce;

    if (a_tap)
    {
      last_tap = millis();
      blecharacc.write(rbuffer, 3*sizeof(float));
    }

    for (uint16_t conn_hdl=0; conn_hdl < QB_MAX_PRPH_CONNECTION; conn_hdl++)
    {
      if ( Bluefruit.connected(conn_hdl) && blecharacc.notifyEnabled(conn_hdl) )
      {
        blecharacc.notify(rbuffer, 3*sizeof(float));
        a_tap && blechartap.notify(rbuffer, 3*sizeof(float));
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
