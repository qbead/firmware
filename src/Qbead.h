#ifndef QBEAD_H
#define QBEAD_H

#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <LSM6DS3.h>
#include <math.h>
#include <bluefruit.h>

// default configs
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

#define QB_MAX_PRPH_CONNECTION 2

const uint8_t QB_UUID_SERVICE[] =
{0x45,0x8d,0x08,0xaa,0xd6,0x63,0x44,0x25,0xbe,0x12,0x9c,0x35,0xc6,0x1f,0x0c,0xe3};
const uint8_t QB_UUID_COL_CHAR[] =
{0x45,0x8d,0x08,0xaa,0xd6,0x63,0x44,0x25,0xbe,0x12,0x9c,0x35,0xc6+1,0x1f,0x0c,0xe3};
const uint8_t QB_UUID_SPH_CHAR[] =
{0x45,0x8d,0x08,0xaa,0xd6,0x63,0x44,0x25,0xbe,0x12,0x9c,0x35,0xc6+2,0x1f,0x0c,0xe3};
const uint8_t QB_UUID_ACC_CHAR[] =
{0x45,0x8d,0x08,0xaa,0xd6,0x63,0x44,0x25,0xbe,0x12,0x9c,0x35,0xc6+3,0x1f,0x0c,0xe3};

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
float phi(float x, float y, float z) {
  float ll = x * x + y * y + z * z;
  float l = sqrt(ll);
  float phi = atan2(y, x);
  return phi;
}

float theta(float x, float y, float z) {
  float ll = x * x + y * y + z * z;
  float l = sqrt(ll);
  float theta = acos(z / l);
  return theta;
}

void connect_callback(uint16_t conn_handle){
  // Get the reference to current connection
  BLEConnection* connection = Bluefruit.Connection(conn_handle);

  char central_name[32] = { 0 };
  connection->getPeerName(central_name, sizeof(central_name));

  Serial.print("Connected to "); // TODO take care of cases where Serial is not available
  Serial.println(central_name);
}

// State Class Definition
class State {
private:
    float x, y, z;    // Cartesian coordinates
    float theta, phi; // Spherical coordinates

    void cartesianToSpherical() {
        float l = sqrt(x * x + y * y + z * z);
        if (l == 0) l = 1;
        theta = acos(z / l) * 180 / PI;
        phi = atan2(y, x) * 180 / PI;
        if (phi < 0) phi += 360;
    }

    void sphericalToCartesian() {
        float r = 1;
        x = r * sin(theta * PI / 180.0) * cos(phi * PI / 180.0);
        y = r * sin(theta * PI / 180.0) * sin(phi * PI / 180.0);
        z = r * cos(theta * PI / 180.0);
    }

public:
    // Constructors
    State() : x(0), y(0), z(1) {
        cartesianToSpherical();
    }

    State(float x_init, float y_init, float z_init) : x(x_init), y(y_init), z(z_init) {
        cartesianToSpherical();
    }

    State(float theta_init, float phi_init) : theta(theta_init), phi(phi_init) {
        sphericalToCartesian();
    }

    // Setters and getters
    void setX(float new_x) {
        x = new_x;
        cartesianToSpherical();
    }

    void setY(float new_y) {
        y = new_y;
        cartesianToSpherical();
    }

    void setZ(float new_z) {
        z = new_z;
        cartesianToSpherical();
    }

    void setXYZ(float new_x, float new_y, float new_z) {
        x = new_x;
        y = new_y;
        z = new_z;
        cartesianToSpherical();
    }

    void setTheta(float new_theta) {
        theta = new_theta;
        sphericalToCartesian();
    }

    void setPhi(float new_phi) {
        phi = new_phi;
        sphericalToCartesian();
    }

    void setThetaPhi(float new_theta, float new_phi) {
        theta = new_theta;
        phi = new_phi;
        sphericalToCartesian();
    }

    float getX() const {
        return x;
    }

    float getY() const {
        return y;
    }

    float getZ() const {
        return z;
    }

    float getTheta() const {
        return theta;
    }

    float getPhi() const {
        return phi;
    }

    // Method to print the state
    void printState() {
        Serial.print("Cartesian: x = "); Serial.print(x);
        Serial.print(", y = "); Serial.print(y);
        Serial.print(", z = "); Serial.println(z);

        Serial.print("Spherical: theta = "); Serial.print(theta);
        Serial.print(", phi = "); Serial.println(phi);
    }
    
    // Apply X gate 
  void Xgate() {
      // Flip theta over the x-axis
      theta = fmod(180.0 - theta, 360.0);
      phi =  fmod(360-phi, 360.0);// Ensure theta stays within 0-360 degrees
      sphericalToCartesian();
  }

  // Apply Y gate 
  void Ygate() {
      theta = fmod(180.0 - theta, 360.0);
      phi = fmod(phi + 180.0, 360.0); // Ensure theta stays within 0-360 degrees
      sphericalToCartesian();
  }

  // Apply Z gate 
  void Zgate() {
      phi = fmod(phi + 180.0, 360.0); // Ensure theta stays within 0-360 degrees
      sphericalToCartesian();
  }

  //Apply arbitrary rotation around X axis 
  void RXgate(float angle) {
      angle = angle*PI/180.0;
      float y1 = y*cos(angle) - z*sin(angle);
      float z1 = y*sin(angle) + z*cos(angle);
      y = y1;
      z = z1;  
      cartesianToSpherical();
  }

  //Apply arbitrary rotation around Y axis 
  void RYgate(float angle) {
      angle = angle*PI/180.0;
      float x1 = x*cos(angle) + z*sin(angle);
      float z1 = z*cos(angle) - x*sin(angle);
      x = x1;
      z = z1;  
      cartesianToSpherical();
  }

  //Apply arbitrary rotation around Z axis 
  void RZgate(float angle) {
      angle = angle*PI/180.0;
      float x1 = x*cos(angle) - y*sin(angle);
      float y1 = x*sin(angle) + y*cos(angle);
      y = y1;
      x = x1;  
      cartesianToSpherical();
  }

  // Apply Hadamard gate 
  void Hgate() {
      RYgate(90);
      Xgate();
      sphericalToCartesian();
  }

  // // Apply RX gate animation 
  // void RXgateAni(float angle, Qbead& bead, int steps, float time_in_ms) {
  //     for (int i=1; i<=steps; i++){
  //         RXgate(angle/steps);
  //         bead.clear();
  //         //bead.getState().printState(); //for debugging
  //         bead.setBloch_deg_smooth(bead.state.getTheta(), bead.state.getPhi(), color(255, 0, 255));
  //         bead.show();
  //         delay(time_in_ms);
  //     }
  // }

  // // Apply RY gate animation 
  // void RYgateAni(float angle, Qbead& bead, int steps, float time_in_ms) {
  //     for (int i=1; i<=steps; i++){
  //         RYgate(angle/steps);
  //         bead.clear();
  //         //bead.getState().printState(); //for debugging
  //         bead.setBloch_deg_smooth(bead.state.getTheta(), bead.state.getPhi(), color(255, 0, 255));
  //         bead.show();
  //         delay(time_in_ms);
  //     }
  // }

  // // Apply RZ gate animation 
  // void RZgateAni(float angle, Qbead& bead, int steps, float time_in_ms) {
  //     for (int i=1; i<=steps; i++){
  //         RZgate(angle/steps);
  //         bead.clear();
  //         //bead.getState().printState(); //for debugging
  //         bead.setBloch_deg_smooth(bead.state.getTheta(), bead.state.getPhi(), color(255, 0, 255));
  //         bead.show();
  //         delay(time_in_ms);
  //     }
  // }
};

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
        blecharacc(QB_UUID_ACC_CHAR)
        {}

  static Qbead *singletoninstance; // we need a global singleton static instance because bluefruit callbacks do not support context variables -- thankfully this is fine because there is indeed only one Qbead in existence at any time

  LSM6DS3 imu;
  Adafruit_NeoPixel pixels;

  BLEService bleservice;
  BLECharacteristic blecharcol;
  BLECharacteristic blecharsph;
  BLECharacteristic blecharacc;
  uint8_t connection_count = 0;

  const uint8_t nsections;
  const uint8_t nlegs;
  const uint8_t theta_quant;
  const uint8_t phi_quant;
  const uint8_t ix, iy, iz;
  const bool sx, sy, sz;
  float rbuffer[3];
  float x, y, z, rx, ry, rz; // filtered and raw acc, in units of g
  float t_acc, p_acc;        // theta and phi according to gravity
  float T_imu;               // last update from the IMU

  float t_ble, p_ble;       // BLE theta and phi

  State state;

  uint32_t c_ble; // color

  static void ble_callback_color(uint16_t conn_hdl, BLECharacteristic* chr, uint8_t* data, uint16_t len) {
      singletoninstance->c_ble =  (data[2] << 16) | (data[1] << 8) | data[0];
  }

  static void ble_callback_theta_phi(uint16_t conn_hdl, BLECharacteristic* chr, uint8_t* data, uint16_t len){
      singletoninstance->t_ble = data[0]*180/255;
      singletoninstance->p_ble = data[1]*360/255;
  }

  void begin() {
    singletoninstance = this;
    Serial.begin(9600);
    while (!Serial); // TODO some form of warning or a way to give up if Serial never becomes available

    pixels.begin();
    clear();
    setBrightness(10);

    state.setXYZ(0, 0, 1);  // Ensure the state starts pointing along the z-axis

    Serial.println("qbead on XIAO BLE Sense + LSM6DS3 compiled on " __DATE__ " at " __TIME__);
    uint16_t imuResult = imu.begin();
    if (imuResult != 0) {
        Serial.print("IMU error: ");
        Serial.println(imuResult);
    } else {
        Serial.println("IMU OK");
    }

    Bluefruit.begin(QB_MAX_PRPH_CONNECTION, 0);
    Bluefruit.setName("qbead | " __DATE__ " " __TIME__);
    Bluefruit.Periph.setConnectCallback(connect_callback);
    bleservice.begin();
    blecharcol.setProperties(CHR_PROPS_READ | CHR_PROPS_WRITE);
    blecharcol.setPermission(SECMODE_OPEN, SECMODE_OPEN);
    blecharcol.setUserDescriptor("rgb color");
    blecharcol.setFixedLen(3);
    blecharcol.setWriteCallback(ble_callback_color);
    blecharcol.begin();
    blecharcol.write(zerobuffer20, 3);
    blecharsph.setProperties(CHR_PROPS_READ | CHR_PROPS_WRITE);
    blecharsph.setPermission(SECMODE_OPEN, SECMODE_OPEN);
    blecharsph.setUserDescriptor("spherical coordinates");
    blecharsph.setFixedLen(2);
    blecharsph.setWriteCallback(ble_callback_theta_phi);
    blecharsph.begin();
    blecharsph.write(zerobuffer20, 2);
    blecharacc.setProperties(CHR_PROPS_READ | CHR_PROPS_NOTIFY);
    blecharacc.setPermission(SECMODE_OPEN, SECMODE_OPEN);
    blecharacc.setUserDescriptor("xyz acceleration");
    blecharacc.setFixedLen(3 * sizeof(float));
    blecharacc.begin();
    blecharacc.write(zerobuffer20, 3 * sizeof(float));
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
    if (theta < 0 || theta > 180 || phi < 0 || phi > 360) {
      return;
    }
    float theta_section = state.getTheta() / theta_quant;
    if (theta_section < 0.5) {
      setLegPixelColor(0, 0, color);
    } else if (theta_section > nsections - 0.5) {
      setLegPixelColor(0, nsections, color);
    } else {
      float phi_leg = state.getPhi() / phi_quant;
      int theta_int = theta_section + 0.5;
      theta_int = theta_int > nsections - 1 ? nsections - 1 : theta_int; // to avoid precision issues near the end of the range
      int phi_int = phi_leg + 0.5;
      phi_int = phi_int % nlegs;
      setLegPixelColor(phi_int, theta_int, color);
    }
  }

  void setBloch_deg_smooth(float theta, float phi, uint32_t c) {
    if (theta < 0 || theta > 180 || phi < 0 || phi > 360) {
      return;
    }
    float theta_section = theta / theta_quant;
    float phi_leg = phi / phi_quant;
    int theta_int = theta_section + 0.5;
    theta_int = theta_int > nsections - 1 ? nsections - 1 : theta_int;  // to avoid precision issues near the end of the range
    int phi_int = phi_leg + 0.5;
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

  void readIMU() {
    rbuffer[0] = imu.readFloatAccelX();
    rbuffer[1] = imu.readFloatAccelY();
    rbuffer[2] = imu.readFloatAccelZ();
    rx = (1-2*sx)*rbuffer[ix];
    ry = (1-2*sy)*rbuffer[iy];
    rz = (1-2*sz)*rbuffer[iz];

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

    t_acc = theta(x, y, z)*180/3.14159;
    p_acc = phi(x, y, z)*180/3.14159;
    if (p_acc<0) {p_acc+=360;}// to bring it to [0,360] range

    Serial.print(x);
    Serial.print("\t");
    Serial.print(y);
    Serial.print("\t");
    Serial.print(z);
    Serial.print("\t-1\t1\t");
    Serial.print(t_acc);
    Serial.print("\t");
    Serial.print(p_acc);
    Serial.print("\t-360\t360\t");
    Serial.println();

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
  }

  void startBLEadv(void)
  {
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