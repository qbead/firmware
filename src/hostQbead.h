// prorotype header for a host qbead
// the host qbead will connect to a client qbead running
// tap_to_measure.io, and will light an LED in the orientation of the client qbead
#ifndef HOSTQBEAD_H
#define HOSTQBEAD_H


#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <LSM6DS3.h>
#include <math.h>

#include <bluefruit.h>


#include <blochVector.h>
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

// LSM6DS3 filter settings
#define LSM6DS3_ACC_GYRO_LPF2_XL_EN 0x80
#define LSM6DS3_ACC_GYRO_LPF2_XL_CUT_ODR_BY_50 0x00
#define LSM6DS3_ACC_GYRO_LPF2_XL_CUT_ODR_BY_100 0x20
#define LSM6DS3_ACC_GYRO_LPF2_XL_CUT_ODR_BY_9 0x40
#define LSM6DS3_ACC_GYRO_LPF2_XL_CUT_ODR_BY_400 0x60

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

static uint32_t scaleColor(float a, uint32_t c) {
  uint8_t r = min(0xff, a * redch(c));
  uint8_t g = min(0xff, a * greench(c));
  uint8_t b = min(0xff, a * bluech(c));
  return color(r, g, b);
}

static uint32_t scaleColorQuad(float a, uint32_t c) {
  float a2 = a * a;
  uint8_t r = min(0xff, a2 * redch(c));
  uint8_t g = min(0xff, a2 * greench(c));
  uint8_t b = min(0xff, a2 * bluech(c));
  return color(r, g, b);
}

static uint32_t scaleColor_8bit(uint8_t a, uint32_t c) {
  uint8_t r = min(0xff, (int)a * redch(c) / 255);
  uint8_t g = min(0xff, (int)a * greench(c) / 255);
  uint8_t b = min(0xff, (int)a * bluech(c) / 255);
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

// #### Parabolic wave
// Similarly to the triangular wave, this function is useful for periodically
// pulsating patterns. However, the profile of this function resembles a beating
// heart more closely and it can provide for more pleasing visuals.
// ![Depiction of the parabolic wave.](./parabola_wave.png)
uint8_t parabolaWave(uint8_t x) {
  uint8_t xm = x;
  if (xm>0x7f) {xm = 0xff-xm;}
  return (xm*xm)>>6;
}

uint32_t colorWheel_deg(float wheelPos) {
  return colorWheel(wheelPos * 255 / 360);
}

float sign(float x) {
  if (x > 0) return +1;
  else return -1;
}




namespace HostQbead {

class HostQbead {
public:
  HostQbead(const uint16_t pin00 = QB_LEDPIN,
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
        qBeadClientService(QB_UUID_SERVICE),
        qBeadTapChar(QB_UUID_TAP_CHAR)
        {}

  static HostQbead *singletoninstance; // we need a global singleton static instance because bluefruit callbacks do not support context variables -- thankfully this is fine because there is indeed only one Qbead in existence at any time

  LSM6DS3 imu;
  Adafruit_NeoPixel pixels;

  uint8_t connection_count = 0;

  BLEClientService qBeadClientService;
  BLEClientCharacteristic qBeadTapChar;

  const uint8_t nsections;
  const uint8_t nlegs;
  const uint8_t theta_quant;
  const uint8_t phi_quant;
  const uint8_t ix, iy, iz;
  const bool sx, sy, sz;
  float rbuffer[3];
  float whentapped_buffer[3];
  float x_whentapped, y_whentapped, z_whentapped; // set when wasTapped is called
  float x, y, z, rx, ry, rz; // filtered and raw acc, in units of g
  float t_acc, p_acc;        // theta and phi according to gravity
  float T_imu;               // last update from the IMU
  bool tapped = false;
  bool tappedrecorded = false;

  float t_ble, p_ble; // theta and phi as sent over BLE connection
  uint32_t c_ble = 0xffffff; // color as sent over BLE connection



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
    // we will get taps from another device!
    // setupIMUTapDetection();

    // BLE Peripheral service setup
    setupBLE();
    startBLEScan();
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

  void setBloch_deg(const BlochVector& state, uint32_t color) {
    setBloch_deg(state.theta, state.phi, color);
  }

  void setBloch_deg(float theta, float phi, uint32_t color) {
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


  /*
  // TODO needs brightness correction for when we have many LEDs on (when they are denser)
  // TODO make sure you are not in situations where no LEDs are lit because none are close (an edge case of brightness correction above)
  // TODO skip far-away pixels so the loop is not so expensive
  void setBloch_deg_smooth(const BlochVector& state, uint32_t color) {
    float width = 40;

    for (int phi_pixel = 0; phi_pixel <= nlegs; ++phi_pixel) {
      for (int theta_pixel = 0; theta_pixel <= nsections; ++theta_pixel) {
        float internal_angle = state.centralAngle(BlochVector(theta_quant * theta_pixel, phi_quant * phi_pixel));
        if (internal_angle > width) {
          continue;
        }

        float brightness = 1 - internal_angle * internal_angle / (width * width);

        addLegPixelColor(phi_pixel, theta_pixel, scaleColor(brightness, color));
      }
    }
  }

  void setBloch_deg_smooth(float theta, float phi, uint32_t color) {
    setBloch_deg_smooth(BlochVector(theta, phi), color);
  }
  */

  void setBloch_deg_smooth(const BlochVector& state, uint32_t color) {
    setBloch_deg_smooth(state.theta, state.phi, color);
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
    uint8_t gc = greench(c);
    uint8_t bc = bluech(c);

    setLegPixelColor(phi_int, theta_int, color(q * rc, q * gc, q * bc));
    setLegPixelColor(phi_int, theta_int + theta_direction, color(p * rc, p * gc, p * bc));
  }

  void testPixels() {
    Serial.println("[INFO] Testing all pixels discretely");
    for (int i = 0; i < pixels.numPixels(); i++) {
      pixels.setPixelColor(i, color(255, 255, 255));
      pixels.show();
      delay(5);
    }
    Serial.println("[INFO] Testing smooth transition between pixels");
    for (int phi = 0; phi < 360; phi += 30) {
      for (int theta = 0; theta < 180; theta += 6) {
        clear();
        setBloch_deg_smooth(theta, phi, colorWheel_deg(phi));
        show();
      }
    }
  }

  bool wasTapped(){
    if (!tappedrecorded) return false;
    // return true if IMU detected a tap since the last time wasTapped() was called
    bool wasTapped = tapped;
    // set tapped to false, so that the next time this function is called, it will return false
    tapped = false;
    tappedrecorded = false;
    // save tapped location
    if (wasTapped){
      x_whentapped = whentapped_buffer[ix];
      y_whentapped = whentapped_buffer[iy];
      z_whentapped = whentapped_buffer[iz];
    }
    return wasTapped;
  }

  static void tap_isr()
  {
    // This function is called when the IMU triggers an interrupt. That is: when a tap is detected!
    // We have to refer to the singletoninstance of the qbead here,
    // because the one and only qbead object does not exist when this isr is defined.
    // Then readout XYZ immediately
    // We could choose to only readout XYZ when we haven't yet processed the last tap,
    // but for now, let's just update the position everytime we tap.
    singletoninstance->tappedrecorded = false;
    singletoninstance->tapped = true;
  }
  // ******************** Begin new functions for BLE central functionality ****************** //
  void setupBLE(void){
    Bluefruit.begin(0, 1);
    Bluefruit.setName("Qbead Host");


    qBeadClientService.begin();
    qBeadTapChar.begin();
    qBeadTapChar.setNotifyCallback(ble_tap_callback);
    // Callbacks for Central
    // Note BvO: Not sure, but I think these are called when *we* connect to another service
    Bluefruit.Central.setConnectCallback(connect_callback);
    Bluefruit.Central.setDisconnectCallback(disconnect_callback);

  }

  void startBLEScan(void){

    // setup the scanner
    /* Start Central Scanning
     * - Enable auto scan if disconnected
     * - Interval = 100 ms, window = 80 ms
     * - Filter only accept bleuart service in advertising
     * - Don't use active scan (used to retrieve the optional scan response adv packet)
     * - Start(0) = will scan forever since no timeout is given
     */
    Bluefruit.Scanner.setRxCallback(scan_callback);
    Bluefruit.Scanner.restartOnDisconnect(true);
    Bluefruit.Scanner.setInterval(160, 80);       // in units of 0.625 ms
    Bluefruit.Scanner.filterUuid(QB_UUID_SERVICE); // this allows us to select a specific service
    Bluefruit.Scanner.useActiveScan(false);       // Don't request scan response data
    Bluefruit.Scanner.start(0);                   // 0 = Don't stop scanning after n seconds
  }

  static void scan_callback(ble_gap_evt_adv_report_t* report)
  {
    // Since we configure the scanner with filterUuid()
    // Scan callback only invoked for device with qbead service advertised
    // Connect to the device with qbead service in advertising packet
    Serial.println("Found Device!");
    // Stop scanning
    Bluefruit.Scanner.stop();

    Bluefruit.Central.connect(report);
  }

  static void connect_callback(uint16_t conn_handle) {
    Serial.println("Connected!");
    // should not be required since we filter
    if (!singletoninstance->qBeadClientService.discover(conn_handle)) {
      Serial.println("Service NOT found");
      Bluefruit.disconnect(conn_handle);
      return;
    }

    Serial.println("Service found");

    if (!singletoninstance->qBeadTapChar.discover()) {
      Serial.println("Characteristic NOT found");
      Bluefruit.disconnect(conn_handle);
      return;
    }

    Serial.println("Characteristic found");

    // Enable notifications if supported
    if (singletoninstance->qBeadTapChar.properties() & CHR_PROPS_NOTIFY) {
      singletoninstance->qBeadTapChar.enableNotify();
      Serial.println("Notifications enabled");
    }

    // Optional: read initial value
    uint8_t buf[20];
    int len = singletoninstance->qBeadTapChar.read(buf, sizeof(buf));
    if (len > 0) {
      Serial.print("Initial value: ");
      Serial.write(buf, len);
      Serial.println();
    }
  }

  static void ble_tap_callback(BLEClientCharacteristic* chr,
                     uint8_t* data, uint16_t len) {
    Serial.print("Notification: ");
    Serial.write(data, len);
    Serial.println();
    singletoninstance->tappedrecorded = true;
    singletoninstance->tapped = true;
    memcpy(singletoninstance->whentapped_buffer, data, 3*sizeof(float));
  }
  static void disconnect_callback(uint16_t conn_handle, uint8_t reason) {
    Serial.println("Disconnected");
    singletoninstance->startBLEScan();  // restart scanning
  }


}; // end class

HostQbead *HostQbead::singletoninstance = nullptr;

} // end namespace

#endif // HOSTQBEAD_H
