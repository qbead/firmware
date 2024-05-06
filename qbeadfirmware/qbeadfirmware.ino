#include "Adafruit_NeoPixel.h"  // from the Adafruit NeoPixel package
#include "LSM6DS3.h"            // from the SEEED LSM6DS package
#include "Wire.h"               // arduino stdlib
#include "math.h"               // stdlib

// default configs
#define QB_LEDPIN 10
#define QB_PIXELCONFIG NEO_GRB + NEO_KHZ800
#define QB_NSECTIONS 6
#define QB_NLEGS 12
#define QB_IMU_ADDR 0x6A

// TODO manage namespaces better
static uint32_t color(uint8_t r, uint8_t g, uint8_t b) {
  return ((uint32_t)r << 16) | ((uint32_t)g << 8) | b;
}

static uint8_t redch(uint32_t rgb) {
  return rgb>>16;
}

static uint8_t greench(uint32_t rgb) {
  return (0x00ff00 & rgb)>>8;
}

static uint8_t bluech(uint32_t rgb) {
  return 0x0000ff & rgb;
}

// TODO predefined color constants

// TODO bring in more of the helpers from SpinWearables https://spinwearables.com/codedoc/src/SpinWearables.h.html
uint32_t colorWheel(uint8_t wheelPos) {
  wheelPos = 255 - wheelPos;
  if(wheelPos < 85) {
    return color(255 - wheelPos * 3, 0, wheelPos * 3);
  }
  if(wheelPos < 170) {
    wheelPos -= 85;
    return color(0, wheelPos * 3, 255 - wheelPos * 3);
  }
  wheelPos -= 170;
  return color(wheelPos * 3, 255 - wheelPos * 3, 0);
}

uint32_t colorWheel_deg(float wheelPos) {
  return colorWheel(wheelPos*255/360);
}

float sign(float x) {
  if (x > 0) return +1;
  else return -1;
}

// z = cos(t)
// x = cos(p)sin(t)
// y = sin(p)sin(t)
float phi(float x, float y, float z) {
  float ll = x*x+y*y+z*z;
  float l = sqrt(l);
  float phi = atan2(y,x);
  float theta = acos(z/l);
  return phi;
}
float theta(float x, float y, float z) {
  float ll = x*x+y*y+z*z;
  float l = sqrt(ll);
  float phi = atan2(y,x);
  float theta = acos(z/l);
  return theta;
}

namespace Qbead {

class Qbead {
public:

  Qbead(const uint16_t pin00 = QB_LEDPIN,
        const uint16_t pixelconfig = QB_PIXELCONFIG,
        const uint16_t nsections = QB_NSECTIONS,
        const uint16_t nlegs = QB_NLEGS,
        const uint8_t imu_addr = QB_IMU_ADDR) :
      imu(LSM6DS3(I2C_MODE,imu_addr)),
      pixels(Adafruit_NeoPixel(nlegs * (nsections - 1) + 2, pin00, pixelconfig)),
      nsections(nsections),
      nlegs(nlegs),
      theta_quant(180 / nsections),
      phi_quant(360 / nlegs) { 
  };

  LSM6DS3 imu;
  Adafruit_NeoPixel pixels;
  const int nsections;
  const int nlegs;
  const int theta_quant;
  const int phi_quant;
  float x,y,z,rx,ry,rz; // filtered and raw acc, in units of g
  float t,p; // theta and phi according to gravity
  float t_imu; // last update from the IMU

  void begin() {
    Serial.begin(9600);
    while (!Serial);
    
    pixels.begin();
    clear();
    setBrightness(10);
    
    Serial.println("XIAO BLE Sense test of LSM6DS3 compiled on " __DATE__ " at " __TIME__);
    Serial.println("Seeed mbed-enabled nrf52 Boards-> Seeed XIAO BLE Sense - nrf52480");
    if (!imu.begin()) {
        Serial.println("IMU error");
    } else {
        Serial.println("IMU OK");
    }
    
  }

  void clear() {
    pixels.clear();
  }

  void show() {
    pixels.show();
  }

  void setLegPixelColor(int leg, int pixel, uint32_t color) {
    leg = nlegs-leg; // invert direction for the phi angle, because the PCB is set up as a left-handed coordinate system // TODO make this easier to configure
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
    float theta_section = theta / theta_quant;
    if (theta_section < 0.5) {
      setLegPixelColor(0, 0, color);
    } else if (theta_section > nsections - 0.5) {
      setLegPixelColor(0, nsections, color);
    } else {
      float phi_leg = phi / phi_quant;
      int theta_int = theta_section + 0.5;
      theta_int = theta_int > nsections - 1 ? nsections - 1 : theta_int;  // to avoid precission issues near the end of the range
      int phi_int = phi_leg + 0.5;
      phi_int = phi_int > nlegs - 1 ? 0 : phi_int;
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
    theta_int = theta_int > nsections - 1 ? nsections - 1 : theta_int;  // to avoid precission issues near the end of the range
    int phi_int = phi_leg + 0.5;
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

  void readIMU() {
    /* // The XYZ axes of the accelerometer do not correspond to the XYZ axes of the sphere // TODO have a principled way to correct for these and to configure different ones
    rx = imu.readFloatAccelX();
    ry = imu.readFloatAccelY();
    rz = imu.readFloatAccelZ();
    */
    rx = -imu.readFloatAccelX();
    ry =  imu.readFloatAccelZ();
    rz =  imu.readFloatAccelY();
    float t_new = micros();
    float delta = t_new - t_imu;
    t_imu = t_new;
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

    t = theta(x, y, z)*180/3.14159;
    p = phi(x, y, z)*180/3.14159;
    if (p<0) {p+=360;}// to bring it to [0,360] range

    Serial.print(x);
    Serial.print("\t");
    Serial.print(y);
    Serial.print("\t");
    Serial.print(z);
    Serial.print("\t-1\t1\t");

    Serial.print(t);
    Serial.print("\t");
    Serial.print(p);
    Serial.print("\t-360\t360\t");
    
    Serial.println();
  }
}; // end class
} // end namespace


Qbead::Qbead bead; 

// You can use bead.strip to access the LED strip object
// You can use bead.imu to access the IMU object

void setup() {
  bead.begin();
  bead.setBrightness(255); // way too bright
  for (int phi = 0; phi < 360; phi += 30) {
    for (int theta = 0; theta < 180; theta+=3) {
      bead.clear();
      bead.setBloch_deg(theta, phi, colorWheel(phi));
      bead.show();
    }
  }
}

void loop() {
  bead.readIMU();

  bead.clear();
  bead.setBloch_deg_smooth(bead.t, bead.p, color(255,255,255));
  bead.show();
  delay(10);
}
