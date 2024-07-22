#include "Adafruit_NeoPixel.h"  // from the Adafruit NeoPixel package
#include "LSM6DS3.h"            // from the SEEED LSM6DS package
#include "Wire.h"               // arduino stdlib
#include "math.h"               // stdlib

#include <bluefruit.h>          // from board setup

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

bool shaking = false;
bool vectorinit = false;
bool rotation_z = false;
bool rotation_x = false;
bool rotation_y = false;
unsigned long startshakingTime = 0;
unsigned long startinitTime = 0;
unsigned long startrotationTime = 0;
const unsigned long shakeDuration = 500000; 
const unsigned long initDuration = 5000000; 
const unsigned long rotationDuration = 5000000; 

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
        const uint8_t  imu_addr = QB_IMU_ADDR,
        const uint8_t  ix = QB_IX,
        const uint8_t  iy = QB_IY,
        const uint8_t  iz = QB_IZ,
        const bool  sx = QB_SX,
        const bool  sy = QB_SY,
        const bool  sz = QB_SZ
        ) :
      imu(LSM6DS3(I2C_MODE,imu_addr)),
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
      { 
  };

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
  float x,y,z,rx,ry,rz; // filtered and raw acc, in units of g
  float t,p; // theta and phi according to gravity
  float t_imu; // last update from the IMU

  void begin() {
    Serial.begin(9600);
    while (!Serial);
    
    pixels.begin();
    clear();
    setBrightness(10);
    
    Serial.println("qbead on XIAO BLE Sense + LSM6DS3 compiled on " __DATE__ " at " __TIME__);
    if (!imu.begin()) {
        Serial.println("IMU error");
    } else {
        Serial.println("IMU OK");
    }

   Bluefruit.begin(QB_MAX_PRPH_CONNECTION, 0);
   Bluefruit.setName("qbead | " __DATE__ " " __TIME__);
   bleservice.begin();
   blecharcol.setProperties(CHR_PROPS_READ | CHR_PROPS_WRITE);
   blecharcol.setPermission(SECMODE_OPEN, SECMODE_OPEN);
   blecharcol.setUserDescriptor("rgb color");
   blecharcol.setFixedLen(3);
   blecharcol.begin();
   blecharcol.write(zerobuffer20, 3);
   blecharsph.setProperties(CHR_PROPS_READ | CHR_PROPS_WRITE);
   blecharsph.setPermission(SECMODE_OPEN, SECMODE_OPEN);
   blecharsph.setUserDescriptor("spherical coordinates");
   blecharsph.setFixedLen(2);
   blecharsph.begin();
   blecharsph.write(zerobuffer20, 2);
   blecharacc.setProperties(CHR_PROPS_READ | CHR_PROPS_NOTIFY);
   blecharacc.setPermission(SECMODE_OPEN, SECMODE_OPEN);
   blecharacc.setUserDescriptor("xyz acceleration");
   blecharacc.setFixedLen(3*sizeof(float));
   blecharacc.begin();
   blecharacc.write(zerobuffer20, 3*sizeof(float));
   startBLEadv();
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

    float p_theta = (theta_section - theta_int);
    int theta_direction = sign(p_theta);
    float p_phi = (phi_leg - phi_int);
    int phi_direction = sign(p_phi);
    p_theta = abs(p_theta);
    p_phi = abs(p_phi);
    float q_theta = 1 - p_theta;
    float q_phi = 1 - p_phi;
    p_phi = p_phi * p_phi;
    p_theta = p_theta * p_theta;
    q_phi = q_phi * q_phi;
    q_theta = q_theta * q_theta;

    uint8_t rc = redch(c);
    uint8_t bc = bluech(c);
    uint8_t gc = greench(c);

    setLegPixelColor(phi_int, theta_int, color(q_phi*q_theta * rc, q_phi*q_theta * bc, q_phi*q_theta * gc));
    setLegPixelColor(phi_int, theta_int + theta_direction, color(p_theta*q_phi * rc, p_theta*q_phi * bc, p_theta*q_phi * gc));
    setLegPixelColor(phi_int+phi_direction, theta_int, color(q_theta*p_phi * rc, q_theta*p_phi * bc, q_theta*p_phi * gc));
    setLegPixelColor(phi_int+phi_direction, theta_int + theta_direction, color(p_theta*p_phi * rc, p_theta*p_phi * bc, p_theta*p_phi * gc));
  }
  
  void lightEverythingUp(uint32_t c){
  for (int i = 0; i < pixels.numPixels(); i++) {
    pixels.setPixelColor(i, c);
    show();
  }
  delay(2000);
  clear();
  show();
  }
  
  void vectorInit(float theta, float phi){
    setBloch_deg_smooth(theta, phi, color(255, 0, 255));
    if ((x * x + y * y + z * z) > 1.07) {
      if (!shaking) {
        shaking = true;
        startshakingTime = micros();
      } else {
        if (micros() - startshakingTime >= shakeDuration) {
          uint32_t c = color(0, 255, 0);
          lightEverythingUp(c);
          startinitTime = micros();
          while (micros()-startinitTime<=initDuration)
          {
            clear();
            readIMU();
            setBloch_deg_smooth(t, p, color(0, 0, 255));
            show();
            delay(10);
            theta = t;
            phi = p;
          }
          delay(5000);
          clear();
          setBloch_deg_smooth(theta, phi, color(0, 255, 0)); //fix the necessary vector
          setLegPixelColor(1, 3, color(255, 255, 255));
          setLegPixelColor(4, 3, color(255, 255, 255));
          setLegPixelColor(7, 3, color(255, 255, 255));
          setLegPixelColor(10, 3, color(255, 255, 255));
          setLegPixelColor(1, 0, color(255, 255, 255));
          setLegPixelColor(1, 6, color(255, 255, 255));
          
          show();
          delay(1000);
          shaking = false;
          startrotationTime = micros();
          while (micros()-startrotationTime<=rotationDuration)
          {
            Serial.print("\nROTATION PERFORMANCE\n");
            readIMU();
            if (0.9<x && x<1.1) 
            {
              rotation_z = true;
            }
            else
            {
              rotation_z = false;
            }
            
          }
          if(rotation_z)
          {
            clear();
            for (int i=0; i<nsections; i++)
            {  
              Serial.print("Rotation over Z is completed");
              setLegPixelColor(4, i, color(0, 255, 0));
              setLegPixelColor(10, i, color(0, 255, 0));
              Serial.print("Rotation over Z is completed\n");
              delay(300);
              show();
            }
            delay(5000);
            clear();
            setBloch_deg(theta+180, phi, color(0, 0, 255));
            show();
            delay(20000);
          }
        }
      }
    } else {
    shaking = false;
    }

    delay(10);
  }
  
  void readIMU() {
    rbuffer[0] = imu.readFloatAccelX();
    rbuffer[1] = imu.readFloatAccelY();
    rbuffer[2] = imu.readFloatAccelZ();
    rx = (1-2*sx)*rbuffer[ix];
    ry = (1-2*sy)*rbuffer[iy];
    rz = (1-2*sz)*rbuffer[iz];
    
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
    Serial.print("\t");
    Serial.print("SQUARE VALUE:");
    Serial.print(x*x+y*y+z*z);
    Serial.print("\t");
    Serial.print("\t-1\t1\t");
    Serial.print(t);
    Serial.print("\t");
    Serial.print(p);
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
//    
//    /* Start Advertising
//    * - Enable auto advertising if disconnected
//    * - Interval:  fast mode = 20 ms, slow mode = 152.5 ms
//    * - Timeout for fast mode is 30 seconds
//    * - Start(timeout) with timeout = 0 will advertise forever (until connected)
//    * 
//    * For recommended advertising interval
//    * https://developer.apple.com/library/content/qa/qa1931/_index.html   
//    */
   Bluefruit.Advertising.restartOnDisconnect(true);
   Bluefruit.Advertising.setInterval(32, 244);    // in unit of 0.625 ms
   Bluefruit.Advertising.setFastTimeout(30);      // number of seconds in fast mode
   Bluefruit.Advertising.start(0);                // 0 = Don't stop advertising after n seconds
 }

}; // end class
} // end namespace

Qbead::Qbead bead;

// You can use bead.strip to access the LED strip object
// You can use bead.imu to access the IMU object

void setup() {
  Serial.println("start");
  bead.begin();
  Serial.println("1");
  bead.setBrightness(25); // way too bright
  for (int i = 0; i < bead.pixels.numPixels(); i++) {
    bead.pixels.setPixelColor(i, color(255,255,255));
    bead.pixels.show();
    delay(5);
  }
  Serial.println("2");
  for (int phi = 0; phi < 360; phi += 30) {
    for (int theta = 0; theta < 180; theta+=3) {
      bead.clear();
      bead.setBloch_deg(theta, phi, colorWheel(phi));
      bead.show();
    }
  }
  Serial.println("3");
}

void loop() {
  bead.readIMU();

  bead.clear();
//  bead.setBloch_deg_smooth(bead.t, bead.p, color(255,0,255));
  bead.vectorInit(bead.t, bead.p);
  bead.show();
  delay(10);
}