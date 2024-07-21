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
#define QB_SX 1
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

const uint8_t zerobuffer20[] = {0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0};

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

  void setBloch_deg_int_fixedcolor(float theta, float phi) {
    if (theta < 0 || theta > 180 || phi < 0 || phi > 360) {
      return;
    }
    float theta_section = theta / theta_quant;
    if (theta_section < 0.5) {
      setLegPixelColor(0, 0, color(0,0,255));
    } else if (theta_section > nsections - 0.5) {
      setLegPixelColor(0, nsections, color(0,0,255));
    } else {
      float phi_leg = phi / phi_quant;
      int theta_int = theta_section + 0.5;
      theta_int = theta_int > nsections - 1 ? nsections - 1 : theta_int;  // to avoid precission issues near the end of the range
      int phi_int = phi_leg + 0.5;
      phi_int = phi_int > nlegs - 1 ? 0 : phi_int;
      setLegPixelColor(phi_int, theta_int, color(0,0,255));
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

  void setBloch_deg_smooth_blue(float theta, float phi) {
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

    uint8_t rc = 0;
    uint8_t bc = 255;
    uint8_t gc = 0;

    setLegPixelColor(phi_int, theta_int, color(q * rc, q * bc, q * gc));
    setLegPixelColor(phi_int, theta_int + theta_direction, color(p * rc, p * bc, p * gc));
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

    /*Serial.print(x);
    Serial.print("\t");
    Serial.print(y);
    Serial.print("\t");
    Serial.print(z);
    Serial.print("\t-1\t1\t");
    Serial.print(t);
    Serial.print("\t");
    Serial.print(p);
    Serial.print("\t-360\t360\t");
    Serial.println();*/

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
} // end namespace

Qbead::Qbead bead;

// You can use bead.strip to access the LED strip object
// You can use bead.imu to access the IMU object



//////////////////////////////////////////////////
//////////////////////////////////////////////////
//////////////////////////////////////////////////

//////// GAME PARAMETERS ////////
float tolerance_distance = 0.3; // sin(30*pi/180)=0.26
//int tolerance_theta = 180/6; // in degrees
//int tolerance_phi = 360/12; // in degrees
int game_duration = 10; // in seconds

//////// GAME VARIABLES ////////
// Score-related
int score = 0;
int highscore = 0;
float theta_target = 0;
float phi_target = 0;

// Start and stop game
bool in_game = false;
int game_timer = 0;
bool generate_new_target = false;
int incomingByte = 0;
//int serial_value = 0;
float distance_target_bead;

// Motion triggers to start game
bool start_trigger1 = false;
bool start_trigger2 = false;
float time_trigger = 0;
int tolerance_start = 0.5;
int delay_start = 500; // in ms

//////// BLINK VARIABLES ////////
int counter_blink = 0;

//////////////////////////////////////////////////
//////////////////////////////////////////////////
//////////////////////////////////////////////////

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

  // BLINK
  pinMode(LED_BUILTIN, OUTPUT);
}

void start_game_animation(){
  for (int i = 0; i < bead.pixels.numPixels(); i++) {
    bead.pixels.setPixelColor(i, color(255,255,255));
    bead.pixels.show();
    delay(5);
  }
  /*for (int phi = 0; phi < 360; phi += 30) {
    for (int theta = 0; theta < 180; theta+=3) {
      bead.clear();
      bead.setBloch_deg(theta, phi, colorWheel(phi));
      bead.show();
    }
  }*/
}

void end_game_animation(bool new_highscore){
  bead.clear();
  for (int j = 0; j < 3; j++) {
    for (int i = 0; i < bead.pixels.numPixels(); i++) {
      // If new_highscore, animation is pink. Otherwise, yellow
      if (new_highscore){
        bead.pixels.setPixelColor(i, color(255,255,0));
      }
      else {
        bead.pixels.setPixelColor(i, color(255,0,255));
      }
    }
    bead.pixels.show();
    delay(500);
    bead.clear();
    bead.pixels.show();
    delay(500);
  }
}

void show_highscore(){
  bead.clear();
  bead.setLegPixelColor(0, 6, color(255,255,255));

  int counter_score = 1;
  for (int leg = 0; leg < 12; leg++){
    for (int pixel = 5; pixel > 0; pixel--){
      if (counter_score > highscore){
        break;
      }
      bead.setLegPixelColor(leg+1, pixel, color(255,255,0));
      counter_score++;
    }
    if (counter_score > highscore){
      break;
    }
  }

  bead.show();
  delay(10);
}

void loop() {
  bead.readIMU();


//////////////////////////////////////////////////
//////////////////////////////////////////////////
//////////////////////////////////////////////////

  // GAME

  // NOT IN GAME: show highscore and check if game must be started
  if (!in_game){
    show_highscore();

    // Check if starting by motion
    if (bead.imu.readFloatAccelY() < -1+tolerance_start && (!start_trigger2)){
      start_trigger1 = true;
      time_trigger = millis();
    }
    if (bead.imu.readFloatAccelY() > 0 && (start_trigger1) && (!start_trigger2)){
      start_trigger2 = true;
      time_trigger = millis();
    }
    if (bead.imu.readFloatAccelY() < -1+tolerance_start && (start_trigger1) && (start_trigger2)){
      start_trigger1 = false;
      start_trigger2 = false;
      in_game = true;
    }
    if (millis() - time_trigger > delay_start){
      start_trigger1 = false;
      start_trigger2 = false;
    }
    //Serial.print(start_trigger1);
    //Serial.print(start_trigger2);
    //Serial.println();

    // Check if starting by serial input
    if (Serial.available() > 0) {
      incomingByte = Serial.read();
      if (incomingByte) {
        in_game = true;
        //highscore = incomingByte-48;
      }
    }

    // If game will be started, play animation and initialize game variables
    if (in_game){
      Serial.println("--- GAME STARTING ---");
      delay(100);
      randomSeed(millis());
      start_game_animation();
      score = 0;
      game_timer = millis();
    }
  }




  // IN-GAME EVENTS
  if (in_game) {
    // Generate new target if needed
    if (generate_new_target){
      // Uniformly random spherical coordinates
      theta_target = acos(2*float(random(0,101))/100-1) * 180/PI;
      phi_target = 2*PI*float(random(0,101))/100 * 180/PI;
      //theta_target = random(0,180);
      //phi_target = random(0,360);
      generate_new_target = false;
    }

    // Light up player
    bead.clear();
    bead.setBloch_deg_smooth_blue(bead.t, bead.p);
    delay(10);

    // Light up target
    bead.setBloch_deg_int_fixedcolor(theta_target, phi_target);
    bead.show();
    delay(10);

    // If target reached, request a new target
    // Distance calculation: https://en.wikipedia.org/wiki/Spherical_coordinate_system#Distance_in_spherical_coordinates
    distance_target_bead = sqrt(1 + 1 - 2*( sin(theta_target*PI/180)*sin(bead.t*PI/180)*cos(phi_target*PI/180-bead.p*PI/180) + cos(theta_target*PI/180)*cos(bead.t*PI/180) ) );

    //if (abs(bead.t - theta_target) < tolerance_theta && abs(bead.p - phi_target) < tolerance_phi) {
    if (distance_target_bead < tolerance_distance) {
      score += 1;
      generate_new_target = true;
      // Show score
      Serial.print("Score: ");
      Serial.println(score);
    }

    // Game over
    if ((millis() - game_timer) > game_duration*1000) {
      in_game = false;
      bool new_highscore = false;
      Serial.print("Highscore");
      if (score > highscore){
        highscore = score;
        new_highscore = true;
        Serial.print(" (new!): ");
      }
      else {
        Serial.print(": ");
      }
      Serial.println(highscore);
      end_game_animation(new_highscore);
    }
  }

//////////////////////////////////////////////////
//////////////////////////////////////////////////
//////////////////////////////////////////////////

  // BLINK
  counter_blink += 1;
  if (counter_blink == 10) {
    digitalWrite(LED_BUILTIN, HIGH);  // turn the LED on (HIGH is the voltage level)
  }
  if (counter_blink == 20) {
    digitalWrite(LED_BUILTIN, LOW);   // turn the LED off by making the voltage LOW
    counter_blink = 0;
  }
}
