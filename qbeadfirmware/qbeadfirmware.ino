#include <Adafruit_NeoPixel.h>

// Which pin on the Arduino is connected to the first (the longest) leg
#define PIXELCONFIG NEO_GRB + NEO_KHZ800
#define SERIALLEGS 1

static uint32_t Color(uint8_t r, uint8_t g, uint8_t b) {
  return ((uint32_t)r << 16) | ((uint32_t)g << 8) | b;
}

float sign(float x) {
  if (x>0) return +1; else return -1;
}

struct QBead {
  static const int nsections = 6;
  static const int nlegs = 6; // 12 in final version
  static const int theta_quant = 180 / nsections;
  static const int phi_quant = 360 / nlegs;

#if SERIALLEGS == 1
  Adafruit_NeoPixel pixels;
  QBead(int pin00, int pixelconfig) {
    pixels = Adafruit_NeoPixel(nlegs*nsections + 2, pin00, pixelconfig);
    pixels.setBrightness(120);
  }
  void setLegPixelColor(int leg, int pixel, uint32_t color) {
    leg = leg % nlegs;
    if (leg==0) {
      pixels.setPixelColor(pixel, color);
    } else if (pixel==0) {
      pixels.setPixelColor(0, color);
    } else if (pixel==6) {
      pixels.setPixelColor(6, color);
    } else {
      pixels.setPixelColor(7+(leg-1)*(nsections-1)+pixel-1, color);
    }
  }
  void begin() {
    pixels.begin();
  }

  void clear() {
    pixels.clear();
  }

  void show() {
    pixels.show();
  }
#else
  Adafruit_NeoPixel* legs[nlegs];
  int legpermute[12] = {7,8,12,9,10,11,5,13,4,6,3,2};
  QBead(int pixelconfig) {
    legs[0] = new Adafruit_NeoPixel(nsections + 1, legpermute[0], pixelconfig);
    for (int i = 1; i < 12; i++) {
      legs[i] = new Adafruit_NeoPixel(nsections - 1, legpermute[i], pixelconfig);
    }
  }
  void setLegPixelColor(int leg, int pixel, uint32_t color) {
    leg = leg%nlegs;
    if (pixel==0) {leg = 0;}
    if (pixel==nsections-1) {leg = 0;}
    if (leg >= legs || leg < 0 || (leg==0 && pixel>=7) || (leg>0 && pixel>=5) || pixel<0) {return;}
    legs[leg]->setPixelColor(pixel, color);
  }
  
  void begin() {
    for (int i = 0; i < 12; i++) {
      legs[i]->begin();
    }
  }

  void clear() {
    for (int i = 0; i < 12; i++) {
      legs[i]->clear();
    }
  }

  void show() {
    for (int i = 0; i < 12; i++) {
      legs[i]->show();
    }
  }
#endif

  void setBloch_deg(float theta, float phi, uint32_t color) {
    if (theta < 0 || theta > 180 || phi < 0 || phi > 360) {
      return;
    }
    float theta_section = theta / theta_quant;
    if (theta_section < 0.5) {
      setLegPixelColor(0, 0, color);
    }
    else if (theta_section > nsections - 0.5) {
      setLegPixelColor(0, nsections, color);
    }
    else {
      float phi_leg = phi / phi_quant;
      int theta_int = theta_section + 0.5;
      theta_int = theta_int > nsections - 1 ? nsections - 1 : theta_int; // to avoid precission issues near the end of the range
      int phi_int = phi_leg + 0.5;
      phi_int = phi_int > nlegs - 1 ? 0 : phi_int;
      setLegPixelColor(phi_int, theta_int, color);
    }
  }

  void setBloch_deg_smoothwhite(float theta, float phi) {
    uint8_t b = 255;
    if (theta < 0 || theta > 180 || phi < 0 || phi > 360) {
      return;
    }
    float theta_section = theta / theta_quant;
    float phi_leg = phi / phi_quant;
    int theta_int = theta_section + 0.5;
    theta_int = theta_int > nsections - 1 ? nsections - 1 : theta_int; // to avoid precission issues near the end of the range
    int phi_int = phi_leg + 0.5;
    phi_int = phi_int > nlegs - 1 ? 0 : phi_int;
    
    float p = (theta_section-theta_int);
    int theta_direction = sign(p);
    p = abs(p);
    float q = 1-p;
    p = p*p;
    q = q*q;

    setLegPixelColor(phi_int, theta_int, Color(q*b,q*b,q*b));
    setLegPixelColor(phi_int, theta_int+theta_direction, Color(p*b,p*b,p*b));
  }

};

#if SERIALLEGS == 1
  QBead bead(2, PIXELCONFIG);
#else
  QBead bead(PIXELCONFIG);
#endif

void setup() {
  bead.begin();
}

void loop() {
  
  for (int phi = 20; phi < 360; phi+=10) {
    for (int theta = 0; theta < 180; theta++) {
      //Serial.print(theta); Serial.print(" "); Serial.print(phi); Serial.print(" "); 
      bead.clear();
      //bead.setBloch_deg(theta, phi, Color(255, 255, 255));
      bead.setBloch_deg_smoothwhite(theta, phi);
      bead.show();
      delay(25);
    }
  }
  //Serial.println("loop");

}
