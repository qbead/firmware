#include<Qbead.h>

Qbead::Qbead bead;

int entg;


void setup() {
  Serial.begin(9600);
  while (!Serial); // TODO some form of warning or a way to give up if Serial never becomes available

  Serial.println("start");
  Serial.println("Do you want to entangle? Write 0 to not entangle, 1 to entangle");
  delay(5000);
  while(!Serial.println()){}
  entg= Serial.parseInt();
  bead.begin(entg);
  bead.setBrightness(25); // way too bright
  for (int i = 0; i < bead.pixels.numPixels(); i++) {
    bead.pixels.setPixelColor(i, color(255,255,255));
    bead.pixels.show();
    delay(5);
  }
  Serial.println("testing smooth transition between pixels");
  for (int phi = 0; phi < 360; phi += 30) {
    for (int theta = 0; theta < 180; theta+=3) {
      bead.clear();
      bead.setBloch_deg(theta, phi, colorWheel(phi));
      bead.show();
    }
  }

}

void loop() {
  
  if(entg==0){
    bead.readIMU();
    bead.state.setXYZ(bead.x, bead.y, bead.z);
    bead.clear();
    bead.setBloch_deg_smooth(bead.state.getTheta(), bead.state.getPhi(), color(255, 0, 255));
    bead.show();
    delay(5000);
  }

  else if(entg==1){
    // Serial.println(bead.t);
    //bead.readIMU();

    bead.clear();
    //bead.setBloch_deg_smooth(bead.t, bead.p, bead.c);
    bead.show();
    delay(10);
  }

}
