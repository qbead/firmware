#include <Arduino.h>
#include <bluefruit.h>

#define BAT_HIGH_CHARGE 22  // HIGH for 50mA, LOW for 100mA
#define BAT_CHARGE_STATE 23 // LOW for charging, HIGH not charging

class Xiao {
public:
  Xiao();
  float GetBatteryVoltage();
  bool IsChargingBattery();
  float GetBatteryLevel();
private:
  // define min and max voltage for battery level determination
  float maxVoltage = 4.11;
  float minVoltage = 3.06;
};

Xiao::Xiao() {
  pinMode(VBAT_ENABLE, OUTPUT);
  pinMode(BAT_CHARGE_STATE, INPUT);

  digitalWrite(BAT_HIGH_CHARGE, LOW); // charge with 100ma if LOW, else 50mA
}

#define VBAT_MV_PER_LBS (0.003395996F)

float Xiao::GetBatteryVoltage() {
  digitalWrite(VBAT_ENABLE, LOW);
  // might want to wait very briefly for the voltage to settle
  uint32_t adcCount = analogRead(PIN_VBAT);
  float adcVoltage = adcCount * VBAT_MV_PER_LBS;
  float vBat = adcVoltage * (1510.0 / 510.0);

  digitalWrite(VBAT_ENABLE, HIGH);

  return vBat;
}

float Xiao::GetBatteryLevel() {
  float batPercentage = (GetBatteryVoltage()-minVoltage)/(maxVoltage - minVoltage) * 100;
  return batPercentage;
}

bool Xiao::IsChargingBattery() { return digitalRead(BAT_CHARGE_STATE) == LOW; }