#include <Qbead.h>

Qbead::Qbead bead;

// Variables to control entanglement and device role
int entg;
int cp;

// BLE Declarations
BLEClientService Service_1 = BLEClientService("e30c1fc6-359c-12be-2544-63d6aa088d45");
BLEClientCharacteristic bleentg_1 = BLEClientCharacteristic("e30c1fc7-359c-12be-2544-63d6aa088d45");
BLEService Service_2 = BLEService("e30c1fc6-359c-12be-2544-63d6aa088d45");
BLECharacteristic bleentg_2 = BLECharacteristic("e30c1fc7-359c-12be-2544-63d6aa088d45");

// Function prototypes
void communicate_1(BLEClientCharacteristic* chr, uint8_t* data, uint16_t len);
void scan_callback(ble_gap_evt_adv_report_t* report);

void setup() {
  // Initialize Qbead
  bead.begin();
  bead.setBrightness(25);

  // Test pixels
  Serial.println("Testing all pixels discretely");
  for (int i = 0; i < bead.pixels.numPixels(); i++) {
    bead.pixels.setPixelColor(i, color(255, 255, 255));
    bead.pixels.show();
    delay(5);
  }

  // Test smooth transition
  Serial.println("Testing smooth transition between pixels");
  for (int phi = 0; phi < 360; phi += 30) {
    for (int theta = 0; theta < 180; theta += 3) {
      bead.clear();
      bead.setBloch_deg(theta, phi, colorWheel(phi));
      bead.show();
    }
  }

  // Entanglement setup
  Serial.println("What do you want to do? Write 1 to entangle, any other number to not entangle.");
  while (!Serial.available()) {}
  entg = Serial.parseInt();

  if (entg == 1) {
    Serial.println("Is this the central device or the peripheral device? Write 1 for the central device, 0 for the peripheral.");
    delay(5000);
    while (!Serial.available()) {}
    cp = Serial.parseInt();

    if (cp == 1) {
      // Central device setup
      Bluefruit.begin(0, 1);
      Serial.println("Bluetooth® Low Energy Central - LED control");

      Service_1.begin();
      bleentg_1.begin();
      bleentg_1.setNotifyCallback(communicate_1);

      // Start scanning for BLE peripherals
      Bluefruit.Scanner.setRxCallback(scan_callback);
      Bluefruit.Scanner.restartOnDisconnect(true);
      Bluefruit.Scanner.setInterval(160, 80);
      Bluefruit.Scanner.useActiveScan(true);
      Bluefruit.Scanner.start(0);

      Serial.println("Central device scanning for peripherals...");
    } else if (cp == 0) {
      // Peripheral device setup
      randomSeed(analogRead(0));  // Seed the random number generator

      Bluefruit.begin();
      Serial.println("Bluetooth® Low Energy Peripheral - LED control");

      Bluefruit.setName("QBEAD");

      Service_2.begin();

      // Configure the notification characteristic
      bleentg_2.setProperties(CHR_PROPS_NOTIFY);
      bleentg_2.setPermission(SECMODE_OPEN, SECMODE_OPEN);
      bleentg_2.begin();

      // Start advertising
      Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
      Bluefruit.Advertising.addTxPower();
      Bluefruit.Advertising.addService(Service_2);
      Bluefruit.Advertising.addName();
      Bluefruit.Advertising.restartOnDisconnect(true);
      Bluefruit.Advertising.setInterval(160, 244);  // Fast connection
      Bluefruit.Advertising.setFastTimeout(30);     // Fast mode timeout in seconds
      Bluefruit.Advertising.start(0);               // 0 = Don't stop advertising

      Serial.println("QBEAD Peripheral is advertising...");
    }
  }
}

void loop() {
  if (entg != 1) {
    // Normal operation
    bead.readIMU();
    bead.clear();
    bead.setBloch_deg_smooth(bead.state.getTheta(), bead.state.getPhi(), bead.c_ble);
    bead.show();
  } else if (entg == 1 && cp == 0) {
    // Peripheral operation when entangled
    if (Bluefruit.connected()) {
      Serial.println("Do you want to measure? Write 1 to measure");
      int measure;
      while (!Serial.available()) {}
      measure = Serial.parseInt();
      if (measure == 1) {
        int randomValue = random(0, 2);  // Generates either 0 or 1
        uint8_t value = randomValue;
        if (randomValue == 0) {
          Serial.println("State up!");
        } else {
          Serial.println("State down!");
        }
        bleentg_2.notify8(value);
        Serial.print("Sent notification: ");
        Serial.println(value);
      }
    }
  }
}

// Callback function to handle scanning results
void scan_callback(ble_gap_evt_adv_report_t* report) {
  uint8_t device_name[32] = { 0 };

  Bluefruit.Scanner.parseReportByType(report, BLE_GAP_AD_TYPE_COMPLETE_LOCAL_NAME, device_name, sizeof(device_name));

  if (device_name[0] != 0) {
    Serial.print("Found device: ");
    Serial.println((const char*)device_name);

    if (strcmp((const char*)device_name, "QBEAD") == 0) {
      Serial.println("QBEAD found! Attempting to connect...");

      // Attempt to connect to the device
      uint16_t conn_handle = Bluefruit.Central.connect(report);
      if (conn_handle) {
        Serial.println("Connected to QBEAD!");
        delay(1000); // Add a bit of delay

        // Discover services and characteristics
        bool discovered = false;

        for (int i = 0; i < 3; i++) {
          if (Service_1.discover(conn_handle)) {
            Serial.println("Service discovered!");

            if (bleentg_1.discover()) {
              Serial.println("Characteristic discovered!");

              if (bleentg_1.enableNotify()) {
                Serial.println("Subscribed to notifications!");
                bleentg_1.setNotifyCallback(communicate_1);  // Set callback for notification
              } else {
                Serial.println("Failed to subscribe to notifications.");
              }
            } else {
              Serial.println("Failed to discover characteristic.");
            }

            discovered = true;
            break;
          } else {
            Serial.println("Failed to discover service.");
            delay(500);
          }
        }

        return;  // Exit the scan callback once connected
      } else {
        Serial.println("Failed to connect!");
      }
    }
  } else {
    Serial.println("Device found, but no name in advertisement packet.");
  }

  Bluefruit.Scanner.resume();  // Resume scanning if "QBEAD" is not found
}

// Callback function to handle received notifications
void communicate_1(BLEClientCharacteristic* chr, uint8_t* data, uint16_t len) {
  if (len == 1) {
    uint8_t value = data[0];
    if (value == 0x01) {
      Serial.println("State up.");
      // Implement LED indication for 'up' state
      bead.clear();
      bead.setBloch_deg(0, 0, color(0, 255, 0));  // Green color for 'up' state
      bead.show();
    } else if (value == 0x00) {
      Serial.println("State down.");
      // Implement LED indication for 'down' state
      bead.clear();
      bead.setBloch_deg(180, 0, color(255, 0, 0));  // Red color for 'down' state
      bead.show();
    } else {
      Serial.println("Unknown value received.");
    }
  } else {
    Serial.println("Unexpected notification length.");
  }
}
