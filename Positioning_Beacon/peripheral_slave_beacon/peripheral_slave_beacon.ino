#include <ArduinoBLE.h>

const char* deviceServiceUuid = "19b10000-e8f2-537e-4f6c-d104768a1214";
const char* deviceServiceCharacteristicUuid = "19b10001-e8f2-537e-4f6c-d104768a1214";

int rss = -1;

BLEService rssService(deviceServiceUuid);
BLEByteCharacteristic rssCharacteristic(deviceServiceCharacteristicUuid, BLERead | BLEWrite);

void setup() {
  Serial.begin(9600);
  while (!Serial);

  if(!BLE.begin()) {
    Serial.println("- Starting Bluetooth FAILED!");
    while(1);
  }

  BLE.setLocalName("Arduino Nano 33 BLE PERIPHERAL");
  BLE.setAdvertisedService(rssService);
  rssService.addCharacteristic(rssCharacteristic);
  BLE.addService(rssService);
  rssCharacteristic.writeValue(-1);
  BLE.advertise();

  Serial.println("Nano 33 BLE PERIPHERAL");
  Serial.println(" ");
}

void loop() {
  BLEDevice central = BLE.central();
  Serial.println("- Discovering central device...");
  delay(500);

  if (central) {
    Serial.println("* CONNECTED to central device!");
    Serial.print("* Device MAC Address: ");
    Serial.println(central.address());
    Serial.println(" ");

    while (central.connected()) {
      if (rssCharacteristic.written()) {
        rss = rssCharacteristic.value();
        writeRss(rss); //WHAT IS THIS????????????????
      }
    }
    
    Serial.println("* DISCONNECTED to central device!");
  }
}

void writeRss(int rss) {
  BLEDevice central;
  Serial.println("- Characteristic <rss> has changed!"); //WHAT???????????????

  rss = BLE.rssi();

}
