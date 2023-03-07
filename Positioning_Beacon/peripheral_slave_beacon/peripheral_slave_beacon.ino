/**
  @file peripheral_slave_beacon
  @author salvador guel
  @brief This is the setup up code for the slave beacon. This beacon will be used for the 
  robots positioning system for the autonomy. In terms of the algorithm, this is the part of the
  system that is used for the magnetism idea.

  @def USING_THE_CONCEPT_OF_REPULSIVE_AND_ATTRACTIVE_FORCES:​
        - The 2 Bluetooth beacons on the sieve will act as​
          > Repulsive force: When the goal is to reach the mining zone to mine more regolith​
          >Attractive force: When the goal is to reach the sieve to dump out the mined regolith​
        - The lidar sensor will be connected to the Arduino using a USB serial interface.​ 
        The received 2D horizontal line of data will be used to change the immediate direction of 
        the robot, while the attractive and repulsive forces will dictate the long-term direction.​
​     NOTE: View image named "repulsive_attractive_idea.png"
 */
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
