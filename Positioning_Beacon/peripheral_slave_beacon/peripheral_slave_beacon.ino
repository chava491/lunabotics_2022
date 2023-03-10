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

const char* deviceServiceUuid = "6BEF468D-C030-4DCB-9EA6-C6B11385664E";
const char* deviceServiceCharacteristicUuid = "0F428CF1-190F-4BFC-BB9E-2833DB622A31";

int rss = -1;

BLEService rssService(deviceServiceUuid);
BLEByteCharacteristic rssCharacteristic(deviceServiceCharacteristicUuid, BLERead | BLEWrite | BLENotify | BLEBroadcast);

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
/*
  the RSSI value is the signal strength of the connection between the peripheral and the central device. 
  The peripheral device is sending the RSSI value to the central device through the RSSI characteristic of the service. 
  The central device can read this value when it is written to the characteristic.
 */
    while (central.connected()) {
        rss = central.rssi();
        rssCharacteristic.writeValue(rss);
        Serial.print("Wrote rssi: ");
        Serial.println(rss);
        delay(500);
    }
    
    Serial.println("* DISCONNECTED from central device!");
  }
}

