/*
  BLE_Central_Device.ino

  This program uses the ArduinoBLE library to set-up an Arduino Nano 33 BLE Sense 
  as a central device and looks for a specified service and characteristic in a 
  peripheral device. If the specified service and characteristic is found in a 
  peripheral device, the last detected value of the on-board gesture sensor of 
  the Nano 33 BLE Sense, the APDS9960, is written in the specified characteristic. 

  The circuit:
  - Arduino Nano 33 BLE Sense. 

  This example code is in the public domain.
*/

#include <ArduinoBLE.h>

const char* deviceServiceUuid = "19b10000-e8f2-537e-4f6c-d104768a1214"; //same call on Periph.
const char* deviceServiceCharacteristicUuid = "19b10001-e8f2-537e-4f6c-d104768a1214";  //same call on Periph.

int rss = -1;
int oldRssValue = -1;

void setup() {
  Serial.begin(9600); //set the serial number to 9600, same as Baud rate
  while (!Serial);

  if (!BLE.begin()) {
    Serial.println("* Starting Bluetooth FAILED!");
    while (1);
  }
  
  BLE.setLocalName("Nano 33 BLE (Central)"); 
  BLE.advertise();

  Serial.println("Arduino Nano 33 BLE CENTRAL");
  Serial.println(" ");
}

void loop() {
  connectToPeripheral();
}

void connectToPeripheral(){
  BLEDevice peripheral;
  
  Serial.println("- Discovering peripheral device...");

  do
  {
    BLE.scanForUuid(deviceServiceUuid);
    peripheral = BLE.available();
  } while (!peripheral);
  
  if (peripheral) {
    Serial.println("* Peripheral device found!");
    Serial.print("* Device MAC address: ");
    Serial.println(peripheral.address());
    Serial.print("* Device name: ");
    Serial.println(peripheral.localName());
    Serial.print("* Advertised service UUID: ");
    Serial.println(peripheral.advertisedServiceUuid());
    Serial.println(" ");
    BLE.stopScan();
    controlPeripheral(peripheral);
  }
}

void controlPeripheral(BLEDevice peripheral) {
  Serial.println("- Connecting to peripheral device...");

  if (peripheral.connect()) {
    Serial.println("* Connected to peripheral device!");
    Serial.println(" ");
  } else {
    Serial.println("* Connection to peripheral device failed!");
    Serial.println(" ");
    return;
  }

  Serial.println("- Discovering peripheral device attributes...");
  if (peripheral.discoverAttributes()) {
    Serial.println("* Peripheral device attributes discovered!");
    Serial.println(" ");
  } else {
    Serial.println("* Peripheral device attributes discovery failed!");
    Serial.println(" ");
    peripheral.disconnect();
    return;
  }

  BLECharacteristic rssCharacteristic = peripheral.characteristic(deviceServiceCharacteristicUuid);

  if (!rssCharacteristic) {
    Serial.println("* Peripheral does not have rss characteristic!");
    peripheral.disconnect();
    return;
  } else if (!rssCharacteristic.canWrite()) {
      Serial.println("* Peripheral does not have a writable rss characteristic!");
      peripheral.disconnect();
      return;
  }

  while (peripheral.connected()) {
    rss = rssDetection();

    if (oldRssValue != rss){
      oldRssValue = rss;
      Serial.print("*Writing value to rss characteristic: ");
      Serial.println(rss);
      rssCharacteristic.writeValue((byte)rss);
      Serial.println("*Writing value to rss characteristic DONE!");
      Serial.println(" ");
    }

    //Serial.print("RSSI bleDevice = ");
    //Serial.println(bleDevice.rssi());
  
  }
  Serial.println("- Peripheral device disconnected!");
}

int rssDetection(){
  BLEDevice peripheral;
  
  rss = BLE.rssi();
  
  return rss;
}
