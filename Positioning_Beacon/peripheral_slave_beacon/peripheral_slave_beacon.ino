/**
  @file peripheral_slave_beacon.ino
  @author salvador guel
  @brief This is the setup up code for the slave beacon. This beacon will be used for the robots positioning system for the autonomy. In terms of the algorithm, this is the 
  part of the system that is used for the magnetism idea. The peripheral device will advertise the service and the characteristic to the central device. The central device 
  will then discover the service and the characteristic of the peripheral device. The central device will read the RSSI value of the connection between the peripheral and the 
  central device from the characteristic of the service of the peripheral device. The RSSI value is the signal strength of the connection between the peripheral and the 
  central device. The RSSI value is a signed 8-bit integer value. The RSSI value is in dBm. dBm is a logarithmic unit of measurement for power. 

  @def USING_THE_CONCEPT_OF_REPULSIVE_AND_ATTRACTIVE_FORCES:​
        - The 2 Bluetooth beacons on the sieve will act as​
          > Repulsive force: When the goal is to reach the mining zone to mine more regolith​
          > Attractive force: When the goal is to reach the sieve to dump out the mined regolith​
        - The lidar sensor will be connected to the Arduino using a USB serial interface.​ 
        The received 2D horizontal line of data will be used to change the immediate direction of 
        the robot, while the attractive and repulsive forces will dictate the long-term direction.​
​     NOTE: View image named "repulsive_attractive_idea.png"
 */
#include <ArduinoBLE.h>

int old_rssi_val;
int rssi_val;
int debounced_rssi_val;
int rss = -1;

const char* deviceServiceUuid = "6BEF468D-C030-4DCB-9EA6-C6B11385664E";               // Service UUID which is used to identify the service of the peripheral device that will be advertised to the central device.
const char* deviceServiceCharacteristicUuid = "0F428CF1-190F-4BFC-BB9E-2833DB622A31"; // Characteristic UUID which is used to identify the characteristic of the service of the peripheral device that will be advertised to the central device.

/* 

  rssService is the service that will be advertised by the peripheral device.
  rssCharacteristic is the characteristic of the service that will be advertised by the peripheral device.

  BLECharacteristic properties/permissions (which are defined in BLECharacteristic.h):
    a) BLERead      - The characteristic can be read by a central device.
    b) BLEWrite     - The characteristic can be written to by  the peripheral device.
    c) BLENotify    - The characteristic can notify a central device of changes to the characteristic value.
    d) BLEBroadcast - The characteristic can be broadcasted by the peripheral device.
 */

BLEService rssService(deviceServiceUuid);                                                                                 // Create a service with the service UUID.  
BLEByteCharacteristic rssCharacteristic(deviceServiceCharacteristicUuid, BLERead | BLEWrite | BLENotify | BLEBroadcast);  // Create a characteristic with the characteristic UUID and the properties/permissions.

BLEDevice central;                                                                                                        // Create a BLEDevice object called central.

/* Smoothing Algorithm taken from: https://www.mdpi.com/1424-8220/19/2/424 
    - Visit website for in depth explanation
*/
int debounced_rssi(){
  debounced_rssi_val = (0.1 * central.rssi()) + (0.9 * old_rssi_val);
  old_rssi_val = debounced_rssi_val;
  return debounced_rssi_val;
}

void setup() {
  Serial.begin(9600);
  while (!Serial);

  if(!BLE.begin()) {                                  // Initialize the BLE hardware
    Serial.println("- Starting Bluetooth FAILED!");   // If BLE.begin() returns false, then the BLE hardware failed to initialize.
    while(1);
  }

  BLE.setLocalName("Arduino Nano 33 BLE PERIPHERAL"); // Set the local name of the peripheral device.
  BLE.setAdvertisedService(rssService);               // Add the service to the list of services that will be advertised by the peripheral device.
  rssService.addCharacteristic(rssCharacteristic);    // Add the characteristic to the service.
  BLE.addService(rssService);                         // Add the service to the BLE device.
  BLE.advertise();                                    // Start advertising the peripheral device.
                                                      /* 
                                                        A faster advertising interval may indirectly affect the RSSI value by increasing the frequency of data 
                                                        transmissions between the central and peripheral devices, which can lead to more accurate and timely RSSI updates. 
                                                        Additionally, a faster advertising interval may reduce the latency of the connection establishment process, which can 
                                                        reduce the time between the initial connection and the first RSSI update.
                                                      */

  Serial.println("Nano 33 BLE PERIPHERAL");
}

void loop() {
  central = BLE.central();                            // Check if a central device has connected to the peripheral device.
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
    old_rssi_val = central.rssi();                // Obtain the RSSI value of the connection between the peripheral and the central device.

    while (central.connected()) {
      int rssi_value = debounced_rssi();
      rssCharacteristic.writeValue(rssi_value);   // Write the RSSI value to the RSSI characteristic of the service.
      Serial.print("Debounced Value: ");
      Serial.println(rssi_value);
    }
    
    Serial.println("* DISCONNECTED from central device!");
  }
}

