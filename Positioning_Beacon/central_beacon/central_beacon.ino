/**
  BLE_Central_Device.ino

  This program uses the ArduinoBLE library to set-up an Arduino Nano 33 BLE Sense 
  as a central device and looks for a specified service and characteristic in a 
  peripheral device. If the specified service and characteristic is found in a 
  peripheral device, the last detected value of the on-board gesture sensor of 
  the Nano 33 BLE Sense, the APDS9960, is written in the specified characteristic. 

  The circuit:
  - Arduino Nano 33 BLE Sense. 

  @def In context of Bluetooth Low Energy (BLE) communication:
    SERVICE: This is simply related data and functionalities that a peripheral device can offer to a central device.
    CHARACTERISTIC: This is specific data that a peripheral has within a service (shows that it has it). These
    characteristics are where we can write/read data, receive notifications with an update on data changes, etc.

    DEF: When we want to interact with a CHARACTERISTIC: we would say we are "look(ing) for a specified service and 
    characteristic in a peripheral device" as above. We can do this by doing the following:

      UUIDS: (Universally Unique Identifiers) 
        - This is how the program can identify the service and characteristics. On succesfull completion of the identification
        of the correct characteristic and service, we can write/read data to the characteristics whenever we want.
        - SIZE: 128-bits

      HOW::TO::OBTAIN::UUIDS:
        - Through Bluetooth Special Interest Group (SIG)
            > Refer to Bluetooth SIG website https://www.bluetooth.com/specifications/assigned-numbers/
              This website contains the list of the standardized services and their corresponding characteristics.
        - Custom UUIDs created by me
            > Use a UUID generator to generate a 128-bit UUID
            > USE "uuidgen" on my mac terminal for a custom UUIDs.
            NOTE: Custom UUID cannot match an existing standardized UUIDs.
      
      RSSI: 
        - RSSI stands for Received Signal Strength Indicator. It is a measurement of the power level of a received radio signal, 
        typically in units of decibels (dB). In the context of wireless communication, RSSI is used to estimate the proximity of 
        a transmitter to a receiver.

        - RSSI is a relative measurement, meaning that it provides an indication of the strength of the received signal compared to 
        a reference level. The reference level can vary depending on the system, but it is typically the minimum signal strength 
        that the receiver can detect.

        - In general, a higher RSSI value indicates a stronger received signal, while a lower value indicates a weaker signal. 
        However, the actual RSSI value can be affected by a number of factors, such as distance, obstacles, interference, and 
        environmental conditions. Therefore, it is important to interpret RSSI values in the context of the specific system and application.
*/

#include <ArduinoBLE.h>
#include <BLECharacteristic.h>

const char* deviceServiceUuid = "6BEF468D-C030-4DCB-9EA6-C6B11385664E"; //same call on Periph.
const char* deviceServiceCharacteristicUuid = "0F428CF1-190F-4BFC-BB9E-2833DB622A31";  //same call on Periph.

int rss = -1;
int oldRssValue = -1;

BLEDevice peripheral;
/**
  @fn begin()
  @brief "When BLE.begin() is called, it initializes the BLE stack, sets the default TX power level to the maximum 
  value, and configures the built-in LED on the Arduino Nano 33 BLE Sense to indicate the status of the BLE connection. 
  It also starts the internal clock used by the BLE stack to schedule events, such as advertising or scanning.
  @returns 0 if initialization fails so the while loop is while(1) and can continue to try to initialize BLE
  @returns non-zero number if initialization is successful.
  */

void setup() {
  Serial.begin(9600); //set the serial port to 9600, same as Baud rate
  while (!Serial);
  Serial.println("* RUNNING CENTRAL BEACON SETUP!");
  while (!BLE.begin()) {
    Serial.println("* Starting Bluetooth FAILED!");
  }
  
  BLE.setLocalName("Nano 33 BLE (Central)"); 
  /**
  @brief When a device calls BLE.advertise(), it starts advertising its presence by broadcasting advertising packets at regular intervals. 
  These advertising packets contain information about the device, such as its name, services offered, and other relevant data.
   */
  BLE.advertise();

  Serial.println("Arduino Nano 33 BLE CENTRAL");
  Serial.println(" ");
}

void loop() {
  connectToPeripheral();
}

void connectToPeripheral(){
  //BLEDevice peripheral;
  
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

void onCharacteristicWritten(BLEDevice peripheral, BLECharacteristic rssCharacteristic) {
  uint8_t rssi_length = 1;
    uint8_t rssi_value[rssi_length];
    //Serial.println(rssCharacteristic.read());
    rssCharacteristic.readValue(rssi_value, rssi_length);

    Serial.print("RSSI value: ");
      for (int i = 0; i < rssi_length; i++) {
      Serial.println(rssi_value[i]-256);
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

  rssCharacteristic.setEventHandler(BLEWritten, onCharacteristicWritten);
  // Subscribe to notifications on the characteristic
  rssCharacteristic.subscribe();
  Serial.println("Subscribed to peripheral characteristic notifications");
  Serial.println("Reading RSSI");

  while (peripheral.connected()) {
    //DO NOTHING EVENT HANDLER WILL TAKE CARE OF IT
  }
  Serial.println("- Peripheral device disconnected!");
}

int rssDetection(){
  rss = BLE.rssi();
  return rss;
}
