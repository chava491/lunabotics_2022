/* This script was written to test the performance of the 
 * cheap chinese laser reciever modules that we purchased 
 * from Amazon
 * 
 * Written by: Abraham Yakisan
 * Date: 3/30/2022
 */

//pins
const int laser_read_pin = 22;

void setup() {
  Serial.begin(57600);
  Serial.println("===============================");
  Serial.println("Starting Laser Reciever Test...");
  Serial.println("===============================");

  pinMode(laser_read_pin, INPUT);
}

void loop() {
  bool hit = digitalRead(laser_read_pin);  //bool value
  String msg1 = "HIT OCURRED AT: ";
  String msg2 = " ms";
  if(hit){
    String hit_msg = msg1 + millis() + msg2;
    Serial.println(hit_msg); 
  }

  //delay(1);

}
