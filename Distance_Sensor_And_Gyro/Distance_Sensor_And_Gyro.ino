/**
   @file Distance_Sensor_And_Gyro.ino
 
   @author salvador guel
   @date February 15, 2023

   @version S1.G0
   @brief Code example to initialize and read values from a VL53L0X T0F distance Sensor and GY-521 3-axis Gyroscope/Accelerometer sensor'
   
   @details In this program, we use the Wire library to communicate with both sensors via I2C on the SDA and SCL lines of the Arduino. 
   We create sensor objects for the VL53L0X and MPU6050 sensors, and initialize them in the setup() function.
   In the loop() function, we take measurements from both sensors and print out the results using the Serial object.

   @section dependecies: Please make sure the following libraries are installed: 
   https://github.com/adafruit/Adafruit_VL53L0X @version 1.2.2
   https://github.com/jarzebski/Arduino-MPU6050 @version 1.0.3
      a) (This is not the Adafruit_MPU6050. That is completely different, need to git clone it into the lib/include of arduino ide environment.)
    
   @todo Gyro and accelerometer need to be recalibrated. I just want to check if this can be done automatically and quickly.
      - Look for the following youtube videos to see how to average out the values and fix the Gyros value drift with the accelerametor.
        1) MPU-6050 6dof IMU tutorial for auto-leveling quadcopters with Arduino source code, by Joop Brokking 
            - https://www.youtube.com/watch?v=4BoIE8YQwM8
        2) MPU-6050 6dof IMU tutorial for auto-leveling quadcopters with Arduino source code - Part 2, by Joop Brokking 
            - https://www.youtube.com/watch?v=j-kE0AMEWy4

      THOSE YOUTUBE VIDEOS GIVE INSANELY GOOD EQUATIONS TO HAVE LONG TERM USABLE DATA WITH AUTOMATE RECALIBRATION
   NOTE: The data sheets for the two devices AND their github repositories are as follow:
    VL53L0X:
      - https://www.adafruit.com/product/3317
      - https://github.com/adafruit/Adafruit_VL53L0X
      - https://learn.adafruit.com/adafruit-vl53l0x-micro-lidar-distance-sensor-breakout/overview (extra info)
    GY-521 (aka MPU6050):
      - https://www.addicore.com/GY-521-MPU6050-p/170.htm
      - https://github.com/jarzebski/Arduino-MPU6050

   NOTE: The MPU6050 is a commonly used integrated circuit that combines a 3-axis accelerometer and a 3-axis gyroscope in a single package. 
    - This Gyroscope measures the rotational motion of the sensor around each of its axes, and its output is expressed in units of degrees per second (dps).
    - This Accelerometer measures in three perpendicular axes, and its output is expressed in three x, y, and z values which describe how much gravity the
    sensor is experiencing on each respective axis. For example, if an accelerometer outputs a value of 2g along its x-axis, this indicates that the sensor is
    experiencing an acceleration of twice the acceleration due to gravity along the x-axis.
 */
#include <Wire.h>             /* Include the Wire library for I2C communication                     */
#include <Adafruit_VL53L0X.h> /* Include the Adafruit_VL53L0X library for the distance sensor
                                 - Link to repository: https://github.com/adafruit/Adafruit_VL53L0X */
#include <MPU6050.h>          /* Include the MPU6050 library for the accelerometer and gyroscope: 
                                 - Library Documentation: https://www.i2cdevlib.com/docs/html/class_m_p_u6050.html#a7c0146d45537e4bd7a0d4c1c476fdab7 */

/* define the I2C addresses of the sensors */
#define VL53L0X_ADDRESS 0x29
#define MPU6050_ADDRESS 0x68

/** 
  @brief The slave address of the MPU-60X0 is b110100X which is 7 bits long. The LSB bit of the 7 bit address is determined by the logic level on pin AD0. 
   This allows two MPU-60X0s to be connected to the same I2C bus. When used in this configuration, the address of the one of the devices should be 
     - b1101000 or 0x68 (pin AD0 is logic low) 
   and the address of the other should be 
     - b1101001 or 0x69 (pin AD0 is logic high).
 
   (NOTE: ADO is default low. Thus, deafult I2C address is: 0x68)
 */
Adafruit_VL53L0X sensor;  // create sensor object
MPU6050 mpu6050;          // create a mpu object
/**
  @brief This function checks all theoretically possible connections to the SCL and SDA I2C bus. (Theoretical max I2C devices connected to bus = 127)
 */
void check_I2C_bus_num_connections(){
  byte err, adr;                       /*variable error is defined with address of I2C*/
  int number_of_devices;
  Serial.println("Scanning.");
  number_of_devices = 0;
  for (adr = 1; adr < 127; adr++ ){
    Wire.beginTransmission(adr);
    err = Wire.endTransmission();

    if (err == 0){
      Serial.print("I2C device at address 0x");
      if (adr < 16)
        Serial.print("0");
      Serial.print(adr, HEX);
      Serial.println("  !");
      number_of_devices++;
    }
    else if (err == 4){
      Serial.print("Unknown error at address 0x");
      if (adr < 16)
        Serial.print("0");
      Serial.println(adr, HEX);
    }
  }
  if (number_of_devices == 0)
    Serial.println("No I2C devices attached\n");
  else
    Serial.println("done\n");
}

/**
  @brief This function is to print out the set configuartions of the the GY-521. Added the F() around string literals to free up space to due using up over 100% of 
  dynamic memory.  obtained from the following, thus, look at the following for more details: https://support.arduino.cc/hc/en-us/articles/360013825179
  
  @return Whether or not it is in sleep mode
  @return What clock source the GY-521 is using
  @return The greatest amount of acceleration the part can measure and accurately represent as an output. This is the level
  of acceleration supported by the sensor's output signal specifications, typically specified in +/- g.
    NOTE: +/- x g means x amount of times more than gravity. So if we have +/- 2 that means we can measure accelerations ranging from -2g to + 2g 
      (where g is earth gravity which is 9.812 m/s^2).
    NOTE: In general, the larger the measurement range of an accelerometer, the more versatile it is, as it can detect a wider range of accelerations. 
      However, a larger range can also reduce the resolution of the measurements, as the same number of digital steps are used to represent a wider range of values. 
      Therefore, the selection of an appropriate range depends on the specific application and the expected acceleration levels.
  @return Accelerometer offset (x / y / z)
    NOTE: This is to account for the deviation from ideal conditions.
 */

void checkSettings(){
  Serial.println();
  
  Serial.print(F(" * Sleep Mode:            "));
  Serial.println(mpu6050.getSleepEnabled() ? F("Enabled") : F("Disabled"));
  
  Serial.print(" * Clock Source:          ");
  switch(mpu6050.getClockSource()){
    case MPU6050_CLOCK_KEEP_RESET:     Serial.println(F("Stops the clock and keeps the timing generator in reset")); break;
    case MPU6050_CLOCK_EXTERNAL_19MHZ: Serial.println(F("PLL with external 19.2MHz reference")); break;
    case MPU6050_CLOCK_EXTERNAL_32KHZ: Serial.println(F("PLL with external 32.768kHz reference")); break;
    case MPU6050_CLOCK_PLL_ZGYRO:      Serial.println(F("PLL with Z axis gyroscope reference")); break;
    case MPU6050_CLOCK_PLL_YGYRO:      Serial.println(F("PLL with Y axis gyroscope reference")); break;
    case MPU6050_CLOCK_PLL_XGYRO:      Serial.println(F("PLL with X axis gyroscope reference")); break;
    case MPU6050_CLOCK_INTERNAL_8MHZ:  Serial.println(F("Internal 8MHz oscillator")); break;
  }
  
  Serial.print(" * Accelerometer:         ");  
  switch(mpu6050.getRange()){
    case MPU6050_RANGE_16G:            Serial.println(F("+/- 16 g")); break;
    case MPU6050_RANGE_8G:             Serial.println(F("+/- 8 g")); break;
    case MPU6050_RANGE_4G:             Serial.println(F("+/- 4 g")); break;
    case MPU6050_RANGE_2G:             Serial.println(F("+/- 2 g")); break;
  }  

  Serial.print(F(" * Accelerometer offsets (x / y / z): "));
  Serial.print(mpu6050.getAccelOffsetX());
  Serial.print(F(" / "));
  Serial.print(mpu6050.getAccelOffsetY());
  Serial.print(F(" / "));
  Serial.println(mpu6050.getAccelOffsetZ());
  Serial.println();
}

/**
  @brief This function will initialize the GY-521 breakout board for the MPU-6050 by setting the desired parameters for the function .begin() which are
    1) degrees per second range for the MPU 6050. 2000 DPS is the max rate at which the MPU6050 can support. 
       The default range is +/- 250 DPS (defined as MPU6050_SCALE_250DPS aka 0b00), which means the sensor can measure rotational velocities up to 250 degrees per second in 
       either direction around each axis. However, the range can also be set to +/- 500 DPS, +/- 1000 DPS, or +/- 2000 DPS, depending on the specific application requirements 
       by using pre-defined:
          a) MPU6050_SCALE_2000DPS (0b11)
          b) MPU6050_SCALE_1000DPS (0b10)
          c) MPU6050_SCALE_500DPS  (0b01)

        which are found in MPU6050.h.

      NOTE: It's important to note that a higher measurement range results in a lower resolution of the sensor, meaning that small changes in angular velocity may not 
      be accurately measured. On the other hand, a lower range provides a higher resolution, but at the expense of limiting the maximum measurable angular velocity. 

    2) The measurement range of the gyroscope in the MPU6050 can be configured to different values using the device's registers. This range simply tells the mpu6050 the 
    max acceleration in relation to gravity it can measure. Thus if it is +/- 2 g (As in MPU6050_RANGE_2G) then the MPU6050 can read accelerations of each axis in the 
    range of -2g to +2g where g = 9.81 m/s^2

    3) The final .begin input I used is for the I2C address. Here I have this address in the variable MPU6050_ADDRESS which is 0x68.
 */
void gyro_init(int message_delay){
  Serial.println("Initialize MPU6050");  
  while (!mpu6050.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G, MPU6050_ADDRESS)){
    Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
    delay(500);
  }
  mpu6050.setI2CBypassEnabled(true);
  checkSettings();
  delay(message_delay);
}

/**
  @brief This function is to intialize the VL53L0X T0F distance Sensor. This is acheived by using the .begin() function.
  @details When the VL53L0X_DEVICEMODE_CONTINUOUS_RANGING mode is set, the sensor continuously performs ranging measurements and outputs the distance to the target object, 
  without any intervention from the host system. The ranging measurements are outputted in millimeters.
    NOTE: The VL53L0X_DEVICEMODE_CONTINUOUS_RANGING mode is useful for applications where continuous ranging is required, such as in robotics, drones, or any other 
    application where real-time distance sensing is important. By setting the VL53L0X sensor to continuous ranging mode, the host system can easily obtain distance 
    measurements without having to repeatedly trigger the sensor.

    TODO: It's important to note that setting the sensor to continuous ranging mode can consume more power than other device modes, since the sensor is constantly measuring 
    distances. Therefore, it's recommended to use the continuous ranging mode only when necessary, and to turn off the sensor when it's not in use to conserve power. 
    
    TODO: Here, I will have to use the XSHUT pin.(By default it is HIGH)
      a) If set LOW (by using a digital I/O and using pin.write(pin_number, LOW))
        i) VL53LOX sensor is turned off.
      b) If set HIGH (by using a digital I/O and using pin.write(pin_number, HIGH))
        i) VL53LOX sensor is turned on.
 */
void distance_sensor_init(){
  Serial.println("Initialize VL53L0X");
  while (!sensor.begin()) {
    Serial.println("VL53L0X ToF Distance Sensor=> Failed to Initialize");
    delay(1000);
  }
  // Use long distance mode and allow up to 50000 us (50 ms) for a measurement.
  sensor.setDeviceMode(VL53L0X_DEVICEMODE_CONTINUOUS_RANGING);
  sensor.setMeasurementTimingBudgetMicroSeconds(100000);
  Serial.println();
}

/**
  @brief This first initializes I2C communication and then Serial communication and then simply calls the previously mentions functions which are:
    @return check_I2C_bus_num_connections()
    @return gyro_init(message_delay) @param message_delay is time in ms (1000 = 1 second) to delay to view debug message
    @return distance_sensor_init()  
 */
void setup() {
  Wire.begin();       // initialize I2C communication, and we join the I2C bus as a master. If we have Wire.begin(7-bit slave address) then we join the bus as I2C slave.
  Serial.begin(9600); // initialize serial communication
  while (!Serial) { } // Wait until serial communication initialization is successfull
  check_I2C_bus_num_connections();
  delay(3000);
  gyro_init(3000);
  distance_sensor_init();
  delay(2000);
}

void loop() {
  // read distance sensor data
  VL53L0X_RangingMeasurementData_t measure; // VL53L0X_RangingMeasurementData_t is a struct with many uints. Defined in VL53L0X_def.h, which is the type definition file for 
                                            //    VL53L0X API
  sensor.rangingTest(&measure, false);      //change bool to true to see DEBUG

  if (measure.RangeStatus != 4) {           // When Range Status is 4 this means Phase Failure (meaning we have incorrect data)
                                            // This is when there is no reading avaialable. (aka nothing is within the range of the sensors ability to "see")
    Serial.print("Distance (mm): ");
    Serial.println(sensor.readRange());     //readRange returns measure.RangeMilliMeter (thus, "Serial.println(measure.RangeMilliMeter);" can do the same)
  } else {
    Serial.println("Distance (mm): *OUT OF BOUNDS*");
  }
  
  Vector rawAccel = mpu6050.readRawAccel(); // read accelerometer and store in a struct named Vector with float XAxis, float YAxis, and float ZAxis;
  Serial.print("Xraw = ");
  Serial.print(rawAccel.XAxis);
  Serial.print(" Yraw = ");
  Serial.print(rawAccel.YAxis);
  Serial.print(" Zraw = ");
  Serial.println(rawAccel.ZAxis);
  Serial.println("----------------------------------------");
  delay(1000); // wait for a second second before taking another measurement
}
