/******************************************************************************
 * SparkFun_VL6180X_demo.ino
 * Example Sketch for VL6180x time of flight range finder.
 * Casey Kuhns @ SparkFun Electronics
 * 10/29/2014
 * https://github.com/sparkfun/SparkFun_ToF_Range_Finder-VL6180_Arduino_Library
 *
 * The VL6180x by ST micro is a time of flight range finder that
 * uses pulsed IR light to determine distances from object at close
 * range.  The average range of a sensor is between 0-200mm
 *
 * Resources:
 * This library uses the Arduino Wire.h to complete I2C transactions.
 *
 * Development environment specifics:
 * 	IDE: Arduino 1.0.5
 * 	Hardware Platform: Arduino Pro 3.3V/8MHz
 * 	VL6180x Breakout Version: 1.0
 *  **Updated for Arduino 1.6.4 5/2015**

 *
 * This code is beerware. If you see me (or any other SparkFun employee) at the
 * local pub, and you've found our code helpful, please buy us a round!
 *
 * Distributed as-is; no warranty is given.
 ******************************************************************************/

#include <Wire.h>

#include <SparkFun_VL6180X.h>

/*const float GAIN_1    = 1.01;  // Actual ALS Gain of 1.01
const float GAIN_1_25 = 1.28;  // Actual ALS Gain of 1.28
const float GAIN_1_67 = 1.72;  // Actual ALS Gain of 1.72
const float GAIN_2_5  = 2.6;   // Actual ALS Gain of 2.60
const float GAIN_5    = 5.21;  // Actual ALS Gain of 5.21
const float GAIN_10   = 10.32; // Actual ALS Gain of 10.32
const float GAIN_20   = 20;    // Actual ALS Gain of 20
const float GAIN_40   = 40;    // Actual ALS Gain of 40
*/
#define VL6180X_ADDRESS 0x29


VL6180xIdentification identification;
VL6180x sensor(VL6180X_ADDRESS);

void setup()
{
  Serial.begin(115200);  // Start Serial at 115200bps
  Serial.println("[Re]Boot of the ArduinoNano");
  Wire.begin();          // Start I2C library
  delay(100);            // delay .1s

  // rebootSensor();

  Serial.println("End of setup, wait 1s");  // Initialize device and check for errors
  delay(1000);  // delay 1s
}

void loop()
{
  // float Lux = sensor.getAmbientLight(GAIN_1);
  // if (isnan(Lux))
  // {
  //   rebootSensor();
  // }

  // Get Distance and report in
  // uint8_t Distance = sensor.getDistance();
  Serial.println("34");
  // Serial.end();
  // Serial.begin(115200);
  delay(5);

};

void rebootSensor()
{
  while (true)
  {
    Serial.println("Try to initialize VL6180x ...");  // Initialize device and check for errors
    uint8_t ret = sensor.VL6180xInit();
    if(ret == 0)
    {
      sensor.VL6180xDefautSettings();  // Load default settings to get started.
      Serial.println("Initialization is successfull");  // Initialize device and check for errors
      break;
    }
    else
    {
      Serial.println("Initialization has failed");  // Initialize device and check for errors
    }
    delay(1000);
    Serial.end();
    Serial.begin(115200);  // Start Serial at 115200bps
  }
}