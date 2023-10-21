
/*
* Original sourse: https://github.com/adafruit/Adafruit-PWM-Servo-Driver-Library
* This is the Arduino code PAC6985 16 channel servo controller
* watch the video for details and demo http://youtu.be/y8X9X10Tn1k
*  *

* Watch video for this code:
*
* Related Videos
V5 video of PCA9685 32 Servo with ESP32 with WiFi https://youtu.be/bvqfv-FrrLM
V4 video of PCA9685 32 Servo with ESP32 (no WiFi): https://youtu.be/JFdXB8Za5Os
V3 video of PCA9685 how to control 32 Servo motors https://youtu.be/6P21wG7N6t4
V2 Video of PCA9685 3 different ways to control Servo motors: https://youtu.be/bal2STaoQ1M
V1 Video introduction to PCA9685 to control 16 Servo  https://youtu.be/y8X9X10Tn1k

* Written by Ahmad Shamshiri for Robojax Video channel www.Robojax.com
* Date: Dec 16, 2017, in Ajax, Ontario, Canada
* Permission granted to share this code given that this
* note is kept with the code.
* Disclaimer: this code is "AS IS" and for educational purpose only.
* this code has been downloaded from http://robojax.com/learn/arduino/
*
*/
/*************************************************** 
  This is an example for our Adafruit 16-channel PWM & Servo driver
  Servo test - this will drive 16 servos, one after the other

  Pick one up today in the adafruit shop!
  ------> http://www.adafruit.com/products/815

  These displays use I2C to communicate, 2 pins are required to  
  interface. For Arduino UNOs, thats SCL -> Analog 5, SDA -> Analog 4

  Adafruit invests time and resources providing this open source code, 
  please support Adafruit and open-source hardware by purchasing 
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.  
  BSD license, all text above must be included in any redistribution
 ****************************************************/

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include "Dictionary.h"

class Servo
{
public:
    explicit Servo(const String& n, uint16_t minAng, uint16_t maxAng) : name(n), minAngle(minAng), maxAngle(maxAng)
    {};
private:
    String name;
    uint16_t minAngle {0};
    uint16_t maxAngle {180};
};

class ServoDriver
{

public:
    Adafruit_PWMServoDriver driver;

    void setup()
    {
        driver.begin();
        driver.setPWMFreq(this->servoFrequency);
    }

    void setServoFrequency(uint8_t frequency)
    {
        this->servoFrequency = frequency;
    }

    void addServo(uint8_t servoIndex, Servo servo)
    {
        servoMap.set(servoIndex, servo);
    }

    void moveServo(const String& servoName, uint16_t angle)
    {

    }

private:
    Dictionary<uint8_t , Servo> servoMap;
    uint8_t servoFrequency {60};
    uint16_t minPulseLength {150};
    uint16_t maxPulseLength {600};


    long angleToPulse(uint16_t angle, uint16_t minAng, uint16_t maxAng)
    {
        if (angle > maxAng || angle < minAng) {
            return -1;
        }
        return map(angle, minAng, maxAng, minPulseLength, maxPulseLength);
    };

};

ServoDriver servoDriver;

void setup()
{
    Serial.begin(9600);
    Serial.println("16 channel Servo test!");

    servoDriver.setup();

    //yield();
}

// the code inside loop() has been updated by Robojax
void loop()
{


    servoDriver.addServo(0, Servo("xaxis", 0,180));
    servoDriver.addServo(0, Servo("yaxis", 0,180));

    servoDriver.moveServo()
    for (int angle = 0; angle < 181; angle += 20) {
        delay(500);
        sd.setPWM(0, 0, angleToPulse(angle));
    }

    delay(1000);

}
