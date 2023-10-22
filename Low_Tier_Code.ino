#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include "Dictionary.h"

class Servo
{
public:
    explicit Servo(const String& name, uint8_t index, uint16_t minAng, uint16_t maxAng) : name(name), index(index),
                                                                                          minAngle(minAng),
                                                                                          maxAngle(maxAng)
    {};

    const String& getName() const
    {
        return this->name;
    }

    uint8_t getIndex() const
    {
        return this->index;
    }

    uint8_t getMinAngle() const
    {
        return this->minAngle;
    }

    uint8_t getMaxAngle() const
    {
        return this->maxAngle;
    }

private:
    String name;
    uint8_t index {0};
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

    void setDriverFrequency(uint8_t frequency)
    {
        this->servoFrequency = frequency;
    }

    void addServo(const Servo& servo)
    {
        if (servosMap.contains(servo.getName())){
            return;
        }

        servosMap.set(servo.getName(), servo);
    }

    void moveServo(const String& servoName, uint16_t angle)
    {
        Servo servo = servosMap.get(servoName);

        if (angle > servo.getMaxAngle() || angle < servo.getMinAngle()) {
            return;
        }

        driver.setPWM(servo.getIndex(), 0, angleToPulse(servo.getMinAngle(), servo.getMaxAngle(), angle));
    }

private:
    Dictionary<String, Servo> servosMap;
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
    servoDriver.setup();
    yield();
}

void loop()
{
    servoDriver.addServo(Servo("xaxis", 0, 0, 180));
    servoDriver.addServo(Servo("yaxis", 1, 0, 180));

    for (uint16_t angle = 0; angle <= 180; angle++) {
        delay(30);
        servoDriver.moveServo("xaxis", angle);
        servoDriver.moveServo("yaxis", angle);
    }
}
