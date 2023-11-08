#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <Dictionary.h>
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Depending on your servo make, the pulse width min and max may vary, you
// want these to be as small/large as possible without hitting the hard stop
// for max range. You'll have to tweak them as necessary to match the servos you
// have!
#define SERVOMIN 125 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX 575 // this is the 'maximum' pulse length count (out of 4096)

#define EYES_horizontal 0 // buvly leva prava 0-80  _40 je stred
#define EYES_vertical 1   // bulvy nahoru dolu 70-110 _90 je stred
#define LID_right_up 2    // pravy horni vicko 90-165 
#define LID_right_down 3  // pravy dolni vicko 90-70
#define LID_left_up 4     // levy horni vicko 90-15
#define LID_left_down 5   // levy dolni vicko 90-110

long angleToPulse(uint16_t angle)
{
  return map(angle, 0, 180, SERVOMIN, SERVOMAX); // map angle of 0 to 180 to Servo min and Servo max
}

void setup()
{
  Serial.begin(9600);
  pwm.begin();
  pwm.setPWMFreq(60);
  // yield();
}
//  90-10

/*void blinkRight()
{

  uint16_t up_lid_max_angle = 80;
  uint16_t down_lid_max_angle = 10;

  uint16_t up_lid_angle = 0;
  uint16_t down_lid_angle = 0;
} 
  while (up_lid_angle <= up_lid_max_angle)
 
  {
    up_lid_angle += 2;
    pwm.setPWM(LID_right_up, 0, angleToPulse(up_lid_angle));

    if (up_lid_angle % 10 == 0)
    {
      down_lid_angle++;
      pwm.setPWM(LID_right_down, 0, angleToPulse(down_lid_angle));
    }
}
  
    delay(40);
    
    
    
      
      
 }

  uint16_t up_lid_angle = 0;   // zjistime jakej je defaultni stav
  uint16_t down_lid_angle = 0; // zjistime jakej je defaultni stav

  for (uint16_t i = 0; i < 5; i++)
  {
    pwm.setPWM(LID_right_up, 0, angleToPulse(up_lid_angle));
    pwm.setPWM(LID_right_down, 0, angleToPulse(down_lid_angle));

    up_lid_angle += 9;
    down_lid_angle += 1;

    delay(1000);
  }
}


*/

/*void blinkLeft()
{
  uint16_t maxAngle = 45;

  for (uint16_t angle = 0; angle < maxAngle; angle++)
  {
    delay(1000);
    pwm.setPWM(LID_left_up, 0, angleToPulse(angle));
    pwm.setPWM(LID_left_down, 0, angleToPulse(maxAngle - angle));
  }
}


void blinkBoth()
{
  uint16_t maxAngle = 45;

  for (uint16_t angle = 0; angle < maxAngle; angle++)
  {
    delay(1000);
    pwm.setPWM(LID_left_up, 0, angleToPulse(angle)); // ted musime zjistit kterej se kam hejbe kdyz se meni to icko
    pwm.setPWM(LID_left_down, 0, angleToPulse(maxAngle- angle));
    pwm.setPWM(LID_right_up, 0, angleToPulse(angle));
    pwm.setPWM(LID_right_down, 0, angleToPulse(maxAngle - angle));
  }
  }
*/

void loop()
{

  for (uint16_t angle = 70; angle <= 110; angle += 5)
  {
    delay(2000);
    pwm.setPWM(EYES_vertical, 1, angleToPulse(angle));
  }
  
  
 
  
  


  delay(2000);
}
