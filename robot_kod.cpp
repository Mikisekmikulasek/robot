#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVOMIN 125 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX 575 // this is the 'maximum' pulse length count (out of 4096)

#define EYES_horizontal 0 // buvly leva prava 0-80  _40 je stred
#define EYES_vertical 1   // bulvy nahoru dolu 70-110 _90 je stred
#define LID_right_up 2    // pravy horni vicko 90-165
#define LID_right_down 3  // pravy dolni vicko 90-70
#define LID_left_up 4     // levy horni vicko 90-15
#define LID_left_down 5   // levy dolni vicko 90-110

#define EYES_HORIZONTAL_LEFT_MAX 0
#define EYES_HORIZONTAL_RIGHT_MAX 80
#define EYES_HORIZONTAL_CENTER 40

#define EYES_VERTICAL_UP_MAX 70
#define EYES_VERTICAL_DOWN_MAX 110
#define EYES_VERTICAL_CENTER 90

#define RIGHT_UP_LID_MAX 165
#define RIGHT_UP_LID_MIN 90
#define RIGHT_DOWN_LID_MAX 90
#define RIGHT_DOWN_LID_MIN 70
#define RIGHT_LIDS_POINT_OF_TOUCH 90

#define LEFT_UP_LID_MAX 90
#define LEFT_UP_LID_MIN 15
#define LEFT_DOWN_LID_MAX 110
#define LEFT_DOWN_LID_MIN 90
#define LEFT_LIDS_POINT_OF_TOUCH 90

uint16_t current_horizontal_eyes_pos{0};
uint16_t current_vertical_eyes_pos{0};
uint16_t current_right_up_lid_pos{0};
uint16_t current_right_down_lid_pos{0};
uint16_t current_left_up_lid_pos{0};
uint16_t current_left_down_lid_pos{0};

#define COSMETIC_DELAY 400
#define DELAY 15

long angleToPulse(uint16_t angle)
{
  return map(angle, 0, 180, SERVOMIN, SERVOMAX);
}

void setup()
{
  Serial.begin(9600);
  pwm.begin();
  pwm.setPWMFreq(60);

  delay(COSMETIC_DELAY);
  centerBothEyes();
  delay(COSMETIC_DELAY);
  openAllLids();

  delay(COSMETIC_DELAY);
  rightBothEyes(EYES_HORIZONTAL_RIGHT_MAX, 1);
  delay(COSMETIC_DELAY);
  leftBothEyes(EYES_HORIZONTAL_LEFT_MAX, 1);
  delay(COSMETIC_DELAY);
  rightBothEyes(EYES_HORIZONTAL_CENTER, 20);

  delay(COSMETIC_DELAY);
  rightLidsBlink(RIGHT_LIDS_POINT_OF_TOUCH);
  // yield();
}

void move(uint8_t part, uint16_t pos)
{
  pwm.setPWM(part, 0, angleToPulse(pos));
}

void centerBothEyes()
{
  move(EYES_horizontal, EYES_HORIZONTAL_CENTER);
  move(EYES_vertical, EYES_VERTICAL_CENTER);
  current_horizontal_eyes_pos = EYES_HORIZONTAL_CENTER;
  current_vertical_eyes_pos = EYES_VERTICAL_CENTER;
}

void openAllLids()
{
  move(LID_right_up, RIGHT_UP_LID_MAX);
  move(LID_right_down, RIGHT_DOWN_LID_MIN);
  current_right_up_lid_pos = RIGHT_UP_LID_MAX;
  current_right_down_lid_pos = RIGHT_DOWN_LID_MIN;

  move(LID_left_up, LEFT_UP_LID_MAX);
  move(LID_left_down, LEFT_DOWN_LID_MIN);
  current_left_up_lid_pos = LEFT_UP_LID_MAX;
  current_left_down_lid_pos = LEFT_DOWN_LID_MIN;
}

void rightBothEyes(uint16_t max_pos, uint16_t step_size)
{
  if (max_pos < 0 || step_size < 0)
  {
    return;
  }

  for (; current_horizontal_eyes_pos < max_pos; current_horizontal_eyes_pos += step_size)
  {
    move(EYES_horizontal, current_horizontal_eyes_pos);
    delay(DELAY);
  }
}

void leftBothEyes(uint16_t max_pos, uint16_t step_size)
{
  if (max_pos < 0 || step_size < 0)
  {
    return;
  }

  for (; current_horizontal_eyes_pos > max_pos; current_horizontal_eyes_pos -= step_size)
  {
    move(EYES_horizontal, current_horizontal_eyes_pos);
    delay(DELAY);
  }
}

void rightLidsBlink(uint16_t max_pos)
{
  for (; current_right_up_lid_pos <= RIGHT_UP_LID_MAX; current_right_up_lid_pos += 3)
  {
    move(LID_right_up, current_right_up_lid_pos);

    if (current_right_down_lid_pos <= RIGHT_DOWN_LID_MAX)
    {
      move(LID_right_down, ++current_right_down_lid_pos);
    }
  }
}

void loop()
{
}

/*
#define RIGHT_UP_LID_MAX 165
#define RIGHT_UP_LID_MIN 90
#define RIGHT_DOWN_LID_MAX 90
#define RIGHT_DOWN_LID_MIN 70
#define RIGHT_LIDS_POINT_OF_TOUCH 90
*/
