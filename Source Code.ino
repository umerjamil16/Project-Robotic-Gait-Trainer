#include "MPU6050.h"
#include "I2Cdev.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif
MPU6050 accelgyro;
int16_t ax, ay, az; //accelerometer values
int16_t gx, gy, gz; //gyro values
#define OUTPUT_READABLE_ACCELGYRO

#define LED_PIN 13
bool blinkState = false;

int BluetoothData;// the data given from Computer
int gyro_var;
int mode_select = 0;
int pwm;
int topple = 3500;
unsigned long button_time = 0;
unsigned long last_button_time = 0;

void setup()
{
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  Serial.begin(38400);

  Serial.println("Initializing I2C devices...");
  accelgyro.initialize();

  Serial.println("Testing device connections...");
  Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

  pinMode(LED_PIN, OUTPUT);
  pinMode(4, INPUT);

  pinMode(7, OUTPUT); //left pwm
  pinMode(6, OUTPUT); //
  pinMode(5, OUTPUT); //

  pinMode(2, OUTPUT); //right pwm
  pinMode(53, OUTPUT); //
  pinMode(51, OUTPUT); //

  pinMode(50, OUTPUT);
  pinMode(48, OUTPUT);
  pinMode(46, OUTPUT);


  pinMode(3, INPUT);
  digitalWrite(3, HIGH);
  attachInterrupt(digitalPinToInterrupt(3), mode, FALLING);
  Serial2.begin(9600);

}

void loop()
{

  if (mode_select == 0)
  {

    digitalWrite(50, HIGH);
    digitalWrite(48, LOW);
    digitalWrite(46, LOW);
    while (digitalRead(3))
    {
      pwm = map(analogRead(A8), 0, 1024, 0, 255);

     
      BluetoothData =    Serial2.read();

      if (BluetoothData == 1) //reverse
      {
        forward();
      }

      else if (BluetoothData == 2) //reverse
      {
        reverse();
      }

      else if (BluetoothData == 4)  //left
      { // if number 0 pressed ....
        left();
      }
      else if (BluetoothData == 3) //right
      { // if number 0 pressed ....
        right();
      }
      else if (BluetoothData == 5) //stop
      { // if number 0 pressed ....
        motor_stop();
      }

    }
  }


  else if (mode_select == 1)
  {

    digitalWrite(50, LOW);
    digitalWrite(48, HIGH);
    digitalWrite(46, LOW);

    while (digitalRead(3))
    {
      pwm = map(analogRead(A8), 0, 1024, 0, 255);
      /*JOYSTIC MODE*/
      int jc_var_p = analogRead(A1);
      int jc_var_s = analogRead(A0);

      while (1)
      {

        accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
        if (ay < topple)
        {
          motor_stop();
          break;
        }
        forward_topple();
      }

      if (jc_var_p < 255)
      {
        analogWrite(7, pwm);
        digitalWrite(6, LOW);
        digitalWrite(5, HIGH);

        analogWrite(2, pwm);
        digitalWrite(53, LOW);
        digitalWrite(51, HIGH);
      }

      else if (jc_var_p > 700)
      {
        analogWrite(7, pwm);
        digitalWrite(6, HIGH);
        digitalWrite(5, LOW);

        analogWrite(2, pwm);
        digitalWrite(53, HIGH);
        digitalWrite(51, LOW);
      }

      else if (jc_var_s > 700)
      {
        analogWrite(7, pwm);
        digitalWrite(6, LOW);
        digitalWrite(5, HIGH);

        analogWrite(2, pwm);
        digitalWrite(53, HIGH);
        digitalWrite(51, LOW);
      }

      else if (jc_var_s < 255)
      {
        analogWrite(7, pwm);
        digitalWrite(6, HIGH);
        digitalWrite(5, LOW);

        analogWrite(2, pwm);
        digitalWrite(53, LOW);
        digitalWrite(51, HIGH);
      }
      else
        motor_stop();
    }
  }

  else if (mode_select == 2)
  {
    /*BCI Control*/

    digitalWrite(50, LOW);
    digitalWrite(48, LOW);
    digitalWrite(46, HIGH);

    while (digitalRead(3))
    {
      while (1)
      {

        accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
        if (ay < topple)
        {
          motor_stop();
          break;
        }
        forward_topple();
      }
      motor_stop();

    }
  }

}


void forward_topple()
{
  analogWrite(7, 255);
  digitalWrite(6, LOW);
  digitalWrite(5, HIGH);

  analogWrite(2, 255);
  digitalWrite(53, LOW);
  digitalWrite(51, HIGH);
}


void forward()
{
  analogWrite(7, pwm);
  digitalWrite(6, LOW);
  digitalWrite(5, HIGH);

  analogWrite(2, pwm);
  digitalWrite(53, LOW);
  digitalWrite(51, HIGH);
}



void reverse()
{ analogWrite(7, pwm);
  digitalWrite(6, HIGH);
  digitalWrite(5, LOW);

  analogWrite(2, pwm);
  digitalWrite(53, HIGH);
  digitalWrite(51, LOW);

}

void right()
{
  analogWrite(7, pwm);
  digitalWrite(6, LOW);
  digitalWrite(5, HIGH);

  analogWrite(2, pwm);
  digitalWrite(53, HIGH);
  digitalWrite(51, LOW);
}

void left()
{
  analogWrite(7, pwm);
  digitalWrite(6, HIGH);
  digitalWrite(5, LOW);

  analogWrite(2, pwm);
  digitalWrite(53, LOW);
  digitalWrite(51, HIGH);
}

void motor_stop()
{
  analogWrite(7, 0);
  digitalWrite(6, LOW);
  digitalWrite(5, HIGH);

  analogWrite(2, 0);
  digitalWrite(51, HIGH);
  digitalWrite(49, LOW);
}

void mode()
{
  button_time = millis();
  //check to see if increment() was called in the last 250 milliseconds
  if (button_time - last_button_time > 250)
  {
    mode_select++;
    mode_select = mode_select % 3;
    last_button_time = button_time;
  }
}








