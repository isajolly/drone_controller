#include "MeAuriga.h"
#include <Wire.h>
#include <stdio.h>

#define ALLLEDS        0
// Auriga on-board light ring has 12 LEDs
#define LEDNUM  12
// on-board LED ring, at PORT0 (onboard)
MeRGBLed led( 0, LEDNUM );

float j, f, k;

MeGyro gyro_ext(0, 0x68); //external gryo sensor
MeGyro gyro(1, 0x69);
MeGyro gyro2(1, 0x69);

void setup()
{
  led.setpin( 44 );
  Serial.begin(115200);
  gyro.begin();
  gyro2.begin();
  Serial.print("X:");
  Serial.print(gyro.getAngleX() );
  Serial.print(" Y:");
  Serial.print(gyro.getAngleY() );
  Serial.print(" Z:");
  Serial.println(gyro.getAngleZ() );
}

void loop()
{
  gyro.fast_update();
  gyro2.update();
  color_loop();
  Serial.read();
  Serial.print("X:");
  Serial.print(gyro.getAngleX() );
  Serial.print(" Y:");
  Serial.print(gyro.getAngleY() );
  Serial.print(" Z:");
  Serial.println(gyro.getAngleZ() );
  Serial.print("X2:");
  Serial.print(gyro.getAngleX() );
  Serial.print(" Y2:");
  Serial.print(gyro.getAngleY() );
  Serial.print(" Z2:");
  Serial.println(gyro.getAngleZ() );
  delay(10);
}

void color_loop()
{
  float a = max(min(100 + 20 * gyro.getAngleX(), 255), 0);
  float b = max(min(100 - 20 * gyro.getAngleX(), 255), 0);
  float c = max(min(100 + 20 * gyro.getAngleY(), 255), 0);
  float d = max(min(100 - 20 * gyro.getAngleY(), 255), 0);

  led.setColorAt( 1, a, b, 30 );
  led.setColorAt( 2, a - 20, b + 20, 30 );
  led.setColorAt( 12, a + 20, b - 20, 30 );
  led.setColorAt( 3, c - 20, d + 20, 30 );
  led.setColorAt( 4, c, d, 30 );
  led.setColorAt( 5, c + 20, d - 20, 30 );
  led.setColorAt( 6, b - 20, a + 20, 30 );
  led.setColorAt( 7, b, a, 30 );
  led.setColorAt( 8, b + 20, a - 20, 30 );
  led.setColorAt( 9, d - 20, c + 20, 30 );
  led.setColorAt( 10, d, c, 30 );
  led.setColorAt( 11, d + 20, c - 20, 30 );


  //  for (uint8_t t = 0; t < LEDNUM; t++ )
  //  {
  //    uint8_t red = 64 * (1 + sin(t / 2.0 + j / 4.0) );
  //    uint8_t green = 64 * (1 + sin(t / 1.0 + f / 9.0 + 2.1) );
  //    uint8_t blue = 64 * (1 + sin(t / 3.0 + k / 14.0 + 4.2) );
  //    led.setColorAt( t, red, green, blue );
  //  }
  led.show();

}
