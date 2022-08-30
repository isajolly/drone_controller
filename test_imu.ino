#include "MeAuriga.h"
#include <Wire.h>
#include <stdio.h>

#define ALLLEDS        0
// Auriga on-board light ring has 12 LEDs
#define LEDNUM  12
// on-board LED ring, at PORT0 (onboard)
MeRGBLed led( 0, LEDNUM );

float x,y;
int passage=0;

MeGyro gyro(1, 0x69);

void setup()
{
  led.setpin( 44 );
  Serial.begin(9600);
  gyro.begin();


}

void loop()
{
  gyro.update();
  color_loop();

    if (passage==80){
      Serial.println("-----------");
      Serial.print("X:");
      Serial.println(gyro.getAngleX() );
      Serial.print(" Y:");
      Serial.println(gyro.getAngleY() );
      Serial.print(" Z:");
      Serial.println(gyro.getAngleZ() );
      passage=0;
    }else {passage+=1;}

  delay(10);
}

void color_loop()
{
  x = gyro.getAngleX();
  y= gyro.getAngleY();

  if (fabs(x)<3. && fabs(y)<3.){no_color(); }
  else if (x>0.){
    if(y>0.){
      if(x>y){color_fill(2);}
      else{color_fill(5);}
    }
    else{
      if(x>fabs(y)){color_fill(2);}
      else{color_fill(11);}
    }
  }
  else{
    if(y>0.){
      if(fabs(x)>y){color_fill(8);}
      else{color_fill(5);}
    }
    else{
      if(fabs(x)>fabs(y)){color_fill(8);}
      else{color_fill(11);}
    }
  }
  


  led.show();

}

void no_color(){
   for (uint8_t t = 0; t < LEDNUM; t++ )
   {
     led.setColorAt( t, 0,0,0 );
   }
}

void color_fill(int place){
  for (uint8_t t = 0; t < LEDNUM; t++ )
   {
    if(t==place){ led.setColorAt( t, 200,0,50 );}
    else{ led.setColorAt( t, 0,0,0 );}
   }
}
