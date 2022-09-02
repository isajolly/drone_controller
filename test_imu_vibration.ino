#include "MeAuriga.h"
#include <Wire.h>
#include <stdio.h>

#define ALLLEDS        0
// Auriga on-board light ring has 12 LEDs
#define LEDNUM  12
// on-board LED ring, at PORT0 (onboard)
MeRGBLed led( 0, LEDNUM );

float x,y;

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

  Serial.print(gyro.getAngleX() );
  Serial.print(",");
  Serial.print(gyro.getAngleY() );
  Serial.print(",");
  Serial.print(gyro.getAngleZ() );
  Serial.println();

  //delay(10);
}

void color_loop()
{
  x = gyro.getAngleX();
  y= gyro.getAngleY();

 /* if (fabs(x)<3. && fabs(y)<3.){no_color(); }
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
  }*/
  
  if (fabs(x)<3. && fabs(y)<3.){no_color(); }
  else if (x>0.){
    if(y>0.){
      if(x/fabs(y)>0.75){color_fill(2);}
      else if(x/fabs(y)>0.5){color_fill(3);}
      else if(x/fabs(y)>0.25){color_fill(4);}
      else{color_fill(5);}
    }
    else{
      if(x/fabs(y)>0.75){color_fill(2);}
      else if(x/fabs(y)>0.5){color_fill(1);}
      else if(x/fabs(y)>0.25){color_fill(0);}
      else{color_fill(11);}
    }
  }
  else{
    if(y>0.){
      if(fabs(x)/fabs(y)>0.75){color_fill(8);}
      else if(fabs(x)/fabs(y)>0.5){color_fill(7);}
      else if(fabs(x)/fabs(y)>0.25){color_fill(6);}
      else{color_fill(5);}
    }
    else{
      if(fabs(x)/fabs(y)>0.75){color_fill(8);}
      else if(fabs(x)/fabs(y)>0.5){color_fill(9);}
      else if(fabs(x)/fabs(y)>0.25){color_fill(10);}
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
    if(t==place){ led.setColorAt( t, min(int(fabs(x)+fabs(y)),255),0,max(60-int(fabs(x)+fabs(y)),0) );}
    else{ led.setColorAt( t, 0,0,0 );}
   }
}
