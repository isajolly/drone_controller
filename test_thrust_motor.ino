/*
 * This code allows you to take measures on the test bench for one motor and one ESC
 * Connect ESC to connector for motor A
 */


#include "MeAuriga.h"
#include <Wire.h>


#define LOOP_PERIOD 6000   //time spent in 1 loop in microseconds


#define sgn(x) ((x) < 0 ? -1 : 1 )

 
volatile unsigned long current_time;
unsigned long now, now_init, attente;
unsigned long loop_timer,loop_timer_init;
unsigned long difference,difference_init;
volatile unsigned long timer[4];

int throttle_pulse_length, roll_pulse_length, pitch_pulse_length, yaw_pulse_length;
float calcul;
unsigned int  pulse_length_escA, pulse_length_escB, pulse_length_escC, pulse_length_escD;
int first;

// Previous state of each channel (HIGH or LOW)
volatile byte previous_state[4];
unsigned int testA,testB,testC,testD;

// Duration of the pulse on each channel in µs (must be within 1000µs & 2000µs)
volatile int pulse_duration[4] = {1000, 1000, 1000, 1000};
int mode_mapping[4];


char data;



/**
* Setup routine.
*
* @return void
*/

void setup()

{

    Serial.begin(9600);


    DDRH |=B01111000;//output port PH5, PH6, PH3 and PH4
    DDRK &= B00111001;//input port PK2,PK7,PK1,PK6

    PCICR |= B00000100; // group PCIE2=1 (PCINT23..16) 17,18,22,23

    first=1;
    difference_init=0;
    now = micros();

    Serial.println("Starting test...");
    Serial.println("Send 0 to stop the test");
    Serial.println("Send 1 to increase command by 10");

}

/**
* Main loop routine.
*
* @return void
*/

void loop()

{
  now_init = micros();
  
  
  if (first) {loop_timer_init=now_init;first=0;}
  
  difference_init = now_init - loop_timer_init;
  
  if (difference_init<1000){
     for(int i=0;i<100;i++){
        PORTH |= B11111000; // activates pins #3 #4 #5 #6 (and 7 the blue light) of port D (HIGH state)
        delay(1);
        PORTH &= B10000111; // deactivates for the rest of the cycle
        delay(3);
        }
     }

  if (Serial.available()) {
        data = Serial.read();

        switch (data) {
          // 0
          case 48 : Serial.println("Sending 1000 throttle");
            pulse_length_escA = 1000;
            break;
          //1
          case 49 : Serial.print("Sending ");Serial.print(pulse_length_escA+10);Serial.println(" throttle");
            pulse_length_escA += 10;
            break;

        }
  }
  

  //motor limitation

  pulse_length_escA=minMax(pulse_length_escA,1000,2000);
 
   //Sending pulses to ESCs

   now = micros();
   loop_timer = now;

  // activates pins #3 #4 #5 #6 (and 7 the blue light) of port D (HIGH state)
   PORTH |= B11111000;
   testA=testB=testC=testD=0;

  //while (PORTH >=191) {

  while ((testA!=1)|(testB!=1)|(testC!=1)|(testD!=1)) {

        now = micros();
        difference = now - loop_timer;

        if ((difference >= pulse_length_escA)&&(!testA))  {PORTH &= B11011111;testA=1;} // Passe la broche #H5 à LOW PORTH &= B11011111;Serial.println("A");
        if ((difference >= pulse_length_escB)&&(!testB))  {PORTH &= B10111111;testB=1;} // Passe la broche #H6 à LOW PORTH &= B10111111;Serial.println("B");
        if ((difference >= pulse_length_escC)&&(!testC))  {PORTH &= B11110111;testC=1;} // Passe la broche #H3 à LOW PORTH &= B11110111;Serial.println("C");
        if ((difference >= pulse_length_escD)&&(!testD))  {PORTH &= B11101111;testD=1;} // Passe la broche #H4 à LOW PORTH &= B11101111;Serial.println("D");
    }

    //PORTH &= B01111111;

    while(micros()-now_init<LOOP_PERIOD);
    //Serial.println(micros()-now_init);
}


 
/**
* Minmax computing
*
* @param float a  Variable to be compared
* @param float b  Lower boundary
* @param float c  Higher boundary
* @return float
*/

float minMax ( float a , float b , float c ) {
  float result = min (a, c);
  result = max(result,b);
  return result;
}
