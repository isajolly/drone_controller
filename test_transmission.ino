#include "MeAuriga.h"
#include <Wire.h>


#define CHANNEL1 0
#define CHANNEL2 1
#define CHANNEL3 2
#define CHANNEL4 3
 
#define YAW      3
#define PITCH    1
#define ROLL     0
#define THROTTLE 2



 
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




/**
* Setup routine.
*
* @return void
*/

void setup()

{

    Serial.begin(9600);

    // Customize mapping of controls: set here which command is on wich channel.

    mode_mapping[YAW]      = CHANNEL4;
    mode_mapping[PITCH]    = CHANNEL2;
    mode_mapping[ROLL]     = CHANNEL1;
    mode_mapping[THROTTLE] = CHANNEL3;

    DDRH |=B01111000;//output port PH5, PH6, PH3 and PH4
    DDRK &= B00111001;//input port PK2,PK7,PK1,PK6

    PCICR |= B00000100; // group PCIE2=1 (PCINT23..16) 17,18,22,23

    PCMSK2 |= (1 << PCINT17); //Set PCINT17 (digital input A9, PK1) to trigger an interrupt on state change.
    PCMSK2 |= (1 << PCINT18); //Set PCINT18 (digital input A10, PK2)to trigger an interrupt on state change.
    PCMSK2 |= (1 << PCINT22); //Set PCINT22 (digital input A14, PK6)to trigger an interrupt on state change.
    PCMSK2 |= (1 << PCINT23); //Set PCINT23 (digital input A15, PK7)to trigger an interrupt on state change.
   
    first=1;
    difference_init=0;
    now = micros();

}

/**
* Main loop routine.
*
* @return void
*/

void loop()

{
  now=micros();
  now_init = now;
  
  
  if (first) {loop_timer_init=now_init;first=0;}
  
  difference_init = now_init - loop_timer_init;
  
  if (difference_init<10000){
     for(int i=0;i<1000;i++){
        PORTH |= B11111000; // activates pins #3 #4 #5 #6 (and 7 the blue light) of port D (HIGH state)
        delay(1);
        PORTH &= B10000111; // deactivates for the rest of the cycle
        delay(3);
        }
     }

  //Plotting the pulse length received on each channel. To display, go to Tools / Serial Plotter
  plotterReceiver();
 
    while(micros()-now_init<7000);

}


 

void plotterReceiver(){
  Serial.print(pulse_duration[CHANNEL1]);
  Serial.print(", ");
  Serial.print(pulse_duration[CHANNEL2]);
  Serial.print(", ");
  Serial.print(pulse_duration[CHANNEL3]);
  Serial.print(", ");
  Serial.print(pulse_duration[CHANNEL4]);
  Serial.println();
}


/**
* This Interrupt Sub Routine is called each time input 8, 9, 10 or 11 changed state.
* Read the receiver signals in order to get flight instructions.
*
* This routine must be as fast as possible to prevent main program to be messed up.
* The trick here is to use port registers to read pin state.
* Doing (PINB & B00000001) is the same as digitalRead(8) with the advantage of using less CPU loops.
* It is less conveniant but more efficient, which is the most important here.
*
* @see https://www.arduino.cc/en/Reference/PortManipulation
*/
 
 
ISR(PCINT2_vect)
{
    current_time = micros();
 
    // Channel 1 -------------------------------------------------
    if (PINK & B00000010) {                                        // Is input A9, PK1 high ?
        if (previous_state[CHANNEL1] == LOW) {                     // Input A9 changed from 0 to 1 (rising edge)
            previous_state[CHANNEL1] = HIGH;                       // Save current state
            timer[CHANNEL1]          = current_time;               // Start timer
        }
    } else if(previous_state[CHANNEL1] == HIGH) {                  // Input A9, PK1 changed from 1 to 0 (falling edge)
        previous_state[CHANNEL1] = LOW;                            // Save current state
        pulse_duration[CHANNEL1] = current_time - timer[CHANNEL1]; // Stop timer & calculate pulse duration
        //Serial.println("pulse_duration[CHANNEL1]");Serial.println(pulse_duration[CHANNEL1]);
    }
 
    // Channel 2 -------------------------------------------------
    if (PINK & B00000100) {                                        // Is input A10, PK2 high ?
        if (previous_state[CHANNEL2] == LOW) {                     // Input 10 changed from 0 to 1 (rising edge)
            previous_state[CHANNEL2] = HIGH;                       // Save current state
            timer[CHANNEL2]          = current_time;               // Start timer
        }
    } else if(previous_state[CHANNEL2] == HIGH) {                  // Input A10 changed from 1 to 0 (falling edge)
        previous_state[CHANNEL2] = LOW;                            // Save current state
        pulse_duration[CHANNEL2] = current_time - timer[CHANNEL2]; // Stop timer & calculate pulse duration
        //Serial.println("pulse_duration[CHANNEL2]");Serial.println(pulse_duration[CHANNEL2]);
    }
 
    // Channel 3 -------------------------------------------------
    if (PINK & B01000000) {                                        // Is input A14, PK6 high ?
        if (previous_state[CHANNEL3] == LOW) {                     // Input A14 changed from 0 to 1 (rising edge)
            previous_state[CHANNEL3] = HIGH;                       // Save current state
            timer[CHANNEL3]          = current_time;               // Start timer
        }
    } else if(previous_state[CHANNEL3] == HIGH) {                  // Input A14 changed from 1 to 0 (falling edge)
        previous_state[CHANNEL3] = LOW;                            // Save current state
        pulse_duration[CHANNEL3] = current_time - timer[CHANNEL3]; // Stop timer & calculate pulse duration
        //Serial.println("pulse_duration[CHANNEL3]");Serial.println(pulse_duration[CHANNEL3]);
    }
 
    // Channel 4 -------------------------------------------------
    if (PINK & B10000000) {                                        // Is input A15, PK7 high ?
        if (previous_state[CHANNEL4] == LOW) {                     // Input 15 changed from 0 to 1 (rising edge)
            previous_state[CHANNEL4] = HIGH;                       // Save current state
            timer[CHANNEL4]          = current_time;               // Start timer
        }
    } else if(previous_state[CHANNEL4] == HIGH) {                  // Input 15 changed from 1 to 0 (falling edge)
        previous_state[CHANNEL4] = LOW;                            // Save current state
        pulse_duration[CHANNEL4] = current_time - timer[CHANNEL4]; // Stop timer & calculate pulse duration
        //Serial.println("pulse_duration[CHANNEL4]");Serial.println(pulse_duration[CHANNEL4]);
    }
}
 
