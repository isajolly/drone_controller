#include "MeAuriga.h"
#include <Wire.h>

#define ALLLEDS        0
#define LEDNUM  12

#define CHANNEL1 0
#define CHANNEL2 1
#define CHANNEL3 2
#define CHANNEL4 3
 
#define YAW      3
#define PITCH    1
#define ROLL     0
#define THROTTLE 2

#define LOOP_PERIOD 6000   //time spent in 1 loop in microseconds

#define MANUAL_MODE 1   // 1 to go into manual mode and 0 to use angular corrections

#define COMMAND_REDUCTION_COEF  3.  //reduces the intensity of the command on roll and pitch and yaw received from the transmitter

#define REDUCE_MOTOR_A  0.87//1.45
#define REDUCE_MOTOR_B  0.65//0.65
#define REDUCE_MOTOR_C  1//0.65
#define REDUCE_MOTOR_D  0.85//0.03

#define IDLE_A  1250
#define IDLE_B  1150
#define IDLE_C  1200
#define IDLE_D  1200

#define COEF_THROTTLE   1.
#define COEF_ROLL 1.
#define COEF_PITCH 1.
#define COEF_YAW   1.

#define USE_PITCH_ROLL 1 // 0 or 1

/*Relation of PD coefficients to other coefficients :
      k0 = 2 //cf complementary filter
      ts = 3 / k0  //settling time
      w0 = 4 / (m*ts)  //natural frequency
      m = 0.7 //damping ratio
      Kp = -w0^2 * I
      Kd = -2 * m * w0 * I
*/

#define COEF_KP   -12.//-9. //0.47//-0.39//-7.9184 //-0.15     //coefficients are proportional to I (inertia) so they should be modified proportionally
#define COEF_KD   -3.//-5. //0.52//-0.43//-1.94   //-0.04     //otherwise they can be modified by changing w0 and w0^2

#define COEF_KD_YAW  30.


#define sgn(x) ((x) < 0 ? -1 : 1 )

MeGyro gyro(1, 0x69);
MeRGBLed led( 0, LEDNUM );
 
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

//PID variables
float roll_pulse_length_pid, pitch_pulse_length_pid, yaw_pulse_length_pid;
float angles[4];
float previous_angles[4];
double error_pulse_length[4];
double previous_error_pulse_length[4] = {0.,0.,0.,0.};
float uT2;
float w1, w1Square, w2, w2Square, w3, w3Square, w4, w4Square, Throttle_PID, Roll_PID, Pitch_PID, Yaw_PID;
float foo;

unsigned long previous_time;
volatile float delta_t;

//Display variables
int passage;



/**
* Setup routine.
*
* @return void
*/

void setup()

{

    Serial.begin(9600);
    gyro.begin();
    led.setpin( 44 );

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

    roll_pulse_length=0;
    pitch_pulse_length=0;
    throttle_pulse_length=0;
    yaw_pulse_length=0;

}

/**
* Main loop routine.
*
* @return void
*/

void loop()

{
now=micros();
//Optional to check that a cycle is under a certain amount of micros seconds. Can be used to see the end of set up.
if(now-now_init<=LOOP_PERIOD+30){led.setColorAt( 3, 0, 255, 0 );led.setColorAt( 2, 0, 0, 0 );}
if(now-now_init>LOOP_PERIOD+30){led.setColorAt( 3, 0, 0, 0 );led.setColorAt( 2, 255, 0, 0 );}
led.show();
//Serial.println(now-now_init);
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

  // Adjusting pulse to scale
  roll_pulse_length=pulse_duration[mode_mapping[ROLL]]-1500;
  pitch_pulse_length=pulse_duration[mode_mapping[PITCH]]-1500;
  yaw_pulse_length=pulse_duration[mode_mapping[YAW]]-1500;
  throttle_pulse_length=pulse_duration[mode_mapping[THROTTLE]]-1000;

  //AUTOMATIC MODE
  if (!MANUAL_MODE && throttle_pulse_length >= 100){
  //if(1){        //use this test instead of the previous one to check on corrections with no throttle power.
    //memorize previous angles
    previous_angles[YAW]=angles[YAW];
    previous_angles[PITCH]=angles[PITCH];
    previous_angles[ROLL]=angles[ROLL];

    gyro.update();
    angles[YAW]=gyro.getAngleZ();
    angles[PITCH]=gyro.getAngleX();
    angles[ROLL]=gyro.getAngleY();
    //angleDisplay(30);
    //plotterAngle();
    
    //memorize time difference in between this measure and the last in seconds
    now = micros();
    delta_t=(now-previous_time)*1e-6;
    previous_time=now;

    
    //Compute pid
    
    error_pulse_length[ROLL] =  angles[ROLL]*25. - 0.*roll_pulse_length ;
    roll_pulse_length_pid = error_pulse_length[ROLL] * COEF_KP + ( error_pulse_length[ROLL]- previous_error_pulse_length[ROLL] )/ delta_t * COEF_KD ;
    roll_pulse_length_pid = -roll_pulse_length_pid;
    //roll_pulse_length_pid = minMax(roll_pulse_length_pid, -400 , 400)/COMMAND_REDUCTION_COEF;
    //errorDisplay(1,ROLL,"roll");

    error_pulse_length[PITCH] = angles[PITCH]*25. - 0.*pitch_pulse_length ;
    pitch_pulse_length_pid = error_pulse_length[PITCH] * COEF_KP + ( error_pulse_length[PITCH]- previous_error_pulse_length[PITCH] )/ delta_t * COEF_KD ;
    //pitch_pulse_length_pid = minMax(pitch_pulse_length_pid, -400, 400)/COMMAND_REDUCTION_COEF;
    //errorDisplay(80,PITCH,"pitch");
    

    error_pulse_length[YAW] = angles[YAW]*25. - 0.*yaw_pulse_length ;
    yaw_pulse_length_pid = 0.*error_pulse_length[YAW] * COEF_KP + ( error_pulse_length[YAW]- previous_error_pulse_length[YAW] ) / delta_t * COEF_KD_YAW ;
    //errorDisplay(80,YAW,"yaw");

    previous_error_pulse_length[ROLL] = error_pulse_length[ROLL];
    previous_error_pulse_length[PITCH] = error_pulse_length[PITCH];
    previous_error_pulse_length[YAW] = error_pulse_length[YAW];

//------------------
if (1) {

    uT2 = (throttle_pulse_length/4.)*(throttle_pulse_length/4.);

    w1Square = uT2 + (yaw_pulse_length_pid + USE_PITCH_ROLL*(-roll_pulse_length_pid -pitch_pulse_length_pid))/COMMAND_REDUCTION_COEF;
    w2Square = uT2 + (-yaw_pulse_length_pid + USE_PITCH_ROLL*(roll_pulse_length_pid -pitch_pulse_length_pid))/COMMAND_REDUCTION_COEF;
    w3Square = uT2 + (-yaw_pulse_length_pid + USE_PITCH_ROLL*(-roll_pulse_length_pid +pitch_pulse_length_pid))/COMMAND_REDUCTION_COEF;
    w4Square = uT2 + (yaw_pulse_length_pid + USE_PITCH_ROLL*(roll_pulse_length_pid +pitch_pulse_length_pid))/COMMAND_REDUCTION_COEF;

    if (0) {
      w1 = sqrt(fabs(w1Square))*sgn(w1Square);
      w2 = sqrt(fabs(w2Square))*sgn(w2Square);
      w3 = sqrt(fabs(w3Square))*sgn(w3Square);
      w4 = sqrt(fabs(w4Square))*sgn(w4Square);
    } else {
      w1 = (w1Square > 0 ? sqrt(w1Square) : 0.);
      w2 = (w2Square > 0 ? sqrt(w2Square) : 0.);
      w3 = (w3Square > 0 ? sqrt(w3Square) : 0.);
      w4 = (w4Square > 0 ? sqrt(w4Square) : 0.);
    }

    Throttle_PID = w1+w2+w3+w4;
    Yaw_PID =  w1-w2-w3+w4;
    Roll_PID = -w1+w2-w3+w4;  
    Pitch_PID = -w1-w2+w3+w4;
    
  } else {
//------------------

  Throttle_PID = throttle_pulse_length;
    Yaw_PID =  yaw_pulse_length_pid;
    Roll_PID = roll_pulse_length_pid;
    Pitch_PID = pitch_pulse_length_pid;
    
}
  

    throttle_pulse_length = sgn(Throttle_PID)*round(fabs(Throttle_PID));

    roll_pulse_length = sgn(Roll_PID)*round(fabs(Roll_PID));
    pitch_pulse_length= sgn(Pitch_PID)*round(fabs(Pitch_PID));
    yaw_pulse_length= sgn(Yaw_PID)*round(fabs(Yaw_PID));
    
    if(0) {
        foo = pitch_pulse_length;
        pitch_pulse_length = roll_pulse_length;
        roll_pulse_length = foo;
    }
    
  //motor calibration
  throttle_pulse_length=minMax(throttle_pulse_length,0,1000);
  //roll_pulse_length=minMax(roll_pulse_length,-300,300);
  //pitch_pulse_length=minMax(pitch_pulse_length,-300,300);
  //yaw_pulse_length=minMax(yaw_pulse_length,-300,300);

  //pidDisplay(10);
  //plotterPid();
    
  pulse_length_escA = IDLE_A + REDUCE_MOTOR_A*( COEF_THROTTLE*throttle_pulse_length + COEF_ROLL*roll_pulse_length - COEF_PITCH*pitch_pulse_length + COEF_YAW*yaw_pulse_length );
  
  pulse_length_escB = IDLE_B + REDUCE_MOTOR_B*( COEF_THROTTLE*throttle_pulse_length - COEF_ROLL*roll_pulse_length - COEF_PITCH*pitch_pulse_length - COEF_YAW*yaw_pulse_length );
  
  pulse_length_escC = IDLE_C + REDUCE_MOTOR_C*( COEF_THROTTLE*throttle_pulse_length + COEF_ROLL*roll_pulse_length + COEF_PITCH*pitch_pulse_length - COEF_YAW*yaw_pulse_length );
  
  pulse_length_escD = IDLE_D + REDUCE_MOTOR_D*( COEF_THROTTLE*throttle_pulse_length - COEF_ROLL*roll_pulse_length + COEF_PITCH*pitch_pulse_length + COEF_YAW*yaw_pulse_length );

//plotterPid();


  }
  else {
  //MANUAL MODE
  //if(MANUAL_MODE || throttle_pulse_length < 50){
    calcul=roll_pulse_length/COMMAND_REDUCTION_COEF;
    roll_pulse_length=sgn(calcul) * round(fabs(calcul));
    calcul=pitch_pulse_length/COMMAND_REDUCTION_COEF;
    pitch_pulse_length=sgn(calcul) * round(fabs(calcul));
    calcul=yaw_pulse_length/COMMAND_REDUCTION_COEF;
    yaw_pulse_length=sgn(calcul) * round(fabs(calcul));
    
    roll_pulse_length=0;    //OPTIONAL use to
    pitch_pulse_length=0;   //prevent mishaps
    yaw_pulse_length=0;     //before take-off

    if (0) {
        roll_pulse_length = -throttle_pulse_length;
        throttle_pulse_length = 0;
    }

    //motor calibration
    
    pulse_length_escA = 1000 + REDUCE_MOTOR_A*( COEF_THROTTLE*throttle_pulse_length + COEF_ROLL*roll_pulse_length - COEF_PITCH*pitch_pulse_length + COEF_YAW*yaw_pulse_length );
    
    pulse_length_escB = 1000 + REDUCE_MOTOR_B*( COEF_THROTTLE*throttle_pulse_length - COEF_ROLL*roll_pulse_length - COEF_PITCH*pitch_pulse_length - COEF_YAW*yaw_pulse_length );
    
    pulse_length_escC = 1000 + REDUCE_MOTOR_C*( COEF_THROTTLE*throttle_pulse_length + COEF_ROLL*roll_pulse_length + COEF_PITCH*pitch_pulse_length - COEF_YAW*yaw_pulse_length );
    
    pulse_length_escD = 1000 + REDUCE_MOTOR_D*( COEF_THROTTLE*throttle_pulse_length - COEF_ROLL*roll_pulse_length + COEF_PITCH*pitch_pulse_length + COEF_YAW*yaw_pulse_length );
  
    
      plotterAngle();

  }

  //motor limitation

  pulse_length_escA=minMax(pulse_length_escA,1000,2000);
  pulse_length_escB=minMax(pulse_length_escB,1000,2000);
  pulse_length_escC=minMax(pulse_length_escC,1000,2000);
  pulse_length_escD=minMax(pulse_length_escD,1000,2000);
  //motorDisplay();

 
   //Sending pulses to ESCs
   //Impulse sent every 6000µs if not more
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

//Debuguing functions :

void plotterPid(){
  Serial.print(throttle_pulse_length);
  Serial.print(", ");
  Serial.print(roll_pulse_length);
  Serial.print(", ");
  Serial.print(pitch_pulse_length);
  Serial.print(", ");
  Serial.print(yaw_pulse_length);
  Serial.println();
}

void plotterAngle()
{
  gyro.update();

  Serial.print(gyro.getAngleX() );
  Serial.print(",");
  Serial.print(gyro.getAngleY() );
  Serial.print(",");
  Serial.print(gyro.getAngleZ() );
  Serial.println();

  //delay(50);
}

void angleDisplay(int frequency) {
  if (passage==frequency){
    Serial.println("\n---------------");
    Serial.println("roll angle : ");  Serial.println(angles[ROLL]);
    Serial.println("pitch angle : "); Serial.println(angles[PITCH]);
    Serial.println("yaw angle : ");   Serial.println(angles[YAW]);
    Serial.print("time elapsed : ");  Serial.print(difference_init/1000);Serial.print("ms");
    passage=0;
  }else{passage+=1;}
}

void pidDisplay(int frequency) {
  if (passage==frequency){
    Serial.println("\n---------------");
    Serial.println("roll_pulse_length : ");  Serial.println(roll_pulse_length);
    Serial.println("pitch_pulse_length : "); Serial.println(pitch_pulse_length);
    Serial.println("yaw_pulse_length : ");   Serial.println(yaw_pulse_length);
    Serial.print("throttle_pulse_length : ");  Serial.print(throttle_pulse_length);
    passage=0;
  }else{passage+=1;}
}

void errorDisplay(int frequency, int angle, String name_angle) {
  if (passage==frequency){
    Serial.println("\n---------------");
    Serial.println("erreur :");     Serial.println((error_pulse_length[angle]- previous_error_pulse_length[angle] )/ delta_t * COEF_KD);
    Serial.println(name_angle);     Serial.println(error_pulse_length[angle]);
    Serial.print("previous");Serial.print(name_angle); Serial.println(previous_error_pulse_length[angle]);
    Serial.println("delta_t");      Serial.println(delta_t);
    passage=0;
  }else {passage+=1;}
}

void motorDisplay(){
  Serial.println("pulse_length_escA :"); Serial.println(pulse_length_escA);
  Serial.println("pulse_length_escB :"); Serial.println(pulse_length_escB);
  Serial.println("pulse_length_escC :"); Serial.println(pulse_length_escC);
  Serial.println("pulse_length_escD :"); Serial.println(pulse_length_escD);
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
 