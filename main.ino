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


#define COMMAND_REDUCTION_COEF  5  //reduces the intensity of the command on roll and pitch and yaw received from the transmitter

#define REDUCE_MOTOR_A  1
#define REDUCE_MOTOR_B  0.93
#define REDUCE_MOTOR_C  1
#define REDUCE_MOTOR_D  0.95

#define COEF_THROTTLE   1
#define COEF_PITCH_ROLL 1
#define COEF_YAW        1

#define COEF_KP 1
#define COEF_KD 1

MeGyro gyro(1, 0x69);
 
volatile unsigned long current_time;
unsigned long now, now_init, attente;
unsigned long loop_timer,loop_timer_init;
unsigned long difference,difference_init;
unsigned long timer_channel_1, timer_channel_2, timer_channel_3, timer_channel_4, esc_timer, esc_loop_timer;

int throttle_pulse_length, roll_pulse_length, pitch_pulse_length, yaw_pulse_length;
float calcul;
unsigned int testA,testB,testC,testD;
unsigned int  pulse_length_escA, pulse_length_escB, pulse_length_escC, pulse_length_escD;

unsigned long zero_timer;
volatile unsigned long timer[4];

int n;
int first;

// Previous state of each channel (HIGH or LOW)
volatile byte previous_state[4];

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
    gyro.begin();

    // Customize mapping of controls: set here which command is on wich channel.

    mode_mapping[YAW]      = CHANNEL4;
    mode_mapping[PITCH]    = CHANNEL2;
    mode_mapping[ROLL]     = CHANNEL1;
    mode_mapping[THROTTLE] = CHANNEL3;

    DDRH |=B01111000;//port PH5, PH6, PH3 and PH4 en sortie
    DDRK &= B00111001;//port PK2,PK7,PK1,PK6 en entrée 

    PCICR |= B00000100; // groupe PCIE2=1 (PCINT23..16) 17,18,22,23

    PCMSK2 |= (1 << PCINT17); //Set PCINT17 (digital input A9, PK1) to trigger an interrupt on state change.
    PCMSK2 |= (1 << PCINT18); //Set PCINT18 (digital input A10, PK2)to trigger an interrupt on state change.
    PCMSK2 |= (1 << PCINT22); //Set PCINT22 (digital input A14, PK6)to trigger an interrupt on state change.
    PCMSK2 |= (1 << PCINT23); //Set PCINT23 (digital input A15, PK7)to trigger an interrupt on state change.
   
    zero_timer = micros();
    n=0;
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

now_init = micros();

if (first) {loop_timer_init=now_init;first=0;}

difference_init = now_init - loop_timer_init;

if (difference_init<10000){

   for(int i=0;i<1000;i++){
      PORTH |= B11111000; // On active les broches #3 #4 #5 #6 (et aussi 7 la lumiere bleue) du port D (état HIGH)
      delay(1);
      PORTH &= B10000111; //on les desactive le temps restant du cycle
      delay(3);
      }
   }

  calcul=(pulse_duration[mode_mapping[ROLL]]-1500)/COMMAND_REDUCTION_COEF;
  roll_pulse_length=abs(calcul)/calcul * round(abs(calcul));

  calcul=(pulse_duration[mode_mapping[PITCH]]-1500)/COMMAND_REDUCTION_COEF;
  pitch_pulse_length=abs(calcul)/calcul * round(abs(calcul));

  calcul=(pulse_duration[mode_mapping[YAW]]-1500)/COMMAND_REDUCTION_COEF;
  yaw_pulse_length=abs(calcul)/calcul * round(abs(calcul));

  throttle_pulse_length=round(pulse_duration[mode_mapping[THROTTLE]]);

  if (throttle_pulse_length >= 1012){
    //memorize previous angles
    gyro.update();
    //memorize time difference in between angle mesures
    //compute pid
    roll_pulse_length = error_roll * COEF_KP + error_derivative_roll * COEF_KD
  }
 

  //Calibration des moteurs

  pulse_length_escA = REDUCE_MOTOR_A*( COEF_THROTTLE*throttle_pulse_length - COEF_PITCH_ROLL*roll_pulse_length - COEF_PITCH_ROLL*pitch_pulse_length + COEF_YAW*yaw_pulse_length );

  pulse_length_escB = REDUCE_MOTOR_B*( COEF_THROTTLE*throttle_pulse_length + COEF_PITCH_ROLL*roll_pulse_length - COEF_PITCH_ROLL*pitch_pulse_length - COEF_YAW*yaw_pulse_length );

  pulse_length_escC = REDUCE_MOTOR_C*( COEF_THROTTLE*throttle_pulse_length - COEF_PITCH_ROLL*roll_pulse_length + COEF_PITCH_ROLL*pitch_pulse_length - COEF_YAW*yaw_pulse_length );

  pulse_length_escD = REDUCE_MOTOR_D*( COEF_THROTTLE*throttle_pulse_length + COEF_PITCH_ROLL*roll_pulse_length + COEF_PITCH_ROLL*pitch_pulse_length + COEF_YAW*yaw_pulse_length );


  pulse_length_escA=minMax(pulse_length_escA,1000,2000);
  pulse_length_escB=minMax(pulse_length_escB,1000,2000);
  pulse_length_escC=minMax(pulse_length_escC,1000,2000);
  pulse_length_escD=minMax(pulse_length_escD,1000,2000);
 

   // Fs = 250Hz : on envoie les impulsions toutes les 4000µs
   now = micros();
   loop_timer = now;

  // On active les broches #3 #4 #5 #6 (et aussi 7 la lumiere bleue) du port D (état HIGH)
   PORTH |= B11111000;
   testA=testB=testC=testD=0;

  //while (PORTH >=191) {

  while ((testA!=1)|(testB!=1)|(testC!=1)|(testD!=1)) {

        now = micros();
        difference = now - loop_timer;

        if ((difference >= pulse_length_escA)&&(!testA)) {PORTH &= ~(1<<PORTH5);testA=1;} // Passe la broche #H5 à LOW PORTH &= B11011111;Serial.println("A");
        if ((difference >= pulse_length_escB)&&(!testB))  {PORTH &= ~(1<<PORTH6);testB=1;} // Passe la broche #H6 à LOW PORTH &= B10111111;Serial.println("B");
        if ((difference >= pulse_length_escC)&&(!testC))  {PORTH &= ~(1<<PORTH3);testC=1;} // Passe la broche #H3 à LOW PORTH &= B11110111;Serial.println("C");
        if ((difference >= pulse_length_escD)&&(!testD))  {PORTH &= ~(1<<PORTH4);testD=1;} // Passe la broche #H4 à LOW PORTH &= B11101111;Serial.println("D");
    }

    //PORTH &= B01111111;
    attente=4000-difference;
    delayMicroseconds(attente); 
}

/**
* Calcul la consigne d'un axe en degré/sec.
*
* @param float angle         Angle mesuré en degré sur l'axe (X, Y, Z)
* @param int   channel_pulse Durée en µs de l'impulsion reçue pour l'axe (comprise entre 1000µs et 2000µs)
* @return float
*/

float calculateSetPoint(float angle, int channel_pulse) {
    float level_adjust = angle * 15;
    float set_point    = 0;

    // On s'accorde une marge 16µs pour de meilleurs résultats
    if (channel_pulse > 1508) {
        set_point = channel_pulse - 1508;
    } else if (channel_pulse <  1492) {
        set_point = channel_pulse - 1492;
    }
 
    set_point -= level_adjust;
    set_point /= 3;

    return set_point;
}

 

/**
* Calcul des erreurs

void calculateErrors() {
    // Calcul des erreurs courantes
    errors[YAW]   = angular_motions[YAW]   - pid_set_points[YAW];
    errors[PITCH] = angular_motions[PITCH] - pid_set_points[PITCH];
    errors[ROLL]  = angular_motions[ROLL]  - pid_set_points[ROLL];
 
    // Calcul des sommes d'erreurs : composante intégrale
    error_sum[YAW]   += errors[YAW];
    error_sum[PITCH] += errors[PITCH];
    error_sum[ROLL]  += errors[ROLL];
 
    // on s'assure que ∫e.Ki ne dépasse pas 400
    error_sum[YAW]   = minMax(error_sum[YAW],   -400/Ki[YAW],   400/Ki[YAW]);
    error_sum[PITCH] = minMax(error_sum[PITCH], -400/Ki[PITCH], 400/Ki[PITCH]);
    error_sum[ROLL]  = minMax(error_sum[ROLL],  -400/Ki[ROLL],  400/Ki[ROLL]);
 
    // Calcul du delta erreur : composante dérivée
    delta_err[YAW]   = errors[YAW]   - previous_error[YAW];
    delta_err[PITCH] = errors[PITCH] - previous_error[PITCH];
    delta_err[ROLL]  = errors[ROLL]  - previous_error[ROLL];
 
    // L'erreur courante devient l'erreur précédente pour le prochain tour de boucle
    previous_error[YAW]   = errors[YAW];
    previous_error[PITCH] = errors[PITCH];
    previous_error[ROLL]  = errors[ROLL];
}

 */
 

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
    }
}

/**
* Calcul de minmax
*
* @param float a  Valeur à comparer
* @param float b  Borne inférieure
* @param float c  Borne supérieure
* @return float
*/

float minMax ( float a , float b , float c ) {
  float result = min (a, c);
  result = max(result,b);
  return result;
}
 
