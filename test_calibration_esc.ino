#include <Servo.h>


Servo motA, motB, motC, motD;

char data;
char selection = 'B';

 

/**

* Initialisation routine

*/

void setup() {

    Serial.begin(9600);

 
    switch (selection){
      case 'A' :
        motA.attach(8, 1000, 2000); // 1000 and 2000 are very important ! Values can be different with other ESCs.
        break;
      case 'B' :
        motB.attach(9, 1000, 2000);
        break;
      case 'C' :
        motC.attach(6, 1000, 2000);
        break;
      case 'D' :
        motD.attach(7, 1000, 2000);
        break;
      default : Serial.println("coding error");break;
    }
 

    displayInstructions();

}

 

/**

* Main function

*/

void loop() {

    if (Serial.available()) {

        data = Serial.read();

 

        switch (data) {

            // 0

            case 48 : Serial.println("Sending 0 throttle");

                    switch (selection){
                      case 'A' :
                        motA.write(0);
                        break;
                      case 'B' :
                        motB.write(0);
                        break;
                      case 'C' :
                        motC.write(0);
                        break;
                      case 'D' :
                        motD.write(0);
                        break;
                      default : Serial.println("coding error");break;
                    }

            break;

 

            // 1

            case 49 : Serial.println("Sending 180 throttle");

                    switch (selection){
                      case 'A' :
                        motA.write(180);
                        break;
                      case 'B' :
                        motB.write(180);
                        break;
                      case 'C' :
                        motC.write(180);
                        break;
                      case 'D' :
                        motD.write(180);
                        break;
                      default : Serial.println("coding error");break;
                    }

            break;

 

            // 2

            case 50 : Serial.print("Running test in 3");

                      delay(1000);

                      Serial.print(" 2");

                      delay(1000);

                      Serial.println(" 1...");

                      delay(1000);

                      test();

            break;

        }

    }

}

 

/**

* Test function sending angle to the ESCs from 0 to 180 degrees

*/

void test()

{

    for (int i=0; i<=180; i++) {

        Serial.print("Speed = ");

        Serial.println(i);

 
        switch (selection){
          case 'A' :
            motA.write(i);
            break;
          case 'B' :
            motB.write(i);
            break;
          case 'C' :
            motC.write(i);
            break;
          case 'D' :
            motD.write(i);
            break;
          default : Serial.println("coding error");break;
        }
 

        delay(200);

    }

 

    Serial.println("STOP");
    
    switch (selection){
      case 'A' :
        motA.write(0);
        break;
      case 'B' :
        motB.write(0);
        break;
      case 'C' :
        motC.write(0);
        break;
      case 'D' :
        motD.write(0);
        break;
      default : Serial.println("coding error");break;
    }

}

 

/**

* Displays instructions to user

*/

void displayInstructions()

{

    Serial.println("READY - PLEASE SEND INSTRUCTIONS AS FOLLOWING :");

    Serial.println("\t0 : Sends 0 throttle");

    Serial.println("\t1 : Sends 180 throttle");

    Serial.println("\t2 : Runs test function\n");

}

 
