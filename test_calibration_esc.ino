#include <Servo.h>

 

Servo motA, motB, motC, motD;

char data;

 

/**

* Initialisation routine

*/

void setup() {

    Serial.begin(9600);

 

    //motA.attach(8, 1000, 2000); // 1000 and 2000 are very important ! Values can be different with other ESCs.

    //motB.attach(9, 1000, 2000);

    //motC.attach(6, 1000, 2000);

    motD.attach(7, 1000, 2000);

 

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

                      //motA.write(0);

                      //motB.write(0);

                      //motC.write(0);

                      motD.write(0);

            break;

 

            // 1

            case 49 : Serial.println("Sending 180 throttle");

                      //motA.write(180);

                      //motB.write(180);

                      //motC.write(180);

                      motD.write(180);

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

 

        //motA.write(i);

        //motB.write(i);

        //motC.write(i);

        motD.write(i);

 

        delay(200);

    }

 

    Serial.println("STOP");

    //motA.write(0);

    //motB.write(0);

    //motC.write(0);

    motD.write(0);

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

 
