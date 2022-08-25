void setup() {
  Serial.begin(9600);
  DDRH |= B11111111;              // outputs, 1 is relevant                                                       //Configure digital poort PB7 as LED output.
  DDRK &= B00111001;             // entries, 0 is relevant                                                        // PK1/2/6/7 en entrée

  PCICR |= B00000100;                                                                    // groupe PCIE2=1 (PCINT23..16)
  PCMSK2 |= B11000110;                                                                   //PCINT17/18/22/23 autorisées

  Serial.println("setup");
  for (int i = 0; i < 1000; i++) {
    PORTH = B11111111;
    delay(1);
    PORTH = B00000000;
    delay(3);
  }

}

void loop() {
  Serial.println("commande de 1100 ms");
  for(int i = 0;i<1000;i++)    //4ms x 2000 = 8 sec
    {
        PORTH = B11111111;
        delayMicroseconds(1100);
        PORTH = B00000000;
        delayMicroseconds(2900);
    }
  Serial.println("commande de 1200 ms");
  for(int i = 0;i<1000;i++)    //4ms x 2000 = 8 sec
    {
        PORTH = B11111111;
        delayMicroseconds(1200);
        PORTH = B00000000;
        delayMicroseconds(2800);
    }
  Serial.println("commande de 1300 ms");
  for(int i = 0;i<1000;i++)    //4ms x 2000 = 8 sec
    {
        PORTH = B11111111;
        delayMicroseconds(1300);
        PORTH = B00000000;
        delayMicroseconds(2700);
    }
  Serial.println("commande de 1400 ms");
  for(int i = 0;i<1000;i++)    //4ms x 2000 = 8 sec
    {
        PORTH = B11111111;
        delayMicroseconds(1400);
        PORTH = B00000000;
        delayMicroseconds(2600);
    }
  Serial.println("commande de 1500 ms");
  for(int i = 0;i<1000;i++)    //4ms x 2000 = 8 sec
    {
        PORTH = B11111111;
        delayMicroseconds(1500);
        PORTH = B00000000;
        delayMicroseconds(3900);
    }
  Serial.println("commande de 1600 ms");
  for(int i = 0;i<1000;i++)    //4ms x 2000 = 8 sec
    {
        PORTH = B11111111;
        delayMicroseconds(1600);
        PORTH = B00000000;
        delayMicroseconds(2400);
    }
  Serial.println("commande de 1700 ms");
  for(int i = 0;i<1000;i++)    //4ms x 2000 = 8 sec
    {
        PORTH = B11111111;
        delayMicroseconds(1700);
        PORTH = B00000000;
        delayMicroseconds(2300);
    }
  Serial.println("commande de 1800 ms");
  for(int i = 0;i<1000;i++)    //4ms x 2000 = 8 sec
    {
        PORTH = B11111111;
        delayMicroseconds(1800);
        PORTH = B00000000;
        delayMicroseconds(2200);
    }

    /*
  Serial.println("commande de 1900 ms");
  for(int i = 0;i<1000;i++)    //4ms x 2000 = 8 sec
    {
        PORTH = B11111111;
        delayMicroseconds(1900);
        PORTH = B00000000;
        delayMicroseconds(3100);
    }
  Serial.println("commande de 2000 ms");
  for(int i = 0;i<1000;i++)    //4ms x 2000 = 8 sec
    {
        PORTH = B11111111;
        delayMicroseconds(2000);
        PORTH = B00000000;
        delayMicroseconds(2000);
    }
    
    */

  Serial.println("end of test");
  while(1){
    PORTH = B11111111;
    delay(1);
    PORTH = B00000000;
    delay(3);
  }
}
