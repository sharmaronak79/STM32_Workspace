
#include <Wire.h>

void setup() {
  Wire.begin();

  Serial.begin(9600);
  while (!Serial); // Leonardo: wait for Serial Monitor
  Serial.println("\nTemp Control Concept");

  //Configure AD7420
  //Write AD7420 address
  Wire.beginTransmission(0x48);
    //Write register address to 0x03 Configuration register
  Wire.write(byte(0x03));
  Wire.write(byte(0x80));  //set to 16 bit operation
  Wire.endTransmission();

  //Configure EMC2301
  //Write EMC2301 address
  Wire.beginTransmission(0x2F);
    //Write register address to 0x32 Fan1 Configuration register
  Wire.write(byte(0x32));
  Wire.write(byte(0x0B));  //set to 16 bit operation
  Wire.endTransmission();

}

int reading = 0;
int reading2;

char PWM = 0;

void loop() {

  
  float temperature;
  int Set_Temp = 29;

  //Read temperature sensor AD7420

  //Write AD7420 address
  Wire.beginTransmission(0x48);
  
  //Write register address to 0x03 Configuration register
  Wire.write(byte(0x03));

  Wire.write(byte(0x80));  //set to 16 bit operation
  
  Wire.endTransmission();

   //Write AD7420 address
  Wire.beginTransmission(0x48);
  
  //Write register address to 0x00 high byte temperature
  Wire.write(byte(0x00));
    
  byte error = Wire.endTransmission();

  if (error == 0 ) {
    Wire.requestFrom(0x48, 0x02);
    Serial.print("Temp = ");
    
    if (2 <= Wire.available()) { // if two bytes were received

      reading = Wire.read();  // receive high byte (overwrites previous reading)

      reading = reading << 8;    // shift high byte to be high 8 bits

      reading |= Wire.read(); // receive low byte as lower 8 bits

                 temperature = reading * 0.0078;

        Serial.print(temperature, 3);
        Serial.print(" , ");
        Serial.println(reading);
    }

  }  
  else {
    Serial.print("Error Reading Temperature\n");
  }
  


  if(temperature > Set_Temp){

      PWM=PWM + 8;  //Increase PWM by 8 each time through the loop

  }else if(temperature < Set_Temp){
    PWM=0;
  }



  

  //Write fan PWM
   //Write EMC2301 address
  Wire.beginTransmission(0x2F);
    //Write register address to 0x30 Fan1 PWM register
  Wire.write(byte(0x30));
  Wire.write(byte(PWM));  //set to 16 bit operation
  Wire.endTransmission();



  //Read Fan Controller

  //Write EMC2301 address
  Wire.beginTransmission(0x2F);
  
  //Write register address to 0x3E Tach reading high byte register
  Wire.write(byte(0x3E));
  Wire.endTransmission();
  
  if (error == 0 ) {
    Wire.requestFrom(0x2F, 0x02);
    Serial.print("Tach = ");
    
    if (2 <= Wire.available()) { // if one byte was received

      
      reading2 = Wire.read();  // receive high byte (overwrites previous reading)

      reading2 = reading2 << 8;    // shift high byte to be high 8 bits

      reading2 |= Wire.read(); // receive low byte as lower 8 bits
      reading2 = reading2 >> 3;  //shift by 3 bits as 3 lowest bits are not used

                float RPM = 3932160 / reading2;

        Serial.print(RPM, 1);
        Serial.print(" , ");
        Serial.println(reading2);

        if(reading2<500){
          PWM = PWM - 8;
          PWM = PWM -8;
        }

      

    }
      
    else {
      Serial.print("Error Reading Fan Controller\n");
    }
  
  }  
  else {
    Serial.print("Error Reading Fan Controller\n");
  }

  
  delay(500); // Wait 5 seconds for next scan
}
