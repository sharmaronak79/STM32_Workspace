// --------------------------------------
// i2c_scanner
//
// Version 1
//    This program (or code that looks like it)
//    can be found in many places.
//    For example on the Arduino.cc forum.
//    The original author is not known.
// Version 2, Juni 2012, Using Arduino 1.0.1
//     Adapted to be as simple as possible by Arduino.cc user Krodal
// Version 3, Feb 26  2013
//    V3 by louarnold
// Version 4, March 3, 2013, Using Arduino 1.0.3
//    by Arduino.cc user Krodal.
//    Changes by louarnold removed.
//    Scanning addresses changed from 0...127 to 1...119,
//    according to the i2c scanner by Nick Gammon
//    https://www.gammon.com.au/forum/?id=10896
// Version 5, March 28, 2013
//    As version 4, but address scans now to 127.
//    A sensor seems to use address 120.
// Version 6, November 27, 2015.
//    Added waiting for the Leonardo serial communication.
//
//
// This sketch tests the standard 7-bit addresses
// Devices with higher bit address might not be seen properly.
//

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
  Wire.write(byte(0x8B));  // value to make it works as RPM based, make ENAG bit 1 
  Wire.endTransmission();

  Wire.beginTransmission(0x2F);
    //Write to Valid TACH Count Register(0x39) 
  Wire.write(byte(0x39));
  Wire.write(byte(0xFF));  //maximum valid TACH count
  Wire.endTransmission();


  Wire.beginTransmission(0x2F);
    //Write to FAN minmum drive Register(0x38)
  Wire.write(byte(0x38));
  //Wire.write(byte(0x19));  //set value for fan minimum drive speed for around 10%
  Wire.write(byte(0x12));  //set value for fan minimum drive speed for around 7%
  Wire.endTransmission();




}

int reading = 0;

//char PWM = 0;
char TACH_Value = 0;

void loop(){
    
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

                float temperature = reading * 0.0078;

        Serial.print(temperature, 3);
        Serial.print(" , ");
        Serial.println(reading);
    }

  }  
  else {
    Serial.print("Error Reading Temperature\n");
  }


  /*
  //PWM=PWM + 8;  //Increase PWM by 8 each time through the loop
  PWM =255;

  //Write fan PWM
   //Write EMC2301 address
  Wire.beginTransmission(0x2F);
    //Write register address to 0x30 Fan1 PWM register
  Wire.write(byte(0x30));
  Wire.write(byte(PWM));  //set to 16 bit operation
  Wire.endTransmission();
  */
  
    
    TACH_Value = 254;
  //Write Fan RPM
  //Write EMC2301 address
  Wire.beginTransmission(0x2F);
    //Write to Register 0x3D for TACH Target High byte
  Wire.write(byte(0x3D));
  Wire.write(byte(TACH_Value));  //set value
  Wire.endTransmission();

  Wire.beginTransmission(0x2F);
    //Write to Register 0x3C for TACH Target low byte
  Wire.write(byte(0x3C));
  Wire.write(byte(0xf8));  //set value
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

      
      reading = Wire.read();  // receive high byte (overwrites previous reading)

      reading = reading << 8;    // shift high byte to be high 8 bits

      reading |= Wire.read(); // receive low byte as lower 8 bits
      reading = reading >> 3;  //shift by 3 bits as 3 lowest bits are not used

                float RPM = 3932160 / reading;

        Serial.print(RPM, 1);
        Serial.print(" , ");
        Serial.println(reading);

    }
      
  else {
    Serial.print("Error Reading Fan Controller\n");
  }
  
  }  
  else {
    Serial.print("Error Reading Fan Controller\n");
  }

  
  delay(5000); // Wait 5 seconds for next scan
}
