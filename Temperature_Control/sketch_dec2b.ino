
#include <Wire.h>
#include<stdint.h>

void setup() {
  Wire.begin();

  Serial.begin(9600);
  while (!Serial)
    ;  // Leonardo: wait for Serial Monitor
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
  Wire.write(byte(0x8B));  // value to make it works as RPM based, 500 minimum RPM,5 edges sampled, 400mSec update time(ENAG,RNG,RNG,EDG,EDG,UDT,UDT,UDT)=(1 0 0 0 1 0 0 1)
  //Wire.write(byte(0x8C)); //value to make it works as RPM based, 500 minimum RPM,5 edges sampled, 500mSec update time(ENAG,RNG,RNG,EDG,EDG,UDT,UDT,UDT)=(1 0 0 0 1 0 0 1)
  //Wire.write(byte(0x8E));   // value to make it works as RPM based, 500 minimum RPM,5 edges sampled, 1200mSec update time(ENAG,RNG,RNG,EDG,EDG,UDT,UDT,UDT)=(1 0 0 0 1 1 1 0)
  //Wire.write(byte(0xAB)); //default
  Wire.endTransmission();

 

  Wire.beginTransmission(0x2F);
  //Write to Fan Config 2 Register (0x33)
  Wire.write(byte(0x33));
  Wire.write(byte(0x68));  // value for enable ramp control, enable glitch filter, step and basic derivative, error window 0 RPM
  Wire.endTransmission();

  Wire.beginTransmission(0x2F);
  //Write to  GAIN - PIDX GAIN REGISTER
  Wire.write(byte(0x35));
  Wire.write(byte(0x2A));  // derivative gain 4x, Integral gain 4x, proportional gain 4x
  Wire.endTransmission();

  Wire.beginTransmission(0x2F);
  //Write to FAN SPIN - SPIN-UP CONFIGURATION REGISTER
  Wire.write(byte(0x36));
  Wire.write(byte(0x11));  //value for 50% SPin up rutine and SPIN up time 500msec
  Wire.endTransmission();

  Wire.beginTransmission(0x2F);
  //Write to MAX Step Register 0x37
  Wire.write(byte(0x37));
  Wire.write(byte(0x05));  //max step size 5 RPM
  Wire.endTransmission();

  
  Wire.beginTransmission(0x2F);
  //Write to FAN minmum drive Register(0x38)
  Wire.write(byte(0x38));
  //Wire.write(byte(0x19));  //set value for fan minimum drive speed for around 10%
  Wire.write(byte(0x12));  //set value for fan minimum drive speed for around 7%
  //Wire.write(byte(0x66));  //set value for fan minimum drive speed for around 40%
  Wire.endTransmission();

  Wire.beginTransmission(0x2F);
  //Write to Valid TACH Count Register(0x39)
  Wire.write(byte(0x39));
  Wire.write(byte(0x3F));  //maximum valid TACH count
  Wire.endTransmission();

  Wire.beginTransmission(0x2F);
  //Write to  DRIVE FAIL - DRIVE FAIL BAND LOW BYTE REGISTER(0x3A)
  Wire.write(byte(0x3A));
  Wire.write(byte(0xF8));  //
  Wire.endTransmission();

  Wire.beginTransmission(0x2F);
  //Write to  DRIVE FAIL - DRIVE FAIL BAND HIGH BYTE REGISTER(0x3B)
  Wire.write(byte(0x3B));
  Wire.write(byte(0x00));  //
  Wire.endTransmission();

  
}

void Read_Temp(void);
void Read_RPM(void);
void Set_Temp(float Set_Temperature);
void Set_RPM(float Target_RPM);


int reading = 0;

//char PWM = 0;
//uint16_t TACH_max = 983;
//uint16_t TACH_min = 436;
uint16_t TACH;

char TTH_Reg;
char TTL_Reg;
float Set_Temperature;
float temperature = 0;
float Target_RPM = 3932160/ TACH;
float last_temperature = 0;
float RPM = 0;
float last_RPM = 0;




void loop() {
  byte error = Wire.endTransmission();
  Read_Temp();
  
  
  Read_RPM(); 
  //Set_Temp(40);
  Set_RPM(4500);
  
  delay(1000);  // Wait 30 seconds for next scan
}




void Read_Temp(void){
  //Read temperature sensor AD7420
  byte error = Wire.endTransmission();

  //Write AD7420 address
  Wire.beginTransmission(0x48);

  //Write register address to 0x00 high byte temperature
  Wire.write(byte(0x00));

  

  if (error == 0) {
    Wire.requestFrom(0x48, 0x02);
    //Serial.print("Temp = ");

    if (2 <= Wire.available()) {  // if two bytes were received

      reading = Wire.read();  // receive high byte (overwrites previous reading)

      reading = reading << 8;  // shift high byte to be high 8 bits

      reading |= Wire.read();  // receive low byte as lower 8 bits

      //temperature = reading * 0.0078;

      temperature = (reading * 0.0078) - last_temperature;
      temperature = temperature * 0.1 + last_temperature;

      last_temperature = temperature;

      Serial.print(temperature, 3);
      Serial.print(" , ");
      //Serial.println(reading);
    }

  } else {
    Serial.print("Error Reading Temperature\n");
  }
}

void Read_RPM(void){
  byte error = Wire.endTransmission();

  //Read Fan Controller
  //Write EMC2301 address
  Wire.beginTransmission(0x2F);

  //Write register address to 0x3E Tach reading high byte register
  Wire.write(byte(0x3E));
  Wire.endTransmission();

  if (error == 0) {
    //Wire.requestFrom(0x2F, 0x02);
    Wire.requestFrom(0x2F, 8);
    //Serial.print("Tach = ");

    if (2 <= Wire.available()) {  // if one byte was received
    
      reading = Wire.read();  // receive high byte (overwrites previous reading)
      reading = reading << 8;  // shift high byte to be high 8 bits      
      reading |= Wire.read();  // receive low byte as lower 8 bits
      reading = reading >> 3;  //shift by 3 bits as 3 lowest bits are not used
     
      //RPM = 3932160 / reading;
      RPM = (3932160 / reading) - last_RPM;
      RPM = RPM * 0.1 + last_RPM;

      last_RPM = RPM;

      Serial.print(" Actual RPM : ");      
      Serial.print(RPM, 1);
      Serial.print(" , ");
      //Serial.println(reading);
      //Serial.println(diff);

    }

    else {
      Serial.print("Error Reading Fan Controller\n");
    }

  } else {
    Serial.print("Error Reading Fan Controller\n");
  }
  
}





void Set_RPM(float Target_RPM){


 TACH = 3932160/Target_RPM;
  //Target_RPM = 3932160/ TACH;
  TTL_Reg = (TACH<<3) & 0xF8;
  TTH_Reg = (TACH>>5) & 0xFF;
  
  //Write Fan RPM
  //Write EMC2301 address
  Wire.beginTransmission(0x2F);
  //Write to Register 0x3C for TACH Target low byte
  Wire.write(byte(0x3C));
  Wire.write(byte(TTL_Reg));  //set value
  Wire.endTransmission();

  Wire.beginTransmission(0x2F);
  //Write to Register 0x3D for TACH Target High byte
  Wire.write(byte(0x3D));
  Wire.write(byte(TTH_Reg));  //set value
  Wire.endTransmission();

  Serial.print(" Target RPM : ");      
  Serial.println(Target_RPM, 1);
  //Serial.println(" , "); 
}




