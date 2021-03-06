//IMU LIBS
#include<Wire.h>
const int MPU=0x68;  // I2C address of the MPU-6050
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;
double compAngleX,compAngleY,/*gzangle,*/compAnglez,timer;
  double accXangle ,accYangle,acczangle ,gyroXrate ,gyroYrate,gyroZrate;
  double gyroXAngle, gyroYAngle, gyroZAngle;
  // float rgyro,w;
  int ap=0.955;





// GPS LIBS
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
// Choose two Arduino pins to use for software serial
int RXPin = 2;
int TXPin = 3;

int GPSBaud = 9600;

// Create a TinyGPS++ object
TinyGPSPlus gps;

// Create a software serial port called "gpsSerial"
SoftwareSerial gpsSerial(RXPin, TXPin);




// PRESSURE LIBS
#include <Wire.h>
#define ADDRESS_SENSOR 0x77 

int16_t  ac1, ac2, ac3, b1, b2, mb, mc, md;
uint16_t ac4, ac5, ac6;

const uint8_t oss = 3;
const uint8_t osd = 26;
float T, P, H; 


void setup() {
  // put your setup code here, to run once:

//IMU

{
   Serial.begin(115200);
/////////////////////// SENSOR READING//////////
 Wire.begin();
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  
  ///////////////////////////////////////
}




//GPS

{
  // Start the Arduino hardware serial port at 9600 baud
  Serial.begin(9600);

  // Start the software serial port at the GPS's default baud
  gpsSerial.begin(GPSBaud);
}



//PRESSURE

{
  Serial.begin(115200);                       
  Wire.begin();                             
  ac1 = read_2_bytes(0xAA);
  ac2 = read_2_bytes(0xAC);
  ac3 = read_2_bytes(0xAE);
  ac4 = read_2_bytes(0xB0);
  ac5 = read_2_bytes(0xB2);
  ac6 = read_2_bytes(0xB4);
  b1  = read_2_bytes(0xB6);
  b2  = read_2_bytes(0xB8);
  mb  = read_2_bytes(0xBA);
  mc  = read_2_bytes(0xBC);
  md  = read_2_bytes(0xBE);                          
  delay(100);
}


}

void loop() {
  // put your main code here, to run repeatedly:


// IMU 

{

  Wire.beginTransmission(MPU);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU,14,true);  // request a total of 14 registers
  AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)   
  AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)



 accXangle = (atan2(AcY, AcZ) * RAD_TO_DEG);
 accYangle = (atan2(AcX, AcZ) * RAD_TO_DEG);
acczangle = (atan2(AcX,AcY) * RAD_TO_DEG);/* my attempt to calculate yaw but not correct*/
 gyroXrate = GyX / 16.5;
 gyroYrate = GyY / 16.5;
 gyroZrate = GyZ/ 16.5;
 Tmp=(Tmp/340+36.53);

 Serial.print("Temp : ");
 Serial.print(Tmp);
 Serial.print("       ");
 Serial.print("X: ");
 Serial.print(accXangle);
 Serial.print("        ");
 Serial.print("Y: ");
 Serial.print(accYangle);
 Serial.print("        ");
 Serial.print("Z: ");
 Serial.print(acczangle);
 Serial.print("        ");
 Serial.print("accX: ");
 Serial.print(gyroXrate);
 Serial.print("        ");
 Serial.print("accY: ");
 Serial.print(gyroYrate);
 Serial.print("        ");
 Serial.print("accZ: ");
 Serial.print(gyroZrate);
 Serial.print("        ");
 Serial.print("gX: ");
 Serial.print(GyX);
 Serial.print("        ");
 Serial.print("gY: ");
 Serial.print(GyY);
 Serial.print("        ");
 Serial.print("gZ: ");
 Serial.println(GyZ);
 delay(100);

}



// GPS

{
  // This sketch displays information every time a new sentence is correctly encoded.
  while (gpsSerial.available() > 0)
    if (gps.encode(gpsSerial.read()))
      displayInfo();

  // If 5000 milliseconds pass and there are no characters coming in
  // over the software serial port, show a "No GPS detected" error
  if (millis() > 5000 && gps.charsProcessed() < 10)
  {
    Serial.println("No GPS detected");
    while(true);
  }
}


// PRESSURE

{
  int32_t b5;
 b5 = temperature();
 P = pressure(b5);
 H = 44330*(1-pow((P/1013.25),1/5.255));
  Serial.print("Temperature: ");
  Serial.print(T, 2);
  Serial.print("*C, "); 
  Serial.print("     ");
  Serial.print("Pression: ");
  Serial.print(P*100, 2);
  Serial.print(" pa");
  Serial.print("     ");
  Serial.print("H: ");
  Serial.print(H);
  Serial.println("m");

  delay(1000);                               
  
}
float pressure(int32_t b5)
{
  int32_t x1, x2, x3, b3, b6, p, UP;
  uint32_t b4, b7; 

  UP = read_pressure();                         

  b6 = b5 - 4000;
  x1 = (b2 * (b6 * b6 >> 12)) >> 11; 
  x2 = ac2 * b6 >> 11;
  x3 = x1 + x2;
  b3 = (((ac1 * 4 + x3) << oss) + 2) >> 2;
  x1 = ac3 * b6 >> 13;
  x2 = (b1 * (b6 * b6 >> 12)) >> 16;
  x3 = ((x1 + x2) + 2) >> 2;
  b4 = (ac4 * (uint32_t)(x3 + 32768)) >> 15;
  b7 = ((uint32_t)UP - b3) * (50000 >> oss);
  if(b7 < 0x80000000) { p = (b7 << 1) / b4; } else { p = (b7 / b4) << 1; } // ou p = b7 < 0x80000000 ? (b7 * 2) / b4 : (b7 / b4) * 2;
  x1 = (p >> 8) * (p >> 8);
  x1 = (x1 * 3038) >> 16;
  x2 = (-7357 * p) >> 16;
  return (p + ((x1 + x2 + 3791) >> 4)) / 100.0f; 
}


int32_t temperature()
{
  int32_t x1, x2, b5, UT;

  Wire.beginTransmission(ADDRESS_SENSOR); 
  Wire.write(0xf4);                       
  Wire.write(0x2e);                       
  Wire.endTransmission();                 
  delay(5);                               
  
  UT = read_2_bytes(0xf6);                 

  // Calcule la vrai température
  x1 = (UT - (int32_t)ac6) * (int32_t)ac5 >> 15;
  x2 = ((int32_t)mc << 11) / (x1 + (int32_t)md);
  b5 = x1 + x2;
  T  = (b5 + 8) >> 4;
  T = T / 10.0;                            
  return b5;  
}


int32_t read_pressure()
{
  int32_t value; 
  Wire.beginTransmission(ADDRESS_SENSOR);   
  Wire.write(0xf4);                         
  Wire.write(0x34 + (oss << 6));            
  Wire.endTransmission();                  
  delay(osd);                               
  Wire.beginTransmission(ADDRESS_SENSOR);
  Wire.write(0xf6);                         
  Wire.endTransmission();
  Wire.requestFrom(ADDRESS_SENSOR, 3);      
  if(Wire.available() >= 3)
  {
    value = (((int32_t)Wire.read() << 16) | ((int32_t)Wire.read() << 8) | ((int32_t)Wire.read())) >> (8 - oss);
  }
  return value;                            
}


uint8_t read_1_byte(uint8_t code)
{
  uint8_t value;
  Wire.beginTransmission(ADDRESS_SENSOR);         
  Wire.write(code);                               
  Wire.endTransmission();                         
  Wire.requestFrom(ADDRESS_SENSOR, 1);            
  if(Wire.available() >= 1)
  {
    value = Wire.read();                          
  }
  return value;                                   
}


uint16_t read_2_bytes(uint8_t code)
{
  uint16_t value;
  Wire.beginTransmission(ADDRESS_SENSOR);         
  Wire.write(code);                               
  Wire.endTransmission();                         
  Wire.requestFrom(ADDRESS_SENSOR, 2);            
  if(Wire.available() >= 2)
  {
    value = (Wire.read() << 8) | Wire.read();     
  }
  return value;                                   
}


}


//GPS

void displayInfo()
{
  if (gps.location.isValid())
  {
    Serial.print("Latitude: ");
    Serial.println(gps.location.lat(), 6);
    Serial.print("Longitude: ");
    Serial.println(gps.location.lng(), 6);
    Serial.print("Altitude: ");
    Serial.println(gps.altitude.meters());
  }
  else
  {
    Serial.println("Location: Not Available");
  }
  
  Serial.print("Date: ");
  if (gps.date.isValid())
  {
    Serial.print(gps.date.month());
    Serial.print("/");
    Serial.print(gps.date.day());
    Serial.print("/");
    Serial.println(gps.date.year());
  }
  else
  {
    Serial.println("Not Available");
  }

  Serial.print("Time: ");
  if (gps.time.isValid())
  {
    if (gps.time.hour() < 10) Serial.print(F("0"));
    Serial.print(gps.time.hour());
    Serial.print(":");
    if (gps.time.minute() < 10) Serial.print(F("0"));
    Serial.print(gps.time.minute());
    Serial.print(":");
    if (gps.time.second() < 10) Serial.print(F("0"));
    Serial.print(gps.time.second());
    Serial.print(".");
    if (gps.time.centisecond() < 10) Serial.print(F("0"));
    Serial.println(gps.time.centisecond());
  }
  else
  {
    Serial.println("Not Available");
  }

  Serial.println();
  Serial.println();
  delay(1000);
}
