#include<SPI.h>
#include<nRF24L01.h>
#include<RF24.h>
#include<Wire.h>                        
RF24 radio(9, 10);
const byte address[6] = "00001";
struct package_IMU {
  const int MPU=0x68;  
  int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;
  double compAngleX,compAngleY,/*gzangle,*/compAnglez,timer;
  double accXangle ,accYangle,acczangle ,gyroXrate ,gyroYrate,gyroZrate;
  double gyroXAngle, gyroYAngle, gyroZAngle;
  int ap=0.955;
} test;
struct package_PRESSURE{
  double T, P, H; 
} untest;
void setup() {
  Serial.begin(115200);
  radio.begin();
  radio.openReadingPipe(0, address);
  radio.startListening();
}
void loop() {
  if (radio.available()>0)
  {
radio.read(&test, sizeof(test));
radio.read(&untest, sizeof(untest));
calculate();
 Serial.print("Temp : ");
 Serial.print(test.Tmp);
 Serial.print("       ");
 Serial.print("X: ");
 Serial.print(test.accXangle);
 Serial.print("       ");
 Serial.print("Y: ");
 Serial.print(test.accYangle);
 Serial.print("       ");
 Serial.print("Z: ");
 Serial.print(test.acczangle);
 Serial.print("       ");
 Serial.print("accX: ");
 Serial.print(test.gyroXrate);
 Serial.print("       ");
 Serial.print("accY: ");
 Serial.print(test.gyroYrate);
 Serial.print("       ");
 Serial.print("accZ: ");
 Serial.print(test.gyroZrate);
 Serial.print("       ");
 Serial.print("gX: ");
 Serial.print(test.GyX);
 Serial.print("       ");
 Serial.print("gY: ");
 Serial.print(test.GyY);
 Serial.print("       ");
 Serial.print("gZ: ");
 Serial.print(test.GyZ);
 Serial.print("       ");
 Serial.print("P: ");
 Serial.print(untest.P);
 Serial.print("       ");
 Serial.print("H: ");
 Serial.println(untest.H);
 delay(50);
  }
}
void calculate(){
test.accXangle = (atan2(test.AcY, test.AcZ) * RAD_TO_DEG);
test.accYangle = (atan2(test.AcX, test.AcZ) * RAD_TO_DEG);
test.acczangle = (atan2(test.AcX,test.AcY) * RAD_TO_DEG);
test.gyroXrate = test.GyX / 16.5;
test.gyroYrate = test.GyY / 16.5;
test.gyroZrate = test.GyZ/ 16.5;
test.Tmp=(test.Tmp/340+36.53);
}
