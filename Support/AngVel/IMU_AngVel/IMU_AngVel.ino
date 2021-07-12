/*****************************************
*
* Modified by Brian Tischler 
* for IMU module CMPS14 for AgOpenGPS
*****************************************/

#include <Wire.h>

// Address of CMPS14 shifted right one bit for arduino wire library
#define CMPS14_ADDRESS 0x60  

//CMPS PGN - 211 D3
byte data[] = {0x80,0x81,0x7D,0xD3,8, 0,0,0,0, 0,0,0,0, 15};

//100hz summing of gyro
float gyroSum, kalGyro;
  
//count to 10 and send to AOG
byte counter;

//Kalman control variances
const float varRoll = 0.01; // variance, larger is more filtering
const float varProcess = 0.025; //process, smaller is more filtering

//variables
float Pc = 0.0, G = 0.0, P = 1.0, Xp = 0.0, Zp = 0.0;
float angVel = 0;
  
void setup()
{
  Serial.begin(38400);  // Start serial port
  Wire.begin();
}

void loop()
{  
  //read gyro at 100 hz
  Wire.beginTransmission(CMPS14_ADDRESS);  
  Wire.write(0x16);                    
  Wire.endTransmission();
 
  Wire.requestFrom(CMPS14_ADDRESS, 2);  
  while(Wire.available() < 2); 
         
  float gyro = (float)(Wire.read()<<8 | Wire.read());

  //Kalman filter
  Pc = P + varProcess;
  G = Pc / (Pc + varRoll);
  P = (1 - G) * Pc;
  Xp = kalGyro;
  Zp = Xp;
  kalGyro = (G * (gyro - Zp) + Xp);
     
  if (counter == 9)
  {
    //the heading x10
    Wire.beginTransmission(CMPS14_ADDRESS);  
    Wire.write(0x02);                     
    Wire.endTransmission();
    
    Wire.requestFrom(CMPS14_ADDRESS, 2); 
    while(Wire.available() < 2);       
  
    data[6] = Wire.read();
    data[5] = Wire.read();
   
    //the roll x10
    Wire.beginTransmission(CMPS14_ADDRESS);  
    Wire.write(0x1C);                    
    Wire.endTransmission();
   
    Wire.requestFrom(CMPS14_ADDRESS, 2);  
    while(Wire.available() < 2);        
  
    data[8] = Wire.read();
    data[7] = Wire.read();
    
    gyroSum += kalGyro+8;
    gyroSum *= 1.3;

    data[9] = (byte)(gyroSum);
    data[10] = (byte)((int(gyroSum))>>8);    

    int CK_A = 0;
    for (int i = 2; i < sizeof(data)-1; i++)
    {
        CK_A = (CK_A + data[i]);
    }
    data[sizeof(data)-1] = CK_A;
    
    Serial.write(data,sizeof(data));

    /*
    Serial.print(0);
    Serial.print(",");
    Serial.println(gyroSum);
    */
    Serial.flush(); 
      
    gyroSum = counter = 0;
    delay(9);       
  }
  else
  {
    gyroSum += kalGyro;
    counter++;
    delay(10);
  }                    
}
