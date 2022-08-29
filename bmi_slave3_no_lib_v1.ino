#include <Wire.h>
#include <SoftwareSerial.h>
#include "RS485_protocol.h"

SoftwareSerial rs485 (6, 7);  // receive pin, transmit pin
const byte ENABLE_PIN = 3;
const unsigned int baud=19200;


//==

const int i2c_addr = 0x68;

const float xcal = -0.8;
const float ycal = 5.3;

#define BMI160_CMD_ACC_MODE_NORMAL  0x11
#define BMI160_CMD_SOFT_RESET       0xB6
#define BMI160_RA_CMD               0x7E

#define ACC_LSB_2_G        16384.0  // [bit/gravity]
#define RAD_2_DEG          57.29578 // [°/rad]
float accX, accY, accZ;
float angleX,angleY; 
float prevY=0,prevX=0;
int i=0,j=0;
int ax,ay,az;
int count=0;
//=======


//===
  
  int len = 20;
  float x_angle[20] = {0};
  float y_angle[20] = {0};
  int pos = 0;

  float newAvgx = 0;
  float newAvgy = 0;
  float sumx = 0;
  float sumy = 0;


//==



//===
unsigned long previousMillis = 0;        // will store last time LED was updated

// constants won't change:
const long interval = 500;           // interval at which to read sensor (milliseconds)
//=
//====
float movingAvg(float *x_anglePtr, float *ptrSum, int pos, int len, float nextAngle)
{
  //Subtract the oldest number from the prev sum, add the new number
  *ptrSum = *ptrSum - x_anglePtr[pos] + nextAngle;
  //Assign the nextNum to the position in the array
  x_anglePtr[pos] = nextAngle;
  //return the average
  return *ptrSum / len;
}





//===========
void fWrite (const byte what)
  {
  rs485.write (what);  
  }
  
int fAvailable ()
  {
  return Serial.available ();  
  }

int fRead ()
  {
  return Serial.read ();  
  }
  
void setup()
{

   Serial.begin(baud);
   Serial.println("initialised...  ");
  rs485.begin (baud);
  pinMode (ENABLE_PIN, OUTPUT);  // driver output enable

//======
// initialize device
bmi160init();
 delay(50);

for(i=0;i<len;i++){
bmi160read();
  accX = ((float)ax) / ACC_LSB_2_G;
  accY = ((float)ay) / ACC_LSB_2_G;
  accZ = ((float)az) / ACC_LSB_2_G;

  angleX = asin(accX) * RAD_2_DEG;
  //Serial.println(angleX);
  angleY = asin(accY) * RAD_2_DEG;

  

  x_angle[i]=angleX;
  y_angle[i]=angleY;

  sumx=sumx+x_angle[i];
  sumy=sumy+y_angle[i];

  delay(50);
}
  //==
//===  
}
//=========
typedef union
{
  float number;
  uint8_t bytes[4];
} floatUnion;


  
//===
void loop()
{
  byte buf [10];
  
  byte devID=3;
//=====read sensor===

  int gx, gy, gz;         // raw gyro values
  unsigned long currentMillis = millis();

if (currentMillis - previousMillis >= interval) {

  previousMillis = currentMillis;
     // read raw gyro measurements from device
 bmi160read();

  accX = ((float)ax) / ACC_LSB_2_G;
  accY = ((float)ay) / ACC_LSB_2_G;
  accZ = ((float)az) / ACC_LSB_2_G;

  angleX = asin(accX) * RAD_2_DEG;
  angleY = asin(accY) * RAD_2_DEG;

    if (angleX!=angleX){
    angleX=prevX;
  }

  if (angleY!=angleY){
    angleY=prevY;
  }

  //avX[i]=angleX;
  //==============

 
    newAvgx = movingAvg(x_angle, &sumx, pos, len, angleX);
    newAvgy = movingAvg(y_angle, &sumy, pos, len, angleY);
    
    pos++;
    if (pos >= len){
      pos = 0;
    }
prevX=angleX; 
prevY=angleY;



  newAvgx=newAvgx + xcal;
  //Serial.print(newAvgx);
  //Serial.print("\t");
//
  newAvgy=newAvgy + ycal;
  //Serial.println(newAvgy);
//  Serial.print("\t");

  //Serial.println();




}


//====read sensor end=================
floatUnion axu,ayu;

axu.number = newAvgx;
ayu.number = newAvgy;

//==========  
  byte received = recvMsg (fAvailable, fRead, buf, sizeof (buf));
  
  if (received)
    {
      Serial.println(buf[0]);
    if (buf [0] != devID)
      return;  // not my device
      
    if (buf [1] != 2)
      return;  // unknown command
    

//=====

 byte msg [] = {
       170,  // device 0 (master)
       axu.bytes[0],  // turn light on command received
       axu.bytes[1],
       axu.bytes[2],
       axu.bytes[3],
       ayu.bytes[0],
       ayu.bytes[1],
       ayu.bytes[2],
       ayu.bytes[3],
       
    };
//===
    
    delay (1);  // give the master a moment to prepare to receive
    digitalWrite (ENABLE_PIN, HIGH);  // enable sending
    sendMsg (fWrite, msg, sizeof msg);

    digitalWrite (ENABLE_PIN, LOW);  // disable sending
    
   
   }   // end if something received
   
}  // end of loop
//====


//===========
void bmi160read(){

  Wire.beginTransmission(i2c_addr);
  Wire.write(byte(0x12)); // reg address for acc values
  Wire.endTransmission(false);
  Wire.requestFrom(i2c_addr, 6, true); // Read 6 registers total, each axis value is stored in 2 registers
  //For a range of +-2g, we need to divide the raw values by 16384, according to the datasheet

  ax = (Wire.read()  | Wire.read() << 8);// / 16384.0; // X-axis value
  ay = (Wire.read()  | Wire.read() << 8); // 16384.0; // Y-axis value
  az = (Wire.read()  | Wire.read() << 8); // / 16384.0; // Z-axis value


 Wire.endTransmission(true);  
  
}

void bmi160init(){
 
 Wire.beginTransmission(i2c_addr);
 Wire.write(byte(BMI160_RA_CMD)); 
 Wire.write(byte(BMI160_CMD_SOFT_RESET)); 
 Wire.endTransmission(true);  
 delay(10);
 Wire.beginTransmission(i2c_addr);
 Wire.write(byte(BMI160_RA_CMD)); 
 Wire.write(byte(BMI160_CMD_ACC_MODE_NORMAL)); 
 Wire.endTransmission(true);  
 delay(10);
  Serial.println("initialised...  "); 
  delay(1000); 
}
//==
