
#include <SoftwareSerial.h>
#include "RS485_protocol.h"

SoftwareSerial rs485 (6, 7);  // receive pin, transmit pin
const byte ENABLE_PIN = 3;
const unsigned int baud=19200;

float angleX= -34.87;

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
  char angle[7];
  byte level = random(255);
  byte devID=1;
//=====

floatUnion myFloat;
floatUnion ax,ay;
myFloat.number = -123.456; // Assign a number to the float
ax.number =36.87;
ay.number = -63.17;
//for (int i=0; i<4; i++)
//{
//  Serial.print(myFloat.bytes[i]); // Print the hex representation of the float
//  Serial.print(' ');
//}
//==========  
  byte received = recvMsg (fAvailable, fRead, buf, sizeof (buf));
  
  if (received)
    {
      Serial.println(buf[0]);
    if (buf [0] != devID)
      return;  // not my device
      
    if (buf [1] != 2)
      return;  // unknown command
    
//    byte msg [] = {
//       0,  // device 0 (master)
//       3,  // turn light on command received
//       level,
//    };
//=====

 byte msg [] = {
       170,  // device 0 (master)
       ax.bytes[0],  // turn light on command received
       ax.bytes[1],
       ax.bytes[2],
       ax.bytes[3],
       ay.bytes[0],
       ay.bytes[1],
       ay.bytes[2],
       ay.bytes[3],
       
    };
//===
    
    delay (1);  // give the master a moment to prepare to receive
    digitalWrite (ENABLE_PIN, HIGH);  // enable sending
    sendMsg (fWrite, msg, sizeof msg);
//    sendMsg (fWrite, ax.bytes, sizeof ax.bytes);
//    sendMsg (fWrite, ay.bytes, sizeof ay.bytes);
    digitalWrite (ENABLE_PIN, LOW);  // disable sending
    
    //analogWrite (11, buf [2]);  // set light level
   }   // end if something received
   
}  // end of loop
//====
