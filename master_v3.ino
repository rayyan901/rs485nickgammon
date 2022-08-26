#include "RS485_protocol.h"
#include <SoftwareSerial.h>

const byte ENABLE_PIN = 4;
const byte LED_PIN = 13;

const unsigned int baud=19200;
SoftwareSerial rs485 (2, 3);  // receive pin, transmit pin


//global var==========

float angleX, angleY;
bool recstatus=0;

// callback routines
  
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
  pinMode (LED_PIN, OUTPUT);  // built-in LED
}  // end of setup
  
byte old_level = 0;

void loop()
{

byte deviceID;

      //deviceID=1;

for(byte i=1;i<4;i++){

      
  // assemble message
  byte msg [] = { 
     deviceID=i,    // device 1
     2    // turn light on
     //level // to what level
  };

  // send to slave  
  digitalWrite (ENABLE_PIN, HIGH);  // enable sending
  sendMsg (fWrite, msg, sizeof msg);
  digitalWrite (ENABLE_PIN, LOW);  // disable sending

  // receive response  
  receiveRS485();
  if(recstatus){
  Serial.print("sensor ");
  Serial.print(deviceID);
  Serial.print(" : ");  
  Serial.print(angleX);
  Serial.print("    ");
  Serial.println(angleY);
  } else {
    Serial.print(" received error from: ");
    Serial.println(deviceID);
  }
  //===





  
delay(500);
}
}  // end of loop

//====
float bytesToFloat(uint8_t *bytes, bool big_endian) {
    float f;
    uint8_t *f_ptr = (uint8_t *) &f;
    if (big_endian) {
        f_ptr[3] = bytes[0];
        f_ptr[2] = bytes[1];
        f_ptr[1] = bytes[2];
        f_ptr[0] = bytes[3];
    } else {
        f_ptr[3] = bytes[3];
        f_ptr[2] = bytes[2];
        f_ptr[1] = bytes[1];
        f_ptr[0] = bytes[0];
    }
    return f;
}

//===
void receiveRS485(){
byte buf [10];
  byte received = recvMsg (fAvailable, fRead, buf, sizeof buf);
  
  digitalWrite (LED_PIN, received == 0);  // turn on LED if error    
  
  // only send once per successful change
  if (received) {
   recstatus=1; 
byte recX[4];
recX[0]=buf[1];
recX[1]=buf[2];
recX[2]=buf[3];
recX[3]=buf[4];
float newfloatX=bytesToFloat(recX,0);
angleX = newfloatX;
//Serial.print(newfloatX);
 byte recY[4];
         recY[0]=buf[5];
         recY[1]=buf[6];
         recY[2]=buf[7];
         recY[3]=buf[8];


float newfloatY=bytesToFloat(recY,0);
angleY = newfloatY;

  } else recstatus=0;

//  received = recvMsg (fAvailable, fRead, buf, sizeof buf);
//  
//  digitalWrite (LED_PIN, received == 0);  // turn on LED if error    
  
//  if (received) {
//     recstatus=1;
//   

//Serial.print("    ");
//Serial.println(newfloatY);

   // old_level = level;
  //} //else recstatus=0;
}
