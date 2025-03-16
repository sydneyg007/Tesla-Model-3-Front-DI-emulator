//CanTX pin - gpio 22
//CanRX pin - gpio 23

//ver = 2: starts sending frames immediately - esp32 designed to be powered from Di wake pin
//V3: code cleanup and adding a lot more dynamics (based on 0x118 drive modes)

#include <esp32_can.h>

byte DIS_frontTorqueChecksum=0x07; // DIS_frontTorqueCounter+0x07
byte DIS_frontTorqueCounter=0x00;// ranges from 0x00 to 0x0F

byte checksum0x187=0x1C;  // counter0x187-0x44
byte counter0x187=0x60;// ranges from 0x60 to 0x6F
byte Dis_frontMode_driveMode=0x60;
byte Dis_frontMode_driveMode2=0xEA;
byte Dis_frontMode_driveModeByte0x03=0x00;
byte Dis_frontActivated0x187=0x60;
byte Dis_frontByte0x05=0xEA;
byte Dis_frontByte0x06=0x00;

byte counter0x2D5 = 0x01; 
byte checksum0x2D5 = 0x8E;
byte mode0x2D5 = 0x10;

CAN_FRAME outframe;

void Frames10MS()
  {
  static unsigned long timer10ms = millis();   

  if (millis() - timer10ms >= 10) 
    {
    timer10ms=millis();
    
    //0x186 (390) Dis_frontTorque (also on vehicle can)
    DIS_frontTorqueCounter ++; if (DIS_frontTorqueCounter == 0x10) {DIS_frontTorqueCounter = 0x00;}
    DIS_frontTorqueChecksum = 0x01 + 0x86 + DIS_frontTorqueCounter + 0x80 ;
    outframe.id = 0x186;            // Set our transmission address ID
    outframe.length = 8;            // Data payload 8 bytes
    outframe.extended = 0;          // Extended addresses - 0=11-bit 1=29bit
    outframe.rtr=0;                 //No request
    outframe.data.uint8 [0]= DIS_frontTorqueChecksum; //DIS_frontTorqueChecksum: 0|8@1+ (1,0) [0|0] ""
    outframe.data.uint8 [1]= DIS_frontTorqueCounter; //DIS_frontTorqueCounter: 8|4@1+ (1,0) [0|0] "", DIS_frontTorqueCommand: 12|13@1- (2,0) [0|0] "Nm"
    outframe.data.uint8 [2]=0x00; //DIS_frontTorqueCommand: 12|13@1- (2,0) [0|0] "Nm" (leave at zero)
    outframe.data.uint8 [3]=0x00; //DIS_frontTorqueCommand: 12|13@1- (2,0) [0|0] "Nm", DIS_frontTorqueActual: 27|13@1- (2,0) [0|0] "Nm" (leave at zero)
    outframe.data.uint8 [4]=0x00; //DIS_frontTorqueActual: 27|13@1- (2,0) [0|0] "Nm" (leave at zero)
    outframe.data.uint8 [5]=0x00; //DIS_frontAxleSpeed: 40|16@1- (0.1,0) [0|0] "RPM"
    outframe.data.uint8 [6]=0x80; //DIS_frontAxleSpeed: 40|16@1- (0.1,0) [0|0] "RPM" 
    outframe.data.uint8 [7]=0x00; //DIS_frontSlavePedalPos : 56|8@1+ (0.4,0) [0|100] "%" (leave at zero)
    Can0.sendFrame(outframe); 


    //0x187 (391)
    counter0x187++; if (counter0x187 > 0x0F) {counter0x187=0x00;}
    checksum0x187 = 0x01 + 0x87 + Dis_frontMode_driveMode + counter0x187 + Dis_frontMode_driveMode2 + Dis_frontMode_driveModeByte0x03 + Dis_frontActivated0x187 + Dis_frontByte0x05 + Dis_frontByte0x06;
    outframe.id = 0x187;            // Set our transmission address ID
    outframe.length = 8;            // Data payload 8 bytes
    outframe.extended = 0;          // Extended addresses - 0=11-bit 1=29bit
    outframe.rtr=0;                 //No request
    outframe.data.uint8 [0]= checksum0x187; //Dis_frontMode_checksum : 0|8@1+ (1,0) [0|255] ""
    outframe.data.uint8 [1]= Dis_frontMode_driveMode + counter0x187; //Dis_frontMode_driveMode : 12|4@1+ (1,0) [0|15] ""  6 "Park" 5 "Drive" 1 "Reverse", Dis_frontMode_driveModecounter : 8|4@1+ (1,0) [0|15] ""
    outframe.data.uint8 [2]= Dis_frontMode_driveMode2; //Dis_frontMode_driveMode2 : 16|8@1+ (1,0) [0|255] ""  0xEA "Park",  0xAA "Drive",   0xA6 "Reverse"
    outframe.data.uint8 [3]= Dis_frontMode_driveModeByte0x03; //starts at 0x00 in park, can got to 0x45 in Drive and 0x01 in reverse
    outframe.data.uint8 [4]= Dis_frontActivated0x187; // 0x60 when in park, when in drive or reverse bits 36 to 39 seem to randomly change and bits 32 to 35 = 0x04
    outframe.data.uint8 [5]= Dis_frontByte0x05; //starts at 0xEA then goes to around 0xC6 when drive engaged
    outframe.data.uint8 [6]= Dis_frontByte0x06; //something temperature related?? changes from 0x00 to higher value (0x40) when drive engaged
    outframe.data.uint8 [7]= 0x00; //always zero
    Can0.sendFrame(outframe); 

    

    // 0x2D5
    counter0x2D5++; if (counter0x2D5 > 0x0F) {counter0x2D5 = 0x00;} 
    checksum0x2D5 = 0x02 + 0xD5 + mode0x2D5 + counter0x2D5 + 0x14 + 0x88 + 0x03 + 0x02 +0x4C;
    outframe.id = 0x2D5;            // Set our transmission address ID
    outframe.length = 8;            // Data payload 8 bytes
    outframe.extended = 0;          // Extended addresses - 0=11-bit 1=29bit
    outframe.rtr=0;                 //No request
    outframe.data.uint8 [0]= checksum0x2D5; //Dis_front0x2D5_checksum : 0|8@1+ (1,0) [0|255] ""
    outframe.data.uint8 [1]= mode0x2D5 + counter0x2D5; //Dis_front0x2D5_mode : 12|4@1+ (1,0) [0|15] "" = 1 "off" 4 "on", Dis_front0x2D5_counter : 8|4@1+ (1,0) [0|15] ""
    outframe.data.uint8 [2]= 0x14; //stays at 0x14
    outframe.data.uint8 [3]= 0x88; //could be temperature
    outframe.data.uint8 [4]= 0x03; //stays at 0x03
    outframe.data.uint8 [5]= 0x00; //stays at 0x00
    outframe.data.uint8 [6]= 0x02; //stays at 0x02
    outframe.data.uint8 [7]= 0x4C; //varies between either 0x4C (mainly) or 0xCC
    Can0.sendFrame(outframe); 


    }
  }//end Frames 10ms-------------------------------------------------------------------



//////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup()
  {
  //Serial.begin(115200);  
  
  CAN_cfg.tx_pin_id = GPIO_NUM_22;
  CAN_cfg.rx_pin_id = GPIO_NUM_23;
  CAN0.begin(500000);// Initialize CAN0 and set baud rate.
  CAN0.watchFor();
  }//end void setup--------------------------------------------------------------------------------------------

void loop()
  {
  CAN_FRAME message;
  if (CAN0.read(message)) {

    if (message.id == 0x118){ //0x118 (280) DI_systemStatus
      byte DI_gear = ((message.data.byte[2] & B11100000) >> 5) ;   //DI_gear: 21|3@1+ (1,0) [0|0] "" 0 "INVALID" 1 "P" 2 "R" 3 "N" 4 "D" 7 "SNA";
      if (DI_gear==1){Dis_frontMode_driveMode=0x60; Dis_frontMode_driveMode2 = 0xEA; Dis_frontMode_driveModeByte0x03=0x00; Dis_frontActivated0x187=0x60; mode0x2D5=0x10; Dis_frontByte0x05=0xEA; Dis_frontByte0x06=0x00;}//Park
      if (DI_gear==2){Dis_frontMode_driveMode=0x10; Dis_frontMode_driveMode2 = 0xA6; Dis_frontMode_driveModeByte0x03=0x01;Dis_frontActivated0x187=0x64; mode0x2D5=0x40; Dis_frontByte0x05=0xC6; Dis_frontByte0x06=0x40;}//reverse
      //if (DI_gear==3){   }//neutral
      if (DI_gear==4){Dis_frontMode_driveMode=0x50; Dis_frontMode_driveMode2 = 0xAA; Dis_frontMode_driveModeByte0x03=0x45; Dis_frontActivated0x187=0x64; mode0x2D5=0x40; Dis_frontByte0x05=0xC6; Dis_frontByte0x06=0x40;}//Drive
      }//end if message 0x118

    }//end can0 read

  Frames10MS();

  }//end void loop---------------------------------------------------------------------------------------------
