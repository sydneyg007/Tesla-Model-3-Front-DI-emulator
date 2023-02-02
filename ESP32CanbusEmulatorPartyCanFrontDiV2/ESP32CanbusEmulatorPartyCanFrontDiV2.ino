//CanTX pin - gpio 22
//CanRX pin - gpio 23

//ver = 2: starts sending frames immediately - esp32 designed to be powered from Di wake pin

#include <esp32_can.h>

byte DIS_frontTorqueChecksum=0x07; // DIS_frontTorqueCounter+0x07
byte DIS_frontTorqueCounter=0x00;// ranges from 0x00 to 0x0F

byte checksum0x187=0x1C;  // counter0x187-0x44
byte counter0x187=0x60;// ranges from 0x60 to 0x6F

byte counter0x2D5=0x40; //0x40 to 0x4F, increment by 0x01
byte checksum0x2D5=0x8E;// 0x8E to 0x9D

byte startTrigger=0;

CAN_FRAME outframe;

void Frames10MS()
  {
  static unsigned long timer10ms = millis();   

  if (millis() - timer10ms >= 10) 
    {
    timer10ms=millis();
    
    // 0x186 (Dis_frontTorque)
    outframe.id = 0x186;            // Set our transmission address ID
    outframe.length = 8;            // Data payload 8 bytes
    outframe.extended = 0;          // Extended addresses - 0=11-bit 1=29bit
    outframe.rtr=0;                 //No request
    outframe.data.uint8 [0]= DIS_frontTorqueChecksum;
    outframe.data.uint8 [1]= DIS_frontTorqueCounter; 
    outframe.data.uint8 [2]=0x00;
    outframe.data.uint8 [3]=0x00;
    outframe.data.uint8 [4]=0x00;
    outframe.data.uint8 [5]=0x00;
    outframe.data.uint8 [6]=0x80;
    outframe.data.uint8 [7]=0x00;
    Can0.sendFrame(outframe); 
    //Byte 1 calc
    DIS_frontTorqueCounter ++;
    if (DIS_frontTorqueCounter ==0x10)
      {DIS_frontTorqueCounter =0x00;}
    //Byte 0 calc
    DIS_frontTorqueChecksum=DIS_frontTorqueCounter+0x07;

    // 0x187 
    outframe.id = 0x187;            // Set our transmission address ID
    outframe.length = 8;            // Data payload 8 bytes
    outframe.extended = 0;          // Extended addresses - 0=11-bit 1=29bit
    outframe.rtr=0;                 //No request
    outframe.data.uint8 [0]= checksum0x187;
    outframe.data.uint8 [1]= counter0x187; 
    outframe.data.uint8 [2]=0xEA;
    outframe.data.uint8 [3]=0x00;
    outframe.data.uint8 [4]=0x60;
    outframe.data.uint8 [5]=0xEA;
    outframe.data.uint8 [6]=0x00;
    outframe.data.uint8 [7]=0x00;
    Can0.sendFrame(outframe); 
    //Byte 1 calc
    counter0x187++;
    if (counter0x187==0x70)
    {counter0x187=0x60;}
    //Byte 0 calc
    checksum0x187=counter0x187-0x44;

    // 0x2D5
    outframe.id = 0x2D5;            // Set our transmission address ID
    outframe.length = 8;            // Data payload 8 bytes
    outframe.extended = 0;          // Extended addresses - 0=11-bit 1=29bit
    outframe.rtr=0;                 //No request
    outframe.data.uint8 [0]= checksum0x2D5;
    outframe.data.uint8 [1]= counter0x2D5; 
    outframe.data.uint8 [2]=0x14;
    outframe.data.uint8 [3]=0x92;
    outframe.data.uint8 [4]=0x03;
    outframe.data.uint8 [5]=0x00;
    outframe.data.uint8 [6]=0x02;
    outframe.data.uint8 [7]=0xCC;
    Can0.sendFrame(outframe); 
    //Byte 1 calc
    counter0x2D5 = counter0x2D5 +0x01;
    if (counter0x2D5 >0x4F)
      {counter0x2D5=0x40;}
    //Byte 0 calc
    checksum0x2D5=checksum0x2D5+0x01;
    if(checksum0x2D5>0x9D)
      {checksum0x2D5=0x8E;}

    }
  }//end Frames 10ms-------------------------------------------------------------------



//////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup()
  {
  //Serial.begin(115200);  
  
  CAN_cfg.tx_pin_id = GPIO_NUM_22;
  CAN_cfg.rx_pin_id = GPIO_NUM_23;
  CAN0.begin(500000);// Initialize CAN0 and set baud rate.
  
  }//end void setup--------------------------------------------------------------------------------------------

void loop()
  {
  Frames10MS();

  }//end void loop---------------------------------------------------------------------------------------------
