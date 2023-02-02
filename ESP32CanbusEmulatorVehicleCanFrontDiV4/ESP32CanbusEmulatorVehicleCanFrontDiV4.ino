//V2 seperating 1000ms frames out and varying timing slightly so all get sent.
//ver = 4: starts sending frames immediately - esp32 designed to be powered from Di wake pin

//CanTX pin - gpio 22
//CanRX pin - gpio 23

#include <esp32_can.h>

byte DIS_frontTorqueChecksum=0x07; // DIS_frontTorqueCounter+0x07
byte DIS_frontTorqueCounter=0x00;// ranges from 0x00 to 0x0F

byte counter0x1D5=0x00; //0x00 to 0xE0, increment by 0x20
byte checksum0x1D5=0xF7;

byte counter0x27A=0xC0; //0xC0 to 0xCF, increment by 0x01
byte checksum0x27A=0x38;// 0x38 to 0x47

byte repeat0x757=0;

CAN_FRAME outframe;

void Frames10MS()
  {
  static unsigned long timer10ms = millis();   

  if (millis() - timer10ms >= 10) 
    {
    timer10ms=millis();
    
    // 0x1D5
    outframe.id = 0x1D5;            // Set our transmission address ID
    outframe.length = 8;            // Data payload 8 bytes
    outframe.extended = 0;          // Extended addresses - 0=11-bit 1=29bit
    outframe.rtr=0;                 //No request
    outframe.data.uint8 [0]= 0x21;
    outframe.data.uint8 [1]= 0x00; 
    outframe.data.uint8 [2]=0x00;
    outframe.data.uint8 [3]=0x00;
    outframe.data.uint8 [4]=0x00;
    outframe.data.uint8 [5]=0x00;
    outframe.data.uint8 [6]= counter0x1D5;
    outframe.data.uint8 [7]= checksum0x1D5;
    Can0.sendFrame(outframe); 
    //Byte 6 calc
    counter0x1D5=counter0x1D5+0x20;
    if (counter0x1D5>0xE0)
        {counter0x1D5=0x00;}
    //Byte 7 calc
    checksum0x1D5= checksum0x1D5+0x20;
    if (checksum0x1D5>0xF7)
      {checksum0x1D5=0x17;}

    // 0x2E5
    outframe.id = 0x2E5;            // Set our transmission address ID
    outframe.length = 8;            // Data payload 8 bytes
    outframe.extended = 0;          // Extended addresses - 0=11-bit 1=29bit
    outframe.rtr=0;                 //No request
    outframe.data.uint8 [0]=0x00;
    outframe.data.uint8 [1]= 0x00;
    outframe.data.uint8 [2]=0x03;
    outframe.data.uint8 [3]=0x0D;
    outframe.data.uint8 [4]=0x00;
    outframe.data.uint8 [5]=0x00;
    outframe.data.uint8 [6]=0xDD;
    outframe.data.uint8 [7]=0x00;
    Can0.sendFrame(outframe); 




    }
  }//end Frames 10ms-------------------------------------------------------------------


void Frames100MS()
  {
  static unsigned long timer100ms = millis();   

  if (millis() - timer100ms >= 100) 
    {
    timer100ms=millis();
    //FDIS 0x186 (vehicle @100ms)
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
      { DIS_frontTorqueCounter =0x00;}
    //Byte 0 calc
    DIS_frontTorqueChecksum=DIS_frontTorqueCounter+0x07;

    // 0x195
    outframe.id = 0x195;            // Set our transmission address ID
    outframe.length = 8;            // Data payload 8 bytes
    outframe.extended = 0;          // Extended addresses - 0=11-bit 1=29bit
    outframe.rtr=0;                 //No request
    outframe.data.uint8 [0]= 0x00;
    outframe.data.uint8 [1]= 0x08; 
    outframe.data.uint8 [2]=0x08;
    outframe.data.uint8 [3]=0x00;
    outframe.data.uint8 [4]=0x28;
    outframe.data.uint8 [5]=0x00;

    outframe.data.uint8 [6]=0x00;
    outframe.data.uint8 [7]=0x00;
    Can0.sendFrame(outframe); 

    // 0x1A5
    outframe.id = 0x1A5;            // Set our transmission address ID
    outframe.length = 8;            // Data payload 8 bytes
    outframe.extended = 0;          // Extended addresses - 0=11-bit 1=29bit
    outframe.rtr=0;                 //No request
    outframe.data.uint8 [0]= 0xE4;
    outframe.data.uint8 [1]= 0x02; 
    outframe.data.uint8 [2]=0x00;
    outframe.data.uint8 [3]=0x00;
    outframe.data.uint8 [4]=0x00;
    outframe.data.uint8 [5]=0xE2;
    outframe.data.uint8 [6]=0x03;
    outframe.data.uint8 [7]=0x00;
    Can0.sendFrame(outframe); 

    // 0x27A
    outframe.id = 0x27A;            // Set our transmission address ID
    outframe.length = 8;            // Data payload 8 bytes
    outframe.extended = 0;          // Extended addresses - 0=11-bit 1=29bit
    outframe.rtr=0;                 //No request
    outframe.data.uint8 [0]= checksum0x27A;
    outframe.data.uint8 [1]= counter0x27A; 
    outframe.data.uint8 [2]=0xFC;
    outframe.data.uint8 [3]=0x00;
    outframe.data.uint8 [4]=0x00;
    outframe.data.uint8 [5]=0x00;
    outframe.data.uint8 [6]=0x00;
    outframe.data.uint8 [7]=0x00;
    Can0.sendFrame(outframe); 
    //Byte 1 calc
    counter0x27A = counter0x27A +0x01;
    if (counter0x27A >0xCF)
      {counter0x27A =0xC0;}
    //Byte 0 calc
    checksum0x27A = checksum0x27A +0x01;
    if (checksum0x27A >0x47)
      {checksum0x27A =0x38;}

    //0x396 (DI_frontOilPump)
    outframe.id = 0x396;            // Set our transmission address ID
    outframe.length = 8;            // Data payload 8 bytes
    outframe.extended = 0;          // Extended addresses - 0=11-bit 1=29bit
    outframe.rtr=0;                 //No request
    outframe.data. uint8 [0]=0x00;
    outframe.data. uint8 [1]=0x11;
    outframe.data. uint8 [2]=0x00;
    outframe.data. uint8 [3]= 0x40;
    outframe.data. uint8 [4]= 0xFF;
    outframe.data. uint8 [5]= 0x00;
    outframe.data. uint8 [6]= 0x00;
    outframe.data. uint8 [7]= 0x7D;
    Can0.sendFrame(outframe);

    //0x757
    outframe.id = 0x757;            // Set our transmission address ID
    outframe.length = 8;            // Data payload 8 bytes
    outframe.extended = 0;          // Extended addresses - 0=11-bit 1=29bit
    outframe.rtr=0;                 //No request
    outframe.data. uint8 [0]=0x32;
    outframe.data. uint8 [1]=0x0E;
    outframe.data. uint8 [2]=0x00;
    outframe.data. uint8 [3]=0x00;
    outframe.data. uint8 [4]=0x00;
    outframe.data. uint8 [5]=0x00;
    outframe.data. uint8 [6]=0x00;
    outframe.data. uint8 [7]=0x00;
    Can0.sendFrame(outframe);
    repeat0x757++;
    if (repeat0x757==10)
      {
      outframe.id = 0x757;            // Set our transmission address ID
      outframe.length = 8;            // Data payload 8 bytes
      outframe.extended = 0;          // Extended addresses - 0=11-bit 1=29bit
      outframe.rtr=0;                 //No request
      outframe.data. uint8 [0]=0x46;
      outframe.data. uint8 [1]=0x3C;
      outframe.data. uint8 [2]=0x3C;
      outframe.data. uint8 [3]=0x3C;
      outframe.data. uint8 [4]=0x3D;
      outframe.data. uint8 [5]=0x58;
      outframe.data. uint8 [6]=0x42;
      outframe.data. uint8 [7]=0x41;
      Can0.sendFrame(outframe);

      outframe.id = 0x757;            // Set our transmission address ID
      outframe.length = 8;            // Data payload 8 bytes
      outframe.extended = 0;          // Extended addresses - 0=11-bit 1=29bit
      outframe.rtr=0;                 //No request
      outframe.data. uint8 [0]=0x48;
      outframe.data. uint8 [1]=0x3B;
      outframe.data. uint8 [2]=0x3B;
      outframe.data. uint8 [3]=0x00;
      outframe.data. uint8 [4]=0x00;
      outframe.data. uint8 [5]=0xC0;
      outframe.data. uint8 [6]=0x0E;
      outframe.data. uint8 [7]=0x00;
      Can0.sendFrame(outframe);

      repeat0x757=0;
      }
    }
  }//end Frames 100ms-------------------------------------------------------------------

void Frames1000MS1()
  {
  static unsigned long timer1000ms1 = millis();
  if (millis() - timer1000ms1 >= 1004) 
    {
    timer1000ms1=millis();
    
    // 0x304
    outframe.id = 0x304;            // Set our transmission address ID
    outframe.length = 8;            // Data payload 8 bytes
    outframe.extended = 0;          // Extended addresses - 0=11-bit 1=29bit
    outframe.rtr=0;                 //No request
    outframe.data.uint8 [0]=0x00;
    outframe.data.uint8 [1]= 0x00;
    outframe.data.uint8 [2]=0x00;
    outframe.data.uint8 [3]=0x00;
    outframe.data.uint8 [4]=0x00;
    outframe.data.uint8 [5]=0x00;
    outframe.data.uint8 [6]=0x00;
    outframe.data.uint8 [7]=0x00;
    Can0.sendFrame(outframe); 
            
    //0x316
    //index 1 
    outframe.id = 0x316;            // Set our transmission address ID
    outframe.length = 8;            // Data payload 8 bytes
    outframe.extended = 0;          // Extended addresses - 0=11-bit 1=29bit
    outframe.rtr=0;                 //No request
    outframe.data. uint8 [0]=0x0A;
    outframe.data. uint8 [1]=0x01;
    outframe.data. uint8 [2]=0x4B;
    outframe.data. uint8 [3]= 0x32;
    outframe.data. uint8 [4]= 0xFC;
    outframe.data. uint8 [5]= 0x00;
    outframe.data. uint8 [6]= 0x09;
    outframe.data. uint8 [7]= 0x00;
    Can0.sendFrame(outframe);

    //index 2
    outframe.id = 0x316;            // Set our transmission address ID
    outframe.length = 8;            // Data payload 8 bytes
    outframe.extended = 0;          // Extended addresses - 0=11-bit 1=29bit
    outframe.rtr=0;                 //No request
    outframe.data. uint8 [0]=0x0B;
    outframe.data. uint8 [1]=0x00;
    outframe.data. uint8 [2]=0x1C;
    outframe.data. uint8 [3]= 0x51;
    outframe.data. uint8 [4]= 0x01;
    outframe.data. uint8 [5]= 0x00;
    outframe.data. uint8 [6]= 0x00;
    outframe.data. uint8 [7]= 0x00;
    Can0.sendFrame(outframe);

    //index 3
    outframe.id = 0x316;            // Set our transmission address ID
    outframe.length = 8;            // Data payload 8 bytes
    outframe.extended = 0;          // Extended addresses - 0=11-bit 1=29bit
    outframe.rtr=0;                 //No request
    outframe.data. uint8 [0]=0x0D;
    outframe.data. uint8 [1]=0x00;
    outframe.data. uint8 [2]=0x00;
    outframe.data. uint8 [3]= 0x00;
    outframe.data. uint8 [4]= 0xA8;
    outframe.data. uint8 [5]= 0xB7;
    outframe.data. uint8 [6]= 0x3F;
    outframe.data. uint8 [7]= 0x79;
    Can0.sendFrame(outframe);

    //index 4
    outframe.id = 0x316;            // Set our transmission address ID
    outframe.length = 8;            // Data payload 8 bytes
    outframe.extended = 0;          // Extended addresses - 0=11-bit 1=29bit
    outframe.rtr=0;                 //No request
    outframe.data. uint8 [0]=0x11;
    outframe.data. uint8 [1]=0x00;
    outframe.data. uint8 [2]=0x00;
    outframe.data. uint8 [3]= 0x00;
    outframe.data. uint8 [4]= 0x00;
    outframe.data. uint8 [5]= 0x00;
    outframe.data. uint8 [6]= 0x00;
    outframe.data. uint8 [7]= 0x00;
    Can0.sendFrame(outframe);

    //index 5
    outframe.id = 0x316;            // Set our transmission address ID
    outframe.length = 8;            // Data payload 8 bytes
    outframe.extended = 0;          // Extended addresses - 0=11-bit 1=29bit
    outframe.rtr=0;                 //No request
    outframe.data. uint8 [0]=0x12;
    outframe.data. uint8 [1]=0x16;
    outframe.data. uint8 [2]=0x48;
    outframe.data. uint8 [3]= 0xB4;
    outframe.data. uint8 [4]= 0x7A;
    outframe.data. uint8 [5]= 0x66;
    outframe.data. uint8 [6]= 0x41;
    outframe.data. uint8 [7]= 0xFD;
    Can0.sendFrame(outframe);

    //index 6
    outframe.id = 0x316;            // Set our transmission address ID
    outframe.length = 8;            // Data payload 8 bytes
    outframe.extended = 0;          // Extended addresses - 0=11-bit 1=29bit
    outframe.rtr=0;                 //No request
    outframe.data. uint8 [0]=0x13;
    outframe.data. uint8 [1]=0x03;
    outframe.data. uint8 [2]=0x00;
    outframe.data. uint8 [3]= 0x00;
    outframe.data. uint8 [4]= 0x00;
    outframe.data. uint8 [5]= 0x00;
    outframe.data. uint8 [6]= 0x00;
    outframe.data. uint8 [7]= 0x00;
    Can0.sendFrame(outframe);

    }
  }//end Frames 1000ms1-------------------------------------------------------------------

void Frames1000MS2()
  {
  static unsigned long timer1000ms2 = millis();
  if (millis() - timer1000ms2 >= 1002) 
    {
    timer1000ms2=millis();
    //index 7
    outframe.id = 0x316;            // Set our transmission address ID
    outframe.length = 8;            // Data payload 8 bytes
    outframe.extended = 0;          // Extended addresses - 0=11-bit 1=29bit
    outframe.rtr=0;                 //No request
    outframe.data. uint8 [0]=0x14;
    outframe.data. uint8 [1]=0x06;
    outframe.data. uint8 [2]=0x00;
    outframe.data. uint8 [3]= 0x00;
    outframe.data. uint8 [4]= 0xED;
    outframe.data. uint8 [5]= 0x43;
    outframe.data. uint8 [6]= 0x34;
    outframe.data. uint8 [7]= 0x4A;
    Can0.sendFrame(outframe);

    //index 8
    outframe.id = 0x316;            // Set our transmission address ID
    outframe.length = 8;            // Data payload 8 bytes
    outframe.extended = 0;          // Extended addresses - 0=11-bit 1=29bit
    outframe.rtr=0;                 //No request
    outframe.data. uint8 [0]=0x15;
    outframe.data. uint8 [1]=0x01;
    outframe.data. uint8 [2]=0x4B;
    outframe.data. uint8 [3]= 0x32;
    outframe.data. uint8 [4]= 0xBF;
    outframe.data. uint8 [5]= 0x97;
    outframe.data. uint8 [6]= 0x29;
    outframe.data. uint8 [7]= 0xDB;
    Can0.sendFrame(outframe);

    //0x356
    outframe.id = 0x356;            // Set our transmission address ID
    outframe.length = 8;            // Data payload 8 bytes
    outframe.extended = 0;          // Extended addresses - 0=11-bit 1=29bit
    outframe.rtr=0;                 //No request
    outframe.data. uint8 [0]=0x00;
    outframe.data. uint8 [1]=0x00;
    outframe.data. uint8 [2]=0x00;
    outframe.data. uint8 [3]= 0x00;
    outframe.data. uint8 [4]= 0x00;
    outframe.data. uint8 [5]= 0x00;
    outframe.data. uint8 [6]= 0x00;
    outframe.data. uint8 [7]= 0x00;
    Can0.sendFrame(outframe);

    //0x357
    outframe.id = 0x357;            // Set our transmission address ID
    outframe.length = 8;            // Data payload 8 bytes
    outframe.extended = 0;          // Extended addresses - 0=11-bit 1=29bit
    outframe.rtr=0;                 //No request
    outframe.data. uint8 [0]=0x00;
    outframe.data. uint8 [1]=0x00;
    outframe.data. uint8 [2]=0x00;
    outframe.data. uint8 [3]= 0x00;
    outframe.data. uint8 [4]= 0x00;
    outframe.data. uint8 [5]= 0x00;
    outframe.data. uint8 [6]= 0x00;
    outframe.data. uint8 [7]= 0x00;
    Can0.sendFrame(outframe);

    //0x35A (DI_alertMatrix3)
    outframe.id = 0x35A;            // Set our transmission address ID
    outframe.length = 8;            // Data payload 8 bytes
    outframe.extended = 0;          // Extended addresses - 0=11-bit 1=29bit
    outframe.rtr=0;                 //No request
    outframe.data. uint8 [0]=0x00;
    outframe.data. uint8 [1]=0x00;
    outframe.data. uint8 [2]=0x00;
    outframe.data. uint8 [3]= 0x00;
    outframe.data. uint8 [4]= 0x00;
    outframe.data. uint8 [5]= 0x00;
    outframe.data. uint8 [6]= 0x00;
    outframe.data. uint8 [7]= 0x00;
    Can0.sendFrame(outframe);

    //0x35B (DI_alertMatrix4)
    outframe.id = 0x35B;            // Set our transmission address ID
    outframe.length = 8;            // Data payload 8 bytes
    outframe.extended = 0;          // Extended addresses - 0=11-bit 1=29bit
    outframe.rtr=0;                 //No request
    outframe.data. uint8 [0]=0x00;
    outframe.data. uint8 [1]=0x00;
    outframe.data. uint8 [2]=0x00;
    outframe.data. uint8 [3]= 0x00;
    outframe.data. uint8 [4]= 0x00;
    outframe.data. uint8 [5]= 0x00;
    outframe.data. uint8 [6]= 0x00;
    outframe.data. uint8 [7]= 0x00;
    Can0.sendFrame(outframe);

    //0x376 (DI_frontInverterTemperature)
    outframe.id = 0x376;            // Set our transmission address ID
    outframe.length = 8;            // Data payload 8 bytes
    outframe.extended = 0;          // Extended addresses - 0=11-bit 1=29bit
    outframe.rtr=0;                 //No request
    outframe.data. uint8 [0]=0x3C;
    outframe.data. uint8 [1]=0x3D;
    outframe.data. uint8 [2]=0x3D;
    outframe.data. uint8 [3]= 0x50;
    outframe.data. uint8 [4]= 0x3E;
    outframe.data. uint8 [5]= 0x8F;
    outframe.data. uint8 [6]= 0x7D;
    outframe.data. uint8 [7]= 0x03;
    Can0.sendFrame(outframe);

    }
  }//end Frames 1000ms2-------------------------------------------------------------------    

void Frames1000MS3()
  {
  static unsigned long timer1000ms3 = millis();
  if (millis() - timer1000ms3 >= 1000) 
    {
    timer1000ms3=millis();
    //0x3D6 
    outframe.id = 0x3D6;            // Set our transmission address ID
    outframe.length = 8;            // Data payload 8 bytes
    outframe.extended = 0;          // Extended addresses - 0=11-bit 1=29bit
    outframe.rtr=0;                 //No request
    outframe.data. uint8 [0]=0x3B;
    outframe.data. uint8 [1]=0x3B;
    outframe.data. uint8 [2]=0x3B;
    outframe.data. uint8 [3]= 0x3B;
    outframe.data. uint8 [4]= 0x3A;
    outframe.data. uint8 [5]= 0x90;
    outframe.data. uint8 [6]= 0x00;
    outframe.data. uint8 [7]= 0x4D;
    Can0.sendFrame(outframe);

    //0x524 
    outframe.id = 0x524;            // Set our transmission address ID
    outframe.length = 7;            // Data payload 8 bytes
    outframe.extended = 0;          // Extended addresses - 0=11-bit 1=29bit
    outframe.rtr=0;                 //No request
    outframe.data. uint8 [0]=0x64;
    outframe.data. uint8 [1]=0x03;
    outframe.data. uint8 [2]=0x08;
    outframe.data. uint8 [3]= 0x0A;
    outframe.data. uint8 [4]= 0x18;
    outframe.data. uint8 [5]= 0xB0;
    outframe.data. uint8 [6]= 0x02;
    Can0.sendFrame(outframe);

    //0x556 
    outframe.id = 0x556;            // Set our transmission address ID
    outframe.length =5;            // Data payload 8 bytes
    outframe.extended = 0;          // Extended addresses - 0=11-bit 1=29bit
    outframe.rtr=0;                 //No request
    outframe.data. uint8 [0]=0x3D;
    outframe.data. uint8 [1]=0x3D;
    outframe.data. uint8 [2]=0x00;
    outframe.data. uint8 [3]=0x3D;
    outframe.data. uint8 [4]=0xFF;
    Can0.sendFrame(outframe);

    //0x557 (DI_frontThermalControl)
    outframe.id = 0x557;            // Set our transmission address ID
    outframe.length = 4;            // Data payload 8 bytes
    outframe.extended = 0;          // Extended addresses - 0=11-bit 1=29bit
    outframe.rtr=0;                 //No request
    outframe.data. uint8 [0]=0x8C;
    outframe.data. uint8 [1]=0x8C;
    outframe.data. uint8 [2]=0x00;
    outframe.data. uint8 [3]= 0x00;
    Can0.sendFrame(outframe);

    //0x656 (DI_Info)
    //Index 1 
    outframe.id = 0x656;            // Set our transmission address ID
    outframe.length = 8;            // Data payload 8 bytes
    outframe.extended = 0;          // Extended addresses - 0=11-bit 1=29bit
    outframe.rtr=0;                 //No request
    outframe.data. uint8 [0]=0x0A;
    outframe.data. uint8 [1]=0x01;
    outframe.data. uint8 [2]=0x4B;
    outframe.data. uint8 [3]= 0x32;
    outframe.data. uint8 [4]=0xFC;
    outframe.data. uint8 [5]=0x00;
    outframe.data. uint8 [6]=0x05;
    outframe.data. uint8 [7]= 0x00;
    Can0.sendFrame(outframe);

    //Index 2 
    outframe.id = 0x656;            // Set our transmission address ID
    outframe.length = 8;            // Data payload 8 bytes
    outframe.extended = 0;          // Extended addresses - 0=11-bit 1=29bit
    outframe.rtr=0;                 //No request
    outframe.data. uint8 [0]=0x0B;
    outframe.data. uint8 [1]=0x00;
    outframe.data. uint8 [2]=0x1C;
    outframe.data. uint8 [3]= 0x51;
    outframe.data. uint8 [4]=0x01;
    outframe.data. uint8 [5]=0x00;
    outframe.data. uint8 [6]=0x00;
    outframe.data. uint8 [7]= 0x00;
    Can0.sendFrame(outframe);

    //Index 3 
    outframe.id = 0x656;            // Set our transmission address ID
    outframe.length = 8;            // Data payload 8 bytes
    outframe.extended = 0;          // Extended addresses - 0=11-bit 1=29bit
    outframe.rtr=0;                 //No request
    outframe.data. uint8 [0]=0x0D;
    outframe.data. uint8 [1]=0x00;
    outframe.data. uint8 [2]=0x00;
    outframe.data. uint8 [3]= 0x00;
    outframe.data. uint8 [4]=0xBF;
    outframe.data. uint8 [5]=0x97;
    outframe.data. uint8 [6]=0x29;
    outframe.data. uint8 [7]= 0xDB;
    Can0.sendFrame(outframe);

    }
  }//end Frames 1000ms3------------------------------------------------------------------- 

void Frames1000MS4()
  {
  static unsigned long timer1000ms4 = millis();
  if (millis() - timer1000ms4 >= 998) 
    {
    timer1000ms4=millis();
    //Index 4
    outframe.id = 0x656;            // Set our transmission address ID
    outframe.length = 8;            // Data payload 8 bytes
    outframe.extended = 0;          // Extended addresses - 0=11-bit 1=29bit
    outframe.rtr=0;                 //No request
    outframe.data. uint8 [0]=0x10;
    outframe.data. uint8 [1]=0x01;
    outframe.data. uint8 [2]=0xC4;
    outframe.data. uint8 [3]= 0x27;
    outframe.data. uint8 [4]=0xCC;
    outframe.data. uint8 [5]=0x6A;
    outframe.data. uint8 [6]=0xFF;
    outframe.data. uint8 [7]=0x00;
    Can0.sendFrame(outframe);

    //Index 5
    outframe.id = 0x656;            // Set our transmission address ID
    outframe.length = 8;            // Data payload 8 bytes
    outframe.extended = 0;          // Extended addresses - 0=11-bit 1=29bit
    outframe.rtr=0;                 //No request
    outframe.data. uint8 [0]=0x11;
    outframe.data. uint8 [1]=0x00;
    outframe.data. uint8 [2]=0x00;
    outframe.data. uint8 [3]= 0x00;
    outframe.data. uint8 [4]=0x00;
    outframe.data. uint8 [5]=0x00;
    outframe.data. uint8 [6]=0x00;
    outframe.data. uint8 [7]=0x00;
    Can0.sendFrame(outframe);

    //Index 6
    outframe.id = 0x656;            // Set our transmission address ID
    outframe.length = 8;            // Data payload 8 bytes
    outframe.extended = 0;          // Extended addresses - 0=11-bit 1=29bit
    outframe.rtr=0;                 //No request
    outframe.data. uint8 [0]=0x12;
    outframe.data. uint8 [1]=0x16;
    outframe.data. uint8 [2]=0x48;
    outframe.data. uint8 [3]= 0xB4;
    outframe.data. uint8 [4]=0x7A;
    outframe.data. uint8 [5]=0x66;
    outframe.data. uint8 [6]=0x41;
    outframe.data. uint8 [7]=0xFD;
    Can0.sendFrame(outframe);

    //Index 7
    outframe.id = 0x656;            // Set our transmission address ID
    outframe.length = 8;            // Data payload 8 bytes
    outframe.extended = 0;          // Extended addresses - 0=11-bit 1=29bit
    outframe.rtr=0;                 //No request
    outframe.data. uint8 [0]=0x13;
    outframe.data. uint8 [1]=0x03;
    outframe.data. uint8 [2]=0x00;
    outframe.data. uint8 [3]= 0x00;
    outframe.data. uint8 [4]=0x00;
    outframe.data. uint8 [5]=0x00;
    outframe.data. uint8 [6]=0x00;
    outframe.data. uint8 [7]=0x00;
    Can0.sendFrame(outframe);

    //Index 8
    outframe.id = 0x656;            // Set our transmission address ID
    outframe.length = 8;            // Data payload 8 bytes
    outframe.extended = 0;          // Extended addresses - 0=11-bit 1=29bit
    outframe.rtr=0;                 //No request
    outframe.data. uint8 [0]=0x14;
    outframe.data. uint8 [1]=0x05;
    outframe.data. uint8 [2]=0x00;
    outframe.data. uint8 [3]= 0x00;
    outframe.data. uint8 [4]=0x10;
    outframe.data. uint8 [5]=0x7E;
    outframe.data. uint8 [6]=0x86;
    outframe.data. uint8 [7]=0x67;
    Can0.sendFrame(outframe);

    //Index 9
    outframe.id = 0x656;            // Set our transmission address ID
    outframe.length = 8;            // Data payload 8 bytes
    outframe.extended = 0;          // Extended addresses - 0=11-bit 1=29bit
    outframe.rtr=0;                 //No request
    outframe.data. uint8 [0]=0x1F;
    outframe.data. uint8 [1]=0x00;
    outframe.data. uint8 [2]=0x00;
    outframe.data. uint8 [3]= 0x00;
    outframe.data. uint8 [4]=0x00;
    outframe.data. uint8 [5]=0x00;
    outframe.data. uint8 [6]=0x00;
    outframe.data. uint8 [7]=0x00;
    Can0.sendFrame(outframe);

    }
  }//end Frames 1000ms4-------------------------------------------------------------------   


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
  Frames1000MS1();
  Frames10MS();
  Frames1000MS2();
  Frames100MS();  
  Frames1000MS3();
  Frames1000MS4();
  }//end void loop---------------------------------------------------------------------------------------------
