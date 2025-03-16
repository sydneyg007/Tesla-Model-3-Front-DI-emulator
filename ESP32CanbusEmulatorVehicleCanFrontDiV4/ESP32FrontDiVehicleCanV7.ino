//V2 seperating 1000ms frames out and varying timing slightly so all get sent.
//ver = 4: starts sending frames immediately - esp32 designed to be powered from Di wake pin
//ver = 5: adding front axlespeed from rear 0x108 to front 0x186, and voltage from rear 0x126 to front 0x1A5
//v6 maybe modify 0x557 front thermal control to not be 100c required? (use rear motor thermal control signals 0x5D7 instead?)
//v7 tidying up 0x195 should be 6 bytes (not 8), 0x1A5 shouyld be 7 bytes (not 8), 0x27A 4 bytes (not 8), 0x186 changed to 10ms (was 100ms)
  

//Frames: 
//0x1D5, 0x2E5, 0x186(Dis_frontTorque), 0x195, 0x1A5(DI_frontHvBusStatus), 0x27A, 0x396(DI_frontOilPump), 0x757, 0x304, 0x316, 0x356, 
//0x357, 0x35A, 0x35B, 0x376(DI_frontInverterTemperature), 0x3D6, 0x524, 0x556, 0x557 (DI_frontThermalControl), 0x656 (DI_Info),

//CanTX pin - gpio 22
//CanRX pin - gpio 23

#include <esp32_can.h>

byte DI_axleSpeedByte5 = 0;
byte DI_axleSpeedByte6 = 0;

byte DI_hvBusStatusByte0 = 0x03;
byte DI_hvBusStatusByte1 = 0xB6;

byte DI_frontThermalControlByte0 = 0x8C;
byte DI_frontThermalControlByte1 = 0x8C;
byte DI_frontThermalControlByte2 = 0x00;
byte DI_frontThermalControlByte3 = 0x00;

byte DIS_frontTorqueChecksum=0x07; // DIS_frontTorqueCounter+0x07
byte DIS_frontTorqueCounter=0x00;// ranges from 0x00 to 0x0F

byte counter0x1D5=0x00; //0x00 to 0xE0, increment by 0x20
byte checksum0x1D5=0xF7;

byte counter0x27A=0xC0; //0xC0 to 0xCF, increment by 0x01
byte checksum0x27A=0x38;// 0x38 to 0x47

byte frameCount0x757=0;

CAN_FRAME outframe;

void Frames10MS()
  {
  static unsigned long timer10ms = millis();   

  if (millis() - timer10ms >= 10) 
    {
    timer10ms=millis();

    //0x186 (390) Dis_frontTorque (also on partycan)    >chkd
    DIS_frontTorqueCounter ++; if (DIS_frontTorqueCounter > 0x0F) {DIS_frontTorqueCounter =0x00;} //DIS_frontTorqueCounter: 8|4@1+ (1,0) [0|0] ""
    DIS_frontTorqueChecksum = 0x01 + 0x86 + DIS_frontTorqueCounter + 0x80; //DIS_frontTorqueChecksum: 0|8@1+ (1,0) [0|0] "" (frame id numbers plus remaining frames)
    outframe.id = 0x186;            // Set our transmission address ID
    outframe.length = 8;            // Data payload
    outframe.extended = 0;          // Extended addresses - 0=11-bit 1=29bit
    outframe.rtr=0;  
    outframe.data.uint8 [0]= DIS_frontTorqueChecksum; //DIS_frontTorqueChecksum: 0|8@1+ (1,0) [0|0] ""
    outframe.data.uint8 [1]= DIS_frontTorqueCounter;  //DIS_frontTorqueCounter: 8|4@1+ (1,0) [0|0] "" DIS_frontTorqueCommand: 12|13@1- (2,0) [0|0] "Nm" (zero)
    outframe.data.uint8 [2]= 0x00;
    outframe.data.uint8 [3]= 0x00;
    outframe.data.uint8 [4]= 0x00;
    outframe.data.uint8 [5]= 0x00;   //DIS_frontAxleSpeed: 40|16@1- (0.1,0) [0|0] "RPM" calculate from 108 (264) DI_torque,   DI_axleSpeed: 40|16@1- (0.1,0) [0|0] "RPM" (leave the same)
    outframe.data.uint8 [6]= 0x80;   //DIS_frontAxleSpeed: 40|16@1- (0.1,0) [0|0] "RPM" calculate from 108 (264) DI_torque,   DI_axleSpeed: 40|16@1- (0.1,0) [0|0] "RPM" (leave at 0x80)
    outframe.data.uint8 [7]= 0x00;
    Can0.sendFrame(outframe);
    
    // 0x1D5 (469) ID1D5FrontTorque   >chkd
    counter0x1D5=counter0x1D5+0x20; if (counter0x1D5>0xFF) {counter0x1D5=0x00;}
    checksum0x1D5 = 0x01 + 0xD5 + 0x21 + counter0x1D5; //(frame id numbers plus remaining frames)
    outframe.id = 0x1D5;            // Set our transmission address ID
    outframe.length = 8;            // Data payload
    outframe.extended = 0;          // Extended addresses - 0=11-bit 1=29bit
    outframe.rtr=0;                 //No request
    outframe.data.uint8 [0]= 0x21;
    outframe.data.uint8 [1]= 0x00; //FrontTorqueRequest1D5 : 8|13@1- (0.222,0) [-909.312|909.09] "NM"  Receiver (leave at zero)
    outframe.data.uint8 [2]= 0x00;  //FrontTorque1D5 : 21|13@1- (0.222,0) [-909.312|909.09] "NM"  Receiver       (leave at zero)
    outframe.data.uint8 [3]= 0x00;  //FrontTorque1D5 : 21|13@1- (0.222,0) [-909.312|909.09] "NM"  Receiver       (leave at zero)
    outframe.data.uint8 [4]= 0x00;
    outframe.data.uint8 [5]= 0x00;
    outframe.data.uint8 [6]= counter0x1D5;
    outframe.data.uint8 [7]= checksum0x1D5;
    Can0.sendFrame(outframe); 

    // 0x2E5 (741) DIS_power //Doesn't change   >chkd
    outframe.id = 0x2E5;            // Set our transmission address ID
    outframe.length = 8;            // Data payload
    outframe.extended = 0;          // Extended addresses - 0=11-bit 1=29bit
    outframe.rtr=0;                 //No request
    outframe.data.uint8 [0]= 0x00;
    outframe.data.uint8 [1]= 0x00;
    outframe.data.uint8 [2]= 0x03;
    outframe.data.uint8 [3]= 0x0D;
    outframe.data.uint8 [4]= 0x00;
    outframe.data.uint8 [5]= 0x00;
    outframe.data.uint8 [6]= 0xDD;
    outframe.data.uint8 [7]= 0x00;
    Can0.sendFrame(outframe); 

    }
  }//end Frames 10ms-------------------------------------------------------------------


void Frames100MS()
  {
  static unsigned long timer100ms = millis();   

  if (millis() - timer100ms >= 100) 
    {
    timer100ms=millis();

    // 0x195 (405) Doesn't change     >chkd
    outframe.id = 0x195;            // Set our transmission address ID
    outframe.length = 6;            // Data payload
    outframe.extended = 0;          // Extended addresses - 0=11-bit 1=29bit
    outframe.rtr=0;                 //No request
    outframe.data.uint8 [0]= 0x00;
    outframe.data.uint8 [1]= 0x08; 
    outframe.data.uint8 [2]= 0x08;
    outframe.data.uint8 [3]= 0x00;
    outframe.data.uint8 [4]= 0x28;
    outframe.data.uint8 [5]= 0x00;
    Can0.sendFrame(outframe); 

    // 0x1A5 (421) DI_frontHvBusStatus     >chkd
    outframe.id = 0x1A5;            // Set our transmission address ID
    outframe.length = 7;            // Data payload
    outframe.extended = 0;          // Extended addresses - 0=11-bit 1=29bit
    outframe.rtr=0;                 //No request
    outframe.data.uint8 [0] = DI_hvBusStatusByte0; //DI_frontVoltage : 0|10@1+ (0.5,0) [0|500] "V" (derived from rear voltage)
    outframe.data.uint8 [1] = DI_hvBusStatusByte1 & B00000011; //DI_frontVoltage : 0|10@1+ (0.5,0) [0|500] "V"(derived from rear voltage), DI_frontCurrent : 10|11@1+ (1,0) [0|2047] "A"(current stays at zero) (derived from rear voltage)
    outframe.data.uint8 [2] = 0x00;
    outframe.data.uint8 [3] = 0x00;
    outframe.data.uint8 [4] = 0x00;
    outframe.data.uint8 [5] = 0xE2;
    outframe.data.uint8 [6] = 0x03;
    Can0.sendFrame(outframe); 

    // 0x27A (634)     >chkd
    counter0x27A = counter0x27A +0x01; if (counter0x27A >0xCF) {counter0x27A =0xC0;}
    checksum0x27A = 0x02 + 0x7A + counter0x27A + 0xFC;
    outframe.id = 0x27A;            // Set our transmission address ID
    outframe.length = 4;            // Data payload
    outframe.extended = 0;          // Extended addresses - 0=11-bit 1=29bit
    outframe.rtr=0;                 //No request
    outframe.data.uint8 [0]= checksum0x27A;
    outframe.data.uint8 [1]= counter0x27A; 
    outframe.data.uint8 [2]= 0xFC; //mainly 0xFC but can also be 0x0C
    outframe.data.uint8 [3]= 0x00; //mainly 0x00 but can also be 0x01
    Can0.sendFrame(outframe); 

    //0x396 (918) DI_frontOilPump     >chkd
    outframe.id = 0x396;            // Set our transmission address ID
    outframe.length = 8;            // Data payload
    outframe.extended = 0;          // Extended addresses - 0=11-bit 1=29bit
    outframe.rtr=0;                 //No request
    outframe.data. uint8 [0]= 0x01; //DI_frontOilPumpState: 0|3@1+ (1,0) [0|0] "", DI_frontOilPumpFluidConfidence : 3|1@1+ (1,0) [0|0] "", DI_frontOilPumpLeadAngle : 4|4@1+ (1.875,0) [0|28.125] "deg" (always 0x01)
    outframe.data. uint8 [1]= 0x32; //DI_frontOilPumpFlowTarget: 8|8@1+ (0.06,0) [0|0] "LPM" (always 0x32)
    outframe.data. uint8 [2]= 0x32; //DI_frontOilPumpFlowActual: 16|8@1+ (0.06,0) [0|0] "LPM" (mainly 0x32)
    outframe.data. uint8 [3]= 0x38; //DI_frontOilPumpPcbT: 24|8@1+ (1,-40) [0|0] "DegC" (mainly 0x38)
    outframe.data. uint8 [4]= 0x4B; //DI_frontOilPumpFluidT: 32|8@1+ (1,-40) [0|0] "DegC" (mainly 0x4B)
    outframe.data. uint8 [5]= 0x3C; //DI_frontOilPumpVoltage: 40|8@1+ (0.1,0) [0|0] "V" (mainly 0x3C)
    outframe.data. uint8 [6]= 0x19; //DI_frontOilPumpCurrent: 48|8@1+ (0.1,0) [0|0] "A" (mainly 0x19)
    outframe.data. uint8 [7]= 0x7D; //DI_frontOilPumpCurrentSensorOffset: 56|8@1+ (0.005,0) [0|0] "A" (always 0x7D)
    Can0.sendFrame(outframe);

    //0x757 (1879) ID757DIF_debug     >chkd
    frameCount0x757++; if(frameCount0x757>12) {frameCount0x757=1;}
    outframe.id = 0x757;            // Set our transmission address ID
    outframe.length = 8;            // Data payload
    outframe.extended = 0;          // Extended addresses - 0=11-bit 1=29bit
    outframe.rtr=0;                 //No request
    if(frameCount0x757<11){
      outframe.data. uint8 [0]= 0x32; //mainly 0x32 Every 11th frame 0x46, Every 12th frame 0x48 all other frames 0x32
      outframe.data. uint8 [1]= 0x0E; //mainly 0x0E Every 11th & 12th frame 0x3A, all other frames 0x0E
      outframe.data. uint8 [2]= 0x00; //mainly 0x00 Every 11th & 12th frame 0x3A, all other frames 0x00
      outframe.data. uint8 [3]= 0x00; //mainly 0x00 Every 11th frame 0x3A, all other frames 0x00
      outframe.data. uint8 [4]= 0x00; //mainly 0x00 Every 11th frame 0x3A, all other frames 0x00
      outframe.data. uint8 [5]= 0x00; //mainly 0x00 Every 11th frame 0x43, every 12th frame 0x80, all other frames 0x00
      outframe.data. uint8 [6]= 0x00; //mainly 0x00 Every 11th frame 0x3B, every 12th frame 0x0E, all other frames 0x00
      outframe.data. uint8 [7]= 0x00; //mainly 0x00 Every 11th frame 0x3B, all other frames 0x00
      }
    if(frameCount0x757==11){
      outframe.data. uint8 [0]= 0x46; //mainly 0x32 Every 11th frame 0x46, Every 12th frame 0x48 all other frames 0x32
      outframe.data. uint8 [1]= 0x3A; //mainly 0x0E Every 11th & 12th frame 0x3A, all other frames 0x0E
      outframe.data. uint8 [2]= 0x3A; //mainly 0x00 Every 11th & 12th frame 0x3A, all other frames 0x00
      outframe.data. uint8 [3]= 0x3A; //mainly 0x00 Every 11th frame 0x3A, all other frames 0x00
      outframe.data. uint8 [4]= 0x3A; //mainly 0x00 Every 11th frame 0x3A, all other frames 0x00
      outframe.data. uint8 [5]= 0x43; //mainly 0x00 Every 11th frame 0x43, every 12th frame 0x80, all other frames 0x00
      outframe.data. uint8 [6]= 0x3B; //mainly 0x00 Every 11th frame 0x3B, every 12th frame 0x0E, all other frames 0x00
      outframe.data. uint8 [7]= 0x3B; //mainly 0x00 Every 11th frame 0x3B, all other frames 0x00
      }
    if(frameCount0x757==12){
      outframe.data. uint8 [0]= 0x48; //mainly 0x32 Every 11th frame 0x46, Every 12th frame 0x48 all other frames 0x32
      outframe.data. uint8 [1]= 0x3A; //mainly 0x0E Every 11th & 12th frame 0x3A, all other frames 0x0E
      outframe.data. uint8 [2]= 0x3A; //mainly 0x00 Every 11th & 12th frame 0x3A, all other frames 0x00
      outframe.data. uint8 [3]= 0x00; //mainly 0x00 Every 11th frame 0x3A, all other frames 0x00
      outframe.data. uint8 [4]= 0x00; //mainly 0x00 Every 11th frame 0x3A, all other frames 0x00
      outframe.data. uint8 [5]= 0x80; //mainly 0x00 Every 11th frame 0x43, every 12th frame 0x80, all other frames 0x00
      outframe.data. uint8 [6]= 0x0E; //mainly 0x00 Every 11th frame 0x3B, every 12th frame 0x0E, all other frames 0x00
      outframe.data. uint8 [7]= 0x00; //mainly 0x00 Every 11th frame 0x3B, all other frames 0x00
      }
    Can0.sendFrame(outframe);
    }
  }//end Frames 100ms-------------------------------------------------------------------

void Frames1000MS1()
  {
  static unsigned long timer1000ms1 = millis();
  if (millis() - timer1000ms1 >= 1004) 
    {
    timer1000ms1=millis();
    //0x304 (772) //everything 0x00     >chkd
    outframe.id = 0x304;            // Set our transmission address ID
    outframe.length = 8;            // Data payload
    outframe.extended = 0;          // Extended addresses - 0=11-bit 1=29bit
    outframe.rtr=0;                 //No request
    outframe.data.uint8 [0]= 0x00;
    outframe.data.uint8 [1]= 0x00;
    outframe.data.uint8 [2]= 0x00;
    outframe.data.uint8 [3]= 0x00;
    outframe.data.uint8 [4]= 0x00;
    outframe.data.uint8 [5]= 0x00;
    outframe.data.uint8 [6]= 0x00;
    outframe.data.uint8 [7]= 0x00;
    Can0.sendFrame(outframe); 
            
    //0x316 (790) PM_info   sends all 8 indexed frames at once (every 1000ms)     >chkd
      outframe.id = 0x316;            // Set our transmission address ID
      outframe.length = 8;            // Data payload
      outframe.extended = 0;          // Extended addresses - 0=11-bit 1=29bit
      outframe.rtr=0;                 //No request
      outframe.data. uint8 [0]= 0x0A; //matrix index
      outframe.data. uint8 [1]= 0x01;
      outframe.data. uint8 [2]= 0x4B;
      outframe.data. uint8 [3]= 0x32;
      outframe.data. uint8 [4]= 0xFC;
      outframe.data. uint8 [5]= 0x00;
      outframe.data. uint8 [6]= 0x09;
      outframe.data. uint8 [7]= 0x00;
      Can0.sendFrame(outframe);

      outframe.id = 0x316;            // Set our transmission address ID
      outframe.length = 8;            // Data payload
      outframe.extended = 0;          // Extended addresses - 0=11-bit 1=29bit
      outframe.rtr=0;                 //No request
      outframe.data. uint8 [0]= 0x0B; //matrix index
      outframe.data. uint8 [1]= 0x00;
      outframe.data. uint8 [2]= 0x1C;
      outframe.data. uint8 [3]= 0x51;
      outframe.data. uint8 [4]= 0x01;
      outframe.data. uint8 [5]= 0x00;
      outframe.data. uint8 [6]= 0x00;
      outframe.data. uint8 [7]= 0x00;
      Can0.sendFrame(outframe);

      outframe.id = 0x316;            // Set our transmission address ID
      outframe.length = 8;            // Data payload
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

      outframe.id = 0x316;            // Set our transmission address ID
      outframe.length = 8;            // Data payload
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

      outframe.id = 0x316;            // Set our transmission address ID
      outframe.length = 8;            // Data payload
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

      outframe.id = 0x316;            // Set our transmission address ID
      outframe.length = 8;            // Data payload
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

      outframe.id = 0x316;            // Set our transmission address ID
      outframe.length = 8;            // Data payload
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

      outframe.id = 0x316;            // Set our transmission address ID
      outframe.length = 8;            // Data payload
      outframe.extended = 0;          // Extended addresses - 0=11-bit 1=29bit
      outframe.rtr=0;                 //No request
      outframe.data. uint8 [0]= 0x15;
      outframe.data. uint8 [1]= 0x01;
      outframe.data. uint8 [2]= 0x4B;
      outframe.data. uint8 [3]= 0x32;
      outframe.data. uint8 [4]= 0xBF;
      outframe.data. uint8 [5]= 0x97;
      outframe.data. uint8 [6]= 0x29;
      outframe.data. uint8 [7]= 0xDB;
      Can0.sendFrame(outframe);

    }//end timer
  }//end Frames 1000ms1-------------------------------------------------------------------

void Frames1000MS2()
  {
  static unsigned long timer1000ms2 = millis();
  if (millis() - timer1000ms2 >= 1002) 
    {
    timer1000ms2=millis();

    //0x356 (854) DI_alertMatrix1 always all zeros     >chkd
    outframe.id = 0x356;            // Set our transmission address ID
    outframe.length = 8;            // Data payload
    outframe.extended = 0;          // Extended addresses - 0=11-bit 1=29bit
    outframe.rtr=0;                 //No request
    outframe.data. uint8 [0]= 0x00;
    outframe.data. uint8 [1]= 0x00;
    outframe.data. uint8 [2]= 0x00;
    outframe.data. uint8 [3]= 0x00;
    outframe.data. uint8 [4]= 0x00;
    outframe.data. uint8 [5]= 0x00;
    outframe.data. uint8 [6]= 0x00;
    outframe.data. uint8 [7]= 0x00;
    Can0.sendFrame(outframe);

    //0x357 (855) DI_alertMatrix2 always all zeros     >chkd
    outframe.id = 0x357;            // Set our transmission address ID
    outframe.length = 8;            // Data payload
    outframe.extended = 0;          // Extended addresses - 0=11-bit 1=29bit
    outframe.rtr=0;                 //No request
    outframe.data. uint8 [0]= 0x00;
    outframe.data. uint8 [1]= 0x00; //motor speed mismatch (leave at zero)
    outframe.data. uint8 [2]= 0x00;
    outframe.data. uint8 [3]= 0x00;
    outframe.data. uint8 [4]= 0x00;
    outframe.data. uint8 [5]= 0x00;
    outframe.data. uint8 [6]= 0x00;
    outframe.data. uint8 [7]= 0x00;
    Can0.sendFrame(outframe);

    //0x35A (858) DI_alertMatrix3 always all zeros     >chkd
    outframe.id = 0x35A;            // Set our transmission address ID
    outframe.length = 8;            // Data payload
    outframe.extended = 0;          // Extended addresses - 0=11-bit 1=29bit
    outframe.rtr=0;                 //No request
    outframe.data. uint8 [0]= 0x00;
    outframe.data. uint8 [1]= 0x00;
    outframe.data. uint8 [2]= 0x00;
    outframe.data. uint8 [3]= 0x00;
    outframe.data. uint8 [4]= 0x00;
    outframe.data. uint8 [5]= 0x00;
    outframe.data. uint8 [6]= 0x00;
    outframe.data. uint8 [7]= 0x00;
    Can0.sendFrame(outframe);

    //0x35B (859) DI_alertMatrix4 always all zeros     >chkd
    outframe.id = 0x35B;            // Set our transmission address ID
    outframe.length = 8;            // Data payload
    outframe.extended = 0;          // Extended addresses - 0=11-bit 1=29bit
    outframe.rtr=0;                 //No request
    outframe.data. uint8 [0]= 0x00;
    outframe.data. uint8 [1]= 0x00;
    outframe.data. uint8 [2]= 0x00;
    outframe.data. uint8 [3]= 0x00;
    outframe.data. uint8 [4]= 0x00;
    outframe.data. uint8 [5]= 0x00;
    outframe.data. uint8 [6]= 0x00;
    outframe.data. uint8 [7]= 0x00;
    Can0.sendFrame(outframe);

    //0x376 (886) DI_frontInverterTemperature     >chkd
    outframe.id = 0x376;            // Set our transmission address ID
    outframe.length = 8;            // Data payload
    outframe.extended = 0;          // Extended addresses - 0=11-bit 1=29bit
    outframe.rtr=0;                 //No request
    outframe.data. uint8 [0]= 0x3D;
    outframe.data. uint8 [1]= 0x3A;
    outframe.data. uint8 [2]= 0x3A;
    outframe.data. uint8 [3]= 0x50;
    outframe.data. uint8 [4]= 0x3A;
    outframe.data. uint8 [5]= 0x8A;
    outframe.data. uint8 [6]= 0x7D;
    outframe.data. uint8 [7]= 0x02;
    Can0.sendFrame(outframe);

    }//end timer
  }//end Frames 1000ms2-------------------------------------------------------------------    

void Frames1000MS3()
  {
  static unsigned long timer1000ms3 = millis();
  if (millis() - timer1000ms3 >= 1000) 
    {
    timer1000ms3=millis();
    //0x3D6 (982) Doesn't really change much      >chkd
    outframe.id = 0x3D6;            // Set our transmission address ID
    outframe.length = 8;            // Data payload
    outframe.extended = 0;          // Extended addresses - 0=11-bit 1=29bit
    outframe.rtr=0;                 //No request
    outframe.data. uint8 [0]= 0x3E;
    outframe.data. uint8 [1]= 0x3E;
    outframe.data. uint8 [2]= 0x3E;
    outframe.data. uint8 [3]= 0x3D;
    outframe.data. uint8 [4]= 0x3E;
    outframe.data. uint8 [5]= 0x85;
    outframe.data. uint8 [6]= 0x10;
    outframe.data. uint8 [7]= 0x4D;
    Can0.sendFrame(outframe);

    //0x524 (1316) Doesn't change      >chkd
    outframe.id = 0x524;            // Set our transmission address ID
    outframe.length = 7;            // Data payload
    outframe.extended = 0;          // Extended addresses - 0=11-bit 1=29bit
    outframe.rtr=0;                 //No request
    outframe.data. uint8 [0]= 0x64;
    outframe.data. uint8 [1]= 0x03;
    outframe.data. uint8 [2]= 0x08;
    outframe.data. uint8 [3]= 0x0A;
    outframe.data. uint8 [4]= 0x18;
    outframe.data. uint8 [5]= 0xB0;
    outframe.data. uint8 [6]= 0x02;
    Can0.sendFrame(outframe);

    //0x556 (1366) ID556FrontDItemps      >chkd
    outframe.id = 0x556;            // Set our transmission address ID
    outframe.length =5;            // Data payload
    outframe.extended = 0;          // Extended addresses - 0=11-bit 1=29bit
    outframe.rtr=0;                 //No request
    outframe.data. uint8 [0]=0x3A;
    outframe.data. uint8 [1]=0x3A;
    outframe.data. uint8 [2]=0x00;
    outframe.data. uint8 [3]=0x3B;
    outframe.data. uint8 [4]=0xFF;
    Can0.sendFrame(outframe);

    //0x557 (1367) DI_frontThermalControl - Derive from rear thermal control      >chkd
    
    //rear thermal control: 0x5D7 Di_thermalControl 
    //DI_activeInletTempReq: 8|8@1+ (1,-40) [0|0] "DegC" X
    //DI_coolantFlowReq: 16|8@1+ (0.2,0) [0|0] "LPM" X
    //DI_oilFlowReq : 24|8@1+ (0.06,0) [0|15] "LPM" X
    //DI_passiveInletTempReq: 0|8@1+ (1,-40) [0|0] "DegC" X

    //DI_frontThermalControl 0x557
    //DI_frontActiveInletTempReq : 8|8@1+ (1,-40) [-40|120] "C" X
    // DI_frontCoolantFlowReq : 16|8@1+ (0.2,0) [0|50] "LPM" X
    //DI_frontOilFlowReq : 24|8@1+ (0.06,0) [0|15] "LPM" X
    //DI_frontPassiveInletTempReq : 0|8@1+ (1,-40) [-40|120] "C" X

    outframe.id = 0x557;            // Set our transmission address ID
    outframe.length = 4;            // Data payload
    outframe.extended = 0;          // Extended addresses - 0=11-bit 1=29bit
    outframe.rtr=0;                 //No request
    outframe.data. uint8 [0]=DI_frontThermalControlByte0;
    outframe.data. uint8 [1]=DI_frontThermalControlByte1;
    outframe.data. uint8 [2]=DI_frontThermalControlByte2;
    outframe.data. uint8 [3]=DI_frontThermalControlByte3;
    Can0.sendFrame(outframe);

    //0x656 (1622) DI_Info      sends all 9 indexed frames at once (every 1000ms)      >chkd
      outframe.id = 0x656;            // Set our transmission address ID
      outframe.length = 8;            // Data payload
      outframe.extended = 0;          // Extended addresses - 0=11-bit 1=29bit
      outframe.rtr=0;                 //No request
      outframe.data. uint8 [0]= 0x0A;
      outframe.data. uint8 [1]= 0x01;
      outframe.data. uint8 [2]= 0x4B;
      outframe.data. uint8 [3]= 0x32;
      outframe.data. uint8 [4]= 0xFC;
      outframe.data. uint8 [5]= 0x00;
      outframe.data. uint8 [6]= 0x05;
      outframe.data. uint8 [7]= 0x00;
      Can0.sendFrame(outframe);

      outframe.id = 0x656;            // Set our transmission address ID
      outframe.length = 8;            // Data payload
      outframe.extended = 0;          // Extended addresses - 0=11-bit 1=29bit
      outframe.rtr=0;                 //No request
      outframe.data. uint8 [0]= 0x0B;
      outframe.data. uint8 [1]= 0x00;
      outframe.data. uint8 [2]= 0x1C;
      outframe.data. uint8 [3]= 0x51;
      outframe.data. uint8 [4]= 0x01;
      outframe.data. uint8 [5]= 0x00;
      outframe.data. uint8 [6]= 0x00;
      outframe.data. uint8 [7]= 0x00;
      Can0.sendFrame(outframe);

      outframe.id = 0x656;            // Set our transmission address ID
      outframe.length = 8;            // Data payload
      outframe.extended = 0;          // Extended addresses - 0=11-bit 1=29bit
      outframe.rtr=0;                 //No request
      outframe.data. uint8 [0]= 0x0D;
      outframe.data. uint8 [1]= 0x00;
      outframe.data. uint8 [2]= 0x00;
      outframe.data. uint8 [3]= 0x00;
      outframe.data. uint8 [4]= 0xBF;
      outframe.data. uint8 [5]= 0x97;
      outframe.data. uint8 [6]= 0x29;
      outframe.data. uint8 [7]= 0xDB;
      Can0.sendFrame(outframe);

      outframe.id = 0x656;            // Set our transmission address ID
      outframe.length = 8;            // Data payload
      outframe.extended = 0;          // Extended addresses - 0=11-bit 1=29bit
      outframe.rtr=0;                 //No request
      outframe.data. uint8 [0]= 0x10;
      outframe.data. uint8 [1]= 0x01;
      outframe.data. uint8 [2]= 0xC4;
      outframe.data. uint8 [3]= 0x27;
      outframe.data. uint8 [4]= 0xCC;
      outframe.data. uint8 [5]= 0x6A;
      outframe.data. uint8 [6]= 0xFF;
      outframe.data. uint8 [7]= 0x00;
      Can0.sendFrame(outframe);

      outframe.id = 0x656;            // Set our transmission address ID
      outframe.length = 8;            // Data payload
      outframe.extended = 0;          // Extended addresses - 0=11-bit 1=29bit
      outframe.rtr=0;                 //No request
      outframe.data. uint8 [0]= 0x11;
      outframe.data. uint8 [1]= 0x00;
      outframe.data. uint8 [2]= 0x00;
      outframe.data. uint8 [3]= 0x00;
      outframe.data. uint8 [4]= 0x00;
      outframe.data. uint8 [5]= 0x00;
      outframe.data. uint8 [6]= 0x00;
      outframe.data. uint8 [7]= 0x00;
      Can0.sendFrame(outframe);

      outframe.id = 0x656;            // Set our transmission address ID
      outframe.length = 8;            // Data payload
      outframe.extended = 0;          // Extended addresses - 0=11-bit 1=29bit
      outframe.rtr=0;                 //No request
      outframe.data. uint8 [0]= 0x12;
      outframe.data. uint8 [1]= 0x16;
      outframe.data. uint8 [2]= 0x48;
      outframe.data. uint8 [3]= 0xB4;
      outframe.data. uint8 [4]= 0x7A;
      outframe.data. uint8 [5]= 0x66;
      outframe.data. uint8 [6]= 0x41;
      outframe.data. uint8 [7]= 0xFD;
      Can0.sendFrame(outframe);

      outframe.id = 0x656;            // Set our transmission address ID
      outframe.length = 8;            // Data payload
      outframe.extended = 0;          // Extended addresses - 0=11-bit 1=29bit
      outframe.rtr=0;                 //No request
      outframe.data. uint8 [0]= 0x13;
      outframe.data. uint8 [1]= 0x03;
      outframe.data. uint8 [2]= 0x00;
      outframe.data. uint8 [3]= 0x00;
      outframe.data. uint8 [4]= 0x00;
      outframe.data. uint8 [5]= 0x00;
      outframe.data. uint8 [6]= 0x00;
      outframe.data. uint8 [7]= 0x00;
      Can0.sendFrame(outframe);

      outframe.id = 0x656;            // Set our transmission address ID
      outframe.length = 8;            // Data payload
      outframe.extended = 0;          // Extended addresses - 0=11-bit 1=29bit
      outframe.rtr=0;                 //No request
      outframe.data. uint8 [0]= 0x14;
      outframe.data. uint8 [1]= 0x05;
      outframe.data. uint8 [2]= 0x00;
      outframe.data. uint8 [3]= 0x00;
      outframe.data. uint8 [4]= 0x10;
      outframe.data. uint8 [5]= 0x7E;
      outframe.data. uint8 [6]= 0x86;
      outframe.data. uint8 [7]= 0x67;
      Can0.sendFrame(outframe);
 
      outframe.id = 0x656;            // Set our transmission address ID
      outframe.length = 8;            // Data payload
      outframe.extended = 0;          // Extended addresses - 0=11-bit 1=29bit
      outframe.rtr=0;                 //No request
      outframe.data. uint8 [0]= 0x1F;
      outframe.data. uint8 [1]= 0x00;
      outframe.data. uint8 [2]= 0x00;
      outframe.data. uint8 [3]= 0x00;
      outframe.data. uint8 [4]= 0x00;
      outframe.data. uint8 [5]= 0x00;
      outframe.data. uint8 [6]= 0x00;
      outframe.data. uint8 [7]= 0x00;
      Can0.sendFrame(outframe);

    }//end 1000ms timer
  }//end Frames 1000ms3------------------------------------------------------------------- 

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup()
{
//Serial.begin(115200); 

/*
//Tcan485 setup for testing:
pinMode(16, OUTPUT); //PIN_5V_EN
digitalWrite(16, HIGH);
pinMode(23, OUTPUT); //CAN_SE_PIN
digitalWrite(23, LOW);
CAN_cfg.tx_pin_id = GPIO_NUM_27;
CAN_cfg.rx_pin_id = GPIO_NUM_26;
*/

//esp32 pins
CAN_cfg.tx_pin_id = GPIO_NUM_22;
CAN_cfg.rx_pin_id = GPIO_NUM_23;

CAN0.begin(500000); // Initialize CAN0 and set baud rate.
CAN0.watchFor();
}//end void setup--------------------------------------------------------------------------------------------

void loop()
{
CAN_FRAME message;
if (CAN0.read(message)) {
  if (message.id==0x108) //(264) DI_torque
    { 
    DI_axleSpeedByte5 = message.data.byte[5]; //DI_axleSpeed: 40|16@1- (0.1,0) [0|0] "RPM"
    DI_axleSpeedByte6 = message.data.byte[6]; //DI_axleSpeed: 40|16@1- (0.1,0) [0|0] "RPM"
    }
  if (message.id==0x126)
    { 
    //DI_voltage : 0|10@1+ (0.5,0) [0|500] "V"
    DI_hvBusStatusByte0 = message.data.byte[0];
    DI_hvBusStatusByte1 = message.data.byte[1];
    }
  if (message.id==0x5D7)
    { 
    //rear thermal control: 0x5D7 Di_thermalControl 
    //DI_activeInletTempReq: 8|8@1+ (1,-40) [0|0] "DegC" X
    //DI_coolantFlowReq: 16|8@1+ (0.2,0) [0|0] "LPM" X
    //DI_oilFlowReq : 24|8@1+ (0.06,0) [0|15] "LPM" X
    //DI_passiveInletTempReq: 0|8@1+ (1,-40) [0|0] "DegC" X
    DI_frontThermalControlByte0 = message.data.byte[0];
    DI_frontThermalControlByte1 = message.data.byte[1];
    DI_frontThermalControlByte2 = message.data.byte[2];
    DI_frontThermalControlByte3 = message.data.byte[3];
    }
  }
Frames10MS();
Frames100MS(); 
Frames1000MS1();
Frames10MS();
Frames100MS(); 
Frames1000MS2();
Frames10MS();
Frames100MS(); 
Frames1000MS3();
}//end void loop---------------------------------------------------------------------------------------------
