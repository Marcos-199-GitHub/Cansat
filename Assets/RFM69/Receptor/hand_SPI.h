// Software SPI module connections
sbit SerIn at PORTB.B6;
sbit SerOut at PORTB.B3;
sbit SCK at PORTB.B4;
sbit SDITest at PORTB.B7;

sbit SDI_Direction at TRISB6_bit;
sbit SDO_Direction at TRISB3_bit;
sbit SCK_Direction at TRISB4_bit;
sbit SDITest_Direction at TRISB7_bit;

typedef unsigned short byte;
void spiInit(){
     SCK_Direction = 0;             // Set pin as Output
     SDI_Direction = 1;
     SDO_Direction = 0;
     SDITest_Direction = 0;
     SDITest=1;
     SerOut = 0;
     SCK = 0;
}
// End Software SPI module connections
short SPItransfer(short WrPara)
{
byte Data_In = 0;
byte bitcnt;

SCK = 1;
 for(bitcnt=8; bitcnt!=0; bitcnt--)
    {
    SCK = 0;
    Delay_Ms(10);
    if(WrPara & 0x80)
       SerOut = 1;   // RFM69 SDI
    else
       SerOut = 0;
    SCK = 1;
    if (SerIn==1)  {
      Data_In |= 1; 
      SDITest = 1;
      }
    else SDITest = 0;
    WrPara <<= 1;
    //16Mhz / (5*10) = 640Khz
    Delay_Ms(500);
    //Delay_Cyc(5);
    }
SCK=0;
SerOut=0;
return(Data_In);
}

short SPIRead8bit(void)
{
byte RdPara = 0;
byte bitcnt;
SerOut=1;   // RFM69 SDI                                                                          //Read one byte data from FIFO, MOSI hold to High
 for(bitcnt=8; bitcnt!=0; bitcnt--)
    {
    SCK=1;

    RdPara <<= 1;
    SCK=0;
    Delay_Ms(10);
    if(SerIn==1) {
       RdPara |= 0x01; 
       SDITest = 1;
       }
    else {

       SDITest = 0;
       }
Delay_Ms(500);
    }
 SCK=0;
 return(RdPara);
}