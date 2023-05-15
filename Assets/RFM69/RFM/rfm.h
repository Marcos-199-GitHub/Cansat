#include "utils.h"
#include "registers.h"

//FREQ_SEL: 0-----315MHz
//FREQ_SEL: 1-----434MHz
//FREQ_SEL: 2-----868MHz
//FREQ_SEL: 3-----915MHz
#define FREQ_SEL    1   
//DIO0 In (En modo Rx da informacion de status)
#define IRQ 8


char sync_word[] = {0x2D,0xD4};
float rssi = 0.0;
int tx_power = 13;
int node = _RH_BROADCAST_ADDRESS;
int destination = _RH_BROADCAST_ADDRESS;
int identifier = 0;
int flags = 0;
int xmit_timeout = 2000;



  

const word RFM69FreqTbl[4][3] = {
 {0x074e, 0x08c0, 0x0900},  //315MHz
 {0x076c, 0x0880, 0x0900},  //434MHz
 {0x07d9, 0x0800, 0x0900},  //868MHz
 {0x07e4, 0x08c0, 0x0900},  //915MHz
};

const word RFM69ConfigTbl[20] = {
 0x0200,        //RegDataModul    FSK Packet  
 0x0502,        //RegFdevMsb    241*61Hz = 35KHz  
 0x0641,        //RegFdevLsb
 0x0334,        //RegBitrateMsb   32MHz/0x3410 = 2.4kpbs
 0x0410,        //RegBitrateLsb
 0x130F,        //RegOcp      Disable OCP
 0x1888,        //RegLNA            200R  
 0x1952,        //RegRxBw     RxBW  83KHz
 0x2C00,        //RegPreambleMsb  
 0x2D05,        //RegPreambleLsb  5Byte Preamble
 0x2E90,        //enable Sync.Word  2+1=3bytes
 0x2FAA,        //0xAA        SyncWord = aa2dd4
 0x302D,        //0x2D
 0x31D4,        //0xD4

 0x3700,        //RegPacketConfig1  Disable CRC��NRZ encode
 0x3815,        //RegPayloadLength  21bytes for length & Fixed length
 0x3C95,        //RegFiFoThresh   

 0x581B,        //RegTestLna        Normal sensitivity
 //0x582D,        //RegTestLna        increase sensitivity with LNA (Note: consumption also increase!)
 0x6F30,        //RegTestDAGC       Improved DAGC
 //0x6F00,        //RegTestDAGC       Normal DAGC
 0x0104,        //����Standby   
};

const word RFM69RxTable[5] = {        
 0x119F,        //RegPaLevel Fifo In for Rx
 0x2544,        //DIO Mapping for Rx
 0x5A55,        //Normal and TRx
 0x5C70,        //Normal and TRx    
 0x0110,        //Entry to Rx
};

const word RFM69TxTable[5] = {
 0x2504,        //
 0x119F,        //RegPaLevel 13dBm
 0x5A55,        //Normal and TRx 
 0x5C70,        //Normal and TRx  
 0x010C,        //Entry to Tx
};
byte RxData[32];

void set_boost(int x){
    if (tx_power>=18){
        spiWrite8(_REG_TEST_PA1,x);
        spiWrite8(_REG_TEST_PA2,x);
    }
}
void set_dio_0_mapping(char x){
    char dato;
    dato = spiRead8(_REG_DIO_MAPPING1);
    spiWrite8(_REG_DIO_MAPPING1,(dato&00111111) | (x<<6));
}
void operation_mode(int mode){
    char dato;
    dato = spiRead8(_REG_OP_MODE);
    dato = (dato & 0b11100011) | (mode << 2);
    spiWrite8(_REG_OP_MODE,dato);
}
void listen(){
    set_boost(_TEST_PA1_NORMAL);
    set_dio_0_mapping(0b01);
    operation_mode(RX_MODE);
}
void idle(){
    set_boost(_TEST_PA1_NORMAL);
    operation_mode(SLEEP_MODE);
    println("Idle");
}
void RFMinit(){
    char id, dato;
    dato = spiRead8(_REG_VERSION);
    print("Id: ");
    compare(dato,0x24,1);
    idle();
    spiWrite8(_REG_FIFO_THRESH,0b10001111);
    spiWrite8(_REG_TEST_DAGC,0x30);
    spiWrite8(_REG_TEST_PA1,_TEST_PA1_NORMAL);
    spiWrite8(_REG_TEST_PA2,_TEST_PA2_NORMAL);
    //spiWriteFrom(_REG_SYNC_VALUE1,sync_word,2);
    spiWrite8(_REG_PREAMBLE_MSB,0);
    spiWrite8(_REG_PREAMBLE_LSB,4);
    //Definir la frecuencia de funcionamiento a 433 MHZ
    //El calculo del valor que se pone en la frecuencia depende del cristal oscilador, en este modulo es de 32MHz(Fxosc),
    //Fstep = Fxosc/2^19 = 32000000/524288, FRF =  433000000/Fstep
    //Para el caso de 433 MHz, el calculo da 7,094,272:
    //0110 1100 ; 0100 0000 ; 0000 0000
    spiWrite8(_REG_FRF_MSB,   0b01101100);
    spiWrite8(_REG_FRF_MSB+1, 0b01000000);
    spiWrite8(_REG_FRF_MSB+2, 0b00000000);


    // Gaussian filter, BT=1.0
    dato = spiRead8(_REG_DATA_MOD);
    spiWrite8(_REG_DATA_MOD,(dato&0b11111100) | 0x01);

    // Bitrate =  250kbs
    //Segun el datasheet, el bitrate se obtiene dividiendo la freq del cristal entre el bitrate
    //32000000/250000    = 1280 = 0101 0000; 000000
    spiWrite8(_REG_BITRATE_MSB, 0b01010000);
    spiWrite8(_REG_BITRATE_MSB+1,0x00);
    // Max frequency deviation: 250 khz. Fdev = 250000/ (Fxosc/2^19) (CUIDADO, en el datasheet esta incorrecta la formula):
    //4096: 0000 1000 ; 00000000
    spiWrite8(_REG_FDEV_MSB, 0x08);
    spiWrite8(_REG_FDEV_MSB+1, 0x00);
    //TODO: documentar
    spiWrite8(_REG_RX_BW, 0xE0);
    spiWrite8(_REG_AFC_BW, 0xE0);
    //Formato de datos: con longitud de paquetes variable:
    //dc_free: whitening
    dato = spiRead8(_REG_PACKET_CONFIG1);
    spiWrite8(_REG_PACKET_CONFIG1,dato & 0b10100000);

    //tx power:
    //pa0,pa1,pa2,output power
    spiWrite8(_REG_PA_LEVEL,0x5F);
    
    //Todavia se pueden configurar mas variables como el nodo, la encriptacion, modo de operacion, modulacion de datos, etc.





}

void transmit(){
    set_boost(_TEST_PA1_BOOST);
    set_dio_0_mapping(0b00);
    operation_mode(TX_MODE);
}
char rfmSend(char* dato,const unsigned char n){
    char* payload = malloc(n+5);
    char d;
    int i;
    payload[n+4] = 0;
    if (n>=60){
        print("Mensaje muy largo");
        return 0;
    }
    idle();
    payload[0] = 4+n;
    payload[1] = destination;
    payload[2] = node;
    payload[3] = identifier;
    payload[4] = flags;
    for (i=0;i<n;i++)payload[i+5] = dato[i];
    spiWriteFrom(_REG_FIFO,payload,n+4);
    transmit();
    i=0;
    while (!d||i>xmit_timeout){
        //Verifica si el paquete ya se envio
        d = spiRead8(_REG_IRQ_FLAGS2 & 0x8) >> 3;
        Delay(100);
        i+=100;
    }
    idle();
    free(payload);
    return d;



}
char* receive(){
    //ms
    char timed_out = 0;
    char with_ack = 0;
    int timeout = 500;
    float last_rssi = rssi;
    char fifo_length = 0;
    int ack_delay=0;
    char packet[60];
    char with_header= 0;
    char keep_listening = 1;
    idle();
    if (!timed_out){
        fifo_length = spiRead8(_REG_FIFO);
        if (fifo_length){
            spiReadInto(_REG_FIFO,packet,fifo_length);
        }
        if (fifo_length<5)empty(packet,60);
        else{
            if (node != _RH_BROADCAST_ADDRESS && packet[0] != _RH_BROADCAST_ADDRESS && packet[0] != node){
               empty(packet,60) ;
            }
            else if (with_ack && (packet[3]&_RH_FLAGS_ACK==0)&& packet[0]!= _RH_BROADCAST_ADDRESS){
                Delay(ack_delay);
                //TODO: terminar
            }
            if (!with_header){
                //TODO: strip first 4 bytes of packet
            }
        }
    }
    if(keep_listening){
        listen();
    }
    else idle();
    
    return packet;
}
char readFIFO(){
    char i;
    for (i=0;i<60;i++){

    Serial.print(spiRead8(_REG_FIFO));
    print("-");
    }
    println("");
    return 0;
}







void readAllRegs()
{
  uint8_t regVal;
  
  println("Address - HEX - BIN");
  for (uint8_t regAddr = 1; regAddr <= 0x4F; regAddr++)
  {
    spiBegin();
    _spiWrite(regAddr & 0x7F); // send address + r/w bit
    regVal = _spiRead(0);
    spiEnd();

    Serial.print(regAddr, HEX);
    print(" - ");
    Serial.print(regVal,HEX);
    print(" - ");
    Serial.println(regVal,BIN);


  }
  spiEnd();
}
/**********************************************************
**Name:     RFM69_Config
**Function: Initialize RFM69 & set it entry to standby mode
**Input:    none
**Output:   none
**********************************************************/
void RFM69_Config(void)
{
 byte i;
 for(i=0;i<3;i++)
  spiWrite(RFM69FreqTbl[FREQ_SEL][i]);                    //Frequency parameter
 for(i=0;i<20;i++)                                        //base parameters
  spiWrite(RFM69ConfigTbl[i]);
}

/**********************************************************
**Name:     RFM69_EntryRx
**Function: Set RFM69 entry Rx_mode
**Input:    None
**Output:   "0" for Error Status
**********************************************************/
byte RFM69_EntryRx(void)
{
 byte i; 
 int SysTime = 0;
 RFM69_Config();                                          //config RFM69 base parameters
 for(i=0;i<5;i++)                                         //config RFM69 RxMode parameters 
  spiWrite(RFM69RxTable[i]);

 for(SysTime=0;SysTime<3;SysTime++)                                //wait for entry Rx
  {
  Delay(200);
  if((spiRead8(0x27)&0xC0)==0xC0)                          //Status OK?
   break;       
  }    
 if(SysTime>=3) {
  println("RX init error");
  return(0);   
  }                                           //over time for error
 else
  return(1);                                              //now, entry in RxMode
}
/**********************************************************
**Name:     RFM69_Standby
**Function: Set RFM69 to Standby mode
**Input:    none
**Output:   none
**********************************************************/
void RFM69_Standby(void)
{
 spiWrite(0x0104);                                        //standby mode
}

/**********************************************************
**Name:     RFM69_EntryTx
**Function: Set RFM69 entry Tx_mode
**Input:    None
**Output:   "0" for Error Status
**********************************************************/
byte RFM69_EntryTx(void)
{
 byte i,SysTime;

 RFM69_Config();                                          //config RFM69 base parameters
 for(i=0;i<5;i++)                                         //config RFM69 TxMode parameters 
  spiWrite(RFM69TxTable[i]);
   
 for(SysTime=0;SysTime<3;SysTime++)                                //wait for entry Tx
  {  
    Delay(1000);  
  if((spiRead8(0x27)&0xA0)==0xA0)                          //Status OK?
   break;       
  }    
 if(SysTime>=3)  {//over time for error
  char c = spiRead8(0x27);
    Serial.println(c,HEX);
    println("Tx error");   
  return(0);   
 }
 else
  {                                                       //now, entry in RxMode
  spiWriteFrom(0x00, "HOLAAAAAAAAAAAAAA MUNDO", 21);                //Send first Packet 
  println("First data sent");
  RFM69_Standby();
  return(1);                                              //return OK 
  }
}


/**********************************************************
**Name:     RFM69_ClearFIFO
**Function: Change to RxMode from StandbyMode, can clear FIFO buffer
**Input:    None
**Output:   None
**********************************************************/
void RFM69_ClearFIFO(void)
{
 spiWrite(0x0104);                                        //Entry Standby Mode
 spiWrite(0x0110);                                        //Change to Rx Mode
}

byte RFM69_RxPacket(void)
{
 byte i; 


 if(1)
  {
  for(i=0;i<21;i++)  
   RxData[i] = 0x00;
  spiReadInto(0x00, RxData, 32); 
  RFM69_ClearFIFO();
  for(i=0;i<32;i++)
   {
   print(RxData[i]);
   }
   println("");
  if(i>=14)                                               //Rx success?
   return(1);
  else
   return(0);
  }
 else
  return(0);
}



  
