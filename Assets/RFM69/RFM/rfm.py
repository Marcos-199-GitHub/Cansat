import time
#Internal constants:
_REG_FIFO = 0x00
_REG_OP_MODE = 0x01
_REG_DATA_MOD = 0x02
_REG_BITRATE_MSB = 0x03
_REG_BITRATE_LSB = 0x04
_REG_FDEV_MSB = 0x05
_REG_FDEV_LSB = 0x06
_REG_FRF_MSB = 0x07
_REG_FRF_MID = 0x08
_REG_FRF_LSB = 0x09
_REG_VERSION = 0x10
_REG_PA_LEVEL = 0x11
_REG_RX_BW = 0x19
_REG_AFC_BW = 0x1A
_REG_RSSI_VALUE = 0x24
_REG_DIO_MAPPING1 = 0x25
_REG_IRQ_FLAGS1 = 0x27
_REG_IRQ_FLAGS2 = 0x28
_REG_PREAMBLE_MSB = 0x2C
_REG_PREAMBLE_LSB = 0x2D
_REG_SYNC_CONFIG = 0x2E
_REG_SYNC_VALUE1 = 0x2F
_REG_PACKET_CONFIG1 = 0x37
_REG_FIFO_THRESH = 0x3C
_REG_PACKET_CONFIG2 = 0x3D
_REG_AES_KEY1 = 0x3E
_REG_TEMP1 = 0x4E
_REG_TEMP2 = 0x4F
_REG_TEST_PA1 = 0x5A
_REG_TEST_PA2 = 0x5C
_REG_TEST_DAGC = 0x6F

_TEST_PA1_NORMAL = 0x55
_TEST_PA1_BOOST = 0x5D
_TEST_PA2_NORMAL = 0x70
_TEST_PA2_BOOST = 0x7C

#The crystal oscillator frequency and frequency synthesizer step size.
#See the datasheet for details of this calculation.

#        #define _FXOSC = 32000000.0
#        #define _FSTEP = _FXOSC / 524288

#RadioHead specific compatibility constants.
_RH_BROADCAST_ADDRESS = 0xFF
#The acknowledgement bit in the FLAGS
#The top 4 bits of the flags are reserved for RadioHead. The lower 4 bits are reserved
#for application layer use.
_RH_FLAGS_ACK = 0x80
_RH_FLAGS_RETRY = 0x40

#User facing constants:
SLEEP_MODE = 0b000
STANDBY_MODE = 0b001
FS_MODE = 0b010
TX_MODE = 0b011
RX_MODE = 0b100
#supervisor.ticks_ms() contants
#          #define _TICKS_PERIOD = const(1 << 29)
#          #define _TICKS_MAX = const(_TICKS_PERIOD - 1)
#          #define _TICKS_HALFPERIOD = const(_TICKS_PERIOD # 2)

#FREQ_SEL: 0-----315MHz
#FREQ_SEL: 1-----434MHz
#FREQ_SEL: 2-----868MHz
#FREQ_SEL: 3-----915MHz
FREQ_SEL =  1   
#DIO0 In (En modo Rx da informacion de status)
IRQ  = 8

0x40


sync_word = [0x2D,0xD4];
rssi = 0.0;
tx_power = 13;
node = _RH_BROADCAST_ADDRESS;
destination = _RH_BROADCAST_ADDRESS;
identifier = 0;
flags = 0;
xmit_timeout = 2000;



  

RFM69FreqTbl = [
 [0x074e, 0x08c0, 0x0900],  #315MHz
 [0x076c, 0x0880, 0x0900],  #434MHz
 [0x07d9, 0x0800, 0x0900],  #868MHz
 [0x07e4, 0x08c0, 0x0900],  #915MHz
]

RFM69ConfigTbl = [
 0x0200,        #RegDataModul    FSK Packet  
 0x0502,        #RegFdevMsb    241*61Hz = 35KHz  
 0x0641,        #RegFdevLsb
 0x0334,        #RegBitrateMsb   32MHz/0x3410 = 2.4kpbs
 0x0410,        #RegBitrateLsb
 0x130F,        #RegOcp      Disable OCP
 0x1888,        #RegLNA            200R  
 0x1952,        #RegRxBw     RxBW  83KHz
 0x2C00,        #RegPreambleMsb  
 0x2D05,        #RegPreambleLsb  5Byte Preamble
 0x2E90,        #enable Sync.Word  2+1=3bytes
 0x2FAA,        #0xAA        SyncWord = aa2dd4
 0x302D,        #0x2D
 0x31D4,        #0xD4

 0x3700,        #RegPacketConfig1  Disable CRC��NRZ encode
 0x3815,        #RegPayloadLength  21bytes for length & Fixed length
 0x3C95,        #RegFiFoThresh   

 0x581B,        #RegTestLna        Normal sensitivity
 #0x582D,        #RegTestLna        increase sensitivity with LNA (Note: consumption also increase!)
 0x6F30,        #RegTestDAGC       Improved DAGC
 #0x6F00,        #RegTestDAGC       Normal DAGC
 0x0104,        #����Standby   
]

RFM69RxTable = [       
 0x119F,        #RegPaLevel Fifo In for Rx
 0x2544,        #DIO Mapping for Rx
 0x5A55,        #Normal and TRx
 0x5C70,        #Normal and TRx    
 0x0110,        #Entry to Rx
]

RFM69TxTable = [
 0x2504,        #
 0x119F,        #RegPaLevel 13dBm
 0x5A55,        #Normal and TRx 
 0x5C70,        #Normal and TRx  
 0x010C,        #Entry to Tx
]
RxData = bytearray(32)

def spiWrite(word):
    #TODO
    pass
def Delay(time):
    time.sleep(time/1000)
def spiRead8(reg):
    #TODO
    pass
def println(s):
    print(s)
def spiWriteFrom(address, data, n):
    #TODO
    pass
def spiReadInto(address,data,n):
    #TODO
    pass

def Print(s):
    print(s,end="")


"""
/**********************************************************
**Name:     RFM69_Config
**Function: Initialize RFM69 & set it entry to standby mode
**Input:    none
**Output:   none
**********************************************************/
"""
def RFM69_Config():
    for i in range (0,3):
        spiWrite(RFM69FreqTbl[FREQ_SEL][i]);                    #Frequency parameter
    for i in range(0,20) :                                        #base parameters
        spiWrite(RFM69ConfigTbl[i]);
"""
/**********************************************************
**Name:     RFM69_EntryRx
**Function: Set RFM69 entry Rx_mode
**Input:    None
**Output:   "0" for Error Status
**********************************************************/
"""
def RFM69_EntryRx():    
    SysTime = 0
    RFM69_Config();                                          #config RFM69 base parameters
    for i in range (0,5):                                        #config RFM69 RxMode parameters 
        spiWrite(RFM69RxTable[i]); 
    for SysTime in range(0,3):                                #wait for entry Rx
        Delay(200)
        if((spiRead8(0x27)&0xC0)==0xC0):                          #Status OK?
            break
    if (SysTime>=3):
        println("RX init error")
        return(0)                                          #over time for error
    else:
        return(1)                                             #now, entry in RxMode
"""
/**********************************************************
**Name:     RFM69_Standby
**Function: Set RFM69 to Standby mode
**Input:    none
**Output:   none
**********************************************************/
"""
def RFM69_Standby():
    spiWrite(0x0104);                                        #standby mode
"""
/**********************************************************
**Name:     RFM69_EntryTx
**Function: Set RFM69 entry Tx_mode
**Input:    None
**Output:   "0" for Error Status
**********************************************************/
"""
def RFM69_EntryTx():
    SysTime = 0
    RFM69_Config();                                          #config RFM69 base parameters
    for i in range(0,5):                                         #config RFM69 TxMode parameters 
        spiWrite(RFM69TxTable[i]); 
    for SysTime in range(0,3):                            #wait for entry Tx
        Delay(100);  
        if((spiRead8(0x27)&0xA0)==0xA0):                          #Status OK?
            break
    if(SysTime>=3):                                         #over time for error
        println("Tx error");   
        return(0);   
    else:                                                #now, entry in RxMode
        spiWriteFrom(0x00, "HOLAAAAAAAAAAAAAA MUNDO", 21);                #Send first Packet 
        println("First data sent");
        RFM69_Standby();
        return(1);                                              #return OK 
    

"""
/**********************************************************
**Name:     RFM69_ClearFIFO
**Function: Change to RxMode from StandbyMode, can clear FIFO buffer
**Input:    None
**Output:   None
**********************************************************/
"""
def RFM69_ClearFIFO():
    spiWrite(0x0104);                                        #Entry Standby Mode
    spiWrite(0x0110);                                        #Change to Rx Mode

def RFM69_RxPacket():
    if(1 == 1):
        for i in range(0,21):
            RxData[i] = 0x00;
        spiReadInto(0x00, RxData, 32)
        RFM69_ClearFIFO()
        for i in range (0,32):
            Print(RxData[i]);
        println("")
        if(i>=14):                                               #Rx success?
            return(1)
        else:
            return(0)

    else:
        return(0)




  
