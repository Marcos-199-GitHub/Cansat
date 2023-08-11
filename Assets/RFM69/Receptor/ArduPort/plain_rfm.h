#include <barerfm69_const.h>
#define RFM69_PLAIN_STATE_RECEIVING 0
#define RFM69_PLAIN_STATE_SENDING 1
//Globales
uint8_t state = 0;
uint16_t buffer_write_index = 0;
uint16_t buffer_read_index = 0;
uint16_t buffer_size = 0;
uint8_t **packet_buffer = 0;
bool use_variable_length = 0 ;
bool use_addressing = 0;
         
 /*
            Returns IRQ1 flags, bitmask, the following bits can be set:

RFM69_IRQ1_MODEREADY
                Set when the operation mode requested in Mode, is ready 
                    - Sleep: Entering Sleep mode
                    - Standby: XO is running
                    - FS: PLL is locked
                    - Rx: RSSI sampling starts
                    - Tx: PA ramp-up completed
                Cleared when changing operating mode.
            RFM69_IRQ1_RXREADY
                Set in Rx mode, after RSSI, AGC and AFC. Cleared when leaving
                Rx.
            RFM69_IRQ1_TXREADY
                Set in Tx mode, after PA ramp-up. Cleared when leaving Tx.
            RFM69_IRQ1_PLLLOCK
                Set (in FS, Rx or Tx) when the PLL is locked.
                Cleared when it is not.
            RFM69_IRQ1_RSSI
                Set in Rx when the RssiValue exceeds RssiThreshold.
                Cleared when leaving Rx.
            RFM69_IRQ1_TIMEOUT
                Set when a timeout occurs (see TimeoutRxStart and
                TimeoutRssiThresh) Cleared when leaving Rx or FIFO is emptied.
            RFM69_IRQ1_AUTOMODE
                Set when entering Intermediate mode. Cleared when exiting
                Intermediate mode. Please note that in Sleep mode a small delay
                can be observed between AutoMode interrupt and the corresponding
                enter/exit condition.
            RFM69_IRQ1_SYNCADDRESSMATCH
                Set when Sync and Address (if enabled) are detected. Cleared
                when leaving Rx or FIFO is emptied.This bit is read only in
                Packet mode, rwc in Continuous mode.

        */
 uint8_t getIRQ1Flags(){
    return spi_read_u8(_REG_IRQ_FLAGS1);
 }
  uint8_t getIRQ2Flags(){
    return spi_read_u8(_REG_IRQ_FLAGS2);
 }
 
        /*
            The enter and exit conditions cannot be used independently of each
            other i.e. both should be enabled at the same time.

            The initial and the final state is the one configured in Mode in RegOpMode.
            The initial & final states can be different by configuring the modes
            register while the module is in intermediate mode.
            The pictorial description of the auto modes is shown below.

            Initial state defined       Intermediate State  Final state defined
            By Mode in RegOpMode       -------------------- By Mode in RegOpMode
                                       |                  |
                                       |                  |
            ----------------------------                  ----------
                        EnterCondition ^    ExitCondition ^

            Where Enter condition is one of:
                RFM69_AUTOMODE_ENTER_NONE_AUTOMODES_OFF
                RFM69_AUTOMODE_ENTER_RISING_FIFONOTEMPTY
                RFM69_AUTOMODE_ENTER_RISING_FIFOLEVEL
                RFM69_AUTOMODE_ENTER_RISING_CRCOK
                RFM69_AUTOMODE_ENTER_RISING_PAYLOADREADY
                RFM69_AUTOMODE_ENTER_RISING_SYNCADDRESS
                RFM69_AUTOMODE_ENTER_RISING_PACKETSENT
                RFM69_AUTOMODE_ENTER_FALLING_FIFONOTEMPTY (I.E. FIFOEMPTY)

            Where Exit condition is one of:
                RFM69_AUTOMODE_EXIT_NONE_AUTOMODES_OFF
                RFM69_AUTOMODE_EXIT_FALLING_FIFONOTEMPTY (I.E._FIFOEMPTY)
                RFM69_AUTOMODE_EXIT_RISING_FIFOLEVEL_OR_TIMEOUT
                RFM69_AUTOMODE_EXIT_RISING_CRCOK_OR_TIMEOUT
                RFM69_AUTOMODE_EXIT_RISING_PAYLOADREADY_OR_TIMEOUT
                RFM69_AUTOMODE_EXIT_RISING_SYNCADDRESS_OR_TIMEOUT
                RFM69_AUTOMODE_EXIT_RISING_PACKETSENT
                RFM69_AUTOMODE_EXIT_RISING_TIMEOUT

            And Intermediate state:
                RFM69_AUTOMODE_INTERMEDIATEMODE_SLEEP
                RFM69_AUTOMODE_INTERMEDIATEMODE_STANDBY
                RFM69_AUTOMODE_INTERMEDIATEMODE_RECEIVER
                RFM69_AUTOMODE_INTERMEDIATEMODE_TRANSMITTER

        */
        
  
 void setAutoMode(uint8_t enter, uint8_t exit, uint8_t intermediate_mode){
   spi_write_u8(RFM69_AUTO_MODES, enter+exit+intermediate_mode);
   }
void setMode(uint8_t mode){
spi_write_u8(RFM69_OPMODE,mode);
}


//Funcion que setea los parametros del RFM para recibir
void recibe(){
    /*
        Setup the automode such that we go into standby mode when a packet is
        available in the FIFO. Automatically go back into receiving mode when it
        is read.

        See the datasheet, p42 for information.
    */

    setAutoMode(RFM69_AUTOMODE_ENTER_RISING_PAYLOADREADY, RFM69_AUTOMODE_EXIT_FALLING_FIFONOTEMPTY, RFM69_AUTOMODE_INTERMEDIATEMODE_STANDBY);
    // one disadvantage of this is that the PayloadReady Interrupt is not asserted.
    // however, the intermediate mode can be detected easily.

    // p22 - Turn off the high power boost registers in receiving mode.
    if (tx_power_get()>13)
    {
        set(0,pa_1_on);
        set(0,pa_2_on);
    }

    // set the mode to receiver.
    setMode(RFM69_MODE_SEQUENCER_ON+RFM69_MODE_RECEIVER);
    
    state = RFM69_PLAIN_STATE_RECEIVING;
}



//Funcion para leer el FIFO
void readPacket(){
//!    // read it into the buffer.
//!    if (this->use_variable_length) {
//!        this->readVariableFIFO(this->packet_buffer[this->buffer_write_index], this->packet_length + this->use_variable_length);
//!    } else{
//!     
//!    }
   //spi_read_into(_REG_FIFO,packet_buffer[buffer_write_index],packet_length);

    spi_read_into(_REG_FIFO,packet_buffer[0],packet_length);
    for (uint16_t i=0;i<packet_length;i++){
      printch(packet_buffer[i],1);
      }
   // increase the write index.
   buffer_write_index = (buffer_write_index+1) % buffer_size;
}



//Funcion de interrupcion del DIO2
void poll(){
    println((char*)"Poll", 3);
    uint8_t flags1;
    // uint8_t flags2;

    flags1 = getIRQ1Flags();
    // flags2 = this->getIRQ2Flags();
    

    // debug_rfm("Flags1: "); debug_rfmln(flags1);
    switch (state){
        case (RFM69_PLAIN_STATE_RECEIVING):
            if (flags1 & RFM69_IRQ1_AUTOMODE){
                println((char*)"Automode in receiving!",2);
                print((char*)"Flags1: ",2); println(flags1,2);
                print((char*)"Flags2: ",2); println(getIRQ2Flags(),2);

                readPacket();
            }
            break;

        case (RFM69_PLAIN_STATE_SENDING):
            if ((flags1 & RFM69_IRQ1_AUTOMODE)==0){ // no longer in automode
                print((char*)"Flags1: ",2); println(flags1,2);
                print((char*)"Flags2: ",2); println(getIRQ2Flags(),2);

                recibe(); // we're done sending, set the receiving mode.
            }
            break;
        default:
            // this should not happen... 
            println((char*)"In undefined state!",2);
    };
}



bool available(){
    // return whether the indices do not align. If they do not align, read 
    // index has to catch up.
    // Overflows are not handled.
    // TODO: handle buffer overflows.
    return (buffer_read_index != buffer_write_index);
}


uint8_t read(void* buffer){
    println((char*)"Read",2);

    // no data to return.
    if (buffer_read_index == buffer_write_index){
        return 0;
    }

    // read packet length.
    uint8_t length = packet_length;
    uint8_t payload_start = 0;
    if (use_variable_length){

        // if variable length is used, read length from the first byte.
        length = packet_buffer[buffer_read_index][0];
        // debug_rfm("Length from packet: ");debug_rfm(length);
        // prevent buffer overflow, take shortest length of Rx length and packet length.
        length = (length > packet_length) ? packet_length : length;
        // debug_rfm("maxed length: ");debug_rfm(length);

        // payload starts one byte later.
        payload_start++;
    }

    // copy the message into the buffer.
    memcpy(buffer, &(packet_buffer[buffer_read_index][payload_start]), length);
    
    // increase the read index.
    buffer_read_index = (buffer_read_index+1) % buffer_size;

    // return the length of the packet written.
    return length;
}



void setBufferSize(uint8_t size){
    buffer_size = size;
}

void setPayloadLength(uint8_t length){
spi_write_u8(RFM69_PAYLOAD_LENGTH, length);
}
void setRawPacketLength(){
//!    // allocate the Tx Buffer
//!    this->tx_buffer = (uint8_t*) malloc(this->packet_length + this->use_variable_length);
    // set the length in the hardware.
    setPayloadLength(packet_length); // packet length byte is not included in length count. So NOT +1

    

}

void setPacketLength(uint8_t length){
    packet_length = length + use_addressing;

    // allocate the packet buffer, first allocate a list of pointers
    packet_buffer = (uint8_t**) malloc(buffer_size * sizeof(uint8_t*));
    for (uint8_t i = 0; i < buffer_size; i++){
        // then allocate the packet length per buffer slot.
        packet_buffer[i] = (uint8_t*) malloc(packet_length + use_variable_length);
    }

    // this is mostly a separate function such that it can be overloaded.
    setRawPacketLength();

}


