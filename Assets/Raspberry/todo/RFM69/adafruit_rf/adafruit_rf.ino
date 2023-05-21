
#include <SPI.h>

#include<stdint.h>

//#include"utils.cpp"
#include"adafruit_rfm69_registers.h"
#define SSPin 10
#define SPIBAUD 100000


#define FREQ_433
//#define FREQ_433
//#define FREQ_433

// # The crystal oscillator frequency and frequency synthesizer step size.
// # See the datasheet for details of this calculation.
const float _FXOSC = 32000000.0;
const float _FSTEP = _FXOSC / 524288;
//Global buffer for SPI commands
uint8_t _BUFFER[4];

void setOutput(int pin, int value){
    digitalWrite(pin,value);
}

void spiBegin(){
SPI.beginTransaction(SPISettings(SPIBAUD, MSBFIRST, SPI_MODE0));
setOutput(SSPin, 0);

}
void spiEnd(){
setOutput(SSPin, 1);    
SPI.endTransaction();
}

void print(char* str){
    Serial.print(str);
}
float timeSec(){
    return millis()/1000;
}
void spi_read_into(uint8_t address,uint8_t* array, uint8_t length){
    int i=0;
    //Select
    spiBegin();
    _BUFFER[0] = address & 0x7F; //Strip MSB byte to read
    //Write address
    SPI.transfer(_BUFFER[0]);
    for (i=0;i<length;i++)
        array[i] = SPI.transfer(0xFF);
    spiEnd();

}
void spi_write_from(uint8_t address,uint8_t* array, uint8_t length){
    int i=0;
    spiBegin();
    SPI.transfer(address | 0b10000000);
    //El address se aumenta en 1 automaticamente
   //Serial.println("Writing SPI");
    for (i=0;i<length;i++){
      // Serial.println((char)array[i]);
      SPI.transfer(array[i]);}
    spiEnd();    
}
uint8_t spi_read_u8(uint8_t address){
    spi_read_into(address,_BUFFER,1);
    return _BUFFER[0];
}
uint8_t spi_write_u8(uint8_t address,uint8_t val){
    _BUFFER[0] = val;
    spi_write_from(address,_BUFFER,1);
    return _BUFFER[0];
}
void sleep_ms(int ms){
    delay(ms);
}

class _RegisterBits{
    public:
    uint8_t address;
    uint8_t mask;
    uint8_t offset;
    _RegisterBits(uint8_t _address, uint8_t _offset,uint8_t bits = 1){
        uint8_t i=0;
        mask=0;
        //TODO: check offset to be [0,7] and bits [1,8]
        address = _address;
        for (i=0;i<bits;i++){
            mask<<=1;
            mask|=1;
        }
        mask <<= _offset;
        offset = _offset;
        }
    void set(uint8_t val){
        uint8_t regVal = spi_read_u8(address);
        regVal &= ~mask;
        regVal |= (val & 0xFF) << offset;
        spi_write_u8(address,regVal);
    }
    uint8_t get(){
        uint8_t regVal = spi_read_u8(address);
        return ((regVal & mask) >> offset);
    }
    uint8_t debug(){
      Serial.print ("Mask: ");
      Serial.println(mask,BIN);
      return mask;     
  
    }
};


class RFM69{
    public:

    //Configuraciones que solo utilizan ciertos bits
    _RegisterBits data_mode = _RegisterBits(_REG_DATA_MOD, 5, 2);
    _RegisterBits modulation_type = _RegisterBits(_REG_DATA_MOD, 3, 2);
    _RegisterBits modulation_shaping = _RegisterBits(_REG_DATA_MOD, 0, 2);
    _RegisterBits temp_start = _RegisterBits(_REG_TEMP1, 3);
    _RegisterBits temp_running = _RegisterBits(_REG_TEMP1, 2);
    _RegisterBits sync_on = _RegisterBits(_REG_SYNC_CONFIG, 7);
    _RegisterBits sync_size = _RegisterBits(_REG_SYNC_CONFIG, 3, 3);
    _RegisterBits aes_on = _RegisterBits(_REG_PACKET_CONFIG2, 0);
    _RegisterBits pa_0_on = _RegisterBits(_REG_PA_LEVEL, 7);
    _RegisterBits pa_1_on = _RegisterBits(_REG_PA_LEVEL, 6);
    _RegisterBits pa_2_on = _RegisterBits(_REG_PA_LEVEL, 5);
    _RegisterBits output_power = _RegisterBits(_REG_PA_LEVEL, 0, 5);
    _RegisterBits rx_bw_dcc_freq = _RegisterBits(_REG_RX_BW, 5, 3);
    _RegisterBits rx_bw_mantissa = _RegisterBits(_REG_RX_BW, 3, 2);
    _RegisterBits rx_bw_exponent = _RegisterBits(_REG_RX_BW, 0, 3);
    _RegisterBits afc_bw_dcc_freq = _RegisterBits(_REG_AFC_BW, 5, 3);
    _RegisterBits afc_bw_mantissa = _RegisterBits(_REG_AFC_BW, 3, 2);
    _RegisterBits afc_bw_exponent = _RegisterBits(_REG_AFC_BW, 0, 3);
    _RegisterBits packet_format = _RegisterBits(_REG_PACKET_CONFIG1, 7, 1);
    _RegisterBits dc_free = _RegisterBits(_REG_PACKET_CONFIG1, 5, 2);
    _RegisterBits crc_on = _RegisterBits(_REG_PACKET_CONFIG1, 4, 1);
    _RegisterBits crc_auto_clear_off = _RegisterBits(_REG_PACKET_CONFIG1, 3, 1);
    _RegisterBits address_filter = _RegisterBits(_REG_PACKET_CONFIG1, 1, 2);
    _RegisterBits mode_ready = _RegisterBits(_REG_IRQ_FLAGS1, 7);
    _RegisterBits dio_0_mapping = _RegisterBits(_REG_DIO_MAPPING1, 6, 2);
    _RegisterBits dio_1_mapping = _RegisterBits(_REG_DIO_MAPPING1, 4, 2);
    _RegisterBits dio_2_mapping = _RegisterBits(_REG_DIO_MAPPING1, 2, 2);
    _RegisterBits dio_3_mapping = _RegisterBits(_REG_DIO_MAPPING1, 0, 2);
    _RegisterBits dio_4_mapping = _RegisterBits(_REG_DIO_MAPPING1+1, 6, 2);
    _RegisterBits dio_5_mapping = _RegisterBits(_REG_DIO_MAPPING1+1, 4, 2);
    //Extras
    int8_t _tx_power;
    int8_t tx_power;
    bool high_power;
    uint8_t* sync_word;
    uint16_t preamble_length;
    uint32_t frequency_mhz;
    float bitrate;
    float rssi;
    float last_rssi;
    float ack_wait;
    float receive_timeout;
    float xmit_timeout;
    uint8_t ack_retries;
    float ack_delay;
    uint8_t sequence_number;
    uint8_t seen_ids[8];
    uint8_t node;
    uint8_t destination;
    uint8_t identifier;
    uint8_t flags;
    uint8_t operation_mode;
    float temperature;
    uint8_t encryption_key[16];
    float frequency_deviation;
    int _reset_pin;

    
    void readAllRegs()
    {
      uint8_t regVal;
      
      Serial.println("Address - HEX - BIN");
      for (uint8_t regAddr = 1; regAddr <= 0x4F; regAddr++)
      {
        spiBegin();
        SPI.transfer(regAddr & 0x7F); // send address + r/w bit
        regVal = SPI.transfer(0);
        spiEnd();

        Serial.print(regAddr, HEX);
        print(" - ");
        Serial.print(regVal,HEX);
        print(" - ");
        Serial.println(regVal,BIN);


      }
      spiEnd();
    }    




    void init(uint8_t* _sync_word, int resetPin,uint8_t _preamble_length=4,bool _high_power=true,uint32_t baudrate = 2000000){
        uint8_t version=0;
        //Serial.println("Initial conf starts");
        //Serial.println("HOLA 2");
        _tx_power = 13;
        _reset_pin = resetPin;
        high_power = _high_power;
        //TODO:set SPI
        //TODO:set reset io
        //TODO:reset
        reset();
        //readAllRegs();
        
        version = spi_read_u8(_REG_VERSION);
        if (version != 0x24){
            Serial.print("Error: ID del RFM incorrecta");
            while(1){}
            //exit(-1);
        }
        idle();

        //Chip setup
        //Set FIFO TX condition to not empty and the default FIFO threshold to 15.
        spi_write_u8(_REG_FIFO_THRESH, 0b10001111);
        //Configure low beta off.
        spi_write_u8(_REG_TEST_DAGC, 0x30);
        //Disable boost.
        spi_write_u8(_REG_TEST_PA1, _TEST_PA1_NORMAL);
        spi_write_u8(_REG_TEST_PA2, _TEST_PA2_NORMAL);

        //set sync word
        //IMPORTANTE: Recuerda alocar el espacio para que no se sobreescriba
        sync_word_set( _sync_word); 
        preamble_length_set(_preamble_length);
        frequency_mhz_set(); 
        //TODO: set encryption key
        
        //PARA USUARIOS AVANZADOS ----------------------------------------------------------------------------------------------

        // Configure modulation for RadioHead library GFSK_Rb250Fd250 mode
        // by default.  Users with advanced knowledge can manually reconfigure
        // for any other mode (consulting the datasheet is absolutely
        // necessary!).
        bitrate_set(250000);  // 250kbs
        frequency_deviation_set(250000);  // 250khz

        modulation_shaping.set(0b01);  // Gaussian filter, BT=1.0
        rx_bw_dcc_freq.set(0b111);  // RxBw register = 0xE0
        rx_bw_mantissa.set(0b00);
        rx_bw_exponent.set(0b000);
        afc_bw_dcc_freq.set(0b111); // AfcBw register = 0xE0
        afc_bw_mantissa.set(0b00);
        afc_bw_exponent.set(0b000);
        packet_format.set(1);  // Variable length.
        dc_free.set(0b10);  // Whitening

        //-----------------------------------------------------------------------------------------------------------------------


        // Set transmit power to 13 dBm, a safe value any module supports.
        tx_power_set(13);

        //
        // initialize last RSSI reading
        last_rssi = 0.0;
        // """The RSSI of the last received packet. Stored when the packet was received.
        //    This instantaneous RSSI value may not be accurate once the
        //    operating mode has been changed.
        // """
        // initialize timeouts and delays delays
        ack_wait = 0.5;
        // """The delay time before attempting a retry after not receiving an ACK"""
        receive_timeout = 0.5;
        // """The amount of time to poll for a received packet.
        //    If no packet is received, the returned packet will be None
        // """
        xmit_timeout = 2.0;
        // """The amount of time to wait for the HW to transmit the packet.
        //    This is mainly used to prevent a hang due to a HW issue
        // """
        ack_retries = 5;
        // """The number of ACK retries before reporting a failure."""
        ack_delay = 0;
        // """The delay time before attemting to send an ACK.
        //    If ACKs are being missed try setting this to .1 or .2.
        // """
        //print("<------------------------------------------------------------------------------------------------------------------------------------------>")

        // initialize sequence number counter for reliabe datagram mode
        sequence_number = 0;
        // create seen Ids list
        //seen_ids = {0};

        // initialize packet header
        // node address - default is broadcast
        node = _RH_BROADCAST_ADDRESS;
        // """The default address of this Node. (0-255).
        //    If not 255 (0xff) then only packets address to this node will be accepted.
        //    First byte of the RadioHead header.
        // """
        // destination address - default is broadcast
        destination = _RH_BROADCAST_ADDRESS;
        // """The default destination address for packet transmissions. (0-255).
        //    If 255 (0xff) then any receiving node should accept the packet.
        //    Second byte of the RadioHead header.
        // """
        // ID - contains seq count for reliable datagram mode
        identifier = 0;
        // """Automatically set to the sequence number when send_with_ack() used.
        //    Third byte of the RadioHead header.
        // """
        // flags - identifies ack/reetry packet for reliable datagram mode
        flags = 0;
        // """Upper 4 bits reserved for use by Reliable Datagram Mode.
        //    Lower 4 bits may be used to pass information.
        //    Fourth byte of the RadioHead header.
        // """



        //Extras: paara algunos registros que no coinciden con la libreria del micropython
        // RSSI_CONFIG: 0x2
        spi_write_u8(0x23,0x02);
        //_REG_DIO_MAPPING1
        spi_write_u8(_REG_DIO_MAPPING1,0x00);
        //101 → FXOSC / 32
        spi_write_u8(_REG_DIO_MAPPING1+1,0b101);
        //RSSI_THRESH
        spi_write_u8(0x29,0xFF);
        //INIT payload length to 0
        spi_write_u8(0x38,0x40);
        //AutoRxRestartOn
        spi_write_u8(_REG_PACKET_CONFIG2,0x02);
        //Con esto se puede colocar un LED en DIO2 y ver los datos que se reciben y se envian
        dio_2_mapping.set(0b01);
        //Asi se puede saber si el buffer FIFO tiene algun dato
        dio_1_mapping.set(0b10);
        //En modo rx, da informacion acerca del RSSI (Recieved Signal Strength Indicator)
        dio_3_mapping.set(0b01);



        Serial.print("Initial configuration end\n");
    }
    void reset(){
        setOutput(_reset_pin,1);
        sleep_ms(1);
        setOutput(_reset_pin,0);
        sleep_ms(5);
    }
    void set_boost(uint8_t setting){
        //Set preamp boost if needed.
        if (_tx_power >= 18){
            spi_write_u8(_REG_TEST_PA1, setting);
            spi_write_u8(_REG_TEST_PA2, setting);
        }
    }
    void idle(){
        //Enter idle standby mode (switching off high power amplifiers if necessary).
        //Like RadioHead library, turn off high power boost if enabled.
        
        set_boost(_TEST_PA1_NORMAL);
        operation_mode_set(STANDBY_MODE);
               
    }
    void sleep(){
        operation_mode_set(SLEEP_MODE); 
    }
    void listen(){
        //Listen for packets to be received by the chip.  Use :py:func:`receive` to listen, wait
        //and retrieve packets as they're available.
        
        // Like RadioHead library, turn off high power boost if enabled.
        
        set_boost(_TEST_PA1_NORMAL);
        // Enable payload ready interrupt for D0 line.
        dio_0_mapping.set(0b01);
        // Enter RX mode (will clear FIFO!).
        //Serial.println("HOLAAA");        
        operation_mode_set(RX_MODE); 

    }
    void transmit(){
        // Transmit a packet which is queued in the FIFO.  This is a low level function for
        // entering transmit mode and more.  For generating and transmitting a packet of data use
        // :py:func:`send` instead.
        
        // # Like RadioHead library, turn on high power boost if enabled.
        set_boost(_TEST_PA1_BOOST);
        // # Enable packet sent interrupt for D0 line.
        dio_0_mapping.set(0b00);
        //readAllRegs(); 
        // # Enter TX mode (will clear FIFO!).

        operation_mode_set(TX_MODE);  
    }
    // .. warning:: Reading this will STOP any receiving/sending that might be happening!
    //WARNING:LOOP infinito
    float temperature_get(){
        // The internal temperature of the chip in degrees Celsius. Be warned this is not
        // calibrated or very accurate.

        // .. warning:: Reading this will STOP any receiving/sending that might be happening!
        // # Start a measurement then poll the measurement finished bit.
        temp_start.set(1);
        //WARNING:LOOP infinito
        while (temp_running.get() > 0){}
        temperature = 166.0 - (float)spi_read_u8(_REG_TEMP2);
        return temperature;
    }
    uint8_t operation_mode_get(){
        // """The operation mode value.  Unless you're manually controlling the chip you shouldn't
        // change the operation_mode with this property as other side-effects are required for
        // changing logical modes--use :py:func:`idle`, :py:func:`sleep`, :py:func:`transmit`,
        // :py:func:`listen` instead to signal intent for explicit logical modes.
        // """
        operation_mode = (spi_read_u8(_REG_OP_MODE) >> 2) &0b111;
        return operation_mode;
    }
    void operation_mode_set(uint8_t val){
        float start;
        //TODO: assert 0 <= val <= 4
        start = timeSec();
        while (!mode_ready.get()){
          
            if ((timeSec() - start) >= 3){
                     
                Serial.print ("Operation Mode couldnt be set\n");
                Serial.println(spi_read_u8(0x27),BIN);
                while (1){}
                //exit(-2);
            }
        }      
        // Set the mode bits inside the operation mode register.
        operation_mode = spi_read_u8(_REG_OP_MODE);
        operation_mode &= 0b11100011;
        operation_mode |= val << 2;
        //Serial.println(operation_mode,BIN);
        spi_write_u8(_REG_OP_MODE,operation_mode);

       
        // Wait for mode to change by polling interrupt bit.
        start = timeSec();
        while (!mode_ready.get()){
            if ((timeSec() - start) >= 3){
                Serial.print ("Timeout on Operation Mode Set\n");
                 Serial.println(spi_read_u8(_REG_OP_MODE),BIN)  ;      
                while (1){}
                //exit(-2);
            }
        }
    }
    //WARNING must free allocated memory after using
    uint8_t* sync_word_get(){
        
        // """The synchronization word value.  This is a byte string up to 8 bytes long (64 bits)
        // which indicates the synchronization word for transmitted and received packets. Any
        // received packet which does not include this sync word will be ignored. The default value
        // is 0x2D, 0xD4 which matches the RadioHead RFM69 library. Setting a value of None will
        // disable synchronization word matching entirely.
        // """
        // # Handle when sync word is disabled..
        if (!sync_on.get())return NULL;
        //WARNING must free allocated memory after using
        sync_word = (uint8_t*)malloc(sync_size.get()+2);
        sync_word[0] = sync_size.get()+1;
        spi_read_into(_REG_SYNC_VALUE1,sync_word+1,sync_size.get()+1);
        return sync_word;
    }
    void sync_word_set(uint8_t* wrd){
      uint8_t len = wrd[0];
        if (len == 0 || wrd == NULL)sync_on.set(0);
        
        else{
            //TODO: assert 1 <= len(val) <= 8
            spi_write_from(_REG_SYNC_VALUE1,wrd+1,len);
            //Sync_size: len(SYNC_WORD) - 1
            sync_size.set(len-1);
            sync_on.set(1);
        }
    }
    uint16_t preamble_length_get(){
        // The length of the preamble for sent and received packets, an unsigned 16-bit value.
        // Received packets must match this length or they are ignored! Set to 4 to match the
        // RadioHead RFM69 library.
        uint8_t msb = spi_read_u8(_REG_PREAMBLE_MSB);
        uint8_t lsb = spi_read_u8(_REG_PREAMBLE_LSB);
        return ((msb << 8) | lsb) & 0xFFFF;
    }
    void preamble_length_set(uint16_t val){
        spi_write_u8(_REG_PREAMBLE_MSB, (val >> 8) & 0xFF);
        spi_write_u8(_REG_PREAMBLE_LSB, val & 0xFF);
    }
    float frequency_mhz_get(){
        // """The frequency of the radio in Megahertz. Only the allowed values for your radio must be
        // specified (i.e. 433 vs. 915 mhz)!
        // """
        // # FRF register is computed from the frequency following the datasheet.
        // # See section 6.2 and FRF register description.
        // # Read bytes of FRF register and assemble into a 24-bit unsigned value.
        uint8_t msb = spi_read_u8(_REG_FRF_MSB);
        uint8_t mid = spi_read_u8(_REG_FRF_MID);
        uint8_t lsb = spi_read_u8(_REG_FRF_LSB);
        uint32_t frf = ((msb << 16) | (mid << 8) | lsb) & 0xFFFFFF;
        float frequency = (frf * _FSTEP) / 1000000.0;
        return frequency;

    }
    //WARNING: No funciona en micros porque requiere de enteros de 32 bits    
    void frequency_mhz_set(){
        //FRF = int((freq/_FSTEP)*1,000,000) & 0xFFFFFF
        uint8_t msb,lsb,mid;
        #ifdef FREQ_433
        msb = 0x6C;
        mid = 0x40;
        lsb = 0x00;
        #endif
        //TODO: añadir soporte para otras frecuencias
        


        //TODO: assert 290 <= val <= 1020
        // Calculate FRF register 24-bit value using section 6.2 of the datasheet.
        // unsigned long frf = int((val/_FSTEP)* 1000000.0) ;
        // Serial.print("FRF: ");
        // Serial.println(frf);
        
        // frf &=  0xFFFFFF;
        // // Extract byte values and update registers.
        // uint8_t msb = frf >> 16;
        // uint8_t mid = (frf >> 8) & 0xFF;
        // uint8_t lsb = frf & 0xFF;
        spi_write_u8(_REG_FRF_MSB, msb);
        spi_write_u8(_REG_FRF_MID, mid);
        spi_write_u8(_REG_FRF_LSB, lsb);
    }
    uint8_t* encryption_key_get(){
        // """The AES encryption key used to encrypt and decrypt packets by the chip. This can be set
        // to None to disable encryption (the default), otherwise it must be a 16 byte long byte
        // string which defines the key (both the transmitter and receiver must use the same key
        // value).
        // """
        // # Handle if encryption is disabled.
        if(aes_on.get()==0)return NULL;
        spi_read_into(_REG_AES_KEY1,encryption_key,16);
        return encryption_key;
    }
    void encryption_key_set(uint8_t* val){
        if (val==0)aes_on.set(0);
        else{
            // Set the encryption key and enable encryption.
            //TODO: assert len(val) == 16
            spi_write_from(_REG_AES_KEY1,val,16);
            aes_on.set(1);
        }
    }
    int8_t tx_power_get(){
        // The transmit power in dBm. Can be set to a value from -2 to 20 for high power devices
        // (RFM69HCW, high_power=True) or -18 to 13 for low power devices. Only integer power
        // levels are actually set (i.e. 12.5 will result in a value of 12 dBm).
        // """
        // # Follow table 10 truth table from the datasheet for determining power
        // # level from the individual PA level bits and output power register.
        uint8_t pa0 = pa_0_on.get();
        uint8_t pa1 = pa_1_on.get();
        uint8_t pa2 = pa_2_on.get();
        uint8_t current_output_power = output_power.get();
        if (pa0 &&  !pa1 &&  !pa2)
            //# -18 to 13 dBm range
            return -18 + current_output_power;
        if (!pa0 && pa1 && !pa2)
            //# -2 to 13 dBm range
            return -18 + current_output_power;
        if (!pa0 && pa1 && pa2 && !high_power)
            //# 2 to 17 dBm range
            return -14 + current_output_power;
        if (!pa0 && pa1 && pa2 && high_power)
            //# 5 to 20 dBm range
            return -11 + current_output_power;
        Serial.print("Tx power power amps state unknown!");
        while (1){}
        //exit(-3);
    }
    void tx_power_set(int8_t val){
        // Determine power amplifier and output power values depending on
        // high power state and requested power.

        uint8_t _pa_0_on = 0;
        uint8_t _pa_1_on = 0;
        uint8_t _pa_2_on = 0;
        uint8_t _output_power = 0;
        if (high_power){
            // Handle high power mode.
            //TODO: assert -2 <= val <= 20
            _pa_1_on = 1;
            if (val <= 13)
                _output_power = val + 18;
            else if (13 < val && val <= 17){
                _pa_2_on = 1;
                _output_power = val + 14;
                }
            else{
                //  # power >= 18 dBm
                //# Note this also needs PA boost enabled separately!
                _pa_2_on = 1;
                _output_power = val + 11;
                }
        }
        else{
            //Handle non-high power mode.
            //TODO: assert -18 <= val <= 13
            // Enable only power amplifier 0 and set output power.
            _pa_0_on = 1;
            _output_power = (val+ 18);
        }
        //# Set power amplifiers and output power as computed above.
        pa_0_on.set(_pa_0_on);
        pa_1_on.set(_pa_1_on);
        pa_2_on.set(_pa_2_on);
        output_power.set(_output_power);
        _tx_power = val;
    }
    float rssi_get(){
        // ""The received strength indicator (in dBm).
        // May be inaccuate if not read immediatey. last_rssi contains the value read immediately
        // receipt of the last packet.
        // """
        // # Read RSSI register and convert to value using formula in datasheet.
        rssi = -spi_read_u8(_REG_RSSI_VALUE) / 2.0;
        return rssi;
    }
    float bitrate_get(){
        //         """The modulation bitrate in bits/second (or chip rate if Manchester encoding is enabled).
        // Can be a value from ~489 to 32mbit/s, but see the datasheet for the exact supported
        // values.
        // """
        uint8_t msb = spi_read_u8(_REG_BITRATE_MSB);
        uint8_t lsb = spi_read_u8(_REG_BITRATE_LSB);
        bitrate = _FXOSC / ((msb << 8) | lsb);
        return bitrate;
    }
    void bitrate_set(float val){
        //TODO: assert (_FXOSC / 65535) <= val <= 32000000.0
        //# Round up to the next closest bit-rate value with addition of 0.5.
        uint32_t _bitrate = (int)((_FXOSC/val)+0.5) & 0xFFF;
        spi_write_u8(_REG_BITRATE_MSB, _bitrate >> 8);
        spi_write_u8(_REG_BITRATE_LSB, _bitrate & 0xFF);
    }
    float frequency_deviation_get(){
        //"""The frequency deviation in Hertz."""
        uint8_t msb = spi_read_u8(_REG_FDEV_MSB);
        uint8_t lsb = spi_read_u8(_REG_FDEV_LSB);
        frequency_deviation = _FSTEP * ((msb << 8) | lsb);
        return frequency_deviation;
    }
    void frequency_deviation_set(float val){
        //TODO: assert 0 <= val <= (_FSTEP * 16383)  # fdev is a 14-bit unsigned value
        // # Round up to the next closest integer value with addition of 0.5.
        uint32_t fdev = int((val / _FSTEP) + 0.5) & 0x3FFF;
        spi_write_u8(_REG_FDEV_MSB, fdev >> 8);
        spi_write_u8(_REG_FDEV_LSB, fdev & 0xFF);
    }

    bool packet_sent(){
        //Transmit status
        return (spi_read_u8(_REG_IRQ_FLAGS2) & 0x8) >> 3;
    }
    bool payload_ready(){
                // """Receive status"""
        uint8_t p = spi_read_u8(_REG_IRQ_FLAGS2);
        //Serial.println(p,BIN);
        return (p & 0x4) >> 2;
    }

    bool send(uint8_t* data,uint8_t len, bool keep_listening = false, uint16_t _destination=256, uint16_t _node=256,uint16_t _identifier= 256, uint16_t _flags = 256){
        // """Send a string of data using the transmitter.
        // You can only send 60 bytes at a time
        // (limited by chip's FIFO size and appended headers).
        // This appends a 4 byte header to be compatible with the RadioHead library.
        // The header defaults to using the initialized attributes:
        // (destination,node,identifier,flags)
        // It may be temporarily overidden via the kwargs - destination,node,identifier,flags.
        // Values passed via kwargs do not alter the attribute settings.
        // The keep_listening argument should be set to True if you want to start listening
        // automatically after the packet is sent. The default setting is False.

        // Returns: True if success or False if the send timed out.
        // """
        // # Disable pylint warning to not use length as a check for zero.
        // # This is a puzzling warning as the below code is clearly the most
        // # efficient and proper way to ensure a precondition that the provided
        // # buffer be within an expected range of bounds.  Disable this check.
        // # pylint: disable=len-as-condition
        // TODO: assert 0 < len(data) <= 60
        // # pylint: enable=len-as-condition
        idle(); //# Stop receiving to clear FIFO and keep it clear.
        // # Fill the FIFO with a packet to send.
        // # Combine header and data to form payload
        uint32_t i=0;
        char* payload = (char*)malloc(5 + len + 1);
        
        payload[0] = 4 + len;
        if (_destination >=256 )  // use attribute
            payload[1] = destination;
        else//  # use kwarg
            payload[1] = _destination;
        if (_node >= 256) // use attribute
            payload[2] = node;
        else  //# use kwarg
            payload[2] = _node;
        if (identifier >=256)  // use attribute
            payload[3] = identifier;
        else  //# use kwarg
            payload[3] = _identifier;
        if (flags >=256)  // use attribute
            payload[4] = flags;
        else  // use kwarg
            payload[4] = _flags;
        for (i=0;i<= len;i++)
            payload[5+i] = data[i];
           
        // # Write payload to transmit fifo
        spi_write_from(_REG_FIFO, payload,5+len);
        Serial.print("Payload: ");
        Serial.println(payload+5);

        // Serial.println((char)spi_read_u8(_REG_FIFO));              
    
        free(payload);
        // # Turn on transmit mode to send out the packet.       
        transmit();
       
        // # Wait for packet sent interrupt with explicit polling (not ideal but
        // # best that can be done right now without interrupts).
        
        float start = timeSec();
        bool timed_out = false;
        while (!timed_out && !packet_sent()){
            if ((timeSec() - start) >= xmit_timeout)
                timed_out = true;
        }
        // # Listen again if requested.
        if (keep_listening)
            listen();
        else  //# Enter idle mode to stop receiving other packets.
      
            idle();
        return !timed_out;
    }
    bool send_with_ack(uint8_t* data,uint8_t len){
        // Reliable Datagram mode:
        // Send a packet with data and wait for an ACK response.
        // The packet header is automatically generated.
        // If enabled, the packet transmission will be retried on failure
        int retries_remaining = 0;
        bool got_ack = false;
        if (ack_retries)retries_remaining = ack_retries;
        else retries_remaining=1;
        sequence_number = (sequence_number+1) & 0xFF;
        while (!got_ack && retries_remaining){
            identifier = sequence_number;
            send(data,len,true);
            // Don't look for ACK from Broadcast message
            if (destination = _RH_BROADCAST_ADDRESS)got_ack = true;
            else{
                //TODO: terminar
            }

        }
        return false;
    }
    char* receive(bool keep_listening=true,bool with_ack = false, float timeout = 0,bool with_header = false){
        // Wait to receive a packet from the receiver. If a packet is found the payload bytes
        // are returned, otherwise None is returned (which indicates the timeout elapsed with no
        // reception).
        // If keep_listening is True (the default) the chip will immediately enter listening mode
        // after reception of a packet, otherwise it will fall back to idle mode and ignore any
        // future reception.
        // All packets must have a 4 byte header for compatibilty with the
        // RadioHead library.
        // The header consists of 4 bytes (To,From,ID,Flags). The default setting will  strip
        // the header before returning the packet to the caller.
        // If with_header is True then the 4 byte header will be returned with the packet.
        // The payload then begins at packet[4].
        // If with_ack is True, send an ACK after receipt (Reliable Datagram mode)
        // """
        bool timed_out = false;
        int start = 0;
        uint8_t fifo_length;
        uint8_t* packet= NULL;
        if (timeout == 0)timeout = receive_timeout;
        if (timeout!=0){
            //readAllRegs();
            //while(1){}
            listen();
            start = timeSec();
            while (!timed_out && !payload_ready()){
                //delay(20);
                if ((timeSec() - start) >= xmit_timeout){
                    Serial.println("Timed out");
                    timed_out = true;
                    }
            }

        }
       
        last_rssi = rssi_get();
        // Enter idle mode to stop receiving other packets.
        idle();
         
        if (!timed_out){
            fifo_length = spi_read_u8(_REG_FIFO);
            Serial.println("FIFO LEN: " + String(fifo_length));
            //  # Handle if the received packet is too small to include the 4 byte
            // # RadioHead header and at least one byte of data --reject this packet and ignore it.
            if (fifo_length > 0){
                packet = (uint8_t*)malloc(fifo_length+2);
                packet[0] = fifo_length;
                packet[fifo_length+1] = '\0';
                spi_read_into(_REG_FIFO,packet+1,fifo_length);
                //print(packet);
            }
            if (fifo_length < 5){
                packet = NULL;
            }
            else{
                if (node != _RH_BROADCAST_ADDRESS && packet[1] != _RH_BROADCAST_ADDRESS && packet[1] != node){
                    free(packet);
                    packet = NULL;
                }
                //# send ACK unless this was an ACK or a broadcast
                else if (with_ack && (packet[4]&_RH_FLAGS_ACK)==0 && packet[1] != _RH_BROADCAST_ADDRESS){
                    if (ack_delay != 0)sleep_ms((int)(ack_delay/1000));
                    //# send ACK packet to sender (data is b'!')
                    send("!",false,packet[2],packet[1],packet[3],packet[4]|_RH_FLAGS_ACK);
                    // # reject Retries if we have seen this idetifier from this source before
                    if (seen_ids[packet[2]] == packet[3] && packet[4]&_RH_FLAGS_RETRY){
                        free(packet);
                        packet= NULL;
                    }
                    else{ //Save identifier from source
                        seen_ids[packet[2]] = packet[3];
                    }

                }
                if (!with_header && packet != NULL){
                     //TODO: # skip the header if not wanted
                }
            }

        }
        
        if (keep_listening)listen();        
        else idle();
        return (char*)packet;

    }











};
















const int slaveSelectPin = 10;
int ResetPin = 9;
//PRIMER BYTE es el tamaño del array 
uint8_t synch[] = {3,0xAA,0x2D,0xD4};
//uint8_t sync_size = 3;
RFM69 radio;



void setup() {
  Serial.begin(9600);
  SPI.begin();
  pinMode(slaveSelectPin,OUTPUT);
  pinMode(ResetPin,OUTPUT);
  digitalWrite(slaveSelectPin,HIGH);
  digitalWrite(ResetPin,LOW);
  radio.init(synch,ResetPin);
  //radio.readAllRegs();
  //while (1);
  
  
  
  

}

void loop() {
  Serial.println("Ciclo");
  //radio.send("hola",4);

  int i=0;
  char* packet = radio.receive();
  if (!(packet == NULL || packet[0] == 0)){
    for (i=0;i<packet[0];i++){
  Serial.print(packet[i+1]);
  }
  Serial.println("");
  }
  //radio.readAllRegs();
  free(packet);
  delay(100);
}
