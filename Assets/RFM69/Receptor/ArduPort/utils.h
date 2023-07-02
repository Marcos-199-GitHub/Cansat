//Global buffer for SPI commands
uint8_t _BUFFER[4];

void setOutput(int pin, int value){
    output_bit(pin,value);
}
void usbPrint(char* str, int debug = 0){
      usb_task();  //Verifica la comunicación USB
      if(usb_enumerated() && debug <= debugLevel) {
      
         printf(usb_cdc_putc,str); 
      }
}
void usbPrint(char str, int debug = 0){
      usb_task();  //Verifica la comunicación USB
      if(usb_enumerated() && debug <= debugLevel) {
         printf(usb_cdc_putc,"%c",str); 
      }
}


void spiBegin(){
//SPI.beginTransaction(SPISettings(SPIBAUD, MSBFIRST, SPI_MODE0));
setOutput(SSPin, 0);

}
void spiEnd(){
setOutput(SSPin, 1);    
//SPI.endTransaction();
}

void print(char* str, int debug = 0){
    //Serial.print(str);
    usbPrint(str, debug);
}

void printch(char str, int debug = 0){
   usbPrint(str, debug);

}
void print(int16 str, int format, int debug = 0){
    char converted[11];
    int i;
    if (format == HEX) sprintf(converted,"0x%02X",str);
    else if (format == BIN){
    //Conversion manual
    converted[0] = '0';
    converted[1] = 'b';
    for (i=0;i<8;i++){
    converted[i+2] = ((str >> (7-i)) & 0x01) + '0';
    }
    converted[10] = '\0';
    }
    else if (format == DEC){
      sprintf(converted,"%Ld",str);
    }
    else if (format == UDEC){
      sprintf(converted,"%Lu",str);
    }
    usbPrint(converted, debug);
    //Serial.print(str,format);
}
void println(char* str, int debug = 0){
    usbPrint(str, debug);
    usbPrint((char*)"\n", debug);
    //Serial.println(str);
}
void println(int str, int format, int debug = 0){
    print(str,format, debug);
    usbPrint((char*)"\n", debug);
    //Serial.println(str,format);
}
void print(float str, int debug = 0){
 char converted[11];
 sprintf(converted,"%03f",str);
 usbPrint(converted, debug);
}
void println(float str, int debug = 0){
    print(str, debug);
    usbPrint((char*)"\n", debug);
}

float timeSec(){
   float t;
   t = (float)(60*globalMin);
   t += (float)globalSec;
   t += (float)((float)globalMs*0.001);
   //println(t);
   return t;
}
void spi_read_into(uint8_t address,uint8_t* array, uint8_t length){
    int i=0;
    //Select
    spiBegin();
    _BUFFER[0] = address & 0x7F; //Strip MSB byte to read
    //Write address
    spi_write(_BUFFER[0]);
    delay_us(100);  // Tiempo para que el esclavo responda
    for (i=0;i<length;i++)
        array[i] = spi_read(0xFF);
    spiEnd();

}
void spi_write_from(uint8_t address,uint8_t* array, uint8_t length){
    int i=0;
    spiBegin();
    spi_write(address | 0b10000000);
    //El address se aumenta en 1 automaticamente
   //Serial.println("Writing SPI");
    for (i=0;i<length;i++){
      // Serial.println((char)array[i]);
      spi_write(array[i]);}
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
    delay_ms(ms);
}
