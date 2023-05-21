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
void print(char* str, int format){
    Serial.print(str,format);
}
void println(char* str){
    Serial.println(str);
}
void println(char* str, int format){
    Serial.println(str,format);
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
