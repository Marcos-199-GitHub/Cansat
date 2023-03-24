//todo esto depende del hardware
#define SSPin 10
void Delay(int ms){
    delay(ms);
}
void _spiWrite(char data){
    SPI.transfer(data);
}
void empty(char* arr,int n){
  int i=0;
  for (i=0;i<n;i++)arr[i]=0;
}
char _spiRead(char dummy){
  return SPI.transfer(dummy);
}
void spiBegin(){
SPI.beginTransaction(SPISettings(100000, MSBFIRST, SPI_MODE0));
digitalWrite(SSPin, LOW);

}
void spiEnd(){
digitalWrite(SSPin, HIGH);    
SPI.endTransaction();
}
void print(char* str){
Serial.print(str);
}
void println(char *str){
Serial.println(str);
}
/////////////////////////
/////////////////////////
////////////////////////
/////////////////////////
//No hay que modificar estas

char spiRead8(char address){
    char r;
    spiBegin();
    _spiWrite(address & 0b01111111);
    r = _spiRead(0xFF);
    spiEnd();
    return r;
}
void spiWrite8(char address,char dato){
    spiBegin();
    _spiWrite(address | 0b10000000);
    _spiWrite(dato);
    spiEnd();
}
void spiWriteFrom(char address, char* datos, unsigned int n){
    int i=0;
    spiBegin();
    _spiWrite(address | 0b10000000);
    //El address se aumenta en 1 automaticamente
    for (i=0;i<n;i++)_spiWrite(datos[i]);
    spiEnd();
}
spiReadInto(char address,char* save, int length){
    int i=0;
    spiBegin();
    _spiWrite(address&0x7F);
    for (i=0;i<length;i++)save[i] = _spiRead(0xFF);
    spiEnd();
}
void compare(char d1, char d2,char block){
    if (d1==d2)println("OK");
    else{
        println("No OK");
        if (block){while(1){}}
    }    
}

void spiWrite(word WrPara)                
{                                                       
 byte bitcnt; 
 char ad = WrPara >> 8;
 char da = WrPara & 0x00FF;   
 spiWrite8(ad,da);
}   
