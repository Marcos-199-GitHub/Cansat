#line 1 "I:/CanSat/Nueva carpeta/RFM69/Receptor/MyProject.c"






sbit Chip_Select at RA4_bit;
sbit Chip_Select_Direction at TRISA4_bit;
sbit Correct at RD7_bit;
sbit Correct_Direction at TRISD7_bit;
#line 26 "I:/CanSat/Nueva carpeta/RFM69/Receptor/MyProject.c"
void Soft_UART_Write_Text(char* udata){
int i=0;
do{

i++;
}
while (udata[i] != 0) ;
}


void main() {
char error;
unsigned char udt = 55;
char str[10];


UART1_Init(9600);
if (error > 0) {
 while(1) ;
}

UART1_Write_Text("Serial correcto\n");
Chip_Select = 1;
Chip_Select_Direction = 0;
Correct_Direction = 0;
Correct = 1;
Chip_Select = 1;





SPI1_Init();


UART1_Write_Text("SPI iniciado\n");
while (1){

Chip_Select = 0;

SPI1_Write(0b01011010);

Delay_ms (10);


udt = SPI1_Read(0b01011010);
 Delay_ms (500);
Chip_Select = 1;
Delay_ms (500);
if (udt != 0b01010101){
IntToStr(udt,str) ;


UART1_Write_Text("Dato recibido\n");
 Correct = 0;

 UART1_Write(udt) ;

UART1_Write_Text(str) ;


UART1_Write('\n') ;
}
else {
Correct = 1;

 UART1_Write_Text("RFM correcto\n");
}
}
}
