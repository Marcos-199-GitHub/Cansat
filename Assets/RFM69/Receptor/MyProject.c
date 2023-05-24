//#include "hand_SPI.h"
//Hardware SPI
//SCK (clock): B1
//SDI (MISO): B0
//SDO (MOSI): C7 (RX)
//////////////////////////////
sbit Chip_Select at RA4_bit;
sbit Chip_Select_Direction at TRISA4_bit;
sbit Correct at RD7_bit;
sbit Correct_Direction at TRISD7_bit;
// End DAC module connections
/*
// Software SPI module connections
sbit SoftSpi_SDI at RB2_bit;
sbit SoftSpi_SDO at RB3_bit;
sbit SoftSpi_CLK at RB4_bit;

sbit SoftSpi_SDI_Direction at TRISB2_bit;
sbit SoftSpi_SDO_Direction at TRISB3_bit;
sbit SoftSpi_CLK_Direction at TRISB4_bit;
// End Software SPI module connections
 */



void Soft_UART_Write_Text(char* udata){
int i=0;
do{
//Soft_UART_Write(udata[i]) ;
i++;
}
while (udata[i] != 0) ;
}


void main() {
char error;
unsigned char udt = 55;
char str[10];

//error = Soft_UART_Init(&PORTD, 7, 6, 9600, 0); // Initialize Soft UART at 9600 bps
UART1_Init(9600);
if (error > 0) {
    while(1) ;                            // Stop program
}
//Soft_UART_Write_Text("Serial por software correcto\n");
UART1_Write_Text("Serial correcto\n");
Chip_Select = 1;                       // Deselect DAC
Chip_Select_Direction = 0;             // Set CS# pin as Output
Correct_Direction = 0;
Correct = 1;
Chip_Select = 1;

 // Initialize the SPI1 module with default settings
// Set SPI1 module to master mode, clock = Fosc/64, data sampled at the middle of interval, clock idle state low and data transmitted at low to high edge:
//SPI1_Init_Advanced(_SPI_MASTER_OSC_DIV16, _SPI_DATA_SAMPLE_MIDDLE, _SPI_CLK_IDLE_LOW, _SPI_LOW_2_HIGH);
//Soft_SPI_Init(); // Init Soft_SPI
SPI1_Init();
//spiInit();
//Soft_UART_Write_Text("SPI iniciado\n");
UART1_Write_Text("SPI iniciado\n");
while (1){
//LOW
Chip_Select = 0;
//Soft_SPI_Write(16);
SPI1_Write(0b01011010);
//SPItransfer(0x10);
Delay_ms (10);
//udat = Soft_SPI_Read(0);
//udat = SPIRead8Bit();
udt = SPI1_Read(0b01011010);
 Delay_ms (500);
Chip_Select = 1;
Delay_ms (500);
if (udt != 0b01010101){
IntToStr(udt,str)   ;
//Soft_UART_Write_Text("RFM incorrecto\n");
//Soft_UART_Write_Text("Dato recibido\n");
UART1_Write_Text("Dato recibido\n");
    Correct = 0;
    //Soft_UART_Write(udt)  ;
    UART1_Write(udt)  ;
//Soft_UART_Write_Text(str)  ;
UART1_Write_Text(str)  ;

//Soft_UART_Write('\n')  ;
UART1_Write('\n')  ;
}
else {
Correct = 1;
 //Soft_UART_Write_Text("RFM correcto\n");
 UART1_Write_Text("RFM correcto\n");
}
}
}