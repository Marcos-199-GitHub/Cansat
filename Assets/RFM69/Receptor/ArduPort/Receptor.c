#define HEX 16
#define BIN 2
#define SSPin PIN_A5

#include <Receptor.h>



#use spi (MASTER, SPI1, ENABLE=PIN_A5, BAUD=10000, MODE=0, BITS=8, STREAM=SPI_1)

#byte porta = 0xf80 // Identificador para el puerto A. 
#byte portb = 0xf81 // Identificador para el puerto B. 
#byte portc = 0xf82 // Identificador para el puerto C. 
#byte portd = 0xf83 // Identificador para el puerto D. 
#byte porte = 0xf84 // Identificador para el puerto E.





#include <stddef.h>
#include <stdlibm.h>
#include <stdint.h>


#include <adafruit_rfm69_registers.h>
#include <utils.h>
#include <afadruit_rfm69.h>

#ZERO_RAM


//Define la interrupci�n por recepci�n Serial
static void RDA_isr(void)
{  
 
}


/* TODO: Use usb_cdc_putc() to transmit data to the USB
virtual COM port. Use usb_cdc_kbhit() and usb_cdc_getc() to
receive data from the USB virtual COM port. usb_enumerated()
can be used to see if connected to a host and ready to
communicate. */

void main()
{

   //RFM69 radio;
   //PRIMER BYTE es el tama�o del array 
   uint8_t synch[] = {3,0xAA,0x2D,0xD4};
   int ResetPin = RF_Reset;
   //radio.init(synch,ResetPin);
   
   setup_adc_ports(NO_ANALOGS, VSS_VDD);
   usb_init();
   //init(synch,ResetPin);
   while(TRUE)
   {
   readAllRegs();
   //radio.readAllRegs();
//!   println((char*)"HOLA USB");
//!   println(134,HEX);
//!   println(47,BIN);
   delay_ms(1000);
      //TODO: User Code
   }

}
