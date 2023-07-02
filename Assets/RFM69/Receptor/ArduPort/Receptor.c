#define HEX 16
#define BIN 2
#define DEC 10
#define UDEC 11
#define SSPin PIN_A5
#define TIMER_START 114
#define TIMER_STEP_MS 3
#define FREQ_433

#include <Receptor.h>



#use spi (MASTER, SPI1, ENABLE=PIN_A5, BAUD=1000000, MODE=0, BITS=8, STREAM=SPI_1)

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


//Define la interrupción por recepción Serial
static void RDA_isr(void)
{  
 
}


/* TODO: Use usb_cdc_putc() to transmit data to the USB
virtual COM port. Use usb_cdc_kbhit() and usb_cdc_getc() to
receive data from the USB virtual COM port. usb_enumerated()
can be used to see if connected to a host and ready to
communicate. */

/*
Conexiones
MOSI: RX (23)
MISO: B0 (00) (SDI)
CLK: B1 (01) (SCK)
CS: A5 (13)
Reset: E0
*/

void main()
{

/*
   //Timer de 8 bits
//timer0 (RTCC_INTERNAL), Preescaler de 256, timer de 8 bits
setup_timer_0(RTCC_INTERNAL | RTCC_DIV_256 | RTCC_8_bit); 
//el timer se va a desbordar dependiendo de la formula:
//tiempo_desbordamiento =  (Valor_maximo_del_timer * (4*Preescaler))/Freq

//EJEMPLO, 
//para un divisor de 64, reloj de 48 MHZ y tiempo de 1ms
//Dado que el oscilador funciona a 48 Mhz, la formula es:
//t = 256*4*64 / 48000000 = 1.365 ms (aprox. 1ms),
//Para hacerlo mas preciso, es posible inicializar el timer a un valor mayor a 0, por lo que el desbordamiento ocurriria antes:
//con un valor de inicio de 67 (contando el 0, esto es 68 pasos) (256-68 = 180), el valor da 1.0026ms


set_rtcc(TIMER_START);
//Interrupciones del timer
enable_interrupts(INT_RTCC);
enable_interrupts(GLOBAL);

*/
   //RFM69 radio;
   //PRIMER BYTE es el tamaño del array 
   uint8_t synch[] = {3,0xAA,0x2D,0xD4};
   int ResetPin = RF_Reset;
   setOutput(INDICATOR_LED,0);
   //radio.init(synch,ResetPin);
   
   setup_adc_ports(NO_ANALOGS, VSS_VDD);
   usb_init();
   
   //Esperar un segundo antes de iniciar
   sleep_ms(1000);
   while (!checkId()){
   usb_task();
   println((char*)"Id incorrecto", 2);
   }
   setOutput(INDICATOR_LED,1);
   
   init(synch,ResetPin);
   println((char*)"INIT DONE", 3);
//!   readAllRegs();
//!   while(1){}
   while(TRUE)
   {
   char* packet = receive(1,0,0,0);
   if (!(packet == NULL || packet[0] == 0)){
    println ((char*)"Packet",2);
    for (uint16_t i=0;i<packet[0]-1;i++){
      printch(packet[i+1],1);
      }
  //println((char*)"",0);
   }
   else println((char*) "Esperando",2);
  //radio.readAllRegs();
  free(packet);
   //println(globalSec,DEC);
   
   //readAllRegs();
   //print((char*)"RFM ");
   
   //if (!checkId())println((char*)"incorrecto");
   //else println((char*)"Correcto");
   //radio.readAllRegs();
//!   println((char*)"HOLA USB");
//!   println(134,HEX);
//!   println(47,BIN);
   delay_ms(10);
      //TODO: User Code
   }

}

/*

//Interrupcion del timer
#INT_RTCC  //TIMER0
void timer0(void){
   set_rtcc(TIMER_START); //Timer0
   //println((char*)"Timer INT");
   globalMs += TIMER_STEP_MS ;       
   if (globalMs >= 1000){
   globalSec ++;
   globalMs -= 1000;
   }
   if (globalSec >= 60){
   globalMin ++;
   globalSec -= 60;
   }
   //Inicializar todo al llegar a 60 minutos
   //Puede introducir un bug rarisimo en el que una diferencia entre tiempos de negativa, pero es muy poco probable
   //Y aun asi el bug ocurrira cuando gloabalMin desborde
   if (globalMin >= 60){
   globalMs = 0;
   globalSec= 0;
   globalMin = 0;
   }
}

*/
