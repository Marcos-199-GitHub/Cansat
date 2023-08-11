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
#include <plain_rfm.h>

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

uint8_t* buff;
uint16_t buff_len = 256;
uint16_t buff_i = 0;
void poll2(){
 printch('?',2);
 //println((char*)"FIFO Not Empty", 2);
 if (using_spi)spiEnd();
    readBytes =0;
    //Select
    spiBegin();
    //Write address
    spi_write(_REG_FIFO& 0x7F);
    delay_us(100);  // Tiempo para que el esclavo responda
    for (readBytes=0;readBytes<packet_length;readBytes++){
        buff[readBytes] = spi_read(0xFF);
        printch(buff[readBytes],1);
        }
    spiEnd();

 //spi_read_into(_REG_FIFO,packet_buffer[buff_i],packet_length);
 
}
void onReady(){
   printch('!',2);
   //println((char*)"Payload Ready", 2);
   if (using_spi)spiEnd();
   buff_i += readBytes;
   spi_read_into(_REG_FIFO,packet_buffer[buff_i],packet_length);
   buff_i += readBytes;
   for (uint16_t i=0;i<buff_i;i++)printch(packet_buffer[i],1);
   buff_i = 0;
}

#INT_EXT2            //Funcion de la Interrupcion
void DIO_INT()
{
   fifoNotEmpty = digitalRead(DIO_2);
   payloadReady = digitalRead(DIO_0);
   fifoThresh  = digitalRead(DIO_1);
   if (fifoThresh) poll2();
   if (payloadReady) onReady();
   
   //println((char*)"DIO INT", 2);
   
}



void main1(){
   uint8_t synch[] = {3,0xAA,0x2D,0xD4};
   int ResetPin = RF_Reset;
   buff = (uint8_t*)malloc(buff_len);
   //setOutput(INDICATOR_LED,0);
   //radio.init(synch,ResetPin);
   setup_adc_ports(NO_ANALOGS, VSS_VDD);
   usb_init();
   //Esperar un segundo antes de iniciar
   sleep_ms(1000);
   while (!checkId()){
   usb_task();
   println((char*)"Id incorrecto", 2);
   }
   //setOutput(INDICATOR_LED,1);
   
   init(synch,ResetPin);
   println((char*)"INIT DONE", 3);
   set(0x00,dio_2_mapping);
   set(0x00,dio_1_mapping);
   
   //Interrupciones
//!   enable_interrupts(GLOBAL);    //Habilita todas las interrupciones
//!   enable_interrupts(INT_EXT2);   //Habilita la interrupción externa en el pin B2
//!   //Interrucion en el cambio de Low to High
//!   ext_int_edge(L_TO_H); 
  
  listen();
  int notEmpty=0;
  set(1,packet_format);  // 1 - Variable length, 0 - Fixed Length
  set(64,payload_length);
  packet_length = get(payload_length);

   while(TRUE)
   {
   notEmpty = (spi_read_u8(_REG_IRQ_FLAGS2)&0b01000000) >> 6;
   payloadReady = payload_ready();
   
   if (payloadReady){
   idle();
   //println((char*)"DATOS",3);
   packet_length = spi_read_u8(_REG_FIFO);
   print((char*)"FIFO LEN: ",2);
   sprintf(str,"%u",packet_length);
   println(str,2);
   
       readBytes =0;
       //Select
       spiBegin();
       //Write address
       spi_write(_REG_FIFO& 0x7F);
       delay_us(100);  // Tiempo para que el esclavo responda
       //packet_length = spi_read(0xFF);

       for (readBytes=0;readBytes<packet_length;readBytes++){
        buff[readBytes] = spi_read(0xFF);
        //if (buff[readBytes]==0)break;
        printch(buff[readBytes],1);
        }
    spiEnd();
    listen();
   }
   }
}

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
   //setOutput(INDICATOR_LED,0);
   //radio.init(synch,ResetPin);
   
   setup_adc_ports(NO_ANALOGS, VSS_VDD);
   usb_init();
   
   //Esperar un segundo antes de iniciar
   sleep_ms(1000);
   while (!checkId()){
   usb_task();
   println((char*)"Id incorrecto", 2);
   }
   //setOutput(INDICATOR_LED,1);
   
   init(synch,ResetPin);
   println((char*)"INIT DONE", 3);
   set(0x00,dio_2_mapping);
   //Interrupciones
//!   enable_interrupts(GLOBAL);    //Habilita todas las interrupciones
//!   enable_interrupts(INT_EXT2);   //Habilita la interrupción externa en el pin B2
//!   //Interrucion en el cambio de Low to High
//!   ext_int_edge(L_TO_H); 
  
  //Esto ya funciona

   while(TRUE)
   {
   //Depuracion
//!   int b3 = digitalRead(BUTTON_3);
//!   if (b3 == 0){
//!      //Alternar el nivel de verbosidad
//!      debugLevel = (debugLevel + 1) % 4;
//!      print((char*)"Verbose level ",1);
//!      sprintf(str,"%u",debugLevel);
//!      println(str,1);
//!   }
//!   button3_last_state = b3;
   char* packet = receive(1,0,0,0);
   if (!(packet == NULL || packet[0] == 0)){
    println ((char*)"Packet",3);
    for (uint16_t i=0;i<packet[0]-1;i++){
      printch(packet[i+1],1);
      }
   }
   else println((char*) "Esperando",1);
  //radio.readAllRegs();
  free(packet);

   }
   
   //Esto quizas sea mejor
   //while (true){
   //receiveFast(1,0,0,0);
   //}

}




void main2(){

uint8_t synch[] = {3,0xAA,0x2D,0xD4};
int ResetPin = RF_Reset;
//setOutput(INDICATOR_LED,0);

setup_adc_ports(NO_ANALOGS, VSS_VDD);
usb_init();
//Esperar un segundo antes de iniciar
sleep_ms(1000);
while (!checkId()){
   usb_task();
   println((char*)"Id incorrecto", 2);
}
println((char*)"Hola", 3);
//setOutput(INDICATOR_LED,1);
   
init(synch,ResetPin);
println((char*)"INIT DONE", 3);
set(0x00,dio_2_mapping);
//Interrupciones
enable_interrupts(GLOBAL);    //Habilita todas las interrupciones
enable_interrupts(INT_EXT2);   //Habilita la interrupción externa en el pin B2
   //Interrucion en el cambio de High-Low
ext_int_edge(H_TO_L); 
  
// tell the RFM to represent whether we are in automode on DIO 2.
set(0b11,dio_2_mapping);
setBufferSize(5);
setPacketLength(64);


uint8_t buffer[64] = {0}; // receive buffer.

// use first four bytes of that buffer as uint32.
uint16_t counter = 0;

uint16_t oldcounter = 0; // keep track of the counter.
uint16_t packetloss = 0; // counter for missed packets.

// counter to indicate total number of received packets.
uint16_t received_packets = 0;

// time on which we start receiving.
/////uint32_t start_time = millis();


   while(TRUE)
   {
         println((char*)"Loop",3);
        uint8_t packets=0;
        while(available()){
            packets++;
            received_packets++;

            // on the receipt of 100 packets, print some information.
            if ((received_packets % 100 == 0)){
                print((char*)"Total packets: ",2); println(received_packets,2);
                print((char*)"Packetloss count: ",2); println(packetloss,2);
                //print("Per second: ",2); Serial.println(received_packets / ((millis() - start_time)/1000));
            }

            uint8_t len = read(&buffer); // read the packet into the buffer.
            for (int i=0;i<len;i++){
            print((char)buffer[i],1);
               }

            // check the entire message if it consists of the same 4 byte blocks.
            uint16_t* this_counter = (uint16_t*) &(buffer[0]);
            for (int i=1; i < (64/4); i++){
                if (*(this_counter) != *((uint16_t*) &(buffer[i*4]))){
                    println((char*)"Message not correct!",2);
                    packetloss++;
                    continue;
                }
            }

            if ((counter+1) != *this_counter){
                println((char*)"Packetloss detected!",2);
                packetloss++;
            }

            counter = *this_counter;
        }

        // we can add some delay here, to show that the internal buffering works.
        // delayMicroseconds(4000);
        // try uncommenting this delay, the if statement below will be true from time to time.
        if (packets > 1){
            print((char*)"Packets this loop: ",2); println(packets,2);
        }
        

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
