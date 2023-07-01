#include <18LF4550.h>
#device ADC=10

//IMPORTANTE: el fuse PLL5 es un predivisor del reloj, que tiene valores 1,2,3,4,5,6,10,12
//Dependiendo del cristal, se debe elegir el PLL para que al dividirlo
//de una frecuencia de 4MHz
#fuses HSPLL, NOWDT, NOPROTECT, NODEBUG, USBDIV, PLL5, CPUDIV1, VREGEN

#use delay(clock=48MHz,crystal=20MHz,USB_FULL)
#use FIXED_IO( E_outputs=PIN_E0 )
#use FIXED_IO( A_outputs=PIN_A4 )
#define RF_RESET   PIN_E0
#define INDICATOR_LED PIN_A4


//#define USB_CABLE_IS_ATTACHED()  input(PIN_B2)
#define USB_CONFIG_VID 0x2405
#define USB_CONFIG_PID 0x000B
#define USB_CONFIG_BUS_POWER 500

// if USB_CDC_ISR is defined, then this function will be called
// by the USB ISR when there incoming CDC (virtual com port) data.
// this is useful if you want to port old RS232 code that was use
// #int_rda to CDC.
#define USB_CDC_ISR() RDA_isr()

// in order for handle_incoming_usb() to be able to transmit the entire
// USB message in one pass, we need to increase the CDC buffer size from
// the normal size and use the USB_CDC_DELAYED_FLUSH option.
// failure to do this would cause some loss of data.
#define USB_CDC_DELAYED_FLUSH
#define USB_CDC_DATA_LOCAL_SIZE  128


static void RDA_isr(void);

#include <usb_cdc.h>

uint16_t globalMs;
uint8_t globalSec;
uint16_t globalMin;








