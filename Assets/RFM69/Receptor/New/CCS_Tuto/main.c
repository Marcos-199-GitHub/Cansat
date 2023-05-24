#include <main.h>

#use delay (clock=16000000)

void main()
{
  while(TRUE)
  {
    output_high(PIN_B7); //LED ON
    delay_ms(1000);      //1 Second Delay
    output_low(PIN_B7);  //LED OFF
    delay_ms(1000);      //1 Second Delay
  }
}
