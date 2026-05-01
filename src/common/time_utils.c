#include "time_utils.h"

// function buffering delay() 
// arduino uno function doesn't work well with interrupts
void _delay(uint32_t ms){
  // if arduino uno and other atmega328p chips
  // use while instad of delay, 
  // due to wrong measurement based on changed timer0
  uint32_t t = _micros();
  while( _micros() - t < ms*1000 ); 
}

void _delay_us(uint32_t us)
{
  uint32_t t = _micros();
  while( _micros() - t < us ); 
}

