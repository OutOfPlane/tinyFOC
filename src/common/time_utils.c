#include "time_utils.h"

// function buffering delay() 
// arduino uno function doesn't work well with interrupts
void _delay(unsigned long ms){
  // if arduino uno and other atmega328p chips
  // use while instad of delay, 
  // due to wrong measurement based on changed timer0
  unsigned long t = _micros();
  while( _micros() - t < ms*1000 ); 
}

void _delay_us(unsigned long us)
{
  unsigned long t = _micros();
  while( _micros() - t < us ); 
}

// function buffering _micros() 
// arduino function doesn't work well with interrupts
static unsigned long default_micros(){
  return 0;
}


unsigned long (*_micros)(void) = default_micros;
