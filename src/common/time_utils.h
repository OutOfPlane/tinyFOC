#ifndef TIME_UTILS_H
#define TIME_UTILS_H

#include "foc_utils.h"

/** 
 * Function implementing delay() function in milliseconds 
 * - blocking function
 * - hardware specific

 * @param ms number of milliseconds to wait
 */
void _delay(uint32_t ms);

void _delay_us(uint32_t us);

/** 
 * Function implementing timestamp getting function in microseconds
 * hardware specific
 */
uint32_t _micros(void);


#endif