
#ifndef __SIMPLEFOCDEBUG_H__
#define __SIMPLEFOCDEBUG_H__
#include "common/foc_utils.h"


/**
 * SimpleFOCDebug class
 * 
 * This class is used to print debug messages to a chosen output.
 * Currently, Print instances are supported as targets, e.g. serial port.
 * 
 * Activate debug output globally by calling enable(), optionally passing
 * in a Print instance. If none is provided "Serial" is used by default.
 * 
 * To produce debug output, use the macro SIMPLEFOC_DEBUG:
 *   SIMPLEFOC_DEBUG("Debug message!");
 *   SIMPLEFOC_DEBUG("a float value:", 123.456f);
 *   SIMPLEFOC_DEBUG("an integer value: ", 123);
 * 
 * Keep debugging output short and simple. Some of our MCUs have limited
 * RAM and limited serial output capabilities.
 * 
 * By default, the SIMPLEFOC_DEBUG macro uses the flash string helper to
 * help preserve memory on Arduino boards.
 * 
 * You can also disable debug output completely. In this case all debug output 
 * and the SimpleFOCDebug class is removed from the compiled code.
 * Add -DSIMPLEFOC_DISABLE_DEBUG to your compiler flags to disable debug in
 * this way.
 * 
 **/

// #define SIMPLEFOC_DISABLE_DEBUG

#ifndef SIMPLEFOC_DISABLE_DEBUG 

void SimpleFOCDebug_enable(Print* debugPrint);
void SimpleFOCDebug_println_s(const char* msg);
void SimpleFOCDebug_println_f(const char* msg, float val);
void SimpleFOCDebug_println_i(const char* msg, int val);
void SimpleFOCDebug_println_c(const char* msg, char val);
void SimpleFOCDebug_println();
void SimpleFOCDebug_println_i(int val);
void SimpleFOCDebug_println_f(float val);
void SimpleFOCDebug_print_s(const char* msg);
void SimpleFOCDebug_print_i(int val);
void SimpleFOCDebug_print_f(float val);


#define SIMPLEFOC_DEBUG(msg) \
    SimpleFOCDebug_println_s(msg)

#define SIMPLEFOC_DEBUG_c(msg, val) \
    SimpleFOCDebug_println_c(msg, val)

#define SIMPLEFOC_DEBUG_f(msg, val) \
    SimpleFOCDebug_println_f(msg, val)

#define SIMPLEFOC_DEBUG_i(msg, str) \
    SimpleFOCDebug_println_i(msg, val)
#else  //ifndef SIMPLEFOC_DISABLE_DEBUG
 
#define SIMPLEFOC_DEBUG(msg)

#define SIMPLEFOC_DEBUG_c(msg, val)

#define SIMPLEFOC_DEBUG_f(msg, val)

#define SIMPLEFOC_DEBUG_i(msg, str)


#endif //ifndef SIMPLEFOC_DISABLE_DEBUG
#endif

