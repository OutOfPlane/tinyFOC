
#ifndef __TinyFOCDEBUG_H__
#define __TinyFOCDEBUG_H__
#include "common/foc_utils.h"


/**
 * TinyFOCDebug class
 * 
 * This class is used to print debug messages to a chosen output.
 * Currently, Print instances are supported as targets, e.g. serial port.
 * 
 * Activate debug output globally by calling enable(), optionally passing
 * in a Print instance. If none is provided "Serial" is used by default.
 * 
 * To produce debug output, use the macro TinyFOC_DEBUG:
 *   TinyFOC_DEBUG("Debug message!");
 *   TinyFOC_DEBUG("a FIXP value:", FIX_FROM_FLOAT(123.456f));
 *   TinyFOC_DEBUG("an integer value: ", 123);
 * 
 * Keep debugging output short and simple. Some of our MCUs have limited
 * RAM and limited serial output capabilities.
 * 
 * By default, the TinyFOC_DEBUG macro uses the flash string helper to
 * help preserve memory on Arduino boards.
 * 
 * You can also disable debug output completely. In this case all debug output 
 * and the TinyFOCDebug class is removed from the compiled code.
 * Add -DTinyFOC_DISABLE_DEBUG to your compiler flags to disable debug in
 * this way.
 * 
 **/

// #define TinyFOC_DISABLE_DEBUG

#ifndef TinyFOC_DISABLE_DEBUG 

void TinyFOCDebug_enable(Print* debugPrint);
void TinyFOCDebug_println_s(const char* msg);
void TinyFOCDebug_println_f(const char* msg, FIXP val);
void TinyFOCDebug_println_i(const char* msg, int val);
void TinyFOCDebug_println_c(const char* msg, char val);
void TinyFOCDebug_println(void);
void TinyFOCDebug_print_s(const char* msg);
void TinyFOCDebug_print_i(int val);
void TinyFOCDebug_print_f(FIXP val);


#define TinyFOC_DEBUG(msg) \
    TinyFOCDebug_println_s(msg)

#define TinyFOC_DEBUG_c(msg, val) \
    TinyFOCDebug_println_c(msg, val)

#define TinyFOC_DEBUG_f(msg, val) \
    TinyFOCDebug_println_f(msg, val)

#define TinyFOC_DEBUG_i(msg, val) \
    TinyFOCDebug_println_i(msg, val)
#else  //ifndef TinyFOC_DISABLE_DEBUG
 
#define TinyFOC_DEBUG(msg)

#define TinyFOC_DEBUG_c(msg, val)

#define TinyFOC_DEBUG_f(msg, val)

#define TinyFOC_DEBUG_i(msg, str)


#endif //ifndef TinyFOC_DISABLE_DEBUG
#endif

