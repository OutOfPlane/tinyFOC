
#include "TinyFOCDebug.h"
#include "stdint.h"
#ifndef TinyFOC_DISABLE_DEBUG


Print* _debugPrint = NULL;


void TinyFOCDebug_enable(Print* debugPrint) {
    _debugPrint = debugPrint;
}


void TinyFOCDebug_println_s(const char* val) {
    if (_debugPrint != NULL) {
        _debugPrint->print(val);
        _debugPrint->newline();
    }
}

void TinyFOCDebug_println_f(const char *msg, float val)
{
}

void TinyFOCDebug_println_i(const char *msg, int val)
{
}

void TinyFOCDebug_println_c(const char *msg, char val)
{
}

void TinyFOCDebug_println()
{
}

void TinyFOCDebug_print_s(const char *msg)
{
}

void TinyFOCDebug_print_i(int val)
{
}

void TinyFOCDebug_print_f(float val)
{
}

#endif