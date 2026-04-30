
#include "TinyFOCDebug.h"
#ifndef TinyFOC_DISABLE_DEBUG


Print* _debugPrint = NULL;


void TinyFOCDebug_enable(Print* debugPrint) {
    _debugPrint = debugPrint;
}


void TinyFOCDebug_println_s(char* val) {
    if (_debugPrint != NULL) {
        _debugPrint->print(val);
        _debugPrint->newline();
    }
}

void TinyFOCDebug_println_f(char *msg, FIXP val)
{
    if (_debugPrint != NULL) {
        _debugPrint->print(msg);
        _debugPrint->print_f(val, 4); // default to 4 decimal places
        _debugPrint->newline();
    }
}

void TinyFOCDebug_println_i(char *msg, int val)
{
    if (_debugPrint != NULL) {
        _debugPrint->print(msg);
        _debugPrint->print_f(FIX_FROM_INT(val), 4); // default to 4 decimal places
        _debugPrint->newline();
    }
}

void TinyFOCDebug_println_c(char *msg, char val)
{
}

void TinyFOCDebug_println(void)
{
}

void TinyFOCDebug_print_s(char *msg)
{
}

void TinyFOCDebug_print_i(int val)
{
}

void TinyFOCDebug_print_f(FIXP val)
{
}

#endif