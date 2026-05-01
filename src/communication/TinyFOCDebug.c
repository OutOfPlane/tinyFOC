
#include "TinyFOCDebug.h"
#ifdef DEBUG_PRINT

void TinyFOCDebug_println_s(char *val)
{
    dbg_print(val);
    dbg_newline();
}

void TinyFOCDebug_println_f(char *msg, FIXP val)
{
    dbg_print(msg);
    dbg_print_f(val, 4); // default to 4 decimal places
    dbg_newline();
}

void TinyFOCDebug_println_i(char *msg, int val)
{
    dbg_print(msg);
    dbg_print_f(FIX_FROM_INT(val), 4); // default to 4 decimal places
    dbg_newline();
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