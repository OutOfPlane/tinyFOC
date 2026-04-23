
#include "TinyFOCDebug.h"
#include "stdint.h"
#ifndef TinyFOC_DISABLE_DEBUG


Print* _debugPrint = NULL;


void TinyFOCDebug_enable(Print* debugPrint) {
    _debugPrint = debugPrint;
}


void TinyFOCDebug_println(int val) {
    if (_debugPrint != NULL) {
        _debugPrint->println(val);
    }
}

void TinyFOCDebug::println(float val) {
    if (_debugPrint != NULL) {
        _debugPrint->println(val);
    }
}



void TinyFOCDebug::println(const char* str) {
    if (_debugPrint != NULL) {
        _debugPrint->println(str);
    }
}

void TinyFOCDebug::println(const __FlashStringHelper* str) {
    if (_debugPrint != NULL) {
        _debugPrint->println(str);
    }
}


void TinyFOCDebug::println(const char* str, float val) {
    if (_debugPrint != NULL) {
        _debugPrint->print(str);
        _debugPrint->println(val);
    }
}

void TinyFOCDebug::println(const __FlashStringHelper* str, float val) {
    if (_debugPrint != NULL) {
        _debugPrint->print(str);
        _debugPrint->println(val);
    }
}

void TinyFOCDebug::println(const char* str, int val) {
    if (_debugPrint != NULL) {
        _debugPrint->print(str);
        _debugPrint->println(val);
    }
}
void TinyFOCDebug::println(const char* str, char val) {
    if (_debugPrint != NULL) {
        _debugPrint->print(str);
        _debugPrint->println(val);
    }
}

void TinyFOCDebug::println(const __FlashStringHelper* str, int val) {
    if (_debugPrint != NULL) {
        _debugPrint->print(str);
        _debugPrint->println(val);
    }
}


void TinyFOCDebug::print(const char* str) {
    if (_debugPrint != NULL) {
        _debugPrint->print(str);
    }
}


void TinyFOCDebug::print(const __FlashStringHelper* str) {
    if (_debugPrint != NULL) {
        _debugPrint->print(str);
    }
}

void TinyFOCDebug::print(const StringSumHelper str) {
    if (_debugPrint != NULL) {
        _debugPrint->print(str.c_str());
    }
}


void TinyFOCDebug::println(const StringSumHelper str) {
    if (_debugPrint != NULL) {
        _debugPrint->println(str.c_str());
    }
}



void TinyFOCDebug::print(int val) {
    if (_debugPrint != NULL) {
        _debugPrint->print(val);
    }
}


void TinyFOCDebug::print(float val) {
    if (_debugPrint != NULL) {
        _debugPrint->print(val);
    }
}


void TinyFOCDebug::println() {
    if (_debugPrint != NULL) {
        _debugPrint->println();
    }
}

#endif