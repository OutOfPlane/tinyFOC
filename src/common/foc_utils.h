#ifndef FOCUTILS_LIB_H
#define FOCUTILS_LIB_H

#include "fixed_point.h"



// sign function
#define _sign(a) ( ( (a) < 0 )  ?  -1   : ( (a) > 0 ) )
#ifndef _round
#define _round(x) ((x)>=0?(long)((x)+0.5f):(long)((x)-0.5f))
#endif
#define _constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#define _sqrt(a) (_sqrtApprox(a))
#define _isset(a) ( (a) != (NOT_SET) )
#define _UNUSED(v) (void) (v)
#define _powtwo(x) (1 << (x))

#define _swap(a, b) { auto temp = a; a = b; b = temp; }

// utility defines
#define _2_SQRT3 1.15470053838f
#define _SQRT3 1.73205080757f
#define _1_SQRT3 0.57735026919f
#define _SQRT3_2 0.86602540378f
#define _SQRT2 1.41421356237f
#define _120_D2R 2.09439510239f
#define _PI 3.14159265359f
#define _PI_2 1.57079632679f
#define _PI_3 1.0471975512f
#define _2PI 6.28318530718f
#define _3PI_2 4.71238898038f
#define _PI_6 0.52359877559f
#define _RPM_TO_RADS 0.10471975512f

#define NOT_SET INT32_MAX
#define _HIGH_IMPEDANCE 0
#define _HIGH_Z _HIGH_IMPEDANCE
#define _ACTIVE 1
#define _NC ((int) NOT_SET)

#define MIN_ANGLE_DETECT_MOVEMENT (_2PI/101.0f)

// dq variables
struct DQ_s
{
    FIXP d;
    FIXP q;
};

// dq voltage structs
typedef struct DQ_s DQVoltage_s;
// dq current structs
typedef struct DQ_s DQCurrent_s;

// alpha-beta variables    
struct AB_s
{
    FIXP alpha;
    FIXP beta;
};
typedef struct AB_s ABVoltage_s;  // NOT USED
typedef struct AB_s ABCurrent_s;

// phase structs
struct Phase_s
{
    FIXP a;
    FIXP b;
    FIXP c;
};
typedef struct Phase_s PhaseVoltage_s; // NOT USED
typedef struct Phase_s PhaseCurrent_s;

#define min(X, Y) (X > Y ? Y : X)
#define max(X, Y) (X > Y ? X : Y)
#define _abs(X) (X > 0 ? X : -X)

typedef struct{
    void (*write)(char val);
    void (*newline)(void);
    void (*print)(char* msg);
    void (*print_f)(FIXP val, int digits);
    int (*read)(char* val, int len);
}Print;

#endif