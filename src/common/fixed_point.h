#ifndef FIXED_POINT_H
#define FIXED_POINT_H

#include <stdint.h>

// Q15.16 Fixed-Point Arithmetic Library
// Format: 15 bits integer (including sign) + 16 bits fractional
// Range: -32768 to +32767.9999847412109375
// Precision: 1/65536 ≈ 0.0000152587890625

// Basic macros
#define FIX_SHIFT 16
#define FIX_ONE (1 << FIX_SHIFT)  // 65536
#define FIX_HALF (1 << (FIX_SHIFT - 1))  // 32768

// Conversion macros
#define FIX_FROM_FLOAT(f) ((int32_t)((f) * 65536.0f))
#define FIX_TO_FLOAT(f) ((float)(f) / 65536.0f)
#define FIX_FROM_INT(i) ((int32_t)(i) << FIX_SHIFT)

// Arithmetic operations
#define FIX_MUL(a, b) ((int64_t)(a) * (b) >> FIX_SHIFT)
#define FIX_DIV(a, b) ((((int64_t)(a)) << FIX_SHIFT) / (b))
#define FIX_ADD(a, b) ((a) + (b))
#define FIX_SUB(a, b) ((a) - (b))

// Comparison operations
#define FIX_LT(a, b) ((a) < (b))
#define FIX_LE(a, b) ((a) <= (b))
#define FIX_GT(a, b) ((a) > (b))
#define FIX_GE(a, b) ((a) >= (b))
#define FIX_EQ(a, b) ((a) == (b))
#define FIX_NE(a, b) ((a) != (b))

// Utility macros
#define FIX_CONSTRAIN(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#define FIX_MIN(a, b) ((a) < (b) ? (a) : (b))
#define FIX_MAX(a, b) ((a) > (b) ? (a) : (b))

// Math operations
#define FIX_NEG(a) (-(a))
#define FIX_ABS(a) ((a) < 0 ? -(a) : (a))
#define FIX_MIN(a, b) ((a) < (b) ? (a) : (b))
#define FIX_MAX(a, b) ((a) > (b) ? (a) : (b))

// Common constants
#define FIX_ZERO 0
#define FIX_PI FIX_FROM_FLOAT(3.14159265359f)
#define FIX_PI_2 FIX_FROM_FLOAT(1.57079632679f)
#define FIX_PI_3 FIX_FROM_FLOAT(1.0471975512f)
#define FIX_2PI FIX_FROM_FLOAT(6.28318530718f)
#define FIX_PI_6 FIX_FROM_FLOAT(0.52359877559f)
#define FIX_SQRT3 FIX_FROM_FLOAT(1.73205080757f)
#define FIX_SQRT3_2 FIX_FROM_FLOAT(0.86602540378f)
#define FIX_SQRT2 FIX_FROM_FLOAT(1.41421356237f)
#define FIX_120_D2R FIX_FROM_FLOAT(2.09439510239f)
#define FIX_RPM_TO_RADS FIX_FROM_FLOAT(0.10471975512f)
#define FIX_2_SQRT3 FIX_FROM_FLOAT(1.15470053838f)
#define FIX_3PI_2 FIX_FROM_FLOAT(4.71238898038f)

// Utility functions (declare here, implement in .c file if needed)
int32_t fix_sin(int32_t angle);
int32_t fix_cos(int32_t angle);
void fix_sincos(int32_t angle, int32_t *sin_val, int32_t *cos_val);
int32_t fix_sqrt(int32_t x);
int32_t fix_normalize_angle(int32_t angle);

#endif // FIXED_POINT_H