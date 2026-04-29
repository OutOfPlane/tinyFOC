#include "fixed_point.h"

// Fixed-point sine approximation using lookup table
// Input: angle in Q15.16 radians
// Output: sine value in Q15.16 format
int32_t fix_sin(int32_t angle) {
    // Normalize angle to [0, 2π)
    angle = fix_normalize_angle(angle);

    // Scale angle to lookup table index
    // Original: i = (unsigned int)(a * (64*4*256.0f/_2PI))
    // 64*4*256/_2PI ≈ 65536/_2PI ≈ 10430.378
    int32_t scaled = FIX_MUL(angle, FIX_FROM_FLOAT(10430.378f));
    unsigned int i = (unsigned int)(scaled >> 16); // convert back to int
    int frac = i & 0xff;
    i = (i >> 8) & 0xff;

    // Lookup table (values from original implementation, scaled to Q15.16)
    static int32_t sine_array[66] = {
        0, 804<<16, 1608<<16, 2411<<16, 3212<<16, 4011<<16, 4808<<16, 5602<<16, 6393<<16, 7180<<16,
        7962<<16, 8740<<16, 9512<<16, 10279<<16, 11039<<16, 11793<<16, 12540<<16, 13279<<16, 14010<<16, 14733<<16,
        15447<<16, 16151<<16, 16846<<16, 17531<<16, 18205<<16, 18868<<16, 19520<<16, 20160<<16, 20788<<16, 21403<<16,
        22006<<16, 22595<<16, 23170<<16, 23732<<16, 24279<<16, 24812<<16, 25330<<16, 25833<<16, 26320<<16, 26791<<16,
        27246<<16, 27684<<16, 28106<<16, 28511<<16, 28899<<16, 29269<<16, 29622<<16, 29957<<16, 30274<<16, 30572<<16,
        30853<<16, 31114<<16, 31357<<16, 31581<<16, 31786<<16, 31972<<16, 32138<<16, 32286<<16, 32413<<16, 32522<<16,
        32610<<16, 32679<<16, 32729<<16, 32758<<16, 32768<<16, 32768<<16
    };

    int32_t t1, t2;
    if (i < 64) {
        t1 = sine_array[i]; t2 = sine_array[i+1];
    } else if (i < 128) {
        t1 = sine_array[128 - i]; t2 = sine_array[127 - i];
    } else if (i < 192) {
        t1 = -sine_array[-128 + i]; t2 = -sine_array[-127 + i];
    } else {
        t1 = -sine_array[256 - i]; t2 = -sine_array[255 - i];
    }

    // Interpolate: t1 + ((t2 - t1) * frac) >> 8
    int32_t diff = t2 - t1;
    int32_t interp = (diff * frac) >> 8;
    return t1 + interp;
}

// Fixed-point cosine
int32_t fix_cos(int32_t angle) {
    // cos(x) = sin(x + π/2)
    return fix_sin(angle + FIX_PI_2);
}

// Combined sin/cos for efficiency
void fix_sincos(int32_t angle, int32_t *sin_val, int32_t *cos_val) {
    *sin_val = fix_sin(angle);
    *cos_val = fix_cos(angle);
}

// Fixed-point square root approximation
int32_t fix_sqrt(int32_t x) {
    if (x <= 0) return 0;

    // Newton-Raphson approximation
    int32_t result = x >> 1; // Start with x/2
    for (int i = 0; i < 8; i++) { // 8 iterations for reasonable precision
        if (result == 0) break;
        result = (result + FIX_DIV(x, result)) >> 1;
    }
    return result;
}

// Fixed-point atan2 approximation
int32_t fix_atan2(int32_t y, int32_t x) {
    // Simplified atan2 - for full implementation, would need more complex algorithm
    // This is a basic approximation
    if (x == 0) {
        return (y > 0) ? FIX_PI_2 : -FIX_PI_2;
    }
    int32_t abs_y = FIX_ABS(y);
    int32_t abs_x = FIX_ABS(x);
    int32_t a = FIX_MIN(abs_x, abs_y);
    int32_t b = FIX_MAX(abs_x, abs_y);
    int32_t ratio = FIX_DIV(a, b);
    int32_t angle = FIX_MUL(ratio, FIX_FROM_FLOAT(0.785398f)); // π/4 approximation

    if (abs_y > abs_x) angle = FIX_PI_2 - angle;
    if (x < 0) angle = FIX_PI - angle;
    if (y < 0) angle = -angle;

    return angle;
}

// Normalize angle to [0, 2π)
int32_t fix_normalize_angle(int32_t angle) {
    // Handle negative angles
    while (angle < 0) {
        angle += FIX_2PI;
    }
    // Wrap around 2π
    while (angle >= FIX_2PI) {
        angle -= FIX_2PI;
    }
    return angle;
}

// Electrical angle calculation
int32_t fix_electrical_angle(int32_t shaft_angle, int pole_pairs) {
    return FIX_MUL(shaft_angle, FIX_FROM_INT(pole_pairs));
}