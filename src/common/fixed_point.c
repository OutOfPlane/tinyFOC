#include "fixed_point.h"

const FIXP sine_array[65] = {
  FIX_FROM_FLOAT(0.0),
  FIX_FROM_FLOAT(0.024541228522912288),
  FIX_FROM_FLOAT(0.049067674327418015),
  FIX_FROM_FLOAT(0.07356456359966743),
  FIX_FROM_FLOAT(0.0980171403295606),
  FIX_FROM_FLOAT(0.1224106751992162),
  FIX_FROM_FLOAT(0.14673047445536175),
  FIX_FROM_FLOAT(0.17096188876030122),
  FIX_FROM_FLOAT(0.19509032201612825),
  FIX_FROM_FLOAT(0.2191012401568698),
  FIX_FROM_FLOAT(0.24298017990326387),
  FIX_FROM_FLOAT(0.26671275747489837),
  FIX_FROM_FLOAT(0.29028467725446233),
  FIX_FROM_FLOAT(0.3136817403988915),
  FIX_FROM_FLOAT(0.33688985339222005),
  FIX_FROM_FLOAT(0.3598950365349881),
  FIX_FROM_FLOAT(0.3826834323650898),
  FIX_FROM_FLOAT(0.40524131400498986),
  FIX_FROM_FLOAT(0.4275550934302821),
  FIX_FROM_FLOAT(0.44961132965460654),
  FIX_FROM_FLOAT(0.47139673682599764),
  FIX_FROM_FLOAT(0.49289819222978404),
  FIX_FROM_FLOAT(0.5141027441932217),
  FIX_FROM_FLOAT(0.5349976198870972),
  FIX_FROM_FLOAT(0.5555702330196022),
  FIX_FROM_FLOAT(0.5758081914178453),
  FIX_FROM_FLOAT(0.5956993044924334),
  FIX_FROM_FLOAT(0.6152315905806268),
  FIX_FROM_FLOAT(0.6343932841636455),
  FIX_FROM_FLOAT(0.6531728429537768),
  FIX_FROM_FLOAT(0.6715589548470183),
  FIX_FROM_FLOAT(0.6895405447370668),
  FIX_FROM_FLOAT(0.7071067811865476),
  FIX_FROM_FLOAT(0.7242470829514669),
  FIX_FROM_FLOAT(0.7409511253549591),
  FIX_FROM_FLOAT(0.7572088465064846),
  FIX_FROM_FLOAT(0.7730104533627369),
  FIX_FROM_FLOAT(0.7883464276266062),
  FIX_FROM_FLOAT(0.8032075314806448),
  FIX_FROM_FLOAT(0.8175848131515837),
  FIX_FROM_FLOAT(0.8314696123025452),
  FIX_FROM_FLOAT(0.844853565249707),
  FIX_FROM_FLOAT(0.8577286100002721),
  FIX_FROM_FLOAT(0.8700869911087113),
  FIX_FROM_FLOAT(0.8819212643483549),
  FIX_FROM_FLOAT(0.8932243011955153),
  FIX_FROM_FLOAT(0.9039892931234433),
  FIX_FROM_FLOAT(0.9142097557035307),
  FIX_FROM_FLOAT(0.9238795325112867),
  FIX_FROM_FLOAT(0.9329927988347388),
  FIX_FROM_FLOAT(0.9415440651830208),
  FIX_FROM_FLOAT(0.9495281805930367),
  FIX_FROM_FLOAT(0.9569403357322089),
  FIX_FROM_FLOAT(0.9637760657954398),
  FIX_FROM_FLOAT(0.970031253194544),
  FIX_FROM_FLOAT(0.9757021300385286),
  FIX_FROM_FLOAT(0.9807852804032304),
  FIX_FROM_FLOAT(0.9852776423889412),
  FIX_FROM_FLOAT(0.989176509964781),
  FIX_FROM_FLOAT(0.99247953459871),
  FIX_FROM_FLOAT(0.9951847266721968),
  FIX_FROM_FLOAT(0.9972904566786902),
  FIX_FROM_FLOAT(0.9987954562051724),
  FIX_FROM_FLOAT(0.9996988186962042),
  FIX_FROM_FLOAT(1.0)
};

// Fixed-point sine approximation using lookup table
// Input: angle in Q15.16 radians
// Output: sine value in Q15.16 format
int32_t fix_sin(int32_t angle) {
    // Normalize angle to [0, 2π)
    angle = fix_normalize_angle(angle);

    // Scale angle to lookup table index
    // Original: i = (unsigned int)(a * (64*4*256.0f/_2PI))
    // 64*4*256/_2PI ≈ 65536/_2PI ≈ 10430.378
    FIXP scaled = FIX_DIV(angle, FIX_2PI);
    unsigned int i = scaled; // convert back to int
    int frac = i & 0xff;
    i = (i >> 8) & 0xff;

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