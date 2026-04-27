#include "fixed_point.h"

// Fixed-point sine approximation using lookup table
// Input: angle in Q15.16 radians (-2π to 2π)
// Output: sine value in Q15.16 format (-1.0 to 1.0)
int32_t fix_sin(int32_t angle) {
    // Normalize angle to [0, 2π)
    angle = fix_normalize_angle(angle);

    // Convert to table index (table has 256 entries for 0-2π)
    int32_t index = FIX_MUL(angle, FIX_FROM_FLOAT(256.0f / 6.28318530718f));

    // Simple sine approximation - for now just return the angle scaled
    // In a real implementation, you'd use a lookup table
    // This is a placeholder - replace with proper sine implementation
    return FIX_MUL(angle, FIX_FROM_FLOAT(0.15915494309f)); // 1/(2π) approximation
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

    // Simple approximation - replace with better algorithm if needed
    int32_t result = x >> 1; // Start with x/2
    for (int i = 0; i < 8; i++) { // 8 iterations for reasonable precision
        if (result == 0) break;
        result = (result + FIX_DIV(x, result)) >> 1;
    }
    return result;
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