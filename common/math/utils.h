#ifndef MATH_UTILS_H
#define MATH_UTILS_H

#include <types.h>

static inline f64 clamp(f64 value, f64 min, f64 max) {
    return value < min ? min : (value > max ? max : value);
}

#endif