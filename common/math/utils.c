#include "utils.h"

static inline f64 clamp(f64 value, f64 min, f64 max) {
    return value < min ? min : (value > max ? max : value);
}