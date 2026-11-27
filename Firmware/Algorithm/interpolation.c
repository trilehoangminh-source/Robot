#include "interpolation.h"
#include <math.h>
interpolation_status_t interpolation_linear(uint32_t start, uint32_t end,
                                             float a, float b, uint32_t now, float *out)
{
    float t, value;
    if (!out || end <= start || !isfinite(a) || !isfinite(b)) return INTERPOLATION_INVALID;
    if (now <= start) value = a;
    else if (now >= end) value = b;
    else {
        t = (float)(now - start) / (float)(end - start);
        value = a * (1.0f - t) + b * t;
    }
    if (!isfinite(value)) return INTERPOLATION_INVALID;
    *out = value;
    return INTERPOLATION_OK;
}
