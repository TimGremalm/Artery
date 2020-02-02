#ifndef __HSL_RGB__
#define __HSL_RGB__

#include "ws2812_i2s/ws2812_i2s.h"

float hueToRgb(float p, float q, float t);
ws2812_pixel_t hslToRgb(float h, float s, float l);

#endif  // __HSL_RGB__

