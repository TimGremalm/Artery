#include "hsl_rgb.h"

/** Helper method that converts hue to rgb */
float hueToRgb(float p, float q, float t) {
	if (t < (float)0)
			t += (float)1;
	if (t > (float)1)
			t -= (float)1;
	if (t < (float)1/(float)6)
			return p + (q - p) * (float)6 * t;
	if (t < (float)1/(float)2)
			return q;
	if (t < (float)2/(float)3)
			return p + (q - p) * ((float)2.0/ (float)3.0 - t) * (float)6;
	return p;
}

/* Converts an HSL color value to RGB. Conversion formula
-- adapted from http://en.wikipedia.org/wiki/HSL_color_space.
-- Assumes h, s, and l are contained in the set [0, 1] and
-- returns r, g, and b as ws2812_pixel_t struct.
--
-- @param	 Number	h			 The hue
-- @param	 Number	s			 The saturation
-- @param	 Number	l			 The lightness
-- @return	Array					 The RGB representation*/
ws2812_pixel_t hslToRgb(float h, float s, float l) {
	float r, g, b;

	if (s == (float)0) {
			r = g = b = l; // achromatic
	} else {
			float q = l < 0.5 ? l * (1 + s) : l + s - l * s;
			float p = 2 * l - q;
			r = hueToRgb(p, q, h + (float)1/(float)3);
			g = hueToRgb(p, q, h);
			b = hueToRgb(p, q, h - (float)1/(float)3);
	}
	ws2812_pixel_t out;
	out.red = (int) (r * 255);
	out.green = (int) (g * 255);
	out.blue = (int) (b * 255);
	return out;
}

