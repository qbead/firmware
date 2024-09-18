#include "Helpers.h"

uint32_t colorWheel(uint8_t wheelPos) {
    wheelPos = 255 - wheelPos;
    if (wheelPos < 85) {
        return color(255 - wheelPos * 3, 0, wheelPos * 3);
    }
    if (wheelPos < 170) {
        wheelPos -= 85;
        return color(0, wheelPos * 3, 255 - wheelPos * 3);
    }
    wheelPos -= 170;
    return color(wheelPos * 3, 255 - wheelPos * 3, 0);
}

uint32_t colorWheel_deg(float wheelPos) {
    return colorWheel(wheelPos * 255 / 360);
}

float sign(float x) {
    return (x > 0) ? +1 : -1;
}
