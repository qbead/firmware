#ifndef HELPERS_H
#define HELPERS_H

#include <Arduino.h>

// Color functions
static uint32_t color(uint8_t r, uint8_t g, uint8_t b) {
    return ((uint32_t)r << 16) | ((uint32_t)g << 8) | b;
}

static uint8_t redch(uint32_t rgb) {
    return rgb >> 16;
}

static uint8_t greench(uint32_t rgb) {
    return (0x00ff00 & rgb) >> 8;
}

static uint8_t bluech(uint32_t rgb) {
    return 0x0000ff & rgb;
}

uint32_t colorWheel(uint8_t wheelPos);

uint32_t colorWheel_deg(float wheelPos);

float sign(float x);

#endif // HELPERS_H
