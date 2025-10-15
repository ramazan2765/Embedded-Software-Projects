#ifndef FILTER_H
#define FILTER_H

#include "main.h"

// --- Moving Average (Sıcaklık gibi yavaş sensörler için) ---
float moving_average(float new_value);

// --- Exponential Moving Average (MPU9250 gibi sensörler için) ---
float ema_filter(float new_value, float *last_value, float alpha);

#endif
