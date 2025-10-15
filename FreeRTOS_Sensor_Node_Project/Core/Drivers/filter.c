#include "filter.h"

// ---------- Moving Average ----------
#define MA_BUFFER_SIZE 10   // Son 10 ölçümün ortalaması alınır

float moving_average(float new_value) {
    static float buffer[MA_BUFFER_SIZE] = {0};
    static int index = 0;
    static int count = 0;
    float sum = 0;

    buffer[index] = new_value;
    index = (index + 1) % MA_BUFFER_SIZE;
    if (count < MA_BUFFER_SIZE) count++;

    for (int i = 0; i < count; i++) {
        sum += buffer[i];
    }

    return sum / count;
}

// ---------- Exponential Moving Average ----------
float ema_filter(float new_value, float *last_value, float alpha) {
    *last_value = alpha * new_value + (1.0f - alpha) * (*last_value);
    return *last_value;
}
