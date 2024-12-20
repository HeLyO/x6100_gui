/*
 *  SPDX-License-Identifier: LGPL-2.1-or-later
 *
 *  Xiegu X6100 LVGL GUI
 *
 *  Copyright (c) 2022-2023 Belousov Oleg aka R1CBU
 */
#include "util.h"

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <sys/time.h>
#include <time.h>
#include <string.h>
#include <string.h>


/**
 * Return time in ms from unix epoch
 */
uint64_t get_time() {
    struct timespec now;

    clock_gettime(CLOCK_MONOTONIC, &now);

    uint64_t usec = (uint64_t) now.tv_sec * 1000L + now.tv_nsec / 1000000L;

    return usec;
}

void get_time_str(char *str, size_t str_size) {
    time_t      now = time(NULL);
    struct tm   *t = localtime(&now);

    snprintf(str, str_size, "%04i-%02i-%02i %02i-%02i-%02i", t->tm_year + 1900, t->tm_mon + 1, t->tm_mday, t->tm_hour, t->tm_min, t->tm_sec);
}

void split_freq(uint64_t freq, uint16_t *mhz, uint16_t *khz, uint16_t *hz) {
    *mhz = freq / 1000000;
    *khz = (freq / 1000) % 1000;
    *hz = freq % 1000;
}

int32_t align_int(int32_t x, uint16_t step) {
    if (step == 0) {
        return x;
    }

    return x - (x % step);
}

uint64_t align_long(uint64_t x, uint16_t step) {
    if (step == 0) {
        return x;
    }

    return x - (x % step);
}

int32_t limit(int32_t x, int32_t min, int32_t max) {
    if (x < min) {
        return min;
    } else if (x > max) {
        return max;
    }

    return x;
}

float sqr(float x) {
    return x * x;
}

void lpf(float *x, float current, float beta, float initial) {
    if (*x == initial){
        *x = current;
    } else {
        *x = *x * beta + current * (1.0f - beta);
    }

}

void lpf_block(float *x, float *current, float beta, unsigned int count) {
    liquid_vectorf_mulscalar(current, count, (1.0f - beta), current);
    liquid_vectorf_mulscalar(x, count, beta, x);
    liquid_vectorf_add(x, current, count, x);
}

void to_bcd(uint8_t bcd_data[], uint64_t data, uint8_t len) {
    int16_t i;

    for (i = 0; i < len / 2; i++) {
        uint8_t a = data % 10;

        data /= 10;
        a |= (data % 10) << 4;
        data /= 10;
        bcd_data[i] = a;
    }

    if (len & 1) {
        bcd_data[i] &= 0x0f;
        bcd_data[i] |= data % 10;
    }
}

void decimalToBCD(uint8_t bcd_data[], uint16_t data, uint8_t len) {
    uint8_t i;

    for (i = 0; i < len / 2; i++) {
        uint8_t a = (data / 10) % 10;
        a <<= 4;
        a |= data % 10;
        bcd_data[len / 2 - 1 - i] = a;
        data /= 100;
    }

    if (len & 1) {
        bcd_data[0] &= 0xF0;
        bcd_data[0] |= data % 10;
    }
}

uint64_t from_bcd(const uint8_t bcd_data[], uint8_t len) {
    int16_t     i;
    uint64_t    data = 0;

    if (len & 1) {
        data = bcd_data[len / 2] & 0x0F;
    }

    for (i = (len / 2) - 1; i >= 0; i--) {
        data *= 10;
        data += bcd_data[i] >> 4;
        data *= 10;
        data += bcd_data[i] & 0x0F;
    }

    return data;
}

uint64_t bcdToDecimal(const uint8_t bcd_data[], uint8_t len) {
    int16_t     i;
    uint64_t    data = 0;

    if (len & 1) {
        data = bcd_data[len / 2] & 0x0F;
    }

    for (i = 0; i <= (len / 2) - 1; i++) {
        data *= 10;
        data += bcd_data[i] >> 4;
        data *= 10;
        data += bcd_data[i] & 0x0F;
    }

    return data;
}

uint64_t ceil_uint64(uint64_t numerator, uint64_t denominator) {
    return (numerator + denominator - 1) / denominator;
}

uint64_t round_up_to_next_50(uint64_t number) {
    return ((number + 49) / 50) * 50;
}

int loop_modes(int16_t dir, int mode, uint64_t modes, int max_val) {
    while (1) {
        if (dir > 0) {
            if (mode == max_val) {
                mode = 0;
            } else {
                mode++;
            }
        } else {
            if (dir < 0)
            {
                if (mode == 0) {
                    mode = max_val;
                } else {
                    mode--;
                }
            }
        }
        if (modes & (1LL << mode)) {
            break;
        }
        if (dir == 0)
        {
            if (mode == max_val) {
                mode = 0;
            } else {
                mode++;
            }
        }
    }
    return mode;
}

int sign(int x) {
    return (x > 0) - (x < 0);
}

// Window rms

struct wrms_s {
    windowf window;
    size_t size;
    size_t delay;
    int16_t remain;
};

wrms_t wrms_create(size_t n, size_t delay) {
    wrms_t wr = (wrms_t) malloc(sizeof(struct wrms_s));
    // window size
    wr->size = n;
    // step size
    wr->delay = delay;
    wr->remain = wr->delay;
    wr->window = windowf_create(n);
    return wr;
}

void wrms_destroy(wrms_t wr) {
    windowf_destroy(wr->window);
    free(wr);
}

size_t wrms_size(wrms_t wr) {
    return wr->size;
}

size_t wrms_delay(wrms_t wr) {
    return wr->delay;
}

void wrms_pushcf(wrms_t wr, liquid_float_complex x) {
    if (wr->remain == 0) {
        wr->remain = wr->delay;
    }
    wr->remain--;
    float x_db = 10.0f * log10f(sqrt(crealf(x * conjf(x))));
    if (x_db < -121.0f) {
        x_db = -121.0f;
    }
    windowf_push(wr->window, x_db);
}

bool wrms_ready(wrms_t wr) {
    return wr->remain == 0;
}

float wrms_get_val(wrms_t wr) {
    float * r;
    windowf_read(wr->window, &r);
    float rms = 0.0;
    for (uint8_t i = 0; i < wr->size; i++) {
        rms += r[i];
    }
    rms = rms / wr->size;
    return rms;
}

size_t argmax(float * x, size_t n) {
    float max = -INFINITY;
    size_t pos;
    for (size_t i = 0; i < n; i++)
    {
        if (x[i] > max) {
            max = x[i];
            pos = i;
        }
    }
    return pos;
}


char * util_canonize_callsign(const char * callsign, bool strip_slashes) {
    if (!callsign) {
        return NULL;
    }

    char *result = NULL;

    if (strip_slashes) {
        char *s = strdup(callsign);
        char *token = strtok(s, "/");
        while(token) {
            if ((
                ((token[0] >= '0') && (token[0] <= '9')) ||
                ((token[1] >= '0') && (token[1] <= '9')) ||
                ((token[2] >= '0') && (token[2] <= '9'))
            ) && strlen(token) >= 4) {
                result = strdup(token);
                break;
            }
            token = strtok(NULL, "/");
        }
        free(s);
    } else {
        // strip < and > from remote call
        size_t callsign_len = strlen(callsign);
        if ((callsign[0] == '<') && (callsign[callsign_len - 1] == '>')) {
            result = strdup(callsign + 1);
            result[callsign_len - 2] = 0;
        }

    }
    if (!result) {
        result = strdup(callsign);
    }
    return result;
}
