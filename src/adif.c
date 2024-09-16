/*
 *  SPDX-License-Identifier: LGPL-2.1-or-later
 *
 *  Xiegu X6100 LVGL GUI
 *
 *  Copyright (c) 2024 Georgy Dyuldin aka R2RFE
 */
#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif

#include "adif.h"

#include "util.h"

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <unistd.h>
#include <regex.h>

#define ARRAY_SIZE(arr) (sizeof((arr)) / sizeof((arr)[0]))
#define COPY_STR(dst, src, len) (copy_str(dst, src, len, sizeof(dst)))

struct adif_log_s {
    FILE *fd;
};

static void write_header(FILE *fd);

static void write_str(FILE *fd, const char * key, const char * val);
static void write_int(FILE *fd, const char * key, int val);

static void write_date_time(FILE *fd, time_t time);
static void write_freq_band(FILE *fd, float freq_mhz);
static void copy_str(char * dst, char * src, size_t val_len, size_t dst_len);


adif_log adif_log_init(const char * path) {
    adif_log log = (adif_log) malloc(sizeof(struct adif_log_s));
    bool new_file = false;
    if (access(path, F_OK) != 0) {
        new_file = true;
    }
    FILE *log_fd = fopen(path, "a");
    if (log_fd == NULL) {
        perror("Unable to open log file:");
        free(log);
        return NULL;
    } else {
        log->fd = log_fd;
        if (new_file) {
            write_header(log->fd);
        }
    }
    return log;
}

void adif_log_close(adif_log l) {
    fclose(l->fd);
}

void adif_add_qso(adif_log l, qso_log_record_t qso)
{
    write_str(l->fd, "STATION_CALLSIGN", qso.local_call);
    write_str(l->fd, "OPERATOR", qso.local_call);
    write_str(l->fd, "CALL", qso.remote_call);
    write_date_time(l->fd, qso.time);
    write_str(l->fd, "MODE", qso.mode);
    write_str(l->fd, "SUBMODE", NULL);
    write_str(l->fd, "NAME", NULL);
    write_str(l->fd, "QTH", NULL);
    write_int(l->fd, "RST_SENT", qso.rsts);
    write_str(l->fd, "STX", NULL);
    write_int(l->fd, "RST_RCVD", qso.rstr);
    write_freq_band(l->fd, qso.freq_mhz);
    write_str(l->fd, "GRIDSQUARE", qso.remote_grid);
    write_str(l->fd, "MY_GRIDSQUARE", qso.local_grid);
    fprintf(l->fd, "<EOR>\r\n");
    fflush(l->fd);
}

int adif_read(const char * path, qso_log_record_t ** records) {
    char * line = NULL;
    size_t len = 0;
    ssize_t read;

    FILE *fp = fopen(path, "r");

    if (fp == NULL) {
        perror("Unable to open log file:");
        return 0;
    }

    static regex_t  regex;
    static const    char re[] = "<([A-Za-z_]+):([0-9]+)>";
    regmatch_t      pmatch[3];
    regoff_t        off, r_len;
    char            *s;

    if (regcomp(&regex, re, REG_NEWLINE | REG_EXTENDED)) {
        printf("Can't compile regexp");
        return -2;
    }

    size_t arr_size = 128;
    *records = malloc(arr_size * sizeof(qso_log_record_t));

    qso_log_record_t *cur_record;
    ssize_t cur_record_id = 0;
    struct tm qso_ts;
    size_t val_len;

    while ((read = getline(&line, &len, fp)) != -1) {
        if (strcmp(line + read - 7, "<EOR>\r\n") != 0) continue;
        s = line;
        cur_record = &(*records)[cur_record_id];
        for (unsigned int i = 0; ; i++) {
            if (regexec(&regex, s, ARRAY_SIZE(pmatch), pmatch, 0))
                break;
            val_len = atoi(s + pmatch[2].rm_so);
            if (val_len > 0) {
                if (strncmp(s + pmatch[1].rm_so, "OPERATOR", pmatch[1].rm_eo - pmatch[1].rm_so) == 0) {
                    COPY_STR(cur_record->local_call, s + pmatch[0].rm_eo, val_len);
                } else if (strncmp(s + pmatch[1].rm_so, "CALL", pmatch[1].rm_eo - pmatch[1].rm_so) == 0) {
                    COPY_STR(cur_record->remote_call, s + pmatch[0].rm_eo, val_len);
                } else if (strncmp(s + pmatch[1].rm_so, "QSO_DATE", pmatch[1].rm_eo - pmatch[1].rm_so) == 0) {
                    strptime(s + pmatch[0].rm_eo, "%Y%m%d", &qso_ts);
                } else if (strncmp(s + pmatch[1].rm_so, "TIME_ON", pmatch[1].rm_eo - pmatch[1].rm_so) == 0) {
                    strptime(s + pmatch[0].rm_eo, "%H%M", &qso_ts);
                } else if (strncmp(s + pmatch[1].rm_so, "MODE", pmatch[1].rm_eo - pmatch[1].rm_so) == 0) {
                    COPY_STR(cur_record->mode, s + pmatch[0].rm_eo, val_len);
                } else if (strncmp(s + pmatch[1].rm_so, "NAME", pmatch[1].rm_eo - pmatch[1].rm_so) == 0) {
                    COPY_STR(cur_record->name, s + pmatch[0].rm_eo, val_len);
                } else if (strncmp(s + pmatch[1].rm_so, "QTH", pmatch[1].rm_eo - pmatch[1].rm_so) == 0) {
                    COPY_STR(cur_record->qth, s + pmatch[0].rm_eo, val_len);
                } else if (strncmp(s + pmatch[1].rm_so, "RST_SENT", pmatch[1].rm_eo - pmatch[1].rm_so) == 0) {
                    cur_record->rsts = atoi(s + pmatch[0].rm_eo);
                } else if (strncmp(s + pmatch[1].rm_so, "RST_RCVD", pmatch[1].rm_eo - pmatch[1].rm_so) == 0) {
                    cur_record->rstr = atoi(s + pmatch[0].rm_eo);
                } else if (strncmp(s + pmatch[1].rm_so, "BAND", pmatch[1].rm_eo - pmatch[1].rm_so) == 0) {
                    COPY_STR(cur_record->band, s + pmatch[0].rm_eo, val_len);
                } else if (strncmp(s + pmatch[1].rm_so, "FREQ", pmatch[1].rm_eo - pmatch[1].rm_so) == 0) {
                    cur_record->freq_mhz = strtof(s + pmatch[0].rm_eo, NULL);
                } else if (strncmp(s + pmatch[1].rm_so, "MY_GRIDSQUARE", pmatch[1].rm_eo - pmatch[1].rm_so) == 0) {
                    COPY_STR(cur_record->local_grid, s + pmatch[0].rm_eo, val_len);
                } else if (strncmp(s + pmatch[1].rm_so, "GRIDSQUARE", pmatch[1].rm_eo - pmatch[1].rm_so) == 0) {
                    COPY_STR(cur_record->remote_grid, s + pmatch[0].rm_eo, val_len);
                }
            }

            s += pmatch[0].rm_eo;
        }

        cur_record->time = mktime(&qso_ts);
        if ((strcmp(cur_record->band, util_freq_to_band(cur_record->freq_mhz)) != 0) &&
            (strcmp(cur_record->band, util_freq_to_band(cur_record->freq_mhz / 1000)) == 0)) {
                cur_record->freq_mhz /= 1000;
        }
        cur_record_id++;
        if (cur_record_id >= arr_size) {
            arr_size *= 2;
            (*records) = realloc((*records), arr_size * sizeof(qso_log_record_t));
        }
    }
    return cur_record_id--;
}

static void write_header(FILE *fd) {
    fprintf(fd, "<PROGRAMID:5>X6100\r\n");
    fprintf(fd, "<PROGRAMVERSION:5>1.0.0\r\n");
    fprintf(fd, "<ADIF_VER:4>3.14\r\n");
    fprintf(fd, "<EOH>\r\n");
}

static void write_str(FILE *fd, const char * key, const char * val) {
    if (val == NULL) {
        fprintf(fd, "<%s:0>", key);
    } else {
        size_t l = strlen(val);
        fprintf(fd, "<%s:%i>%s", key, l, val);
    }
}

static void write_int(FILE *fd, const char * key, int val) {
    char str_val[8];
    sprintf(str_val, "%i", val);
    write_str(fd, key, str_val);
}

static void write_date_time(FILE *fd, time_t time) {
    struct tm *ts = localtime(&time);
    fprintf(fd, "<QSO_DATE:8>%04i%02i%02i", ts->tm_year + 1900, ts->tm_mon + 1, ts->tm_mday);
    fprintf(fd, "<QSO_DATE_OFF:8>%04i%02i%02i", ts->tm_year + 1900, ts->tm_mon + 1, ts->tm_mday);
    fprintf(fd, "<TIME_ON:4>%02i%02i", ts->tm_hour, ts->tm_min);
    fprintf(fd, "<TIME_OFF:4>%02i%02i", ts->tm_hour, ts->tm_min);
}

static void write_freq_band(FILE *fd, float freq_mhz) {
    char * band = util_freq_to_band(freq_mhz);

    write_str(fd, "BAND", band);

    char str_freq[8];
    sprintf(str_freq, "%0.4f", freq_mhz);
    write_str(fd, "FREQ", str_freq);
}


static void copy_str(char * dst, char * src, size_t val_len, size_t dst_len) {
    if (val_len > (dst_len - 1)) {
        val_len = dst_len - 1;
    }
    strncpy(dst, src, val_len);
    dst[val_len] = 0;
}
