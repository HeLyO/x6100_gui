/*
 *  SPDX-License-Identifier: LGPL-2.1-or-later
 *
 *  Xiegu X6100 LVGL GUI
 *
 *  Copyright (c) 2022-2023 Belousov Oleg aka R1CBU
 */

#pragma once

#include <stdint.h>

void cat_init();
void cat_transceive(uint8_t cmd, uint8_t subcmd, uint8_t value);
void cat_transceive_level(uint8_t cmd, uint8_t subcmd, uint16_t level);
void cat_transceive_freq();
void cat_transceive_mode(uint8_t vfo, uint8_t mode);
