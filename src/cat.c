/*
 *  SPDX-License-Identifier: LGPL-2.1-or-later
 *
 *  Xiegu X6100 LVGL GUI
 *
 *  Copyright (c) 2022-2023 Belousov Oleg aka R1CBU
 */

/*
 * X6100 protocol implementation (Mfg 3087)
 */

#include "cat.h"

#include "radio.h"
#include "params/params.h"
#include "util.h"
#include "events.h"
#include "waterfall.h"
#include "spectrum.h"
#include "scheduler.h"
#include "main_screen.h"
#include "msg.h" //
#include "info.h" //

#include <aether_radio/x6100_control/low/gpio.h>
#include "lvgl/lvgl.h"
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <pthread.h>
#include <sys/poll.h>


#define FRAME_PRE       0xFE
#define FRAME_END       0xFD

#define CODE_OK         0xFB
#define CODE_NG         0xFA

#define C_SND_FREQ      0x00    /* Send frequency data  transceive mode does not ack*/
#define C_SND_MODE      0x01    /* Send mode data, Sc  for transceive mode does not ack */
#define C_RD_BAND       0x02    /* Read band edge frequencies */
#define C_RD_FREQ       0x03    /* Read display frequency */
#define C_RD_MODE       0x04    /* Read display mode */
#define C_SET_FREQ      0x05    /* Set frequency data(1) */
#define C_SET_MODE      0x06    /* Set mode data, Sc */
#define C_SET_VFO       0x07    /* Set VFO */
#define C_SET_MEM       0x08    /* Set channel, Sc(2) */
#define C_WR_MEM        0x09    /* Write memory */
#define C_MEM2VFO       0x0a    /* Memory to VFO */
#define C_CLR_MEM       0x0b    /* Memory clear */
#define C_RD_OFFS       0x0c    /* Read duplex offset frequency; default changes with HF/6M/2M */
#define C_SET_OFFS      0x0d    /* Set duplex offset frequency */
#define C_CTL_SCAN      0x0e    /* Control scan, Sc */
#define C_CTL_SPLT      0x0f    /* Control split, and duplex mode Sc */
#define C_SET_TS        0x10    /* Set tuning step, Sc */
#define C_CTL_ATT       0x11    /* Set/get attenuator, Sc */
#define C_CTL_ANT       0x12    /* Set/get antenna, Sc */
#define C_CTL_ANN       0x13    /* Control announce (speech synth.), Sc */
#define C_CTL_LVL       0x14    /* Set AF/RF/squelch, Sc */
#define C_RD_SQSM       0x15    /* Read squelch condition/S-meter level, Sc */
#define C_CTL_FUNC      0x16    /* Function settings (AGC,NB,etc.), Sc */
#define C_SND_CW        0x17    /* Send CW message */
#define C_SET_PWR       0x18    /* Set Power ON/OFF, Sc */
#define C_RD_TRXID      0x19    /* Read transceiver ID code */
#define C_CTL_MEM       0x1a    /* Misc memory/bank/rig control functions, Sc */
#define C_SET_TONE      0x1b    /* Set tone frequency */
#define C_CTL_PTT       0x1c    /* Control Transmit On/Off, Sc */
#define C_CTL_EDGE      0x1e    /* Band edges */
#define C_CTL_DVT       0x1f    /* Digital modes calsigns & messages */
#define C_CTL_DIG       0x20    /* Digital modes settings & status */
#define C_CTL_RIT       0x21    /* RIT/XIT control */
#define C_CTL_DSD       0x22    /* D-STAR Data */
#define C_SEND_SEL_FREQ 0x25    /* Send/Recv sel/unsel VFO frequency */
#define C_SEND_SEL_MODE 0x26
#define C_CTL_SCP       0x27    /* Scope control & data */
#define C_SND_VOICE     0x28    /* Transmit Voice Memory Contents */
#define C_CTL_MTEXT     0x70    /* Microtelecom Extension */
#define C_CTL_MISC      0x7f    /* Miscellaneous control, Sc */

#define S_VFOA          0x00    /* Set to VFO A */
#define S_VFOB          0x01    /* Set to VFO B */
#define S_BTOA          0xa0    /* VFO A=B */
#define S_XCHNG         0xb0    /* Switch VFO A and B */
#define S_SUBTOMAIN     0xb1    /* MAIN = SUB */
#define S_DUAL_OFF      0xc0    /* Dual watch off */
#define S_DUAL_ON       0xc1    /* Dual watch on */
#define S_DUAL          0xc2    /* Dual watch (0 = off, 1 = on) */
#define S_MAIN          0xd0    /* Select MAIN band */
#define S_SUB           0xd1    /* Select SUB band */
#define S_SUB_SEL       0xd2    /* Read/Set Main/Sub selection */
#define S_FRONTWIN      0xe0    /* Select front window */

// modes
#define M_LSB           0x00
#define M_USB           0x01
#define M_AM            0x02
#define M_CW            0x03
#define M_NFM           0x05
#define M_CWR           0x07

// memory/bank/rig control
#define MEM_BS_REG      0x01    /* Get band stacking register */
#define MEM_IF_FW       0x03    /* Get IF filter width */
#define MEM_DM_FG       0x06    /* Get data mode switch and filter group */

// AGC modes
#define AGC_OFF         0x00
#define AGC_FAST        0x01
#define AGC_SLOW        0x02
#define AGC_AUTO        0x03


static int      fd;

static uint8_t  frame[256];

static void log_msg(const char * msg, uint16_t len) {
    char buf[512];
    char *buf_ptr = buf;
    for (size_t i = 0; i < len; i++) {
        buf_ptr += sprintf(buf_ptr, "%02X:", msg[i]);
    }
    *(buf_ptr - 1) = '\0';
    LV_LOG_USER("Cmd %s (Len %i)", buf, len);
}

static uint16_t frame_get() {
    uint16_t    len = 0;
    uint8_t     c;

    memset(frame, 0, sizeof(frame));

    while (true) {
        int res = read(fd, &c, 1);

        if (res > 0) {
            frame[len++] = c;

            if (c == FRAME_END) {
                return len;
            }

            if (len >= sizeof(frame)) {
                return 0;
            }
        } else {
            usleep(10000);
        }
    }

    return 0;
}

static void prepare_answer() {
    // set dst address from sender, src address is fixed - 0xA4
    frame[2] = frame[3];
    frame[3] = 0xA4;
}

static void send_frame(uint16_t len) {
    frame[len - 1] = FRAME_END;
    write(fd, frame, len);
}

static void send_code(uint8_t code) {
    frame[4] = code;
    send_frame(6);
}

static void set_freq(void * arg) {
    if (!arg) {
        LV_LOG_ERROR("arg is NULL");
    }
    uint64_t freq = * (uint64_t*) arg;
    if (params_bands_find(freq, &params.freq_band)) {
        bands_activate(&params.freq_band, NULL);
    }

    radio_set_freq(freq);
    lv_event_send(lv_scr_act(), EVENT_SCREEN_UPDATE, NULL);
}

static void set_vfo(void * arg) {
    if (!arg) {
        LV_LOG_ERROR("arg is NULL");
    }
    x6100_vfo_t vfo = * (x6100_vfo_t*) arg;
    radio_set_vfo(vfo);
    lv_event_send(lv_scr_act(), EVENT_SCREEN_UPDATE, NULL);
}


static x6100_mode_t ci_mode_2_x_mode(uint8_t mode, uint8_t *dig_mode) {
    x6100_mode_t r_mode;
    bool data_mode = (dig_mode != NULL) && *dig_mode;
    switch (mode)
    {
    case M_LSB:
        r_mode = data_mode ? x6100_mode_lsb_dig : x6100_mode_lsb;
        break;
    case M_USB:
        r_mode = data_mode ? x6100_mode_usb_dig : x6100_mode_usb;
        break;
    case M_AM:
        r_mode = x6100_mode_am;
        break;
    case M_CW:
        r_mode = x6100_mode_cw;
        break;
    case M_NFM:
        r_mode = x6100_mode_nfm;
        break;
    case M_CWR:
        r_mode = x6100_mode_cwr;
        break;
    default:
        break;
    }
    return r_mode;
}

static uint8_t x_mode_2_ci_mode(x6100_mode_t mode) {
    switch (mode)
    {
    case x6100_mode_lsb:
    case x6100_mode_lsb_dig:
        return M_LSB;
        break;
    case x6100_mode_usb:
    case x6100_mode_usb_dig:
        return M_USB;
        break;
    case  x6100_mode_cw:
        return M_CW;
        break;
    case  x6100_mode_cwr:
        return M_CWR;
        break;
    case  x6100_mode_am:
        return M_AM;
        break;
    case  x6100_mode_nfm:
        return M_NFM;
        break;
    default:
        return 0;
        break;
    }
}

static uint8_t get_agc_mode() {
    x6100_agc_t     agc = params_band_cur_agc_get();

    switch (agc)
    {
    case x6100_agc_off:
        return AGC_OFF;
        break;
    case x6100_agc_fast:
        return AGC_FAST;
        break;
    case x6100_agc_slow:
        return AGC_SLOW;
        break;
    case x6100_agc_auto:
        return AGC_AUTO;
        break;
    default:
        return 0;
        break;
    }
}

static uint8_t get_if_bandwidth() {
    uint32_t bw = params_current_mode_filter_bw_get();
    switch (params_band_cur_mode_get())
    {
    case x6100_mode_cw:
    case x6100_mode_cwr:
    case x6100_mode_lsb:
    case x6100_mode_lsb_dig:
    case x6100_mode_usb:
    case x6100_mode_usb_dig:
        if (bw <= 500)
        {
            return (bw - 25) / 50;
        } else {
            return (bw - 50) / 100 + 5;
        }
        break;
    case x6100_mode_am:
    case x6100_mode_nfm:
        return (bw - 100) / 200;
    default:
        return 31;
        break;
    }
}

static void handle_level_change(uint8_t level_id, uint16_t min_value, uint16_t max_value, uint16_t (*radio_change_func)(int16_t), uint16_t *param, uint8_t *frame) {
    if (frame[6] == FRAME_END) {
        //uint16_t level = radio_change_func(0) * 255 / max_value;
        uint16_t level = (radio_change_func(0) - min_value) * 255 / ( max_value - min_value);
        decimalToBCD(&frame[6], level, 4);
        send_frame(9);
    } else {
        uint64_t level = bcdToDecimal(&frame[6], 4);
        level = ceil_uint64(level * max_value, 255) - *param;
        uint16_t x = radio_change_func((int16_t)level);
        frame[4] = CODE_OK;
        send_frame(6);
    }
}

static void frame_parse(uint16_t len) {
    if (frame[0] != FRAME_PRE && frame[1] != FRAME_PRE) {
        LV_LOG_ERROR("Incorrect frame");
        return;
    }
    uint64_t new_freq;
    x6100_vfo_t cur_vfo = params_band_vfo_get();
    uint64_t cur_freq = params_band_cur_freq_get();
    x6100_mode_t cur_mode = params_band_cur_mode_get();
    x6100_vfo_t target_vfo = cur_vfo;

#if 0
    log_msg(frame, len);
#endif

    // echo input frame
    send_frame(len);
    prepare_answer();

    switch (frame[4]) {
        case C_RD_FREQ: {
            to_bcd(&frame[5], cur_freq, 10);
            send_frame(11);
            break;
        }
        case C_RD_MODE: {
            uint8_t v = x_mode_2_ci_mode(cur_mode);
            frame[5] = v;
            frame[6] = v;
            send_frame(8);
            break;
        }
        case C_SET_FREQ: {
            new_freq = from_bcd(&frame[5], 10);
            if (new_freq != cur_freq)
            {
                scheduler_put(set_freq, &new_freq, sizeof(new_freq));
            }
            send_code(CODE_OK);
            break;
        }
        case C_SET_MODE: {
            x6100_mode_t new_mode = ci_mode_2_x_mode(frame[5], NULL);
            if (new_mode != cur_mode) {
                scheduler_put(main_screen_set_mode, &new_mode, sizeof(new_mode));
            }
            send_code(CODE_OK);
            break;
        }
        case C_CTL_LVL: {
            if (frame[5] == 0x01) { // AF level
                if (frame[6] == FRAME_END) {
                    uint16_t af_lvl = radio_change_vol(0) * 255 / 55;
                    decimalToBCD(&frame[6], af_lvl, 4);
                    send_frame(9);
                } else {
                    uint64_t af_lvl = bcdToDecimal(&frame[6], 4);
                    af_lvl = ceil_uint64(af_lvl * 55, 255) - params.vol;
                    uint16_t x = radio_change_vol((int16_t)(af_lvl));
                    frame[4] = CODE_OK;
                    send_frame(6);
                }
            }
            if (frame[5] == 0x02) { // RF level
                if (frame[6] == FRAME_END) {
                    uint16_t rf_lvl = radio_change_rfg(0) * 255 / 100;
                    decimalToBCD(&frame[6], rf_lvl, 4);
                    send_frame(9);
                } else {
                    uint64_t rf_lvl = bcdToDecimal(&frame[6], 4);
                    rf_lvl = ceil_uint64(rf_lvl * 100, 255) - params_band_rfg_get();
                    uint16_t x = radio_change_rfg((int16_t)(rf_lvl));
                    frame[4] = CODE_OK;
                    send_frame(6);
                }                
            }
            if (frame[5] == 0x03) { // SQL level
                if (frame[6] == FRAME_END) {
                    uint16_t sql_lvl = radio_change_sql(0) * 255 / 100;
                    decimalToBCD(&frame[6], sql_lvl, 4);
                    send_frame(9);
                } else {
                    uint64_t sql_lvl = bcdToDecimal(&frame[6], 4);
                    sql_lvl = ceil_uint64(sql_lvl * 100, 255) - params.sql;
                    uint16_t x = radio_change_sql((int16_t)(sql_lvl));
                    frame[4] = CODE_OK;
                    send_frame(6);
                }                
            } 
            if (frame[5] == 0x06) { // NR level
                if (frame[6] == FRAME_END) {
                    uint16_t nr_lvl = radio_change_nr_level(0) * 255 / 60;
                    decimalToBCD(&frame[6], nr_lvl, 4);
                    send_frame(9);
                } else {
                    uint64_t nr_lvl = bcdToDecimal(&frame[6], 4);
                    nr_lvl = ceil_uint64(nr_lvl * 60, 255) - params.nr_level;
                    uint16_t x = radio_change_nr_level((int16_t)(nr_lvl / 5));
                    frame[4] = CODE_OK;
                    send_frame(6);
                }
            }
            if (frame[5] == 0x12) { // NB level
                if (frame[6] == FRAME_END) {
                    uint16_t nb_lvl = radio_change_nb_level(0) * 255 / 100;
                    decimalToBCD(&frame[6], nb_lvl, 4);
                    send_frame(9);
                } else {
                    uint64_t nb_lvl = bcdToDecimal(&frame[6], 4);
                    nb_lvl = ceil_uint64(nb_lvl * 100, 255) - params.nb_level;
                    uint16_t x = radio_change_nb_level((int16_t)(nb_lvl / 5));
                    frame[4] = CODE_OK;
                    send_frame(6);
                }                               
            }
            if (frame[5] == 0x0A) { // TX Power level
                if (frame[6] == FRAME_END) {
                    uint16_t pwr_lvl = radio_change_pwr(0) * 255 / 10;
                    decimalToBCD(&frame[6], pwr_lvl, 4);
                    send_frame(9);
                } else {
                    uint64_t pwr_lvl = bcdToDecimal(&frame[6], 4);
                    pwr_lvl = ceil_uint64(pwr_lvl * 10, 255) - params.pwr;
                    uint16_t x = radio_change_pwr((int16_t)(pwr_lvl * 10));
                    frame[4] = CODE_OK;
                    send_frame(6);
                }                               
            }              
            if (frame[5] == 0x0D) { // DNF level
                if (frame[6] == FRAME_END) {
                    uint16_t dnf_lvl = (radio_change_dnf_center(0) - 100) * 255 / (3000 - 100);
                    decimalToBCD(&frame[6], dnf_lvl, 4);
                    send_frame(9);
                } else {
                    uint64_t dnf_lvl = bcdToDecimal(&frame[6], 4);
                    dnf_lvl = round_up_to_next_50(ceil_uint64(dnf_lvl * (3000 - 100), 255)) + 100 - params.dnf_center;
                    uint16_t x = radio_change_dnf_center((int16_t)(dnf_lvl / 50));
                    frame[4] = CODE_OK;
                    send_frame(6);
                }                               
            }                                     
            break;
        }
        // case C_CTL_LVL: {
        //     switch (frame[5]) {
        //         case 0x01: // AF level
        //             handle_level_change(0x01, 0, 55, radio_change_vol, &params.vol, frame);
        //             break;
        //         case 0x02: // RF level
        //         {
        //             uint16_t rfg = params_band_rfg_get();
        //             handle_level_change(0x02, 0, 100, radio_change_rfg, &rfg, frame);
        //             break;
        //         }
        //         case 0x03: // SQL level
        //             handle_level_change(0x03, 0, 100, radio_change_sql, &params.sql, frame);
        //             break;
        //         case 0x06: // NR level
        //             handle_level_change(0x06, 0, 60, radio_change_nr_level, &params.nr_level, frame);
        //             break;
        //         case 0x12: // NB level
        //             handle_level_change(0x12, 0, 100, radio_change_nb_level, &params.nb_level, frame);
        //             break;
        //         case 0x0A: // TX Power level
        //             handle_level_change(0x0A, 0.1, 10, radio_change_pwr, &params.pwr, frame);
        //             break;
        //         case 0x0D: // DNF level
        //             handle_level_change(0x0D, 100, 3000, radio_change_dnf_center, &params.dnf_center, frame);
        //             break;
        //     }
        //     break;
        // }              
        case C_CTL_PTT: {
            if (frame[5] == 0x00) { // PTT function
                if (frame[6] == FRAME_END) {
                    frame[6] = (radio_get_state() == RADIO_RX) ? 0 : 1;
                    send_frame(8);
                } else {
                    switch (frame[6]) {
                        case 0:
                            radio_set_ptt(false);
                            break;
                        case 1:
                            radio_set_ptt(true);
                            break;
                    }
                    frame[6] = CODE_OK;
                    send_frame(8);
                }
            }
            if (frame[5] == 0x01) { // ATU function
                if (frame[6] == FRAME_END) {
                    frame[6] = (params.atu == 0) ? 0 : 1;
                    send_frame(8);
                } else {
                    switch (frame[6]) {
                        case 0:
                        case 1:
                            radio_change_atu();
                            info_params_set();
                            break;
                        case 2:
                            radio_start_atu();
                            break;
                    }
                    frame[6] = CODE_OK;
                    send_frame(8);
                }
            }            
            break;
        }
        case C_CTL_ATT: {
            if (frame[5] == FRAME_END) { // ATT function
                frame[5] = (params_band_cur_att_get() == 0) ? 0 : 1;
                send_frame(7);
            } else {
                    radio_change_att();
                    info_params_set();
                    frame[5] = CODE_OK;
                    send_frame(8);             
                    }
            break;
        }
        case C_CTL_FUNC: {
            if (frame[5] == 0x02) { // PRE function
                if (frame[6] == FRAME_END) {
                    frame[6] = (params_band_cur_pre_get() == 0) ? 0 : 1;
                    send_frame(8);
                } else {
                        radio_change_pre();
                        info_params_set();
                        frame[6] = CODE_OK;
                        send_frame(8);
                    }
            }
            if (frame[5] == 0x12) { // AGC function
                if (frame[6] == FRAME_END) {
                    frame[6] = get_agc_mode();
                    send_frame(8);
                } else {
                        radio_change_agc();
                        info_params_set();
                        frame[6] = CODE_OK;
                        send_frame(8);
                    }
            }            
            if (frame[5] == 0x40) { // NR function
                if (frame[6] == FRAME_END) {
                    frame[6] = (radio_change_nr(0) == 0) ? 0 : 1;
                    send_frame(8);
                } else {
                    bool b;
                    b = radio_change_nr(1);
                    msg_update_text_fmt("#FFFFFF NR: %s", b ? "On" : "Off");
                    frame[6] = CODE_OK;
                    send_frame(8);
                }
            }
            if (frame[5] == 0x22) { // NB function
                if (frame[6] == FRAME_END) {
                    frame[6] = (radio_change_nb(0) == 0) ? 0 : 1;
                    send_frame(8);
                } else {
                    bool b;
                    b = radio_change_nb(1);
                    msg_update_text_fmt("#FFFFFF NB: %s", b ? "On" : "Off");
                    frame[6] = CODE_OK;
                    send_frame(8);
                }
            }
            if (frame[5] == 0x41) { // DNF function
                if (frame[6] == FRAME_END) {
                    frame[6] = (radio_change_dnf(0) == 0) ? 0 : 1;
                    send_frame(8);
                } else {
                bool b;
                b = radio_change_dnf(1);
                msg_update_text_fmt("#FFFFFF DNF: %s", b ? "On" : "Off");
                frame[6] = CODE_OK;
                send_frame(8);
                }             
            }                                 
            break;
        }      
        case C_SET_VFO: {
            x6100_vfo_t new_vfo;
            switch (frame[5]) {
            case S_VFOA: {              
                if (cur_vfo != X6100_VFO_A) {
                    new_vfo = X6100_VFO_A;
                    scheduler_put(set_vfo, &new_vfo, sizeof(new_vfo));
                }
                send_code(CODE_OK);
                break;
            }
            case S_VFOB: {
                if (cur_vfo != X6100_VFO_B) {
                    new_vfo = X6100_VFO_B;
                    scheduler_put(set_vfo, &new_vfo, sizeof(new_vfo));
                }
                send_code(CODE_OK);
                break;
            }
            case S_XCHNG: { // Switch VFO A and B
                radio_toggle_vfo();
                info_params_set();
                waterfall_set_freq(params_band_cur_freq_get());
                spectrum_clear();
                main_screen_band_set();
                send_code(CODE_OK);
                break;
            }
            default:
                LV_LOG_WARN("Unsupported %02X:%02X (Len %i)", frame[4], frame[5], len);
                send_code(CODE_NG);
                break;
            }
            break;
        }
        case C_SEND_SEL_FREQ: {
            if (frame[5]) {
                target_vfo = (cur_vfo == X6100_VFO_A) ? X6100_VFO_B : X6100_VFO_A;
            }
            if (frame[6] == FRAME_END) {
                uint64_t freq = params_band_vfo_freq_get(target_vfo);
                to_bcd(&frame[6], freq, 10);
                send_frame(12);
            } else {
                uint64_t freq = from_bcd(&frame[6], 10);
                if (params_band_vfo_freq_get(target_vfo) != freq){
                    params_band_vfo_freq_set(target_vfo, freq);

                    if (cur_vfo == target_vfo) {
                        scheduler_put(set_freq, &freq, sizeof(freq));
                    }
                }
                send_code(CODE_OK);
            }
            break;
        }
        case C_CTL_SPLT: {
            if (frame[5] == FRAME_END) { // SPLT function
                frame[5] = (params_band_split_get() == 0) ? 0 : 1;
                send_frame(7);
            } else {
                    radio_toggle_split();
                    info_params_set();
                    waterfall_set_freq(params_band_cur_freq_get());
                    spectrum_clear();
                    main_screen_band_set();
                    if (params.mag_info.x) {
                        msg_tiny_set_text_fmt("%s", info_params_vfo());
                    }                                       
                    frame[5] = CODE_OK;
                    send_frame(8);             
                    }
            break;
        }
        case C_SEND_SEL_MODE: {
            if (frame[5]) {
                target_vfo = (cur_vfo == X6100_VFO_A) ? X6100_VFO_B : X6100_VFO_A;
            }
            if (frame[6] == FRAME_END) {
                uint8_t v = x_mode_2_ci_mode(params_band_vfo_mode_get(target_vfo));
                frame[6] = v;
                frame[7] = 0;
                frame[8] = 1;
                send_frame(10);
            } else {
                // TODO: Add filters applying
                x6100_mode_t new_mode = ci_mode_2_x_mode(frame[6], &frame[7]);
                scheduler_put(main_screen_set_mode, &new_mode, sizeof(new_mode));
                send_code(CODE_OK);
            }
            break;
        }
        case C_CTL_MEM: {
            // TODO: Implement another options
            if (frame[6] == FRAME_END) {
                switch (frame[5])
                {
                case MEM_IF_FW:
                    frame[6] = get_if_bandwidth();
                    send_frame(8);
                    break;

                default:
                    LV_LOG_WARN("Unsupported %02X:%02X (Len %i)", frame[4], frame[5], len);
                    send_code(CODE_NG);
                    break;
                }
            } else {
                LV_LOG_WARN("Unsupported %02X:%02X (Len %i)", frame[4], frame[5], len);
                send_code(CODE_NG);
            }
            break;
        }
        case C_RD_TRXID: {
            if (frame[5] == 0) {
                frame[6] = 0xa4;
                send_frame(8);
            }
            break;
        }
        default:
            LV_LOG_WARN("Unsupported %02X:%02X (Len %i)", frame[4], frame[5], len);
            send_code(CODE_NG);
            break;
    }
}

static void * cat_thread(void *arg) {
    while (true) {
        uint16_t len = frame_get();

        if (len >= 0) {
            frame_parse(len);
        }
    }
}

void cat_init() {
    /* UART */

    x6100_gpio_set(x6100_pin_usb, 1);  /* USB -> CAT */

    fd = open("/dev/ttyS2", O_RDWR | O_NONBLOCK | O_NOCTTY);

    if (fd > 0) {
        struct termios attr;

        tcgetattr(fd, &attr);

        cfsetispeed(&attr, B19200);
        cfsetospeed(&attr, B19200);
        cfmakeraw(&attr);

        if (tcsetattr(fd, 0, &attr) < 0) {
            close(fd);
            LV_LOG_ERROR("UART set speed");
        }
    } else {
        LV_LOG_ERROR("UART open");
    }

    /* * */

    pthread_t thread;

    pthread_create(&thread, NULL, cat_thread, NULL);
    pthread_detach(thread);
}
