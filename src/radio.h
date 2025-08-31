/* Copyright (C)
* 2015 - John Melton, G0ORX/N6LYT
* 2025 - Christoph van Wüllen, DL1YCF
*
*   This program is free software: you can redistribute it and/or modify
*   it under the terms of the GNU General Public License as published by
*   the Free Software Foundation, either version 3 of the License, or
*   (at your option) any later version.
*
*   This program is distributed in the hope that it will be useful,
*   but WITHOUT ANY WARRANTY; without even the implied warranty of
*   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*   GNU General Public License for more details.
*
*   You should have received a copy of the GNU General Public License
*   along with this program.  If not, see <https://www.gnu.org/licenses/>.
*
*/

#ifndef _RADIO_H_
#define _RADIO_H_

#include "adc.h"
#include "discovered.h"
#include "receiver.h"
#include "transmitter.h"

enum _pa_power_enum {
  PA_1W = 0,
  PA_5W,
  PA_10W,
  PA_30W,
  PA_50W,
  PA_100W,
  PA_200W,
  PA_500W,
  PA_1KW
};

//
// BOOLEAN conversion macros
//
// SET    converts an int to a boolean (the result is 0 or 1)
// NOT    converts an int to a boolean and inverts it
// TOGGLE has an l-value as argument and inverts it,
//        TOGGLE(a) is equivalent to a = NOT(a)

#define SET(a) (a) ? 1 : 0
#define NOT(a) (a) ? 0 : 1
#define TOGGLE(a) a = (a) ? 0 : 1

extern DISCOVERED *radio;
extern int radio_is_remote;

extern GtkWidget *fixed;

enum _filter_board_enum {
  NO_FILTER_BOARD = 0,
  ALEX,
  APOLLO,
  CHARLY25,
  N2ADR
};

enum _region_enum {
  REGION_OTHER = 0,
  REGION_UK,
  REGION_WRC15  // 60m band allocation for countries implementing WRC15
};

enum _capture_state {
  CAP_INIT = 0,            // util first recording
  CAP_RECORDING,           // audio is being recorded
  CAP_RECORD_DONE,         // record buffer full
  CAP_AVAIL,               // audio recording finished
  CAP_REPLAY,              // audio is being re-played
  CAP_REPLAY_DONE,         // all audio has been sent
  CAP_GOTOSLEEP,           // hide capture state display
  CAP_SLEEPING             // capture state is hidden
};

//
// WDSP-independent constants used for display, detector and alc modes
// We start at 100 so no confusion with values from "old" props files
//
enum _display_enum {
  SMETER_PEAK     = 100,
  SMETER_AVERAGE,
  ALC_PEAK,
  ALC_AVERAGE,
  ALC_GAIN,  // no longer used but kept to stabilise enum valus
  DET_PEAK,
  DET_AVERAGE,
  DET_ROSENFELL,
  DET_SAMPLEHOLD,
  AVG_NONE,
  AVG_RECURSIVE,
  AVG_LOGRECURSIVE,
  AVG_TIMEWINDOW
};

extern long long frequency_calibration;
extern int region;

extern int RECEIVERS;
extern int PS_TX_FEEDBACK;
extern int PS_RX_FEEDBACK;

extern RECEIVER *receiver[];
extern RECEIVER *active_receiver;

extern TRANSMITTER *transmitter;

enum _cw_keyer_mode_enum {
  KEYER_STRAIGHT = 0,
  KEYER_MODE_A,
  KEYER_MODE_B
};

enum _mic_input_xlr_enum {
  MIC3P55MM = 0,
  MICXLR
};

enum _sat_mode_enum {
  SAT_NONE,
  SAT_MODE,
  RSAT_MODE
};

extern int sat_mode;

extern int soapy_radio_sample_rate;
extern int soapy_iqswap;

extern int atlas_penelope;
extern int atlas_clock_source_10mhz;
extern int atlas_clock_source_128mhz;
extern int atlas_mic_source;
extern int atlas_janus;

extern int tx_out_of_band_allowed;

extern int filter_board;
extern int pa_enabled;
extern int pa_power;
extern double pa_trim[11];
extern const int pa_power_list[];

extern int display_zoompan;
extern int display_sliders;
extern int display_toolbar;

extern int mic_linein;
extern double linein_gain;
extern int mic_boost;
extern int mic_bias_enabled;
extern int mic_ptt_enabled;
extern int mic_ptt_tip_bias_ring;
extern int mic_input_xlr;

extern int receivers;

extern ADC adc[3];

extern int locked;

extern int duplex;
extern int mute_rx_while_transmitting;
extern int rx_height;

extern int cw_keys_reversed;
extern int cw_keyer_speed;
extern int cw_keyer_mode;
extern int cw_keyer_weight;
extern int cw_keyer_spacing;
extern int cw_keyer_internal;
extern int cw_keyer_sidetone_volume;
extern int cw_keyer_ptt_delay;
extern int cw_keyer_hang_time;
extern int cw_keyer_sidetone_frequency;
extern int cw_breakin;
extern int cw_ramp_width;

extern int enable_auto_tune;
extern int auto_tune_flag;
extern int auto_tune_end;

extern int enable_tx_inhibit;
extern int TxInhibit;

extern int vfo_encoder_divisor;
extern int vfo_snap;

extern int protocol;
extern int device;
extern int new_pa_board;
extern int ozy_software_version;
extern int mercury_software_version[2];
extern int penelope_software_version;
extern int ptt;
extern int mox;
extern int pre_mox;
extern int memory_tune;
extern int full_tune;

extern int tx_fifo_underrun;
extern int tx_fifo_overrun;
extern int high_swr_seen;
extern int sequence_errors;

extern unsigned int exciter_power;
extern unsigned int alex_forward_power;
extern unsigned int alex_reverse_power;
extern unsigned int ADC1;
extern unsigned int ADC0;

extern int split;

extern unsigned char OCtune;
extern int OCfull_tune_time;
extern int OCmemory_tune_time;
extern long long tune_timeout;

extern int analog_meter;

extern int vox_enabled;
extern double vox_threshold;
extern double vox_hang;
extern int vox;
extern int CAT_cw_is_active;
extern int MIDI_cw_is_active;
extern int radio_ptt;
extern int cw_key_hit;
extern int n_adc;

extern int diversity_enabled;
extern double div_cos, div_sin;
extern double div_gain, div_phase;

extern int capture_state;
extern const int capture_max;
extern int capture_record_pointer;
extern int capture_replay_pointer;
extern double *capture_data;

extern int can_transmit;

extern int have_rx_gain;         // programmable RX gain available
extern int have_rx_att;          // step attenuator available -31 ... 0 dB
extern int have_preamp;          // switchable preamp
extern int have_dither;          // Dither bit can be used
extern int have_alex_att;        // ALEX board does have 0/10/20/30 dB attenuator
extern int have_saturn_xdma;     // Saturn can use Network or XDMA interface
extern int have_lime;            // The radio is a LIME-SDR
extern int have_radioberry1;     // RadioBerry with first-generation  firmware
extern int have_radioberry2;     // RadioBerry with second-generation firmware
extern int rx_gain_calibration;  // used to calibrate the input signal

extern double drive_min;         // minimum value of the drive slider
extern double drive_max;         // maximum value of the drive slider
extern double drive_digi_max;    // maximum value allowed in DIGU/DIGL

extern int display_warnings;
extern int display_pacurr;

extern int hl2_audio_codec;
extern int hl2_cl1_input;

extern int anan10E;

extern int mute_spkr_amp;        // Mute audio amplifier in radio    (ANAN-7000, G2)
extern int mute_spkr_xmit;       // Mute audio amplifier in radio upon transmitting (ANAN-7000, G2)

extern int VFO_WIDTH;
extern int VFO_HEIGHT;
extern const int MIN_METER_WIDTH;
extern int METER_WIDTH;
extern const int MENU_WIDTH;

extern int rx_stack_horizontal;
extern int suppress_popup_sliders;
extern const int tx_dialog_width;
extern const int tx_dialog_height;

//
// All global functions declared here start with "radio_",
// exception: my_combo_attach()
//
extern gboolean radio_keypress_cb(GtkWidget *widget, GdkEventKey *event, gpointer data);
extern void   radio_set_mox(int state);
extern void   radio_toggle_mox(void);
extern void   radio_toggle_tune(void);
extern void   radio_save_state();
extern void   radio_stop(void);
extern void   radio_reconfigure(void);
extern void   radio_reconfigure_screen(void);
extern void   radio_start_radio(void);
extern void   radio_change_receivers(int r);
extern void   radio_change_sample_rate(int rate);
extern void   radio_apply_band_settings(int flag);
extern void   radio_tx_vfo_changed(void);
extern void   radio_split_toggle(void);
extern void   radio_set_split(int v);
extern void   radio_set_mox(int state);
extern void   radio_set_twotone(TRANSMITTER *tx, int state);
extern int    radio_get_mox(void);
extern void   radio_set_tune(int state);
extern void   radio_set_vox(int state);
extern double radio_get_drive(void);
extern void   radio_set_drive(double d);
extern void   radio_calc_drive_level(void);
extern void   radio_calc_tune_drive_level(void);
extern void   radio_set_attenuation(int id, int value);
extern void   radio_set_random(int id, int value);
extern void   radio_set_dither(int id, int value);
extern void   radio_set_preamp(int id, int value);
extern void   radio_set_c25_att(int id, int value);
extern void   radio_set_alex_attenuation(int v);
extern int    radio_is_transmitting(void);
extern void   radio_set_satmode(int mode);
extern int    radio_max_band(void);
extern void   radio_start_playback(void);
extern void   radio_end_playback(void);
extern void   radio_start_capture(void);
extern void   radio_end_capture(void);
extern void   radio_protocol_run(void);
extern void   radio_protocol_stop(void);
extern void   radio_protocol_restart(void);
extern void   radio_start_auto_tune(void);
extern void   radio_set_anan10E(int new);
extern void   radio_set_squelch_enable(int id, int enable);
extern void   radio_set_squelch(int id, double value);
extern void   radio_set_agc_gain(int id, double value);
extern void   radio_set_af_gain(int id, double value);
extern void   radio_set_rf_gain(int id, double value);
extern void   radio_set_mic_gain(double value);
extern void   radio_set_linein_gain(double value);

extern int compare_doubles(const void *a, const void *b);

extern int  radio_remote_change_receivers(gpointer data);
extern int  radio_remote_start(gpointer data);
extern int  radio_remote_set_mox(gpointer data);
extern int  radio_remote_set_vox(gpointer data);
extern int  radio_remote_set_tune(gpointer data);
extern int  radio_remote_set_twotone(gpointer data);
extern int  radio_remote_protocol_run(gpointer data);
extern int  radio_remote_protocol_stop(gpointer data);

extern int optimize_for_touchscreen;
extern void my_combo_attach(GtkGrid *grid, GtkWidget *combo, int row, int col, int spanrow, int spancol);

//
// Macro to flag an unimplemented client/server feature,
// or a client trying to do things only a server should do.
//
// This results in a fatal error with a dialog box
//
#define ASSERT_SERVER(ret)                                                    \
    if (radio_is_remote) {                                                    \
      char *msg = g_new(char, 512);                                           \
      snprintf(msg, 512, "WARNING %s:\nClient/Server Not Implemented!", __FUNCTION__); \
      g_idle_add(fatal_error, msg);                                           \
      return ret;                                                             \
   }

//
// Macro for a memory barrier, preventing changing the execution order
// or memory accesses (used for ring buffers)
//
#define MEMORY_BARRIER asm volatile ("" ::: "memory")
#endif
