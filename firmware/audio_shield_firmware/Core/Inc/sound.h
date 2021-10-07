#ifndef __SOUND_H
#define __SOUND_H

#define STOP -1
#define REPEAT -2

#define FIXED_PERIOD // use fixed period

#ifdef FIXED_PERIOD
#define BUZZER_ARR 47
#endif

#define BUZZER_RATIO 2 // Volume -> 2 = minimum, bigger -> quieter
#define BUZZER_DELAY 0
#define BUZZER_WAIT_TIMEOUT 1000
#define NOTE_LENGTH 300
#define N_SPI_PER_NOTE 1
#define SWEEP_SIZE 20
#define MELODIES_COUNT 11


typedef const struct {
	uint16_t f;
	uint16_t PSC;
	uint16_t ARR;
} freq_list_t;

typedef const struct {
	int16_t index;
	int16_t *notes;
	uint8_t length;
} melody;

typedef enum {
	BUZZER_IDLE, BUZZER_WAIT_REC, BUZZER_RECORD, BUZZER_PLAY_NEXT, BUZZER_CHOOSE_NEXT, BUZZER_WAIT_SEND, BUZZER_STOP
} state_note_t;

// combinations found with pwm_tuning.py script, RESTRICT_BINS=True
#ifndef FIXED_PERIOD
freq_list_t freq_list_tim[] = {
		{2502, 684, 48},// error: 2.6
		{2625, 652, 48},// error: 0.2
		{2688, 624, 49},// error: 0.5
		{2814, 608, 48},// error: 2.4
		{2876, 595, 48},// error: 1.3
		{3002, 570, 48},// error: 2.3
		{3125, 559, 47},// error: 0.0
		{3187, 548, 47},// error: 0.1
		{3313, 506, 49},// error: 1.1
		{3375, 487, 50},// error: 0.1
		{3500, 479, 49},// error: 0.0
		{3625, 492, 46},// error: 0.2
		{3688, 437, 51},// error: 0.6
		{3812, 458, 47},// error: 0.1
		{3875, 424, 50},// error: 0.4
		{4000, 419, 49},// error: 0.0
		{4127, 423, 47},// error: 2.4
		{4189, 400, 49},// error: 2.0
		{4314, 353, 54},// error: 1.8
		{4375, 383, 49},// error: 0.0
		{4500, 365, 50},// error: 0.2
		{4626, 355, 50},// error: 1.6
		{4689, 337, 52},// error: 1.6
		{4815, 355, 48},// error: 2.9
		{4876, 324, 52},// error: 1.6
		{5000, 335, 49},// error: 0.0
		{5125, 297, 54},// error: 0.1
		{5187, 351, 45},// error: 0.2
		{5313, 309, 50},// error: 0.6
		{5377, 354, 43},// error: 2.7
		{5500, 331, 45},// error: 0.3
		{5627, 310, 47},// error: 2.0
		{3000, 0, 0}, // means we play no sound but measure at 3000
};
#else
freq_list_t freq_list_tim[] = {
		{3002, 570, 48},// error: 2.0 (0)
		{3094, 553, 48},// error: 0.25
		{3140, 545, 48},// error: -0.625
		{3186, 537, 48},// error: -1.5
		{3284, 521, 48},// error: 2.75
		{3329, 514, 48},// error: 0.875
		{3422, 500, 48},// error: 0.125
		{3470, 493, 48},// error: 1.25
		{3513, 487, 48},// error: -2.625
		{3609, 474, 48},// error: -0.375 (9)
        {3941, 434, 48},// error: 3.5
        {3987, 429, 48},// error: 2.625
        {4034, 424, 48},// error: 2.75 (12)
        {4121, 415, 48},// error: -4.0
        {4171, 410, 48},// error: -0.875
        {4222, 405, 48},// error: 3.25
        {4307, 397, 48},// error: -5.5
        {4362, 392, 48},// error: 2.625
        {4453, 384, 48},// error: -0.125
        {4499, 380, 48},// error: -1.0 (19)
		{ 1000, 0, 0 }, // means we play no sound but measure at f
		{ 2000, 0, 0 }, //
		{ 3000, 0, 0 }, //
		{ 4000, 0, 0 }, //
		{ 5000, 0, 0 }, //
		{ 1000, 1714, 48 }, // 22 error: 0.0
		{ 2000, 856, 48 }, // 23 error: 0.0
		};
#endif
// @formatter:off

int16_t sweep[] = {
		0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, REPEAT
};

int16_t sweep_three[] = {
		0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19,
		0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19,
		0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, STOP
};

int16_t mono1000[] = { SWEEP_SIZE + 5, REPEAT };
int16_t mono2000[] = { SWEEP_SIZE + 6, REPEAT };
int16_t mono3000[] = { 0, REPEAT };
int16_t mono4000[] = { 12, REPEAT };
int16_t monoBLANK1000[] = { SWEEP_SIZE + 0, REPEAT };
int16_t monoBLANK2000[] = { SWEEP_SIZE + 1, REPEAT };
int16_t monoBLANK3000[] = { SWEEP_SIZE + 2, REPEAT };
int16_t monoBLANK4000[] = { SWEEP_SIZE + 3, REPEAT };
int16_t monoBLANK5000[] = { SWEEP_SIZE + 4, REPEAT };

melody melodies[] = {
		{ 1, sweep, SWEEP_SIZE },
		{ 3, sweep_three, SWEEP_SIZE },
		{ 1000, mono1000, 1 },
		{ 2000, mono2000, 1 },
		{ 3000, mono3000, 1 },
		{ 4000, mono4000, 1 },
		{ 11000, monoBLANK1000, 1 },
		{ 12000, monoBLANK2000, 1 },
		{ 13000, monoBLANK3000, 1 },
		{ 14000, monoBLANK4000, 1 },
		{ 15000, monoBLANK5000, 1 },
};
// @formatter:on
#endif /* __SOUND_H */
