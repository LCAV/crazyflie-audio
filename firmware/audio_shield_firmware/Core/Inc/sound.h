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

typedef const struct {
	uint16_t f;
	uint16_t PSC;
	uint16_t ARR;
} freq_list_t;


typedef const struct {
	int16_t index;
	int16_t* notes;
	uint8_t length;
} melody;


typedef enum {
	BUZZER_IDLE,
	BUZZER_WAIT_REC,
	BUZZER_RECORD,
	BUZZER_PLAY_NEXT,
	BUZZER_CHOOSE_NEXT,
	BUZZER_WAIT_SEND,
	BUZZER_STOP
} state_note_t;

// combinations found with pwm_tuning.py script, RESTRICT_BINS=True
#ifndef FIXED_PERIOD
freq_list_t freq_list_tim[] = {
        {2502, 684, 48},// error: 2.6
        {2625, 652, 48},// error: 0.2
        {2751, 622, 48},// error: 1.7
        {2876, 595, 48},// error: 1.3
        {3002, 570, 48},// error: 2.3
        {3125, 559, 47},// error: 0.0
        {3252, 526, 48},// error: 2.9
        {3375, 487, 50},// error: 0.1
        {3500, 479, 49},// error: 0.0
        {3625, 492, 46},// error: 0.2
        {3751, 456, 48},// error: 1.2
        {3875, 424, 50},// error: 0.4
        {4000, 419, 49},// error: 0.0
        {4127, 423, 47},// error: 2.4
        {4251, 379, 51},// error: 1.0
        {4375, 383, 49},// error: 0.0
        {5000, 335, 49},// error: 0.0
		{3000, 0, 0}, // means we play no sound but measure at 3000
};
#else
freq_list_t freq_list_tim[] = {
        {2499, 685, 48},// error: -1.0
        {2625, 652, 48},// error: 0.0
        {2752, 622, 48},// error: 2.0
        {2876, 595, 48},// error: 1.0
        {3002, 570, 48},// error: 2.0
        {3123, 548, 48},// error: -2.0
        {3247, 527, 48},// error: -3.0
        {3375, 507, 48},// error: 0.0
        {3499, 489, 48},// error: -1.0
        {3624, 472, 48},// error: -1.0
        {3751, 456, 48},// error: 1.0
        {3878, 441, 48},// error: 3.0
        {3996, 428, 48},// error: -4.0
        {4121, 415, 48},// error: -4.0
        {4254, 402, 48},// error: 4.0
        {4373, 391, 48},// error: -2.0
        {1000, 0, 0},// means we play no sound but measure at f
		{2000, 0, 0},//
		{3000, 0, 0}, //
		{4000, 0, 0},//
		{5000, 0, 0}, //
		{1000, 1714, 48},// 22 error: 0.0
		{2000, 856, 48},// 23 error: 0.0
};
#endif

int16_t sweep[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, REPEAT};

int16_t sweep_three[] = {
		0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15,
		0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15,
		0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15,
		STOP
};

int16_t mono1000[] = {21, REPEAT};
int16_t mono2000[] = {22, REPEAT};
int16_t mono3000[] = {4, REPEAT};
int16_t mono4000[] = {12, REPEAT};
int16_t mono5000[] = {16, REPEAT};
int16_t monoBLANK1000[] = {17, REPEAT};
int16_t monoBLANK2000[] = {18, REPEAT};
int16_t monoBLANK3000[] = {19, REPEAT};
int16_t monoBLANK4000[] = {20, REPEAT};
int16_t monoBLANK5000[] = {21, REPEAT};

#define MELODIES_COUNT 6

melody melodies[] = {
	{1, sweep, 16},
	{3, sweep_three, 16},
	{1000, mono1000, 1},
	{2000, mono2000, 1},
	{3000, mono3000, 1},
	{4000, mono4000, 1},
	{5000, mono5000, 1},
	{11000, monoBLANK1000, 1},
	{12000, monoBLANK2000, 1},
	{13000, monoBLANK3000, 1},
	{14000, monoBLANK4000, 1},
	{15000, monoBLANK5000, 1}
};

#endif /* __SOUND_H */
