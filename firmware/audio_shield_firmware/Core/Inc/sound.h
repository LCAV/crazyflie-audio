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
	BUZZER_STOP
} state_note_t;


/* optimal combinations found by Adrien
freq_list_t freq_list_tim[] = {
		{3000, 26, 1038, 35 },
		{3125, 0, 26879, 0 },
		{3250, 70, 363, 86 },
		{3375, 407, 60, 36 },
		{3500, 0, 23999, 0 },
		{3625, 46, 492, 61 },
		{3750, 0, 22399, 0 },
		{3875, 52, 408, 19 },
		{4000, 0, 20999, 0 },
		{4125, 10, 1850, 12 },
		{4250, 35, 548, 36 },
		{4375, 0, 19199, 0 },
		{4500, 17, 1036, 36 },
		{4625, 2017, 8, 9 },
		{4750, 1359, 12, 125 },
		{4875, 3445, 4, 45 }
};
*/
// combinations found with pwm_tuning.py script, RESTRICT_BINS=True
#ifndef FIXED_PERIOD
freq_list_t freq_list_tim[] = {
        {2502, 684, 48},// error: 2.6
        {2688, 624, 49},// error: 0.5
        {2814, 608, 48},// error: 2.4
        {3002, 570, 48},// error: 2.3
        {3187, 548, 47},// error: 0.1
        {3313, 506, 49},// error: 1.1
        {3500, 479, 49},// error: 0.0
        {3688, 437, 51},// error: 0.6
        {3812, 458, 47},// error: 0.1
        {4000, 419, 49},// error: 0.0
        {4189, 400, 49},// error: 2.0
        {4314, 353, 54},// error: 1.8
        {4500, 365, 50},// error: 0.2
        {4689, 337, 52},// error: 1.6
        {4815, 355, 48},// error: 2.9
        {5000, 335, 49},// error: 0.0
		{3000, 0, 0}, // means we play no sound but measure at 3000
};
#else
freq_list_t freq_list_tim[] = {
        {2499, 685, 48},// error: -1.0
        {2687, 637, 48},// error: -0.5
        {2810, 609, 48},// error: -2.5
        {3002, 570, 48},// error: 2.0
        {3186, 537, 48},// error: -1.5
        {3309, 517, 48},// error: -3.5
        {3499, 489, 48},// error: -1.0
        {3687, 464, 48},// error: -0.5
        {3810, 449, 48},// error: -2.5
        {3996, 428, 48},// error: -4.0
        {4191, 408, 48},// error: 3.5
        {4307, 397, 48},// error: -5.5
        {4499, 380, 48},// error: -1.0
        {4684, 365, 48},// error: -3.5
        {4815, 355, 48},// error: 2.5
        {4998, 342, 48},// error: -2.0 //15
        {1000, 0, 0},// means we play no sound but measure at f
		{2000, 0, 0},//
		{3000, 0, 0}, //
		{4000, 0, 0},//
		{5000, 0, 0}, //
		{1000, 1714, 48},// 21 error: 0.0
		{2000, 856, 48},// 22 error: 0.0
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
int16_t mono3000[] = {3, REPEAT};
int16_t mono4000[] = {6, REPEAT};
int16_t mono5000[] = {9, REPEAT};
int16_t monoBLANK1000[] = {16, REPEAT};
int16_t monoBLANK2000[] = {17, REPEAT};
int16_t monoBLANK3000[] = {18, REPEAT};
int16_t monoBLANK4000[] = {19, REPEAT};
int16_t monoBLANK5000[] = {20, REPEAT};

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
