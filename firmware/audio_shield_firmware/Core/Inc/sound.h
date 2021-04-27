#ifndef __SOUND_H
#define __SOUND_H


#define STOP -1
#define REPEAT -2
#define BUZZER_ARR 256
#define NOTE_LENGTH 300
#define N_SPI_PER_NOTE 50


typedef const struct {
	uint16_t f;
	uint16_t PSC;
	uint16_t ARR;
	uint32_t ERR;
} freq_list_t;


typedef const struct {
	uint16_t index;
	int16_t* notes;
} melody;


typedef enum {
	BUZZER_IDLE,
	BUZZER_RECORD,
	BUZZER_PLAY_NEXT,
	BUZZER_CHOOSE_NEXT,
	BUZZER_STOP
} state_note_t;


/*
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
freq_list_t freq_list_tim[] = {
        {3010, 109, 256, -10},
        {3125, 105, 256, 0},
        {3249, 101, 256, 1},
        {3383, 97, 256, -8},
        {3491, 94, 256, 9},
        {3606, 91, 256, 19},
        {3729, 88, 256, 21},
        {3860, 85, 256, 15},
        {4002, 82, 256, -2},
        {4102, 80, 256, 23},
        {4261, 77, 256, -11},
        {4375, 75, 256, 0},
        {4495, 73, 256, 5},
        {4621, 71, 256, 4},
        {4755, 69, 256, -5},
        {4825, 68, 256, 50},
};


int16_t sweep[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, REPEAT};
int16_t mono3000[] = {0, REPEAT};

melody melodies[] = {
	{1, sweep},
	{3000, mono3000}
};

#endif /* __SOUND_H */
