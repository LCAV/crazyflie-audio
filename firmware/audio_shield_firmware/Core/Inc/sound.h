#ifndef __SOUND_H
#define __SOUND_H


#define STOP -1
#define REPEAT -2
#define BUZZER_ARR 256
#define BUZZER_RATIO 20 // Volume -> 2 = minimum, bigger -> quieter
#define BUZZER_DELAY 16
#define NOTE_LENGTH 300
#define N_SPI_PER_NOTE 1


typedef const struct {
	uint16_t f;
	uint16_t PSC;
	uint16_t ARR;
	uint32_t ERR;
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
// combinations found by frederike, with fixed ARR.
// ERR corresponds to deviation from uniform frequencies between 3000, 4875
freq_list_t freq_list_tim[] = {
        {2999, 108, 256, -1},
        {3113, 104, 256, -12},
        {3236, 100, 256, -14},
        {3370, 96, 256, -5},
        {3514, 92, 256, 14},
        {3632, 89, 256, 7},
        {3757, 86, 256, 7},
        {3891, 83, 256, 16},
        {3986, 81, 256, -14},
        {4137, 78, 256, 12},
        {4245, 76, 256, -5},
        {4358, 74, 256, -17},
        {4477, 72, 256, -23},
        {4603, 70, 256, -22},
        {4737, 68, 256, -13},
        {4878, 66, 256, 3},
        {2000, 0, 0, 0},
};


int16_t sweep[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, REPEAT};
int16_t sweep_three[] = {
		0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15,
		0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15,
		0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15,
		STOP
};

int16_t mono3000[] = {0, REPEAT};
int16_t mono4000[] = {8, REPEAT};
int16_t mono5000[] = {15, REPEAT};
int16_t monoBLANK2000[] = {16, REPEAT};

#define MELODIES_COUNT 6

melody melodies[] = {
	{1, sweep, 16},
	{3, sweep_three, 16},
	{3000, mono3000, 1},
	{4000, mono4000, 1},
	{5000, mono5000, 1},
	{12000, monoBLANK2000, 1}
};

#endif /* __SOUND_H */
