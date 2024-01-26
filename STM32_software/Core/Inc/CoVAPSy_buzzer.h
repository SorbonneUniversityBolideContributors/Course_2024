#include "tim.h"
#include "stdint.h"

#define NOTE_DO3 261.63
#define NOTE_RE3 293.66
#define NOTE_MI3 329.63
#define NOTE_FA3 349.23
#define NOTE_SOL3 392
#define NOTE_LA3 440
#define NOTE_SI3 493.88
#define NOTE_DO4 523.25

//notes musique pirates des C
#define NOTE_C4  262   //Defining note frequency
#define NOTE_D4  294
#define NOTE_E4  330
#define NOTE_F4  349
#define NOTE_G4  392
#define NOTE_A4  440
#define NOTE_B4  494
#define NOTE_C5  523
#define NOTE_D5  587
#define NOTE_E5  659
#define NOTE_F5  698
#define NOTE_G5  784
#define NOTE_A5  880
#define NOTE_B5  988


#define FREQ_TIMER_6 1000000 //1MHz

void buzzer_start_frequency_Hz(float frequency_Hz);

void buzzer_start(void);

void buzzer_stop(void);

void buzzer_gamme(void);
