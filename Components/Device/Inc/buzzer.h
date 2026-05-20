/**
  ******************************************************************************
  * @file    buzzer.h
  * @brief   Passive buzzer — PB15 / TIM12 CH2
  ******************************************************************************
  */

#ifndef BUZZER_H
#define BUZZER_H

#include "main.h"
#include "tim.h"
#include "cmsis_os.h"

/* ============================================================
   HARDWARE
   ============================================================ */
#define BUZZER_TIM          htim12
#define BUZZER_TIM_CHANNEL  TIM_CHANNEL_2
#define BUZZER_PSC          24U             /* 240 MHz / 24 = 10 MHz        */
#define BUZZER_TIM_CLK_HZ   10000000UL     /* ARR = 10_000_000 / freq_hz   */

/* ============================================================
   NOTE FREQUENCIES (Hz)
   ============================================================ */
#define PAUSE      0

#define NOTE_C1    33
#define NOTE_CS1   35
#define NOTE_D1    37
#define NOTE_DS1   39
#define NOTE_E1    41
#define NOTE_F1    44
#define NOTE_FS1   46
#define NOTE_G1    49
#define NOTE_GS1   52
#define NOTE_A1    55
#define NOTE_AS1   58
#define NOTE_B1    62

#define NOTE_C2    65
#define NOTE_CS2   69
#define NOTE_D2    73
#define NOTE_DS2   78
#define NOTE_E2    82
#define NOTE_F2    87
#define NOTE_FS2   93
#define NOTE_G2    98
#define NOTE_GS2   104
#define NOTE_A2    110
#define NOTE_AS2   117
#define NOTE_B2    123

#define NOTE_C3    131
#define NOTE_CS3   139
#define NOTE_D3    147
#define NOTE_DS3   156
#define NOTE_E3    165
#define NOTE_F3    175
#define NOTE_FS3   185
#define NOTE_G3    196
#define NOTE_GS3   208
#define NOTE_A3    220
#define NOTE_AS3   233
#define NOTE_B3    247

#define NOTE_C4    262
#define NOTE_CS4   277
#define NOTE_D4    294
#define NOTE_DS4   311
#define NOTE_E4    330
#define NOTE_F4    349
#define NOTE_FS4   370
#define NOTE_G4    392
#define NOTE_GS4   415
#define NOTE_A4    440
#define NOTE_AS4   466
#define NOTE_B4    494

#define NOTE_C5    523
#define NOTE_CS5   554
#define NOTE_D5    587
#define NOTE_DS5   622
#define NOTE_E5    659
#define NOTE_F5    698
#define NOTE_FS5   740
#define NOTE_G5    784
#define NOTE_GS5   831
#define NOTE_A5    880
#define NOTE_AS5   932
#define NOTE_B5    988

#define NOTE_C6    1047
#define NOTE_CS6   1109
#define NOTE_D6    1175
#define NOTE_DS6   1245
#define NOTE_E6    1319
#define NOTE_F6    1397
#define NOTE_FS6   1480
#define NOTE_G6    1568
#define NOTE_GS6   1661
#define NOTE_A6    1760
#define NOTE_AS6   1865
#define NOTE_B6    1976

#define NOTE_C7    2093
#define NOTE_CS7   2217
#define NOTE_D7    2349
#define NOTE_DS7   2489
#define NOTE_E7    2637
#define NOTE_F7    2794
#define NOTE_FS7   2960
#define NOTE_G7    3136
#define NOTE_GS7   3322
#define NOTE_A7    3520
#define NOTE_AS7   3729
#define NOTE_B7    3951

///* ============================================================
//   DURATIONS (ms) @ 120 BPM
//   ============================================================ */
//#define WHOLE            2000
//#define HALF             1000
//#define QUARTER           500
//#define EIGHTH            250
//#define SIXTEENTH         125
//#define DOTTED_QUARTER    750
//#define DOTTED_EIGHTH     375

/* ============================================================
   DURATIONS 
   ============================================================ */
#define WHOLE            1
#define HALF             2
#define QUARTER          4
#define EIGHTH           8
#define SIXTEENTH        16
#define DOTTED_QUARTER   5  
#define DOTTED_EIGHTH    9   


extern uint32_t current_bpm;
void Buzzer_SetBPM(uint32_t bpm);


// Call once at startup — sets prescaler and starts PWM 
void Buzzer_Init(void);
// Play a note at given frequency for duration_ms
void Buzzer_PlayNote(uint32_t freq_hz, uint32_t duration_ms);
// Play a melody given the notes and durations
void Buzzer_PlayMelody(const uint32_t *notes, const uint32_t *durations);
// Stop the buzzer 
void Buzzer_Stop(void);

extern const uint32_t Imperial_March_Notes[];
extern const uint32_t Imperial_March_Durations[];

extern const uint32_t Tetris_Theme_Notes[];
extern const uint32_t Tetris_Theme_Durations[];

extern const uint32_t Mario_Theme_Notes[];
extern const uint32_t Mario_Theme_Durations[];

extern const uint32_t Tetris_Long_Notes[];
extern const uint32_t Tetris_Long_Durations[];

#endif /* BUZZER_H */
