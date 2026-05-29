/**
  ******************************************************************************
  * @file    buzzer.c
  * @brief   Passive buzzer — PB15 / TIM12 CH2
  ******************************************************************************
  */
// os_delay has been changed with HAL_Delay for start since task scheduler has not started during the execution
#include "buzzer.h"
void Buzzer_Init(void)
{
    __HAL_TIM_SET_PRESCALER(&BUZZER_TIM, BUZZER_PSC);
    __HAL_TIM_SET_COMPARE(&BUZZER_TIM, BUZZER_TIM_CHANNEL, 0);
    HAL_TIM_PWM_Start(&BUZZER_TIM, BUZZER_TIM_CHANNEL);
}

void Buzzer_PlayNote(uint32_t freq_hz, uint32_t duration_ms)
{
    if (freq_hz == 0)
    {
        Buzzer_Stop();
        
				osDelay(duration_ms);
        return;
    }

    uint32_t arr = BUZZER_TIM_CLK_HZ / freq_hz; //Calculate buzzer input from desired frequency

    __HAL_TIM_SET_AUTORELOAD(&BUZZER_TIM, arr); //Set note
    __HAL_TIM_SET_COMPARE(&BUZZER_TIM, BUZZER_TIM_CHANNEL, arr / 2); /* 50% = max volume */

    osDelay(duration_ms);
  
}
uint32_t current_bpm = 200;

// Adapt durations on bpm
static uint32_t TokenToMs(uint32_t duration_token)
{
    if (current_bpm == 0) return 0;

    uint32_t whole = (60000UL * 4) / current_bpm;

    switch (duration_token)
    {
        case WHOLE:            return whole;
        case HALF:             return whole / 2;
        case QUARTER:          return whole / 4;
        case EIGHTH:           return whole / 8;
        case SIXTEENTH:        return whole / 16;
        case DOTTED_QUARTER:   return (whole / 4) + (whole / 8); 
        case DOTTED_EIGHTH:    return (whole / 8) + (whole / 16);
        default:               return 0;
    }
}

void Buzzer_SetBPM(uint32_t bpm)
{
    if (bpm > 0) current_bpm = bpm;
}

void Buzzer_PlayMelody(const uint32_t *notes, const uint32_t *durations)
{
    if (notes == NULL || durations == NULL)
        return;

    for (; *durations != 0; notes++, durations++)
    {
        
        uint32_t actual_duration = TokenToMs(*durations);
        
        Buzzer_PlayNote(*notes, (actual_duration/PLAY_RATE));
        
        Buzzer_PlayNote(0, 10); 
    }

    Buzzer_Stop();
}

void Buzzer_Stop(void)
{
    __HAL_TIM_SET_COMPARE(&BUZZER_TIM, BUZZER_TIM_CHANNEL, 0);
}

/* ============================================================
   MELODIES 
   ============================================================ */

const uint32_t Imperial_March_Notes[] = {
    NOTE_A4, NOTE_A4, NOTE_A4, NOTE_F4, NOTE_C5, NOTE_A4, NOTE_F4, NOTE_C5, NOTE_A4,
    NOTE_E5, NOTE_E5, NOTE_E5, NOTE_F5, NOTE_C5, NOTE_GS4, NOTE_F4, NOTE_C5, NOTE_A4,
    0 
};

const uint32_t Imperial_March_Durations[] = {
    QUARTER, QUARTER, QUARTER, EIGHTH, SIXTEENTH, QUARTER, EIGHTH, SIXTEENTH, HALF,
    QUARTER, QUARTER, QUARTER, EIGHTH, SIXTEENTH, QUARTER, EIGHTH, SIXTEENTH, HALF,
    0 
};

const uint32_t Tetris_Theme_Notes[] = {
    NOTE_E5, NOTE_B4, NOTE_C5, NOTE_D5, NOTE_C5, NOTE_B4, NOTE_A4, NOTE_A4, NOTE_C5, NOTE_E5, NOTE_D5, NOTE_C5, NOTE_B4, NOTE_B4, NOTE_C5, NOTE_D5, NOTE_E5, NOTE_C5, NOTE_A4, NOTE_A4, PAUSE,
    NOTE_D5, NOTE_F5, NOTE_A5, NOTE_G5, NOTE_F5, NOTE_E5, NOTE_E5, NOTE_C5, NOTE_E5, NOTE_D5, NOTE_C5, NOTE_B4, NOTE_B4, NOTE_C5, NOTE_D5, NOTE_E5, NOTE_C5, NOTE_A4, NOTE_A4,
    0 
};

const uint32_t Tetris_Theme_Durations[] = {
    QUARTER, EIGHTH,  EIGHTH,  QUARTER, EIGHTH,  EIGHTH,  QUARTER, EIGHTH,  EIGHTH,  QUARTER, EIGHTH,  EIGHTH,  DOTTED_QUARTER, EIGHTH, QUARTER, QUARTER, QUARTER, QUARTER, QUARTER, QUARTER, QUARTER,
    DOTTED_QUARTER, EIGHTH, QUARTER, EIGHTH,  EIGHTH,  DOTTED_QUARTER, EIGHTH, QUARTER, EIGHTH,  EIGHTH,  QUARTER, EIGHTH,  EIGHTH,  QUARTER, QUARTER, QUARTER, QUARTER, QUARTER, QUARTER,
    0 
};

const uint32_t Mario_Theme_Notes[] = {
    NOTE_E5, NOTE_E5, PAUSE,  NOTE_E5, PAUSE,  NOTE_C5, NOTE_E5, PAUSE, NOTE_G5, PAUSE, PAUSE, PAUSE, NOTE_G4, PAUSE, PAUSE, PAUSE,
    NOTE_C5, PAUSE,  PAUSE,  NOTE_G4, PAUSE,  PAUSE,  NOTE_E4, PAUSE,  PAUSE,  NOTE_A4, PAUSE, NOTE_B4, PAUSE, NOTE_AS4, NOTE_A4, PAUSE,
    NOTE_G4, NOTE_E5, NOTE_G5, NOTE_A5, PAUSE,  NOTE_F5, NOTE_G5, PAUSE,  NOTE_E5, PAUSE, NOTE_C5, NOTE_D5, NOTE_B4, PAUSE, PAUSE,
    0 
};

const uint32_t Mario_Theme_Durations[] = {
    EIGHTH, EIGHTH, EIGHTH, EIGHTH, EIGHTH, EIGHTH, QUARTER, EIGHTH, QUARTER, EIGHTH, QUARTER, EIGHTH, QUARTER, EIGHTH, QUARTER, EIGHTH,
    QUARTER, EIGHTH, EIGHTH, QUARTER, EIGHTH, EIGHTH, QUARTER, EIGHTH, EIGHTH, QUARTER, EIGHTH, QUARTER, EIGHTH, EIGHTH, QUARTER, EIGHTH,
    EIGHTH, EIGHTH, EIGHTH, QUARTER, EIGHTH, EIGHTH, QUARTER, EIGHTH, QUARTER, EIGHTH, EIGHTH, EIGHTH, QUARTER, EIGHTH, EIGHTH,
    0 
};

const uint32_t Tetris_Long_Notes[] = {
    NOTE_E5, NOTE_B4, NOTE_C5, NOTE_D5, NOTE_C5, NOTE_B4, NOTE_A4, NOTE_A4, NOTE_C5, NOTE_E5, NOTE_D5, NOTE_C5, NOTE_B4,
    NOTE_B4, NOTE_C5, NOTE_D5, NOTE_E5, NOTE_C5, NOTE_A4, NOTE_A4, PAUSE,
    
    NOTE_D5, NOTE_F5, NOTE_A5, NOTE_G5, NOTE_F5, NOTE_E5, NOTE_E5, NOTE_C5, NOTE_E5, NOTE_D5, NOTE_C5, NOTE_B4,
    NOTE_B4, NOTE_C5, NOTE_D5, NOTE_E5, NOTE_C5, NOTE_A4, NOTE_A4, PAUSE,

    NOTE_E4, NOTE_C4, NOTE_D4, NOTE_B3, NOTE_C4, NOTE_A3, NOTE_GS3, NOTE_B3, PAUSE,
    NOTE_E4, NOTE_C4, NOTE_D4, NOTE_B3, NOTE_E4, NOTE_A4, NOTE_GS4, PAUSE,
    
    NOTE_E5, NOTE_C5, NOTE_D5, NOTE_B4, NOTE_C5, NOTE_A4, NOTE_GS4, NOTE_B4,
    NOTE_E5, NOTE_C5, NOTE_D5, NOTE_B4, NOTE_E5, NOTE_A5, NOTE_GS5, PAUSE,
    
    0
};

const uint32_t Tetris_Long_Durations[] = {
    QUARTER, EIGHTH,  EIGHTH,  QUARTER, EIGHTH,  EIGHTH,  QUARTER, EIGHTH,  EIGHTH,  QUARTER, EIGHTH,  EIGHTH,  DOTTED_QUARTER,
    EIGHTH,  QUARTER, QUARTER, QUARTER, QUARTER, QUARTER, QUARTER, QUARTER,
    
    DOTTED_QUARTER, EIGHTH, QUARTER, EIGHTH,  EIGHTH,  DOTTED_QUARTER, EIGHTH, QUARTER, EIGHTH,  EIGHTH,  QUARTER, EIGHTH,
    EIGHTH,  QUARTER, QUARTER, QUARTER, QUARTER, QUARTER, QUARTER, QUARTER,

    HALF,    HALF,    HALF,    HALF,    HALF,    HALF,    HALF,    QUARTER, QUARTER,
    HALF,    HALF,    HALF,    HALF,    QUARTER, QUARTER, HALF,    HALF,
    
    QUARTER, QUARTER, QUARTER, QUARTER, QUARTER, QUARTER, QUARTER, QUARTER,
    QUARTER, QUARTER, QUARTER, QUARTER, QUARTER, QUARTER, HALF,    QUARTER,
    
    0 
};




