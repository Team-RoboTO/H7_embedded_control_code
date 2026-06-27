#include "power_estimation.h"
#include "robot_config.h"
#include "cubemars_motor.h"
#include "LPF.h"

// Costanti fisiche e parametri calibrati con MATLAB
const float KT_OUT         = 0.056f * 10.0f;      // KT * Rapporto di riduzione = 0.56
const float K1_JOULE       = 0.733f;              // Perdite nel rame (0.75 * R_phase_to_phase)
const float K2_IRON        = 0.0094727f;          // Attrito viscoso ottimizzato
const float P0_STATIC      = 1.8125f;             // Consumo statico in standby

#if IS_STD || IS_SENTRY
const float CHASSIS_POWER_SCALE = 0.40692f;       // Fattore di scala globale calcolato
#elif IS_HERO
const float CHASSIS_POWER_SCALE = 0.7f;//         // Fattore di scala globale calcolato
#endif

// Strutture per Filtro Passa Basso (LPF)
LowPassFilter1p_Info_TypeDef Torque1_LPF1p;
LowPassFilter1p_Info_TypeDef Torque2_LPF1p;
LowPassFilter1p_Info_TypeDef Torque3_LPF1p;
LowPassFilter1p_Info_TypeDef Torque4_LPF1p;

// Buffer per filtro mediano
static float torque_prev1[4] = {0};
static float torque_prev2[4] = {0};

// Stato del governor — mantenuto tra i cicli
static float alpha_governor = 1.0f;

// Variabili globali di telemetria
float estimated_total_power = 0.0f;
float values[4];
bool is_first_iter = true;

static float MIT_kd_base = 0.2f;


// Funzione ausiliaria per filtro mediano a 3 elementi
static float median3(float a, float b, float c) {
    if (a > b) { float t = a; a = b; b = t; }
    if (b > c) { float t = b; b = c; c = t; }
    if (a > b) { float t = a; a = b; b = t; }
    return b;
}


/**
 * @brief Algoritmo di stima e limitazione di potenza dello chassis
 *
 * Strategia di taglio (MIT mode):
 *   - Si scala sia il riferimento di velocità r_x[i] che il kd del motore
 *     per lo stesso fattore alpha_governor ∈ [0, 1].
 *   - In questo modo la coppia massima erogabile scala con alpha², mentre
 *     con il solo taglio di ω_ref scalava con alpha (meno efficace).
 *   - Il recovery è proporzionale al margine disponibile per evitare
 *     chattering (oscillazione alpha su/giù a ogni ciclo).
 *
 * @param limit  Limite di potenza imposto dal Referee [W]
 * @param r_x    Array delle 4 velocità angolari target [rad/s] — modificato in-place
 */
void chassis_power_control(uint16_t limit, float *r_x, float *mit_kd)
{
    float chassis_power_limit = (float)limit;
    float estimated_give_power[4] = {0};

    /***********************************************************
     * 0. INIZIALIZZAZIONE FILTRI (solo al primo ciclo)
     ***********************************************************/
    if (is_first_iter) {
        LowPassFilter1p_Init(&Torque1_LPF1p, 0.90f);
        LowPassFilter1p_Init(&Torque2_LPF1p, 0.90f);
        LowPassFilter1p_Init(&Torque3_LPF1p, 0.90f);
        LowPassFilter1p_Init(&Torque4_LPF1p, 0.90f);
        is_first_iter = false;
    }

    LowPassFilter1p_Info_TypeDef *lpf_array[4] = {
        &Torque1_LPF1p, &Torque2_LPF1p, &Torque3_LPF1p, &Torque4_LPF1p
    };

    /***********************************************************
     * 1. STIMA POTENZA
     ***********************************************************/
    estimated_total_power = 0.0f;

    for (int8_t i = 0; i < 4; i++) {

        float raw_current = CM_Chassis_Motor[i].Data.Torque;
        float velocity    = CM_Chassis_Motor[i].Data.Velocity;

        // Fase A: rimozione spike tramite mediana a 3 campioni
        float deglitched_current = median3(torque_prev2[i], torque_prev1[i], raw_current);
        torque_prev2[i] = torque_prev1[i];
        torque_prev1[i] = raw_current;

        // Fase B: filtro passa basso 1° ordine
        float filtered_current = LowPassFilter1p_Update(lpf_array[i], deglitched_current);

        // Modello fisico calibrato
        float p_mech    = (filtered_current * KT_OUT) * velocity;
        float p_joule   = K1_JOULE * filtered_current * filtered_current;
        float p_viscous = K2_IRON  * velocity * velocity;

        estimated_give_power[i] = ((p_mech + p_joule + p_viscous) * CHASSIS_POWER_SCALE) + P0_STATIC;

        // Clamping anti-rigenerazione
        if (estimated_give_power[i] < 0.0f)
            estimated_give_power[i] = 0.0f;

        estimated_total_power += estimated_give_power[i];
    }

    
    /***********************************************************
     * 3. CALCOLO ALPHA_GOVERNOR
     *
     * Discesa: istantanea e aggressiva se c'è sforo.
     * Risalita: proporzionale al margine disponibile per evitare
     *           chattering. Più sei vicino al limite, più risali piano.
     ***********************************************************/
    if (estimated_total_power > chassis_power_limit) {

        // Taglio istantaneo proporzionale allo sforo
        float instantaneous_scale = chassis_power_limit / estimated_total_power;
        if (instantaneous_scale < alpha_governor)
            alpha_governor = instantaneous_scale;

    } else {

        // Recovery proporzionale al margine: da +0.5%/ciclo (vicino al limite)
        // a +2%/ciclo (lontano dal limite)
        float headroom   = 1.0f - (estimated_total_power / chassis_power_limit);
        float recovery   = 0.005f + 0.015f * headroom;
        alpha_governor  += recovery;
    }

    // Clamp [0, 1]
    if (alpha_governor > 1.0f) alpha_governor = 1.0f;
    if (alpha_governor < 0.0f) alpha_governor = 0.0f;

    /***********************************************************
     * 4. APPLICAZIONE DEL TAGLIO (MIT mode)
     *
     * Si scala sia ω_ref che kd dello stesso fattore alpha_governor.
     * La coppia massima erogabile risultante scala con alpha²:
     *   τ ≈ kd * (ω_ref - ω)
     *     = (kd * α) * (ω_ref * α - ω)
     * → effetto di taglio molto più diretto rispetto al solo ω_ref.
     ***********************************************************/
    for (uint8_t i = 0; i < 4; i++) {
        r_x[i]                        *= alpha_governor;
        *mit_kd = MIT_kd_base * alpha_governor;
    }
}