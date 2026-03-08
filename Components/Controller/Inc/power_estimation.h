#ifndef POWER_ESTIMATION_H
#define POWER_ESTIMATION_H

#include "controlled_system.h"

/**
 * @brief Variabili di debug per monitoraggio power consumption
 * 
 * db_power_out: Potenza in uscita stimata    [W]
 * db_power_in:  Potenza in ingresso misurata [W]
 */
extern float db_power_out;
extern float db_power_in;

void chassis_power_control(uint16_t level, float *u);

#endif
