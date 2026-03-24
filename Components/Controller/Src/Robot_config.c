#include "robot_config.h"

  /*****************************/
 /*   CHECK ROBOT SELECTION   */
/*****************************/

#if IS_ROBOT_CONFIG_WRONG
#error "Robot type configuration is wrong"
#endif

#if CONFLICT_EVENT_SHOOTING
#error "Conflict between event mode and shooting wheels on"

#endif
