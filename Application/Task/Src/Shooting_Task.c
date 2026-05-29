/* Includes ------------------------------------------------------------------*/
#include "cmsis_os.h"
#include "shooting_control.h"
#include "state_machine.h"
#include "buzzer.h"

void Shooting_Task(void const * argument)
{
    /* Keep the task loop at 1ms (1000Hz) so our split halves result in 500Hz */
    const TickType_t xPeriod = 1 / portTICK_PERIOD_MS;
    Buzzer_PlayMelody(Mario_Theme_Notes, Mario_Theme_Durations);
    for(;;)
    {	
				robot_states_update_state_machine();
        control_loop_shooting();
        vTaskDelay(xPeriod); // Wait 1ms
    }
}