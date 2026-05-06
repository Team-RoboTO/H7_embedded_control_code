/* Includes ------------------------------------------------------------------*/
#include "cmsis_os.h"
#include "Chassis_control.h"
#include "state_machine.h"
#include "INA228.h"

void Chassis_Task(void const * argument)
{
    /* Keep the task loop at 1ms (1000Hz) so our split halves result in 500Hz */
    const TickType_t xPeriod = 1 / portTICK_PERIOD_MS;
		
//		while(INA228_Init())										   	// init of the INA226 peripheral
//			{
//        osDelay(100);
//			}
			
    for(;;)
    {
				robot_states_update_state_machine();
        control_loop_chassis();
        vTaskDelay(xPeriod); // Wait 1ms
    }
	}