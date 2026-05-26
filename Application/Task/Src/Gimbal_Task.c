/* Includes ------------------------------------------------------------------*/
#include "cmsis_os.h"
#include "gimbal_control.h"
#include "state_machine.h"
#include "lidar_lifter_control.h"

void Gimbal_Task(void const * argument)
{
    /* Keep the task loop at 1ms (1000Hz) so our split halves result in 500Hz */
    const TickType_t xPeriod = 1 / portTICK_PERIOD_MS;
	
    for(;;)
    {
				robot_states_update_state_machine();
        control_loop_gimbal();
				control_loop_lidar_lifter();
        vTaskDelay(xPeriod); // Wait 1ms
    }
}