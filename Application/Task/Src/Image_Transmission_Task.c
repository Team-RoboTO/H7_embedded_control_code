/* Includes ------------------------------------------------------------------*/
#include "cmsis_os.h"
#include "Image_Transmission.h"


void Image_Transmission_Task(void const * argument)
{
    /* Keep the task loop at 1ms (1000Hz) so our split halves result in 500Hz */
    const TickType_t xPeriod = 100 / portTICK_PERIOD_MS;
    
    for(;;)
    {
        //Robot_Data_to_Custom_(uint8_t *Data);
        vTaskDelay(xPeriod); // Wait 1ms
    }
}