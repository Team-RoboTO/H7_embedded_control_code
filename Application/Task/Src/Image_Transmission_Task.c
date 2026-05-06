/* Includes ------------------------------------------------------------------*/
#include "cmsis_os.h"
#include "Image_Transmission.h"


void Image_Transmission_Task(void const * argument)
{
    /* Keep the task loop at 1ms (1000Hz) so our split halves result in 500Hz */
    const TickType_t xPeriod = 100 / portTICK_PERIOD_MS;
    
    for(;;)
    {
        //uint8_t dummy_data[30] = {0};
        // Robot_Data_to_Custom_(dummy_data); // Commentato per evitare conflitti di banda con la UI
        vTaskDelay(xPeriod); // Wait 100ms (as defined by xPeriod)
    }
}