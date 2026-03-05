@echo off
copy /Y ".\correct_files\portmacro.h" ".\Middlewares\Third_Party\FreeRTOS\Source\portable\RVDS\ARM_CM4F\portmacro.h"
copy /Y ".\correct_files\port.c" ".\Middlewares\Third_Party\FreeRTOS\Source\portable\RVDS\ARM_CM4F\port.c"