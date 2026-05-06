@echo off
copy /Y ".\PORTMACRO\portmacro.h" ".\Middlewares\Third_Party\FreeRTOS\Source\portable\RVDS\ARM_CM4F\portmacro.h"
copy /Y ".\PORTMACRO\port.c" ".\Middlewares\Third_Party\FreeRTOS\Source\portable\RVDS\ARM_CM4F\port.c"