#ifndef __MAIN_H
#define __MAIN_H
#include "at32f4xx.h"
#include <stdlib.h>
#include <stdio.h>
#include "uart_manager.h"


SemaphoreHandle_t xsemaphore_recf1_get(void);
SemaphoreHandle_t xsemaphore_recf2_get(void);


#endif