#ifndef _COMPASS_H
#define _COMPASS_H

#include <stdint.h>
#include "main.h"

extern I2C_HandleTypeDef hi2c1;

void data_ready_interrupt(TimerHandle_t xTimer);

void compass_init(void);

#endif /* _COMPASS_H */