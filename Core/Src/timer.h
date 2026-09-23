#ifndef TIMER_H
#define TIMER_H

#include "main.h"

void dwt_init();
float dwt_dt_s(uint32_t cpu_hz);

#endif
