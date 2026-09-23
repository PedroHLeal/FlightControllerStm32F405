#include "timer.h"

uint32_t last_cyccnt = 0;

void dwt_init(void) {
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	DWT->CYCCNT = 0;
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

float dwt_dt_s(uint32_t cpu_hz) {
	uint32_t now = DWT->CYCCNT;
	uint32_t delta = now - last_cyccnt;
	last_cyccnt = now;
	return (float) delta / (float) cpu_hz;
}
