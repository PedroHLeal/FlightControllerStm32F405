#include "battery.h"
#include "main.h"

static const uint32_t CH_VOLT = 10, CH_CURR = 12;
static uint32_t activeChannel = CH_VOLT;
static float voltage = 0, current = 0;
static float sampledMs = 0;
static bool started = false;

static void startConversion(uint32_t channel) {
	activeChannel = channel;
	ADC1->SQR3 = channel;
	ADC1->CR2 |= ADC_CR2_SWSTART;
}

void batteryInit() {
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;

	GPIOC->MODER |= (3u << (0 * 2)) | (3u << (2 * 2));   // PC0, PC2 analog

	ADC->CCR = (ADC->CCR & ~ADC_CCR_ADCPRE) | ADC_CCR_ADCPRE_0;   // PCLK2/4 = 12 MHz
	ADC1->CR1 = 0;                                               // 12-bit, single
	ADC1->SMPR1 = (7u << 0) | (7u << 6);                         // 480 cycles on IN10, IN12
	ADC1->SQR1 = 0;                                              // one conversion
	ADC1->CR2 = ADC_CR2_ADON;
	for (volatile int i = 0; i < 1000; i++)
		;
	startConversion(CH_VOLT);
	started = true;
}

void batteryUpdate(float dt) {
	if (!started || !(ADC1->SR & ADC_SR_EOC))
		return;

	float volts = (ADC1->DR / 4095.0f) * 3.3f;   // reading DR clears EOC
	float alpha = dt / (VBAT_FILTER_TAU + dt);
	if (activeChannel == CH_VOLT) {
		float v = volts * VBAT_DIVIDER * VBAT_CAL;
		voltage += (v - voltage) * (sampledMs < 100 ? 1.0f : alpha);   // snap during the first samples
		startConversion(CH_CURR);
	} else {
		float a = volts * IBAT_AMP_PER_VOLT;
		current += (a - current) * (sampledMs < 100 ? 1.0f : alpha);
		startConversion(CH_VOLT);
	}
	if (sampledMs < 1e6f)
		sampledMs += dt * 1000.0f;
}

float batteryVoltage() {
	return voltage;
}

float batteryCurrent() {
	return current;
}

bool batteryValid() {
	return sampledMs >= 1000.0f && voltage > 6.0f && voltage < 26.0f;
}

float batterySagCompensation() {
	if (!VBAT_SAG_COMP || !batteryValid())
		return 1.0f;
	float comp = VBAT_REF_V / voltage;
	if (comp < VBAT_SAG_COMP_MIN)
		comp = VBAT_SAG_COMP_MIN;
	if (comp > VBAT_SAG_COMP_MAX)
		comp = VBAT_SAG_COMP_MAX;
	return comp;
}
