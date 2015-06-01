#include "therm.h"
#include "stm32f411xe.h"
#include "pin.h"
#include <math.h>

// thermistor bridge is connected to PORTB pin 0, which is ADC input 8
#define PORTB_THERM 0

static float g_therm_filtered = 0;

void therm_init()
{
  pin_set_analog(GPIOB, PORTB_THERM);
  RCC->APB2ENR |= RCC_APB2ENR_ADC1EN; // turn on ADC clock gate
  //ADC1->CR1
  ADC->CCR = ADC_CCR_ADCPRE; // set ADC clock to PCLK2 / 8 = 24 / 8 = 3 MHz
  ADC1->CR2 = ADC_CR2_CONT | ADC_CR2_ADON;
  ADC1->SQR3 = 8; // sequence register 0 = ADC8 = pin B8
  ADC1->SMPR2 = 7; // set the slowest ADC sample rate possible (480 clocks)
  ADC1->CR2 |= ADC_CR2_SWSTART; // kick off the first conversion
}

#define THERM_FILTER_GAIN 0.95

float therm_celsius()
{
  const uint16_t raw = ADC1->DR;
  const float therm_volts = raw * 3.3f / 4095.0f;
  //const float therm_r_meas = //(3.3f / therm_volts - 1) * 10000.0f - 2.0f*15000.0f;
  
                             //10000.0f / (3.3f / therm_volts/* - 1.0f*/) - 
                             //2.0f * 15000.0f; 
  const float VA = 3.3f, VB = therm_volts;
  const float R1 = 10000.0f, R2 = 10000.0f, R4 = 10000.0f;
  const float therm_r_meas = (VA*(R2+R4) - VB*(R1+R2+R4)) / (VB - VA);
  if (therm_r_meas < 0)
    return -99; // brrrr that's cold
  const float THERM_B = 4288.0f;
  const float THERM_R_25C = 50000.0f;
  if (therm_volts > 0.01f)
  {
    // ewww math
    float c = THERM_B / logf(therm_r_meas /
                             (THERM_R_25C * expf(-THERM_B / 298.15f))) -
              273.15f; // last subtraction is to convert kelvin to celsius
    if (g_therm_filtered == 0)
      g_therm_filtered = c;
    else
      g_therm_filtered =         THERM_FILTER_GAIN  * g_therm_filtered +
                         (1.0f - THERM_FILTER_GAIN) * c;
    return g_therm_filtered;
  }
  else return 0;
}

