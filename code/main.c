#include "at32f421_clock.h"

/**
  * @brief  init adc clock function
  * @param  none
  * @retval none
  */
void mx_adc_clock_init(void)
{
  crm_adc_clock_div_set(CRM_ADC_DIV_2);
}

/**
  * @brief  main function.
  * @param  none
  * @retval none
  */
int main(void)
{
  system_clock_config();

  mx_adc_clock_init();

  while(1)
  {
  }
}

