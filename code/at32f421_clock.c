/* includes ------------------------------------------------------------------*/
#include "at32f421_clock.h"

/**
  * @brief  system clock config program
  * @note   the system clock is configured as follow:
  *         - system clock        = (hick 8)
  *         - system clock source = hick
  *         - sclk                = 8000000
  *         - ahbdiv              = 1
  *         - ahbclk              = 8000000
  *         - apb1div             = 1
  *         - apb1clk             = 8000000
  *         - apb2div             = 1
  *         - apb2clk             = 8000000
  *         - flash_wtcyc         = 0 cycle
  * @param  none
  * @retval none
  */
void system_clock_config(void)
{
  /* config flash psr register */
  flash_psr_set(FLASH_WAIT_CYCLE_0);

 
  /* reset crm */
  crm_reset();

  /* enable hick */
  crm_clock_source_enable(CRM_CLOCK_SOURCE_HICK, TRUE);

   /* wait till hick is ready */
  while(crm_flag_get(CRM_HICK_STABLE_FLAG) != SET)
  {
  }

  /* config ahbclk */
  crm_ahb_div_set(CRM_AHB_DIV_1);

  /* config apb2clk */
  crm_apb2_div_set(CRM_APB2_DIV_1);

  /* config apb1clk */
  crm_apb1_div_set(CRM_APB1_DIV_1);

  /* select hick as system clock source */
  crm_sysclk_switch(CRM_SCLK_HICK);

  /* wait till hick is used as system clock source */
  while(crm_sysclk_switch_status_get() != CRM_SCLK_HICK)
  {
  }

  /* update system_core_clock global variable */
  system_core_clock_update();
}
