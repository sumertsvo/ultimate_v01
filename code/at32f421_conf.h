/* define to prevent recursive inclusion -------------------------------------*/
#ifndef __AT32F421_CONF_H
#define __AT32F421_CONF_H

#ifdef __cplusplus
extern "C" {
#endif
  
/**
  * @brief in the following line adjust the value of high speed exernal crystal (hext)
  * used in your application
  *
  * tip: to avoid modifying this file each time you need to use different hext, you
  *      can define the hext value in your toolchain compiler preprocessor.
  *
  */
#if !defined  HEXT_VALUE
#define HEXT_VALUE               ((uint32_t)8000000) /*!< value of the high speed exernal crystal in hz */
#endif

/**
  * @brief in the following line adjust the high speed exernal crystal (hext) startup
  * timeout value
  */
#define HEXT_STARTUP_TIMEOUT     ((uint16_t)0x3000) /*!< time out for hext start up */
#define HICK_VALUE               ((uint32_t)8000000) /*!< value of the high speed internal clock in hz */

/* module define -------------------------------------------------------------*/
#define CRM_MODULE_ENABLED
#define CMP_MODULE_ENABLED
#define TMR_MODULE_ENABLED
#define ERTC_MODULE_ENABLED
#define GPIO_MODULE_ENABLED
#define I2C_MODULE_ENABLED
#define USART_MODULE_ENABLED
#define PWC_MODULE_ENABLED
#define ADC_MODULE_ENABLED
#define SPI_MODULE_ENABLED
#define DMA_MODULE_ENABLED
#define DEBUG_MODULE_ENABLED
#define FLASH_MODULE_ENABLED
#define CRC_MODULE_ENABLED
#define WWDT_MODULE_ENABLED
#define WDT_MODULE_ENABLED
#define EXINT_MODULE_ENABLED
#define MISC_MODULE_ENABLED
#define SCFG_MODULE_ENABLED

/* includes ------------------------------------------------------------------*/
#ifdef CRM_MODULE_ENABLED
#include "at32f421_crm.h"
#endif
#ifdef CMP_MODULE_ENABLED
#include "at32f421_cmp.h"
#endif
#ifdef TMR_MODULE_ENABLED
#include "at32f421_tmr.h"
#endif
#ifdef ERTC_MODULE_ENABLED
#include "at32f421_ertc.h"
#endif
#ifdef GPIO_MODULE_ENABLED
#include "at32f421_gpio.h"
#endif
#ifdef I2C_MODULE_ENABLED
#include "at32f421_i2c.h"
#endif
#ifdef USART_MODULE_ENABLED
#include "at32f421_usart.h"
#endif
#ifdef PWC_MODULE_ENABLED
#include "at32f421_pwc.h"
#endif
#ifdef ADC_MODULE_ENABLED
#include "at32f421_adc.h"
#endif
#ifdef SPI_MODULE_ENABLED
#include "at32f421_spi.h"
#endif
#ifdef DMA_MODULE_ENABLED
#include "at32f421_dma.h"
#endif
#ifdef DEBUG_MODULE_ENABLED
#include "at32f421_debug.h"
#endif
#ifdef FLASH_MODULE_ENABLED
#include "at32f421_flash.h"
#endif
#ifdef CRC_MODULE_ENABLED
#include "at32f421_crc.h"
#endif
#ifdef WWDT_MODULE_ENABLED
#include "at32f421_wwdt.h"
#endif
#ifdef WDT_MODULE_ENABLED
#include "at32f421_wdt.h"
#endif
#ifdef EXINT_MODULE_ENABLED
#include "at32f421_exint.h"
#endif
#ifdef MISC_MODULE_ENABLED
#include "at32f421_misc.h"
#endif
#ifdef SCFG_MODULE_ENABLED
#include "at32f421_scfg.h"
#endif

#ifdef __cplusplus
}
#endif

#endif
