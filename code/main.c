#include "at32f421_clock.h"


//#define DEBUG
#define DEBUG_PCB
//#define DEBUG_AUTOROTATION



#ifndef DEBUG_PCB

#define PIN_MOTOR_4	GPIOA,GPIO_PINS_13	 
#define	PIN_MOTOR_1	GPIOA,GPIO_PINS_14	
#define PIN_OUT		GPIOA,GPIO_PINS_4		 
#define	PIN_OPTO  GPIOA,GPIO_PINS_2

#else

#define PIN_MOTOR_4	GPIOA,GPIO_PINS_0	 
#define	PIN_MOTOR_1	GPIOA,GPIO_PINS_1	


#define PIN_OUT	 GPIOA,GPIO_PINS_4		 
#define	PIN_OPTO  GPIOA,GPIO_PINS_2
#endif






#define OPTO_MIN 0x0400
#define OUT_MIN 0x0C00
//const  uint16_t OPTO_MIN   = 12000;
//const uint16_t OUT_MIN	   =45000;
const uint8_t	DELAY_COUNT       =3;
	
static union {
    uint32_t value;
    struct {

        unsigned MOTOR_FORWARD : 1;
        unsigned MOTOR_BACKWARD : 1;
        unsigned FUN_HIGH : 1;
        unsigned FUN_LOW : 1;
        unsigned ALLOW_MEASURE : 1;
        unsigned ALLOW_FUN : 1;
        unsigned MEASURING: 1;
        unsigned TARGET_POS_CLOSED: 1;

        unsigned TARGET_POS_OPENED: 1;
        unsigned OPENING : 1;
        unsigned OPENED : 1;
        unsigned CLOSING : 1;
        unsigned CLOSED : 1;
        unsigned RELAY_POWER_ON : 1;
        unsigned RELAY_CONTROL_ON : 1;
        unsigned TONE_ON : 1;

        unsigned TONE_OFF : 1;
        unsigned SIREN : 1;
        unsigned ZUM_BUSY : 1;
        unsigned MOVING_ALLOWED : 1;
        unsigned : 1;
        unsigned : 1;
        unsigned LED_ON : 1;
        unsigned SEC_LOCK : 1;

        unsigned AUTOROTATION_WORK : 1;
        unsigned MELODY_ON : 1;
        unsigned  LAST_BEEP_LONG: 1;
        unsigned  : 1;
        unsigned  : 1;
        unsigned  : 1;
        unsigned  : 1;
        unsigned  : 1;
    } bits;
} ff;

/*_____________________________________________________________________*/


uint64_t millis = 0 ;

/*_____________________________________________________________________*/




/*¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦*/
/*¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦*/

/*SERVICE*/



uint16_t adc_result_out;
uint16_t adc_result_opto;
char sensor_index;


void gpio_set(gpio_type *PORT, uint32_t PIN, gpio_drive_type DRIVE, gpio_mode_type MODE, gpio_output_type OUT_TYPE, gpio_pull_type PULL ) {

    gpio_init_type pinx;

    gpio_init_type *pina = &pinx;

    pinx.gpio_drive_strength= DRIVE;
    pinx.gpio_mode =MODE;
    pinx.gpio_out_type=OUT_TYPE;
    pinx.gpio_pins = PIN;
    pinx.gpio_pull = PULL;

    gpio_init( PORT,pina);

}

void mx_adc_clock_init(void){
  crm_adc_clock_div_set(CRM_ADC_DIV_2);
}

void timer_init() {

    nvic_irq_enable(TMR6_GLOBAL_IRQn,35,36);
    TMR6->iden_bit.ovfien =1;
    TMR6 ->ctrl1_bit.ocmen = 0;
    TMR6 ->ctrl1_bit.ovfen = 0;
    tmr_channel_buffer_enable(TMR6,TRUE);
    tmr_base_init(TMR6,1,500);
    tmr_counter_enable(TMR6,TRUE);
}


void hardware_init() {
	
	 /*
#ifndef DEBUG
    wdt_register_write_enable(TRUE);
    wdt_divider_set(WDT_CLK_DIV_8);
    wdt_register_write_enable(FALSE);
    wdt_enable();
    wdt_counter_reload();
#endif
     //*/ 
	
    crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK,TRUE);
	crm_periph_clock_enable(CRM_GPIOB_PERIPH_CLOCK,TRUE);
    crm_periph_clock_enable(CRM_ADC1_PERIPH_CLOCK,TRUE);
    crm_periph_clock_enable(CRM_TMR6_PERIPH_CLOCK,TRUE);
	
	gpio_set(PIN_MOTOR_4,
             GPIO_DRIVE_STRENGTH_MODERATE,
             GPIO_MODE_OUTPUT,
             GPIO_OUTPUT_PUSH_PULL,
             GPIO_PULL_DOWN);
			 
	gpio_set(PIN_MOTOR_1,
             GPIO_DRIVE_STRENGTH_MODERATE,
             GPIO_MODE_OUTPUT,
             GPIO_OUTPUT_PUSH_PULL,
             GPIO_PULL_DOWN);
			 
	gpio_set(PIN_OUT,
             GPIO_DRIVE_STRENGTH_MODERATE,
             GPIO_MODE_ANALOG,
             GPIO_OUTPUT_OPEN_DRAIN,
             GPIO_PULL_NONE);
			 
	gpio_set(PIN_OPTO,
             GPIO_DRIVE_STRENGTH_MODERATE,
             GPIO_MODE_ANALOG,
             GPIO_OUTPUT_OPEN_DRAIN,
             GPIO_PULL_NONE);
			 
			 
			 timer_init();

    nvic_irq_enable(ADC1_CMP_IRQn,37,38);
	
	adc_base_config_type *adc1;
    adc_base_default_para_init(adc1);
    adc_base_config(ADC1,adc1);

    adc_enable(ADC1,TRUE);
    adc_interrupt_enable(ADC1,ADC_CCE_INT,TRUE);
    adc_ordinary_channel_set(ADC1,ADC_CHANNEL_2,1,ADC_SAMPLETIME_239_5);
	
}





void ms_tick() {

    static uint64_t ms1000_count = 0;

	
	if (sensor_index == 1)
            {
                adc_ordinary_channel_set(ADC1,ADC_CHANNEL_4,1,ADC_SAMPLETIME_239_5);
				   sensor_index = 0;
				adc_ordinary_conversion_trigger_set(ADC1,ADC12_ORDINARY_TRIG_SOFTWARE,TRUE);
              

            }
            else
            {
                adc_ordinary_channel_set(ADC1,ADC_CHANNEL_2,1,ADC_SAMPLETIME_239_5);
				 sensor_index = 1;
				 adc_ordinary_conversion_trigger_set(ADC1,ADC12_ORDINARY_TRIG_SOFTWARE,TRUE);
                
            }
	


}

 void TMR6_GLOBAL_IRQHandler(void) {

    static char i =0;
    ++i;

    if (i>=8) {
        ms_tick();
        i=0;
    }

    tmr_period_value_set(TMR6,1);
    TMR6 ->ists_bit.ovfif =0;
}
 

void ADC1_CMP_IRQHandler(void) {
  
      if(sensor_index==0)
    {
        adc_result_out	= adc_ordinary_conversion_data_get(ADC1);
    }
    else
    {
        adc_result_opto	= adc_ordinary_conversion_data_get(ADC1);
    }
}


void hardware_work() {
	
static u8 count;	
	
	if  (
		((adc_result_opto <= OPTO_MIN) && (adc_result_out <= OUT_MIN))
				|| 
			((adc_result_opto >= OPTO_MIN) && (adc_result_out >= OUT_MIN)) 
)	//*/
			    
		
{
	
     if (count<= DELAY_COUNT) 	 ++count;
	
}     else
{
	if (count>0 ) --count;
}
	
	
	if (count>= DELAY_COUNT )
		{
		
		if (ff.bits.MOTOR_FORWARD)
			{  
				gpio_bits_reset(PIN_MOTOR_4);
				gpio_bits_set(PIN_MOTOR_1);
				
			}
		else if (ff.bits.MOTOR_BACKWARD)
			{
				gpio_bits_reset(PIN_MOTOR_1);
				gpio_bits_set(PIN_MOTOR_4);
			}
		} 
		else
			{
				gpio_bits_reset (PIN_MOTOR_1);
				gpio_bits_reset (PIN_MOTOR_4);
			}

}


void start_setup() {
    
	system_clock_config();

	mx_adc_clock_init(); 
	
	hardware_init(); // initialize the device
   
    ff.value = 0;
   

}
/*¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦*/


/*opening

  
black->white


out ffff

//*/
int main(void) {

start_setup();
   
  ff.bits.MOTOR_FORWARD =1;
  while(1)
  {  
	wdt_counter_reload();  
	hardware_work();  
	 
  }
}

