/**
* \file  main.c
*
* \brief LORAWAN Demo Application main file
*		
*
* Copyright (c) 2018 Microchip Technology Inc. and its subsidiaries. 
*
* \asf_license_start
*
* \page License
*
* Subject to your compliance with these terms, you may use Microchip
* software and any derivatives exclusively with Microchip products. 
* It is your responsibility to comply with third party license terms applicable 
* to your use of third party software (including open source software) that 
* may accompany Microchip software.
*
* THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS".  NO WARRANTIES, 
* WHETHER EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, 
* INCLUDING ANY IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, 
* AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT WILL MICROCHIP BE 
* LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, INCIDENTAL OR CONSEQUENTIAL 
* LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND WHATSOEVER RELATED TO THE 
* SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS BEEN ADVISED OF THE 
* POSSIBILITY OR THE DAMAGES ARE FORESEEABLE.  TO THE FULLEST EXTENT 
* ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN ANY WAY 
* RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY, 
* THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
*
* \asf_license_stop
*
*/
/*
* Support and FAQ: visit <a href="https://www.microchip.com/support/">Microchip Support</a>
*/
 
/****************************** INCLUDES **************************************/
#include "system_low_power.h"
#include "radio_driver_hal.h"
#include "lorawan.h"
#include "sys.h"
#include "system_init.h"
#include "system_assert.h"
#include "aes_engine.h"
#include "enddevice_demo.h"
#include "sio2host.h"
#include "extint.h"
#include "conf_app.h"
#include "sw_timer.h"
#ifdef CONF_PMM_ENABLE
#include "pmm.h"
#include  "conf_pmm.h"
#include "sleep_timer.h"
#include "sleep.h"
#endif
#include "conf_sio2host.h"
#if (ENABLE_PDS == 1)
#include "pds_interface.h"
#endif
#if (CERT_APP == 1)
#include "conf_certification.h"
#include "enddevice_cert.h"
#endif
#include "sal.h"
/************************** Macro definition ***********************************/
/* Button debounce time in ms */
#define APP_DEBOUNCE_TIME       50
#define ADC_SAMPLES 3
/************************** Global variables ***********************************/
bool button_pressed = false;
bool factory_reset = false;
bool bandSelected = false;
uint32_t longPress = 0;
uint8_t demoTimerId = 0xFF;
uint8_t lTimerId = 0xFF;
extern bool certAppEnabled;
#ifdef CONF_PMM_ENABLE
bool deviceResetsForWakeup = false;
#endif

struct adc_module adc_instance;


/************************** Extern variables ***********************************/

/************************** Function Prototypes ********************************/
static void driver_init(void);
static void	setup_ADC(void);
static void config_board(void);

#if (_DEBUG_ == 1)
static void assertHandler(SystemAssertLevel_t level, uint16_t code);
#endif /* #if (_DEBUG_ == 1) */

/*********************************************************************//**
 \brief      Uninitializes app resources before going to low power mode
*************************************************************************/
#ifdef CONF_PMM_ENABLE
static void app_resources_uninit(void);
#endif

/****************************** FUNCTIONS **************************************/

static void print_reset_causes(void)
{
    enum system_reset_cause rcause = system_get_reset_cause();
    printf("Last reset cause: ");
    if(rcause & (1 << 6)) {
        printf("System Reset Request\r\n");
    }
    if(rcause & (1 << 5)) {
        printf("Watchdog Reset\r\n");
    }
    if(rcause & (1 << 4)) {
        printf("External Reset\r\n");
    }
    if(rcause & (1 << 2)) {
        printf("Brown Out 33 Detector Reset\r\n");
    }
    if(rcause & (1 << 1)) {
        printf("Brown Out 12 Detector Reset\r\n");
    }
    if(rcause & (1 << 0)) {
        printf("Power-On Reset\r\n");
    }
}

#ifdef CONF_PMM_ENABLE
static void appWakeup(uint32_t sleptDuration)
{
    HAL_Radio_resources_init();
    sio2host_init();
    printf("\r\nsleep_ok %ld ms\r\n", sleptDuration);

}
#endif

#if (_DEBUG_ == 1)
static void assertHandler(SystemAssertLevel_t level, uint16_t code)
{
    printf("\r\n%04x\r\n", code);
    (void)level;
}
#endif /* #if (_DEBUG_ == 1) */

/**
 * \mainpage
 * \section preface Preface
 * This is the reference manual for the LORAWAN Demo Application of EU Band
 */
int main(void)
{
	
    /* System Initialization */
    system_init();
    /* Initialize the delay driver */
    delay_init();
    /* Initialize the board target resources */
    board_init();

    INTERRUPT_GlobalInterruptEnable();
	/* Initialize the Serial Interface */
	sio2host_init();
#ifndef CRYPTO_DEV_ENABLED
 	/* Read DEV EUI from EDBG */
    dev_eui_read();
#endif
    /* Initialize Hardware and Software Modules */
	driver_init();
	
    delay_ms(5);
    print_reset_causes();
#if (_DEBUG_ == 1)
    SYSTEM_AssertSubscribe(assertHandler);
#endif
    /* Initialize demo application */
    Stack_Init();

    SwTimerCreate(&demoTimerId);
    SwTimerCreate(&lTimerId);

    mote_demo_init();
	setup_ADC();   //Temperature
	config_board();
	
	//Test code to avoid Lorawan pairing
	//while(1){
//
		//uint16_t adc_buffer[ADC_SAMPLES];
		//
		//unsigned char temp_upper_byte = 0;  //PA06
		//unsigned char temp_lower_byte = 0;  //PA06
		//unsigned char vBatt_upper_byte = 0;  //PA07
		//unsigned char vBatt_lower_byte = 0;  //PA07
		//unsigned char PA08_upper_byte = 0;  //PA08
		//unsigned char PA08_lower_byte = 0;  //PA08
					//
		////Prepare the buffer arrays
		//for (int n=0;n<ADC_SAMPLES;n++){
			//adc_buffer[n] = 0;
//
		//}
		////Get ADC voltages
		//
		////Set Battery Voltage measurement sink
		////port_pin_set_output_level(PIN_PB03, false);  //v0,5 will be PB03
		//port_pin_set_output_level(PIN_PA15, false);  
		////Set VSen on
		//port_pin_set_output_level(PIN_PA27, true);  
		//
		////Does there need to be a delay here to allow VSEN to settle and sensors to turn on?
		//
		////adc_start_conversion(&adc_instance); //Not needed as it happens in adc_read_buffer_job()
		//adc_read_buffer_job(&adc_instance, adc_buffer, ADC_SAMPLES);
		////Make sure the job is finished.
		//while(adc_get_job_status(&adc_instance, ADC_JOB_READ_BUFFER ) == STATUS_BUSY){
		//}
		//
		////Remove Battery Voltage measurement sink
		////port_pin_set_output_level(PIN_PB03, true);//v0,5 will be PB03
		//port_pin_set_output_level(PIN_PA15, true); 
		////Set VSen off
		//port_pin_set_output_level(PIN_PA27, false);  
	//
		////Setup data for LoRaWAN transmission
		////Conversion from 16bit ADC to float done in payload decoder, or Node-Red.		
		//temp_lower_byte = adc_buffer[0] & 0xFF;
		//temp_upper_byte = (adc_buffer[0] & 0xFF00)>>8;
		//
		//vBatt_lower_byte = adc_buffer[1] & 0xFF;
		//vBatt_upper_byte = (adc_buffer[1] & 0xFF00)>>8;
		//
		////This input isn't working correctly yet.  Needs to be working for battery measurement.
		//PA08_lower_byte = adc_buffer[2] & 0xFF;
		//PA08_upper_byte = (adc_buffer[2] & 0xFF00)>>8;
		//
		//////snprintf(temp_sen_str,sizeof(temp_sen_str),"%c%c%c%c\n", temp_lower_byte, temp_upper_byte, vBatt_lower_byte, vBatt_upper_byte);
		//printf("\rn%c%c,%c%c,%c%c\rn", temp_lower_byte, temp_upper_byte, vBatt_lower_byte, vBatt_upper_byte, PA08_lower_byte, PA08_upper_byte);
//
	//}
	
	
	
	
    while (1)
    {
		serial_data_handler();
        SYSTEM_RunTasks();
#ifdef CONF_PMM_ENABLE
        if (false == certAppEnabled)
        {
            if(bandSelected == true)
            {
                PMM_SleepReq_t sleepReq;
                /* Put the application to sleep */
                sleepReq.sleepTimeMs = DEMO_CONF_DEFAULT_APP_SLEEP_TIME_MS;
                sleepReq.pmmWakeupCallback = appWakeup;
                sleepReq.sleep_mode = CONF_PMM_SLEEPMODE_WHEN_IDLE;
                if (CONF_PMM_SLEEPMODE_WHEN_IDLE == SLEEP_MODE_STANDBY)
                {
                    deviceResetsForWakeup = false;
                }
                if (true == LORAWAN_ReadyToSleep(deviceResetsForWakeup))
                {
                    app_resources_uninit();
                    if (PMM_SLEEP_REQ_DENIED == PMM_Sleep(&sleepReq))
                    {
                        HAL_Radio_resources_init();
                        sio2host_init();
                        /*printf("\r\nsleep_not_ok\r\n");*/
                    }
                }
            }
        }
#endif
    }
}

/* Initializes all the hardware and software modules used for Stack operation */
static void driver_init(void)
{
	SalStatus_t sal_status = SAL_SUCCESS;
    /* Initialize the Radio Hardware */
    HAL_RadioInit();
    /* Initialize the Software Timer Module */
    SystemTimerInit();
#ifdef CONF_PMM_ENABLE
    /* Initialize the Sleep Timer Module */
    SleepTimerInit();
#endif
#if (ENABLE_PDS == 1)
    /* PDS Module Init */
    PDS_Init();
#endif
	/* Initializes the Security modules */
	sal_status = SAL_Init();
	
	if (SAL_SUCCESS != sal_status)
	{
		printf("Initialization of Security module is failed\r\n");
		/* Stop Further execution */
		while (1) {
		}
	}
}

static void setup_ADC(void){
	//Get the default ADC config settings
	struct adc_config config_adc;

	adc_get_config_defaults(&config_adc);

	/*Adjust the configuration*/
	/*Default is as follows: see adc_feature.h
		GCLK generator 0 (GCLK main) clock source
		[adc_reference] Internal bandgap reference
		[adc_clock_prescaler] Div 2 clock prescaler
		[adc_resolution] 12-bit resolution
		[adc_window_mode] Window monitor disabled
		[adc_positive_input] Positive input on ADC PIN 1
		[adc_negative_input] Negative input on Internal ground
		[adc_accumulate_samples] Averaging disabled
		[adc_oversampling_and_decimation] Oversampling disabled
		[left_adjust] Right adjust data  Moves position of result in register.
		[differential_mode] Single-ended mode
		[freerunning] Free running disabled
		All events (input and generation) disabled
		ADC run in standby disabled
		ADC On demand disabled
		No sampling time compensation
		Disable the positive input sequence
		No reference compensation
		No gain/offset correction
		No added sampling time
	*/
	//Set the first pin to be read by the ADC
	config_adc.positive_input = ADC_POSITIVE_INPUT_PIN6;  //PA06 ADC+   //Only need the first positive channel in the sequence
	//Set the absolute pins that will be used for ADC.
	//This should set the SEQCTRL register, but it doesn't.
	config_adc.positive_input_sequence_mask_enable = 0x100C0;  //Pins PA06,PA07,PA08 are Ain 6,7,16  1 0000 0000 1100 0000b
	//Instead, manually set the SEQCTRL register.  Doing in 3 calls because I couldn't think to do it properly.
	//Set the Sequence control
	memset((uint8_t *)0x43000C28U, 0xC0U, sizeof(uint8_t));  //ADC SEQCTRL base address.
	memset((uint8_t *)0x43000C29U, 0x00U, sizeof(uint8_t));  //
	memset((uint8_t *)0x43000C2AU, 0x01U, sizeof(uint8_t));  //
	
	//Set the analogue reference and differential mode.
	config_adc.reference = ADC_REFERENCE_INTVCC2; //VDD_ANA  (Should be 3.3V, default gives 0-1V)
	config_adc.differential_mode = false;

	//Make ADC 16 bit.  Resolution will depend on sensor range.
	config_adc.resolution = ADC_RESOLUTION_16BIT;
	config_adc.accumulate_samples = ADC_ACCUMULATE_SAMPLES_16;
	config_adc.divide_result = ADC_DIVIDE_RESULT_DISABLE;
	
	//initialize ADC
	adc_init(&adc_instance, ADC, &config_adc);

	//Enables ADC
	adc_enable(&adc_instance);
}


static void config_board(void)
{
	//Pins to control
	/*
	P04 - UART Tx
	P05 - UART Rx
	P06 - ADC +
	P07 - ADC -
	PA14 - CapSense IN
	PA15 - CapSense OUT
	PA16 - I2C SDA
	PA17 - I2C SCL
	PA18 - LED GREEN
	PA19 - LED YELLOW
	PA27 - Sensor Power Switch
	*/
	struct port_config pin_conf;
	
	//UART
	//port_get_config_defaults(&pin_conf);
	//pin_conf.direction  = PORT_PIN_DIR_OUTPUT;
	//port_pin_set_config(PIN_PA04, &pin_conf);
	//port_pin_set_output_level(PIN_PA04, false);
	
	//port_get_config_defaults(&pin_conf);
	//pin_conf.direction  = PORT_PIN_DIR_OUTPUT;
	//port_pin_set_config(PIN_PA05, &pin_conf);
	//port_pin_set_output_level(PIN_PA05, false);
	
	//ADC
	
	//port_get_config_defaults(&pin_conf);
	//pin_conf.direction  = PORT_PIN_DIR_INPUT;
	//port_pin_set_config(PIN_PA06, &pin_conf);
	//port_pin_set_output_level(PIN_PA06, false);
	
	//port_get_config_defaults(&pin_conf);
	//pin_conf.direction  = PORT_PIN_DIR_OUTPUT;
	//port_pin_set_config(PIN_PA07, &pin_conf);
	//port_pin_set_output_level(PIN_PA07, false);
	
	//CapSense
	//port_get_config_defaults(&pin_conf);
	//pin_conf.direction  = PORT_PIN_DIR_OUTPUT;
	//port_pin_set_config(PIN_PA14, &pin_conf);
	//port_pin_set_output_level(PIN_PA14, false);
	
	//BATTERY TEST CHANGE
	//PIN_PA15 is being used for battery measurement sink.
	port_get_config_defaults(&pin_conf);
	pin_conf.direction  = PORT_PIN_DIR_OUTPUT;
	port_pin_set_config(PIN_PA15, &pin_conf);
	port_pin_set_output_level(PIN_PA15, true);  // Set high to reduce current sink with resistor divider
	//I2C
	//port_get_config_defaults(&pin_conf);
	//pin_conf.direction  = PORT_PIN_DIR_OUTPUT;
	//port_pin_set_config(PIN_PA16, &pin_conf);
	//port_pin_set_output_level(PIN_PA16, false);
	
	//port_get_config_defaults(&pin_conf);
	//pin_conf.direction  = PORT_PIN_DIR_OUTPUT;
	//port_pin_set_config(PIN_PA17, &pin_conf);
	//port_pin_set_output_level(PIN_PA17, false);
	
	//LED_GREEN
	//port_get_config_defaults(&pin_conf);
	//pin_conf.direction  = PORT_PIN_DIR_OUTPUT;
	//port_pin_set_config(PIN_PA18, &pin_conf);
	//port_pin_set_output_level(PIN_PA18, false);
	//LED_YELLOW
	//port_get_config_defaults(&pin_conf);
	//pin_conf.direction  = PORT_PIN_DIR_OUTPUT;
	//port_pin_set_config(PIN_PA19, &pin_conf);
	//port_pin_set_output_level(PIN_PA19, false);
	//VSen Power
	port_get_config_defaults(&pin_conf);
	pin_conf.direction  = PORT_PIN_DIR_OUTPUT;
	port_pin_set_config(PIN_PA27, &pin_conf);
	port_pin_set_output_level(PIN_PA27, false);
	
	//VBatt Sink
	//v0.5 code
	//port_get_config_defaults(&pin_conf);
	//pin_conf.direction  = PORT_PIN_DIR_OUTPUT;
	//port_pin_set_config(PIN_PB03, &pin_conf);
	//port_pin_set_output_level(PIN_PB03, true);
	
}


#ifdef CONF_PMM_ENABLE
static void app_resources_uninit(void)
{
    /* Disable USART TX and RX Pins */
    struct port_config pin_conf;
    port_get_config_defaults(&pin_conf);
    pin_conf.powersave  = true;
    port_pin_set_config(HOST_SERCOM_PAD0_PIN, &pin_conf);
    port_pin_set_config(HOST_SERCOM_PAD1_PIN, &pin_conf);
    /* Disable UART module */
    sio2host_deinit();
    /* Disable Transceiver SPI Module */
    HAL_RadioDeInit();
}
#endif
/**
 End of File
 */
