/**
* \file  conf_app.h
*
* \brief LORAWAN Demo Application include file
*
*
* Copyright (c) 2020 Microchip Technology Inc. and its subsidiaries.
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



#ifndef APP_CONFIG_H_
#define APP_CONFIG_H_

/****************************** INCLUDES **************************************/

/****************************** MACROS **************************************/
/* Number of software timers */
#define TOTAL_NUMBER_OF_TIMERS            (25u)

/* If enabled, app will use preprogrammed devEUI from module's NVM location */
#if (MODULE_EUI_READ == 1)
	#define NVM_UID_ADDRESS   ((volatile uint16_t *)(0x0080400AU))
#endif

/*Define the Sub band of Channels to be enabled by default for the application*/
#define SUBBAND 2
#if ((SUBBAND < 1 ) || (SUBBAND > 8 ) )
#error " Invalid Value of Subband"
#endif

/* Activation method constants */
#define OVER_THE_AIR_ACTIVATION           LORAWAN_OTAA
#define ACTIVATION_BY_PERSONALIZATION     LORAWAN_ABP

/* Message Type constants */
#define UNCONFIRMED                       LORAWAN_UNCNF
#define CONFIRMED                         LORAWAN_CNF

/* Enable one of the activation methods */
#define DEMO_APP_ACTIVATION_TYPE               OVER_THE_AIR_ACTIVATION
//#define DEMO_APP_ACTIVATION_TYPE               ACTIVATION_BY_PERSONALIZATION

/* Select the Type of Transmission - Confirmed(CNF) / Unconfirmed(UNCNF) */
#define DEMO_APP_TRANSMISSION_TYPE              UNCONFIRMED
//#define DEMO_APP_TRANSMISSION_TYPE            CONFIRMED

/* FPORT Value (1-255) */
#define DEMO_APP_FPORT                           1

/* Device Class - Class of the device (CLASS_A/CLASS_C) */
#define DEMO_APP_ENDDEVICE_CLASS                 CLASS_A
//#define DEMO_APP_ENDDEVICE_CLASS                 CLASS_C


/* ABP Join Parameters */

#define DEMO_DEVICE_ADDRESS                     0xdeafface
#define DEMO_APPLICATION_SESSION_KEY            {0x41, 0x63, 0x74, 0x69, 0x6C, 0x69, 0x74, 0x79, 0x00, 0x04, 0xA3, 0x0B, 0x00, 0x04, 0xA3, 0x0B}
#define DEMO_NETWORK_SESSION_KEY                {0x61, 0x63, 0x74, 0x69, 0x6C, 0x69, 0x74, 0x79, 0x00, 0x04, 0xA3, 0x0B, 0x00, 0x04, 0xA3, 0x0B}


/* OTAA Join Parameters */
////SN346
#define DEMO_DEVICE_EUI                         {0x6A, 0x27, 0x19, 0xFF, 0xFE, 0xC5, 0x00, 0x27} //SN346 DevEUI {0x27, 0x00, 0xC5, 0xFE, 0xFF, 0x19, 0x27, 0x6A}
#define DEMO_JOIN_EUI							{0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x05} //Default Value
#define DEMO_APPLICATION_KEY                    {0x2E, 0x67, 0x6B, 0xC0, 0x73, 0xAA, 0xD2, 0x4A, 0x4B, 0x62, 0x1F, 0xBC, 0x3C, 0x86, 0xDD, 0x27} //Default Value
	
//#define DEMO_DEVICE_EUI							{0x63, 0x17, 0xC1, 0xFE, 0xFF, 0x1B, 0xEB, 0xEA } //cn-0002 DevEUI  //Actual value read from module
//#define DEMO_JOIN_EUI							{0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x05} //Default Value    
//#define DEMO_APPLICATION_KEY					{0xF8, 0x64, 0x2A, 0x2C, 0x8D, 0xAC, 0xF6, 0x82, 0xE4, 0x23, 0x37, 0x3C, 0x35, 0x37, 0x50, 0x0D} //cn-0002 TTN Value

/* Multicast Parameters */
#define DEMO_APP_MCAST_GROUPID                  0
#define DEMO_APP_MCAST_ENABLE                   true
#define DEMO_APP_MCAST_GROUP_ADDRESS            0x0037CC56
#define DEMO_APP_MCAST_APP_SESSION_KEY          {0x2B, 0x7E, 0x15, 0x16, 0x28, 0xAE, 0xD2, 0xA6, 0x2B, 0x7E, 0x15, 0x16, 0x28, 0xAE, 0xD2, 0xA6}
#define DEMO_APP_MCAST_NWK_SESSION_KEY          {0x3C, 0x8F, 0x26, 0x27, 0x39, 0xBF, 0xE3, 0xB7, 0xBC, 0x08, 0x26, 0x99, 0x1A, 0xD0, 0x50, 0x4D}

/* This macro defines the application's default sleep duration in milliseconds */
#define DEMO_CONF_DEFAULT_APP_SLEEP_TIME_MS     30000

#endif /* APP_CONFIG_H_ */

