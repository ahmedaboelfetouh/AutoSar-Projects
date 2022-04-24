 /******************************************************************************
 *
 * Module: Port
 *
 * File Name: Port_Cfg.h
 *
 * Description: Pre-Compile Configuration Header file for TM4C123GH6PM Microcontroller - Port Driver
 *
 * Author: Ahmed Aboelfetouh
 ******************************************************************************/

#ifndef PORT_CFG_H
#define PORT_CFG_H

/*
 * Module Version 1.0.0
 */
#define PORT_CFG_SW_MAJOR_VERSION              (1U)
#define PORT_CFG_SW_MINOR_VERSION              (0U)
#define PORT_CFG_SW_PATCH_VERSION              (0U)

/*
 * AUTOSAR Version 4.0.3
 */
#define PORT_CFG_AR_RELEASE_MAJOR_VERSION     (4U)
#define PORT_CFG_AR_RELEASE_MINOR_VERSION     (0U)
#define PORT_CFG_AR_RELEASE_PATCH_VERSION     (3U)

/* Pre-compile option for Development Error Detect */
#define PORT_DEV_ERROR_DETECT                (STD_ON)

/* Pre-compile option for Version Info API */
#define PORT_VERSION_INFO_API                (STD_OFF)
   
/* Pre-compile option for set pin direction during run time */
#define PORT_SET_PIN_DIRECTION_API           (STD_ON)

/* Number of the configured Dio Channels */
#define PORT_CONFIGURED_PINS                  (39U)

/* Channel Index in the array of structures in Port_PBcfg.c */
#define PortConf_LED1_CHANNEL_ID_INDEX        (uint8)0x24     /* Pin no. 36 (PF1) */
#define PortConf_SW1_CHANNEL_ID_INDEX         (uint8)0x27     /* Pin no. 39 (PF4) */

/* Port Configured Port ID's  */
#define PortConf_LED1_PORT_NUM                (Port_PortType)5 /* PORTF */
#define PortConf_SW1_PORT_NUM                 (Port_PortType)5 /* PORTF */

/* Port Configured Channel ID's */
#define PortConf_LED1_CHANNEL_NUM             (Port_PinType)1 /* Pin 1 in PORTF */
#define PortConf_SW1_CHANNEL_NUM              (Port_PinType)4 /* Pin 4 in PORTF */
   
/* GPIO PORTS */
#define PORTA               (Port_PortType)0
#define PORTB               (Port_PortType)1
#define PORTC               (Port_PortType)2
#define PORTD               (Port_PortType)3
#define PORTE               (Port_PortType)4
#define PORTF               (Port_PortType)5

/* GPIO PINS */
#define PIN0                 (Port_PinType)0
#define PIN1                 (Port_PinType)1
#define PIN2                 (Port_PinType)2
#define PIN3                 (Port_PinType)3
#define PIN4                 (Port_PinType)4
#define PIN5                 (Port_PinType)5
#define PIN6                 (Port_PinType)6
#define PIN7                 (Port_PinType)7

#endif /* Port_CFG_H */
