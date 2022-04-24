 /******************************************************************************
 *
 * Module: Port
 *
 * File Name: Port.c
 *
 * Description: Source file for TM4C123GH6PM Microcontroller - Port Driver.
 *
 * Author: Ahmed Aboelfetouh
 ******************************************************************************/

#include "Port.h"
#include "Port_Regs.h"

#if (PORT_DEV_ERROR_DETECT == STD_ON)

#include "Det.h"
/* AUTOSAR Version checking between Det and PORT Modules */
#if ((DET_AR_MAJOR_VERSION != PORT_AR_RELEASE_MAJOR_VERSION)\
 || (DET_AR_MINOR_VERSION != PORT_AR_RELEASE_MINOR_VERSION)\
 || (DET_AR_PATCH_VERSION != PORT_AR_RELEASE_PATCH_VERSION))
  #error "The AR version of Det.h does not match the expected version"
#endif

/* Local array to hold all the modes that each pin supports. (To check valid mode or not)
 * where each pin has 2 bytes to hold all supported modes.
 * Each mode stored in a specific bit according to its number. */
STATIC const uint16 Port_PinModesArray[PORT_CONFIGURED_PINS] = {
       ((1<<DIO) | (1<<UART) | (1<<CAN)),                                         /* PA0 suports DIO, UART and CAN modes*/
       ((1<<DIO) | (1<<UART) | (1<<CAN)),                                         /* PA1 suports DIO, UART and CAN modes*/
       ((1<<DIO) | (1<<SSI)),                                                     /* PA2 supports DIO and SSI modes*/
       ((1<<DIO) | (1<<SSI)),                                                     /* PA3 supports DIO and SSI modes*/
       ((1<<DIO) | (1<<SSI)),                                                     /* PA4 supports DIO and SSI modes*/
       ((1<<DIO) | (1<<SSI)),                                                     /* PA5 supports DIO and SSI modes*/
       ((1<<DIO) | (1<<I2C) |(1<<PWM5)),                                          /* PA6 supports DIO, I2C and PWM modes*/
       ((1<<DIO) | (1<<I2C) |(1<<PWM5)),                                          /* PA7 supports DIO, I2C and PWM modes*/
       ((1<<DIO) | (1<<UART) | (1<<TIMER)),                                       /* PB0 supports DIO, UART and Timer modes*/
       ((1<<DIO) | (1<<UART) | (1<<TIMER)),                                       /* PB1 supports DIO, UART and Timer modes*/
       ((1<<DIO) | (1<<I2C) | (1<<TIMER)),                                        /* PB2 supports DIO, I2C and Timer modes*/
       ((1<<DIO) | (1<<I2C) | (1<<TIMER)),                                        /* PB3 supports DIO, I2C and Timer modes*/
       ((1<<DIO) | (1<<ANALOG) |(1<<SSI) | (1<<PWM4) |(1<<TIMER) |(1<<CAN)),      /* PB4 supports DIO, Analog, SSI, PWM, CAN and Timer modes*/
       ((1<<DIO) | (1<<ANALOG) |(1<<SSI) | (1<<PWM4) |(1<<TIMER) |(1<<CAN)),      /* PB5 supports DIO, Analog, SSI, PWM, CAN and Timer modes*/
       ((1<<DIO) | (1<<SSI) | (1<<PWM4) |(1<<TIMER) ),                            /* PB6 supports DIO, SSI, PWM and Timer modes*/
       ((1<<DIO) | (1<<SSI) | (1<<PWM4) |(1<<TIMER) ),                            /* PB7 supports DIO, SSI, PWM and Timer modes*/
       ((1<<DIO) | (1<<UART) | (1<<PWM4) | (1<<IDX)),                             /* PC4 supports DIO, UART, PWM and IDX modes*/
       ((1<<DIO) | (1<<UART) | (1<<PWM4) | (1<<IDX)),                             /* PC5 supports DIO, UART, PWM and IDX modes*/
       ((1<<DIO) | (1<<UART)),                                                    /* PC6 supports DIO and UART modes*/
       ((1<<DIO) | (1<<UART)),                                                    /* PC7 supports DIO and UART modes*/
       ((1<<DIO) | (1<<ANALOG) |(1<<SSI) |(1<<I2C) |(1<<PWM4)),                   /* PD0 supports DIO, Analog, SSI, I2C and PWM modes*/
       ((1<<DIO) | (1<<ANALOG) |(1<<SSI) |(1<<I2C) |(1<<PWM4)),                   /* PD1 supports DIO, Analog, SSI, I2C and PWM modes*/
       ((1<<DIO) | (1<<ANALOG) |(1<<SSI)),                                        /* PD2 supports DIO, Analog and SSI*/
       ((1<<DIO) | (1<<ANALOG) |(1<<SSI) | (1<<IDX)),                             /* PD3 supports DIO, Analog, IDX and SSI*/
       ((1<<DIO) | (1<<UART)),                                                    /* PD4 supports DIO and UART */
       ((1<<DIO) | (1<<UART)),                                                    /* PD5 supports DIO and UART */
       ((1<<DIO) | (1<<UART)),                                                    /* PD6 supports DIO and UART */
       ((1<<DIO) | (1<<UART)),                                                    /* PD7 supports DIO and UART */
       ((1<<DIO) | (1<<ANALOG) |(1<<UART)),                                       /* PE0 supports DIO, Analog and UART */
       ((1<<DIO) | (1<<ANALOG) |(1<<UART)),                                       /* PE1 supports DIO, Analog and UART */
       ((1<<DIO) | (1<<ANALOG)),                                                  /* PE2 supports DIO and Analog */
       ((1<<DIO) | (1<<ANALOG)),                                                  /* PE3 supports DIO and Analog */
       ((1<<DIO) | (1<<ANALOG) |(1<<UART) |(1<<I2C) |(1<<PWM5) |(1<<CAN)),        /* PE4 supports DIO, UART, I2C, PWM, CAN and Analog */
       ((1<<DIO) | (1<<ANALOG) |(1<<UART) |(1<<I2C) |(1<<PWM5) |(1<<CAN)),        /* PE5 supports DIO, UART, I2C, PWM, CAN and Analog */
       ((1<<DIO) | (1<<SSI) |(1<<UART) |(1<<TIMER) |(1<<PWM5)),                   /* PF0 supports DIO, UART, SSI, PWM and timer */
       ((1<<DIO) | (1<<SSI) |(1<<UART) |(1<<TIMER) |(1<<PWM5) |(1<<TRD)),         /* PF1 supports DIO, UART, SSI, PWM, TRD and timer */
       ((1<<DIO) | (1<<SSI) |(1<<TIMER) |(1<<PWM5)| (1<<TRD)),                    /* PF2 supports DIO, SSI, PWM, TRD and timer */
       ((1<<DIO) | (1<<SSI) |(1<<TIMER) |(1<<PWM5)| (1<<TRD)),                    /* PF3 supports DIO, SSI, PWM, TRD and timer */
       ((1<<DIO) | (1<<IDX) |(1<<TIMER)),                                         /* PF4 supports DIO, IDX and timer */
};
#endif   /* PORT_DEV_ERROR_DETECT == STD_ON */

STATIC const Port_ConfigPin * Port_PortPins = NULL_PTR;
STATIC uint8 Port_Status = PORT_NOT_INITIALIZED;

/*************************************************************************************************************************************************
* Service Name: Port_Init
* Service ID[hex]: 0x00
* Sync/Async: Synchronous
* Reentrancy: Non reentrant
* Parameters (in): ConfigPtr - Pointer to post-build configuration data
* Parameters (inout): None
* Parameters (out): None
* Return value: None
* Description: Function to Initialize the port module.
**************************************************************************************************************************************************/
void Port_Init(const Port_ConfigType* ConfigPtr)
{
  /* Counter to loop through all the configured Pins */
  uint8 PinCounter;
  /* point to the current pin to be initialized in the PB structure */
  const Port_ConfigPin * CurrentPin = NULL_PTR;
  /* point to the required Port Registers base address */
  volatile uint32 * PortGpio_Ptr = NULL_PTR;
  
#if (PORT_DEV_ERROR_DETECT == STD_ON)  
	/* check if the input configuration pointer is not a NULL_PTR */
	if (NULL_PTR == ConfigPtr)
	{
		Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, PORT_INIT_SID, PORT_E_PARAM_CONFIG);
	}
	else
#endif
        {
          /* Enable clock for all PORTs */
          SYSCTL_REGCGC2_REG |= 0x3F;
          /* Point to the first pin to be configured and allow time for clock to start*/
          CurrentPin = ConfigPtr->Pins;
          /* Loop to initialize all the configured pins */
          for(PinCounter = 0; PinCounter < PORT_CONFIGURED_PINS; PinCounter++)
          {
             /* point to the current pin base regisiter */
              switch(CurrentPin[PinCounter].Port_Num)
              {
                  case  PORTA: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTA_BASE_ADDRESS; /* PORTA Base Address */
                               break;
                  case  PORTB: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTB_BASE_ADDRESS; /* PORTB Base Address */
                               break;
                  case  PORTC: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTC_BASE_ADDRESS; /* PORTC Base Address */
                               break;
                  case  PORTD: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTD_BASE_ADDRESS; /* PORTD Base Address */
                               break;
                  case  PORTE: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTE_BASE_ADDRESS; /* PORTE Base Address */
                               break;
                  case  PORTF: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTF_BASE_ADDRESS; /* PORTF Base Address */
                               break;
              }
              
              if( ((CurrentPin[PinCounter].Port_Num == PORTD) && (CurrentPin[PinCounter].Pin_Num == PIN7)) 
                 || ((CurrentPin[PinCounter].Port_Num == PORTF) && (CurrentPin[PinCounter].Pin_Num == PIN0)) ) /* PD7 or PF0 */
              {
                /* Unlock the GPIOCR register */ 
                *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_LOCK_REG_OFFSET) = 0x4C4F434B;     
                /* Set the corresponding bit in GPIOCR register to allow changes on this pin */
                SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_COMMIT_REG_OFFSET) , CurrentPin[PinCounter].Pin_Num);  
              }
              
              else if( (CurrentPin[PinCounter].Port_Num == PORTC) && (CurrentPin[PinCounter].Pin_Num <= PIN3) ) /* PC0 to PC3 */
              {
                  /* Do Nothing ...  this is the JTAG pins */
                   continue;  /* Skip this pin */
              }
                        
              if(CurrentPin[PinCounter].Pin_mode == DIO)
              {
                 /* Disable Analog */
                 CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET) , CurrentPin[PinCounter].Pin_Num);
                 /* Disable alternative functions */
                 CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , CurrentPin[PinCounter].Pin_Num);
                 /*Enable Digital I/O*/
                 SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET) , CurrentPin[PinCounter].Pin_Num);
                 /* Put the mode number in the PCTL register for the current pin by clearing the 4 bits for GPIO */
                  *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(0x0000000F << CurrentPin[PinCounter].Pin_Num * 4);
              }
              else if (CurrentPin[PinCounter].Pin_mode == ANALOG)
              {
                  /*Enable Analog*/
                  SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET) , CurrentPin[PinCounter].Pin_Num);
                  /* Enable alternative functions */
                   SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , CurrentPin[PinCounter].Pin_Num);
                  /*Disable Digital I/O*/
                   CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET) , CurrentPin[PinCounter].Pin_Num);
              }
              else
              {
                  /* Disable Analog */
                   CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET) , CurrentPin[PinCounter].Pin_Num);
                   /* Enable alternative functions */
                   SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , CurrentPin[PinCounter].Pin_Num);
                   /*Enable Digital I/O*/
                   SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET) , CurrentPin[PinCounter].Pin_Num);
                   /* Put the mode number in the PCTL register for the current pin Clearing the 4 bits and then insert the mode number */
                   *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(0x0000000F << CurrentPin[PinCounter].Pin_Num * 4);
                   *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (CurrentPin[PinCounter].Pin_mode << CurrentPin[PinCounter].Pin_Num * 4); 
              }
              
              if(CurrentPin[PinCounter].direction == OUTPUT)
              {
                  /* Set the corresponding bit in the GPIODIR register to configure it as output pin */
                  SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIR_REG_OFFSET) , CurrentPin[PinCounter].Pin_Num);
                  
                  if(CurrentPin[PinCounter].initial_value == STD_HIGH)
                  {
                       /* Set the corresponding bit in the GPIODATA register to provide initial value 1 */
                      SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DATA_REG_OFFSET) , CurrentPin[PinCounter].Pin_Num);
                  }
                  else
                  {
                      /* Clear the corresponding bit in the GPIODATA register to provide initial value 0 */
                      CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DATA_REG_OFFSET) , CurrentPin[PinCounter].Pin_Num);
                  }
              }
              else if(CurrentPin[PinCounter].direction == INPUT)
              {
                  /* Clear the corresponding bit in the GPIODIR register to configure it as input pin */
                  CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIR_REG_OFFSET) , CurrentPin[PinCounter].Pin_Num);
                  
                  if(CurrentPin[PinCounter].resistor == PULL_UP)
                  {
                      /* Set the corresponding bit in the GPIOPUR register to enable the internal pull up pin */
                      SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_PULL_UP_REG_OFFSET) , CurrentPin[PinCounter].Pin_Num);
                  }
                  else if(CurrentPin[PinCounter].resistor == PULL_DOWN)
                  {
                      /* Set the corresponding bit in the GPIOPDR register to enable the internal pull down pin */
                      SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_PULL_DOWN_REG_OFFSET) ,CurrentPin[PinCounter].Pin_Num);
                  }
                  else
                  {
                      /* Clear the corresponding bit in the GPIOPUR register to disable the internal pull up pin */
                      CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_PULL_UP_REG_OFFSET) , CurrentPin[PinCounter].Pin_Num);
                      /* Clear the corresponding bit in the GPIOPDR register to disable the internal pull down pin */
                      CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_PULL_DOWN_REG_OFFSET) , CurrentPin[PinCounter].Pin_Num);
                  }
              }
              else 
              {
                /* No Action Required */
              }              
          }
         /* Set the module state to initialized and point to the PB configuration structure using a global pointer.
         * This global pointer is global to be used by other functions to read the PB configuration structures */
          Port_Status       = PORT_INITIALIZED;
          Port_PortPins = ConfigPtr->Pins;
        }
}

/**************************************************************************************************************************************************
* Service Name: Port_SetPinDirection
* Service ID[hex]: 0x01
* Sync/Async: Synchronous
* Reentrancy: Reentrant
* Parameters (in): Pin           - Port Pin ID number
                   Direction     - Port Pin Direction
* Parameters (inout): None
* Parameters (out): None
* Return value: None
* Description: Sets the port pin direction
**************************************************************************************************************************************************/
#if (PORT_SET_PIN_DIRECTION_API == STD_ON)
void Port_SetPinDirection(Port_PinType Pin,Port_PinDirectionType Direction)
{
        /* point to the required Port Registers base address */
        volatile uint32 * PortGpio_Ptr = NULL_PTR;
        /* Flag to hold if there is det error or not */
        boolean error = FALSE;
        
#if (PORT_DEV_ERROR_DETECT == STD_ON)
        /* Check if the Driver is initialized before using this function */
	if (PORT_NOT_INITIALIZED == Port_Status)
	{
		Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, PORT_SET_PIN_DIRECTION_SID, PORT_E_UNINIT);
		error = TRUE;
	}
	else
	{
		/* No Action Required */
	}
        /* Check if the used pin is within the valid range */
        if (PORT_CONFIGURED_PINS <= Pin)
        {
              Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, PORT_SET_PIN_DIRECTION_SID, PORT_E_PARAM_PIN);
              error = TRUE;
        }
        else
        {
          /* No Action Required */
        }
        /* Check if the used pin is allowed to change its direction or not */
        if (DISABLLED == Port_PortPins[Pin].Pin_IsDirectionChangable)
        {
              Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, PORT_SET_PIN_DIRECTION_SID, PORT_E_DIRECTION_UNCHANGEABLE);
              error = TRUE;
        }
        else
        {
          /* No Action Required */
        }    
#endif  /* (PORT_DEV_ERROR_DETECT == STD_ON) */
        
        /* In-case there are no errors */
	if(FALSE == error)
	{
           /* point to the current pin base regisiter */
            switch(Port_PortPins[Pin].Port_Num)
            {
                case  PORTA: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTA_BASE_ADDRESS; /* PORTA Base Address */
                             break;
                case  PORTB: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTB_BASE_ADDRESS; /* PORTB Base Address */
                             break;
                case  PORTC: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTC_BASE_ADDRESS; /* PORTC Base Address */
                             break;
                case  PORTD: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTD_BASE_ADDRESS; /* PORTD Base Address */
                             break;
                case  PORTE: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTE_BASE_ADDRESS; /* PORTE Base Address */
                             break;
                case  PORTF: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTF_BASE_ADDRESS; /* PORTF Base Address */
                             break;
            }
            
            if(Direction == OUTPUT)
              {
                  /* Set the corresponding bit in the GPIODIR register to configure it as output pin */
                  SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIR_REG_OFFSET) , Port_PortPins[Pin].Pin_Num);
              }
            else if(Direction == INPUT)
              {
                  /* Clear the corresponding bit in the GPIODIR register to configure it as input pin */
                  CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIR_REG_OFFSET) , Port_PortPins[Pin].Pin_Num);
              }
        }
}
#endif  /* (PORT_SET_PIN_DIRECTION_API == STD_ON) */

/**************************************************************************************************************************************************
* Service Name: Port_RefreshPortDirection
* Service ID[hex]: 0x02
* Sync/Async: Synchronous
* Reentrancy: Non-reentrant
* Parameters (in): None
* Parameters (inout): None
* Parameters (out): None
* Return value: None
* Description: Refreshes port direction.
**************************************************************************************************************************************************/
void Port_RefreshPortDirection(void)
{
        /* point to the required Port Registers base address */
        volatile uint32 * PortGpio_Ptr = NULL_PTR;
        /* Counter to loop through all the configured Pins */
        uint8 PinCounter;
        
#if (PORT_DEV_ERROR_DETECT == STD_ON)
        /* Check if the Driver is initialized before using this function */
	if (PORT_NOT_INITIALIZED == Port_Status)
	{
		Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, PORT_REFRESH_PORT_DIRECTION_SID, PORT_E_UNINIT);
	}
	else
#endif  /* (PORT_DEV_ERROR_DETECT == STD_ON) */
	{
          /* Loop to referesh all the configured pins */
          for(PinCounter = 0; PinCounter < PORT_CONFIGURED_PINS; PinCounter++)
          {
            /*Excluding all Port pins that are configured as ‘pin direction changeable during runtime‘.*/
            if(DISABLLED == Port_PortPins[PinCounter].Pin_IsDirectionChangable)
            {
                 /* point to the current pin base regisiter */
                switch(Port_PortPins[PinCounter].Port_Num)
                {
                    case  PORTA: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTA_BASE_ADDRESS; /* PORTA Base Address */
                                 break;
                    case  PORTB: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTB_BASE_ADDRESS; /* PORTB Base Address */
                                 break;
                    case  PORTC: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTC_BASE_ADDRESS; /* PORTC Base Address */
                                 break;
                    case  PORTD: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTD_BASE_ADDRESS; /* PORTD Base Address */
                                 break;
                    case  PORTE: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTE_BASE_ADDRESS; /* PORTE Base Address */
                                 break;
                    case  PORTF: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTF_BASE_ADDRESS; /* PORTF Base Address */
                                 break;
                }
                    if(Port_PortPins[PinCounter].direction == OUTPUT)
                    {
                        /* Set the corresponding bit in the GPIODIR register to configure it as output pin */
                        SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIR_REG_OFFSET) , Port_PortPins[PinCounter].Pin_Num);
                       
                    }
                    else if(Port_PortPins[PinCounter].direction == INPUT)
                    {
                        /* Clear the corresponding bit in the GPIODIR register to configure it as input pin */
                        CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIR_REG_OFFSET) , Port_PortPins[PinCounter].Pin_Num);
                    }
            }
            else
            {
               /* No Action needed as the pin direction chengeability is enalled */
            }
          }
        }
}

/**************************************************************************************************************************************************
* Service Name: Port_GetVersionInfo
* Service ID[hex]: 0x03
* Sync/Async: Synchronous
* Reentrancy: Non-reentrant
* Parameters (in): None
* Parameters (inout): None
* Parameters (out): versioninfo    -Pointer to where to store the version information of this module.
* Return value: None
* Description: Returns the version information of this module.
**************************************************************************************************************************************************/
#if (PORT_VERSION_INFO_API == STD_ON)
void Port_GetVersionInfo(Std_VersionInfoType* versioninfo)
{
 #if (PORT_DEV_ERROR_DETECT == STD_ON)
	/* Check if input pointer is not Null pointer */
	if(NULL_PTR == versioninfo)
	{
		/* Report to DET  */
		Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, PORT_GET_VERSION_INFO_SID, PORT_E_PARAM_POINTER);
	}
	else
#endif /* (PORT_DEV_ERROR_DETECT == STD_ON) */
	{
		/* Copy the vendor Id */
		versioninfo->vendorID = (uint16)PORT_VENDOR_ID;
		/* Copy the module Id */
		versioninfo->moduleID = (uint16)PORT_MODULE_ID;
		/* Copy Software Major Version */
		versioninfo->sw_major_version = (uint8)PORT_SW_MAJOR_VERSION;
		/* Copy Software Minor Version */
		versioninfo->sw_minor_version = (uint8)PORT_SW_MINOR_VERSION;
		/* Copy Software Patch Version */
		versioninfo->sw_patch_version = (uint8)PORT_SW_PATCH_VERSION;
	} 
}
#endif /* (PORT_VERSION_INFO_API == STD_ON) */

/**************************************************************************************************************************************************
* Service Name: Port_SetPinMode
* Service ID[hex]: 0x04
* Sync/Async: Synchronous
* Reentrancy: Reentrant
* Parameters (in): Pin           - Port Pin ID number
                   Mode          - New Port Pin mode to be set on port pin.
* Parameters (inout): None
* Parameters (out): None
* Return value: None
* Description: Sets the port pin mode.
**************************************************************************************************************************************************/
void Port_SetPinMode(Port_PinType Pin,Port_PinModeType Mode)
{
        /* point to the required Port Registers base address */
        volatile uint32 * PortGpio_Ptr = NULL_PTR;
        /* Flag to hold if there is det error or not */
        boolean error = FALSE;
        
#if (PORT_DEV_ERROR_DETECT == STD_ON)
        /* Check if the Driver is initialized before using this function */
	if (PORT_NOT_INITIALIZED == Port_Status)
	{
		Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, Port_SET_PIN_MODE_SID, PORT_E_UNINIT);
		error = TRUE;
	}
	else
	{
		/* No Action Required */
	}
        /* Check if the used pin is within the valid range */
        if (PORT_CONFIGURED_PINS <= Pin)
        {
              Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, Port_SET_PIN_MODE_SID, PORT_E_PARAM_PIN);
              error = TRUE;
        }
        else
        {
          /* No Action Required */
        }
        /* Check if the used pin is allowed to change its direction or not */
        if (DISABLLED == Port_PortPins[Pin].Port_IsModeChangable)
        {
              Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, Port_SET_PIN_MODE_SID, PORT_E_MODE_UNCHANGEABLE);
              error = TRUE;
        }
        else
        {
          /* No Action Required */
        }
        /* Checking if the required mode is suportted by the pin or not
         by Checking the corrosponding bit to the mode number is set or not
         in the 2 bytes crrosponding to the current pin in the mode array*/
        if BIT_IS_CLEAR(Port_PinModesArray[Pin],Mode)
        {
              Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, Port_SET_PIN_MODE_SID, PORT_E_PARAM_INVALID_MODE);
              error = TRUE;
        }
        else
        {
          /* No Action Required */
        }
     
#endif  /* (PORT_DEV_ERROR_DETECT == STD_ON) */
        
        /* In-case there are no errors */
	if(FALSE == error)
	{
           /* point to the current pin base regisiter */
            switch(Port_PortPins[Pin].Port_Num)
            {
                case  PORTA: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTA_BASE_ADDRESS; /* PORTA Base Address */
                             break;
                case  PORTB: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTB_BASE_ADDRESS; /* PORTB Base Address */
                             break;
                case  PORTC: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTC_BASE_ADDRESS; /* PORTC Base Address */
                             break;
                case  PORTD: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTD_BASE_ADDRESS; /* PORTD Base Address */
                             break;
                case  PORTE: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTE_BASE_ADDRESS; /* PORTE Base Address */
                             break;
                case  PORTF: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTF_BASE_ADDRESS; /* PORTF Base Address */
                             break;
            }
            if(Port_PortPins[Pin].Pin_mode == DIO)
              {
                 /* Disable Analog */
                 CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET) , Port_PortPins[Pin].Pin_Num);
                 /* Disable alternative functions */
                 CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , Port_PortPins[Pin].Pin_Num);
                 /*Enable Digital I/O*/
                 SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET) , Port_PortPins[Pin].Pin_Num);
              }
              else if (Port_PortPins[Pin].Pin_mode == ANALOG)
              {
                  /*Enable Analog*/
                  SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET) , Port_PortPins[Pin].Pin_Num);
                  /* Enable alternative functions */
                   SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , Port_PortPins[Pin].Pin_Num);
                  /*Disable Digital I/O*/
                   CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET) , Port_PortPins[Pin].Pin_Num);
              }
              else
              {
                  /* Disable Analog */
                   CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET) , Port_PortPins[Pin].Pin_Num);
                   /* Enable alternative functions */
                   SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET) , Port_PortPins[Pin].Pin_Num);
                   /*Enable Digital I/O*/
                   SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET) , Port_PortPins[Pin].Pin_Num); 
              }
        }
            
         
}