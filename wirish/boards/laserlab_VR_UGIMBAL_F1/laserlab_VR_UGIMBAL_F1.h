#ifndef _BOARD_LASERLAB_VR_UGIMBAL_F1_H_
#define _BOARD_LASERLAB_VR_UGIMBAL_F1_H_

#include <hal.h>


#define CYCLES_PER_MICROSECOND  (SystemCoreClock/1000000)
#define SYSTICK_RELOAD_VAL      (CYCLES_PER_MICROSECOND*1000-1)

void boardInit(void);

#undef  STM32_PCLK1
#undef  STM32_PCLK2
/* APB1 and APB2 Prescaler are set to 1 */
#define STM32_PCLK1   (SystemCoreClock)
#define STM32_PCLK2   (SystemCoreClock)

/* LEDs ********************************************/
#define BOARD_LED_RED_PIN       46  //PC14
#define BOARD_LED_GRE_PIN       47  //PC15

//




//#define BOARD_IR_CMD

/* MOTORs ********************************************/

#define BOARD_MOTOR_EN	        42  //PC10

#define BOARD_MOTOR_COUNT 2

#define BOARD_MOT1_CMD_A        21  //PB5
#define BOARD_MOT1_CMD_B        16  //PB0
#define BOARD_MOT1_CMD_C        17  //PB1
//#define BOARD_MOT1_ISENSE

#define BOARD_MOT2_CMD_A        38  //PC6
#define BOARD_MOT2_CMD_B        39  //PC7
#define BOARD_MOT2_CMD_C        40  //PC8
//#define BOARD_MOT2_ISENSE










//#define BOARD_MOT3_CMD_A
//#define BOARD_MOT3_CMD_B
//#define BOARD_MOT3_CMD_C
//#define BOARD_MOT3_ISENSE



/* USB *********************************************/
#define BOARD_USB_DISC          36  //PC4  --> mappato ma non connesso!!!
#define BOARD_USB_DISC_DEV      _GPIOC
#define BOARD_USB_DISC_BIT      4
#define BOARD_USBM  	    	11  //PA11
#define BOARD_USBP	    	    12  //PA12


/* ANALOG *********************************************/
#define BOARD_ANALOG_COUNT 0

#define BOARD_ANA_VBAT			4 //PA4 //#define BOARD_ANA1
//#define BOARD_ANA2
//#define BOARD_ANA3
//#define BOARD_ANA4
//#define BOARD_ANA5
//#define BOARD_ANA6

#define BOARD_CS_SDCARD			5	//PA5

#define BOARD_EXT_VREF			37	//PC5
#define BOARD_PULS_1			45	//PC13

/* REMOTE COMMAND INPUT ********************************/








#define BOARD_PWM_IN0            0  //PA0
#define BOARD_PWM_IN1  	         1  //PA1
#define BOARD_PWM_IN2      	     2  //PA2
#define BOARD_PWM_IN3          	 3  //PA3


/* IMU *********************************************/
//#define BOARD_INT_IMU
//#define BOARD_INERTIALSENSOR_INT  BOARD_INT_IMU


//IMU SPI
//#define BOARD_CS_IMU_SPI
//#define BOARD_INT_IMU_SPI


//vedi config I2C sotto


/* EEPROM *********************************************/
#define BOARD_CS_EEPROM	        28  //PB12

#define BOARD_EEPROM_SIZE		512 //128
//vedi config SPI sotto

/* USARTs *********************************************/
//Serial Port for CLI
#define SERIAL_CLI_PORT			 4
#define SERIAL_CLI_BAUD			115200 //9600





#define BOARD_NR_USARTS          5

#define BOARD_USART1_TX_PIN      9  //PA9 	//NOT USED
#define BOARD_USART1_RX_PIN     10  //PA10 	//NOT USED
#define BOARD_USART2_TX_PIN      2			//NOT USED
#define BOARD_USART2_RX_PIN      3			//NOT USED
#define BOARD_USART3_TX_PIN     26			//NOT USED
#define BOARD_USART3_RX_PIN     27			//NOT USED
#define BOARD_UART4_TX_PIN      42			//NOT USED
#define BOARD_UART4_RX_PIN      43			//NOT USED
#define BOARD_UART5_TX_PIN      44	//PC12
#define BOARD_UART5_RX_PIN      50	//PD2
/* SPIs *****************************************************/
#define BOARD_NR_SPI             2

#define BOARD_SPI1_SCK_PIN       5			//NOT USED
#define BOARD_SPI1_MISO_PIN      6			//NOT USED
#define BOARD_SPI1_MOSI_PIN      7			//NOT USED
#define BOARD_SPI2_SCK_PIN      29	//PB13
#define BOARD_SPI2_MISO_PIN     30	//PB14
#define BOARD_SPI2_MOSI_PIN     31	//PB15

//#define BOARD_SPI1_CS_DF_PIN    83
//#define BOARD_SPI1_CS_BR_PIN    94

/* I2Cs *****************************************************/
#define BOARD_NR_I2C             2

#define BOARD_I2C1_SCL_PIN      22 //PB6 //24
#define BOARD_I2C1_SDA_PIN      23 //PB7 //25
#define BOARD_I2C2_SCL_PIN      26 //PB10
#define BOARD_I2C2_SDA_PIN      27 //PB11



/************************************************************/
#define BOARD_NR_GPIO_PINS      51





#endif
