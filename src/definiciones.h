/*
 * definiciones.h
 *
 * Created: 07/05/2015 01:51:43 p.m.
 *  Author: Elio
 */ 


#ifndef DEFINICIONES_H_
#define DEFINICIONES_H_


//DEFINICIONES PROPIAS

//LEDS
#define LED_1 AVR32_PIN_PA04
#define LED_2 AVR32_PIN_PA11
#define LED_3 AVR32_PIN_PB09

//RF SPI
#define SPI_SCK_PIN AVR32_SPI_SCK_0_1_PIN
#define SPI_MISO_PIN AVR32_SPI_MISO_0_1_PIN
#define SPI_MOSI_PIN AVR32_SPI_MOSI_0_1_PIN
#define SPI_CS_PIN AVR32_SPI_NPCS_3_0_PIN
#define SPI_CS 3

#define SPI_SCK_FUNCTION AVR32_SPI_SCK_0_1_FUNCTION
#define SPI_MISO_FUNCTION AVR32_SPI_MISO_0_1_FUNCTION
#define SPI_MOSI_FUNCTION AVR32_SPI_MOSI_0_1_FUNCTION
#define SPI_CS_FUNCTION AVR32_SPI_NPCS_3_0_FUNCTION
#define AT86RFX_IRQ_PIN AVR32_PIN_PA13
#define AT86RFX_SLP_PIN AVR32_PIN_PB07
#define AT86RFX_RST_PIN AVR32_PIN_PB08
#define AT86RFX_SPI (&AVR32_SPI)
#define AT86RFX_SPI_BAUDRATE 1000000

//I2C Sensor temperatura
#define TARGET_ADDRESS		0x48


//DB9 - RS232
#define USART2_RX_PIN AVR32_USART2_RXD_0_1_PIN
#define USART2_TX_PIN AVR32_USART2_TXD_0_1_PIN
#define USART2_RX_FUNCTION AVR32_USART2_RXD_0_1_FUNCTION
#define USART2_TX_FUNCTION AVR32_USART2_TXD_0_1_FUNCTION
#define tamano_cola 200


//! \note TC1 module is used in this example.
#define EXAMPLE_TC                 (&AVR32_TC1)
//! \note TC Channel 0 is used.
#define EXAMPLE_TC_CHANNEL         0
//! \note IRQ0 line of TC1 Channel0 is used.
#define EXAMPLE_TC_IRQ             AVR32_TC1_IRQ0
//! \note IRQ Group of TC1 module
#define EXAMPLE_TC_IRQ_GROUP       AVR32_TC1_IRQ_GROUP
//! \note Interrupt priority 0 is used for TC in this example.
#define EXAMPLE_TC_IRQ_PRIORITY    AVR32_INTC_INT3

//----------------------------------------------------------





//Probando definciones sens temp





#define AT30TSE_TEMPERATURE_REG         0x00
#define AT30TSE_TEMPERATURE_REG_SIZE    2
#define AT30TSE_NON_VOLATILE_REG        0x00
#define AT30TSE_VOLATILE_REG            0x10

#define AT30TSE_CONFIG_REG              0x01
#define AT30TSE_CONFIG_REG_SIZE         2
#define AT30TSE_TLOW_REG                0x02
#define AT30TSE_TLOW_REG_SIZE           2
#define AT30TSE_THIGH_REG               0x03
#define AT30TSE_THIGH_REG_SIZE          2

#define AT30TSE_CONFIG_RES_Pos          13
#define AT30TSE_CONFIG_RES_Msk          (0x03 << AT30TSE_CONFIG_RES_Pos)
#define AT30TSE_CONFIG_RES(value)       ((AT30TSE_CONFIG_RES_Msk & \
		((value) << AT30TSE_CONFIG_RES_Pos)))

#define AT30TSE_CONFIG_RES_9_bit    0
#define AT30TSE_CONFIG_RES_10_bit   1
#define AT30TSE_CONFIG_RES_11_bit   2
#define AT30TSE_CONFIG_RES_12_bit   3

#define AT30TSE_CONFIG_FTQ_Pos      13
#define AT30TSE_CONFIG_FTQ_Msk      (0x03 << AT30TSE_CONFIG_FTQ_Pos)
#define AT30TSE_CONFIG_FTQ(value)   ((AT30TSE_CONFIG_FTQ_Msk & \
		((value) << AT30TSE_CONFIG_FTQ_Pos)))

#define AT30TSE_CONFIG_FTQ_1_fault  0
#define AT30TSE_CONFIG_RES_2_fault  1
#define AT30TSE_CONFIG_RES_4_fault  2
#define AT30TSE_CONFIG_RES_6_fault  3

/* R/W bits. */
#define AT30TSE_CONFIG_OS           (1 << 15)
#define AT30TSE_CONFIG_R1           (1 << 14)
#define AT30TSE_CONFIG_R0           (1 << 13)
#define AT30TSE_CONFIG_FT1          (1 << 12)
#define AT30TSE_CONFIG_FT0          (1 << 11)
#define AT30TSE_CONFIG_POL          (1 << 10)
#define AT30TSE_CONFIG_CMP_INT      (1 << 9)
#define AT30TSE_CONFIG_SD           (1 << 8)

/* Read only bits */
#define AT30TSE_CONFIG_NVRBSY       (1 << 0)

/* definiciones */


// radio statuses
enum{
	RADIO_SUCCESS = 0x40,                       /**< The requested service was performed successfully. */
	RADIO_UNSUPPORTED_DEVICE,                   /**< The connected device is not an Atmel AT86RF212. */
	RADIO_INVALID_ARGUMENT,                     /**< One or more of the supplied function arguments are invalid. */
	RADIO_TIMED_OUT,                            /**< The requested service timed out. */
	RADIO_WRONG_STATE,                          /**< The end-user tried to do an invalid state transition. */
	RADIO_BUSY_STATE,                           /**< The radio transceiver is busy receiving or transmitting. */
	RADIO_STATE_TRANSITION_FAILED,              /**< The requested state transition could not be completed. */
	RADIO_CCA_IDLE,                             /**< Channel is clear, available to transmit a new frame. */
	RADIO_CCA_BUSY,                             /**< Channel busy. */
	RADIO_TRX_BUSY,                             /**< Transceiver is busy receiving or transmitting data. */
	RADIO_BAT_LOW,                              /**< Measured battery voltage is lower than voltage threshold. */
	RADIO_BAT_OK,                               /**< Measured battery voltage is above the voltage threshold. */
	RADIO_CRC_FAILED,                           /**< The CRC failed for the actual frame. */
	RADIO_CHANNEL_ACCESS_FAILURE,               /**< The channel access failed during the auto mode. */
	RADIO_NO_ACK,                               /**< No acknowledge frame was received. */
};
// funciones
enum{
	TIME_RST_PULSE_WIDTH        = 1,
	TIME_P_ON_TO_CLKM_AVAIL     = 380,
	TIME_SLEEP_TO_TRX_OFF       = 240,
	TIME_TRX_OFF_TO_SLEEP       = 35,
	TIME_PLL_ON_TRX_OFF         = 1,
	TIME_TRX_OFF_RX_ON          = 110,
	TIME_RX_ON_TRX_OFF          = 1,
	TIME_PLL_ON_RX_ON           = 1,
	TIME_RX_ON_PLL_ON           = 1,
	TIME_PLL_LOCK_TIME          = 110,
	TIME_BUSY_TX_PLL_ON         = 32,
	TIME_ALL_STATES_TRX_OFF     = 1,
	TIME_RESET_TRX_OFF          = 26,
	TIME_TRX_IRQ_DELAY          = 9,
	TIME_TRX_OFF_PLL_ON         = 110,
	TIME_IRQ_PROCESSING_DLY     = 32
};
enum{
	BAUDRATE    = 1,
	TEMPERATURA = 2
};

typedef struct
{
	uint8_t addr;
	uint8_t cmd;	
	uint8_t payload[2];	
	uint8_t crc;	
}config_package;

void escribir_linea_pc (char*);
static void inicializar_interrupciones();
void spi_init_pins(void);
void estadoPorPc();
#endif /* DEFINICIONES_H_ */