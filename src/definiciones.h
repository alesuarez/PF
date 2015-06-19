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


enum
{
	TRX_STATUS              = 0x01,
	TRX_STATE               = 0x02,
	TRX_CTRL_0              = 0x03,
	TRX_CTRL_1              = 0x04,
	PHY_TX_PWR              = 0x05,
	PHY_RSSI                = 0x06,
	PHY_ED_LEVEL            = 0x07,
	PHY_CC_CCA              = 0x08,
	CCA_THRES               = 0x09,
	RX_CTRL                 = 0x0a,
	SFD_VALUE               = 0x0b,
	TRX_CTRL_2              = 0x0c,
	ANT_DIV                 = 0x0d,
	IRQ_MASK                = 0x0e,
	IRQ_STATUS              = 0x0f,
	VREG_CTRL               = 0x10,
	BATMON                  = 0x11,
	XOSC_CTRL               = 0x12,
	CC_CTRL_0               = 0x13,
	CC_CTRL_1               = 0x14,
	RX_SYN                  = 0x15,
	RF_CTRL_0               = 0x16,
	XAH_CTRL_1              = 0x17,
	FTN_CTRL                = 0x18,
	RF_CTRL_1               = 0x19,
	PLL_CF                  = 0x1a,
	PLL_DCU                 = 0x1b,
	PART_NUM                = 0x1c,
	VERSION_NUM             = 0x1d,
	MAN_ID_0                = 0x1e,
	MAN_ID_1                = 0x1f,
	SHORT_ADDR_0            = 0x20,
	SHORT_ADDR_1            = 0x21,
	PAN_ID_0                = 0x22,
	PAN_ID_1                = 0x23,
	IEEE_ADDR_0             = 0x24,
	IEEE_ADDR_1             = 0x25,
	IEEE_ADDR_2             = 0x26,
	IEEE_ADDR_3             = 0x27,
	IEEE_ADDR_4             = 0x28,
	IEEE_ADDR_5             = 0x29,
	IEEE_ADDR_6             = 0x2a,
	IEEE_ADDR_7             = 0x2b,
	XAH_CTRL_0              = 0x2c,
	CSMA_SEED_0             = 0x2d,
	CSMA_SEED_1             = 0x2e,
	CSMA_BE                 = 0x2f
};







#endif /* DEFINICIONES_H_ */