/**
 * \file
 *
 * \brief Empty user application template
 *
 */

/**
 * \mainpage User Application template doxygen documentation
 *
 * \par Empty user application template
 *
 * Bare minimum empty user application template
 *
 * \par Content
 *
 * -# Include the ASF header files (through asf.h)
 * -# Minimal main function that starts with a call to board_init()
 * -# "Insert application code here" comment
 *
 */

/*
 * Include header files for all drivers that have been imported from
 * Atmel Software Framework (ASF).
 */
#include <asf.h>
#include <string.h>
#include "tal.h"
#include "tal_helper.h"
#include "tal_internal.h"
#include "common_sw_timer.h"
#include "ieee_const.h"
#include "tal_constants.h"
#include "compiler.h"

#define DEBOUNCE_DELAY_MS               (200)
#define DEFAULT_CHANNEL                 CCPU_ENDIAN_TO_LE16(21)
#define FRAME_CONTROL_FIELD             CCPU_ENDIAN_TO_LE16(0x8001)
#define DEFAULT_PAN_ID					CCPU_ENDIAN_TO_LE16(0xCAFE)
#define SOURCE_ADDRESS                  CCPU_ENDIAN_TO_LE16(0xBABE)


typedef struct
{
	uint8_t  phr;   /**< PHY header (frame length field). */
	uint16_t fcf;   /**< Frame control field */
	uint8_t  seq;   /**< Frame sequence number. */
	uint16_t span;  /**< source PAN identifier */
	uint16_t saddr; /**< source address */
	char  data[5]; /**< Frame payload */
	uint16_t crc;   /**< CRC16 field of the frame.  */	
} app_frame_t;

/** Alert to indicate something has gone wrong in the application */
static void app_alert(void);
void app_task(void);

bool tx_ready_flag = 1;
static uint8_t seq_num = 0;
frame_info_t tx_frame_info;
app_frame_t tx_frame, rx_frame;
uint8_t msg[5] = "Hello";

int main(void)
{
	irq_initialize_vectors();
	
	sysclk_init();

	/* Initialize the board.
	 * The board-specific conf_board.h file contains the configuration of
	 * the board initialization.
	 */
	board_init();

	/* Initialize the software timer.
	 * The conf_hw_timer.h,conf_common_sw_timer.h and app_config.h file 
	 * contains the configuration for the timer initialization.
	 */
	sw_timer_init();

	/* Initialize the TAL layer */
	if (tal_init() != MAC_SUCCESS) {
		/* something went wrong during initialization */
		app_alert();
	}

	/* Channel default configuration  */
	uint8_t temp = DEFAULT_CHANNEL;
	tal_pib_set(phyCurrentChannel, (pib_value_t *)&temp);

	/* Channel page default configuration*/
	temp = TAL_CURRENT_PAGE_DEFAULT;
	tal_pib_set(phyCurrentPage, (pib_value_t *)&temp);

	/* Tx power default configurations */
	temp = TAL_TRANSMIT_POWER_DEFAULT;
	tal_pib_set(phyTransmitPower, (pib_value_t *)&temp);
    
	/* Sets transceiver state */
	set_trx_state(CMD_RX_ON);	
	pal_global_irq_enable();
	
	while(1)
	{
		pal_task(); /* Handle platform specific tasks, like serial interface */
		tal_task(); /* Handle transceiver specific tasks */
		app_task(); /* Application task */		
	}

}

void app_task(void)
{
	if (!ioport_get_pin_level(GPIO_PUSH_BUTTON_ON_BOARD))
	{
		delay_ms(DEBOUNCE_DELAY_MS);
		if (!ioport_get_pin_level(GPIO_PUSH_BUTTON_ON_BOARD))
		{
			if (tx_ready_flag == 1)
			{
				tx_ready_flag = 0;
				tx_frame_info.msg_type = DATAREQUEST;
				tx_frame_info.msduHandle = seq_num;
				tx_frame.phr  = (sizeof(app_frame_t));
				tx_frame.fcf  = CCPU_ENDIAN_TO_LE16(FRAME_CONTROL_FIELD);
				tx_frame.seq  = seq_num;
				tx_frame.span = CCPU_ENDIAN_TO_LE16(DEFAULT_PAN_ID);
				tx_frame.saddr= CCPU_ENDIAN_TO_LE16(SOURCE_ADDRESS);
				memcpy(&tx_frame.data, &msg, sizeof(msg));
				tx_frame_info.mpdu = (uint8_t *)&tx_frame;
				tal_tx_frame(&tx_frame_info, CSMA_UNSLOTTED,false);
			}

		}
	}
	
}

void tal_tx_frame_done_cb(retval_t status, frame_info_t *frame)
{
	if (status == MAC_SUCCESS)
	{
		seq_num++;
	}
	tx_ready_flag = 1;
	/* free buffer that was used for frame reception */
	bmm_buffer_free((buffer_t *)(frame->buffer_header));
	set_trx_state(CMD_RX_ON);
}

void tal_rx_frame_cb(frame_info_t *frame)
{
	if (CRC16_VALID == pal_trx_bit_read(SR_RX_CRC_VALID))
	{
		memset(&rx_frame,0,sizeof(rx_frame));
		memcpy(&rx_frame,frame->mpdu, sizeof(rx_frame));
		if (rx_frame.span  == CCPU_ENDIAN_TO_LE16(DEFAULT_PAN_ID) &&
			rx_frame.saddr == CCPU_ENDIAN_TO_LE16(SOURCE_ADDRESS))
		{
			LED_Toggle(LED0);
		}
	}
	/* free buffer that was used for frame reception */
	bmm_buffer_free((buffer_t *)(frame->buffer_header));
}

/* Alert to indicate something has gone wrong in the application */
static void app_alert(void)
{
	while (1)
	{
		#if LED_COUNT > 0
		LED_Toggle(LED0);
		#endif

		#if LED_COUNT > 1
		LED_Toggle(LED1);
		#endif

		#if LED_COUNT > 2
		LED_Toggle(LED2);
		#endif

		#if LED_COUNT > 3
		LED_Toggle(LED3);
		#endif

		#if LED_COUNT > 4
		LED_Toggle(LED4);
		#endif

		#if LED_COUNT > 5
		LED_Toggle(LED5);
		#endif

		#if LED_COUNT > 6
		LED_Toggle(LED6);
		#endif

		#if LED_COUNT > 7
		LED_Toggle(LED7);
		#endif
		delay_us(0xFFFF);
	}
}

