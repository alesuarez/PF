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
#include <inttypes.h>
#include <stdio.h>
#include <string.h>
#include "conf_board.h"
#include "avr2025_mac.h"
#include "delay.h"
#include "common_sw_timer.h"
#include "tal_constants.h"

/** Defines the PAN ID of the network. */
#define DEFAULT_PAN_ID                  CCPU_ENDIAN_TO_LE16(0xBABE)

/** This is a device which will communicate with a single coordinator.
 *  Therefore, the maximum number of devices this code needs to
 *  handle is one.
 */
#define MAX_NUMBER_OF_DEVICES           (1)

/** Define the LED on duration time. */
#define LED_ON_DURATION                 (500000)

#define CHANNEL_OFFSET                  (0)

#define DEFAULT_CHANNEL					(21)

#define SCAN_CHANNEL                    (1ul << DEFAULT_CHANNEL)

/** Defines the short scan duration time. */
#define SCAN_DURATION_SHORT             (5)
/** Defines the long scan duration time. */
#define SCAN_DURATION_LONG              (6)

#if (LED_COUNT >= 3)
#define LED_START                       (LED0)
#define LED_NWK_SETUP                   (LED1)
#define LED_DATA                        (LED2)
#elif (LED_COUNT == 2)
#define LED_START                       (LED0)
#define LED_NWK_SETUP                   (LED0)
#define LED_DATA                        (LED1)
#else
#define LED_START                       (LED0)
#define LED_NWK_SETUP                   (LED0)
#define LED_DATA                        (LED0)
#endif

/** This is the time period in micro seconds for sending data to coordinator. */
#define APP_DATA_PERIOD_MS              (5000)
#define APP_CAPTION     "DEVICE"
#define APP_CAPTION_SIZE  (sizeof(APP_CAPTION) - 1)

typedef struct AppMessage_t
{
	uint8_t     messageType;
	uint8_t     nodeType;
	uint64_t    extAddr;
	uint16_t    shortAddr;
	uint32_t    softVersion;
	uint32_t    channelMask;
	uint16_t    panId;
	uint8_t     workingChannel;
	uint16_t    parentShortAddr;
	uint8_t     lqi;
	int8_t      rssi;

	struct
	{
		uint8_t   type;
		uint8_t   size;
		int32_t   battery;
		int32_t   temperature;
		int32_t   light;
	} sensors;

	struct
	{
		uint8_t   type;
		uint8_t   size;
		char      text[APP_CAPTION_SIZE];
	} caption;
} AppMessage_t;
static AppMessage_t msg;

/** This structure stores the short address of the coordinator and the device. */
static uint16_t coord_addr, device_addr;

static uint8_t APP_TIMER_SEND_DATA;

/** Alert to indicate something has gone wrong in the application */
static void app_alert(void);
/**
 * @brief function for the sending received data back to coordinator
 *
 * @param data Pointer to received data
 *
 */
static void send_data(void);
static void wsn_monitor_init(void);

/**
 * @brief Main function of the device application
 */
int main(void)
{
	irq_initialize_vectors();
	sysclk_init();

	/* Initialize the board.
	 * The board-specific conf_board.h file contains the configuration of
	 * the board initialization.
	 */
	board_init();

	sw_timer_init();

	if (MAC_SUCCESS != wpan_init()) {
		app_alert();
	}
	
	wsn_monitor_init();
	
	/* Initialize LEDs. */
	LED_On(LED_START);     /* indicating application is started */
	LED_Off(LED_NWK_SETUP); /* indicating node is associated */
	LED_Off(LED_DATA);     /* indicating successfull data transmission */

	cpu_irq_enable();

 	sw_timer_get_id(&APP_TIMER_SEND_DATA);

	wpan_mlme_reset_req(true);

	while (true) {
		wpan_task();
	}
}

/*
 * @brief Callback function usr_mlme_reset_conf
 *
 * @param status Result of the reset procedure
 */
void usr_mlme_reset_conf(uint8_t status)
{
	if (status == MAC_SUCCESS) {
		wpan_mlme_get_req(macIeeeAddress);		
	} else {
		/* Something went wrong; restart. */
		wpan_mlme_reset_req(true);
	}
}

/*
 * Callback function usr_mlme_get_conf
 *
 * @param status            Result of requested PIB attribute get operation.
 * @param PIBAttribute      Retrieved PIB attribute.
 * @param PIBAttributeIndex Index of the PIB attribute to be read.
 * @param PIBAttributeValue Pointer to data containing retrieved PIB attribute,
 *
 * @return void
 */
void usr_mlme_get_conf(uint8_t status,
		uint8_t PIBAttribute,
		void *PIBAttributeValue)
{
		if ((status == MAC_SUCCESS) && (PIBAttribute == macIeeeAddress)) {
			msg.extAddr =  *(uint64_t *)PIBAttributeValue;
		}			
		/*
		 * Initiate an active scan over all channels to determine
		 * which channel is used by the coordinator.
		 * Use: bool wpan_mlme_scan_req(uint8_t ScanType,
		 *                              uint32_t ScanChannels,
		 *                              uint8_t ScanDuration,
		 *                              uint8_t ChannelPage);
		 *
		 * This request leads to a scan confirm message ->
		 *usr_mlme_scan_conf
		 * Scan for about 50 ms on each channel -> ScanDuration = 1
		 * Scan for about 1/2 second on each channel -> ScanDuration = 5
		 * Scan for about 1 second on each channel -> ScanDuration = 6
		 */
		 wpan_mlme_scan_req(MLME_SCAN_TYPE_ACTIVE,
				SCAN_CHANNEL,
				SCAN_DURATION_SHORT,
				TAL_CURRENT_PAGE_DEFAULT);
}

#if ((MAC_SCAN_ED_REQUEST_CONFIRM == 1)      ||	\
	(MAC_SCAN_ACTIVE_REQUEST_CONFIRM == 1)  || \
	(MAC_SCAN_PASSIVE_REQUEST_CONFIRM == 1) || \
	(MAC_SCAN_ORPHAN_REQUEST_CONFIRM == 1))

/*
 * @brief Callback function usr_mlme_scan_conf
 *
 * @param status            Result of requested scan operation
 * @param ScanType          Type of scan performed
 * @param ChannelPage       Channel page on which the scan was performed
 * @param UnscannedChannels Bitmap of unscanned channels
 * @param ResultListSize    Number of elements in ResultList
 * @param ResultList        Pointer to array of scan results
 */
void usr_mlme_scan_conf(uint8_t status,
		uint8_t ScanType,
		uint8_t ChannelPage,
		uint32_t UnscannedChannels,
		uint8_t ResultListSize,
		void *ResultList)
{
	if (status == MAC_SUCCESS) {
		wpan_pandescriptor_t *coordinator;
		uint8_t i;

		/*
		 * Analyze the ResultList.
		 * Assume that the first entry of the result list is our
		 *coordinator.
		 */
		coordinator = (wpan_pandescriptor_t *)ResultList;

		for (i = 0; i < ResultListSize; i++) {
			/*
			 * Check if the PAN descriptor belongs to our
			 *coordinator.
			 * Check if coordinator allows association.
			 */
			if ((coordinator->LogicalChannel == DEFAULT_CHANNEL) &&
					(coordinator->ChannelPage ==
					TAL_CURRENT_PAGE_DEFAULT) &&
					(coordinator->CoordAddrSpec.PANId ==
					DEFAULT_PAN_ID) &&
					((coordinator->SuperframeSpec &
					((uint16_t)1 <<
					ASSOC_PERMIT_BIT_POS)) ==
					((uint16_t)1 << ASSOC_PERMIT_BIT_POS))
					) {
				/* Store the coordinator's address information.
				 **/
				if (coordinator->CoordAddrSpec.AddrMode ==
						WPAN_ADDRMODE_SHORT) {
					coord_addr
						= coordinator->CoordAddrSpec.
							Addr
							.short_address;
				} else {
					/* Something went wrong; restart. */
					wpan_mlme_reset_req(true);
					return;
				}

				/*
				 * Associate to our coordinator.
				 * Use: bool wpan_mlme_associate_req(uint8_t
				 *LogicalChannel,
				 *                                   uint8_t
				 *ChannelPage,
				 *
				 *
				 *
				 *
				 *
				 *                              wpan_addr_spec_t
				 **CoordAddrSpec,
				 *                                   uint8_t
				 *CapabilityInformation);
				 * This request will cause a mlme associate
				 *confirm message ->
				 * usr_mlme_associate_conf.
				 */
				wpan_mlme_associate_req(
						coordinator->LogicalChannel,
						coordinator->ChannelPage,
						&(coordinator->CoordAddrSpec),
						WPAN_CAP_ALLOCADDRESS);
				return;
			}

			/* Get the next PAN descriptor. */
			coordinator++;
		}

		/*
		 * If here, the result list does not contain our expected
		 *coordinator.
		 * Let's scan again.
		 */
		wpan_mlme_scan_req(MLME_SCAN_TYPE_ACTIVE,
				SCAN_CHANNEL,
				SCAN_DURATION_SHORT,
				TAL_CURRENT_PAGE_DEFAULT);
	} else if (status == MAC_NO_BEACON) {
		/*
		 * No beacon is received; no coordiantor is located.
		 * Scan again, but used longer scan duration.
		 */
		wpan_mlme_scan_req(MLME_SCAN_TYPE_ACTIVE,
				SCAN_CHANNEL,
				SCAN_DURATION_LONG,
				TAL_CURRENT_PAGE_DEFAULT);
	} else {
		/* Something went wrong; restart. */
		wpan_mlme_reset_req(true);
	}

	/* Keep compiler happy. */
	ScanType = ScanType;
	ChannelPage = ChannelPage;
	UnscannedChannels = UnscannedChannels;
}
#endif

#if (MAC_ASSOCIATION_REQUEST_CONFIRM == 1)

/*
 * Callback function usr_mlme_associate_conf.
 *
 * @param AssocShortAddress    Short address allocated by the coordinator.
 * @param status               Result of requested association operation.
 *
 * @return void
 *
 */
void usr_mlme_associate_conf(uint16_t AssocShortAddress,
		uint8_t status)
{
	if (status == MAC_SUCCESS) {
				/* Start a timer that polls for pending data at the coordinator.
		 **/
		sw_timer_start(APP_TIMER_SEND_DATA,
				((uint32_t)APP_DATA_PERIOD_MS * 1000),
				SW_TIMEOUT_RELATIVE,
				(FUNC_PTR)send_data,
				NULL);
		device_addr = AssocShortAddress;
		msg.shortAddr = device_addr;
	} else {
		/* Something went wrong; restart. */
		wpan_mlme_reset_req(true);
	}
}

#endif  /* (MAC_ASSOCIATION_REQUEST_CONFIRM == 1) */

/*
 * @brief function for the sending data to coordinator
 *
 * @param data Pointer to received data
 *
 */
static void send_data(void)
{
	/*
	 * Send some data and restart timer.
	 * Use: bool wpan_mcps_data_req(uint8_t SrcAddrMode,
	 *                              wpan_addr_spec_t *DstAddrSpec,
	 *                              uint8_t msduLength,
	 *                              uint8_t *msdu,
	 *                              uint8_t msduHandle,
	 *                              uint8_t TxOptions);
	 *
	 * This request will cause a mcps data confirm message ->
	 * usr_mcps_data_conf
	 */

	uint8_t src_addr_mode;
	wpan_addr_spec_t dst_addr;
	static uint8_t msduHandle = 0;

	src_addr_mode = WPAN_ADDRMODE_SHORT;

	dst_addr.AddrMode = WPAN_ADDRMODE_SHORT;
	dst_addr.PANId = DEFAULT_PAN_ID;
	ADDR_COPY_DST_SRC_16(dst_addr.Addr.short_address, coord_addr);
	msduHandle++;
	
	msg.parentShortAddr = 0;
	msg.sensors.battery     = rand();
	msg.sensors.temperature = rand() & 0x7f;
	msg.sensors.light       = rand() & 0xff;
		
	wpan_mcps_data_req(src_addr_mode,
			&dst_addr,
			sizeof(msg),
			(uint8_t *)&msg,
			msduHandle,
			WPAN_TXOPT_ACK);
			
		sw_timer_start(APP_TIMER_SEND_DATA,
		((uint32_t)APP_DATA_PERIOD_MS * 1000),
		SW_TIMEOUT_RELATIVE,
		(FUNC_PTR)send_data,
		NULL);			
}

static void wsn_monitor_init(void)
{
	msg.messageType          = 1;
	msg.nodeType             = 2;
	msg.softVersion          = 0x01010100;
	msg.channelMask          = (1L << DEFAULT_CHANNEL);
	msg.panId                = DEFAULT_PAN_ID;
	msg.workingChannel       = DEFAULT_CHANNEL;
	msg.parentShortAddr      = 0;
	msg.lqi                  = 0;
	msg.rssi                 = 0;

	msg.sensors.type        = 1;
	msg.sensors.size        = sizeof(int32_t) * 3;
	msg.sensors.battery     = 0;
	msg.sensors.temperature = 0;
	msg.sensors.light       = 0;

	msg.caption.type         = 32;
	msg.caption.size         = APP_CAPTION_SIZE;
	memcpy(msg.caption.text, APP_CAPTION, APP_CAPTION_SIZE);
}

/* Alert to indicate something has gone wrong in the application */
static void app_alert(void)
{
	while (1) {
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
