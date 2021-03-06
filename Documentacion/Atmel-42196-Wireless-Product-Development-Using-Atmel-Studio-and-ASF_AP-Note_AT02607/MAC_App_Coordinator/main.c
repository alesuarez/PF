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
#include "tal_helper.h"
#include "sio2host.h"
#include "conf_sio2host.h"
#include "compiler.h"

static void wsn_task(void);
static void app_alert(void);
static void led_off_cb(void *parameter);
static void send_uart_data(void *);
/* Application specific function to assign a short address */
static bool assign_new_short_addr(uint64_t addr64, uint16_t *addr16);
void wsn_monitor_init(void);

#define DEFAULT_CHANNEL					(21)

/** Defines the maximum number of devices the coordinator will handle. */
#define MAX_NUMBER_OF_DEVICES           (6)
#define DEFAULT_PAN_ID                  CCPU_ENDIAN_TO_LE16(0xBABE)

/** Defines the short address of the coordinator. */
#define COORD_SHORT_ADDR                (0x0000)

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

#define SCAN_CHANNEL                    (1ul << DEFAULT_CHANNEL)
/** Defines the scan duration time. */
#define SCAN_DURATION_COORDINATOR       (5)
/** Defines Beacon Order for Nobeacon Network. */
#define NOBEACON_BO                     (15)
/** Defines Superframe Order for Nobeacon Network. */
#define NOBEACON_SO                     (15)
/** Define the LED on duration time. */
#define LED_ON_DURATION                 (500000)
#define APP_CAPTION     "DEMO_BOARD_FFD"
#define APP_CAPTION_SIZE  (sizeof(APP_CAPTION) - 1)
/** This is the time period in micro seconds for sending data to coordinator. */
#define APP_WSN_DATA_PERIOD_MS              (2000)

/** This type definition of a structure stores the short address and the
 *  extended address of a device.
 */
typedef struct associated_device_tag {
	uint16_t short_addr;
	uint64_t ieee_addr;
} associated_device_t;

/** This array stores all device related information. */
static associated_device_t device_list[MAX_NUMBER_OF_DEVICES];

/** Stores the number of associated devices. */
static uint8_t no_of_assoc_devices;

static bool wsn_tx_ready_flag = 0;
static uint8_t *wsn_data;  
static uint8_t TIMER_LED_OFF;
static uint8_t APP_TIMER_WSN;

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
static AppMessage_t msg,rx_msg;

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
	sio2host_init();
	
	/* Initialize LEDs. */
	LED_On(LED_START);     /* indicating application is started */
	LED_Off(LED_NWK_SETUP); /* indicating network is started */
	LED_Off(LED_DATA);     /* indicating data reception */

	cpu_irq_enable();

 	sw_timer_get_id(&TIMER_LED_OFF);
 	sw_timer_get_id(&APP_TIMER_WSN);

	/*
	 * Reset the MAC layer to the default values.
	 * This request will cause a mlme reset confirm message ->
	 * usr_mlme_reset_conf
	 */
	wpan_mlme_reset_req(true);

	while (true) {
		wpan_task();
		wsn_task();
	}
}

void wsn_task(void)
{
	if (wsn_tx_ready_flag)
	{
		/* holds check sum value */
		uint8_t cs = 0;
		wsn_tx_ready_flag = 0;

		putchar(0x10);
		putchar(0x02);
		for (uint8_t i = 0; i < sizeof(AppMessage_t); i++)
		{
			if (wsn_data[i] == 0x10)
			{
				putchar(0x10);
				cs += 0x10;
			}
			putchar(wsn_data[i]);
			cs += wsn_data[i];
		}

		putchar(0x10);
		putchar(0x03);
		cs += 0x10 + 0x02 + 0x10 + 0x03;
		putchar(cs);
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
		sw_timer_start(APP_TIMER_WSN,
		((uint32_t)APP_WSN_DATA_PERIOD_MS * 1000),
		SW_TIMEOUT_RELATIVE,
		(FUNC_PTR)send_uart_data,
		NULL);
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
		/*
		 * Set the short address of this node.
		 * Use: bool wpan_mlme_set_req(uint8_t PIBAttribute,
		 *                             void *PIBAttributeValue);
		 *
		 * This request leads to a set confirm message ->
		 *usr_mlme_set_conf
		 */
		
		uint8_t short_addr[2];

		short_addr[0] = (uint8_t)COORD_SHORT_ADDR;  /* low byte */
		short_addr[1] = (uint8_t)(COORD_SHORT_ADDR >> 8); /* high byte */
		wpan_mlme_set_req(macShortAddress, short_addr);
	}
}

/*
 * @brief Callback function usr_mlme_set_conf
 *
 * @param status        Result of requested PIB attribute set operation
 * @param PIBAttribute  Updated PIB attribute
 */
void usr_mlme_set_conf(uint8_t status,
		uint8_t PIBAttribute)
{
	if ((status == MAC_SUCCESS) && (PIBAttribute == macShortAddress)) {
		/*
		 * Allow other devices to associate to this coordinator.
		 * Use: bool wpan_mlme_set_req(uint8_t PIBAttribute,
		 *                             void *PIBAttributeValue);
		 *
		 * This request leads to a set confirm message ->
		 *usr_mlme_set_conf
		 */
		uint8_t association_permit = true;

		wpan_mlme_set_req(macAssociationPermit, &association_permit);
	} else if ((status == MAC_SUCCESS) &&
			(PIBAttribute == macAssociationPermit)) {
		/*
		 * Set RX on when idle to enable the receiver as default.
		 * Use: bool wpan_mlme_set_req(uint8_t PIBAttribute,
		 *                             void *PIBAttributeValue);
		 *
		 * This request leads to a set confirm message ->
		 *usr_mlme_set_conf
		 */
		bool rx_on_when_idle = true;

		wpan_mlme_set_req(macRxOnWhenIdle, &rx_on_when_idle);
	} else if ((status == MAC_SUCCESS) &&
			(PIBAttribute == macRxOnWhenIdle)) {
		/*
		 * Initiate an active scan over all channels to determine
		 * which channel to use.
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
				DEFAULT_CHANNEL,
				SCAN_DURATION_COORDINATOR,
				TAL_CURRENT_PAGE_DEFAULT);
	} else {
		/* Something went wrong; restart. */
		wpan_mlme_reset_req(true);
	}
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
	/*
	 * We are not interested in the actual scan result,
	 * because we start our network on the pre-defined channel anyway.
	 * Start a nonbeacon-enabled network
	 * Use: bool wpan_mlme_start_req(uint16_t PANId,
	 *                               uint8_t LogicalChannel,
	 *                               uint8_t ChannelPage,
	 *                               uint8_t BeaconOrder,
	 *                               uint8_t SuperframeOrder,
	 *                               bool PANCoordinator,
	 *                               bool BatteryLifeExtension,
	 *                               bool CoordRealignment)
	 *
	 * This request leads to a start confirm message -> usr_mlme_start_conf
	 */
	wpan_mlme_start_req(DEFAULT_PAN_ID,
			DEFAULT_CHANNEL,
			TAL_CURRENT_PAGE_DEFAULT,
			NOBEACON_BO,
			NOBEACON_SO,
			true, false, false);

	/* Keep compiler happy. */
	status = status;
	ScanType = ScanType;
	ChannelPage = ChannelPage;
	UnscannedChannels = UnscannedChannels;
	ResultListSize = ResultListSize;
	ResultList = ResultList;
}
#endif

#if (MAC_START_REQUEST_CONFIRM == 1)

/*
 * @brief Callback function usr_mlme_start_conf
 *
 * @param status        Result of requested start operation
 */
void usr_mlme_start_conf(uint8_t status)
{
	if (status == MAC_SUCCESS) {
		/*
		 * Network is established.
		 * Waiting for association indication from a device.
		 * -> usr_mlme_associate_ind
		 */
		LED_On(LED_NWK_SETUP);
	} else {
		/* Something went wrong; restart. */
		wpan_mlme_reset_req(true);
	}
}
#endif  /* (MAC_START_REQUEST_CONFIRM == 1) */

#if (MAC_ASSOCIATION_INDICATION_RESPONSE == 1)

/*
 * @brief Callback function usr_mlme_associate_ind
 *
 * @param DeviceAddress         Extended address of device requesting
 *association
 * @param CapabilityInformation Capabilities of device requesting association
 */
void usr_mlme_associate_ind(uint64_t DeviceAddress,
		uint8_t CapabilityInformation)
{
	/*
	 * Any device is allowed to join the network.
	 * Use: bool wpan_mlme_associate_resp(uint64_t DeviceAddress,
	 *                                    uint16_t AssocShortAddress,
	 *                                    uint8_t status);
	 *
	 * This response leads to comm status indication ->
	 *usr_mlme_comm_status_ind
	 * Get the next available short address for this device.
	 */
	uint16_t associate_short_addr = macShortAddress_def;

	if (assign_new_short_addr(DeviceAddress,
			&associate_short_addr) == true) {
		wpan_mlme_associate_resp(DeviceAddress,
				associate_short_addr,
				ASSOCIATION_SUCCESSFUL);
	} else {
		wpan_mlme_associate_resp(DeviceAddress, associate_short_addr,
				PAN_AT_CAPACITY);
	}

	/* Keep compiler happy. */
	CapabilityInformation = CapabilityInformation;
}
#endif  /* (MAC_ASSOCIATION_INDICATION_RESPONSE == 1) */

#if ((MAC_ORPHAN_INDICATION_RESPONSE == 1) || \
	(MAC_ASSOCIATION_INDICATION_RESPONSE == 1))

/*
 * @brief Callback function usr_mlme_comm_status_ind
 *
 * @param SrcAddrSpec      Pointer to source address specification
 * @param DstAddrSpec      Pointer to destination address specification
 * @param status           Result for related response operation
 */
void usr_mlme_comm_status_ind(wpan_addr_spec_t *SrcAddrSpec,
		wpan_addr_spec_t *DstAddrSpec,
		uint8_t status)
{
	if (status == MAC_SUCCESS) {
		/*
		 * Now the association of the device has been successful and its
		 * information, like address, could  be stored.
		 * But for the sake of simple handling it has been done
		 * during assignment of the short address within the function
		 * assign_new_short_addr()
		 */
	}

	/* Keep compiler happy. */
	SrcAddrSpec = SrcAddrSpec;
	DstAddrSpec = DstAddrSpec;
}
#endif  /* ((MAC_ORPHAN_INDICATION_RESPONSE == 1) ||
         *(MAC_ASSOCIATION_INDICATION_RESPONSE == 1)) */

/*
 * @brief Callback function usr_mcps_data_ind
 *
 * @param SrcAddrSpec      Pointer to source address specification
 * @param DstAddrSpec      Pointer to destination address specification
 * @param msduLength       Number of octets contained in MSDU
 * @param msdu             Pointer to MSDU
 * @param mpduLinkQuality  LQI measured during reception of the MPDU
 * @param DSN              DSN of the received data frame.
 * @param Timestamp        The time, in symbols, at which the data were received
 *                         (only if timestamping is enabled).
 */
void usr_mcps_data_ind(wpan_addr_spec_t *SrcAddrSpec,
		wpan_addr_spec_t *DstAddrSpec,
		uint8_t msduLength,
		uint8_t *msdu,
		uint8_t mpduLinkQuality,
#ifdef ENABLE_TSTAMP
		uint8_t DSN,
		uint32_t Timestamp)
#else
		uint8_t DSN)
#endif  /* ENABLE_TSTAMP */
{
	/*
	 * Dummy data has been received successfully.
	 */
	LED_On(LED_DATA);

	/* Start a timer switching off the LED. */
	sw_timer_start(TIMER_LED_OFF,
			LED_ON_DURATION,
			SW_TIMEOUT_RELATIVE,
			(FUNC_PTR)led_off_cb,
			NULL);
	
	if (wsn_tx_ready_flag == 0)
	{
		wsn_tx_ready_flag = 1;
		memcpy(&rx_msg, msdu, msduLength);
		rx_msg.lqi = mpduLinkQuality;
		tal_trx_reg_read(RG_PHY_RSSI, (uint8_t *)&rx_msg.rssi);
		wsn_data = (uint8_t *)&rx_msg;
	}
	SrcAddrSpec = SrcAddrSpec;
	DstAddrSpec = DstAddrSpec;
	/* Keep compiler happy. */
	DSN = DSN;
#ifdef ENABLE_TSTAMP
	Timestamp = Timestamp;
#endif  /* ENABLE_TSTAMP */
}

/*
 * @brief Application specific function to assign a short address
 *
 */
static bool assign_new_short_addr(uint64_t addr64, uint16_t *addr16)
{
	uint8_t i;
	/* Check if device has been associated before. */
	for (i = 0; i < MAX_NUMBER_OF_DEVICES; i++) {
		if (device_list[i].short_addr == 0x0000) {
			/* If the short address is 0x0000, it has not been used
			 *before. */
			continue;
		}

		if (device_list[i].ieee_addr == addr64) {
			/* Assign the previously assigned short address again.
			 **/
			*addr16 = device_list[i].short_addr;
			return true;
		}
	}

	for (i = 0; i < MAX_NUMBER_OF_DEVICES; i++) {
		if (device_list[i].short_addr == 0x0000) {
			*addr16 = CPU_ENDIAN_TO_LE16(i + 0x0001);
			device_list[i].short_addr = CPU_ENDIAN_TO_LE16(
					i + 0x0001);                    /* Get
			                                                 *next
			                                                 *short
			                                                 *address.
			                                                 **/
			device_list[i].ieee_addr = addr64; /* Store extended
			                                    *address. */
			no_of_assoc_devices++;
			return true;
		}
	}

	/* If we are here, no short address could be assigned. */
	return false;
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

void wsn_monitor_init(void)
{
	msg.messageType          = 1;
	msg.nodeType             = 0;
	msg.shortAddr            = COORD_SHORT_ADDR;
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

void send_uart_data(void *parameter)
{
	msg.parentShortAddr = 0;
	msg.sensors.battery     = rand();
	msg.sensors.temperature = rand() & 0x7f;
	msg.sensors.light       = rand() & 0xff;
	if (wsn_tx_ready_flag == 0)
	{
		wsn_data = (uint8_t *)&msg;
		wsn_tx_ready_flag = 1;
	}
	
	sw_timer_start(APP_TIMER_WSN,
	((uint32_t)APP_WSN_DATA_PERIOD_MS * 1000),
	SW_TIMEOUT_RELATIVE,
	(FUNC_PTR)send_uart_data,
	NULL);

	parameter = parameter; /* Keep compiler happy. */
}


/*
 * @brief Callback function switching off the LED
 *
 * @param parameter Pointer to callback parameter
 *                  (not used in this application, but could be used
 *                  to indicated LED to be switched off)
 */
static void led_off_cb(void *parameter)
{
	LED_Off(LED_DATA);

	parameter = parameter; /* Keep compiler happy. */
}
