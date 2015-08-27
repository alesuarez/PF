//Inicializa modulos RF, TEMPERATURA Y RS232

//EN CASO DE RECIBIR EL CARACTER 't' ENVIA LA TEMPERATURA MEDIDA



/*
 * Include header files for all drivers that have been imported from
 * Atmel Software Framework (ASF).
 */
/*
 * Support and FAQ: visit <a href="http://www.atmel.com/design-support/">Atmel Support</a>
 */
#include <asf.h>
#include <stdio.h>
#include <string.h>

#include "ASF/common/components/at86rfx_driver.h"
#include "conf_at86rfx.h"
#include "definiciones.h"
#include "ASF/common/components/at86rf212.h"

#define BUFFER_SIZE       (15)

char cola_PC[tamano_cola];
int cola_PC_nw = 0;
int cola_PC_nr = 0;

volatile static uint32_t tc_tick = 1;

volatile avr32_tc_t *tc = EXAMPLE_TC;

volatile uint8_t resolution = AT30TSE_CONFIG_RES_12_bit;

uint8_t tx_buffer[5]="hola";

uint8_t TRX_STATUS = 0;
uint8_t register_value = 0;
uint8_t TRX_CTRL_0 = 0;
uint8_t PHY_TX_PWR = 0;
uint8_t PHY_CC_CCA = 0;

uint8_t IRQ_MASK= 0;
uint8_t irq_status= 0;
uint8_t algo=0;
uint8_t algo2=0;
uint8_t TRX_CTRL_2=0;
uint8_t aux2=0;
uint8_t tx_end=0;
uint8_t variable1;
uint8_t variable2;
uint8_t variable3;
uint8_t TX_;
usart_options_t usart_opt = {
	//! Baudrate is set in the conf_example_usart.h file.
	.baudrate    = 9600,
	.channelmode = USART_NORMAL_CHMODE, //ECHO UART
	.charlength  = 8,
	.paritytype  = USART_NO_PARITY,
	.stopbits    = USART_1_STOPBIT,
};
//eic = External Interrupt Controller
eic_options_t eic_options2 = {    
	// Enable level-triggered interrupt.
	.eic_mode   = EIC_MODE_LEVEL_TRIGGERED,
	// Interrupt will trigger on low-level.
	.eic_level  = EIC_LEVEL_HIGH_LEVEL,
	// Enable filter.
	.eic_filter = EIC_FILTER_ENABLED,
	// For Wake Up mode, initialize in asynchronous mode
	.eic_async  = EIC_ASYNCH_MODE,
	// Choose External Interrupt Controller Line
	.eic_line   = AVR32_EIC_INT2
};


//-----------------------------------------------------------

#if defined (__GNUC__)
__attribute__((__interrupt__))
#elif defined (__ICCAVR32__)
#pragma handler = EXAMPLE_TC_IRQ_GROUP, 1
__interrupt
#endif

// Existe la variable timer_eneable que controla la ejecucion de los procesos dentro de la interrupcion
static void tc_irq(void)
{
	// Clear the interrupt flag. This is a side effect of reading the TC SR.
	tc_read_sr(EXAMPLE_TC, EXAMPLE_TC_CHANNEL);

	tc_tick++;	// contador para controlar el tiempo de las interrupciones
	
	if (tc_tick < 20)
	{
		// la funcion toggle pin (parece) que pone el pin en 1
		gpio_toggle_pin(AVR32_PIN_PB09); // PB09 = led 3
		gpio_toggle_pin(AVR32_PIN_PA04); // PA04 = led 1
		gpio_toggle_pin(AVR32_PIN_PA11); // PA11 = led 2 
		return;
	}
	
	if (tc_tick == 21)	
		gpio_toggle_pin(AVR32_PIN_PB09);
		
	if (tc_tick == 22)
		gpio_toggle_pin(AVR32_PIN_PA04);
	
	if (tc_tick == 23)
	{
		gpio_toggle_pin(AVR32_PIN_PA11);	
		tc_tick = 1;
	}
	
		

	
	
	
}

#if __GNUC__
__attribute__((__interrupt__))
#elif __ICCAVR32__
__interrupt
#endif

// Manejo INTERRUPCION externa 2 (RF)

// PA13/GPIO 13/GLOC-OUT[0]/GLOC-IN[7]/TC0-A0/SCIF-GCLK[2]/PWMA-PWMA[13]/CAT-SMP/EIC-EXTINT[2]/CAT-CSA[0]/XIN32_2
static void eic_int_handler2(void)
{
		uint8_t IRQ_STATUS;
		// Interrupt Line must be cleared to enable
		eic_clear_interrupt_line(&AVR32_EIC, AVR32_EIC_INT2);
		//IRQ2 Pin 26 MCU --> Pin 24 T
		IRQ_STATUS = pal_trx_reg_read(RG_IRQ_STATUS);
		switch (IRQ_STATUS)
		{
			case TRX_IRQ_TRX_END:
				escribir_linea_pc("Trama enviada\r\n");
			break;
			case TRX_IRQ_RX_START:
				rxTrama();
			break;
		}
}

#if defined (__GNUC__)
__attribute__((__interrupt__))
#elif defined(__ICCAVR32__)
__interrupt
#endif


// Manejo INTERRUPCION UART
static void usart_int_handler_RS232(void)
{
	// TDW sensor de temperatura -> RX UART2 Pin 24 MCU
	tc_stop(tc,EXAMPLE_TC_CHANNEL);
	
	int c=0;
	/*
	 * In the code line below, the interrupt priority level does not need to
	 * be explicitly masked as it is already because we are within the
	 * interrupt handler.
	 * The USART Rx interrupt flag is cleared by side effect when reading
	 * the received character.
	 * Waiting until the interrupt has actually been cleared is here useless
	 * as the call to usart_putchar will take enough time for this before
	 * the interrupt handler is left and the interrupt priority level is
	 * unmasked by the CPU.
	 */
		
	if (usart_read_char(&AVR32_USART2, &c) != USART_SUCCESS) //aqui lee el caracter por el puerto uart2
		return;

	
		cola_PC[cola_PC_nw] = (char) c;
		cola_PC_nw++;	
	
		if (cola_PC_nw >= tamano_cola)
			cola_PC_nw = 0;
	
	tc_start(tc,EXAMPLE_TC_CHANNEL);
	return;

	
}



void escribir_linea_pc (char *str)
{
	usart_write_line(&AVR32_USART2,str);
}


static void inicializar_interrupciones()
{
	// Disable all interrupts.
	Disable_global_interrupt();
	// Initialize interrupt vectors.
	INTC_init_interrupts();

	/*
	 * Register the USART interrupt handler to the interrupt controller.
	 * usart_int_handler is the interrupt handler to register.
	 * EXAMPLE_USART_IRQ is the IRQ of the interrupt handler to register.
	 * AVR32_INTC_INT0 is the interrupt priority level to assign to the
	 * group of this IRQ.
	 */
	
	INTC_register_interrupt(&usart_int_handler_RS232, AVR32_USART2_IRQ, AVR32_INTC_INT0);

	INTC_register_interrupt(&eic_int_handler2, AVR32_EIC_IRQ_2, AVR32_INTC_INT2);
	
	// Register the TC interrupt handler to the interrupt controller.
	INTC_register_interrupt(&tc_irq, EXAMPLE_TC_IRQ, EXAMPLE_TC_IRQ_PRIORITY);
	
	// INTERRUPCIONES EXTERNAS (#2)
	eic_init(&AVR32_EIC, &eic_options2,1);

	// Enable External Interrupt Controller Line
	eic_enable_line(&AVR32_EIC, AVR32_EIC_INT2);
	eic_enable_interrupt_line(&AVR32_EIC, AVR32_EIC_INT2);	
	
	// Enable USART Rx interrupt.
	(&AVR32_USART2)->ier = AVR32_USART_IER_RXRDY_MASK;
	
	// Enable all interrupts.
	Enable_global_interrupt();
}

static void tc_init(volatile avr32_tc_t *tc)
{
	// Options for waveform generation.
	static const tc_waveform_opt_t waveform_opt = {
		// Channel selection.
		.channel  = EXAMPLE_TC_CHANNEL,
		// Software trigger effect on TIOB.
		.bswtrg   = TC_EVT_EFFECT_NOOP,
		// External event effect on TIOB.
		.beevt    = TC_EVT_EFFECT_NOOP,
		// RC compare effect on TIOB.
		.bcpc     = TC_EVT_EFFECT_NOOP,
		// RB compare effect on TIOB.
		.bcpb     = TC_EVT_EFFECT_NOOP,
		// Software trigger effect on TIOA.
		.aswtrg   = TC_EVT_EFFECT_NOOP,
		// External event effect on TIOA.
		.aeevt    = TC_EVT_EFFECT_NOOP,
		// RC compare effect on TIOA.
		.acpc     = TC_EVT_EFFECT_NOOP,
		/*
		 * RA compare effect on TIOA.
		 * (other possibilities are none, set and clear).
		 */
		.acpa     = TC_EVT_EFFECT_NOOP,
		/*
		 * Waveform selection: Up mode with automatic trigger(reset)
		 * on RC compare.
		 */
		.wavsel   = TC_WAVEFORM_SEL_UP_MODE_RC_TRIGGER,
		// External event trigger enable.
		.enetrg   = false,
		// External event selection.
		.eevt     = 0,
		// External event edge selection.
		.eevtedg  = TC_SEL_NO_EDGE,
		// Counter disable when RC compare.
		.cpcdis   = false,
		// Counter clock stopped with RC compare.
		.cpcstop  = false,
		// Burst signal selection.
		.burst    = false,
		// Clock inversion.
		.clki     = false,
		// Internal source clock 5, connected to fPBA / 128.
		.tcclks   = TC_CLOCK_SOURCE_TC5
	};

	// Options for enabling TC interrupts
	static const tc_interrupt_t tc_interrupt = {
		.etrgs = 0,
		.ldrbs = 0,
		.ldras = 0,
		.cpcs  = 1, // Enable interrupt on RC compare alone
		.cpbs  = 0,
		.cpas  = 0,
		.lovrs = 0,
		.covfs = 0
	};
	
	// Initialize the timer/counter.
	tc_init_waveform(tc, &waveform_opt);

	/*
	 * Set the compare triggers.
	 * We configure it to count every 200 millisechonds.
	 * We want: (1 / (fPBA / 128)) * RC = 200 ms, hence RC = (fPBA / 128) / 5
	 * to get an interrupt every 10 ms.
	 */
	
		
	tc_write_rc(tc, EXAMPLE_TC_CHANNEL,((sysclk_get_pba_hz() / 128) /5));
	// configure the timer interrupt
	tc_configure_interrupts(tc, EXAMPLE_TC_CHANNEL, &tc_interrupt);
	// Start the timer/counter.
	tc_start(tc, EXAMPLE_TC_CHANNEL);
}


void spi_init_pins(void)
{
	gpio_map_t SPI_GPIO_MAP =
	{
		{SPI_SCK_PIN,  SPI_SCK_FUNCTION },  // SPI Clock.
		{SPI_MISO_PIN, SPI_MISO_FUNCTION},  // MISO.
		{SPI_MOSI_PIN, SPI_MOSI_FUNCTION},  // MOSI.
		{SPI_CS_PIN, SPI_CS_FUNCTION}  // CS.
	};

	gpio_enable_module( SPI_GPIO_MAP,sizeof (SPI_GPIO_MAP) / sizeof (SPI_GPIO_MAP[0]));
}
void spi_init_module(void)
{
	spi_options_t spiOptions =
	{
		.reg          = 0,
		.baudrate     = AT86RFX_SPI_BAUDRATE,
		.bits         = 8,
		.spck_delay   = 0,
		.trans_delay  = 0,
		.stay_act     = 1,
		.spi_mode     = 0,
		.modfdis      = 0
	};

	spi_master_init(AT86RFX_SPI);
	
	spiOptions.reg= SPI_CS;
	spi_setupChipReg(AT86RFX_SPI, &spiOptions, sysclk_get_peripheral_bus_hz(&AVR32_SPI));
	spi_selectionMode(AT86RFX_SPI, 0, 0, 0);
	spi_enable(AT86RFX_SPI);
}

void led_init_pins(void)
{
		gpio_configure_pin(LED_1, (GPIO_DIR_OUTPUT | GPIO_PULL_UP));	
		gpio_configure_pin(LED_2, (GPIO_DIR_OUTPUT | GPIO_PULL_UP));
		gpio_configure_pin(LED_3, (GPIO_DIR_OUTPUT | GPIO_PULL_UP));
}

void init_i2c_pins(void)
{
	const gpio_map_t TWI_GPIO_MAP = {
		{AVR32_TWIMS0_TWCK_0_3_PIN, AVR32_TWIMS0_TWCK_0_3_FUNCTION},
		{AVR32_TWIMS0_TWD_0_1_PIN, AVR32_TWIMS0_TWD_0_1_FUNCTION}
	};

	gpio_enable_module (TWI_GPIO_MAP,
			sizeof (TWI_GPIO_MAP) / sizeof (TWI_GPIO_MAP[0]));
			
}
void init_i2c_module(void)
{
	sysclk_enable_pba_module(SYSCLK_TWIM0);
	
	const twi_options_t TWIM_OPTIONS = 
	{
		.pba_hz = sysclk_get_cpu_hz(),
		.speed = 10000,
		.chip = TARGET_ADDRESS,
		.smbus = 0,
	};
	
	// Initialize as master.
	int status = twim_master_init (&AVR32_TWIM0, &TWIM_OPTIONS);
	
	// Check whether TARGET device is connected
	
	if (status == STATUS_OK) {
		// display test result to user
		escribir_linea_pc("Sensor Temp:\tPASS\r\n");
	} else {
		// display test result to user
		escribir_linea_pc("Sensor Temp:\tFAILED\r\n");
	}
} 

void init_rf_pins(void)
{
	//Configuracion de los pines para SPI
	spi_init_pins();

	//PIN para interrupcion externa RF PA13-> IRQ2
	gpio_configure_pin (AVR32_PIN_PA13, (GPIO_DIR_INPUT | GPIO_PULL_UP)); // PA13 IRQ2
	gpio_enable_module_pin(AVR32_EIC_EXTINT_2_0_PIN, AVR32_EIC_EXTINT_2_0_FUNCTION); // Habilito interrupcion externa con este pin
//	gpio_enable_pin_interrupt(AT86RFX_IRQ_PIN, GPIO_RISING_EDGE);
	gpio_clear_pin_interrupt_flag(AT86RFX_IRQ_PIN);

// 	gpio_configure_pin(AT86RFX_RST_PIN, GPIO_DIR_OUTPUT | GPIO_INIT_HIGH);
// 	gpio_configure_pin(AT86RFX_SLP_PIN, GPIO_DIR_OUTPUT | GPIO_INIT_HIGH);	
// 	

}

void rs_232_init_pins(void)
{
	gpio_map_t COMPORT0_GPIO_MAP =
	{
		{ USART2_RX_PIN, USART2_RX_FUNCTION },
		{ USART2_TX_PIN, USART2_TX_FUNCTION }
	};

	gpio_enable_module(COMPORT0_GPIO_MAP,sizeof(COMPORT0_GPIO_MAP) / sizeof (COMPORT0_GPIO_MAP[0]));
}


int rs_232_init_usart()
{
	sysclk_enable_peripheral_clock(&AVR32_USART2);	
	int estado_usart2 = usart_init_rs232(&AVR32_USART2, &usart_opt, sysclk_get_peripheral_bus_hz(&AVR32_USART2));	
	return estado_usart2;
}

void leer_temp(char* temps)
{
	double temp = 0;
	twim_package_t packet_received;
	uint8_t read_data[2];
	float temperature = 0;
	
	read_data[0] = read_data[1] = 0;
	
	status_code_t status;
	// TWI chip address to communicate with
	packet_received.chip = TARGET_ADDRESS;
	
	// Where to find the data to be written
	packet_received.buffer = read_data;
	
	// How many bytes do we want to read
	packet_received.length = AT30TSE_TEMPERATURE_REG_SIZE;
	
	//! Transfer direction
	//packet_received.read = true;
	
	// Registry Address
	packet_received.addr[0] = AT30TSE_TEMPERATURE_REG;
	
	// # of BYTES for Address
	packet_received.addr_length = 1;
	
	//print_dbg ("Reading data from TARGET\r\n");
	// Read data from TARGET
	status = twi_master_read(&AVR32_TWIM0, &packet_received);
	
	
	if (status == STATUS_OK)
	{
		uint16_t data = (read_data[0] << 8) | read_data[1];
		int8_t sign = 1;

		//Check if negative and clear sign bit.
		if (data & (1 << 15)) {
			sign *= -1;
			data &= ~(1 << 15);
		}
		
		
		// Convert to temperature.
		switch (resolution) {
			case AT30TSE_CONFIG_RES_9_bit:
			data = (data >> 7);
			temperature = data * sign * 0.5;
			break;

			case AT30TSE_CONFIG_RES_10_bit:
			data = (data >> 6);
			temperature = data * sign * 0.25;
			break;

			case AT30TSE_CONFIG_RES_11_bit:
			data = (data >> 5);
			temperature = data * sign * 0.125;
			break;

			case AT30TSE_CONFIG_RES_12_bit:
			data = (data >> 4);
			temperature = data * sign * 0.0625;
			break;

			default:
			break;
		}
		
		
		sprintf(temps,"%.1f",temperature);
		
	}
	
	else
		sprintf(temps,"%s","X");
}
uint8_t getStateAT86RF212(void)
{
	return pal_trx_reg_read(RG_TRX_STATUS) & 0x1F;
}

uint8_t txTrama(uint8_t *data)
{
	uint8_t state = getStateAT86RF212();
	//pcb_t *pcb = chb_get_pcb();
	
	
	if ((state == BUSY_TX) || (state == BUSY_TX_ARET))
	{
		return RADIO_WRONG_STATE;
	}
	DISABLE_TRX_IRQ();
	
	SLP_TR_LOW();
	pal_trx_reg_write(RG_TRX_STATE,CMD_FORCE_TRX_OFF); // 
	
	while (getStateAT86RF212()!= CMD_TRX_OFF);

	pal_trx_reg_write(RG_TRX_STATE,CMD_TX_ARET_ON); // 
	DELAY_US(TRX_OFF_TO_PLL_ON_TIME_US);
	
	while (getStateAT86RF212()!=CMD_TX_ARET_ON)// 
	{
		DELAY_US(300);
		pal_trx_reg_write(RG_TRX_STATE,CMD_TX_ARET_ON);
	}
	
	// write frame to buffer. first write header into buffer (add 1 for len byte), then data.
	pal_trx_frame_write(data,data[0] - LENGTH_FIELD_LEN);
	
	pal_trx_reg_write(RG_TRX_STATE,CMD_TX_START);

	ENABLE_TRX_IRQ();
	
	variable1=getStateAT86RF212();
	return variable1;
}

uint8_t txTramaManual(uint8_t *data)
{
	uint8_t state = getStateAT86RF212();
	if (state==CMD_RX_ON) {
		DISABLE_TRX_IRQ();
		
		SLP_TR_LOW();
		pal_trx_reg_write(RG_TRX_STATE,CMD_FORCE_TRX_OFF); //
		
		while (getStateAT86RF212()!=CMD_TRX_OFF);
		
		pal_trx_reg_write(RG_TRX_STATE,CMD_TX_START); //
		
		pal_trx_frame_write(data,data[0] - LENGTH_FIELD_LEN);
		
		pal_trx_reg_write(RG_TRX_STATE,CMD_TX_START);
	}
}



uint8_t init_AT86RF212(void)
{
	RESET();
	pal_trx_reg_write(RG_TRX_STATE, CMD_FORCE_TRX_OFF); // Forzar el estado off
	while(getStateAT86RF212()!= CMD_TRX_OFF); // espero el estado off
	pal_trx_reg_write(RG_TRX_CTRL_0, CMD_NOP); 
//	pal_trx_reg_write(RG_PHY_CC_CCA,||SR_SUB_MODE); // 914Mhz set channel ->
	pal_trx_reg_write(RG_IRQ_MASK, 0x0C);  // IRQ_RX_START && IRQ_TRX_END
	pal_trx_reg_write(RG_TRX_CTRL_1, 0x02); // AACK_PROM_MODE Promiscuous mode is enabled 
	PAL_WAIT_1_US();
	pal_trx_reg_write(RG_TRX_STATE, CMD_RX_ON);// seteo el tran en RX
	while ((pal_trx_reg_read(RG_TRX_STATUS)&0x1F)!=CMD_RX_ON);
}

void RESET()
{
	RST_LOW();
	DELAY_US(P_ON_TO_CLKM_AVAILABLE_TYP_US);
	RST_HIGH();
	delay_ms(1);
}
int main (void)
{
	char temps[10] = "\0";
	int i=0;
	
	//board_init();
	// configuracion del clock del sistema ver archivo "conf_clock.h"	
	sysclk_init();	
	
	//Configuracion de los pines para los LEDS 
	led_init_pins();

	//Configuracion de los pines para el RS-232
	rs_232_init_pins();
	
	//Configuracion pins para RF
	init_rf_pins();
	
	//Inicializacion del SPI
	spi_init_module();
	
	//Inicializacion de la USART	
	int estado_rs_232 = rs_232_init_usart();

		
	//Inicializacion de las interrupciones
	inicializar_interrupciones();
	
	// Inicializacion del timer
	tc_init(tc);
	// inicializacion de AT86RF212
	//while (trx_init()!=TRX_SUCCESS);
	//at86rfx_init();
	//Inicializacion Modulo RF (Depurar!)
// 	while (at86rfx_init() != AT86RFX_SUCCESS) {
//  	 		escribir_linea_pc("Modulo RF:\tFAILED\r\n");
//  	 	}
//  		escribir_linea_pc("Modulo RF:\tPASS\r\n");
 
   	register_value = pal_trx_reg_read(RG_PART_NUM);//pedido de identificacion del modulo. Debe devolver 0x07

	if (register_value == PART_NUM_AT86RF212) 
 		escribir_linea_pc("Modulo RF:\tPASS\r\n");
	else
		escribir_linea_pc("Modulo RF:\tFAILED\r\n"); 			
	escribir_linea_pc(register_value);
	
	//Inicializacion del sensor de temp
	
	init_i2c_pins();
	init_i2c_module();
	// inicializacion del tran
	init_AT86RF212();
	//------------------Fin de conguracion
	
	escribir_linea_pc("TESIS TUCUMAN 2015\n\r");
		
	while(true)
	{
		if (cola_PC_nr != cola_PC_nw )
		{
			if (cola_PC[cola_PC_nr] == 't')
			{
				leer_temp(temps);
				escribir_linea_pc("Temp: ");
				escribir_linea_pc(temps);
				escribir_linea_pc("*C\r\n");
			}
			cola_PC_nr++;
			if (cola_PC_nr >= tamano_cola)
				cola_PC_nr = 0;
		}
 	}
}

