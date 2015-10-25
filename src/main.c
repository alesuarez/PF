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
#define ADDRESS 0x31

uint8_t cola_PC[tamano_cola];
uint8_t cola_PC_nw = 0;
uint8_t cola_PC_nr = 0;

volatile static uint32_t tc_tick = 1;

volatile avr32_tc_t *tc = EXAMPLE_TC;

volatile uint8_t resolution = AT30TSE_CONFIG_RES_12_bit;

uint8_t tx_buffer[5]="aleja";

uint8_t TRX_STATUS = 0;
uint8_t register_value = 0;
uint8_t TRX_CTRL_0 = 0;
uint8_t PHY_TX_PWR = 0;
uint8_t PHY_CC_CCA = 0;

uint8_t IRQ_MASK= 0;
uint8_t IRQ_STATUS;
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
uint8_t address;
bool configuracion = false;
uint8_t pConfiguracion=0;
config_package tramaConfiguracion;
uint8_t spi = 0;
uint8_t colaRX[tamano_cola];
uint8_t contadorRX = 0;
uint8_t dato;

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
	
	IRQ_STATUS = pal_trx_reg_read(RG_IRQ_STATUS) & 0x0C;
	
	//variable1=pal_trx_reg_read(RG_IRQ_STATUS);
	//variable2=pal_trx_reg_read(RG_IRQ_MASK);
		
		// Interrupt Line must be cleared to enable
		eic_clear_interrupt_line(&AVR32_EIC, AVR32_EIC_INT2);
		//IRQ2 Pin 26 MCU --> Pin 24 T
		//IRQ_STATUS = pal_trx_reg_read(RG_IRQ_STATUS);
		//variable1=pal_trx_reg_read(RG_IRQ_STATUS);
		//variable2=pal_trx_reg_read(RG_IRQ_MASK);
		
		switch (IRQ_STATUS){
// 			case TRX_IRQ_TRX_END:
// 				escribir_linea_pc("\n\n --> Trama enviada :) :) \r\n");
// 				pal_trx_frame_read(&colaRX[contadorRX],6);
// 				contadorRX=contadorRX+6;
// 			
// 						
// 			break;
			case TRX_IRQ_RX_START:
				pal_trx_frame_read(&colaRX[contadorRX],120); // para 200kbps
				escribir_linea_pc("\n\n - = T r a m a   r e c i b i d a  = -\n\n");
				escribir_linea_pc(colaRX);
				contadorRX = 0;
				
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
	
	uint8_t c=0;
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
	
	cola_PC[cola_PC_nw] = c;
	
	if (cola_PC[cola_PC_nw] == 0x01)
	{	
		if (!configuracion){
			pConfiguracion = cola_PC_nw;
		}
		configuracion = true;
		
	}
	cola_PC_nw++;
	
	if (cola_PC_nw >= tamano_cola)
	cola_PC_nw = 0;
	
	tc_start(tc,EXAMPLE_TC_CHANNEL);
	return;

	
}

// bool check_pack(uint8_t tampack) //tampack es la cantidad de bytes del paquete hasta antes de EOT, para cdo lo hagamos variable
// {
// 	uint8_t i=3; //cosa que no tome los SOH
// 	char lrc= cola_PC[i];
// 	
// 	while(i<tampack){
// 	i=i+1;
// 	lrc=lrc ^ cola_Pc[i]; //este es el XOR
// 	}
// 	if (lrc==cola_Pc[i]){
// 		return true; //el LRC del paquete y el calculado son iguales
// 	}else 
// 	{
// 		return false; //el LRC del paquete y el calculado no coinciden
// 	}
// }

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
void setStateAT86RF212(uint8_t state, uint8_t time)
{
	pal_trx_reg_write(RG_TRX_STATE, state);
	DELAY_US(time);
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
	
	//SLP_TR_LOW();
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
	//Set register bit TX_AUTO_CRC_ON = 1 register 0x04, TRX_CTRL_1
	//Set MAX_FRAME_RETRIES register 0x2C, XAH_CTRL_0
	//Set MAX_CSMA_RETRIES register 0x2C, XAH_CTRL_0
	//Set CSMA_SEED registers 0x2D, 0x2E
	//Set MAX_BE, MIN_BE register 0x2F, CSMA_BE
	//Configure CCA see Section 8.6
	
	if (state==CMD_RX_ON) {
		DISABLE_TRX_IRQ();
		
		//variable1=getStateAT86RF212();
		escribir_linea_pc("AT86RF por transmitir...\r\n");	
		//estadoPorPc();
		pal_trx_reg_write(RG_TRX_STATE,CMD_FORCE_PLL_ON); //pongo en PLL ON
		while(getStateAT86RF212()!=CMD_PLL_ON);  //espero q se ponga en PLL ON
		//estadoPorPc();
		//pal_trx_reg_write(RG_IRQ_MASK,0x0C);
		//variable1=pal_trx_reg_read(RG_IRQ_MASK);
		pal_trx_reg_write(RG_TRX_STATE,CMD_TX_START); // inicio tx - segun manual: Write TRX_CMD = TX_START, or assert pin 11 (SLP_TR)
		pal_trx_frame_write(data,4);  //esto para mi hay q ponerlo arriba donde dije escribir trama
		// spi_write_packet(&AVR32_SPI,data,4);
		//escribo la trama de datos en el buffer - segun pag 158
		
		//DELAY_US(RST_PULSE_WIDTH_US); // hacia el estado busy_tx
		
		// espero IRQ_3 (TRX_END) issued
		// Read IRQ_STATUS register, pin 24 (IRQ) deasserted
		ENABLE_TRX_IRQ(); 
		//estadoPorPc();
		//variable1=getStateAT86RF212();
		
		
	} else {
		escribir_linea_pc(" no se puede enviar la trama \n");
	}
	pal_trx_reg_write(RG_TRX_STATE,CMD_RX_ON); //vuelvo a estador RX ON
	//estadoPorPc();
}
uint8_t txTramachibi(uint8_t *data)
{
	
	
	variable1=getStateAT86RF212();
	if (getStateAT86RF212()==CMD_RX_ON)
	{
		escribir_linea_pc("\n\n\n\nAT86RF por transmitir... en modo manual con altos de pin\r\n");
		estadoPorPc();
		pal_trx_reg_write(RG_TRX_STATE,CMD_FORCE_PLL_ON);
		while(getStateAT86RF212()!=CMD_PLL_ON);
		pal_trx_frame_write(RG_TRX_STATE,CMD_TX_START);
		DELAY_US(RST_PULSE_WIDTH_NS);
		
		pal_trx_frame_write(data,data[0] - LENGTH_FIELD_LEN);
		escribir_linea_pc("\n\n\n FINNNNNN \n\n\n\n");
	}
	else
	{
		
		escribir_linea_pc("\nno se puede tx\r\n");
		pal_trx_frame_write(RG_TRX_STATE,CMD_RX_ON);
	}
	
}
void promiscuous_mode()
{
	
	for (address=0x20; address<0x2C; address++)
	{
		pal_trx_reg_write(address, 0x00);
	}
	pal_trx_reg_write(RG_XAH_CTRL_1, 0x02);	// AACK_PROM_MODE Promiscuous mode is enabled
//	PAL_WAIT_1_US();
	pal_trx_reg_write(RG_CSMA_SEED_1, 0xD2); // AACK_DIS_ACK = 1 && AACK_FVN_MODE = 3
//	PAL_WAIT_1_US();
	
}

void RESET()
{
	RST_LOW();
	DELAY_US(RST_PULSE_WIDTH_NS);
	RST_HIGH();
	delay_ms(1);
}
void reset()
{
	SLP_TR_LOW();
	RST_HIGH();
	DELAY_US(400);
	RST_LOW();
	DELAY_US(63);
	RST_LOW();
	pal_trx_reg_write(RG_TRX_CTRL_0,0x08);
	//pal_trx_reg_write(RG_TRX_STATE,CMD_FORCE_TRX_OFF);
	
}
void estadoPorPc(){
	delay_ms(1);
	switch (getStateAT86RF212()){
		case P_ON:
		escribir_linea_pc("\r\n AT86RF212 en estado ON\n");
		break;
		case BUSY_RX:
		escribir_linea_pc("\r\n AT86RF212 en estado BUSY_RX\n");
		break;
		case RX_ON:
		escribir_linea_pc("\r\n AT86RF212 en estado RX\n");
		break;
		case TRX_OFF:
		escribir_linea_pc("\r\n AT86RF212 en estado TRX_OFF \n");
		break;
		case PLL_ON:
		escribir_linea_pc("\r\n AT86RF212 en estado PLL_ON\n");
		break;
		case TRX_SLEEP:
		escribir_linea_pc("\r\n AT86RF212 en estado TRX_SLEEP \n");
		break;
		case BUSY_RX_AACK:
		escribir_linea_pc("\r\n AT86RF212 en estado BUSY_RX_AACK\n");
		break;
		case BUSY_TX_ARET:
		escribir_linea_pc("\r\n AT86RF212 en estado BUSY_TX_ARET \n");
		break;
		case RX_AACK_ON:
		escribir_linea_pc("\r\n AT86RF212 en estado RX_AACK_ON\n");
		break;
		case CMD_TX_ARET_ON:
		escribir_linea_pc("\r\n AT86RF212 en estado TX_ARET_ON\n");
		case RX_ON_NOCLK :
		escribir_linea_pc("\r\n AT86RF212 en estado RX_ON_NOCLK \n");
		case RX_AACK_ON_NOCLK:
		escribir_linea_pc("\r\n AT86RF212 en estado RX_AACK_ON_NOCLK \n");
		case STATE_TRANSITION_IN_PROGRESS:
		escribir_linea_pc("\r\n :'(  STATE_TRANSITION_IN_PROGRESS ");
		break;
		
		default:
		escribir_linea_pc("\r\n estado no contemplado");
		break;
	}
	delay_ms(1);
}

uint8_t init_AT86RF212(void)
{
	escribir_linea_pc("\n Inicializando AT86RF212 \n\n");
	Disable_global_interrupt();
	//estadoPorPc();
	
	//variable1=getStateAT86RF212();
	
	//SLP_TR_LOW();
	
	
	//estadoPorPc();
	RESET();
	//PAL_WAIT_1_US();
	pal_trx_reg_write(RG_IRQ_MASK, 0x00);
	PAL_WAIT_1_US();
	pal_trx_reg_write(RG_TRX_STATE, CMD_FORCE_TRX_OFF); // Forzar el estado off
	DELAY_US(RST_PULSE_WIDTH_US); //tTR10
	
	//variable1=getStateAT86RF212();
	while(getStateAT86RF212()!= CMD_TRX_OFF); // espero el estado off
	PAL_WAIT_1_US();
	pal_trx_reg_write(RG_TRX_CTRL_0, 0x08);
	//pal_trx_reg_write(RG_PHY_CC_CCA,||SR_SUB_MODE); // 914Mhz set channel ->
	PAL_WAIT_1_US();
	pal_trx_reg_write(RG_TRX_CTRL_1, 0x22); // 1 -> TX AUTO_CRC && 1-> IRQ_MASK_MODE
	PAL_WAIT_1_US();
	//pal_trx_reg_write(RG_RX_CTRL, 0x20);
	pal_trx_reg_write(RG_IRQ_MASK, 0x0C);
	PAL_WAIT_1_US();
	pal_trx_reg_write(RG_TRX_CTRL_2, 0x68); // O-QPSK 100kb/s
	//pal_trx_reg_write(RG_TRX_CTRL_2, 0x29); // O-QPSK 200kb/s
	
	//pal_trx_reg_write(RG_XOSC_CTRL, 0x40); // manejo del cristal externo y capacitores se muere cuando se activa
	PAL_WAIT_1_US();
	promiscuous_mode();
	PAL_WAIT_1_US();
	pal_trx_reg_write(RG_TRX_STATE, CMD_FORCE_PLL_ON);// seteo el tran en
	DELAY_US(TIME_PLL_ON_RX_ON);

	while (getStateAT86RF212()!=CMD_PLL_ON);
	PAL_WAIT_1_US();
	pal_trx_reg_write(RG_TRX_STATE, CMD_RX_ON);
	PAL_WAIT_1_US();
 	
	cpu_irq_enable();
	Enable_global_interrupt();
	
	escribir_linea_pc("\n Terminando configuracion AT86RF212 \n\n");
}
void modeConfig()
{
// cuando esta en modo de configuracion no hace nada, solo espera que le lleguen los datos
	uint8_t tam=8;
	while(cola_PC_nw < (pConfiguracion + 0x09));
	// comprobar CRC
//	if (check_pack(tam)){
	// solo si pasa el crc sigo la configuracion
		tramaConfiguracion.crc = cola_PC[pConfiguracion+8];
	
		// fin comprobacion
		
		tramaConfiguracion.cmd = cola_PC[pConfiguracion+3];
	
		tramaConfiguracion.payload[0] = cola_PC[pConfiguracion+5];	
		tramaConfiguracion.payload[1] = cola_PC[pConfiguracion+6];
		tramaConfiguracion.payload[2] = cola_PC[pConfiguracion+7];
	
		switch (tramaConfiguracion.cmd){
			case BAUDRATE:
				escribir_linea_pc("\r\nConfiguracion del baud rate\n");
			break;
			case TEMPERATURA:
				escribir_linea_pc("\r\nVeo la temperatura\n");
			break;
			
		}
//	} //ver que onda cuando sale de aca, si falla el LRC no hace nada, quiza deberia hacer algo?
	return;
}

int main (void)
{
// nodo II
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
	//pal_trx_reg_write(RG_TRX_STATE,CMD_FORCE_PLL_ON);

	

	init_AT86RF212();
	//------------------Fin de conguracion
	
	escribir_linea_pc("TESIS TUCUMAN 2015\n\r\n");
	
	//setStateAT86RF212(CMD_RX_ON, TIME_PLL_ON_RX_ON);// seteo el tran en RX
	//pal_trx_reg_write(RG_IRQ_MASK, 0x0C);
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
				
			if (configuracion && (cola_PC_nr >= pConfiguracion + 4))
			{
				if ((cola_PC[pConfiguracion] & cola_PC[pConfiguracion+1] & cola_PC[pConfiguracion+2]) == 0x01)
				{
					if (cola_PC[pConfiguracion+3] == ADDRESS)
					{
						modeConfig();
					}
					
				}
				configuracion = false;
			}
		}
		//at86rfx_tx_frame(tx_buffer);
		//txTramaManual(tx_buffer);
		//txTramachibi(tx_buffer);
		//txTramachibi(tx_buffer);
		//estadoPorPc();
 		delay_ms(500);
		//txTramachibi(tx_buffer); // funcion creada segun el manual
	//	txTrama(tx_buffer); // funcion creada segun un ejemplo LwMesh
		// para Rx lo hace cuando hay interrupcion y muestra por pantalla

 	}
}

