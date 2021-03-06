/*******************************************************************
    Copyright (C) 2009 FreakLabs
    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions
    are met:

    1. Redistributions of source code must retain the above copyright
       notice, this list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright
       notice, this list of conditions and the following disclaimer in the
       documentation and/or other materials provided with the distribution.
    3. Neither the name of the the copyright holder nor the names of its contributors
       may be used to endorse or promote products derived from this software
       without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE THE COPYRIGHT HOLDERS AND CONTRIBUTORS ``AS IS'' AND
    ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
    IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
    ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
    FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
    DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
    OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
    HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
    LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
    OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
    SUCH DAMAGE.

    Originally written by Christopher Wang aka Akiba.
    Please post support questions to the FreakLabs forum.

*******************************************************************/
#include <stdio.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include "chb.h"
#include "chb_drvr.h"
#include "chb_buf.h"
#include "chb_spi.h"
#include "chb_eeprom.h"

// store string messages in flash rather than RAM
const char chb_err_overflow[] PROGMEM = "BUFFER FULL. TOSSING INCOMING DATA\n";
const char chb_err_init[] PROGMEM = "RADIO NOT INITIALIZED PROPERLY\n";

/**************************************************************************/
/*!

*/
/**************************************************************************/
static U8 chb_get_state()
{
    return chb_reg_read(TRX_STATUS) & 0x1f;
}

/**************************************************************************/
/*!

*/
/**************************************************************************/
static U8 chb_get_status()
{
    return chb_reg_read(TRX_STATE) >> CHB_TRAC_STATUS_POS;
}

/**************************************************************************/
/*! 
  Create variable usec delay. 
*/
/**************************************************************************/
static void chb_delay_us(U16 usec)
{
    _delay_us((double)usec);
}

/**************************************************************************/
/*!

*/
/**************************************************************************/
void chb_reset()
{
    CHB_RST_ENABLE();
    CHB_SLPTR_DISABLE();
    chb_delay_us(TIME_RESET);
    CHB_RST_DISABLE();
}

/**************************************************************************/
/*!

*/
/**************************************************************************/
U8 chb_reg_read(U8 addr)
{
    U8 val = 0;

    /* Add the register read command to the register address. */
    addr |= 0x80;

    CHB_ENTER_CRIT();
    CHB_SPI_ENABLE();

    /*Send Register address and read register content.*/
    val = chb_xfer_byte(addr);
    val = chb_xfer_byte(val);

    CHB_SPI_DISABLE();
    CHB_LEAVE_CRIT();

    return val;
}

/**************************************************************************/
/*! 
 
*/
/**************************************************************************/
U16 chb_reg_read16(U8 addr)
{
    U8 i;
    U16 val = 0;

    for (i=0; i<2; i++)
    {
        addr |= chb_reg_read(addr + i) << (8 * i);
    }
    return val;
}

/**************************************************************************/
/*!

*/
/**************************************************************************/
void chb_reg_write(U8 addr, U8 val)
{
    U8 dummy; 

    /* Add the Register Write command to the address. */
    addr |= 0xC0;

    CHB_ENTER_CRIT();
    CHB_SPI_ENABLE();

    /*Send Register address and write register content.*/
    dummy = chb_xfer_byte(addr);
    dummy = chb_xfer_byte(val);

    CHB_SPI_DISABLE();
    CHB_LEAVE_CRIT();
}

/**************************************************************************/
/*!

*/
/**************************************************************************/
void chb_reg_write16(U8 addr, U16 val)
{
    U8 i;

    for (i=0; i<2; i++)
    {
        chb_reg_write(addr + i, val >> (8 * i));
    }
}

/**************************************************************************/
/*!

*/
/**************************************************************************/
void chb_reg_write64(U8 addr, U8 *val)
{
    U8 i;

    for (i=0; i<8; i++)
    {
        chb_reg_write(addr + i, *(val + i));
    }
}

/**************************************************************************/
/*!

*/
/**************************************************************************/
void chb_reg_read_mod_write(U8 addr, U8 val, U8 mask)
{
    U8 tmp;

    tmp = chb_reg_read(addr);
    val &= mask;                // mask off stray bits from val
    tmp &= ~mask;               // mask off bits in reg val
    tmp |= val;                 // copy val into reg val
    chb_reg_write(addr, tmp);   // write back to reg
}

/**************************************************************************/
/*!

*/
/**************************************************************************/
void chb_frame_write(U8 *hdr, U8 hdr_len, U8 *data, U8 data_len)
{
    U8 i, dummy;

    // dont allow transmission longer than max frame size
    if ((hdr_len + data_len) > 127)
    {
        return;
    }

    // initiate spi transaction
    CHB_ENTER_CRIT();
    CHB_SPI_ENABLE(); 

    // send fifo write command
    dummy = chb_xfer_byte(CHB_SPI_CMD_FW);

    // write hdr contents to fifo
    for (i=0; i<hdr_len; i++)
    {
        dummy = chb_xfer_byte(*hdr++);
    }

    // write data contents to fifo
    for (i=0; i<data_len; i++)
    {
        dummy = chb_xfer_byte(*data++);
    }

    // terminate spi transaction
    CHB_SPI_DISABLE(); 
    CHB_LEAVE_CRIT();
}

/**************************************************************************/
/*!

*/
/**************************************************************************/
static void chb_frame_read()
{
    U8 i, len, data;

    CHB_ENTER_CRIT();
    CHB_SPI_ENABLE();

    /*Send frame read command and read the length.*/
    chb_xfer_byte(CHB_SPI_CMD_FR);
    len = chb_xfer_byte(0);

    /*Check for correct frame length.*/
    if ((len >= CHB_MIN_FRAME_LENGTH) && (len <= CHB_MAX_FRAME_LENGTH))
    {
        // check to see if there is room to write the frame in the buffer. if not, then drop it
        if (len < (CHB_BUF_SZ - chb_buf_get_len()))
        {
            chb_buf_write(len);
            
            for (i=0; i<len; i++)
            {
                data = chb_xfer_byte(0);
                chb_buf_write(data);
            }
        }
        else
        {
            // we've overflowed the buffer. toss the data and do some housekeeping
            pcb_t *pcb = chb_get_pcb();
            char buf[50];

            // read out the data and throw it away
            for (i=0; i<len; i++)
            {
                data = chb_xfer_byte(0);
            }

            // Increment the overflow stat
            pcb->overflow++;

            // grab the message from flash & print it out
            strcpy_P(buf, chb_err_overflow);
            printf(buf);
        }
    }

    CHB_SPI_DISABLE();
    CHB_LEAVE_CRIT();
}

/**************************************************************************/
/*!
    Read directly from the SRAM on the radio. This is only used for debugging
    purposes.
*/
/**************************************************************************/
#ifdef CHB_DEBUG
void chb_sram_read(U8 addr, U8 len, U8 *data)
{
    U8 i, dummy;

    CHB_ENTER_CRIT();
    CHB_SPI_ENABLE();

    /*Send SRAM read command.*/
    dummy = chb_xfer_byte(CHB_SPI_CMD_SR);

    /*Send address where to start reading.*/
    dummy = chb_xfer_byte(addr);

    for (i=0; i<len; i++)
    {
        *data++ = chb_xfer_byte(0);
    }

    CHB_SPI_DISABLE();
    CHB_LEAVE_CRIT();
}

/**************************************************************************/
/*!

*/
/**************************************************************************/
void chb_sram_write(U8 addr, U8 len, U8 *data)
{    
    U8 i, dummy;

    CHB_ENTER_CRIT();
    CHB_SPI_ENABLE();

    /*Send SRAM write command.*/
    dummy = chb_xfer_byte(CHB_SPI_CMD_SW);

    /*Send address where to start writing to.*/
    dummy = chb_xfer_byte(addr);

    for (i=0; i<len; i++)
    {
        dummy = chb_xfer_byte(*data++);
    }

    CHB_SPI_DISABLE();
    CHB_LEAVE_CRIT();
}
#endif

/**************************************************************************/
/*!

*/
/**************************************************************************/
U8 chb_set_channel(U8 channel)
{
    U8 state;
    
    chb_reg_read_mod_write(PHY_CC_CCA, channel, 0x1f); 

    // add a delay to allow the PLL to lock if in active mode.
    state = chb_get_state();
    if ((state == CHB_RX_ON) || (state == CHB_PLL_ON))
    {
        chb_delay_us(TIME_PLL_LOCK);
    }

    return ((chb_reg_read(PHY_CC_CCA) & 0x1f) == channel) ? RADIO_SUCCESS : RADIO_TIMED_OUT;
}

/**************************************************************************/
/*!
    Set the TX/RX state machine state. Some manual manipulation is required 
    for certain operations. Check the datasheet for more details on the state 
    machine and manipulations.
*/
/**************************************************************************/
static U8 chb_set_state(U8 state)
{
    U8 curr_state, delay;

    // if we're sleeping then don't allow transition
    if (CHB_SLPTR_PORT & _BV(CHB_SLPTR_PIN))
    {
        return RADIO_WRONG_STATE;
    }

    // if we're in a transition state, wait for the state to become stable
    curr_state = chb_get_state();
    if ((curr_state == CHB_BUSY_TX_ARET) || (curr_state == CHB_BUSY_RX_AACK) || (curr_state == CHB_BUSY_RX) || (curr_state == CHB_BUSY_TX))
    {
        while (chb_get_state() == curr_state);
    }

    // At this point it is clear that the requested new_state is:
    // TRX_OFF, RX_ON, PLL_ON, RX_AACK_ON or TX_ARET_ON.
    // we need to handle some special cases before we transition to the new state
    switch (state)
    {
    case CHB_TRX_OFF:
        /* Go to TRX_OFF from any state. */
        CHB_SLPTR_DISABLE();
        chb_delay_us(TIME_NOCLK_TO_WAKE);
        chb_reg_read_mod_write(TRX_STATE, CMD_FORCE_TRX_OFF, 0x1f);
        chb_delay_us(TIME_CMD_FORCE_TRX_OFF);
        break;

    case CHB_TX_ARET_ON:
        if (curr_state == CHB_RX_AACK_ON)
        {
            /* First do intermediate state transition to PLL_ON, then to TX_ARET_ON. */
            chb_reg_read_mod_write(TRX_STATE, CMD_PLL_ON, 0x1f);
            chb_delay_us(TIME_STATE_TRANSITION_PLL_ACTIVE);
        }
        break;

    case CHB_RX_AACK_ON:
        if (curr_state == CHB_TX_ARET_ON)
        {
            /* First do intermediate state transition to RX_ON, then to RX_AACK_ON. */
            chb_reg_read_mod_write(TRX_STATE, CMD_PLL_ON, 0x1f);
            chb_delay_us(TIME_STATE_TRANSITION_PLL_ACTIVE);
        }
        break;
    }

    /* Now we're okay to transition to any new state. */
    chb_reg_read_mod_write(TRX_STATE, state, 0x1f);

    /* When the PLL is active most states can be reached in 1us. However, from */
    /* TRX_OFF the PLL needs time to activate. */
    delay = (curr_state == CHB_TRX_OFF) ? TIME_TRX_OFF_TO_PLL_ACTIVE : TIME_STATE_TRANSITION_PLL_ACTIVE;
    chb_delay_us(delay);

    if (chb_get_state() == state)
    {
        return RADIO_SUCCESS;
    }
    return RADIO_TIMED_OUT;
}

/**************************************************************************/
/*! 
 
*/
/**************************************************************************/
void chb_set_ieee_addr(U8 *addr)
{
    chb_eeprom_write(CHB_EEPROM_IEEE_ADDR, addr, 8); 
    chb_reg_write64(IEEE_ADDR_0, addr); 
}

/**************************************************************************/
/*!

*/
/**************************************************************************/
void chb_get_ieee_addr(U8 *addr)
{
    chb_eeprom_read(CHB_EEPROM_IEEE_ADDR, addr, 8);
}

/**************************************************************************/
/*!

*/
/**************************************************************************/
void chb_set_short_addr(U16 addr)
{
    U8 *addr_ptr = (U8 *)&addr;
    pcb_t *pcb = chb_get_pcb();

    chb_eeprom_write(CHB_EEPROM_SHORT_ADDR, addr_ptr, 2);
    chb_reg_write16(SHORT_ADDR_0, addr);
    pcb->src_addr = addr;
}

/**************************************************************************/
/*!

*/
/**************************************************************************/
U16 chb_get_short_addr()
{
    U8 addr[2];

    chb_eeprom_read(CHB_EEPROM_SHORT_ADDR, addr, 2);
    return *(U16 *)addr;
}

/**************************************************************************/
/*!
    Load the data into the fifo, initiate a transmission attempt,
    and return the status of the transmission attempt.
*/
/**************************************************************************/
U8 chb_tx(U8 *hdr, U8 *data, U8 len)
{
    U8 state = chb_get_state();
    pcb_t *pcb = chb_get_pcb();

    if ((state == CHB_BUSY_TX) || (state == CHB_BUSY_TX_ARET))
    {
        return RADIO_WRONG_STATE;
    }

    // TODO: check why we need to transition to the off state before we go to tx_aret_on
    chb_set_state(CHB_TRX_OFF);
    chb_set_state(CHB_TX_ARET_ON);

    // TODO: try and start the frame transmission by writing TX_START command instead of toggling
    // sleep pin...i just feel like it's kind of weird...

    // write frame to buffer. first write header into buffer (add 1 for len byte), then data. 
    chb_frame_write(hdr, CHB_HDR_SZ + 1, data, len);

    //Do frame transmission.
    chb_reg_read_mod_write(TRX_STATE, CMD_TX_START, 0x1F);

    // wait for the transmission to end, signalled by the TRX END flag
    while (!pcb->tx_end);
    pcb->tx_end = false;

    // check the status of the transmission
    return chb_get_status();
}

/**************************************************************************/
/*!
    Enable or disable the radio's sleep mode.
*/
/**************************************************************************/
void chb_sleep(U8 enb)
{
    if (enb)
    {
        // first we need to go to TRX OFF state
        chb_set_state(CHB_TRX_OFF);

        // set the SLPTR pin
        CHB_SLPTR_PORT |= _BV(CHB_SLPTR_PIN);
    }
    else
    {
        // make sure the SLPTR pin is low first
        CHB_SLPTR_PORT &= ~(_BV(CHB_SLPTR_PIN));

        // we need to allow some time for the PLL to lock
        chb_delay_us(TIME_SLEEP_TO_TRX_OFF);

        // Turn the transceiver back on
        chb_set_state(RX_STATE);
    }
}

/**************************************************************************/
/*!

*/
/**************************************************************************/
static void chb_radio_init()
{
    U8 ieee_addr[8];

    // reset chip
    chb_reset();

    // disable intps while we config the radio
    chb_reg_write(IRQ_MASK, 0);

    // force transceiver off while we configure the intps
    chb_reg_read_mod_write(TRX_STATE, CMD_FORCE_TRX_OFF, 0x1F);
    chb_delay_us(TIME_P_ON_TO_TRX_OFF);

    // set radio cfg parameters
    // **note** uncomment if these will be set to something other than default
    //chb_reg_read_mod_write(XAH_CTRL_0, CHB_MAX_FRAME_RETRIES << CHB_MAX_FRAME_RETRIES_POS, 0xF << CHB_MAX_FRAME_RETRIES_POS);
    //chb_reg_read_mod_write(XAH_CTRL_0, CHB_MAX_CSMA_RETRIES << CHB_MAX_CSMA_RETIRES_POS, 0x7 << CHB_MAX_CSMA_RETIRES_POS);
    //chb_reg_read_mod_write(CSMA_SEED_1, CHB_MIN_BE << CHB_MIN_BE_POS, 0x3 << CHB_MIN_BE_POS);     
    //chb_reg_read_mod_write(CSMA_SEED_1, CHB_CSMA_SEED1 << CHB_CSMA_SEED1_POS, 0x7 << CHB_CSMA_SEED1_POS);
    //chb_ret_write(CSMA_SEED0, CHB_CSMA_SEED0);     
    //chb_reg_read_mod_write(PHY_CC_CCA, CHB_CCA_MODE << CHB_CCA_MODE_POS,0x3 << CHB_CCA_MODE_POS);
    //chb_reg_write(CCA_THRES, CHB_CCA_ED_THRES);
    //chb_reg_read_mod_write(PHY_TX_PWR, CHB_TX_PWR, 0xf);

    // set the channel
    chb_set_channel(CHB_CHANNEL);

    #if (CHB_PROMISCUOUS == 0)
    // set autocrc mode
    chb_reg_read_mod_write(PHY_TX_PWR, 1 << CHB_AUTO_CRC_POS, 1 << CHB_AUTO_CRC_POS);
    #endif

    // set fsm state
    // put trx in rx auto ack mode
    chb_set_state(RX_STATE);

    // set pan ID
    chb_reg_write16(PAN_ID_0, CHB_PAN_ID);

    // set short addr
    // NOTE: Possibly get this from EEPROM
    chb_reg_write16(SHORT_ADDR_0, chb_get_short_addr());

    // set long addr
    // NOTE: Possibly get this from EEPROM
    chb_get_ieee_addr(ieee_addr);
    chb_reg_write64(IEEE_ADDR_0, ieee_addr);

    // re-enable intps while we config the radio
    chb_reg_write(IRQ_MASK, 0xc);

    // enable mcu intp pin on rising edge
    CFG_CHB_INTP_RISE_EDGE();

    if (chb_get_state() != RX_STATE)
    {
        // ERROR occurred initializing the radio. Print out error message.
        char buf[50];

        // grab the error message from flash & print it out
        strcpy_P(buf, chb_err_init);
        printf(buf);
    }

}

/**************************************************************************/
/*!

*/
/**************************************************************************/
void chb_drvr_init()
{
    // config SPI for at86rf230 access
    chb_spi_init();

    // configure IOs
    CHB_SLPTR_DDIR |= (_BV(CHB_SLPTR_PIN));
    CHB_RST_DDIR |= (_BV(CHB_RST_PIN));

    // config radio
    chb_radio_init();
}

/**************************************************************************/
/*!

*/
/**************************************************************************/
ISR(CHB_RADIO_IRQ)
{
    U8 dummy, state, intp_src = 0;
    pcb_t *pcb = chb_get_pcb();

    CHB_ENTER_CRIT();

    /*Read Interrupt source.*/
    CHB_SPI_ENABLE();

    /*Send Register address and read register content.*/
    dummy = chb_xfer_byte(IRQ_STATUS | CHB_SPI_CMD_RR);
    intp_src = chb_xfer_byte(0);

    CHB_SPI_DISABLE();

    while (intp_src)
    {
        /*Handle the incomming interrupt. Prioritized.*/
        if ((intp_src & CHB_IRQ_RX_START_MASK))
        {
            intp_src &= ~CHB_IRQ_RX_START_MASK;
        }
        else if (intp_src & CHB_IRQ_TRX_END_MASK)
        {
            state = chb_get_state();

            if ((state == CHB_RX_ON) || (state == CHB_RX_AACK_ON) || (state == CHB_BUSY_RX_AACK))
            {
                // get the ed measurement
                pcb->ed = chb_reg_read(PHY_ED_LEVEL);

                // get the crc
                pcb->crc = (chb_reg_read(PHY_RSSI) & (1<<7)) ? 1 : 0;

                // if the crc is not valid, then do not read the frame and set the rx flag
                if (pcb->crc)
                {
                    // get the data
                    chb_frame_read();
                    pcb->rcvd_xfers++;
                    pcb->data_rcv = true;
                }
            }
            else
            {
                pcb->tx_end = true;
            }
            intp_src &= ~CHB_IRQ_TRX_END_MASK;
            while (chb_set_state(RX_STATE) != RADIO_SUCCESS);
        }
        else if (intp_src & CHB_IRQ_TRX_UR_MASK)
        {
            intp_src &= ~CHB_IRQ_TRX_UR_MASK;
            pcb->underrun++;
        }
        else if (intp_src & CHB_IRQ_PLL_UNLOCK_MASK)
        {
            intp_src &= ~CHB_IRQ_PLL_UNLOCK_MASK;
        }
        else if (intp_src & CHB_IRQ_PLL_LOCK_MASK)
        {
            intp_src &= ~CHB_IRQ_PLL_LOCK_MASK;
        }
        else if (intp_src & CHB_IRQ_BAT_LOW_MASK)
        {
            intp_src &= ~CHB_IRQ_BAT_LOW_MASK;
            pcb->battlow++;
        }
        else
        {
        }
    }
    CHB_LEAVE_CRIT();
}
