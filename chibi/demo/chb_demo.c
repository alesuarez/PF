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
/*!
    \file 
    \ingroup


*/
/**************************************************************************/
#include <string.h>
#include <avr/pgmspace.h>
#include "chb.h"
#include "chb_demo.h"
#include "chb_buf.h"
#include "chb_drvr.h"
#include "cmd.h"

// this is where the prototypes for the commands you write go. add them
// here first so the compiler knows that the function in the command table 
// can be found later on in this file. The static keyword just means that
// these prototypes are valid for this file only and not global.
static void cmd_hello(U8 argc, char **argv);
static void cmd_set_short_addr(U8 argc, char **argv);
static void cmd_get_short_addr(U8 argc, char **argv);
static void cmd_set_ieee_addr(U8 argc, char **argv);
static void cmd_get_ieee_addr(U8 argc, char **argv);
static void cmd_tx(U8 argc, char **argv);
static void cmd_reg_read(U8 argc, char **argv);
static void cmd_reg_write(U8 argc, char **argv);

#ifdef CHB_AT86RF212
    #if (CHB_CC1190_PRESENT)
    static void cmd_set_hgm(U8 argc, char **argv);
    #endif
#endif

// command table - this is where you add the commands you want to use
// in the command line
cmd_t cmd_tbl[] = 
{
    {"hello",   cmd_hello},
    {"setsaddr", cmd_set_short_addr},
    {"getsaddr", cmd_get_short_addr},
    {"setiaddr", cmd_set_ieee_addr},
    {"getiaddr", cmd_get_ieee_addr},
    {"rd", cmd_reg_read},
    {"wr", cmd_reg_write},
    {"send", cmd_tx},

#ifdef CHB_AT86RF212
    #if (CHB_CC1190_PRESENT)
    {"hgm", cmd_set_hgm},
    #endif
#endif

    {NULL,      NULL}
};

/**************************************************************************/
/*!
    Return a pointer to the command table. You don't need to do anything with
    this. This is used by the command line parser to retrieve the table when
    it compares what was written on the command line with the available commands.
*/
/**************************************************************************/
cmd_t *cmd_tbl_get()
{
    return cmd_tbl;
}

/**************************************************************************/
/*!
    This is the main function in the code and the first user function to get
    executed.
*/
/**************************************************************************/
int main()
{ 
    pcb_t *pcb = chb_get_pcb();
    chb_rx_data_t rx_data;

    // init the chibi stack
    chb_init();

    // init the command line
    cmd_init();

    // and off we go...
    while (1)
    {
        cmd_poll();

        if (pcb->data_rcv)
        {
            // get the length of the data
            rx_data.len = chb_read(&rx_data);

            // make sure the length is nonzero
            if (rx_data.len)
            {
#if (CHB_PROMISCUOUS)
                U8 i;
                for (i=0; i<rx_data.len; i++)
                {
                    chb_putchar(rx_data.data[i], NULL);
                }
#else
                printf_P(PSTR("Message received from node %02X: %s, len=%d, rssi=%02X.\n"), rx_data.src_addr, rx_data.data, rx_data.len, pcb->ed);
#endif
            }
        }
    }
    return 0;
}

/**************************************************************************/
/*!
    Print a hello string to the terminal. Just a reality check to make sure
    your setup is correct.
*/
/**************************************************************************/
void cmd_hello(U8 argc, char **argv)
{
    printf_P(PSTR("Hello World!\n"));
}

/**************************************************************************/
/*!
    Set the short address of the wireless node. This is the 16-bit "nickname" 
    of your node and what will be used to identify it. 
*/
/**************************************************************************/
void cmd_set_short_addr(U8 argc, char **argv)
{
    U16 addr = strtol(argv[1], NULL, 16);
    chb_set_short_addr(addr);
}

/**************************************************************************/
/*!
    Retrieve the short address of the node.
*/
/**************************************************************************/
void cmd_get_short_addr(U8 argc, char **argv)
{
    U16 addr = chb_get_short_addr();
    printf_P(PSTR("Short Addr = %04X.\n"), addr);
}

/**************************************************************************/
/*!
    Set the IEEE address of the wireless node. This is the 64-bit globally
    unique address for this node. Unless you have a block of IEEE addresses
    you probably don't need to se
*/
/**************************************************************************/
void cmd_set_ieee_addr(U8 argc, char **argv)
{
    U8 i, addr[8];

    memset(addr, 0, 8);
    for (i=0; i<argc-1; i++)
    {
        addr[i] = strtol(argv[i+1], NULL, 16);
    }
    chb_set_ieee_addr(addr);
}

/**************************************************************************/
/*!
    Display the IEEE address of the node.
*/
/**************************************************************************/
void cmd_get_ieee_addr(U8 argc, char **argv)
{
    U8 i, addr[8];

    chb_get_ieee_addr(addr);
    
    printf_P(PSTR("IEEE Addr = "));
    for (i=8; i>0; i--)
    {
        printf("%02X", addr[i-1]);
    }
}

/**************************************************************************/
/*!
    Read radio registers via SPI.
    Usage: rd <addr>
*/
/**************************************************************************/
void cmd_reg_read(U8 argc, char **argv)
{
    U8 addr, val;

    addr = strtol(argv[1], NULL, 16);
    val = chb_reg_read(addr);
    printf_P(PSTR("Reg Read: %04X, %02X.\n"), addr, val);
}

/**************************************************************************/
/*!
    Write radio registers via SPI
    Usage: wr <addr> <val>
*/
/**************************************************************************/
void cmd_reg_write(U8 argc, char **argv)
{
    U8 addr, val;

    addr = strtol(argv[1], NULL, 16);
    val = strtol(argv[2], NULL, 16);

    chb_reg_write(addr, val);
    printf_P(PSTR("Write: %04X, %02X.\n"), addr, val);

    val = chb_reg_read(addr);
    printf_P(PSTR("Readback: %04X, %02X.\n"), addr, val);
}

/**************************************************************************/
/*!
    Transmit data to another node wirelessly using Chibi stack.
    Usage: send <addr> <string...>
*/
/**************************************************************************/
void cmd_tx(U8 argc, char **argv)
{
    U8 i, len, *data_ptr, data[50];
    U16 addr;

    addr = strtol(argv[1], NULL, 16);

    data_ptr = data;
    for (i=0; i<argc-2; i++)
    {
        len = strlen(argv[i+2]);
        strcpy((char *)data_ptr, (char *)argv[i+2]);
        data_ptr += len;
        *data_ptr++ = ' ';
    }
    *data_ptr++ = '\0';

    chb_write(addr, data, data_ptr - data);
}


#ifdef CHB_AT86RF212
    #if (CHB_CC1190_PRESENT)
    /**************************************************************************/
    /*!
        Set or disable the high gain mode on the CC1190.
        Usage: hgm <val>
        val = 1 : Enable high gain mode
        val = 0 : Disable high gain mode
    */
    /**************************************************************************/
    void cmd_set_hgm(U8 argc, char **argv)
    {
        U8 enb = strtol(argv[1], NULL, 10);
        chb_set_hgm(enb);
    }
    #endif
#endif

