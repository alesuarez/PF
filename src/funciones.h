/**************************************************************************/
/*!
    Set the channel mode BPSK 915 MHz
*/
/**************************************************************************/
void set_mode()
{
	chb_reg_read_mod_write(TRX_CTRL_2, 0x00, 0x3f);                 // 802.15.4-2006, BPSK
	chb_reg_read_mod_write(RF_CTRL_0, CHB_BPSK_TX_OFFSET, 0x3);     //         
}
