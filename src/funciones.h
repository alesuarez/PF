/**************************************************************************/
/*!
    Set the channel mode, 915 MHZ
*/
/**************************************************************************/
void set_mode(U8 mode)
{
    chb_reg_read_mod_write(TRX_CTRL_2, 0x00, 0x3f);                
    chb_reg_read_mod_write(RF_CTRL_0, CHB_BPSK_TX_OFFSET, 0x3);     
}
