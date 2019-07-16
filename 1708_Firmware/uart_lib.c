/*
 * CFile1.c
 *
 * Created: 6/19/2018 8:25:41 AM
 *  Author: kangm
 */ 
 #include "uart_lib.h"
 #define F_CPU 3333333UL
 #define CLK_PER 3333333
 #include <util/delay.h>

 void initUSART(uint8_t baud_rate_code){  // Initialize USART

	int baud_rate = 9600;
	int cal_baud = 0;
	
	switch(baud_rate_code)
	{
		case 0x02: // 9600
			baud_rate = 9600;
			break;
		case 0x04: // 19200
			baud_rate = 19200;
			break;
		case 0x05: // 1200
			baud_rate = 1200;
			break;
		case 0x06:	// 4800
			baud_rate = 4800;
			break;
		case 0x07: // baud rate 300
			baud_rate = 300;
			break;
		default:
			baud_rate = 300;
			break;
	}
	
	 cal_baud = 64 * F_CPU / 16 / baud_rate;
		
	 PORTB_DIR |= (1 << PIN2_bp);

	 PORTB_OUT |= (1 << PIN2_bp);

	 PORTB_DIR &= ~(1 << PIN3_bp);

     PORTB_DIR |= (1 << PIN0_bp);
	 
	 USART0_BAUDH = (uint8_t)(cal_baud>>8); // Set the baud rate
	 USART0_BAUDL = (uint8_t)cal_baud;

	 USART0_CTRLA |= (1 << USART_RS4850_bp);  // enable RS485 Support
		
	 USART0_CTRLB = (1 << USART_RXEN_bp) | (1 << USART_TXEN_bp);  // enable RX and TX

	 USART0_CTRLC = (1 << USART_CHSIZE0_bp) | (1 << USART_CHSIZE1_bp); // 8 bit None parity 1stop bit

	
 }

 void transmit_byte( uint8_t data ){
	
	 while ( !( USART0_STATUS & (1 << USART_DREIF_bp)) );  // Wait for empty buffer.
	 USART0_TXDATAL = data;            // Put data into buffer.
 }

 uint8_t receive_byte(void) {
	 loop_until_bit_is_set(USART0_STATUS, USART_RXCIE_bp);       /* Wait for incoming data */
	 return USART0_RXDATAL;                                /* return register value */
 }

 char nibbleToHexCharacter(uint8_t nibble) {
	 /* Converts 4 bits into hexadecimal */
	 if (nibble < 10) {
		 return ('0' + nibble);
	 }
	 else {
		 return ('A' + nibble - 10);
	 }
 }

 void GetHexString(uint8_t byte, char *out)
 {
	 uint8_t nibble;
	 nibble = (byte & 0b11110000) >> 4;
	 out[0] = nibbleToHexCharacter(nibble);
	 nibble = byte & 0b00001111;
	 out[1] = nibbleToHexCharacter(nibble);
 }
 
 uint8_t GetByteFromString(char *in)
 {
	 uint8_t out = 0x00;
	 
	 if (in[0] >= 0x30 && in[0] <= 0x39)
	 {
		 out += (in[0] - 0x30) << 4;
	 }
	 else if (in[0] >= 0x41 && in[0] <= 0x46)
	 {
		 out += (in[0] - 0x37) << 4;
	 }
	 
	 if (in[1] >= 0x30 && in[1] <= 0x39)
	 {
		 out += (in[1] - 0x30);
	 }
	 else if (in[1] >= 0x41 && in[1] <= 0x46)
	 {
		 out += (in[1] - 0x37);
		 
	 }
	 
	 
	 return out;
 }

 void print_bytes(uint8_t myBytes[], uint8_t len, int check_sum_enable)
 {
	 uint8_t i = 0;
	 int checksum = 0;
	 char tmp[2];
	 
	 if (check_sum_enable == 1)
	 {
		 for(i = 0; i < len; i++)
		 {
			 transmit_byte(myBytes[i]);
			 checksum += myBytes[i];
		 }
		 checksum = checksum & 0xff;
		 GetHexString((uint8_t)checksum, tmp);
		 transmit_byte(tmp[0]);
		 transmit_byte(tmp[1]);
	 }
	 else{
		 for(i = 0; i < len; i++)
		 {
			 transmit_byte(myBytes[i]);
		 }
	 }

	 transmit_byte(0x0d);
	 
 }