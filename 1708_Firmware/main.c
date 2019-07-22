/*
 * RH_SENSOR_RS485.c
 *
 * Created: 6/13/2018 7:10:11 AM
 * Author : kangm
 */


#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdio.h>

#define F_CPU 3333333UL
#define CLK_PER 3333333

#define TWI_BAUDRATE 100000

#define TWI_SBAUD (CLK_PER/(2*TWI_BAUDRATE) - 5)

#include <util/delay.h>
#include <avr/sfr_defs.h>
#include "uart_lib.h"
#include "sht_sts_driver.h"

#define PB5 5

#define PB_SCL_PIN 0
#define PB_SDA_PIN 1


#define DEFAULT_CAL_A 0.0f
#define DEFAULT_CAL_B 1.0f
#define DEFAULT_CAL_C 0.0f
#define DEFAULT_CAL_D 0.0f
#define DEFAULT_CAL_E 1.0f
#define DEFAULT_CAL_F 0.0f

#define DEFAULT_SETTING "41020030"
// setting = [Single Address x 2 bytes] + [uart baud rate x 2 bytes] + [check_sum_enable x 1 byte] + [delay x 1 byte] + [avg_level x 1 byte]

#define DEFAULT_ADDR "08A012"
#define DEFAULT_SINGLE_PRIMARY_ID 0x41
#define DEFAULT_SINGLE_TEMP_ADDR  0x42
#define DEFAULT_SINGLE_RH_ADDR	  0x43
#define DEFAULT_BAUD_BYTE		  0x02   // baud rate 9600 

#define TEMP_ADDR_BYTE			   0x61   // a
#define RH_ADDR_BYTE			   0x6E   // n
#define RH_TEMP_ADDR_BYTE		   0x6D   // m

#define SINGLE_ADDRESS_LEN 1
#define ADDRESS_LEN 6
#define CMD_LEN2 2
#define CMD_LEN3 2

#define CAL_NUM_LEN 12

#define ALWAYS_RESP_ADDR "999999"

#define ERROR_DATA 999.99f

#define MAX_BUFFER_SIZE 128

const float ERROR_GEN = 99999.99;
const float ERROR_BAD_CHECKSUM = 99999.01;
const float ERROR_SYNTAX = 99999.02;
const float ERROR_ADDRESS = 99999.03;
const float ERROR_COMMAND = 99999.04;
const float ERROR_VALUE = 99999.05;
const float ERROR_WRITE_PROTECT = 99999.06;
const float ERROR_READING_ERROR = 99999.07;

uint8_t single_primary_addr;
uint8_t single_temp_addr;
uint8_t single_rh_addr;
uint8_t uart_baud_rate;

char addr[6],setting[8];
uint8_t data_buf[16];
uint8_t parse_buf[20];
uint8_t parse_buf_len = 0;
uint8_t write_enabled = 0;
uint8_t temp_write_enabled = 0;
uint8_t rh_write_enabled = 0;
uint8_t address_type = 0;
uint8_t address_length = 0;

int check_sum_enable = 0;
int delay = 0;
int data_size = 0;
int avg_level = 0;
volatile uint8_t result = 0;

float temp_data, rh_data;
float tmp_reading_temp, tmp_reading_rh, tmp_reading_rh_temp;

uint8_t rev = 0x00;
volatile uint8_t v = 0, m = 0, n = 0;
uint8_t recv[MAX_BUFFER_SIZE];
int send_enable = 0;
int send_size = 0;
uint8_t tmp[sizeof(float)];
int fstr_size = 0;
float cal_a,cal_b,cal_c,cal_d,cal_e,cal_f;
uint8_t tmp_addr, tmp_uart;
char hex_tmp[2];

uint8_t bUpdateUart = 0;

#define EEPROM_INIT_STATUS_BYTE 	 0x00

#define EEPROM_ADDR_START_BYTE  	 0x01

#define EEPROM_CAL_TEMP_START_BYTE   0x07
#define EEPROM_CAL_HUM_START_BYTE	 0x13

#define EEPROM_TEMP_ADDR_START_BYTE  0x1F
#define EEPROM_RH_ADDR_START_BYTE	 0x20
#define EEPROM_PRIMARY_ID_START_BYTE 0x21

#define EEPROM_SETTING_START_BYTE    0x22

#define INITED 0x01

#define DATA_RETAIN_SIZE 30
#define DATA_SIZE_PER_AVG_LEVEL 3  // max avg level = 9, so max data size = 27
#define TIMER0_TOP_VALUE 0xFF

const char test_float_str[] = "1";

volatile int data_index = 0;
volatile uint8_t temp_flag = 0;
volatile unsigned long timer0_count = 0;
volatile int current_data_index = 0;
volatile int handle_uart_flag = 0;
volatile int data_overflow_flag = 0;
float data_arr_temp[DATA_RETAIN_SIZE] = {0.0};
float data_arr_rh[DATA_RETAIN_SIZE] = {0.0};
float data_arr_rh_temp[DATA_RETAIN_SIZE] = {0.0};

volatile uint8_t read_data_flag = 1;

enum Cmd {
	RD = 0,
	WE,
	RCAL,
	WCAL,
	RSRN,
	RS,
	SU
};
const int cmd_len[] = {
	2,
	2,
	4,
	4,
	4,
	2,
	2
};

const char *cmd_list[] = {
	"RD",
	"WE",
	"RCAL",
	"WCAL" ,
	"RSRN",
	"RS",
	"SU"
};

ISR(USART0_RXC_vect)
{
	if (handle_uart_flag == 0)
	{
		rev = USART0_RXDATAL;
		if (rev == '#' && v == 0)
		{
			v = 1;
			memset(recv,0,sizeof(recv));
			recv[v - 1] = rev;
		}
		else if (v > 0) {
			recv[v] = rev;
			v++;

			if (v >= MAX_BUFFER_SIZE)
			{
				v = 0;
			}
			
			if (rev == 0x0d)
			{
				handle_uart_flag = 1;
			}
			} else {
			v = 0;
		}
		} else {
		USART0_RXDATAL;  // discard data when processing
	}
	USART0_STATUS |= ( 1 << USART_RXCIE_bp);
}

ISR(RTC_CNT_vect) {
	read_data_flag = 1;
	RTC_INTFLAGS |= (1 << RTC_OVF_bp);
}

unsigned char EEPROM_read(unsigned int ucAddress)
{
	while (NVMCTRL.STATUS & NVMCTRL_EEBUSY_bm);

	return *(uint8_t *)(EEPROM_START + ucAddress);
}

void EEPROM_write(unsigned int ucAddress, unsigned char ucData)
{

	while (NVMCTRL.STATUS & NVMCTRL_EEBUSY_bm);

	*(uint8_t *)(EEPROM_START + ucAddress) = ucData;

	CCP = CCP_SPM_gc;
	NVMCTRL.CTRLA = NVMCTRL_CMD_PAGEERASEWRITE_gc;
}


void get_avg_data(float *temp, int type) {
	
	float sum = 0;
	int data_count = 0;
	int i = 0;
	int start_index = 0;
	if (data_index == 0) {
		(*temp) = ERROR_DATA;
		return;
	}
	
	float* data_arr = NULL;
	
	switch(type) {
		case 0: 
			data_arr = data_arr_temp;
			break;
		case 1:
			data_arr = data_arr_rh;
			break;
		case 2:
			data_arr = data_arr_rh_temp;
			break;
	}
	
	if (avg_level == 0) {
		(*temp) = data_arr[data_index - 1];
		} else {
		if (data_overflow_flag && data_index - avg_level * DATA_SIZE_PER_AVG_LEVEL < 0) { // data overflowed
			for(i = 0; i < data_index; i++) {
				sum	+= data_arr[i];
				data_count++;
			}
			start_index = DATA_RETAIN_SIZE - avg_level * DATA_SIZE_PER_AVG_LEVEL + data_index;
			for(i = start_index; i < DATA_RETAIN_SIZE; i++) {
				sum += data_arr[i];
				data_count++;
			}
			
			
			} else {
			start_index = data_index - avg_level * DATA_SIZE_PER_AVG_LEVEL;
			if (start_index < 0) start_index = 0;
			for(i = start_index; i < data_index; i++) {
				sum += data_arr[i];
				data_count++;
			}
		}
		
		
		(*temp) = sum / (float)data_count;
	}

	
	
}


void build_output(float data)
{
	v--;
	write_enabled = 0;
	fstr_size = snprintf((char*)(recv + v), MAX_BUFFER_SIZE - v, "%+09.2f", data);
	v += fstr_size;
	send_size = v;
	send_enable = 1;
}

void build_cal_out_put(float a, float b, float c)
{
	v--;
	write_enabled = 0;
	fstr_size = snprintf((char*)(recv + v), MAX_BUFFER_SIZE - v, "%+012.4E", a);
	v += fstr_size;
	fstr_size = snprintf((char*)(recv + v), MAX_BUFFER_SIZE - v, "%+012.4E", b);
	v += fstr_size;
	fstr_size = snprintf((char*)(recv + v), MAX_BUFFER_SIZE - v, "%+012.4E", c);
	v += fstr_size;
	send_size = v;
	send_enable = 1;
}

void write_temp_cal_eeprom()
{
	int i;
	
	for(i = 0; i < sizeof(float); i++)
	{
		memset(tmp,0,sizeof(float));
		memcpy(tmp,&cal_a,sizeof(float));
		EEPROM_write(EEPROM_CAL_TEMP_START_BYTE + i, tmp[i]);
	}
	
	for(i = 0; i < sizeof(float); i++)
	{
		memset(tmp,0,sizeof(float));
		memcpy(tmp,&cal_b,sizeof(float));
		EEPROM_write(EEPROM_CAL_TEMP_START_BYTE + sizeof(float) + i, tmp[i]);
	}
	
	for(i = 0; i < sizeof(float); i++)
	{
		memset(tmp,0,sizeof(float));
		memcpy(tmp,&cal_c,sizeof(float));
		EEPROM_write(EEPROM_CAL_TEMP_START_BYTE + 2 * sizeof(float) + i, tmp[i]);
	}
}

void write_rh_cal_eeprom()
{
	int i;
	
	for(i = 0; i < sizeof(float); i++)
	{
		memset(tmp,0,sizeof(float));
		memcpy(tmp,&cal_d,sizeof(float));
		EEPROM_write(EEPROM_CAL_HUM_START_BYTE + i, tmp[i]);
	}
	
	for(i = 0; i < sizeof(float); i++)
	{
		memset(tmp,0,sizeof(float));
		memcpy(tmp,&cal_e,sizeof(float));
		EEPROM_write(EEPROM_CAL_HUM_START_BYTE + sizeof(float) + i, tmp[i]);
	}
	
	for(i = 0; i < sizeof(float); i++)
	{
		memset(tmp,0,sizeof(float));
		memcpy(tmp,&cal_f,sizeof(float));
		EEPROM_write(EEPROM_CAL_HUM_START_BYTE + 2 * sizeof(float) + i, tmp[i]);
	}
}

void handle_uart_buffer(){
	if (handle_uart_flag == 1) {
		
		if (recv[1] == single_primary_addr)
		{
			address_type = 0x01;
			address_length = SINGLE_ADDRESS_LEN;
			
			if (recv[2] == TEMP_ADDR_BYTE) // a
			{
				address_type = 0x02;
				address_length = SINGLE_ADDRESS_LEN + 1;
			}
			else if (recv[2] == RH_ADDR_BYTE) // n
			{
				address_type = 0x03;
				address_length = SINGLE_ADDRESS_LEN + 1;
			}
			else if (recv[2] == RH_TEMP_ADDR_BYTE) // m
			{
				address_type = 0x04;
				address_length = SINGLE_ADDRESS_LEN + 1;
			}
			
		}
		else if (recv[1] == single_temp_addr)
		{
			address_type = 0x05;
			address_length = SINGLE_ADDRESS_LEN;
		}
		else if (recv[1] == single_rh_addr)
		{
			address_type = 0x06;
			address_length = SINGLE_ADDRESS_LEN;
		}
		else if (memcmp(recv + 1, ALWAYS_RESP_ADDR,ADDRESS_LEN) == 0)
		{
			address_type = 0x07; // always_respond
			address_length = ADDRESS_LEN;
			
			if (recv[7] == TEMP_ADDR_BYTE) // a
			{
				address_type = 0x08;
				address_length = ADDRESS_LEN + 1;
			}
			else if (recv[7] == RH_ADDR_BYTE) // n
			{
				address_type = 0x09;
				address_length = ADDRESS_LEN + 1;
			}
			else if (recv[7] == RH_TEMP_ADDR_BYTE) // m
			{
				address_type = 0x0a;
				address_length = ADDRESS_LEN + 1;
			}
		}
		else if (memcmp(recv + 1, addr, ADDRESS_LEN) == 0)
		{
			address_type = 0x0b; // serial_number
			address_length = ADDRESS_LEN;
			
			if (recv[7] == TEMP_ADDR_BYTE) // a
			{
				address_type = 0x0c;
				address_length = ADDRESS_LEN + 1;
			}
			else if (recv[7] == RH_ADDR_BYTE) // n
			{
				address_type = 0x0d;
				address_length = ADDRESS_LEN + 1;
			}
			else if (recv[7] == RH_TEMP_ADDR_BYTE) // m
			{
				address_type = 0x0e;
				address_length = ADDRESS_LEN + 1;
			}
			
		}
		else
		{
			address_type = 0;
		}
		
		recv[0] = '*';
		/* RD temperature sensor response, all address cases */
		if ((address_type == 0x02 || address_type == 0x05 || address_type == 0x08 || address_type == 0x0c) &&
		memcmp(recv + 1 + address_length, cmd_list[RD], strlen(cmd_list[RD])) == 0 &&
		v == address_length + 2 + cmd_len[RD])
		{
			get_avg_data(&temp_data, 0);
			temp_data =  cal_a + cal_b * temp_data + cal_c * temp_data * temp_data;
			build_output(temp_data);
		} // RD RH sensor response, all address cases
		else if((address_type == 0x03 || address_type == 0x06 || address_type == 0x09 || address_type == 0x0d) &&
		memcmp(recv + 1 + address_length, cmd_list[RD], strlen(cmd_list[RD])) == 0 &&
		v == address_length + 2 + cmd_len[RD])
		{
			get_avg_data(&rh_data, 1);
			rh_data =  cal_d +  cal_e * rh_data + cal_f * rh_data * rh_data  ;
			build_output(rh_data);
		} // RD RH temp sensor response, all address cases
		else if((address_type == 0x04 || address_type == 0x0a || address_type == 0x0e) &&
		memcmp(recv + 1 + address_length, cmd_list[RD], strlen(cmd_list[RD])) == 0 &&
		v == address_length + 2 + cmd_len[RD])
		{
			get_avg_data(&temp_data, 2);
			build_output(temp_data);
		} // Other RD output
		else if ((address_type == 0x01 || address_type == 0x07 || address_type == 0x0b) &&
		memcmp(recv + 1 + address_length, cmd_list[RD], strlen(cmd_list[RD])) == 0 &&
		v == address_length + 2 + cmd_len[RD])
		{
			build_output(ERROR_SYNTAX);
		} // WE
		else if ((address_type == 0x01 || address_type == 0x02 || address_type == 0x03 || address_type == 0x07 || address_type == 0x08 || address_type == 0x09 || address_type == 0x0b || address_type == 0x0c || address_type == 0x0d) &&
		memcmp(recv + 1 + address_length, cmd_list[WE], strlen(cmd_list[WE])) == 0 &&
		v == address_length + 2 + cmd_len[WE])
		{
			write_enabled = 1;
			send_enable = 1;
			send_size = v - 1;
		} // Read Temp Cal
		else if ((address_type == 0x02 || address_type == 0x08 || address_type == 0x0c) &&
		memcmp(recv + 1 + address_length, cmd_list[RCAL], strlen(cmd_list[RCAL])) == 0 &&
		v == address_length + 2 + cmd_len[RCAL])
		{
			build_cal_out_put(cal_a, cal_b, cal_c);
		} // Read RH Cal
		else if ((address_type == 0x03 || address_type == 0x09 || address_type == 0x0d) &&
		memcmp(recv + 1 + address_length, cmd_list[RCAL], strlen(cmd_list[RCAL])) == 0 &&
		v == address_length + 2 + cmd_len[RCAL])
		{
			build_cal_out_put(cal_d, cal_e, cal_f);
		} // write temp cal
		else if ((address_type == 0x02 || address_type == 0x08 || address_type == 0x0c) &&
		memcmp(recv + 1 + address_length, cmd_list[WCAL], strlen(cmd_list[WCAL])) == 0 &&
		v == address_length + 2 + cmd_len[WCAL] + CAL_NUM_LEN * 3 &&
		write_enabled == 1)
		{
			memset(parse_buf, 0, sizeof(parse_buf));
			memcpy(parse_buf, recv + address_length + 1 + cmd_len[WCAL], CAL_NUM_LEN);
			cal_a = atof((char*)parse_buf);
			memset(parse_buf, 0, sizeof(parse_buf));
			memcpy(parse_buf, recv + address_length + 1 + CAL_NUM_LEN + cmd_len[WCAL], CAL_NUM_LEN);
			cal_b = atof((char*)parse_buf);
			memset(parse_buf, 0, sizeof(parse_buf));
			memcpy(parse_buf, recv + address_length + 1 + CAL_NUM_LEN * 2 + cmd_len[WCAL], CAL_NUM_LEN);
			cal_c = atof((char*)parse_buf);
			v -= CAL_NUM_LEN * 3;
			
			write_temp_cal_eeprom();
			build_cal_out_put(cal_a, cal_b, cal_c);
		} // write RH cal
		else if ((address_type == 0x03 || address_type == 0x09 || address_type == 0x0d) &&
		memcmp(recv + 1 + address_length, cmd_list[WCAL], strlen(cmd_list[WCAL])) == 0 &&
		v == address_length + 2 + cmd_len[WCAL] + CAL_NUM_LEN * 3 &&
		write_enabled == 1)
		{
			memset(parse_buf, 0, sizeof(parse_buf));
			memcpy(parse_buf, recv + address_length + 1 + cmd_len[WCAL], CAL_NUM_LEN);
			cal_d = atof((char*)parse_buf);
			memset(parse_buf, 0, sizeof(parse_buf));
			memcpy(parse_buf, recv + address_length + 1 + CAL_NUM_LEN + cmd_len[WCAL], CAL_NUM_LEN);
			cal_e = atof((char*)parse_buf);
			memset(parse_buf, 0, sizeof(parse_buf));
			memcpy(parse_buf, recv + address_length + 1 + CAL_NUM_LEN * 2 + cmd_len[WCAL], CAL_NUM_LEN);
			cal_f = atof((char*)parse_buf);
			v -= CAL_NUM_LEN * 3;
			
			write_rh_cal_eeprom();
			build_cal_out_put(cal_d, cal_e, cal_f);
		} // RSRN
		else if ((address_type == 0x01 || address_type == 0x07 || address_type == 0x0b) &&
		memcmp(recv + 1 + address_length, cmd_list[RSRN], strlen(cmd_list[RSRN])) == 0 &&
		v == address_length + 2 + cmd_len[RSRN])
		{
			v--;
			write_enabled = 0;
			memcpy(recv + v, addr, ADDRESS_LEN);
			v += ADDRESS_LEN;
			send_size = v;
			send_enable = 1;
		} // RS read settings address + baud + rate
		else if ((address_type == 0x01 || address_type == 0x07 || address_type == 0x0b) &&
		memcmp(recv + 1 + address_length, cmd_list[RS], strlen(cmd_list[RS])) == 0 &&
		v == address_length + 2 + cmd_len[RS])
		{
			v--;
			write_enabled = 0;
			memcpy(recv + v, setting, sizeof(setting));
			v += sizeof(setting);
			send_size = v;
			send_enable = 1;
		}  // RS read settings temp address
		else if ((address_type == 0x02 || address_type == 0x05 || address_type == 0x08 || address_type == 0x0c) &&
		memcmp(recv + 1 + address_length, cmd_list[RS], strlen(cmd_list[RS])) == 0 &&
		v == address_length + 2 + cmd_len[RS])
		{
			v--;
			write_enabled = 0;
			GetHexString(single_temp_addr, hex_tmp);
			memcpy(recv + v, hex_tmp, 2);
			v += 2;
			send_size = v;
			send_enable = 1;
		} // RS read settings rh address
		else if ((address_type == 0x03 || address_type == 0x06 || address_type == 0x09 || address_type == 0x0d) &&
		memcmp(recv + 1 + address_length, cmd_list[RS], strlen(cmd_list[RS])) == 0 &&
		v == address_length + 2 + cmd_len[RS])
		{
			v--;
			write_enabled = 0;
			GetHexString(single_rh_addr, hex_tmp);
			memcpy(recv + v, hex_tmp, 2);
			v += 2;
			send_size = v;
			send_enable = 1;
		} // SU write settings address + baud + rate
		else if ((address_type == 0x01 || address_type == 0x07 || address_type == 0x0b) &&
		memcmp(recv + 1 + address_length, cmd_list[SU], strlen(cmd_list[SU])) == 0 &&
		v >= address_length + 2 + cmd_len[SU] &&
		write_enabled == 1)
		{
			v--;
			write_enabled = 0;
			memcpy(hex_tmp, recv + address_length + 1 + cmd_len[SU], 2);
			tmp_addr = GetByteFromString(hex_tmp);
			memcpy(hex_tmp, recv + address_length + 3 + cmd_len[SU], 2);
			tmp_uart = GetByteFromString(hex_tmp);
			
			if (tmp_addr >= 0x21 && temp_data <= 0x7E)
			{
				setting[0] = recv[address_length + 1 + cmd_len[SU]];
				setting[1] = recv[address_length + 2 + cmd_len[SU]];
				single_primary_addr = tmp_addr;
				EEPROM_write(EEPROM_PRIMARY_ID_START_BYTE, single_primary_addr);
			}
			
			if (tmp_uart == 0x02 || tmp_uart == 0x04 || tmp_uart == 0x05 || tmp_uart == 0x06 || tmp_uart == 0x07)
			{
				if (tmp_uart != uart_baud_rate) {
					bUpdateUart = 1;
				}
				uart_baud_rate = tmp_uart;
				setting[2] = recv[address_length + 3 + cmd_len[SU]];
				setting[3] = recv[address_length + 4 + cmd_len[SU]];
			}
			
			setting[4] = recv[address_length + 5 + cmd_len[SU]];
			setting[5] = recv[address_length + 6 + cmd_len[SU]];
			setting[6] = recv[address_length + 7 + cmd_len[SU]];
			
			check_sum_enable = setting[4] - 0x30;
			delay = setting[5] - 0x30;
			avg_level = setting[6] - 0x30;
			
			for(int i = 0; i < 8; i++) {
				EEPROM_write(EEPROM_SETTING_START_BYTE + i, setting[i]);
			}
						
			send_size = v;
			send_enable = 1;
		}  // SU write settings temp address
		else if ((address_type == 0x02 || address_type == 0x05 || address_type == 0x08 || address_type == 0x0c) &&
		memcmp(recv + 1 + address_length, cmd_list[SU], strlen(cmd_list[SU])) == 0 &&
		v >= address_length + 2 + cmd_len[SU] &&
		write_enabled == 1)
		{
			v--;
			write_enabled = 0;
			memcpy(hex_tmp, recv + address_length + 1 + cmd_len[SU], 2);
			tmp_addr = GetByteFromString(hex_tmp);
			
			if (tmp_addr >= 0x21 && temp_data <= 0x7E)
			{
				single_temp_addr = tmp_addr;
				EEPROM_write(EEPROM_TEMP_ADDR_START_BYTE, single_temp_addr);
			}
			send_size = v;
			send_enable = 1;
		} // SU write settings rh address
		else if ((address_type == 0x03 || address_type == 0x06 || address_type == 0x09 || address_type == 0x0d) &&
		memcmp(recv + 1 + address_length, cmd_list[SU], strlen(cmd_list[SU])) == 0 &&
		v >= address_length + 2 + cmd_len[SU] &&
		write_enabled == 1)
		{
			v--;
			write_enabled = 0;
			memcpy(hex_tmp, recv + address_length + 1 + cmd_len[SU], 2);
			tmp_addr = GetByteFromString(hex_tmp);
			
			if (tmp_addr >= 0x21 && temp_data <= 0x7E)
			{
				single_rh_addr = tmp_addr;
				EEPROM_write(EEPROM_RH_ADDR_START_BYTE, single_rh_addr);
			}
			send_size = v;
			send_enable = 1;
		}
		
		if (send_enable == 1)
		{
			send_enable = 0;
			if (delay != 0)
			{
				for(n = 0; n < delay; n++)
				{
					_delay_us(1040);
				}
			}
			print_bytes((uint8_t *)recv, send_size, check_sum_enable);
			
			if (bUpdateUart)
			{
				_delay_ms(1000);
				bUpdateUart = 0;
				initUSART(uart_baud_rate);
			}
		}
		address_type = 0;
		v = 0;
		
		handle_uart_flag = 0;
	}
}

void init_config()
{
	int i;
	
	
	cal_a = DEFAULT_CAL_A;
	cal_b = DEFAULT_CAL_B;
	cal_c = DEFAULT_CAL_C;
	
	cal_d = DEFAULT_CAL_D;
	cal_e = DEFAULT_CAL_E;
	cal_f = DEFAULT_CAL_F;

	for(i = 0; i < 6; i++)
	{
		addr[i] = DEFAULT_ADDR[i];
		EEPROM_write(EEPROM_ADDR_START_BYTE + i, DEFAULT_ADDR[i]);
	}
	
	// setting = [Single Address x 2 bytes] + [uart baud rate x 2 bytes] + [check_sum_enable x 1 byte] + [delay x 1 byte] + [avg_level x 1 byte]
	for(i = 0; i < 8; i++)
	{
		setting[i] = DEFAULT_SETTING[i];
		EEPROM_write(EEPROM_SETTING_START_BYTE + i, DEFAULT_SETTING[i]);
	}
	
	memcpy(hex_tmp, setting, 2);
	single_primary_addr = GetByteFromString(hex_tmp);
			
	memcpy(hex_tmp, setting + 2, 2);
	uart_baud_rate = GetByteFromString(hex_tmp);
	
	check_sum_enable = DEFAULT_SETTING[4] - 0x30;
	delay = DEFAULT_SETTING[5] - 0x30;
	avg_level = DEFAULT_SETTING[6] - 0x30;
	

	
	write_temp_cal_eeprom();
	write_rh_cal_eeprom();


	
	EEPROM_write(EEPROM_TEMP_ADDR_START_BYTE, DEFAULT_SINGLE_TEMP_ADDR);
	EEPROM_write(EEPROM_RH_ADDR_START_BYTE, DEFAULT_SINGLE_RH_ADDR);
	EEPROM_write(EEPROM_PRIMARY_ID_START_BYTE, DEFAULT_SINGLE_PRIMARY_ID);
	
}

uint8_t read_init()
{
	return EEPROM_read(EEPROM_INIT_STATUS_BYTE);
}

void load_config()
{
	int i;
	for(i = 0; i < 6; i++)
	{
		addr[i] = EEPROM_read(EEPROM_ADDR_START_BYTE + i);
	}
	
	for(i = 0; i < 8; i++)
	{
		setting[i] = EEPROM_read(EEPROM_SETTING_START_BYTE + i);
	}
	
	check_sum_enable = setting[4] - 0x30;
	delay = setting[5] - 0x30;
	avg_level = setting[6] - 0x30;
	
	memcpy(hex_tmp, setting, 2);
	single_primary_addr = GetByteFromString(hex_tmp);

	memcpy(hex_tmp, setting + 2, 2);
	uart_baud_rate = GetByteFromString(hex_tmp);
	
	single_temp_addr = EEPROM_read(EEPROM_TEMP_ADDR_START_BYTE);
	
	single_rh_addr = EEPROM_read(EEPROM_RH_ADDR_START_BYTE);
	

	memset(tmp,0,sizeof(tmp));
	for(m = 0; m < 4; m++)
	{
		tmp[m] = EEPROM_read(EEPROM_CAL_TEMP_START_BYTE + m);
	}
	memcpy(&cal_a, tmp, sizeof(float));
	
	memset(tmp,0,sizeof(tmp));
	for(m = 0; m < 4; m++)
	{
		tmp[m] = EEPROM_read(EEPROM_CAL_TEMP_START_BYTE + 0x04 + m);
	}
	memcpy(&cal_b, tmp, sizeof(float));

	memset(tmp,0,sizeof(tmp));
	for(m = 0; m < 4; m++)
	{
		tmp[m] = EEPROM_read(EEPROM_CAL_TEMP_START_BYTE + 0x08 + m);
	}
	memcpy(&cal_c, tmp, sizeof(float));


	memset(tmp,0,sizeof(tmp));
	for(m = 0; m < 4; m++)
	{
		tmp[m] = EEPROM_read(EEPROM_CAL_HUM_START_BYTE + m);
	}
	memcpy(&cal_d, tmp, sizeof(float));
	
	memset(tmp,0,sizeof(tmp));
	for(m = 0; m < 4; m++)
	{
		tmp[m] = EEPROM_read(EEPROM_CAL_HUM_START_BYTE + 0x04 + m);
	}
	memcpy(&cal_e, tmp, sizeof(float));

	memset(tmp,0,sizeof(tmp));
	for(m = 0; m < 4; m++)
	{
		tmp[m] = EEPROM_read(EEPROM_CAL_HUM_START_BYTE + 0x08 + m);
	}
	memcpy(&cal_f, tmp, sizeof(float));
}

int main(void)
{
	RTC.CLKSEL = 0x00;
	
	loop_until_bit_is_clear(RTC.STATUS, RTC_PERBUSY_bp);
	
	RTC_PER = 0x8000;  // set overflow to 32768, since clock is 32768Hz

	RTC.INTCTRL = (1 << RTC_OVF_bp);
	
	RTC.CTRLA = 0x01;
	
	if (read_init() != INITED)
	{
		EEPROM_write(EEPROM_INIT_STATUS_BYTE, INITED);
		init_config();
	}

	load_config();
		
	initUSART(uart_baud_rate);
		
	init_sensor();
	
	enable_periodic_daq();
	
	sei();	
	
	while(1)
	{
		handle_uart_buffer();
		
		if (read_data_flag) {
			read_data_flag = 0;
			// read temp and save to arr
			if (data_index >= DATA_RETAIN_SIZE) {
				data_index = 0;
				data_overflow_flag = 1;
			}
					

			if (current_data_index < data_index) {
				current_data_index = data_index;
			}

			get_periodic_temp_data(&tmp_reading_temp);
			get_periodic_rh_temp_data(&tmp_reading_rh,&tmp_reading_rh_temp);
			
			if (tmp_reading_temp >= -40 && tmp_reading_temp <= 125 && tmp_reading_rh_temp >= -40 && tmp_reading_rh_temp <= 125 && tmp_reading_rh >= 0 && tmp_reading_rh <= 100) {
				data_arr_temp[data_index] = tmp_reading_temp;
				data_arr_rh[data_index] = tmp_reading_rh;
				data_arr_rh_temp[data_index] = tmp_reading_rh_temp;
				data_index++;
			}
		}
	}
}
