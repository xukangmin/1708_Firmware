/*
 * sts35_driver.c
 *
 * Created: 7/16/2019 10:42:07 AM
 *  Author: kangmin
 */ 
#define STS3X_DEFAULT_ADDR			   0x4A
#define STS3X_MEAS_HIGHREP_STRETCH   0x2C06
#define STS3X_MEAS_MEDREP_STRETCH    0x2C0D
#define STS3X_MEAS_LOWREP_STRETCH    0x2C10
#define STS3X_MEAS_HIGHREP           0x2400
#define STS3X_MEAS_MEDREP            0x240B
#define STS3X_MEAS_LOWREP            0x2416
#define STS3X_READSTATUS             0xF32D
#define STS3X_CLEARSTATUS            0x3041
#define STS3X_SOFTRESET              0x30A2
#define STS3X_HEATEREN               0x306D
#define STS3X_HEATERDIS              0x3066

#define STS3X_FETCH_DATA			 0xE000
#define STS3X_STOP_PERIODIC_DAQ      0x3093

#define STS3X_PERIODIC_DAQ_0_5_HIGH  0x2032
#define STS3X_PERIODIC_DAQ_0_5_MED   0x2024
#define STS3X_PERIODIC_DAQ_0_5_LOW	 0x202F
#define STS3X_PERIODIC_DAQ_1_HIGH    0x2130
#define STS3X_PERIODIC_DAQ_1_MED     0x2126
#define STS3X_PERIODIC_DAQ_1_LOW     0x212D
#define STS3X_PERIODIC_DAQ_2_HIGH    0x2236
#define STS3X_PERIODIC_DAQ_2_MED     0x2220
#define STS3X_PERIODIC_DAQ_2_LOW     0x222B
#define STS3X_PERIODIC_DAQ_4_HIGH    0x2334
#define STS3X_PERIODIC_DAQ_4_MED     0x2322
#define STS3X_PERIODIC_DAQ_4_LOW     0x2329
#define STS3X_PERIODIC_DAQ_10_HIGH   0x2737
#define STS3X_PERIODIC_DAQ_10_MED    0x2721
#define STS3X_PERIODIC_DAQ_10_LOW    0x272A

#define SHT3X_DEFAULT_ADDR    0x44
#define SHT3X_MEAS_HIGHREP_STRETCH 0x2C06
#define SHT3X_MEAS_MEDREP_STRETCH  0x2C0D
#define SHT3X_MEAS_LOWREP_STRETCH  0x2C10
#define SHT3X_MEAS_HIGHREP         0x2400
#define SHT3X_MEAS_MEDREP          0x240B
#define SHT3X_MEAS_LOWREP          0x2416
#define SHT3X_READSTATUS           0xF32D
#define SHT3X_CLEARSTATUS          0x3041
#define SHT3X_SOFTRESET            0x30A2
#define SHT3X_HEATEREN             0x306D
#define SHT3X_HEATERDIS            0x3066

#define F_CPU 3333333UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include "sht_sts_driver.h"
#include "twi_master.h"
#include <util/delay.h>

#define CLK_PER 3333333

#define TWI_BAUDRATE 100000

#define TWI_SBAUD (CLK_PER/(2*TWI_BAUDRATE) - 5)

uint8_t data_buf[6]; // for i2c data transmission

TWI_Master_t twi_master;
TWI_t inst;

ISR(TWI0_TWIM_vect)
{
	/* Needed for the TWI to complete transactions */
	TWI_MasterInterruptHandler(&twi_master);
}


void init_i2c() {

	PORTMUX_CTRLB |= PORTMUX_TWI0_ALTERNATE_gc;

	volatile uint8_t baud_rate = (CLK_PER/(2*TWI_BAUDRATE) - 5);
	
	TWI_MasterInit(&twi_master, &TWI0, (TWI_RIEN_bm | TWI_WIEN_bm), baud_rate);
}

void init_sensor() {
	init_i2c();
}

uint8_t crc8(const uint8_t *data, int len) {
  /*
   *
   * CRC-8 formula from page 14 of SHT spec pdf
   *
   * Test data 0xBE, 0xEF should yield 0x92
   *
   * Initialization data 0xFF
   * Polynomial 0x31 (x8 + x5 +x4 +1)
   * Final XOR 0x00
   */

  const uint8_t POLYNOMIAL = 0x31;
  uint8_t crc = 0xFF;

  for (int j = len; j; --j) {
    crc ^= *data++;

    for (int i = 8; i; --i) {
      crc = (crc & 0x80) ? (crc << 1) ^ POLYNOMIAL : (crc << 1);
    }
  }
  return crc;
}

void enable_periodic_daq() {
	
	data_buf[0] = STS3X_PERIODIC_DAQ_1_HIGH >> 8;
	data_buf[1] = (uint8_t)STS3X_PERIODIC_DAQ_1_HIGH;
	
	TWI_MasterWrite(&twi_master, SHT3X_DEFAULT_ADDR, data_buf, 2);
	
	_delay_ms(100);
	
	TWI_MasterWrite(&twi_master, STS3X_DEFAULT_ADDR, data_buf, 2);
	
	_delay_ms(100);
	

}


void get_periodic_temp_data(float *temp_out) {
	float temp = 0;
	
	data_buf[0] = STS3X_FETCH_DATA >> 8;
	data_buf[1] = (uint8_t)STS3X_FETCH_DATA;
	
	TWI_MasterWrite(&twi_master, STS3X_DEFAULT_ADDR, data_buf, 2);
	
	_delay_ms(50);
	
	TWI_MasterRead(&twi_master, STS3X_DEFAULT_ADDR, 3);

	_delay_ms(10);

	if (crc8((uint8_t*)twi_master.readData, 2) == twi_master.readData[2])
	{
		uint16_t tmp_raw = (twi_master.readData[0] * 256) + twi_master.readData[1];
		
		temp = (float)tmp_raw;
		
		temp = -45.0 + (175.0 * temp / 65535.0);
	} 
	else 
	{
		temp = 999.99;
	}



	(*temp_out) = temp;
}

void get_periodic_rh_temp_data(float *rh_out, float *temp_out) {
	float temp = 0;
	float hum = 0;
	uint16_t tmp_rd = 0;
	uint16_t hum_rd = 0;
	
	data_buf[0] = STS3X_FETCH_DATA >> 8;
	data_buf[1] = (uint8_t)STS3X_FETCH_DATA;
	
	TWI_MasterWrite(&twi_master, SHT3X_DEFAULT_ADDR, data_buf, 2);
	
	_delay_ms(20);
	
	TWI_MasterRead(&twi_master, SHT3X_DEFAULT_ADDR, 6);

	_delay_ms(10);

	
	tmp_rd = (twi_master.readData[0] * 256) + twi_master.readData[1];
	temp = (float)tmp_rd;
	temp = -45.0 + (175.0 * temp / 65535.0);
	hum_rd = (twi_master.readData[3] * 256) + twi_master.readData[4];
	hum = (float)hum_rd;
	hum = (100.0 * hum) / 65535.0;
	
	(*rh_out) = hum;
	(*temp_out) = temp;
}

void read_temp(float *temp_out)
{
	float temp = 0;
	
	data_buf[0] = STS3X_MEAS_HIGHREP >> 8;
	data_buf[1] = (uint8_t)STS3X_MEAS_HIGHREP;
	
	TWI_MasterWrite(&twi_master, STS3X_DEFAULT_ADDR, data_buf, 2);
	
	_delay_ms(50);
	
	TWI_MasterRead(&twi_master, STS3X_DEFAULT_ADDR, 3);

	_delay_ms(10);


	uint16_t tmp_raw = (twi_master.readData[0] * 256) + twi_master.readData[1];
	
	temp = (float)tmp_raw;
	
	temp = -45.0 + (175.0 * temp / 65535.0);


	(*temp_out) = temp;
}


void read_temp_rh(float *rh_out, float *temp_out)
{
	data_buf[0] = SHT3X_MEAS_HIGHREP_STRETCH >> 8;
	data_buf[1] = (uint8_t)SHT3X_MEAS_HIGHREP_STRETCH;
	
	TWI_MasterWrite(&twi_master, SHT3X_DEFAULT_ADDR, data_buf, 2);
	
	_delay_ms(50);
	
	TWI_MasterRead(&twi_master, SHT3X_DEFAULT_ADDR, 6);

	_delay_ms(10);
	
	// discard fist reading
	
	// average 3 readings
	float temp = 0;
	float hum = 0;
	uint16_t tmp_rd = 0;
	uint16_t hum_rd = 0;

	TWI_MasterWrite(&twi_master, SHT3X_DEFAULT_ADDR, data_buf, 2);

	_delay_ms(20);

	TWI_MasterRead(&twi_master, SHT3X_DEFAULT_ADDR, 6);

	_delay_ms(10);

	tmp_rd = (twi_master.readData[0] * 256) + twi_master.readData[1];
	temp = (float)tmp_rd;
	temp = -45.0 + (175.0 * temp / 65535.0);
	hum_rd = (twi_master.readData[3] * 256) + twi_master.readData[4];
	hum = (float)hum_rd;
	hum = (100.0 * hum) / 65535.0;
		

	(*temp_out) = temp;
	(*rh_out) = hum;
}