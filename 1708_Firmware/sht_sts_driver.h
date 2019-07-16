/*
 * sts35_driver.h
 *
 * Created: 7/16/2019 10:42:17 AM
 *  Author: kangmin
 */ 


#ifndef STS35_DRIVER_H_
#define STS35_DRIVER_H_

void init_sts35_sensor();
void get_periodic_data(float *temp_out);
void enable_periodic_daq();
void read_temp(float *temp_out);

#endif /* STS35_DRIVER_H_ */