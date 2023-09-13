#ifndef __DATA_SHOW_H
#define __DATA_SHOW_H
#include <stdint.h>


void data_show_init();
void data_show_push(float *chns, int chn_cnt);
void data_show_pull(unsigned char dat);
void data_show_parse();

void data_show_pull_array(uint8_t* datas, uint16_t length);

extern void data_show_on_config_pid(unsigned char chn, unsigned char pid_type, float kp, float ki, float kd);


#endif