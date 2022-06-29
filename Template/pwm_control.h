/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _PWM_CONTROL_H_
#define _PWM_CONTROL_H_

#include "stdint.h"

#define MOTOR_NUM 4

typedef struct edge_data_
{
	uint32_t edge[4][21];
	uint8_t edge_cnt[4];
} edge_data_str;

typedef struct edge_dispose_
{
	uint8_t start_flag ;
	uint8_t signal_cnt;
	uint8_t hight_low_status;
	uint8_t old_hight_low_status;
	uint8_t vessel_number;
} edge_dispose_str;

extern uint8_t dshoot_pin[MOTOR_NUM];
void pwm_control_timeout(void);
void pwm_control_protocol(uint8_t* datas);
uint32_t decodeTelemetryPacket(uint32_t buffer[], uint32_t count);
void run_cycle_1ms(void);
void data_parse(void);
void send_dshot600_ok(void);
void receive_dshoot_data_dispose(uint8_t *channel, uint16_t *data);

#endif
