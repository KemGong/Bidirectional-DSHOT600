/* Includes ------------------------------------------------------------------*/
#include "pwm_control.h"
#include "gd32e23x.h"
#include "systick.h"

#define TIMEOUT_CONTROL_PWM_COUNT (8 - 4)
#define ESC_CMD_BUFFER_LEN 18 

#define ESC_BIT_0     0
#define ESC_BIT_1     1

uint16_t dshot_cmd_ch[MOTOR_NUM][ESC_CMD_BUFFER_LEN]={0};
uint8_t dshoot_pin[MOTOR_NUM] = {0,1,4,5};
uint32_t bit_dshoot_cmd_data[MOTOR_NUM+1][ESC_CMD_BUFFER_LEN*3] = {0};

enum PWM_INDEX_NUMBER {
    PWM_1 = 0,
    PWM_2 = 1,
    PWM_3 = 2,
    PWM_4 = 3,
    PWM_MAX = 4,
};

uint32_t success_cnt = 0;
uint32_t fail_cnt = 0;
uint32_t fail_cnt1,fail_cnt2,fail_cnt3;
float percent;
uint32_t run_cnt = 0;

uint32_t decodeTelemetryPacket(uint32_t buffer[], uint32_t count)
{
    volatile uint32_t value = 0;
    uint32_t oldValue = buffer[0];
    int bits = 0;
    int len;
    
    run_cnt++;

    for (uint32_t i = 1; i <= count; i++) {
        if (i < count) {
            int diff = buffer[i] - oldValue;
            if (bits >= 21) {
                fail_cnt1++;
                break;
            }
            len = (diff + 3) / 6;
        } else {
            len = 21 - bits;
        }

        value <<= len;
        value |= 1 << (len - 1);
        oldValue = buffer[i];
        bits += len;
    }
    if (bits != 21) {
        //if (start_flag) {
        fail_cnt2++;//}
        return 0xffff;
    }

    static const uint32_t decode[32] = {
        0, 0, 0, 0, 0, 0, 0, 0, 0, 9, 10, 11, 0, 13, 14, 15,
        0, 0, 2, 3, 0, 5, 6, 7, 0, 0, 8, 1, 0, 4, 12, 0 };

    volatile uint32_t decodedValue = decode[value & 0x1f];
    decodedValue |= decode[(value >> 5) & 0x1f] << 4;
    decodedValue |= decode[(value >> 10) & 0x1f] << 8;
    decodedValue |= decode[(value >> 15) & 0x1f] << 12;

    uint32_t csum = decodedValue;
    csum = csum ^ (csum >> 8); // xor bytes
    csum = csum ^ (csum >> 4); // xor nibbles

    if ((csum & 0xf) != 0xf) {
        //if (start_flag) {
        fail_cnt3++;//}
        return 0xffff;
    }
    decodedValue >>= 4;

    if (decodedValue == 0x0fff) {
        return 0;
    }
    decodedValue = (decodedValue & 0x000001ff) << ((decodedValue & 0xfffffe00) >> 9);
    if (!decodedValue) {
        return 0xffff;
    }
    uint32_t ret = (1000000 * 60 / 100 + decodedValue / 2) / decodedValue;
		volatile float erpm = ret * 100;
		volatile float rpm = ret * 100 * 2 / 7;
    success_cnt++;
    //percent = ((fail_cnt1+fail_cnt2+fail_cnt3)*100)/(success_cnt+fail_cnt1+fail_cnt2+fail_cnt3);
    percent = ((fail_cnt2+fail_cnt3)*100)/(run_cnt);
    return ret;
}




#define DSHOT_NOP_DELAY() \
 __NOP();\
 __NOP();\
 __NOP();\
 __NOP();

#define DSHOT_CONTROL_H(x)  do { \
																			GPIO_BC(GPIOB) = ((uint32_t)((uint32_t)0x01U<<(x)));\
																			DSHOT_NOP_DELAY();\
																			DSHOT_NOP_DELAY();\
																			DSHOT_NOP_DELAY();\
																			DSHOT_NOP_DELAY();\
																			DSHOT_NOP_DELAY();\
																			DSHOT_NOP_DELAY();\
																			DSHOT_NOP_DELAY();\
																			DSHOT_NOP_DELAY();\
																			DSHOT_NOP_DELAY();\
																			DSHOT_NOP_DELAY();\
																			DSHOT_NOP_DELAY();\
																			DSHOT_NOP_DELAY();\
																			DSHOT_NOP_DELAY();\
																			DSHOT_NOP_DELAY();\		
																			DSHOT_NOP_DELAY();\		
																			DSHOT_NOP_DELAY();\		
																			DSHOT_NOP_DELAY();\					
																			DSHOT_NOP_DELAY()\
																			GPIO_BOP(GPIOB) = ((uint32_t)((uint32_t)0x01U<<(x)));\
																			DSHOT_NOP_DELAY();\	
																			DSHOT_NOP_DELAY();\	
																			DSHOT_NOP_DELAY();\	
																			DSHOT_NOP_DELAY();\	
																			DSHOT_NOP_DELAY();\	
																			DSHOT_NOP_DELAY();\	
																			DSHOT_NOP_DELAY();\																				
																	  	DSHOT_NOP_DELAY()\
                                      } while(0)
#define DSHOT_CONTROL_L(x)  do { \
																		  GPIO_BC(GPIOB) = ((uint32_t)((uint32_t)0x01U<<(x)));\
																		  DSHOT_NOP_DELAY();\
																	  	DSHOT_NOP_DELAY();\
																			DSHOT_NOP_DELAY();\
																			DSHOT_NOP_DELAY();\
																			DSHOT_NOP_DELAY();\
																		  DSHOT_NOP_DELAY();\																					
																	  	DSHOT_NOP_DELAY()\
																			GPIO_BOP(GPIOB) = ((uint32_t)((uint32_t)0x01U<<(x)));\
																			DSHOT_NOP_DELAY();\
																	  	DSHOT_NOP_DELAY();\		
																			DSHOT_NOP_DELAY();\
																			DSHOT_NOP_DELAY();\
																			DSHOT_NOP_DELAY();\
																			DSHOT_NOP_DELAY();\
																			DSHOT_NOP_DELAY();\
																			DSHOT_NOP_DELAY();\
																			DSHOT_NOP_DELAY();\
																			DSHOT_NOP_DELAY();\
																			DSHOT_NOP_DELAY();\
																			DSHOT_NOP_DELAY();\
																			DSHOT_NOP_DELAY();\
																			DSHOT_NOP_DELAY();\
																			DSHOT_NOP_DELAY();\
																			DSHOT_NOP_DELAY();\							
																			DSHOT_NOP_DELAY();\					
																			DSHOT_NOP_DELAY();\
																			DSHOT_NOP_DELAY();\
																			DSHOT_NOP_DELAY();\
																			DSHOT_NOP_DELAY();\
																			DSHOT_NOP_DELAY()\
                                      } while(0)
           
void dshot_control(uint16_t channel,uint16_t *dshot_data)
{	 	
			if(dshot_data[0]) DSHOT_CONTROL_H(channel);
	    else DSHOT_CONTROL_L(channel);
			if(dshot_data[1]) DSHOT_CONTROL_H(channel);
	    else DSHOT_CONTROL_L(channel);
			if(dshot_data[2]) DSHOT_CONTROL_H(channel);
	    else DSHOT_CONTROL_L(channel);
			if(dshot_data[3]) DSHOT_CONTROL_H(channel);
	    else DSHOT_CONTROL_L(channel);
			if(dshot_data[4]) DSHOT_CONTROL_H(channel);
	    else DSHOT_CONTROL_L(channel);
			if(dshot_data[5]) DSHOT_CONTROL_H(channel);
	    else DSHOT_CONTROL_L(channel);
			if(dshot_data[6]) DSHOT_CONTROL_H(channel);
	    else DSHOT_CONTROL_L(channel);
			if(dshot_data[7]) DSHOT_CONTROL_H(channel);
	    else DSHOT_CONTROL_L(channel);
			if(dshot_data[8]) DSHOT_CONTROL_H(channel);
	    else DSHOT_CONTROL_L(channel);
			if(dshot_data[9]) DSHOT_CONTROL_H(channel);
	    else DSHOT_CONTROL_L(channel);
			if(dshot_data[10]) DSHOT_CONTROL_H(channel);
	    else DSHOT_CONTROL_L(channel);
			if(dshot_data[11]) DSHOT_CONTROL_H(channel);
	    else DSHOT_CONTROL_L(channel);
			if(dshot_data[12]) DSHOT_CONTROL_H(channel);
	    else DSHOT_CONTROL_L(channel);
			if(dshot_data[13]) DSHOT_CONTROL_H(channel);
	    else DSHOT_CONTROL_L(channel);
			if(dshot_data[14]) DSHOT_CONTROL_H(channel);
	    else DSHOT_CONTROL_L(channel);
			if(dshot_data[15]) DSHOT_CONTROL_H(channel);
	    else DSHOT_CONTROL_L(channel);		
}


#define DSHOT_CONTROL(x,y,z)  do { \
																			GPIO_BOP(GPIOB) = x;\
																			DSHOT_NOP_DELAY();\
																			DSHOT_NOP_DELAY();\
																			DSHOT_NOP_DELAY();\
																			DSHOT_NOP_DELAY();\
																			DSHOT_NOP_DELAY();\
																			DSHOT_NOP_DELAY();\
																			DSHOT_NOP_DELAY();\
																			GPIO_BOP(GPIOB) = y;\
																			DSHOT_NOP_DELAY();\
																			DSHOT_NOP_DELAY();\
																			DSHOT_NOP_DELAY();\
																			DSHOT_NOP_DELAY();\
																			DSHOT_NOP_DELAY();\
																			DSHOT_NOP_DELAY();\
																			DSHOT_NOP_DELAY();\
																			DSHOT_NOP_DELAY();\
																			DSHOT_NOP_DELAY();\
																			DSHOT_NOP_DELAY();\
																			DSHOT_NOP_DELAY()\
																			GPIO_BOP(GPIOB) = z;\
																			DSHOT_NOP_DELAY();\
																			DSHOT_NOP_DELAY();\
																			DSHOT_NOP_DELAY();\
																			DSHOT_NOP_DELAY();\
																			DSHOT_NOP_DELAY();\
																			DSHOT_NOP_DELAY();\
																			DSHOT_NOP_DELAY();\
																			DSHOT_NOP_DELAY()\
																			} while(0)


void dshot_control_4_gpio(uint32_t (*data)[54])
{
	DSHOT_CONTROL(data[MOTOR_NUM][0],data[MOTOR_NUM][1],data[MOTOR_NUM][2]);
	DSHOT_CONTROL(data[MOTOR_NUM][3],data[MOTOR_NUM][4],data[MOTOR_NUM][5]);
	DSHOT_CONTROL(data[MOTOR_NUM][6],data[MOTOR_NUM][7],data[MOTOR_NUM][8]);
	DSHOT_CONTROL(data[MOTOR_NUM][9],data[MOTOR_NUM][10],data[MOTOR_NUM][11]);
	DSHOT_CONTROL(data[MOTOR_NUM][12],data[MOTOR_NUM][13],data[MOTOR_NUM][14]);
	DSHOT_CONTROL(data[MOTOR_NUM][15],data[MOTOR_NUM][16],data[MOTOR_NUM][17]);
	DSHOT_CONTROL(data[MOTOR_NUM][18],data[MOTOR_NUM][19],data[MOTOR_NUM][20]);
	DSHOT_CONTROL(data[MOTOR_NUM][21],data[MOTOR_NUM][22],data[MOTOR_NUM][23]);
	DSHOT_CONTROL(data[MOTOR_NUM][24],data[MOTOR_NUM][25],data[MOTOR_NUM][26]);
	DSHOT_CONTROL(data[MOTOR_NUM][27],data[MOTOR_NUM][28],data[MOTOR_NUM][29]);
	DSHOT_CONTROL(data[MOTOR_NUM][30],data[MOTOR_NUM][31],data[MOTOR_NUM][32]);
	DSHOT_CONTROL(data[MOTOR_NUM][33],data[MOTOR_NUM][34],data[MOTOR_NUM][35]);
	DSHOT_CONTROL(data[MOTOR_NUM][36],data[MOTOR_NUM][37],data[MOTOR_NUM][38]);
	DSHOT_CONTROL(data[MOTOR_NUM][39],data[MOTOR_NUM][40],data[MOTOR_NUM][41]);
	DSHOT_CONTROL(data[MOTOR_NUM][42],data[MOTOR_NUM][43],data[MOTOR_NUM][44]);
	DSHOT_CONTROL(data[MOTOR_NUM][45],data[MOTOR_NUM][46],data[MOTOR_NUM][47]);
	DSHOT_CONTROL(data[MOTOR_NUM][48],data[MOTOR_NUM][49],data[MOTOR_NUM][50]);
	DSHOT_CONTROL(data[MOTOR_NUM][51],data[MOTOR_NUM][52],data[MOTOR_NUM][53]);
}


#define DSHOT_RECEIVE(x,y,z)	x = GPIO_ISTAT(GPIOB);\
								DSHOT_NOP_DELAY();\
								DSHOT_NOP_DELAY();\
								DSHOT_NOP_DELAY();\
								DSHOT_NOP_DELAY();\
								DSHOT_NOP_DELAY();\
								DSHOT_NOP_DELAY();\
								DSHOT_NOP_DELAY();\
								y = GPIO_ISTAT(GPIOB);\
								DSHOT_NOP_DELAY();\
								DSHOT_NOP_DELAY();\
								DSHOT_NOP_DELAY();\
								DSHOT_NOP_DELAY();\
								DSHOT_NOP_DELAY();\
								DSHOT_NOP_DELAY();\
								DSHOT_NOP_DELAY();\
								z = GPIO_ISTAT(GPIOB);\
								DSHOT_NOP_DELAY();\
								DSHOT_NOP_DELAY();\
								DSHOT_NOP_DELAY();\
								DSHOT_NOP_DELAY();\
								DSHOT_NOP_DELAY();\
								DSHOT_NOP_DELAY();\
								DSHOT_NOP_DELAY()
								//} while(0)

uint16_t dshoot_dma_buffe_4[140];//32
void dshot_receive_4_gpio(void)//uint16_t *data
{
	for (uint8_t i = 0; i < 140; i+=3)
	{
		DSHOT_RECEIVE(dshoot_dma_buffe_4[i],dshoot_dma_buffe_4[i+1],dshoot_dma_buffe_4[i+2]);
	}
	/*
	DSHOT_RECEIVE(dshoot_dma_buffer[3],dshoot_dma_buffer[4],dshoot_dma_buffer[5]);
	DSHOT_RECEIVE(dshoot_dma_buffer[6],dshoot_dma_buffer[7],dshoot_dma_buffer[8]);
	DSHOT_RECEIVE(dshoot_dma_buffer[9],dshoot_dma_buffer[10],dshoot_dma_buffer[11]);
	DSHOT_RECEIVE(dshoot_dma_buffer[12],dshoot_dma_buffer[13],dshoot_dma_buffer[14]);
	DSHOT_RECEIVE(dshoot_dma_buffer[15],dshoot_dma_buffer[16],dshoot_dma_buffer[17]);
	DSHOT_RECEIVE(dshoot_dma_buffer[18],dshoot_dma_buffer[19],dshoot_dma_buffer[20]);
	DSHOT_RECEIVE(dshoot_dma_buffer[21],dshoot_dma_buffer[22],dshoot_dma_buffer[23]);
	DSHOT_RECEIVE(dshoot_dma_buffer[24],dshoot_dma_buffer[25],dshoot_dma_buffer[26]);
	DSHOT_RECEIVE(dshoot_dma_buffer[27],dshoot_dma_buffer[28],dshoot_dma_buffer[29]);
	DSHOT_RECEIVE(dshoot_dma_buffer[30],dshoot_dma_buffer[31],dshoot_dma_buffer[32]);
	DSHOT_RECEIVE(dshoot_dma_buffer[33],dshoot_dma_buffer[34],dshoot_dma_buffer[35]);
	DSHOT_RECEIVE(dshoot_dma_buffer[36],dshoot_dma_buffer[37],dshoot_dma_buffer[38]);
	DSHOT_RECEIVE(dshoot_dma_buffer[39],dshoot_dma_buffer[40],dshoot_dma_buffer[41]);
	DSHOT_RECEIVE(dshoot_dma_buffer[42],dshoot_dma_buffer[43],dshoot_dma_buffer[44]);
	DSHOT_RECEIVE(dshoot_dma_buffer[45],dshoot_dma_buffer[46],dshoot_dma_buffer[47]);
	DSHOT_RECEIVE(dshoot_dma_buffer[48],dshoot_dma_buffer[49],dshoot_dma_buffer[50]);
	DSHOT_RECEIVE(dshoot_dma_buffer[51],dshoot_dma_buffer[52],dshoot_dma_buffer[53]);
	DSHOT_RECEIVE(dshoot_dma_buffer[54],dshoot_dma_buffer[55],dshoot_dma_buffer[56]);
	DSHOT_RECEIVE(dshoot_dma_buffer[57],dshoot_dma_buffer[58],dshoot_dma_buffer[59]);
	DSHOT_RECEIVE(dshoot_dma_buffer[60],dshoot_dma_buffer[61],dshoot_dma_buffer[62]);
	DSHOT_RECEIVE(dshoot_dma_buffer[63],dshoot_dma_buffer[64],dshoot_dma_buffer[65]);
	DSHOT_RECEIVE(dshoot_dma_buffer[66],dshoot_dma_buffer[67],dshoot_dma_buffer[68]);
	DSHOT_RECEIVE(dshoot_dma_buffer[69],dshoot_dma_buffer[70],dshoot_dma_buffer[71]);
	DSHOT_RECEIVE(dshoot_dma_buffer[72],dshoot_dma_buffer[73],dshoot_dma_buffer[74]);
	DSHOT_RECEIVE(dshoot_dma_buffer[75],dshoot_dma_buffer[76],dshoot_dma_buffer[77]);
	DSHOT_RECEIVE(dshoot_dma_buffer[78],dshoot_dma_buffer[79],dshoot_dma_buffer[80]);
	DSHOT_RECEIVE(dshoot_dma_buffer[81],dshoot_dma_buffer[82],dshoot_dma_buffer[83]);
	DSHOT_RECEIVE(dshoot_dma_buffer[84],dshoot_dma_buffer[85],dshoot_dma_buffer[86]);
	DSHOT_RECEIVE(dshoot_dma_buffer[87],dshoot_dma_buffer[88],dshoot_dma_buffer[89]);
	DSHOT_RECEIVE(dshoot_dma_buffer[90],dshoot_dma_buffer[91],dshoot_dma_buffer[92]);
	DSHOT_RECEIVE(dshoot_dma_buffer[93],dshoot_dma_buffer[94],dshoot_dma_buffer[95]);
	DSHOT_RECEIVE(dshoot_dma_buffer[96],dshoot_dma_buffer[97],dshoot_dma_buffer[98]);
	DSHOT_RECEIVE(dshoot_dma_buffer[99],dshoot_dma_buffer[100],dshoot_dma_buffer[101]);
	DSHOT_RECEIVE(dshoot_dma_buffer[102],dshoot_dma_buffer[103],dshoot_dma_buffer[104]);
	DSHOT_RECEIVE(dshoot_dma_buffer[105],dshoot_dma_buffer[106],dshoot_dma_buffer[107]);
	DSHOT_RECEIVE(dshoot_dma_buffer[108],dshoot_dma_buffer[109],dshoot_dma_buffer[110]);
	DSHOT_RECEIVE(dshoot_dma_buffer[111],dshoot_dma_buffer[112],dshoot_dma_buffer[113]);
	DSHOT_RECEIVE(dshoot_dma_buffer[114],dshoot_dma_buffer[115],dshoot_dma_buffer[116]);
	DSHOT_RECEIVE(dshoot_dma_buffer[117],dshoot_dma_buffer[118],dshoot_dma_buffer[119]);
	DSHOT_RECEIVE(dshoot_dma_buffer[120],dshoot_dma_buffer[121],dshoot_dma_buffer[122]);
	DSHOT_RECEIVE(dshoot_dma_buffer[123],dshoot_dma_buffer[124],dshoot_dma_buffer[125]);
	DSHOT_RECEIVE(dshoot_dma_buffer[126],dshoot_dma_buffer[127],dshoot_dma_buffer[128]);
	DSHOT_RECEIVE(dshoot_dma_buffer[129],dshoot_dma_buffer[130],dshoot_dma_buffer[131]);
	DSHOT_RECEIVE(dshoot_dma_buffer[132],dshoot_dma_buffer[133],dshoot_dma_buffer[134]);
	DSHOT_RECEIVE(dshoot_dma_buffer[135],dshoot_dma_buffer[136],dshoot_dma_buffer[137]);
	DSHOT_RECEIVE(dshoot_dma_buffer[138],dshoot_dma_buffer[139],dshoot_dma_buffer[140]);*/
}

void dshot_control_4(uint8_t *channel,uint16_t (*dshot_data)[18], uint32_t (*data)[54])
{
    for(uint8_t i=0; i<4; i++)
    {
        //data head and end respectively pull high
        data[i][0] = 0x01U<<(channel[i]);
        data[i][1] = 0x01U<<(channel[i]);
        data[i][2] = 0x01U<<(channel[i]);
        data[i][51] = 0x01U<<(channel[i]);
        data[i][52] = 0x01U<<(channel[i]);
        data[i][53] = 0x01U<<(channel[i]);
        
        for(uint8_t j=0;j<16;j++)
        {
            if(dshot_data[i][j]) 
            {
                data[i][3+j*3] = 0x01U<<(channel[i]+16);
                data[i][3+j*3+1] = 0x01U<<(channel[i]+16);
                data[i][3+j*3+2] = 0x01U<<(channel[i]);
            }
            else
            {

                data[i][3+j*3] = 0x01U<<(channel[i]+16);
                data[i][3+j*3+1] = 0x01U<<(channel[i]);
                data[i][3+j*3+2] = 0x01U<<(channel[i]);

            }
        }
    }
    
    for(uint8_t i=0; i<54; i++)
    {
        data[4][i] = data[0][i] | data[1][i] | data[2][i] | data[3][i];
    }

}


uint16_t add_checksum_and_telemetry(uint16_t packet) {
    uint16_t packet_telemetry = (packet << 1) | 0;
    uint8_t i;
    int csum = 0;
    int csum_data = packet_telemetry;

    for (i = 0; i < 3; i++) {
        csum ^=  csum_data;   // xor data by nibbles
        csum_data >>= 4;
    }
    csum &= 0xf;
		packet_telemetry = (packet_telemetry << 4) | csum;
		
    return packet_telemetry;    //append checksum
}

uint16_t add_checksum_and_telemetry_double(uint16_t packet) {
    uint16_t packet_telemetry = (packet << 1) | 0;
    uint8_t i;
    int crc = 0;
    int csum_data = packet_telemetry;
	//crc = (csum_data ^ (csum_data >> 4) ^ (csum_data >>8)) & 0x0f;
	crc = (~(csum_data ^ (csum_data >> 4) ^ (csum_data >> 8))) & 0x0F;

	packet_telemetry = (packet_telemetry << 4) | crc;
		
    return packet_telemetry;    //append checksum
}


//static uint32_t raw_data[12] = {0, 16, 48, 80, 112, 144, 176, 208, 240, 272, 304, 336};
//static uint32_t raw_data[12] = {0, 16, 32, 96, 144, 176, 192, 208, 256, 288, 304, 336};
//static uint32_t raw_data[14] = {0, 16, 48, 80, 112, 144, 160, 176, 208, 224, 240, 256, 288, 304}; //////////////////
//static uint32_t raw_data[12] = {0, 32, 80, 96, 144, 176, 208, 240, 256, 272, 304, 320};
//static uint32_t raw_data[14] = {0, 15, 55, 70, 95, 110, 120, 130, 145, 170, 185, 195, 240, 265};//duque

volatile uint32_t dshoot_value = 0;

/*uint32_t decodeTelemetryPacket(uint32_t buffer[], uint32_t count)
{
    volatile uint32_t value = 0;
    uint32_t oldValue = buffer[0];
    int bits = 0;
    int len;
    for (uint32_t i = 1; i <= count; i++) {
        if (i < count) {
            int diff = buffer[i] - oldValue;
            if (bits >= 21) {
                break;
            }
            len = (diff + 8) / 16;
        } else {
            len = 21 - bits;
        }

        value <<= len;
        value |= 1 << (len - 1);
        oldValue = buffer[i];
        bits += len;
    }
    if (bits != 21) {
        return 0xffff;
    }

    static const uint32_t decode[32] = {
        0, 0, 0, 0, 0, 0, 0, 0, 0, 9, 10, 11, 0, 13, 14, 15,
        0, 0, 2, 3, 0, 5, 6, 7, 0, 0, 8, 1, 0, 4, 12, 0 };

    volatile uint32_t decodedValue = decode[value & 0x1f];
    decodedValue |= decode[(value >> 5) & 0x1f] << 4;
    decodedValue |= decode[(value >> 10) & 0x1f] << 8;
    decodedValue |= decode[(value >> 15) & 0x1f] << 12;

    uint32_t csum = decodedValue;
    csum = csum ^ (csum >> 8); // xor bytes
    csum = csum ^ (csum >> 4); // xor nibbles

    if ((csum & 0xf) != 0xf) {
        return 0xffff;
    }
    decodedValue >>= 4;

    if (decodedValue == 0x0fff) {
        return 0;
    }
    decodedValue = (decodedValue & 0x000001ff) << ((decodedValue & 0xfffffe00) >> 9);
    if (!decodedValue) {
        return 0xffff;
    }
    uint32_t ret = (1000000 * 60 / 100 + decodedValue / 2) / decodedValue;
		volatile float erpm = ret * 100;
		volatile float rpm = ret * 100 * 2 / 7;

    return ret;
}*/

void pwmWriteDigital(uint16_t *esc_cmd, uint16_t value)
{
    value = ( (value > 2047) ? 2047 : value );
	//value = 1046;
	//value = 2046;
    //volatile uint16_t value1 = add_checksum_and_telemetry(value);
	volatile uint16_t value1 = add_checksum_and_telemetry_double(value);
	//dshoot_value = decodeTelemetryPacket(raw_data, sizeof(raw_data)/sizeof(uint32_t));
    //timer_interrupt_disable(PWM_1_4_TIMER, TIMER_INT_FLAG_UP);
    esc_cmd[0]  = (value1 & 0x8000) ? ESC_BIT_1 : ESC_BIT_0;
    esc_cmd[1]  = (value1 & 0x4000) ? ESC_BIT_1 : ESC_BIT_0;
    esc_cmd[2]  = (value1 & 0x2000) ? ESC_BIT_1 : ESC_BIT_0;
    esc_cmd[3]  = (value1 & 0x1000) ? ESC_BIT_1 : ESC_BIT_0;
    esc_cmd[4]  = (value1 & 0x0800) ? ESC_BIT_1 : ESC_BIT_0;
    esc_cmd[5]  = (value1 & 0x0400) ? ESC_BIT_1 : ESC_BIT_0;
    esc_cmd[6]  = (value1 & 0x0200) ? ESC_BIT_1 : ESC_BIT_0;
    esc_cmd[7]  = (value1 & 0x0100) ? ESC_BIT_1 : ESC_BIT_0;
    esc_cmd[8]  = (value1 & 0x0080) ? ESC_BIT_1 : ESC_BIT_0;
    esc_cmd[9]  = (value1 & 0x0040) ? ESC_BIT_1 : ESC_BIT_0;
    esc_cmd[10] = (value1 & 0x0020) ? ESC_BIT_1 : ESC_BIT_0;
    esc_cmd[11] = (value1 & 0x0010) ? ESC_BIT_1 : ESC_BIT_0;     
    esc_cmd[12] = (value1 & 0x8) ? ESC_BIT_1 : ESC_BIT_0;
    esc_cmd[13] = (value1 & 0x4) ? ESC_BIT_1 : ESC_BIT_0;
    esc_cmd[14] = (value1 & 0x2) ? ESC_BIT_1 : ESC_BIT_0;
    esc_cmd[15] = (value1 & 0x1) ? ESC_BIT_1 : ESC_BIT_0;
    //timer_interrupt_enable(PWM_1_4_TIMER, TIMER_INT_FLAG_UP);
}


extern uint32_t send_buffer_dshoot[54];


#if 0
/**
  * @brief  This function handles TIMER15 interrupt request.
  * @param  None
  * @retval None
  */
extern uint16_t dshoot_dma_buffer[140];//32
extern uint8_t dma_status;
void TIMER15_IRQHandler(void)
{
    static uint8_t cnt = 0;
    static bool add_sub_flag = false;
    static  uint32_t index;
    static uint32_t ssec =0;
    if(SET == timer_interrupt_flag_get(TIMER15, TIMER_INT_UP)) {
        /* clear channel 0 interrupt bit */
        timer_interrupt_flag_clear(TIMER15, TIMER_INT_UP);
        //led_module_update();
        cnt_freq++;
        cnt++;

			  index++;
				if(index>=400) 
				{
					ssec++;
					index = 0;
				}
		//static uint8_t raw_data[] = {0XD0, 0X07, 0XD0, 0X07, 0XD0, 0X07,0XD0, 0X07};
		//static uint8_t raw_data[8] = {0X90, 0X01, 0X90, 0X01, 0X90, 0X01, 0X90, 0X01};
		static uint8_t raw_data[8] = {0XE8, 0X03, 0XE8, 0X03, 0XE8, 0X03, 0XE8, 0X03};
        uint8_t *data;
        data = raw_data;
		if (ssec < 5) {
			//raw_data[8] = {0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00};
			pwm_control_protocol(data);
			/*dshot_control(0,dshot_cmd_ch[0]);
			dshot_control(1,dshot_cmd_ch[1]);
			dshot_control(4,dshot_cmd_ch[2]);
			dshot_control(5,dshot_cmd_ch[3]);*/
            gpio_configuration_output();
            dshot_control_4(dshoot_pin,dshot_cmd_ch, bit_dshoot_cmd_data);
			memcpy(send_buffer_dshoot, &bit_dshoot_cmd_data[4][0], sizeof(send_buffer_dshoot));
			//__set_PRIMASK(1);
			dshot_control_4_gpio(bit_dshoot_cmd_data);
			/*dma_status = 0;
			dma_configuration_send_shoot();
			timer_configuration_send_dshoot();*/
			//__set_PRIMASK(0);
            //gpio_configuration_input();
            //timer_configuration_receive_dshoot();
            //dma_configuration_receive_dshoot();
			//memcpy(send_buffer_dshoot, &bit_dshoot_cmd_data[4][0], sizeof(send_buffer_dshoot));
		} else {
			raw_data[0] = 0X4c;
			raw_data[1] = 0X04;
			raw_data[2] = 0X4c;
			raw_data[3] = 0X04;
			raw_data[4] = 0X4c;
			raw_data[5] = 0X04;
			raw_data[6] = 0X4c;
			raw_data[7] = 0X04;
			pwm_control_protocol(data);
            gpio_configuration_output();
			dshot_control_4(dshoot_pin,dshot_cmd_ch, bit_dshoot_cmd_data);
			//__set_PRIMASK(1);
			/*dma_status = 0;
			memcpy(send_buffer_dshoot, &bit_dshoot_cmd_data[4][0], sizeof(send_buffer_dshoot));
			dma_configuration_send_shoot();
			timer_configuration_send_dshoot();*/
			dshot_control_4_gpio(bit_dshoot_cmd_data);
            gpio_configuration_input();
			dma_configuration_receive_dshoot();
            timer_configuration_receive_dshoot();
			//dshot_receive_4_gpio();
			//receive_dshoot_data_dispose(dshoot_pin,dshoot_dma_buffe_4,&edge_data);
			//__set_PRIMASK(0);
            //timer_configuration_receive_dshoot();
            //dma_configuration_receive_dshoot();
			/*memcpy(send_buffer_dshoot, &bit_dshoot_cmd_data[4][0], sizeof(send_buffer_dshoot));
			dma_configuration_send_shoot();
			timer_configuration_send_dshoot();*/
		}
    }
	  return;
      #if 0
    if(SET == timer_interrupt_flag_get(TIMER15, TIMER_INT_UP)){
        /* clear channel 0 interrupt bit */
        timer_interrupt_flag_clear(TIMER15, TIMER_INT_UP);
      //  gpio_bit_write(TEST_POINT_PORT,TEST_POINT_PIN,SET);
      
        system_timer = system_timer_coefficient * SYSTEM_TIMER_MAX_COUNT + timer_counter_read(SYSTEM_TIMER);
      
        //agen_run(&agen_roll,0.2f);
        //ctl_roll.power_f = 0.5f;
        //ctl_roll.angle_f = agen_roll.angle;
        
        ctl_roll.power_f = 1.0f/32768*ctl_roll.power_i16;
        ctl_roll.angle_f = 180.0f/32768*ctl_roll.angle_i16;
        control_servo_init(&ctl_roll.ctl_servo,ctl_roll.power_f,ctl_roll.angle_f);
        control_servo_run(&ctl_roll.ctl_servo);
        ctl_roll.range_u16 = PWM_RESOLUTION;
        ctl_roll.u_u16 = (uint16_t)(ctl_roll.ctl_servo.u * ctl_roll.range_u16);
        ctl_roll.v_u16 = (uint16_t)(ctl_roll.ctl_servo.v * ctl_roll.range_u16);
        ctl_roll.w_u16 = (uint16_t)(ctl_roll.ctl_servo.w * ctl_roll.range_u16);
      
        timer_channel_output_pulse_value_config(PWM_ROLL_TIMER, PWM_ROLL_U_CH, ctl_roll.u_u16);
        timer_channel_output_pulse_value_config(PWM_ROLL_TIMER, PWM_ROLL_V_CH, ctl_roll.v_u16);
        timer_channel_output_pulse_value_config(PWM_ROLL_TIMER, PWM_ROLL_W_CH, ctl_roll.w_u16);


        ctl_yaw.power_f = 1.0f/32768*ctl_yaw.power_i16;
        ctl_yaw.angle_f = 180.0f/32768*ctl_yaw.angle_i16;
        control_servo_init(&ctl_yaw.ctl_servo,ctl_yaw.power_f,ctl_yaw.angle_f);
        control_servo_run(&ctl_yaw.ctl_servo);
        ctl_yaw.range_u16 = PWM_RESOLUTION;
        ctl_yaw.u_u16 = (uint16_t)(ctl_yaw.ctl_servo.u * ctl_yaw.range_u16);
        ctl_yaw.v_u16 = (uint16_t)(ctl_yaw.ctl_servo.v * ctl_yaw.range_u16);
        ctl_yaw.w_u16 = (uint16_t)(ctl_yaw.ctl_servo.w * ctl_yaw.range_u16);
      
        timer_channel_output_pulse_value_config(PWM_YAW_TIMER, PWM_YAW_U_CH, ctl_yaw.u_u16);
        timer_channel_output_pulse_value_config(PWM_YAW_TIMER, PWM_YAW_V_CH, ctl_yaw.v_u16);
        timer_channel_output_pulse_value_config(PWM_YAW_TIMER, PWM_YAW_W_CH, ctl_yaw.w_u16);
        
        /*
        memcpy((uint8_t*)&com_usart_tx_raw_data[0],(uint8_t*)&system_timer,4);
        
        memcpy((uint8_t*)&com_usart_tx_raw_data[4],(uint8_t*)&ctl_roll.power_i16,2);
        memcpy((uint8_t*)&com_usart_tx_raw_data[6],(uint8_t*)&ctl_roll.angle_i16,2);
        memcpy((uint8_t*)&com_usart_tx_raw_data[8],(uint8_t*)&ctl_roll.u_u16,2);
        memcpy((uint8_t*)&com_usart_tx_raw_data[10],(uint8_t*)&ctl_roll.v_u16,2);
        memcpy((uint8_t*)&com_usart_tx_raw_data[12],(uint8_t*)&ctl_roll.w_u16,2);
        
        memcpy((uint8_t*)&com_usart_tx_raw_data[14],(uint8_t*)&ctl_yaw.power_i16,2);
        memcpy((uint8_t*)&com_usart_tx_raw_data[16],(uint8_t*)&ctl_yaw.angle_i16,2);
        memcpy((uint8_t*)&com_usart_tx_raw_data[18],(uint8_t*)&ctl_yaw.u_u16,2);
        memcpy((uint8_t*)&com_usart_tx_raw_data[20],(uint8_t*)&ctl_yaw.v_u16,2);
        memcpy((uint8_t*)&com_usart_tx_raw_data[22],(uint8_t*)&ctl_yaw.w_u16,2);
        
        memcpy((uint8_t*)&com_usart_tx_raw_data[24],(uint8_t*)&ctl_roll.hall_sin,2);
        memcpy((uint8_t*)&com_usart_tx_raw_data[26],(uint8_t*)&ctl_roll.hall_cos,2);
        memcpy((uint8_t*)&com_usart_tx_raw_data[28],(uint8_t*)&ctl_yaw.hall_sin,2);
        memcpy((uint8_t*)&com_usart_tx_raw_data[30],(uint8_t*)&ctl_yaw.hall_cos,2);
        
        com_usart_transmit_dma(com_usart_tx_raw_data, sizeof(com_usart_tx_raw_data));
        */
        
       // gpio_bit_write(TEST_POINT_PORT,TEST_POINT_PIN,RESET);
    }
    #endif
}
#endif


#define HIGHT_VALUE  0x01U<<(4+16)|0x01U<<(0+16)|0x01U<<(1+16)|0x01U<<(5+16),0x01U<<(4+16)|0x01U<<(0+16)|0x01U<<(1+16)|0x01U<<(5+16),0x01U<<(4)|0x01U<<(0)|0x01U<<(1)|0x01U<<(5)
#define LOW_VALUE    0x01U<<(4+16)|0x01U<<(0+16)|0x01U<<(1+16)|0x01U<<(5+16),0x01U<<(4)|0x01U<<(0)|0x01U<<(1)|0x01U<<(5),0x01U<<(4)|0x01U<<(0)|0x01U<<(1)|0x01U<<(5)
#define PULL_HIGH    0x01U<<(4)|0x01U<<(0)|0x01U<<(1)|0x01U<<(5),0x01U<<(4)|0x01U<<(0)|0x01U<<(1)|0x01U<<(5),0x01U<<(4)|0x01U<<(0)|0x01U<<(1)|0x01U<<(5)


uint32_t send_buffer_speed[54] = {PULL_HIGH,LOW_VALUE,LOW_VALUE,LOW_VALUE,HIGHT_VALUE,HIGHT_VALUE,//3+3*16+3
                            LOW_VALUE,LOW_VALUE,HIGHT_VALUE,LOW_VALUE,LOW_VALUE,LOW_VALUE,LOW_VALUE,LOW_VALUE,
                            HIGHT_VALUE,HIGHT_VALUE,HIGHT_VALUE,PULL_HIGH};

uint32_t send_buffer_0[54] = {PULL_HIGH,LOW_VALUE,LOW_VALUE,LOW_VALUE,LOW_VALUE,LOW_VALUE,//3+3*16+3
                            LOW_VALUE,LOW_VALUE,LOW_VALUE,LOW_VALUE,LOW_VALUE,LOW_VALUE,LOW_VALUE,HIGHT_VALUE,
                            HIGHT_VALUE,HIGHT_VALUE,HIGHT_VALUE,PULL_HIGH};

uint32_t cnt = 0;
uint8_t start_value_flag = 0;
uint32_t start_cnt = 0;
extern uint32_t run_cnt;
extern uint8_t dma_status;

uint32_t ms_cnt = 0;
uint8_t dma_status = 0;
                            
static uint8_t raw_data[8] = {0XE8, 0X03, 0XE8, 0X03, 0XE8, 0X03, 0XE8, 0X03};    
uint8_t *data;
void run_cycle_1ms(void)
{ 
        ms_cnt++;
		if (ms_cnt < 3000) {
            start_value_flag = 0;
            data = raw_data;
            start_cnt = run_cnt;
			pwm_control_protocol(data);
            gpio_configuration_output();
            dshot_control_4(dshoot_pin,dshot_cmd_ch, bit_dshoot_cmd_data);
			memcpy(send_buffer_dshoot, &bit_dshoot_cmd_data[4][0], sizeof(send_buffer_dshoot));
            
			//__set_PRIMASK(1);
			//dshot_control_4_gpio(bit_dshoot_cmd_data);//直接使用io口模式发送
			dma_status = 0;
			dma_configuration_send_shoot();
			timer_configuration_send_dshoot();
			//__set_PRIMASK(0);
		} else {
            //__set_PRIMASK(1);

            raw_data[0] = 0X4c;
            raw_data[1] = 0X04;
            raw_data[2] = 0X4c;
            raw_data[3] = 0X04;
            raw_data[4] = 0X4c;
            raw_data[5] = 0X04;
            raw_data[6] = 0X4c;
            raw_data[7] = 0X04;
            data = raw_data;
            start_value_flag = 1;

            pwm_control_protocol(data);
            dshot_control_4(dshoot_pin,dshot_cmd_ch, bit_dshoot_cmd_data);
            memcpy(send_buffer_dshoot, &bit_dshoot_cmd_data[4][0], sizeof(send_buffer_dshoot));
            gpio_configuration_output();
			
			dma_status = 0;
			dma_configuration_send_shoot();
			timer_configuration_send_dshoot();
            //__set_PRIMASK(0);
			//dshot_control_4_gpio(bit_dshoot_cmd_data);//直接使用io口模式发送


		}
}



void receive_dshoot_data_dispose(uint8_t *channel, uint16_t *data)
{
	edge_data_str edge_data = {0};
	edge_dispose_str edge_dispose = {0};
	for(uint8_t i=0;i<4;i++)
	{
		edge_dispose.old_hight_low_status = 0;
		edge_dispose.hight_low_status = 0;
		edge_dispose.start_flag = 0;
		edge_dispose.signal_cnt = 0;
        edge_dispose.vessel_number = 0;

		for(int j=0; j<140; j++)
		{
			if (!(data[j]>>channel[i] & 1))
			{
				edge_dispose.hight_low_status = 1;//低电平
				edge_dispose.start_flag = 1;
			}
			else
			{
				edge_dispose.hight_low_status = 0;//高电平
			}
			if (edge_dispose.start_flag)
			{
				edge_dispose.signal_cnt++;//信号次数
			}
			if (edge_dispose.old_hight_low_status != edge_dispose.hight_low_status)
			{
				edge_data.edge[i][edge_dispose.vessel_number++] = (edge_dispose.signal_cnt-1)*2;//信号计算从0开始
				edge_data.edge_cnt[i]++;

			}
			edge_dispose.old_hight_low_status = edge_dispose.hight_low_status;
		}

		if (start_value_flag)
		decodeTelemetryPacket(&edge_data.edge[i][0],edge_data.edge_cnt[i]);
	}
}


void send_dshot600_ok(void)
{
    cnt++;
    if (cnt<3000) {
        start_value_flag = 0;
        memcpy(send_buffer_dshoot, send_buffer_0, sizeof(send_buffer_dshoot));
        start_cnt = run_cnt;
    } else {
        start_value_flag = 1;
        memcpy(send_buffer_dshoot, send_buffer_speed, sizeof(send_buffer_dshoot));
    }
    dma_status = 0;
    gpio_configuration_output();
    __set_PRIMASK(1);//关总中断
    dma_configuration_send_shoot();
    timer_configuration_send_dshoot();
    __set_PRIMASK(0);//开总中断
}

extern uint16_t dshoot_dma_buffer[140];//32
uint16_t data_handle[140] = {0};
uint8_t handl_data_flag = 0;
extern uint8_t dma_status;
extern uint8_t start_value_flag;
//extern void receive_dshoot_data_dispose(uint8_t *channel, uint16_t * data);
void DMA_Channel3_4_IRQHandler(void)
{
	if(SET == dma_flag_get(DMA_CH3, DMA_FLAG_FTF)) 
	{
		timer_disable(TIMER2);
		dma_channel_disable(DMA_CH3);
		dma_flag_clear(DMA_CH3, DMA_FLAG_FTF);
        if(dma_status == 0){
            //接收设置
            __set_PRIMASK(1);//关中断
            gpio_configuration_input();
			dma_configuration_receive_dshoot();
            timer_configuration_receive_dshoot();
            __set_PRIMASK(0);//开中断
			dma_status = 1;
        } else if (dma_status == 1) {
            memcpy(data_handle, dshoot_dma_buffer, sizeof(dshoot_dma_buffer));
			//handl_data_flag = 1;
            receive_dshoot_data_dispose(dshoot_pin,data_handle);
            //data_parse();
        }
    }
}

void data_parse(void)
{
    uint8_t start_flag = 0;
    uint32_t edge[140] = {0};
    uint8_t edge_cnt = 0;
    uint8_t signal_cnt = 0;
    uint8_t hight_low_status = 0;
    uint8_t old_hight_low_status = 0;
    uint8_t j = 0;

    for(int i=0; i<140; i++)
    {
        if (!(data_handle[i]>>4 & 1))
        {
            hight_low_status = 1;//低电平
            start_flag = 1;
        }
        else
        {
            hight_low_status = 0;//高电平
        }
        if (start_flag)
        {
            signal_cnt++;//信号次数
        } 
        if (old_hight_low_status != hight_low_status)
        {
            edge[j++] = (signal_cnt-1)*2;//信号计算从0开始
            edge_cnt++;
        }
        
        old_hight_low_status = hight_low_status;
    }

    if (start_value_flag)
    decodeTelemetryPacket(edge,edge_cnt);
}



void pwm_control_protocol(uint8_t* datas)
{
//    ACT_cmd_turn_s *cmd_turn = (ACT_cmd_turn_s *)datas;

//    for (uint8_t i = 0; i < PWM_MAX; i++) {
//        if (cmd_turn->pwm[i] <= PWM_CNT_MAX) {
//            //lowlevel_pwm_control((enum PWM_INDEX_NUMBER)i, cmd_turn->pwm[i]);
//					  pwmWriteDigital(dshot_cmd_ch[i], (cmd_turn->pwm[i]-1000)*2);
//        }
//    }
	   volatile uint16_t pwm[4] = {0};
		 for (uint8_t i = 0,j = 0; i < PWM_MAX; i++ ,j++) {
			    pwm[i]=  (datas[j+1]<<8|datas[j]); 
			    pwmWriteDigital(dshot_cmd_ch[i], (pwm[i]-1000)*2);
			 		j++;
		}
}


void test_dshot600_1ms(void)
{

    delay_1ms(1);
    //if (handl_data_flag)
    //receive_dshoot_data_dispose(dshoot_pin,data_handle);
    //handl_data_flag = 0;
    run_cycle_1ms();
    
    //send_dshot600_ok();
}
