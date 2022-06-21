/*!
    \file    main.c
    \brief   led spark with systick, USART print and key example
    
    \version 2019-02-19, V1.0.0, firmware for GD32E23x
    \version 2020-12-12, V1.1.0, firmware for GD32E23x
*/

/*
    Copyright (c) 2020, GigaDevice Semiconductor Inc.

    Redistribution and use in source and binary forms, with or without modification, 
are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice, this 
       list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright notice, 
       this list of conditions and the following disclaimer in the documentation 
       and/or other materials provided with the distribution.
    3. Neither the name of the copyright holder nor the names of its contributors 
       may be used to endorse or promote products derived from this software without 
       specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. 
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, 
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT 
NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR 
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, 
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY 
OF SUCH DAMAGE.
*/

#include "gd32e23x.h"
#include "systick.h"
#include <stdio.h>
#include "main.h"
#include "gd32e230c_eval.h"
//#include "cmsis_armclang.h"



#define TIMER2_CH0_ADDR     (0x40000400+0x34)

#define GPIOA_ISTAT_ADDR 		(0X48000010)
#define GPIOB_ISTAT_ADDR		(0X48000410)
#define GPIOC_ISTAT_ADDR		(0X48000810)

#define GPIOA_BOP_ADDR          (0x48000018)
#define GPIOB_BOP_ADDR          (0x48000418)
#define GPIOC_BOP_ADDR          (0x48000818)

uint16_t buffer[140] = {0};//32

#define HIGHT_VALUE  0x01U<<(6+16),0x01U<<(6+16),0x01U<<(6)
#define LOW_VALUE    0x01U<<(6+16),0x01U<<(6),0x01U<<(6)
#define PULL_HIGH    0x01U<<(6),0x01U<<(6),0x01U<<(6)

uint8_t dma_status = 0;

uint32_t send_buffer[54] = {0};
uint32_t send_buffer_speed[54] = {PULL_HIGH,LOW_VALUE,LOW_VALUE,LOW_VALUE,HIGHT_VALUE,HIGHT_VALUE,//3+3*16+3
                            LOW_VALUE,LOW_VALUE,HIGHT_VALUE,LOW_VALUE,LOW_VALUE,LOW_VALUE,LOW_VALUE,LOW_VALUE,
                            HIGHT_VALUE,HIGHT_VALUE,HIGHT_VALUE,PULL_HIGH};

uint32_t send_buffer_0[54] = {PULL_HIGH,LOW_VALUE,LOW_VALUE,LOW_VALUE,LOW_VALUE,LOW_VALUE,//3+3*16+3
                            LOW_VALUE,LOW_VALUE,LOW_VALUE,LOW_VALUE,LOW_VALUE,LOW_VALUE,LOW_VALUE,HIGHT_VALUE,
                            HIGHT_VALUE,HIGHT_VALUE,HIGHT_VALUE,PULL_HIGH};

extern uint16_t data_handle[140];
extern uint8_t receive_dshoot_data_flag;

/* configure the GPIO ports */
void gpio_config(void);
/* configure the TIMER peripheral */
void timer_config(void);
/* configure the TIMER interrupt */
void nvic_config(void);
/* retarget the C library printf function to the USART */
int fputc(int ch, FILE *f);

/* retarget the C library printf function to the USART */
int fputc(int ch, FILE *f)
{
    usart_data_transmit(EVAL_COM, (uint8_t)ch);
    while(RESET == usart_flag_get(EVAL_COM, USART_FLAG_TBE));
    return ch;
}

/**
    \brief      configure the GPIO ports
    \param[in]  none
    \param[out] none
    \retval     none
  */
void gpio_configuration_output(void)
{
#if 0
    rcu_periph_clock_enable(RCU_GPIOA);

    /*configure PA6(TIMER2 CH0) as alternate function*/
    gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_6);
		//gpio_mode_set(GPIOA, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO_PIN_6);
    gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ,GPIO_PIN_6);
    gpio_af_set(GPIOA, GPIO_AF_1, GPIO_PIN_6);
#endif
    
    rcu_periph_clock_enable(RCU_GPIOA);
    //PA6
    gpio_mode_set(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_PIN_6);
    gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_6);
    gpio_bit_set(GPIOA, GPIO_PIN_6);
}

void gpio_configuration_input(void)
{
#if 0
    rcu_periph_clock_enable(RCU_GPIOA);

    /*configure PA6(TIMER2 CH0) as alternate function*/
    gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_6);
		//gpio_mode_set(GPIOA, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO_PIN_6);
    gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ,GPIO_PIN_6);
    gpio_af_set(GPIOA, GPIO_AF_1, GPIO_PIN_6);
#endif
    gpio_mode_set(GPIOA, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO_PIN_6);
}


/**
    \brief      configure the nested vectored interrupt controller
    \param[in]  none
    \param[out] none
    \retval     none
  */
void nvic_configuration(void)
{
    nvic_irq_enable(DMA_Channel3_4_IRQn, 0);
}

/**
    \brief      configure the TIMER peripheral
    \param[in]  none
    \param[out] none
    \retval     none
  */
void timer_configuration_receive_dshoot(void)
{
 /* TIMER2 configuration: PWM input mode ------------------------
     the external signal is connected to TIMER2 CH0 pin(PA6)
     the rising edge is used as active edge
     the TIMER2 CH0CV is used to compute the frequency value 
     the TIMER2 CH1CV is used to compute the duty cycle value
  ------------------------------------------------------------ */
    //timer_ic_parameter_struct timer_icinitpara;
    timer_parameter_struct timer_initpara;
    /* enable the TIMER clock */
    rcu_periph_clock_enable(RCU_TIMER2);

    /* deinit a TIMER */
    timer_deinit(TIMER2);
    /* initialize TIMER init parameter struct */
    timer_struct_para_init(&timer_initpara);
    /* TIMER2 configuration */
    timer_initpara.prescaler         = 0;//35 72
    timer_initpara.alignedmode       = TIMER_COUNTER_EDGE;
    timer_initpara.counterdirection  = TIMER_COUNTER_UP;
    timer_initpara.period            = (31);//32-1
    timer_initpara.clockdivision     = TIMER_CKDIV_DIV1;
    timer_init(TIMER2, &timer_initpara);

    /* TIMER2 configuration */
    /* initialize TIMER channel input parameter struct */
    //timer_channel_input_struct_para_init(&timer_icinitpara);
    /* TIMER2 CH0 PWM input capture configuration */
    /*timer_icinitpara.icpolarity  = TIMER_IC_POLARITY_BOTH_EDGE;
    timer_icinitpara.icselection = TIMER_IC_SELECTION_DIRECTTI;
    timer_icinitpara.icprescaler = TIMER_IC_PSC_DIV1;
    timer_icinitpara.icfilter    = 0x0;
    timer_input_pwm_capture_config(TIMER2, TIMER_CH_0, &timer_icinitpara);*/
#if 0
    /* slave mode selection: TIMER2 */
    timer_input_trigger_source_select(TIMER2, TIMER_SMCFG_TRGSEL_CI0FE0);
    /* select TIMER slave mode: restart mode */
    timer_slave_mode_select(TIMER2, TIMER_SLAVE_MODE_RESTART);

    /* select the master slave mode */
    timer_master_slave_mode_config(TIMER2, TIMER_MASTER_SLAVE_MODE_ENABLE);
 #endif   
    // /* clear channel 0 interrupt bit */
    // timer_interrupt_flag_clear(TIMER2, TIMER_INT_FLAG_CH0);
    // /* channel 0 interrupt enable */
    // timer_interrupt_enable(TIMER2, TIMER_INT_CH0);

    //timer_channel_dma_request_source_select(TIMER2, TIMER_DMAREQUEST_CHANNELEVENT);


    /* enable the TIMER DMA */
    timer_dma_enable(TIMER2, TIMER_DMA_CH0D);

    /* auto-reload preload enable */
    //timer_auto_reload_shadow_enable(TIMER2);

    /* TIMER2 counter enable */
    timer_enable(TIMER2);
}



void dma_configuration_receive_dshoot(void)
{
    dma_parameter_struct dma_init_struct;

    /* enable DMA clock */
    rcu_periph_clock_enable(RCU_DMA);

    /* initialize DMA channel4 */
    dma_deinit(DMA_CH3);
    /* DMA channel4 initialize */
    dma_init_struct.periph_addr  = (uint32_t)GPIOA_ISTAT_ADDR;
    dma_init_struct.periph_inc   = DMA_PERIPH_INCREASE_DISABLE;
    dma_init_struct.memory_addr  = (uint32_t)buffer;
    dma_init_struct.memory_inc   = DMA_MEMORY_INCREASE_ENABLE;
    dma_init_struct.periph_width = DMA_PERIPHERAL_WIDTH_16BIT;
    dma_init_struct.memory_width = DMA_MEMORY_WIDTH_16BIT;
    dma_init_struct.direction    = DMA_PERIPHERAL_TO_MEMORY;
    dma_init_struct.number       = 140;
    dma_init_struct.priority     = DMA_PRIORITY_ULTRA_HIGH;
    dma_init( DMA_CH3, &dma_init_struct);
    
    /* enable DMA circulation mode */
    dma_circulation_enable(DMA_CH3);
    /* enable DMA channel4 */
    dma_channel_enable(DMA_CH3);
		
	dma_interrupt_enable(DMA_CH3, DMA_INT_FTF);
}

/**
    \brief      configure the TIMER peripheral
    \param[in]  none
    \param[out] none
    \retval     none
  */
void timer_configuration_send_dshoot(void)
{
 /* TIMER2 configuration: PWM input mode ------------------------
     the external signal is connected to TIMER2 CH0 pin(PA6)
     the rising edge is used as active edge
     the TIMER2 CH0CV is used to compute the frequency value 
     the TIMER2 CH1CV is used to compute the duty cycle value
  ------------------------------------------------------------ */
    timer_ic_parameter_struct timer_icinitpara;
    timer_parameter_struct timer_initpara;
    /* enable the TIMER clock */
    rcu_periph_clock_enable(RCU_TIMER2);

    /* deinit a TIMER */
    timer_deinit(TIMER2);
    /* initialize TIMER init parameter struct */
    timer_struct_para_init(&timer_initpara);
    /* TIMER2 configuration */
    timer_initpara.prescaler         = 0;//35 72
    timer_initpara.alignedmode       = TIMER_COUNTER_EDGE;
    timer_initpara.counterdirection  = TIMER_COUNTER_UP;
    timer_initpara.period            = (37);//一个中断的时间为528ns  38-1   new// 40-1   557
    timer_initpara.clockdivision     = TIMER_CKDIV_DIV1;
    timer_init(TIMER2, &timer_initpara);

    /* TIMER2 configuration */
    /* initialize TIMER channel input parameter struct */
    timer_channel_input_struct_para_init(&timer_icinitpara);
    /* TIMER2 CH0 PWM input capture configuration */
    /*timer_icinitpara.icpolarity  = TIMER_IC_POLARITY_BOTH_EDGE;
    timer_icinitpara.icselection = TIMER_IC_SELECTION_DIRECTTI;
    timer_icinitpara.icprescaler = TIMER_IC_PSC_DIV1;
    timer_icinitpara.icfilter    = 0x0;
    timer_input_pwm_capture_config(TIMER2, TIMER_CH_0, &timer_icinitpara);*/
#if 0
    /* slave mode selection: TIMER2 */
    timer_input_trigger_source_select(TIMER2, TIMER_SMCFG_TRGSEL_CI0FE0);
    /* select TIMER slave mode: restart mode */
    timer_slave_mode_select(TIMER2, TIMER_SLAVE_MODE_RESTART);

    /* select the master slave mode */
    timer_master_slave_mode_config(TIMER2, TIMER_MASTER_SLAVE_MODE_ENABLE);
 #endif   
    // /* clear channel 0 interrupt bit */
    // timer_interrupt_flag_clear(TIMER2, TIMER_INT_FLAG_CH0);
    // /* channel 0 interrupt enable */
    // timer_interrupt_enable(TIMER2, TIMER_INT_CH0);

    timer_channel_dma_request_source_select(TIMER2, TIMER_DMAREQUEST_CHANNELEVENT);


    /* enable the TIMER DMA */
    timer_dma_enable(TIMER2, TIMER_DMA_CH0D);

    /* auto-reload preload enable */
    timer_auto_reload_shadow_enable(TIMER2);

    /* TIMER2 counter enable */
    timer_enable(TIMER2);
}



void dma_configuration_send_shoot(void)
{
    dma_parameter_struct dma_init_struct;

    /* enable DMA clock */
    rcu_periph_clock_enable(RCU_DMA);

    /* initialize DMA channel4 */
    dma_deinit(DMA_CH3);
    /* DMA channel4 initialize */
    dma_init_struct.periph_addr  = (uint32_t)GPIOA_BOP_ADDR;
    dma_init_struct.periph_inc   = DMA_PERIPH_INCREASE_DISABLE;
    dma_init_struct.memory_addr  = (uint32_t)send_buffer;
    dma_init_struct.memory_inc   = DMA_MEMORY_INCREASE_ENABLE;
    dma_init_struct.periph_width = DMA_PERIPHERAL_WIDTH_32BIT;
    dma_init_struct.memory_width = DMA_MEMORY_WIDTH_32BIT;
    dma_init_struct.direction    = DMA_MEMORY_TO_PERIPHERAL;
    dma_init_struct.number       = 16*3+3+3;
    dma_init_struct.priority     = DMA_PRIORITY_ULTRA_HIGH;
    dma_init( DMA_CH3, &dma_init_struct);
    
    /* enable DMA circulation mode */
    dma_circulation_enable(DMA_CH3);
    /* enable DMA channel4 */
    dma_channel_enable(DMA_CH3);
		
	dma_interrupt_enable(DMA_CH3, DMA_INT_FTF);
}


extern uint32_t ic1value, ic2value;

/*!
    \brief      main function
    \param[in]  none
    \param[out] none
    \retval     none
*/
uint32_t cnt = 0;
uint8_t start_flag = 0;
uint32_t start_cnt = 0;
extern uint32_t run_cnt;
int main(void)
{
    /* configure systick */
    systick_config();
    /* configure the GPIO ports */
    gpio_configuration_output();
    /* configure COM port */
    gd_eval_com_init(EVAL_COM);
    /* configure the TIMER interrupt */
    nvic_configuration(); 

    /** configure the timer2 ch0 dma*/
    //dma_configuration();

    /* configure the TIMER peripheral */
    //timer_configuration();
    
    /*timer_configuration_send_dshoot();
    dma_configuration_send_shoot();*/



    while (1){
        delay_1ms(1);
        cnt++;
        if (cnt<3000) {
            start_flag = 0;
            memcpy(send_buffer, send_buffer_0, sizeof(send_buffer));
            start_cnt = run_cnt;
        } else {
            start_flag = 1;
            memcpy(send_buffer, send_buffer_speed, sizeof(send_buffer));
        }
        dma_status = 0;
        gpio_configuration_output();
        //__set_FAULTMASK(1);
        dma_configuration_send_shoot();
        timer_configuration_send_dshoot();
        //__set_FAULTMASK(0);
        
        //printf("\r /**** TIMER2 PWM Input Capture Demo ****/\r\n");
        // printf(" the dutycycle is %d ", dutycycle);
        // printf(" the frequence is %d \r\n", frequency);
        //printf(" icvala %d, icvalb %d \r\n", ic1value, ic2value);
    }
}
