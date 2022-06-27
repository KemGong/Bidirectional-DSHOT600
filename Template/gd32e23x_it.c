/*!
    \file    gd32e23x_it.c
    \brief   interrupt service routines
    
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

#include "gd32e23x_it.h"
#include "main.h"
#include "systick.h"

uint32_t ic1value = 0, ic2value = 0;
__IO uint16_t dutycycle = 0;
__IO uint16_t frequency = 0;
extern uint8_t dma_status;
extern uint16_t buffer[140];
uint16_t data_handle[140] = {0};
uint8_t receive_dshoot_data_flag = 0;

/*!
    \brief      this function handles NMI exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void NMI_Handler(void)
{
}

/*!
    \brief      this function handles HardFault exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void HardFault_Handler(void)
{
    /* if Hard Fault exception occurs, go to infinite loop */
    while(1){
    }
}

/*!
    \brief      this function handles SVC exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void SVC_Handler(void)
{
}

/*!
    \brief      this function handles PendSV exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void PendSV_Handler(void)
{
}

/*!
    \brief      this function handles SysTick exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void SysTick_Handler(void)
{
    delay_decrement();
}

uint32_t run_cnt = 0;
uint32_t success_cnt = 0;
uint32_t fail_cnt = 0;
uint32_t fail_cnt1,fail_cnt2,fail_cnt3;
extern uint32_t cnt;
extern uint8_t start_flag;
float percent;
extern uint32_t start_cnt;
uint32_t decodeTelemetryPacket(uint32_t buffer[], uint32_t count)
{
    volatile uint32_t value = 0;
    uint32_t oldValue = buffer[0];
    int bits = 0;
    int len;
    
    run_cnt ++;
    
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
    if (bits != 21 && start_flag==1) {
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
        if (start_flag) {
        fail_cnt3++;}
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
		//volatile float erpm = ret * 100;
		//volatile float rpm = ret * 100 * 2 / 7;
    success_cnt++;
    percent = ((fail_cnt2+fail_cnt3)*100)/(run_cnt);
    return ret;
}

void DMA_Channel3_4_IRQHandler(void)
{
	if(SET == dma_flag_get(DMA_CH3, DMA_FLAG_FTF)) 
	{
		uint16_t get_num = dma_transfer_number_get(DMA_CH3) + 1;
        if(dma_status == 0){
            timer_disable(TIMER2);
            dma_channel_disable(DMA_CH3);
            dma_flag_clear(DMA_CH3, DMA_FLAG_FTF);
            //接收设置
            dma_status = 1;
            gpio_configuration_input();
            timer_configuration_receive_dshoot();
            dma_configuration_receive_dshoot();
        } else if (dma_status == 1) {
            timer_disable(TIMER2);
            dma_channel_disable(DMA_CH3);
            dma_flag_clear(DMA_CH3, DMA_FLAG_FTF);
            //memcpy(data_handle, buffer, sizeof(data_handle));
            //this
            receive_dshoot_data_flag = 1;
            uint8_t start_flag = 0;
            uint32_t edge[140] = {0};
            uint8_t edge_cnt = 0;
            uint8_t signal_cnt = 0;
            uint8_t hight_low_status = 0;
            uint8_t old_hight_low_status = 0;
            uint8_t j = 0;
            if(receive_dshoot_data_flag)
            {
                receive_dshoot_data_flag = 0;
                for(int i=0; i<140; i++)
                {
                    if (!(buffer[i]>>4 & 1))
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
            }
            decodeTelemetryPacket(edge,edge_cnt);
        }
        
		/*timer_disable(TIMER2);
		dma_channel_disable(DMA_CH3);
		dma_flag_clear(DMA_CH3, DMA_FLAG_FTF);
        gpio_configuration_input();*/

	}
	if(SET == dma_interrupt_flag_get(DMA_CH3, DMA_INT_FLAG_FTF)){
		dma_interrupt_flag_clear(DMA_CH3, DMA_INT_FLAG_FTF);
	}
}

#if 0
/**
  * @brief  This function handles TIMER2 interrupt request.
  * @param  None
  * @retval None
  */
void TIMER2_IRQHandler(void)
{
    
    if(SET == timer_interrupt_flag_get(TIMER2, TIMER_INT_FLAG_CH0)){
        /* clear channel 0 interrupt bit */
        timer_interrupt_flag_clear(TIMER2, TIMER_INT_FLAG_CH0);
        /* read channel 0 capture value */
        ic1value = timer_channel_capture_value_register_read(TIMER2, TIMER_CH_0) + 1;

        if(0 != ic1value){
            /* read channel 1 capture value */
            ic2value = timer_channel_capture_value_register_read(TIMER2, TIMER_CH_1) + 1;

            /* calculate the duty cycle value */
            dutycycle = (ic2value * 100.0) / ic1value;
            /* calculate the frequency value */
            frequency = 1000000U / ic1value;
        }else{
            /* clear calculation value */
            dutycycle = 0;
            frequency = 0;
        }
    }
}
#else
/**
  * @brief  This function handles TIMER2 interrupt request.
  * @param  None
  * @retval None
  */
void TIMER2_IRQHandler(void)
{
    
    if(SET == timer_interrupt_flag_get(TIMER2, TIMER_INT_FLAG_CH0)){
        /* clear channel 0 interrupt bit */
        timer_interrupt_flag_clear(TIMER2, TIMER_INT_FLAG_CH0);
        /* read channel 0 capture value */
        if(RESET == gpio_input_bit_get(GPIOA, GPIO_PIN_6))
        {
            ic2value = timer_channel_capture_value_register_read(TIMER2, TIMER_CH_0) + 1;
        }
        else
        {
            ic1value = timer_channel_capture_value_register_read(TIMER2, TIMER_CH_0) + 1;
        }
    }
}

#endif
