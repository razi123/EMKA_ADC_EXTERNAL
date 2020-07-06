/**
 * Copyright (c) 2015 - 2019, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
#include "nrf_drv_spi.h"
#include "app_util_platform.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "boards.h"
#include "app_error.h"
#include <string.h>
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#define SPI_INSTANCE  0 /**< SPI instance index. */
static const nrf_drv_spi_t spi = NRF_DRV_SPI_INSTANCE(SPI_INSTANCE);  /**< SPI instance. */
static volatile bool spi_xfer_done;  /**< Flag used to indicate that SPI instance completed the transfer. */

#define TEST_STRING "Nordic"
//static uint8_t       m_tx_buf[4] = {0,0,0,0x01};           /**< TX buffer. */
//static uint8_t       m_rx_buf[sizeof(m_tx_buf) + 1];    /**< RX buffer. */
//static const uint8_t m_length = sizeof(m_tx_buf);        /**< Transfer length. */

static uint8_t       m_tx_buf[4] = {0,0,0,0x01};           
static uint8_t       m_rx_buf[]; 
static const uint8_t m_length= sizeof(m_tx_buf); 
static uint8_t       temp[];

uint8_t volatile spi_flag = 0;
/**
 * @brief SPI user event handler.
 * @param event
 */
void spi_event_handler(nrf_drv_spi_evt_t const * p_event,
                       void *                    p_context)
{
    spi_xfer_done = true;
    NRF_LOG_INFO("Transfer completed.");
    spi_flag=1;
    if (m_rx_buf[0] != 0)
    {
        
        NRF_LOG_INFO(" Received:");
        NRF_LOG_HEXDUMP_INFO(m_rx_buf, strlen((const char *)m_rx_buf));
    }
}


void spi_mode_init()
{
 nrf_drv_spi_config_t spi_config = NRF_DRV_SPI_DEFAULT_CONFIG;
    spi_config.ss_pin   = SPI_SS_PIN;             // pin 31 
    spi_config.miso_pin = SPI_MISO_PIN;           // pin 31                                                 
    spi_config.mosi_pin = SPI_MOSI_PIN;           // pin 31 
    spi_config.sck_pin  = SPI_SCK_PIN;            // pin 26 
    APP_ERROR_CHECK(nrf_drv_spi_init(&spi, &spi_config, spi_event_handler, NULL));
  
}

/*
void spi_config(uint8_t *ptr_config, uint8_t m_len)
{
   memset(m_rx_buf, 0, 4);
   ret_code_t err_code;

   for(uint8_t i = 0; ptr_config[i]!='\0';i++)
   {
      temp[i] = *ptr_config[i];
   }
    

    err_code = nrf_drv_spi_transfer(&spi, temp, m_len, m_rx_buf, m_len);
    APP_ERROR_CHECK(err_code);
    while(!spi_flag);
    spi_flag=0;
   

}

*/
void spi_config()
{
   memset(m_rx_buf, 0, 4);
   ret_code_t err_code;

   
    

    err_code = nrf_drv_spi_transfer(&spi, m_tx_buf, m_length, m_rx_buf, m_length);
    APP_ERROR_CHECK(err_code);
    while(!spi_flag);
    spi_flag=0;
   

}

int main(void)
{
    uint32_t err_code;
    bsp_board_init(BSP_INIT_LEDS);

    APP_ERROR_CHECK(NRF_LOG_INIT(NULL));
    NRF_LOG_DEFAULT_BACKENDS_INIT();

    spi_mode_init();
    NRF_LOG_INFO("SPI example started.");
  
  
   
   // ADC_SPI configuration 



// ****************** Channel configuration ****************

    m_tx_buf[0] = 0x09;                                             // communication register setup to select the channel register
    m_rx_buf[sizeof(m_tx_buf) + 1];
    m_length = sizeof(m_tx_buf);

    //spi_config(&m_tx_buf, m_lenght);    
    spi_config();

// configure the channel_0 in channel register 

    m_tx_buf[0] = 0x80; 
    m_tx_buf[1] = 0x01;                                             // channel_0 enable with positive and negative input
    m_rx_buf[sizeof(m_tx_buf) + 1];
    m_length = sizeof(m_tx_buf);

   //spi_config(&m_tx_buf, m_lenght);    
    spi_config();   






// ****************** ADC SETUP *****************************


    m_tx_buf[0] = 0x21;                                             // communication register setup to select the filter register
    m_rx_buf[sizeof(m_tx_buf) + 1];
    m_length = sizeof(m_tx_buf);

    //spi_config(&m_tx_buf, m_lenght);    
    spi_config();                              

// configure the filter0 in channel register 

    m_tx_buf[0] = 0x01; 
    m_tx_buf[1] = 0x00;                                             // Sinc filter, Single cycle 
    m_tx_buf[2] = 0x00;
    m_rx_buf[sizeof(m_tx_buf) + 1];
    m_length = sizeof(m_tx_buf);

    //spi_config(&m_tx_buf, m_lenght);    
    spi_config(); 
     
  

// ****************** Diagnostic control **********************


    m_tx_buf[0] = 0x07;                                             // communication register setup to select the Error_registr
    m_rx_buf[sizeof(m_tx_buf) + 1];
    m_length = sizeof(m_tx_buf);

    //spi_config(&m_tx_buf, m_lenght);    
    spi_config();                               

// configure the filter0 in channel register 

    m_tx_buf[0] = 0x00; 
    m_tx_buf[1] = 0x00;                                             // SPI read-write_error_enable, spi_crc_err_enable  
    m_tx_buf[2] = 0x7E;
    m_rx_buf[sizeof(m_tx_buf) + 1];
    m_length = sizeof(m_tx_buf);

   //spi_config(&m_tx_buf, m_lenght);    
    spi_config();   
     
 
  
  // ****************** ADC Control ****************************


    m_tx_buf[0] = 0x01;                                             // communication register setup to select ADC control register
    m_rx_buf[sizeof(m_tx_buf) + 1];
    m_length = sizeof(m_tx_buf);

    //spi_config(&m_tx_buf, m_lenght);    
    spi_config();                              

// configure the filter0 in channel register 

    m_tx_buf[0] = 0x0E; 
    m_tx_buf[1] = 0x01;                                             // Continues read, data status, low power mode, continues conersion mode etc  
 
    m_rx_buf[sizeof(m_tx_buf) + 1];
    m_length = sizeof(m_tx_buf);

    //spi_config(&m_tx_buf, m_lenght);    
    spi_config();

    
 
       

    while (1)
    {
       while (!spi_xfer_done)
        {
            __WFE();
        }

        NRF_LOG_FLUSH();

        bsp_board_led_invert(BSP_BOARD_LED_0);
        nrf_delay_ms(200);
    
    }


}
