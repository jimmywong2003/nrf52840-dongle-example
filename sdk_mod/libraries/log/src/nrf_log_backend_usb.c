/**
 * Copyright (c) 2016 - 2019, Nordic Semiconductor ASA
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
#include "sdk_common.h"
#if NRF_MODULE_ENABLED(NRF_LOG) && NRF_MODULE_ENABLED(NRF_LOG_BACKEND_USB)
#include "nrf_log_backend_usb.h"
#include "nrf_log_backend_serial.h"
#include "nrf_log_internal.h"

#include "app_error.h"

#include "nrf_drv_usbd.h"
#include "app_usbd_core.h"
#include "app_usbd.h"
#include "app_usbd_string_desc.h"
#include "app_usbd_cdc_acm.h"

static bool m_log_usb_is_initialized = false;


#define READ_SIZE 1

static char m_rx_buffer[READ_SIZE];
static char m_tx_buffer[NRF_DRV_USBD_EPSIZE];
static bool m_send_flag = 0;

// nrf_drv_uart_t m_uart = NRF_DRV_UART_INSTANCE(0);

static uint8_t m_string_buff[NRF_LOG_BACKEND_UART_TEMP_BUFFER_SIZE];
static volatile bool m_xfer_done;
static bool m_async_mode;

static volatile bool m_usb_log_is_opened = false;

#define LOG_CDC_ACM_COMM_INTERFACE  2//0
#define LOG_CDC_ACM_COMM_EPIN       NRF_DRV_USBD_EPIN4//NRF_DRV_USBD_EPIN2

#define LOG_CDC_ACM_DATA_INTERFACE  3//1
#define LOG_CDC_ACM_DATA_EPIN       NRF_DRV_USBD_EPIN3//NRF_DRV_USBD_EPIN1
#define LOG_CDC_ACM_DATA_EPOUT      NRF_DRV_USBD_EPOUT2//NRF_DRV_USBD_EPOUT1



static void LOG_CDC_ACM_user_ev_handler(app_usbd_class_inst_t const * p_inst,
                                        app_usbd_cdc_acm_user_event_t event);

/**
 * @brief CDC_ACM class instance
 * */
APP_USBD_CDC_ACM_GLOBAL_DEF(m_log_app_cdc_acm,
                            LOG_CDC_ACM_user_ev_handler,
                            LOG_CDC_ACM_COMM_INTERFACE,
                            LOG_CDC_ACM_DATA_INTERFACE,
                            LOG_CDC_ACM_COMM_EPIN,
                            LOG_CDC_ACM_DATA_EPIN,
                            LOG_CDC_ACM_DATA_EPOUT,
                            APP_USBD_CDC_COMM_PROTOCOL_AT_V250
                            );


/**
 * @brief User event handler @ref app_usbd_cdc_acm_user_ev_handler_t (headphones)
 * */
static void LOG_CDC_ACM_user_ev_handler(app_usbd_class_inst_t const * p_inst,
                                        app_usbd_cdc_acm_user_event_t event)
{
        app_usbd_cdc_acm_t const * p_cdc_acm = app_usbd_cdc_acm_class_get(p_inst);

        switch (event)
        {
        case APP_USBD_CDC_ACM_USER_EVT_PORT_OPEN:
        {

                /*Setup first transfer*/
                ret_code_t ret = app_usbd_cdc_acm_read(&m_log_app_cdc_acm,
                                                       m_rx_buffer,
                                                       READ_SIZE);
                UNUSED_VARIABLE(ret);

                m_usb_log_is_opened = true;

                break;
        }
        case APP_USBD_CDC_ACM_USER_EVT_PORT_CLOSE:
                m_usb_log_is_opened = false;

                break;
        case APP_USBD_CDC_ACM_USER_EVT_TX_DONE:
                m_xfer_done = true;
                break;
        case APP_USBD_CDC_ACM_USER_EVT_RX_DONE:
        {
                ret_code_t ret;
//                NRF_LOG_INFO("Bytes waiting: %d", app_usbd_cdc_acm_bytes_stored(p_cdc_acm));
                do
                {
                        /*Get amount of data transfered*/
                        size_t size = app_usbd_cdc_acm_rx_size(p_cdc_acm);
//                        NRF_LOG_INFO("RX: size: %lu char: %c", size, m_rx_buffer[0]);

                        /* Fetch data until internal buffer is empty */
                        ret = app_usbd_cdc_acm_read(&m_log_app_cdc_acm,
                                                    m_rx_buffer,
                                                    READ_SIZE);
                } while (ret == NRF_SUCCESS);

                break;
        }
        default:
                break;
        }
}



//static void usb_evt_handler(nrf_drv_uart_event_t * p_event, void * p_context)
//{
//        m_xfer_done = true;
//}

static void log_usb_init(bool async_mode)
{
        app_usbd_class_inst_t const * log_class_cdc_acm =
                app_usbd_cdc_acm_class_inst_get(&m_log_app_cdc_acm);
        ret_code_t ret = app_usbd_class_append(log_class_cdc_acm);
        APP_ERROR_CHECK(ret);

        m_async_mode = async_mode;

//        app_usbd_class_inst_t const * class_cdc_acm_1 =
////        app_usbd_cdc_acm_class_inst_get(&m_app_cdc_acm_1);
////        ret = app_usbd_class_append(class_cdc_acm_1);
////        APP_ERROR_CHECK(ret);
}

void nrf_log_backend_usb_init(void)
{
        bool async_mode = NRF_LOG_DEFERRED ? true : false;
        log_usb_init(async_mode);
}

static void serial_tx(void const * p_context, char const * p_buffer, size_t len)
{
        uint8_t len8 = (uint8_t)(len & 0x000000FF);
        m_xfer_done = false;
        ret_code_t err_code;

        if (!m_usb_log_is_opened)
            return;

        size_t size = sprintf(m_tx_buffer, "%s", (char *)p_buffer);
        //                    size_t size = sprintf(m_tx_buffer, "Received LED OFF!\n");
        err_code = app_usbd_cdc_acm_write(&m_log_app_cdc_acm, m_tx_buffer, size);
   //     err_code = app_usbd_cdc_acm_write(&m_log_app_cdc_acm, (uint8_t*)p_buffer, len8);
        APP_ERROR_CHECK(err_code);
        while (m_async_mode && (m_xfer_done == false))
        {

        }
}

static void nrf_log_backend_usb_put(nrf_log_backend_t const * p_backend,
                                    nrf_log_entry_t * p_msg)
{
        nrf_log_backend_serial_put(p_backend, p_msg, m_string_buff,
                                   NRF_LOG_BACKEND_UART_TEMP_BUFFER_SIZE, serial_tx);
}

static void nrf_log_backend_usb_flush(nrf_log_backend_t const * p_backend)
{
        // ret_code_t err_code = app_usbd_uninit();
        // APP_ERROR_CHECK(err_code);

}

static void nrf_log_backend_usb_panic_set(nrf_log_backend_t const * p_backend)
{
//        nrf_drv_uart_uninit(&m_uart);
//
//        uart_init(false);
}

const nrf_log_backend_api_t nrf_log_backend_usb_api = {
        .put       = nrf_log_backend_usb_put,
        .flush     = nrf_log_backend_usb_flush,
        .panic_set = nrf_log_backend_usb_panic_set,
};
#endif //NRF_MODULE_ENABLED(NRF_LOG) && NRF_MODULE_ENABLED(NRF_LOG_BACKEND_USB)
