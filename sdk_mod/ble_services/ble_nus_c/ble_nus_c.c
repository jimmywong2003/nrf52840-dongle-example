/**
 * Copyright (c) 2012 - 2018, Nordic Semiconductor ASA
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
#if NRF_MODULE_ENABLED(BLE_NUS_C)
#include <stdlib.h>

#include "ble.h"
#include "ble_nus_c.h"
#include "ble_gattc.h"
#include "ble_srv_common.h"
#include "app_error.h"

#define NRF_LOG_MODULE_NAME ble_nus_c
#include "nrf_log.h"
NRF_LOG_MODULE_REGISTER();

#define TX_BUFFER_MASK         0x07                  /**< TX Buffer mask, must be a mask of continuous zeroes, followed by continuous sequence of ones: 000...111. */
#define TX_BUFFER_SIZE         (TX_BUFFER_MASK + 1)  /**< Size of send buffer, which is 1 higher than the mask. */

#define WRITE_MESSAGE_LENGTH   BLE_CCCD_VALUE_LEN    /**< Length of the write message for CCCD. */
#define WRITE_MESSAGE_LENGTH   BLE_CCCD_VALUE_LEN    /**< Length of the write message for CCCD. */

typedef enum
{
        READ_REQ, /**< Type identifying that this tx_message is a read request. */
        WRITE_REQ /**< Type identifying that this tx_message is a write request. */
} tx_request_t;

/**@brief Structure for writing a message to the peer, i.e. CCCD.
 */
typedef struct
{
        uint8_t gattc_value[WRITE_MESSAGE_LENGTH];               /**< The message to write. */
        ble_gattc_write_params_t gattc_params;                   /**< GATTC parameters for this message. */
} write_params_t;

/**@brief Structure for holding data to be transmitted to the connected central.
 */
typedef struct
{
        uint16_t conn_handle;  /**< Connection handle to be used when transmitting this message. */
        tx_request_t type;     /**< Type of this message, i.e. read or write message. */
        union
        {
                uint16_t read_handle; /**< Read request message. */
                write_params_t write_req; /**< Write request message. */
        } req;
} tx_message_t;

static tx_message_t m_tx_buffer[TX_BUFFER_SIZE];  /**< Transmit buffer for messages to be transmitted to the central. */
static uint32_t m_tx_insert_index = 0;            /**< Current index in the transmit buffer where the next message should be inserted. */
static uint32_t m_tx_index = 0;                   /**< Current index in the transmit buffer from where the next message to be transmitted resides. */

/**@brief Function for passing any pending request from the buffer to the stack.
 */
static void tx_buffer_process(void)
{
        if (m_tx_index != m_tx_insert_index)
        {
                uint32_t err_code;

                if (m_tx_buffer[m_tx_index].type == READ_REQ)
                {
                        err_code = sd_ble_gattc_read(m_tx_buffer[m_tx_index].conn_handle,
                                                     m_tx_buffer[m_tx_index].req.read_handle,
                                                     0);
                }
                else
                {
                        err_code = sd_ble_gattc_write(m_tx_buffer[m_tx_index].conn_handle,
                                                      &m_tx_buffer[m_tx_index].req.write_req.gattc_params);
                }
                if (err_code == NRF_SUCCESS)
                {
                        NRF_LOG_DEBUG("SD Read/Write API returns Success..");
                        m_tx_index++;
                        m_tx_index &= TX_BUFFER_MASK;
                }
                else
                {
                        NRF_LOG_DEBUG("SD Read/Write API returns error. This message sending will be "
                                      "attempted again..");
                }
        }
}


void ble_nus_c_on_db_disc_evt(ble_nus_c_t * p_ble_nus_c, ble_db_discovery_evt_t * p_evt)
{
        ble_nus_c_evt_t nus_c_evt;
        memset(&nus_c_evt,0,sizeof(ble_nus_c_evt_t));

        ble_gatt_db_char_t * p_chars = p_evt->params.discovered_db.charateristics;

        // Check if the NUS was discovered.
        if (    (p_evt->evt_type == BLE_DB_DISCOVERY_COMPLETE)
                &&  (p_evt->params.discovered_db.srv_uuid.uuid == BLE_UUID_NUS_SERVICE)
                &&  (p_evt->params.discovered_db.srv_uuid.type == p_ble_nus_c->uuid_type))
        {
                for (uint32_t i = 0; i < p_evt->params.discovered_db.char_count; i++)
                {
                        switch (p_chars[i].characteristic.uuid.uuid)
                        {
                        case BLE_UUID_NUS_RX_CHARACTERISTIC:
                                nus_c_evt.handles.nus_rx_handle = p_chars[i].characteristic.handle_value;
                                break;

                        case BLE_UUID_NUS_TX_CHARACTERISTIC:
                                nus_c_evt.handles.nus_tx_handle = p_chars[i].characteristic.handle_value;
                                nus_c_evt.handles.nus_tx_cccd_handle = p_chars[i].cccd_handle;
                                break;

                        default:
                                break;
                        }
                }
                NRF_LOG_DEBUG("NUS Service discovered at peer.");
                if (p_ble_nus_c->evt_handler != NULL)
                {
                        if (p_ble_nus_c->conn_handle != BLE_CONN_HANDLE_INVALID)
                        {
                                nus_c_evt.conn_handle = p_evt->conn_handle;
                                nus_c_evt.evt_type    = BLE_NUS_C_EVT_DISCOVERY_COMPLETE;
                                p_ble_nus_c->evt_handler(p_ble_nus_c, &nus_c_evt);
                        }
                }
        }
}

/**@brief Function for handling write response events.
 *
 * @param[in] p_ble_lbs_c Pointer to the Led Button Client structure.
 * @param[in] p_ble_evt   Pointer to the BLE event received.
 */
static void on_write_rsp(ble_nus_c_t * p_ble_nus_c, ble_evt_t const * p_ble_evt)
{
        // Check if the event if on the link for this instance
        if (p_ble_nus_c->conn_handle != p_ble_evt->evt.gattc_evt.conn_handle)
        {
                return;
        }
        // Check if there is any message to be sent across to the peer and send it.
        tx_buffer_process();
}



/**@brief     Function for handling Handle Value Notification received from the SoftDevice.
 *
 * @details   This function will uses the Handle Value Notification received from the SoftDevice
 *            and checks if it is a notification of the NUS TX characteristic from the peer. If
 *            it is, this function will decode the data and send it to the
 *            application.
 *
 * @param[in] p_ble_nus_c Pointer to the NUS Client structure.
 * @param[in] p_ble_evt   Pointer to the BLE event received.
 */
static void on_hvx(ble_nus_c_t * p_ble_nus_c, ble_evt_t const * p_ble_evt)
{
        // Check if the event is on the link for this instance
        if (p_ble_nus_c->conn_handle != p_ble_evt->evt.gattc_evt.conn_handle)
        {
                return;
        }
        
        if ((p_ble_evt->evt.gattc_evt.params.hvx.handle == p_ble_nus_c->handles.nus_tx_handle) \
            && (p_ble_nus_c->evt_handler != NULL))
        {
                //if (p_ble_evt->evt.gattc_evt.params.hvx.len == 1)
                {
                        ble_nus_c_evt_t ble_nus_c_evt;

                        ble_nus_c_evt.evt_type                   = BLE_NUS_C_EVT_NUS_TX_EVT;
                        ble_nus_c_evt.conn_handle                = p_ble_nus_c->conn_handle;
                        ble_nus_c_evt.p_data   = (uint8_t *)p_ble_evt->evt.gattc_evt.params.hvx.data;
                        ble_nus_c_evt.data_len = p_ble_evt->evt.gattc_evt.params.hvx.len;

                        p_ble_nus_c->evt_handler(p_ble_nus_c, &ble_nus_c_evt);
                        NRF_LOG_DEBUG("Client sending data.");
                }
        }
}

uint32_t ble_nus_c_init(ble_nus_c_t * p_ble_nus_c, ble_nus_c_init_t * p_ble_nus_c_init)
{
        uint32_t err_code;
        ble_uuid_t uart_uuid;
        ble_uuid128_t nus_base_uuid = NUS_BASE_UUID;

        VERIFY_PARAM_NOT_NULL(p_ble_nus_c);
        VERIFY_PARAM_NOT_NULL(p_ble_nus_c_init);

        p_ble_nus_c->conn_handle           = BLE_CONN_HANDLE_INVALID;
        p_ble_nus_c->handles.nus_tx_handle = BLE_GATT_HANDLE_INVALID;
        p_ble_nus_c->handles.nus_rx_handle = BLE_GATT_HANDLE_INVALID;
        p_ble_nus_c->evt_handler           = p_ble_nus_c_init->evt_handler;

        err_code = sd_ble_uuid_vs_add(&nus_base_uuid, &p_ble_nus_c->uuid_type);
        VERIFY_SUCCESS(err_code);

        uart_uuid.type = p_ble_nus_c->uuid_type;
        uart_uuid.uuid = BLE_UUID_NUS_SERVICE;

        return ble_db_discovery_evt_register(&uart_uuid);
}

void ble_nus_c_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context)
{
        ble_nus_c_t * p_ble_nus_c = (ble_nus_c_t *)p_context;

        if ((p_ble_nus_c == NULL) || (p_ble_evt == NULL))
        {
                return;
        }

        if ( (p_ble_nus_c->conn_handle != BLE_CONN_HANDLE_INVALID)
             &&(p_ble_nus_c->conn_handle != p_ble_evt->evt.gap_evt.conn_handle)
             )
        {
                return;
        }

        switch (p_ble_evt->header.evt_id)
        {
        case BLE_GATTC_EVT_HVX:
                on_hvx(p_ble_nus_c, p_ble_evt);
                break;

        case BLE_GATTC_EVT_WRITE_RSP:
                on_write_rsp(p_ble_nus_c, p_ble_evt);
                break;

        case BLE_GAP_EVT_DISCONNECTED:
                if (p_ble_evt->evt.gap_evt.conn_handle == p_ble_nus_c->conn_handle
                    && p_ble_nus_c->evt_handler != NULL)
                {
                        ble_nus_c_evt_t nus_c_evt;

                        nus_c_evt.evt_type = BLE_NUS_C_EVT_DISCONNECTED;
                        nus_c_evt.conn_handle = p_ble_evt->evt.gap_evt.conn_handle;

                        p_ble_nus_c->conn_handle = BLE_CONN_HANDLE_INVALID;
                        p_ble_nus_c->evt_handler(p_ble_nus_c, &nus_c_evt);
                }
                break;

        default:
                // No implementation needed.
                break;
        }
}

/**@brief Function for creating a message for writing to the CCCD. */
static uint32_t cccd_configure(uint16_t conn_handle, uint16_t cccd_handle, bool enable)
{
        NRF_LOG_DEBUG("Configuring CCCD. CCCD Handle = %d, Connection Handle = %d",
                      cccd_handle,conn_handle);

        tx_message_t * p_msg;
        uint16_t cccd_val = enable ? BLE_GATT_HVX_NOTIFICATION : 0;

        p_msg              = &m_tx_buffer[m_tx_insert_index++];
        m_tx_insert_index &= TX_BUFFER_MASK;

        p_msg->req.write_req.gattc_params.handle   = cccd_handle;
        p_msg->req.write_req.gattc_params.len      = WRITE_MESSAGE_LENGTH;
        p_msg->req.write_req.gattc_params.p_value  = p_msg->req.write_req.gattc_value;
        p_msg->req.write_req.gattc_params.offset   = 0;
        p_msg->req.write_req.gattc_params.write_op = BLE_GATT_OP_WRITE_REQ;
        p_msg->req.write_req.gattc_value[0]        = LSB_16(cccd_val);
        p_msg->req.write_req.gattc_value[1]        = MSB_16(cccd_val);
        p_msg->conn_handle                         = conn_handle;
        p_msg->type                                = WRITE_REQ;

        tx_buffer_process();
        return NRF_SUCCESS;
}


uint32_t ble_nus_c_tx_notif_enable(ble_nus_c_t * p_ble_nus_c)
{
        VERIFY_PARAM_NOT_NULL(p_ble_nus_c);

        if ( (p_ble_nus_c->conn_handle == BLE_CONN_HANDLE_INVALID)
             ||(p_ble_nus_c->handles.nus_tx_cccd_handle == BLE_GATT_HANDLE_INVALID)
             )
        {
                return NRF_ERROR_INVALID_STATE;
        }
        return cccd_configure(p_ble_nus_c->conn_handle,p_ble_nus_c->handles.nus_tx_cccd_handle, true);
}


uint32_t ble_nus_c_string_send(ble_nus_c_t * p_ble_nus_c, uint8_t * p_string, uint16_t length)
{
        VERIFY_PARAM_NOT_NULL(p_ble_nus_c);

        if (length > BLE_NUS_MAX_DATA_LEN)
        {
                NRF_LOG_WARNING("Content too long.");
                return NRF_ERROR_INVALID_PARAM;
        }
        if (p_ble_nus_c->conn_handle == BLE_CONN_HANDLE_INVALID)
        {
                NRF_LOG_WARNING("Connection handle invalid.");
                return NRF_ERROR_INVALID_STATE;
        }

        ble_gattc_write_params_t const write_params =
        {
                .write_op = BLE_GATT_OP_WRITE_CMD,
                .flags    = BLE_GATT_EXEC_WRITE_FLAG_PREPARED_WRITE,
                .handle   = p_ble_nus_c->handles.nus_rx_handle,
                .offset   = 0,
                .len      = length,
                .p_value  = p_string
        };

        return sd_ble_gattc_write(p_ble_nus_c->conn_handle, &write_params);
}


uint32_t ble_nus_c_handles_assign(ble_nus_c_t               * p_ble_nus,
                                  uint16_t conn_handle,
                                  ble_nus_c_handles_t const * p_peer_handles)
{
        VERIFY_PARAM_NOT_NULL(p_ble_nus);

        p_ble_nus->conn_handle = conn_handle;
        if (p_peer_handles != NULL)
        {
                p_ble_nus->handles.nus_tx_cccd_handle = p_peer_handles->nus_tx_cccd_handle;
                p_ble_nus->handles.nus_tx_handle      = p_peer_handles->nus_tx_handle;
                p_ble_nus->handles.nus_rx_handle      = p_peer_handles->nus_rx_handle;
        }
        return NRF_SUCCESS;
}
#endif // NRF_MODULE_ENABLED(BLE_NUS_C)
