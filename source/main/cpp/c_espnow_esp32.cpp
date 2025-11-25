
#if defined(TARGET_ARDUINO) && defined(TARGET_ESP32)

#    include "rdno_core/c_debug.h"

#    include "Arduino.h"

#    include "esp_now.h"
#    include "esp_wifi.h"

#    include "freertos/FreeRTOS.h"
#    include "freertos/semphr.h"
#    include "freertos/queue.h"
#    include "freertos/task.h"

#    include "rdno_espnow/private/c_peer_list.h"
#    include "rdno_espnow/c_espnow.h"

// ----------------------------------------------------------------------------------------
// ----------------------------------------------------------------------------------------
namespace ncore
{
    namespace nqespnow
    {
        DEBUG_TAG(QESPNOW_TAG, "QESPNOW");

        typedef enum
        {
            COMMS_SEND_OK                   = 0,  /* Data was enqued for sending successfully */
            COMMS_SEND_PARAM_ERROR          = -1, /* Data was not sent due to parameter call error */
            COMMS_SEND_PAYLOAD_LENGTH_ERROR = -2, /* Data was not sent due to payload too long */
            COMMS_SEND_QUEUE_FULL_ERROR     = -3, /* Data was not sent due to queue full */
            COMMS_SEND_MSG_ENQUEUE_ERROR    = -4, /* Data was not sent due to message queue push error */
            COMMS_SEND_CONFIRM_ERROR        = -5, /* Data was not sent due to confirmation error (only for synchronous send) */
        } comms_send_error_t;

        static u8        ESPNOW_BROADCAST_ADDRESS[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
        static const u8  MIN_WIFI_CHANNEL           = 0;
        static const u8  MAX_WIFI_CHANNEL           = 14;
        static const u8  CURRENT_WIFI_CHANNEL       = 255;  // Initialize with current channel (check your router settings)
        static const u32 ESPNOW_MAX_MESSAGE_LENGTH  = 240;  // Maximum message length
        static const u8  ESPNOW_ADDR_LEN            = 6;    // Address length
        static const u8  ESPNOW_QUEUE_SIZE          = 3;    // Queue size

        typedef struct
        {
            uint16_t frame_head;
            uint16_t duration;
            u8       destination_address[6];
            u8       source_address[6];
            u8       broadcast_address[6];
            uint16_t sequence_control;
            u8       category_code;
            u8       organization_identifier[3];  // 0x18fe34
            u8       random_values[4];
            struct
            {
                u8 element_id;                  // 0xdd
                u8 lenght;                      //
                u8 organization_identifier[3];  // 0x18fe34
                u8 type;                        // 4
                u8 version;
                u8 body[0];
            } vendor_specific_content;
        } __attribute__((packed)) espnow_frame_format_t;

        typedef struct
        {
            u8  dstAddress[ESPNOW_ADDR_LEN];        /* Message topic*/
            u8  payload[ESPNOW_MAX_MESSAGE_LENGTH]; /* Message payload*/
            u32 payload_len;                        /* Payload length*/
        } comms_tx_queue_item_t;

        typedef struct
        {
            u8     srcAddress[ESPNOW_ADDR_LEN];        /* Source Address */
            u8     dstAddress[ESPNOW_ADDR_LEN];        /* Destination Address */
            u8     payload[ESPNOW_MAX_MESSAGE_LENGTH]; /* Message payload */
            u32    payload_len;                        /* Payload length */
            int8_t rssi;                               /* RSSI */
        } comms_rx_queue_item_t;

        // #    define MEASURE_THROUGHPUT

#    ifdef MEASURE_THROUGHPUT
        static const time_t MEAS_TP_EVERY_MS = 10000;  // @brief Measurement time period

        struct QuickEspNowDataThroughput
        {
            void addBytesDropped(uint64_t bytes) { txDataDropped += bytes; }
            void addBytesSent(uint64_t bytes) { txDataSent += bytes; }
            void addBytesReceived(uint64_t bytes) { rxDataReceived += bytes; }

            void calculate(uint64_t now)
            {
                uint64_t measTime = (now - lastDataTPMeas);
                lastDataTPMeas    = now;

                if (txDataSent > 0)
                {
                    txDataTP           = txDataSent * 1024 / measTime;
                    txDroppedDataRatio = (float)txDataDropped / (float)txDataSent;
                    txDataSent         = 0;
                }
                else
                {
                    txDataTP           = 0;
                    txDroppedDataRatio = 0;
                }

                if (rxDataReceived > 0)
                {
                    rxDataTP       = rxDataReceived * 1024 / measTime;
                    rxDataReceived = 0;
                }
                else
                {
                    rxDataTP = 0;
                }
                txDataDropped = 0;
            }

        private:
            uint64_t txDataSent         = 0;  // Total transmitted data in bytes
            uint64_t rxDataReceived     = 0;  // Total received data in bytes
            uint64_t txDataDropped      = 0;  // Total dropped transmitted data in bytes
            uint64_t lastDataTPMeas     = 0;  // Time of last throughput measurement
            float    txDataTP           = 0;  // in Kbps
            float    rxDataTP           = 0;  // in Kbps
            float    txDroppedDataRatio = 0;  // in %
        };
#    else
        struct QuickEspNowDataThroughput
        {
            void addBytesDropped(uint64_t) {}
            void addBytesSent(uint64_t) {}
            void addBytesReceived(uint64_t) {}
            void calculate(uint64_t) {}
        };
#    endif  // MEASURE_THROUGHPUT

        class instance_t
        {
        public:
            bool               begin(u8 channel = CURRENT_WIFI_CHANNEL, u32 interface = 0, bool synchronousSend = true);
            void               stop();
            comms_send_error_t send(const u8* dstAddress, const u8* payload, u32 payload_len);
            comms_send_error_t sendBcast(const u8* payload, u32 payload_len) { return send(ESPNOW_BROADCAST_ADDRESS, payload, payload_len); }
            void               onDataRcvd(rcvd_data_cb dataRcvd);
            void               onDataSent(sent_data_cb sentResult);
            u8                 getAddressLength() { return ESPNOW_ADDR_LEN; }
            u8                 getMaxMessageLength() { return ESPNOW_MAX_MESSAGE_LENGTH; }
            void               enableTransmit(bool enable);
            bool               setChannel(u8 channel, wifi_second_chan_t ch2 = WIFI_SECOND_CHAN_NONE);
            bool               setWiFiBandwidth(wifi_interface_t iface = WIFI_IF_AP, wifi_bandwidth_t bw = WIFI_BW_HT20);
            bool               readyToSendData();

        protected:
            // u8 gateway[ESPNOW_ADDR_LEN]; // @brief Gateway address
            u8           channel;         // @brief Comms channel to be used
            rcvd_data_cb dataRcvd   = 0;  // @brief Pointer to a function to be called on every received message
            sent_data_cb sentResult = 0;  // @brief Pointer to a function to be called to notify last sending status
                                          // peerType_t _ownPeerType; // @brief Stores peer type, node or gateway

            wifi_interface_t wifi_if;
            peer_list_t      peer_list;
            TaskHandle_t     espnowTxTask;
            TaskHandle_t     espnowRxTask;

            QuickEspNowDataThroughput dataThroughput;

            bool readyToSend            = true;
            bool waitingForConfirmation = false;
            bool synchronousSend        = false;
            u8   sentStatus;
            int  queueSize = ESPNOW_QUEUE_SIZE;

            QueueHandle_t tx_queue;
            QueueHandle_t rx_queue;

            // SemaphoreHandle_t espnow_send_mutex;
            // u8 channel;
            bool followWiFiChannel = false;

            bool initialize();
            bool addPeer(const u8* peer_addr);

            s32 sendEspNowMessage(comms_tx_queue_item_t* message);

            static void espnowTxTask_cb(void* param);
            static void espnowRxTask_cb(void* param);

            void espnowTxHandle();
            void espnowRxHandle();

            static void ICACHE_FLASH_ATTR rx_cb(const esp_now_recv_info_t* esp_now_info, const u8* data, int data_len);
            static void ICACHE_FLASH_ATTR tx_cb(const esp_now_send_info_t* tx_info, esp_now_send_status_t status);
        };

        instance_t quickEspNow;

        bool instance_t::begin(u8 channel, u32 wifi_interface, bool synchronousSend)
        {
            wifi_second_chan_t ch2 = WIFI_SECOND_CHAN_NONE;
            this->synchronousSend  = synchronousSend;

            DEBUG_DBG(QESPNOW_TAG, "Channel: %d, Interface: %d", channel, wifi_interface);
            // Set the wifi interface
            switch (wifi_interface)
            {
                case WIFI_IF_STA: wifi_if = WIFI_IF_STA; break;
                case WIFI_IF_AP: wifi_if = WIFI_IF_AP; break;
                default:
                    DEBUG_ERROR(QESPNOW_TAG, "Unknown wifi interface");
                    return false;
                    break;
            }

            // check channel
            if (channel != CURRENT_WIFI_CHANNEL && (channel < MIN_WIFI_CHANNEL || channel > MAX_WIFI_CHANNEL))
            {
                DEBUG_ERROR(QESPNOW_TAG, "Invalid wifi channel %d", channel);
                return false;
            }

            // use current channel
            if (channel == CURRENT_WIFI_CHANNEL)
            {
                u8 ch;
                esp_wifi_get_channel(&ch, &ch2);
                channel = ch;
                DEBUG_DBG(QESPNOW_TAG, "Current channel: %d : %d", channel, ch2);
                followWiFiChannel = true;
            }
            setChannel(channel, ch2);

            DEBUG_INFO(QESPNOW_TAG, ARDUHAL_LOG_COLOR(ARDUHAL_LOG_COLOR_RED) "Starting ESP-NOW in in channel %u interface %s", channel, wifi_if == WIFI_IF_STA ? "STA" : "AP");
            this->channel = channel;

            return initialize();
        }

        void instance_t::stop()
        {
            DEBUG_INFO(QESPNOW_TAG, "-------------> ESP-NOW STOP");
            vTaskDelete(espnowTxTask);
            vTaskDelete(espnowRxTask);
            esp_now_unregister_recv_cb();
            esp_now_unregister_send_cb();
            esp_now_deinit();
        }

        bool instance_t::readyToSendData() { return uxQueueMessagesWaiting(tx_queue) < queueSize; }

        bool instance_t::setChannel(u8 channel, wifi_second_chan_t ch2)
        {
            if (followWiFiChannel)
            {
                DEBUG_WARN(QESPNOW_TAG, "Cannot set channel while following WiFi channel");
                return false;
            }

            esp_err_t err_ok;
            if ((err_ok = esp_wifi_set_promiscuous(true)))
            {
                DEBUG_ERROR(QESPNOW_TAG, "Error setting promiscuous mode: %s", esp_err_to_name(err_ok));
                return false;
            }
            if ((err_ok = esp_wifi_set_channel(channel, ch2)))
            {  // This is needed even in STA mode. If not done and using IDF > 4.0, ESP-NOW will not work.
                DEBUG_DBG(QESPNOW_TAG, "Error setting wifi channel: %d - %s", err_ok, esp_err_to_name(err_ok));
                return false;
            }
            if ((err_ok = esp_wifi_set_promiscuous(false)))
            {
                DEBUG_ERROR(QESPNOW_TAG, "Error setting promiscuous mode off: %s", esp_err_to_name(err_ok));
                return false;
            }

            this->channel = channel;

            return true;
        }

        comms_send_error_t instance_t::send(const u8* dstAddress, const u8* payload, u32 payload_len)
        {
            comms_tx_queue_item_t message;

            if (!dstAddress || !payload || !payload_len)
            {
                DEBUG_WARN(QESPNOW_TAG, "Parameters error");
                return COMMS_SEND_PAYLOAD_LENGTH_ERROR;
            }

            if (payload_len > ESP_NOW_MAX_DATA_LEN)
            {
                DEBUG_WARN(QESPNOW_TAG, "Length error. %d", payload_len);
                return COMMS_SEND_PAYLOAD_LENGTH_ERROR;
            }

            if (uxQueueMessagesWaiting(tx_queue) >= queueSize)
            {
                // comms_tx_queue_item_t tempBuffer;
                // xQueueReceive (tx_queue, &tempBuffer, 0);
                dataThroughput.addBytesDropped(payload_len);
                // DEBUG_DBG (QESPNOW_TAG, "Message dropped");
                return COMMS_SEND_QUEUE_FULL_ERROR;
            }
            memcpy(message.dstAddress, dstAddress, ESP_NOW_ETH_ALEN);
            message.payload_len = payload_len;
            memcpy(message.payload, payload, payload_len);

            if (xQueueSend(tx_queue, &message, pdMS_TO_TICKS(10)))
            {
                dataThroughput.addBytesSent(message.payload_len);

                DEBUG_DBG(QESPNOW_TAG, "--------- %d Comms messages queued. Len: %d", uxQueueMessagesWaiting(tx_queue), payload_len);
                DEBUG_VERBOSE(QESPNOW_TAG, "--------- Ready to send is %s", readyToSend ? "true" : "false");
                DEBUG_VERBOSE(QESPNOW_TAG, "--------- SyncronousSend is %s", synchronousSend ? "true" : "false");
                if (synchronousSend)
                {
                    waitingForConfirmation = true;
                    DEBUG_INFO(QESPNOW_TAG, "--------- Waiting for send confirmation");
                    while (waitingForConfirmation)
                    {
                        taskYIELD();
                    }
                    DEBUG_INFO(QESPNOW_TAG, "--------- Confirmation is %s", sentStatus == ESP_NOW_SEND_SUCCESS ? "true" : "false");
                    return (sentStatus == ESP_NOW_SEND_SUCCESS) ? COMMS_SEND_OK : COMMS_SEND_CONFIRM_ERROR;
                }
                return COMMS_SEND_OK;
            }
            else
            {
                DEBUG_WARN(QESPNOW_TAG, "Error queuing Comms message to " MACSTR, MAC2STR(dstAddress));
                return COMMS_SEND_MSG_ENQUEUE_ERROR;
            }
        }

        void instance_t::onDataRcvd(rcvd_data_cb dataRcvd) { this->dataRcvd = dataRcvd; }
        void instance_t::onDataSent(sent_data_cb sentResult) { this->sentResult = sentResult; }

        s32 instance_t::sendEspNowMessage(comms_tx_queue_item_t* message)
        {
            s32 error;

            if (!message)
            {
                DEBUG_WARN(QESPNOW_TAG, "Message is null");
                return -1;
            }
            if (!(message->payload_len) || (message->payload_len > ESP_NOW_MAX_DATA_LEN))
            {
                DEBUG_WARN(QESPNOW_TAG, "Message length error");
                return -1;
            }

            DEBUG_VERBOSE(QESPNOW_TAG, "ESP-NOW message to " MACSTR, MAC2STR(message->dstAddress));

            addPeer(message->dstAddress);
            DEBUG_DBG(QESPNOW_TAG, "Peer added " MACSTR, MAC2STR(message->dstAddress));
            readyToSend = false;
            DEBUG_VERBOSE(QESPNOW_TAG, "-------------- Ready to send: false");

            error = esp_now_send(message->dstAddress, message->payload, message->payload_len);
            DEBUG_DBG(QESPNOW_TAG, "esp now send result = %s", esp_err_to_name(error));
            if (error != ESP_OK)
            {
                DEBUG_WARN(QESPNOW_TAG, "Error sending message: %s", esp_err_to_name(error));
            }
            // if (error == ESP_OK) {
            //     txDataSent += message->payload_len;
            // }
            if (error == ESP_ERR_ESPNOW_NO_MEM)
            {
                delay(2);
            }

            return error;
        }

        void instance_t::espnowTxHandle()
        {
            if (readyToSend)
            {
                // DEBUG_WARN ("Process queue: Elements: %d", tx_queue.size ());
                comms_tx_queue_item_t message;
                while (xQueueReceive(tx_queue, &message, pdMS_TO_TICKS(1000)))
                {
                    DEBUG_DBG(QESPNOW_TAG, "Comms message got from queue. %d left", uxQueueMessagesWaiting(tx_queue));
                    while (!readyToSend && !synchronousSend)
                    {
                        delay(0);
                    }
                    if (!sendEspNowMessage(&message))
                    {
                        DEBUG_DBG(QESPNOW_TAG, "Message to " MACSTR " sent. Len: %u", MAC2STR(message.dstAddress), message.payload_len);
                    }
                    else
                    {
                        DEBUG_WARN(QESPNOW_TAG, "Error sending message to " MACSTR ". Len: %u", MAC2STR(message.dstAddress), message.payload_len);
                    }
                    // message.payload_len = 0;
                    DEBUG_DBG(QESPNOW_TAG, "Comms message pop. Queue size %d", uxQueueMessagesWaiting(tx_queue));
                }
            }
            else
            {
                DEBUG_DBG(QESPNOW_TAG, "Not ready to send");
            }
        }

        void instance_t::enableTransmit(bool enable)
        {
            DEBUG_DBG(QESPNOW_TAG, "Send esp-now task %s", enable ? "enabled" : "disabled");
            if (enable)
            {
                vTaskResume(espnowTxTask);
                vTaskResume(espnowRxTask);
            }
            else
            {
                vTaskSuspend(espnowTxTask);
                vTaskSuspend(espnowRxTask);
            }
        }

        bool instance_t::addPeer(const u8* peer_addr)
        {
            esp_now_peer_info_t peer;
            esp_err_t           error = ESP_OK;

            if (peer_list.is_full())
            {
                DEBUG_VERBOSE(QESPNOW_TAG, "Peer list full. Deleting older");
                s32 oldest_peer = peer_list.find_oldest_peer();
                if (oldest_peer >= 0)
                {
                    u8* mac_of_oldest_peer = peer_list.get_mac(oldest_peer);
                    esp_now_del_peer(mac_of_oldest_peer);
                    peer_list.delete_peer(oldest_peer);
                }
                else
                {
                    DEBUG_ERROR(QESPNOW_TAG, "Error deleting peer");
                    return false;
                }
            }

            s32 peer_index = peer_list.find_active_peer(peer_addr);
            if (peer_index >= 0)
            {
                DEBUG_VERBOSE(QESPNOW_TAG, "Peer already exists");
                error = esp_now_get_peer(peer_addr, &peer);
                if (error == ESP_ERR_ESPNOW_NOT_FOUND)
                {
                    peer_list.delete_peer(peer_index);
                    DEBUG_ERROR(QESPNOW_TAG, "Peer not found in ESPNOW, adding again");
                    return addPeer(peer_addr);
                }
                else if (error != ESP_OK)
                {
                    ESP_ERROR_CHECK(error);
                }

                u8 currentChannel = peer.channel;
                DEBUG_DBG(QESPNOW_TAG, "Peer " MACSTR " is using channel %d", MAC2STR(peer_addr), currentChannel);
                if (currentChannel != this->channel)
                {
                    DEBUG_DBG(QESPNOW_TAG, "Peer channel has to change from %d to %d", currentChannel, this->channel);
                    ESP_ERROR_CHECK_WITHOUT_ABORT(esp_now_get_peer(peer_addr, &peer));
                    peer.channel = this->channel;
                    ESP_ERROR_CHECK_WITHOUT_ABORT(esp_now_mod_peer(&peer));
                    DEBUG_ERROR(QESPNOW_TAG, "Peer channel changed to %d", this->channel);
                }
                return true;
            }

            // Setup ESPNOW peer
            memcpy(peer.peer_addr, peer_addr, ESP_NOW_ETH_ALEN);
            u8                 ch;
            wifi_second_chan_t secondCh;
            esp_wifi_get_channel(&ch, &secondCh);
            peer.channel = ch;
            peer.ifidx   = wifi_if;
            peer.encrypt = false;

            error = esp_now_add_peer(&peer);
            if (!error)
            {
                DEBUG_DBG(QESPNOW_TAG, "Peer added");
                peer_list.add_peer(peer_addr);
            }
            else
            {
                DEBUG_ERROR(QESPNOW_TAG, "Error adding peer: %s", esp_err_to_name(error));
                return false;
            }
            DEBUG_DBG(QESPNOW_TAG, "Peer " MACSTR " added on channel %u. Result 0x%X %s", MAC2STR(peer_addr), ch, error, esp_err_to_name(error));
            return error == ESP_OK;
        }

        bool instance_t::initialize()
        {
            if (esp_now_init())
            {
                DEBUG_ERROR(QESPNOW_TAG, "Failed to init ESP-NOW");
                ESP.restart();
                delay(1);
                return false;
            }

            esp_now_register_recv_cb(rx_cb);
            esp_now_register_send_cb(tx_cb);

            int txQueueSize = queueSize;
            if (synchronousSend)
            {
                txQueueSize = 1;
            }

            tx_queue = xQueueCreate(txQueueSize, sizeof(comms_tx_queue_item_t));
            xTaskCreateUniversal(espnowTxTask_cb, "espnow_loop", 8 * 1024, NULL, 1, &espnowTxTask, CONFIG_ARDUINO_RUNNING_CORE);

            rx_queue = xQueueCreate(queueSize, sizeof(comms_rx_queue_item_t));
            xTaskCreateUniversal(espnowRxTask_cb, "receive_handle", 4 * 1024, NULL, 1, &espnowRxTask, CONFIG_ARDUINO_RUNNING_CORE);

            return true;
        }

        void instance_t::espnowTxTask_cb(void* param)
        {
            for (;;)
            {
                quickEspNow.espnowTxHandle();
            }
        }

        void instance_t::espnowRxHandle()
        {
            comms_rx_queue_item_t rxMessage;

            if (xQueueReceive(rx_queue, &rxMessage, portMAX_DELAY))
            {
                DEBUG_DBG(QESPNOW_TAG, "Comms message got from queue. %d left", uxQueueMessagesWaiting(rx_queue));
                DEBUG_VERBOSE(QESPNOW_TAG, "Received message from " MACSTR " Len: %u", MAC2STR(rxMessage.srcAddress), rxMessage.payload_len);
                DEBUG_VERBOSE(QESPNOW_TAG, "Message: %.*s", rxMessage.payload_len, rxMessage.payload);

                if (quickEspNow.dataRcvd)
                {
                    bool broadcast = !memcmp(rxMessage.dstAddress, ESPNOW_BROADCAST_ADDRESS, ESP_NOW_ETH_ALEN);
                    quickEspNow.dataRcvd(rxMessage.srcAddress, rxMessage.payload, rxMessage.payload_len, rxMessage.rssi, broadcast);  // rssi should be in dBm but it has added almost 100 dB. Do not know why
                }
            }
            else
            {
                DEBUG_DBG(QESPNOW_TAG, "No message in queue");
            }
        }

        void instance_t::espnowRxTask_cb(void* param)
        {
            while (true)
            {
                quickEspNow.espnowRxHandle();
            }
        }

        // typedef void (*esp_now_recv_cb_t)(const esp_now_recv_info_t * esp_now_info, const u8 *data, int data_len);
        void instance_t::rx_cb(const esp_now_recv_info_t* esp_now_info, const u8* data, int data_len)
        {
            espnow_frame_format_t*  espnow_data     = (espnow_frame_format_t*)(data - sizeof(espnow_frame_format_t));
            wifi_promiscuous_pkt_t* promiscuous_pkt = (wifi_promiscuous_pkt_t*)(data - sizeof(wifi_pkt_rx_ctrl_t) - sizeof(espnow_frame_format_t));
            wifi_pkt_rx_ctrl_t*     rx_ctrl         = &promiscuous_pkt->rx_ctrl;

            DEBUG_DBG(QESPNOW_TAG, "Received message with RSSI %d from " MACSTR " length: %u", rx_ctrl->rssi, MAC2STR(mac_addr), len);

            comms_rx_queue_item_t message;
            memcpy(message.srcAddress, esp_now_info->src_addr, ESP_NOW_ETH_ALEN);
            memcpy(message.payload, data, data_len);
            message.payload_len = data_len;
            message.rssi        = rx_ctrl->rssi;
            memcpy(message.dstAddress, espnow_data->destination_address, ESP_NOW_ETH_ALEN);

            if (uxQueueMessagesWaiting(quickEspNow.rx_queue) >= quickEspNow.queueSize)
            {
                comms_rx_queue_item_t tempBuffer;
                xQueueReceive(quickEspNow.rx_queue, &tempBuffer, 0);
                DEBUG_DBG(QESPNOW_TAG, "Rx Message dropped");
            }
            quickEspNow.dataThroughput.addBytesReceived(data_len);

            if (!xQueueSend(quickEspNow.rx_queue, &message, pdMS_TO_TICKS(100)))
            {
                DEBUG_WARN(QESPNOW_TAG, "Error sending message to rx queue");
            }
        }

        void instance_t::tx_cb(const esp_now_send_info_t* tx_info, esp_now_send_status_t status)
        {
            quickEspNow.readyToSend            = true;
            quickEspNow.sentStatus             = status;
            quickEspNow.waitingForConfirmation = false;
            DEBUG_DBG(QESPNOW_TAG, "-------------- Ready to send: true. Status: %d", status);
            if (quickEspNow.sentResult)
            {
                quickEspNow.sentResult(tx_info->des_addr, status);
            }
        }

        bool instance_t::setWiFiBandwidth(wifi_interface_t iface, wifi_bandwidth_t bw)
        {
            esp_err_t err_ok;
            if ((err_ok = esp_wifi_set_bandwidth(iface, bw)))
            {
                DEBUG_ERROR(QESPNOW_TAG, "Error setting wifi bandwidth: %s", esp_err_to_name(err_ok));
            }
            return !err_ok;
        }

        // Public API functions

        bool init(bool initialize_wifi, bool synchronous_send)
        {
            const bool synchronousSend = false;
            return quickEspNow.begin(11, WIFI_IF_STA, synchronousSend);
        }

        void start(rcvd_data_cb rcvd_cb, sent_data_cb sent_cb)
        {
            quickEspNow.onDataRcvd(rcvd_cb);
            quickEspNow.onDataSent(sent_cb);
        }

        bool deinit(void)
        {
            quickEspNow.stop();
            return true;
        }

        bool send(const u8* target, const u8* data, const u8 data_len)
        {
            // Send data using ESP-NOW
            return quickEspNow.send(target, data, data_len) == COMMS_SEND_OK;
        }

    }  // namespace nqespnow
}  // namespace ncore

#endif
