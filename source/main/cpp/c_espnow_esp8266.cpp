#if defined(TARGET_ARDUINO) && defined(TARGET_ESP8266)

#    include "rcore/c_debug.h"
#    include "respnow/c_espnow.h"

#    include "Arduino.h"
#    include "espnow.h"
#    include "ESP8266WiFi.h"

// ----------------------------------------------------------------------------------------
// ----------------------------------------------------------------------------------------
namespace ncore
{
    namespace nqespnow
    {

        DEBUG_TAG(RINGBUFFER_DEBUG_TAG, "RINGBUFFER");

        /**
         * @brief Ring buffer class. Used to implement message buffer
         *
         */
        template <typename Telement>
        class RingBuffer
        {
        protected:
            int       maxSize;          // Buffer size
            int       numElements = 0;  // Number of elements that buffer currently has
            int       readIndex   = 0;  // Pointer to next item to be read
            int       writeIndex  = 0;  // Pointer to next position to write onto
            Telement* buffer;           // Actual buffer

        public:
            /**
             * @brief Creates a ring buffer to hold `Telement` objects
             * @param range Buffer depth
             */
            RingBuffer<Telement>(int range)
                : maxSize(range)
            {
                buffer = new Telement[maxSize];
            }

            /**
             * @brief EnigmaIOTRingBuffer destructor
             * @param range Free up buffer memory
             */
            ~RingBuffer()
            {
                maxSize = 0;
                delete[] (buffer);
            }

            /**
             * @brief Returns actual number of elements that buffer holds
             * @return Returns Actual number of elements that buffer holds
             */
            int size() { return numElements; }

            /**
             * @brief Checks if buffer is full
             * @return Returns `true`if buffer is full, `false` otherwise
             */
            bool isFull() { return numElements == maxSize; }

            /**
             * @brief Checks if buffer is empty
             * @return Returns `true`if buffer has no elements stored, `false` otherwise
             */
            bool empty() { return (numElements == 0); }

            /**
             * @brief Adds a new item to buffer, deleting older element if it is full
             * @param item Element to add to buffer
             * @return Returns `false` if buffer was full before inserting the new element, `true` otherwise
             */
            bool push(Telement* item)
            {
                bool wasFull = isFull();
                DEBUG_DBG(RINGBUFFER_DEBUG_TAG, "Add element. Buffer was %s", wasFull ? "full" : "not full");
                DEBUG_DBG(RINGBUFFER_DEBUG_TAG, "Before -- > ReadIdx: %d. WriteIdx: %d. Size: %d", readIndex, writeIndex, numElements);
#    ifdef ESP32
                portMUX_TYPE myMutex = portMUX_INITIALIZER_UNLOCKED;
                portENTER_CRITICAL(&myMutex);
#    endif
                memcpy(&(buffer[writeIndex]), item, sizeof(Telement));
                // Serial.printf ("Copied: %d bytes\n", sizeof (Telement));
                writeIndex++;
                if (writeIndex >= maxSize)
                {
                    writeIndex %= maxSize;
                }
                if (wasFull)
                {  // old value is no longer valid
                    readIndex++;
                    if (readIndex >= maxSize)
                    {
                        readIndex %= maxSize;
                    }
                }
                else
                {
                    numElements++;
                }
#    ifdef ESP32
                portEXIT_CRITICAL(&myMutex);
#    endif
                DEBUG_DBG(RINGBUFFER_DEBUG_TAG, "After -- > ReadIdx: %d. WriteIdx: %d. Size: %d", readIndex, writeIndex, numElements);
                return !wasFull;
            }

            /**
             * @brief Deletes older item from buffer, if buffer is not empty
             * @return Returns `false` if buffer was empty before trying to delete element, `true` otherwise
             */
            bool pop()
            {
                bool wasEmpty = empty();
                DEBUG_DBG(RINGBUFFER_DEBUG_TAG, "Remove element. Buffer was %s", wasEmpty ? "empty" : "not empty");
                DEBUG_DBG(RINGBUFFER_DEBUG_TAG, "Before -- > ReadIdx: %d. WriteIdx: %d. Size: %d", readIndex, writeIndex, numElements);
                if (!wasEmpty)
                {
                    readIndex++;
                    if (readIndex >= maxSize)
                    {
                        readIndex %= maxSize;
                    }
                    numElements--;
                }
                DEBUG_DBG(RINGBUFFER_DEBUG_TAG, "After -- > ReadIdx: %d. WriteIdx: %d. Size: %d", readIndex, writeIndex, numElements);
                return !wasEmpty;
            }

            /**
             * @brief Gets a pointer to older item in buffer, if buffer is not empty
             * @return Returns pointer to element. If buffer was empty before calling this method it returns `NULL`
             */
            Telement* front()
            {
                DEBUG_DBG(RINGBUFFER_DEBUG_TAG, "Read element. ReadIdx: %d. WriteIdx: %d. Size: %d", readIndex, writeIndex, numElements);
                if (!empty())
                {
                    return &(buffer[readIndex]);
                }
                else
                {
                    return NULL;
                }
            }
        };

        // ----------------------------------------------------------------------------------------
        // ----------------------------------------------------------------------------------------

        typedef std::function<void(uint8_t* address, uint8_t* data, uint8_t len, signed int rssi, bool broadcast)> comms_hal_rcvd_data;
        typedef std::function<void(uint8_t* address, uint8_t status)>                                              comms_hal_sent_data;

        typedef enum
        {
            COMMS_SEND_OK                   = 0,  /* Data was enqued for sending successfully */
            COMMS_SEND_PARAM_ERROR          = -1, /* Data was not sent due to parameter call error */
            COMMS_SEND_PAYLOAD_LENGTH_ERROR = -2, /* Data was not sent due to payload too long */
            COMMS_SEND_QUEUE_FULL_ERROR     = -3, /* Data was not sent due to queue full */
            COMMS_SEND_MSG_ENQUEUE_ERROR    = -4, /* Data was not sent due to message queue push error */
            COMMS_SEND_CONFIRM_ERROR        = -5, /* Data was not sent due to confirmation error (only for synchronous send) */
        } comms_send_error_t;

        DEBUG_TAG(QESPNOW_TAG, "QESPNOW");

        static const uint8_t ESPNOW_BROADCAST_ADDRESS[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
        static const uint8_t MIN_WIFI_CHANNEL           = 0;
        static const uint8_t MAX_WIFI_CHANNEL           = 14;
        static const uint8_t CURRENT_WIFI_CHANNEL       = 255;  // Initialize with current channel (check your router settings)
        static const size_t  ESPNOW_MAX_MESSAGE_LENGTH  = 240;  // Maximum message length
        static const uint8_t ESPNOW_ADDR_LEN            = 6;    // Address length
        static const uint8_t ESPNOW_QUEUE_SIZE          = 4;    // Queue size
        static const int     TASK_PERIOD                = 10;   // Rx and Tx tasks period

#    define ESP_NOW_ETH_ALEN     6
#    define ESP_NOW_MAX_DATA_LEN 250
#    define WIFI_IF_STA          STATION_IF
#    define WIFI_IF_AP           SOFTAP_IF

        typedef enum
        {
            ESP_NOW_SEND_SUCCESS = 0, /* Send ESPNOW data successfully */
            ESP_NOW_SEND_FAIL,        /* Send ESPNOW data fail */
        } esp_now_send_status_t;

        typedef struct
        {
            uint16_t frame_head;
            uint16_t duration;
            uint8_t  destination_address[6];
            uint8_t  source_address[6];
            uint8_t  broadcast_address[6];
            uint16_t sequence_control;

            uint8_t category_code;
            uint8_t organization_identifier[3];  // 18 fe 34
            uint8_t random_values[4];
            struct
            {
                uint8_t element_id;                  // 0xdd
                uint8_t lenght;                      //
                uint8_t organization_identifier[3];  // 18 fe 34
                uint8_t type;                        // 4
                uint8_t version;
                uint8_t body[0];
            } vendor_specific_content;
        } __attribute__((packed)) espnow_frame_format_t;

        typedef struct
        {
            uint8_t dstAddress[ESPNOW_ADDR_LEN];        /* Message topic*/
            uint8_t payload[ESPNOW_MAX_MESSAGE_LENGTH]; /* Message payload*/
            size_t  payload_len;                        /* Payload length*/
        } comms_tx_queue_item_t;

        typedef struct
        {
            uint8_t srcAddress[ESPNOW_ADDR_LEN];        /* Source Address */
            uint8_t dstAddress[ESPNOW_ADDR_LEN];        /* Destination Address */
            uint8_t payload[ESPNOW_MAX_MESSAGE_LENGTH]; /* Message payload */
            size_t  payload_len;                        /* Payload length */
            int8_t  rssi;                               /* RSSI */
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

        class QuickEspNow
        {
        public:
            QuickEspNow()
                : tx_queue(ESPNOW_QUEUE_SIZE)
                , rx_queue(ESPNOW_QUEUE_SIZE)
            {
            }

            bool               begin(uint8_t channel = 255, uint32_t interface = 0, bool synchronousSend = true);
            void               stop();
            comms_send_error_t send(const uint8_t* dstAddress, const uint8_t* payload, size_t payload_len);
            comms_send_error_t sendBcast(const uint8_t* payload, size_t payload_len) { return send(ESPNOW_BROADCAST_ADDRESS, payload, payload_len); }
            void               onDataRcvd(comms_hal_rcvd_data dataRcvd);
            void               onDataSent(comms_hal_sent_data sentResult);
            uint8_t            getAddressLength() { return ESPNOW_ADDR_LEN; }
            uint8_t            getMaxMessageLength() { return ESPNOW_MAX_MESSAGE_LENGTH; }
            void               enableTransmit(bool enable);
            bool               setChannel(uint8_t channel);
            bool               readyToSendData();

        protected:
            // uint8_t gateway[ESPNOW_ADDR_LEN]; // Gateway address
            uint8_t channel;  // Comms channel to be used

            comms_hal_rcvd_data dataRcvd   = 0;  // Pointer to a function to be called on every received message
            comms_hal_sent_data sentResult = 0;  // Pointer to a function to be called to notify last sending status
                                                 // peerType_t _ownPeerType; // Stores peer type, node or gateway
            uint8_t  wifi_if;
            ETSTimer espnowTxTask;
            ETSTimer espnowRxTask;

            bool    readyToSend            = true;
            bool    waitingForConfirmation = false;
            bool    synchronousSend        = false;
            bool    followWiFiChannel      = false;
            int     queueSize              = ESPNOW_QUEUE_SIZE;
            uint8_t sentStatus;

            QuickEspNowDataThroughput dataThroughput;

            RingBuffer<comms_tx_queue_item_t> tx_queue;
            RingBuffer<comms_rx_queue_item_t> rx_queue;

            bool        initialize();
            static void espnowTxTask_cb(void* param);
            static void espnowRxTask_cb(void* param);
            int32_t     sendEspNowMessage(comms_tx_queue_item_t* message);
            void        espnowTxHandle();
            void        espnowRxHandle();

            static void ICACHE_FLASH_ATTR rx_cb(uint8_t* mac_addr, uint8_t* data, uint8_t len);
            static void ICACHE_FLASH_ATTR tx_cb(uint8_t* mac_addr, uint8_t status);
        };

        typedef struct
        {
            signed   rssi : 8;
            unsigned rate : 4;
            unsigned is_group : 1;
            unsigned : 1;
            unsigned sig_mode : 2;
            unsigned legacy_length : 12;
            unsigned damatch0 : 1;
            unsigned damatch1 : 1;
            unsigned bssidmatch0 : 1;
            unsigned bssidmatch1 : 1;
            unsigned MCS : 7;
            unsigned CWB : 1;
            unsigned HT_length : 16;
            unsigned Smoothing : 1;
            unsigned Not_Sounding : 1;
            unsigned : 1;
            unsigned Aggregation : 1;
            unsigned STBC : 2;
            unsigned FEC_CODING : 1;
            unsigned SGI : 1;
            unsigned rxend_state : 8;
            unsigned ampdu_cnt : 8;
            unsigned channel : 4;
            unsigned : 12;
        } wifi_pkt_rx_ctrl_t;

        typedef struct
        {
            wifi_pkt_rx_ctrl_t rx_ctrl;
            uint8_t            payload[0]; /* ieee80211 packet buff */
        } wifi_promiscuous_pkt_t;

        QuickEspNow quickEspNow;

        bool QuickEspNow::begin(uint8_t channel, uint32_t wifi_interface, bool synchronousSend)
        {
            this->synchronousSend = synchronousSend;

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
                uint8_t ch;
                ch = WiFi.channel();
                DEBUG_DBG(QESPNOW_TAG, "Current channel: %d", ch);
                channel           = ch;
                followWiFiChannel = true;
            }
            else
            {
                setChannel(channel);
            }
            DEBUG_INFO(QESPNOW_TAG, "Starting ESP-NOW using channel %u, interface %s", channel, wifi_if == WIFI_IF_STA ? "STA" : "AP");
            this->channel = channel;

            // addPeer (ESPNOW_BROADCAST_ADDRESS); // Not needed ?

            return initialize();
        }

        bool QuickEspNow::initialize()
        {
            if (esp_now_init())
            {
                DEBUG_ERROR(QESPNOW_TAG, "Failed to init ESP-NOW");
                return false;
            }

            if (wifi_if == WIFI_IF_STA)
            {
                esp_now_set_self_role(ESP_NOW_ROLE_SLAVE);
            }
            else
            {
                esp_now_set_self_role(ESP_NOW_ROLE_CONTROLLER);
            }

            esp_now_register_recv_cb(rx_cb);
            esp_now_register_send_cb(tx_cb);

            os_timer_setfn(&espnowTxTask, espnowTxTask_cb, NULL);
            os_timer_arm(&espnowTxTask, TASK_PERIOD, true);

            os_timer_setfn(&espnowRxTask, espnowRxTask_cb, NULL);
            os_timer_arm(&espnowRxTask, TASK_PERIOD, true);

            return true;
        }

        void QuickEspNow::stop()
        {
            DEBUG_INFO(QESPNOW_TAG, "Stop ESP-NOW");
            os_timer_disarm(&espnowTxTask);
            os_timer_disarm(&espnowRxTask);
            esp_now_unregister_recv_cb();
            esp_now_unregister_send_cb();
            esp_now_deinit();
        }

        bool QuickEspNow::readyToSendData() { return tx_queue.size() < queueSize; }

        bool QuickEspNow::setChannel(uint8_t channel)
        {
            if (followWiFiChannel)
            {
                DEBUG_WARN(QESPNOW_TAG, "Cannot set channel while following WiFi channel");
                return false;
            }

            if (!wifi_set_channel(channel))
            {
                DEBUG_ERROR(QESPNOW_TAG, "Error setting wifi channel: %u", channel);
                return false;
            }

            this->channel = channel;

            return true;
        }

        comms_send_error_t QuickEspNow::send(const uint8_t* dstAddress, const uint8_t* payload, size_t payload_len)
        {
            comms_tx_queue_item_t message;

            if (!dstAddress || !payload || !payload_len)
            {
                DEBUG_WARN(QESPNOW_TAG, "Parameters error");
                return COMMS_SEND_PARAM_ERROR;
            }

            if (payload_len > ESP_NOW_MAX_DATA_LEN)
            {
                DEBUG_WARN(QESPNOW_TAG, "Length error. %d", payload_len);
                return COMMS_SEND_PAYLOAD_LENGTH_ERROR;
            }

            if (tx_queue.size() >= ESPNOW_QUEUE_SIZE)
            {
                return COMMS_SEND_QUEUE_FULL_ERROR;
            }

            memcpy(message.dstAddress, dstAddress, ESP_NOW_ETH_ALEN);
            message.payload_len = payload_len;
            memcpy(message.payload, payload, payload_len);

            if (tx_queue.push(&message))
            {
                dataThroughput.addBytesSent(message.payload_len);
                DEBUG_DBG(QESPNOW_TAG, "--------- %d Comms messages queued. Len: %d", tx_queue.size(), payload_len);
                DEBUG_VERBOSE(QESPNOW_TAG, "--------- Ready to send is %s", readyToSend ? "true" : "false");
                if (synchronousSend)
                {
                    waitingForConfirmation = true;
                    DEBUG_INFO(QESPNOW_TAG, "--------- Waiting for send confirmation");
                    while (waitingForConfirmation)
                    {
                        esp_yield();
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

        void QuickEspNow::onDataRcvd(comms_hal_rcvd_data dataRcvd) { this->dataRcvd = dataRcvd; }
        void QuickEspNow::onDataSent(comms_hal_sent_data sentResult) { this->sentResult = sentResult; }

        int32_t QuickEspNow::sendEspNowMessage(comms_tx_queue_item_t* message)
        {
            int32_t error;

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

            readyToSend = false;
            DEBUG_VERBOSE(QESPNOW_TAG, "-------------- Ready to send: false");

            error = esp_now_send(message->dstAddress, message->payload, message->payload_len);
            DEBUG_DBG(QESPNOW_TAG, "esp now send result = %d", error);

            return error;
        }

        void QuickEspNow::espnowTxHandle()
        {
            if (readyToSend)
            {
                // DEBUG_WARN ("Process queue: Elements: %d", tx_queue.size ());
                comms_tx_queue_item_t* message;
                while (!tx_queue.empty())
                {
                    if (!readyToSend)
                        return;
                    message = tx_queue.front();
                    DEBUG_DBG(QESPNOW_TAG, "Comms message got from queue. %d left", tx_queue.size());
                    DEBUG_VERBOSE(QESPNOW_TAG, "Ready to send is %s", readyToSend ? "true" : "false");
                    DEBUG_VERBOSE(QESPNOW_TAG, "synchrnousSend is %s", synchronousSend ? "true" : "false");
                    if (!sendEspNowMessage(message))
                    {
                        DEBUG_DBG(QESPNOW_TAG, "Message to " MACSTR " sent. Len: %u", MAC2STR(message->dstAddress), message->payload_len);
                    }
                    else
                    {
                        DEBUG_WARN(QESPNOW_TAG, "Error sending message to " MACSTR ". Len: %u", MAC2STR(message->dstAddress), message->payload_len);
                    }
                    message->payload_len = 0;
                    tx_queue.pop();
                    DEBUG_DBG(QESPNOW_TAG, "Comms message pop. Queue size %d", tx_queue.size());
                }
            }
            else
            {
                DEBUG_DBG(QESPNOW_TAG, "Not ready to send");
            }
        }

        void QuickEspNow::enableTransmit(bool enable)
        {
            DEBUG_DBG(QESPNOW_TAG, "Send esp-now task %s", enable ? "enabled" : "disabled");
            if (enable)
            {
                os_timer_arm(&espnowTxTask, TASK_PERIOD, true);
                os_timer_arm(&espnowRxTask, TASK_PERIOD, true);
            }
            else
            {
                os_timer_disarm(&espnowTxTask);
                os_timer_disarm(&espnowRxTask);
            }
        }

        void QuickEspNow::espnowTxTask_cb(void* param) { quickEspNow.espnowTxHandle(); }

        void QuickEspNow::rx_cb(uint8_t* mac_addr, uint8_t* data, uint8_t len)
        {
            espnow_frame_format_t*  espnow_data     = (espnow_frame_format_t*)(data - sizeof(espnow_frame_format_t));
            wifi_promiscuous_pkt_t* promiscuous_pkt = (wifi_promiscuous_pkt_t*)(data - sizeof(wifi_pkt_rx_ctrl_t) - sizeof(espnow_frame_format_t));
            wifi_pkt_rx_ctrl_t*     rx_ctrl         = &promiscuous_pkt->rx_ctrl;

            comms_rx_queue_item_t message;

            DEBUG_DBG(QESPNOW_TAG, "Received message with RSSI %d from " MACSTR " Len: %u", rx_ctrl->rssi, MAC2STR(mac_addr), len);

            memcpy(message.srcAddress, mac_addr, ESP_NOW_ETH_ALEN);
            memcpy(message.payload, data, len);
            message.payload_len = len;
            message.rssi        = rx_ctrl->rssi - 100;
            memcpy(message.dstAddress, espnow_data->destination_address, ESP_NOW_ETH_ALEN);

            if (quickEspNow.rx_queue.size() >= ESPNOW_QUEUE_SIZE)
            {
                quickEspNow.tx_queue.pop();
                DEBUG_DBG(QESPNOW_TAG, "Rx Message dropped");
            }

            quickEspNow.dataThroughput.addBytesReceived(len);

            if (quickEspNow.rx_queue.push(&message))
            {
                DEBUG_DBG(QESPNOW_TAG, "Message pushed to queue");
            }
            else
            {
                DEBUG_WARN(QESPNOW_TAG, "Error queuing message");
            }
        }

        void QuickEspNow::espnowRxTask_cb(void* param) { quickEspNow.espnowRxHandle(); }

        void QuickEspNow::espnowRxHandle()
        {
            comms_rx_queue_item_t* rxMessage;

            if (!rx_queue.empty())
            {
                rxMessage = rx_queue.front();
                DEBUG_DBG(QESPNOW_TAG, "Comms message got from queue. %d left", rx_queue.size());
                DEBUG_VERBOSE(QESPNOW_TAG, "Received message from " MACSTR " Len: %u", MAC2STR(rxMessage->srcAddress), rxMessage->payload_len);
                DEBUG_VERBOSE(QESPNOW_TAG, "Message: %.*s", rxMessage->payload_len, rxMessage->payload);

                if (quickEspNow.dataRcvd)
                {
                    bool broadcast = !memcmp(rxMessage->dstAddress, ESPNOW_BROADCAST_ADDRESS, ESP_NOW_ETH_ALEN);
                    // quickEspNow.dataRcvd (mac_addr, data, len, rx_ctrl->rssi - 98); // rssi should be in dBm but it has added almost 100 dB. Do not know why
                    quickEspNow.dataRcvd(rxMessage->srcAddress, rxMessage->payload, rxMessage->payload_len, rxMessage->rssi, broadcast);  // rssi should be in dBm but it has added almost 100 dB. Do not know why
                }

                rxMessage->payload_len = 0;
                rx_queue.pop();
                DEBUG_DBG(QESPNOW_TAG, "RX Comms message pop. Queue size %d", rx_queue.size());
            }
        }

        void QuickEspNow::tx_cb(uint8_t* mac_addr, uint8_t status)
        {
            quickEspNow.readyToSend = true;
            quickEspNow.sentStatus  = status;
            DEBUG_DBG(QESPNOW_TAG, "-------------- Tx Confirmed %s", status == ESP_NOW_SEND_SUCCESS ? "true" : "false");
            quickEspNow.waitingForConfirmation = false;
            DEBUG_DBG(QESPNOW_TAG, "-------------- Ready to send: true");
            if (quickEspNow.sentResult)
            {
                quickEspNow.sentResult(mac_addr, status);
            }
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
