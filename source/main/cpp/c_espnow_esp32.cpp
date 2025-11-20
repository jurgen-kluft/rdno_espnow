
#if defined(TARGET_ARDUINO) && defined(TARGET_ESP32)

#    include "rdno_core/c_debug.h"

#    include "Arduino.h"

#    include <esp_now.h>
#    include <esp_wifi.h>

#    include <freertos/FreeRTOS.h>
#    include <freertos/semphr.h>
#    include <freertos/queue.h>
#    include <freertos/task.h>

#    include "rdno_espnow/private/c_peer_list.h"

// ----------------------------------------------------------------------------------------
// ----------------------------------------------------------------------------------------

DEBUG_TAG(QESPNOW_TAG, "QESPNOW");

typedef std::function<void(uint8_t* address, uint8_t* data, uint8_t len, int rssi, bool broadcast)> comms_hal_rcvd_data;
typedef std::function<void(uint8_t* address, uint8_t status)>                                       comms_hal_sent_data;

typedef enum
{
    COMMS_SEND_OK                   = 0,  /* Data was enqued for sending successfully */
    COMMS_SEND_PARAM_ERROR          = -1, /* Data was not sent due to parameter call error */
    COMMS_SEND_PAYLOAD_LENGTH_ERROR = -2, /* Data was not sent due to payload too long */
    COMMS_SEND_QUEUE_FULL_ERROR     = -3, /* Data was not sent due to queue full */
    COMMS_SEND_MSG_ENQUEUE_ERROR    = -4, /* Data was not sent due to message queue push error */
    COMMS_SEND_CONFIRM_ERROR        = -5, /* Data was not sent due to confirmation error (only for synchronous send) */
} comms_send_error_t;

static uint8_t       ESPNOW_BROADCAST_ADDRESS[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
static const uint8_t MIN_WIFI_CHANNEL           = 0;
static const uint8_t MAX_WIFI_CHANNEL           = 14;
static const uint8_t CURRENT_WIFI_CHANNEL       = 255;  // Initialize with current channel (check your router settings)
static const size_t  ESPNOW_MAX_MESSAGE_LENGTH  = 240;  // Maximum message length
static const uint8_t ESPNOW_ADDR_LEN            = 6;    // Address length
static const uint8_t ESPNOW_QUEUE_SIZE          = 3;    // Queue size

typedef struct
{
    uint16_t frame_head;
    uint16_t duration;
    uint8_t  destination_address[6];
    uint8_t  source_address[6];
    uint8_t  broadcast_address[6];
    uint16_t sequence_control;
    uint8_t  category_code;
    uint8_t  organization_identifier[3];  // 0x18fe34
    uint8_t  random_values[4];
    struct
    {
        uint8_t element_id;                  // 0xdd
        uint8_t lenght;                      //
        uint8_t organization_identifier[3];  // 0x18fe34
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

//#    define MEASURE_THROUGHPUT

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
    bool               begin(uint8_t channel = CURRENT_WIFI_CHANNEL, uint32_t interface = 0, bool synchronousSend = true);
    void               stop();
    comms_send_error_t send(const uint8_t* dstAddress, const uint8_t* payload, size_t payload_len);
    comms_send_error_t sendBcast(const uint8_t* payload, size_t payload_len) { return send(ESPNOW_BROADCAST_ADDRESS, payload, payload_len); }
    void               onDataRcvd(comms_hal_rcvd_data dataRcvd);
    void               onDataSent(comms_hal_sent_data sentResult);
    uint8_t            getAddressLength() { return ESPNOW_ADDR_LEN; }
    uint8_t            getMaxMessageLength() { return ESPNOW_MAX_MESSAGE_LENGTH; }
    void               enableTransmit(bool enable);
    bool               setChannel(uint8_t channel, wifi_second_chan_t ch2 = WIFI_SECOND_CHAN_NONE);
    bool               setWiFiBandwidth(wifi_interface_t iface = WIFI_IF_AP, wifi_bandwidth_t bw = WIFI_BW_HT20);
    bool               readyToSendData();

protected:
    // uint8_t gateway[ESPNOW_ADDR_LEN]; // @brief Gateway address
    uint8_t             channel;         // @brief Comms channel to be used
    comms_hal_rcvd_data dataRcvd   = 0;  // @brief Pointer to a function to be called on every received message
    comms_hal_sent_data sentResult = 0;  // @brief Pointer to a function to be called to notify last sending status
                                         // peerType_t _ownPeerType; // @brief Stores peer type, node or gateway

    wifi_interface_t wifi_if;
    peer_list_t      peer_list;
    TaskHandle_t     espnowTxTask;
    TaskHandle_t     espnowRxTask;

    QuickEspNowDataThroughput dataThroughput;

    bool    readyToSend            = true;
    bool    waitingForConfirmation = false;
    bool    synchronousSend        = false;
    uint8_t sentStatus;
    int     queueSize = ESPNOW_QUEUE_SIZE;

    QueueHandle_t tx_queue;
    QueueHandle_t rx_queue;
    // SemaphoreHandle_t espnow_send_mutex;
    // uint8_t channel;
    bool followWiFiChannel = false;

    void        initComms();
    bool        addPeer(const uint8_t* peer_addr);
    static void espnowTxTask_cb(void* param);
    int32_t     sendEspNowMessage(comms_tx_queue_item_t* message);
    void        espnowTxHandle();

    static void espnowRxTask_cb(void* param);
    void        espnowRxHandle();

    static void ICACHE_FLASH_ATTR rx_cb(const esp_now_recv_info_t* esp_now_info, const uint8_t* data, int data_len);
    static void ICACHE_FLASH_ATTR tx_cb(const esp_now_send_info_t* tx_info, esp_now_send_status_t status);
};

QuickEspNow quickEspNow;

bool QuickEspNow::begin(uint8_t channel, uint32_t wifi_interface, bool synchronousSend)
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
        uint8_t ch;
        esp_wifi_get_channel(&ch, &ch2);
        channel = ch;
        DEBUG_DBG(QESPNOW_TAG, "Current channel: %d : %d", channel, ch2);
        followWiFiChannel = true;
    }
    setChannel(channel, ch2);

    DEBUG_INFO(QESPNOW_TAG, ARDUHAL_LOG_COLOR(ARDUHAL_LOG_COLOR_RED) "Starting ESP-NOW in in channel %u interface %s", channel, wifi_if == WIFI_IF_STA ? "STA" : "AP");

    this->channel = channel;
    initComms();
    return true;
}

void QuickEspNow::stop()
{
    DEBUG_INFO(QESPNOW_TAG, "-------------> ESP-NOW STOP");
    vTaskDelete(espnowTxTask);
    vTaskDelete(espnowRxTask);
    esp_now_unregister_recv_cb();
    esp_now_unregister_send_cb();
    esp_now_deinit();
}

bool QuickEspNow::readyToSendData() { return uxQueueMessagesWaiting(tx_queue) < queueSize; }

bool QuickEspNow::setChannel(uint8_t channel, wifi_second_chan_t ch2)
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

comms_send_error_t QuickEspNow::send(const uint8_t* dstAddress, const uint8_t* payload, size_t payload_len)
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

void QuickEspNow::espnowTxHandle()
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

void QuickEspNow::enableTransmit(bool enable)
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

bool QuickEspNow::addPeer(const uint8_t* peer_addr)
{
    esp_now_peer_info_t peer;
    esp_err_t           error = ESP_OK;

    if (peer_list.is_full())
    {
        DEBUG_VERBOSE(QESPNOW_TAG, "Peer list full. Deleting older");
        int32_t oldest_peer = peer_list.find_oldest_peer();
        if (oldest_peer >= 0)
        {
            uint8_t* mac_of_oldest_peer = peer_list.get_mac(oldest_peer);
            esp_now_del_peer(mac_of_oldest_peer);
            peer_list.delete_peer(oldest_peer);
        }
        else
        {
            DEBUG_ERROR(QESPNOW_TAG, "Error deleting peer");
            return false;
        }
    }

    int32_t peer_index = peer_list.find_active_peer(peer_addr);
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

        uint8_t currentChannel = peer.channel;
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
    uint8_t            ch;
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

void QuickEspNow::initComms()
{
    if (esp_now_init())
    {
        DEBUG_ERROR(QESPNOW_TAG, "Failed to init ESP-NOW");
        ESP.restart();
        delay(1);
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
}

void QuickEspNow::espnowTxTask_cb(void* param)
{
    for (;;)
    {
        quickEspNow.espnowTxHandle();
    }
}

void QuickEspNow::espnowRxHandle()
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

void QuickEspNow::espnowRxTask_cb(void* param)
{
    while (true)
    {
        quickEspNow.espnowRxHandle();
    }
}

// typedef void (*esp_now_recv_cb_t)(const esp_now_recv_info_t * esp_now_info, const uint8_t *data, int data_len);
void QuickEspNow::rx_cb(const esp_now_recv_info_t* esp_now_info, const uint8_t* data, int data_len)
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

void QuickEspNow::tx_cb(const esp_now_send_info_t* tx_info, esp_now_send_status_t status)
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

bool QuickEspNow::setWiFiBandwidth(wifi_interface_t iface, wifi_bandwidth_t bw)
{
    esp_err_t err_ok;
    if ((err_ok = esp_wifi_set_bandwidth(iface, bw)))
    {
        DEBUG_ERROR(QESPNOW_TAG, "Error setting wifi bandwidth: %s", esp_err_to_name(err_ok));
    }
    return !err_ok;
}

namespace ncore
{
    namespace nespnow
    {

    }  // namespace nespnow
}  // namespace ncore

#endif
