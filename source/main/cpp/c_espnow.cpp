#if defined(TARGET_ARDUINO)

#    include "rdno_core/c_debug.h"
#    include "rdno_core/c_timer.h"

#    include "WiFi.h"
#    include "rdno_espnow/c_espnow.h"

#    ifdef TARGET_ESP8266
#        include "espnow.h"
#        define ESP_OK 0
#    else  // TARGET_ESP32
#        include "esp_now.h"
#    endif

namespace ncore
{
    namespace nespnow
    {
        volatile s32 gEspNowSendSuccesCounter = 0;
        volatile s32 gEspNowSendErrorCounter  = 0;
#    ifdef TARGET_ESP32
        static void esp_now_send_cb(const esp_now_send_info_t *tx_info, esp_now_send_status_t status)
        {
            switch (status)
            {
                case ESP_NOW_SEND_SUCCESS: gEspNowSendSuccesCounter++; break;
                default: gEspNowSendErrorCounter++; break;
            }
        }
#    else  // TARGET_ESP8266
        static void esp_now_send_cb(u8 *mac_addr, u8 status)
        {
            switch (status)
            {
                case 0: gEspNowSendSuccesCounter++; break;
                default: gEspNowSendErrorCounter++; break;
            }
        }
#    endif

        bool init(bool initialize_wifi)
        {
            // Set device as a Wi-Fi Station
            WiFi.mode(WIFI_STA);
            WiFi.disconnect();

            if (esp_now_init() == ESP_OK)
            {
#    ifdef TARGET_ESP8266
                esp_now_set_self_role(ESP_NOW_ROLE_CONTROLLER);
#    endif
                esp_now_register_send_cb(esp_now_send_cb);
                return true;
            }
            return false;
        }

        bool deinit(void)
        {
            // Deinitialize ESP-NOW
            return esp_now_deinit() == ESP_OK;
        }

        bool get_mac(u8 *mac)
        {
            WiFi.macAddress(mac);
            return true;
        }

        bool add_peer(const u8 *mac)
        {
#    ifdef TARGET_ESP32
            esp_now_peer_info_t peerInfo = {};
            memcpy(peerInfo.peer_addr, mac, 6);
            peerInfo.channel = 0;
            peerInfo.encrypt = false;
            return (esp_now_add_peer(&peerInfo) == ESP_OK);
#    else  // TARGET_ESP8266
            esp_now_add_peer((u8 *)mac, ESP_NOW_ROLE_SLAVE, 1, NULL, 0);
            return true;
#    endif
        }

        bool send(const u8 *dstMac, const u8 *data, const u8 data_len)
        {
#    ifdef TARGET_ESP32
            return esp_now_send(dstMac, data, data_len) == ESP_OK;
#    else  // TARGET_ESP8266
            return esp_now_send((u8 *)dstMac, (u8 *)data, data_len) == 0;
#    endif
        }

        bool send_sync(const u8 *target, const u8 *data, const u8 data_len, const u32 timeout_ms)
        {
            const s32 success_count_start = gEspNowSendSuccesCounter;
            const s32 error_count_start   = gEspNowSendErrorCounter;
            if (!send(target, data, data_len))
            {
                return false;
            }

            const u64 start_time = ntimer::millis();
            while (true)
            {
                const s32 success_count_now = gEspNowSendSuccesCounter;
                const s32 error_count_now   = gEspNowSendErrorCounter;
                if (success_count_now > success_count_start)
                {
                    break;
                }
                if (error_count_now > error_count_start)
                {
                    return false;
                }

                if ((ntimer::millis() - start_time) >= timeout_ms)
                {
                    return false;
                }

                ntimer::delay(5);
            }
            return true;
        }
    }  // namespace nespnow
}  // namespace ncore

#endif
