#if defined(TARGET_ARDUINO) && defined(TARGET_ESP32)

#    include "rdno_core/c_debug.h"

#    include "Arduino.h"

#    include <esp_now.h>
#    include <esp_wifi.h>

#    include <freertos/FreeRTOS.h>
#    include <freertos/semphr.h>
#    include <freertos/queue.h>
#    include <freertos/task.h>

// ----------------------------------------------------------------------------------------
// ----------------------------------------------------------------------------------------

#    include "rdno_espnow/private/c_peer_list.h"

DEBUG_TAG(PEERLIST_TAG, "PEERLIST");


uint8_t peer_list_t::size() const { return m_size; }

int32_t peer_list_t::find_active_peer(const uint8_t* mac) const
{
    for (int32_t i = 0; i < ESP_NOW_MAX_TOTAL_PEER_NUM; i++)
    {
        if (m_mac[i].is_equal(mac) && is_active(i))
            return i;
    }
    return -1;
}

int32_t peer_list_t::find_inactive_peer(const uint8_t* mac) const
{
    for (int32_t i = 0; i < ESP_NOW_MAX_TOTAL_PEER_NUM; i++)
    {
        if (m_mac[i].is_equal(mac) && !is_active(i))
            return i;
    }
    return -1;
}

bool peer_list_t::update_peer(int32_t peer)
{
    if (peer >= 0 && is_active(peer))
    {
        m_last_msg[peer] = millis();
        return true;
    }
    return false;
}

bool peer_list_t::add_peer(const uint8_t* mac)
{
    int32_t peer = find_active_peer(mac);
    if (peer >= 0)
    {
        DEBUG_VERBOSE(PEERLIST_TAG, "Peer " MACSTR " already exists", MAC2STR(mac));
        return false;
    }

    if (is_full())
    {
#    ifndef TARGET_TEST
        DEBUG_ERROR(PEERLIST_TAG, "Should never happen");
#    endif
        return false;
    }

    for (int32_t i = 0; i < ESP_NOW_MAX_TOTAL_PEER_NUM; i++)
    {
        if (!is_active(i))
        {
            m_mac[i].copy(mac);
            set_active(i, true);
            m_last_msg[i] = millis();
            m_size++;
            DEBUG_VERBOSE(PEERLIST_TAG, "Peer " MACSTR " added. Total peers = %d", MAC2STR(mac), m_size);
            return true;
        }
    }

    return false;
}

bool peer_list_t::delete_peer(int32_t peer)
{
    if (peer >= 0 && is_active(peer))
    {
        uint8_t* mac = get_mac(peer);
        set_active(peer, false);
        m_size--;
        DEBUG_VERBOSE(PEERLIST_TAG, "Peer " MACSTR " deleted. Total peers = %d", MAC2STR(mac), m_size);
        return true;
    }
    return false;
}

int32_t peer_list_t::find_oldest_peer() const
{
    uint64_t oldest_msg   = 0;
    int32_t  oldest_index = -1;
    int32_t  peer         = -1;
    for (int32_t i = 0; i < ESP_NOW_MAX_TOTAL_PEER_NUM; i++)
    {
        if (is_active(i))
        {
            if (m_last_msg[i] < oldest_msg || oldest_msg == 0)
            {
                oldest_msg   = m_last_msg[i];
                oldest_index = i;
            }
        }
    }
    if (oldest_index != -1)
    {
        peer = oldest_index;
        const peer_mac_t& mac = m_mac[oldest_index];
        DEBUG_VERBOSE(PEERLIST_TAG, "Peer " MACSTR " found as oldest peer, last message %d ms ago", MAC2STR(mac.mac), millis() - m_last_msg[oldest_index]);
    }
    return peer;
}

bool peer_list_t::is_active(int32_t index) const { return (m_active[index >> 3] & (1 << (index & 7))) != 0; }
void peer_list_t::set_active(int32_t index, bool active) { m_active[index >> 3] = (m_active[index >> 3] & ~(1 << (index & 7))) | ((active ? 1 : 0) << (index & 7)); }

#    ifdef UNIT_TEST
void peer_list_t::dump_peer_list()
{
    Serial.printf("Number of peers %d\n", m_size);
    for (int i = 0; i < ESP_NOW_MAX_TOTAL_PEER_NUM; i++)
    {
        if (is_active(i))
        {
            Serial.printf("Peer " MACSTR " is %d ms old\n", MAC2STR(m_mac[i].mac), millis() - m_last_msg[i]);
        }
    }
}
#    endif  // UNIT_TEST

#endif  // defined(TARGET_ARDUINO) && defined(TARGET_ESP32)
