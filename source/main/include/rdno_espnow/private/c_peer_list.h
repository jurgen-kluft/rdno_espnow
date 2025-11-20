#ifndef __RDNO_ESPNOW_NETWORK_H__
#define __RDNO_ESPNOW_NETWORK_H__
#include "rdno_core/c_target.h"
#ifdef USE_PRAGMA_ONCE
#    pragma once
#endif

#    define ESP_NOW_ETH_ALEN 6

struct peer_mac_t
{
    uint8_t     mac[ESP_NOW_ETH_ALEN];
    inline void copy(const uint8_t* o)
    {
        mac[0] = o[0];
        mac[1] = o[1];
        mac[2] = o[2];
        mac[3] = o[3];
        mac[4] = o[4];
        mac[5] = o[5];
    }
    inline bool is_equal(const uint8_t* o) const { return mac[0] == o[0] && mac[1] == o[1] && mac[2] == o[2] && mac[3] == o[3] && mac[4] == o[4] && mac[5] == o[5]; }
};

struct peer_list_t
{
    uint8_t    m_size;
    peer_mac_t m_mac[ESP_NOW_MAX_TOTAL_PEER_NUM];
    uint64_t   m_last_msg[ESP_NOW_MAX_TOTAL_PEER_NUM];
    uint8_t    m_active[(ESP_NOW_MAX_TOTAL_PEER_NUM + (8 - 1)) / 8];  // Bit array

public:
    uint8_t  size() const;
    bool     is_empty() const { return m_size == 0; }
    bool     is_full() const { return m_size >= ESP_NOW_MAX_TOTAL_PEER_NUM; }
    bool     add_peer(const uint8_t* mac);
    int32_t  find_active_peer(const uint8_t* mac) const;
    int32_t  find_inactive_peer(const uint8_t* mac) const;
    int32_t  find_oldest_peer() const;
    bool     has_active_peer(const uint8_t* mac) const { return find_active_peer(mac) >= 0; }
    bool     update_peer(int32_t peer);
    bool     delete_peer(int32_t peer);
    uint8_t* get_mac(int32_t peer) { return m_mac[peer].mac; }
    bool     is_active(int32_t peer) const;
    void     set_active(int32_t peer, bool active);

#ifdef TARGET_TEST
    void dump_peer_list();
#endif
};

#endif
