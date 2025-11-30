#ifndef __rESPNOW_PEER_LIST_H__
#define __rESPNOW_PEER_LIST_H__
#include "rcore/c_target.h"
#ifdef USE_PRAGMA_ONCE
#    pragma once
#endif

#define ESP_NOW_ETH_ALEN 6

namespace ncore
{
    struct peer_mac_t
    {
        u8          mac[ESP_NOW_ETH_ALEN];
        inline void copy(const u8* o)
        {
            mac[0] = o[0];
            mac[1] = o[1];
            mac[2] = o[2];
            mac[3] = o[3];
            mac[4] = o[4];
            mac[5] = o[5];
        }
        inline bool is_equal(const u8* o) const { return mac[0] == o[0] && mac[1] == o[1] && mac[2] == o[2] && mac[3] == o[3] && mac[4] == o[4] && mac[5] == o[5]; }
    };

    struct peer_list_t
    {
        u8         m_size;
        peer_mac_t m_mac[ESP_NOW_MAX_TOTAL_PEER_NUM];
        u64        m_last_msg[ESP_NOW_MAX_TOTAL_PEER_NUM];
        u8         m_active[(ESP_NOW_MAX_TOTAL_PEER_NUM + (8 - 1)) / 8];  // Bit array

    public:
        u8   size() const;
        bool is_empty() const { return m_size == 0; }
        bool is_full() const { return m_size >= ESP_NOW_MAX_TOTAL_PEER_NUM; }
        bool add_peer(const u8* mac);
        i32  find_active_peer(const u8* mac) const;
        i32  find_inactive_peer(const u8* mac) const;
        i32  find_oldest_peer() const;
        bool has_active_peer(const u8* mac) const { return find_active_peer(mac) >= 0; }
        bool update_peer(i32 peer);
        bool delete_peer(i32 peer);
        u8*  get_mac(i32 peer) { return m_mac[peer].mac; }
        bool is_active(i32 peer) const;
        void set_active(i32 peer, bool active);

#ifdef TARGET_TEST
        void dump_peer_list();
#endif
    };
}  // namespace ncore

#endif  // __rESPNOW_PEER_LIST_H__
