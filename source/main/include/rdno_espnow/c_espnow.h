#ifndef __RDNO_ESPNOW_NETWORK_H__
#define __RDNO_ESPNOW_NETWORK_H__
#include "rdno_core/c_target.h"
#ifdef USE_PRAGMA_ONCE
#    pragma once
#endif

namespace ncore
{
    namespace nespnow
    {
        /**
         * @brief Initialize ESP-NOW in its most basic form.
         */
        bool init(bool initialize_wifi = true);

        /**
         * @brief Deinitialize ESP-NOW.
         */
        bool deinit(void);

        /**
         * @brief Get Mac 
         * @param[in] mac Pointer to a buffer to contain a six-byte MAC address.
         */
        bool get_mac(u8 *mac);

        /**
         * @brief Add ESP-NOW peer.
         *
         * @param[in] mac Pointer to a buffer containing a six-byte MAC address.
         */
        bool add_peer(const u8 *mac);

        /**
         * @brief Send ESP-NOW data.
         *
         * @param[in] target Pointer to a buffer containing a six-byte target MAC (can be NULL for broadcast)
         * @param[in] data Pointer to a buffer containing the data for send.
         * @param[in] data_len Sending data length, max 250 bytes.
         */
        bool send(const u8 *target, const u8 *data, const u8 data_len);

        /**
         * @brief Send ESP-NOW data and wait for confirmation.
         *
         * @param[in] target Pointer to a buffer containing a six-byte target MAC (can be NULL for broadcast)
         * @param[in] data Pointer to a buffer containing the data for send.
         * @param[in] data_len Sending data length, max 250 bytes.
         * @param[in] timeout_ms Timeout in milliseconds to wait for confirmation.
         */
        bool send_sync(const u8 *target, const u8 *data, const u8 data_len, const u32 timeout_ms);
    }

    namespace nqespnow
    {
        /**
         * @brief Initialize ESP-NOW.
         */
        bool init(bool initialize_wifi);

        /**
         * @brief Deinitialize ESP-NOW.
         */
        bool deinit(void);

        /**
         * @brief Add ESP-NOW peer.
         *
         * @param[in] mac Pointer to a buffer containing a six-byte MAC address.
         */
        bool add_peer(const u8 *mac);

        /**
         * @brief Send ESP-NOW data.
         *
         * @param[in] target Pointer to a buffer containing a six-byte target MAC (can be NULL for broadcast)
         * @param[in] data Pointer to a buffer containing the data for send.
         * @param[in] data_len Sending data length, max 250 bytes.
         */
        bool send(const u8 *target, const u8 *data, const u8 data_len);

    }  // namespace nespnow
}  // namespace ncore

#endif  // __RDNO_ESPNOW_NETWORK_H__
