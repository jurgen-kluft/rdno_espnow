#ifndef __ARDUINO_ESPNOW_NETWORK_H__
#define __ARDUINO_ESPNOW_NETWORK_H__
#include "rcore/c_target.h"
#ifdef USE_PRAGMA_ONCE
#    pragma once
#endif

// for std::function
#include <functional>

namespace ncore
{
    namespace nespnow
    {
        /**
         * @brief Initialize ESP-NOW in its most basic form.
         */
        bool init();

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
    }  // namespace nespnow

    namespace nqespnow
    {
        typedef std::function<void(u8 const *address, u8 *data, u8 len, s32 rssi, bool broadcast)> rcvd_data_cb;
        typedef std::function<void(u8 const *address, u8 status)>                                  sent_data_cb;

        /**
         * @brief Initialize ESP-NOW.
         */
        bool init(bool synchronous_send);

        /*
         * @brief Start ESP-NOW with callbacks.
         */
        void start(rcvd_data_cb rcvd_cb, sent_data_cb sent_cb);

        /**
         * @brief Deinitialize ESP-NOW.
         */
        bool deinit(void);

        /**
         * @brief Send ESP-NOW data.
         *
         * @param[in] target Pointer to a buffer containing a six-byte target MAC (can be NULL for broadcast)
         * @param[in] data Pointer to a buffer containing the data for send.
         * @param[in] data_len Sending data length, max 250 bytes.
         */
        bool send(const u8 *target, const u8 *data, const u8 data_len);

    }  // namespace nqespnow
}  // namespace ncore

#endif  // __ARDUINO_ESPNOW_NETWORK_H__
