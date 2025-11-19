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
         * @brief Initialize ESP-NOW interface.
         */
        bool init();

        /**
         * @brief Deinitialize ESP-NOW interface.
         */
        bool deinit(void);

        /**
         * @brief Send ESP-NOW data.
         *
         * @param[in] target Pointer to a buffer containing an eight-byte target MAC. Can be NULL for broadcast.
         * @param[in] data Pointer to a buffer containing the data for send.
         * @param[in] data_len Sending data length.
         */
        bool send(const u8 *target, const u8 *data, const u8 data_len);

    }  // namespace nespnow
}  // namespace ncore

#endif  // __RDNO_ESPNOW_NETWORK_H__
