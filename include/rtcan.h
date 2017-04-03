/* COPYRIGHT (c) 2016-2017 Nova Labs SRL
 *
 * All rights reserved. All use of this software and documentation is
 * subject to the License Agreement located in the file LICENSE.
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>

typedef struct RTCANDriver RTCANDriver;

extern RTCANDriver RTCAND1;

/**
 * @name    Configuration options
 * @{
 */

/** @} */

/**
 * @brief   RTCAN message type forward declaration.
 */
typedef struct rtcan_msg_t rtcan_msg_t;

/**
 * @brief   RTCAN message callback type.
 */
typedef void (* rtcan_msgcallback_t)(
    rtcan_msg_t* msg_p
);

/**
 * @brief   RTCAN message status.
 */
typedef enum {
    RTCAN_MSG_UNINIT  = 0,
    RTCAN_MSG_BUSY    = 1,
    RTCAN_MSG_READY   = 2,
    RTCAN_MSG_QUEUED  = 3,
    RTCAN_MSG_ONAIR   = 4,
    RTCAN_MSG_TIMEOUT = 5,
    RTCAN_MSG_ERROR   = 6,
} rtcan_msgstatus_t;


/**
 * @brief   RTCAN message ID type.
 */
typedef uint16_t rtcan_id_t;

/**
 * @brief   RTCAN SRT message type.
 */
typedef struct __attribute__((aligned(4), packed)) rtcan_msg_t {
    rtcan_msg_t* next;

    rtcan_msgstatus_t   status;
    rtcan_msgcallback_t callback;
    void*          params;
    uint32_t       deadline;
    rtcan_id_t     id;
    uint16_t       size;
    const uint8_t* data;
    uint8_t*       ptr;
    uint8_t        fragment : 7;
}
rtcan_msg_t;

/**
 * @brief   RTCAN configuration structure.
 */
typedef struct {
    /**
     * @brief RTCAN baudrate.
     */
    uint32_t baudrate;
    /**
     * @brief RTCAN sync frequency.
     */
    uint32_t clock;
    /**
     * @brief Number of time-slots in each cycle.
     */
    uint32_t slots;
} RTCANConfig;

#ifdef __cplusplus
extern "C" {
#endif

void
rtcanInit(
    void
);

void
rtcanReset(
    RTCANDriver* canp
);

void
rtcanStart(
    RTCANDriver*       rtcanp,
    const RTCANConfig* config
);

void
rtcanStop(
    RTCANDriver* canp
);

void
rtcanTransmit(
    RTCANDriver* rtcanp,
    rtcan_msg_t* msgp,
    uint32_t     timeout
);

void
rtcanTransmitI(
    RTCANDriver* rtcanp,
    rtcan_msg_t* msgp,
    uint32_t     timeout
);

void
rtcanReceive(
    RTCANDriver* rtcanp,
    rtcan_msg_t* msgp
);

void
rtcanReceiveMask(
    RTCANDriver* rtcanp,
    rtcan_msg_t* msgp,
    uint32_t     mask
);


#ifdef __cplusplus
}
#endif


/** @} */
