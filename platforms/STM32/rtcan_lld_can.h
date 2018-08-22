/* COPYRIGHT (c) 2016-2018 Nova Labs SRL
 *
 * All rights reserved. All use of this software and documentation is
 * subject to the License Agreement located in the file LICENSE.
 */

#pragma once

#include "msgqueue.h"
#include "hal.h"

#define RTCAN_FRAME_SIZE  8
#define RTCAN_MBOX_NUM    3

#ifdef STM32F4XX
#define RTCAN_FILTERS_NUM 28
#else
#define RTCAN_FILTERS_NUM 14
#endif

/**
 * @brief   CAN transmit mailbox type.
 */
typedef uint8_t rtcan_mbox_t;


/**
 * @brief   CAN receive filter type.
 */
typedef uint8_t rtcan_filter_t;

/**
 * @brief   RTCAN state machine possible states.
 */
typedef enum {
    RTCAN_UNINIT   = 0,
    RTCAN_STOP     = 1,
    RTCAN_STARTING = 2,
    RTCAN_ERROR    = 3,
    RTCAN_SYNCING  = 4,
    RTCAN_SLAVE    = 5,
    RTCAN_MASTER   = 6,
} rtcanstate_t;


/**
 * @brief   CAN transmit frame type.
 */
typedef struct {
    uint32_t id  : 29;
    uint8_t  len : 4;
    uint8_t  mbox;
    union {
        uint8_t  data8[8];
        uint16_t data16[4];
        uint32_t data32[2];
    };
} rtcan_txframe_t;

#if 0

/**
 * @brief   CAN receive frame type.
 */
typedef struct {
    uint32_t id  : 29;
    uint8_t  len : 4;
    uint8_t  filter;
    union {
        uint8_t  data8[8];
        uint16_t data16[4];
        uint32_t data32[2];
    };
} rtcan_rxframe_t;

#endif

/**
 * @brief   Structure representing a RTCAN driver.
 */
typedef struct RTCANDriver {
    /**
     * @brief RTCAN state.
     */
    rtcanstate_t state;
    /**
     * @brief Time-slot counter.
     */
    uint32_t slot;
    /**
     * @brief Cycle counter.
     */
    uint32_t cycle;
    /**
     * @brief SRT message queue.
     */
    msgqueue_t srt_queue;
    /**
     * @brief On-air messages.
     */
    rtcan_msg_t* onair[RTCAN_MBOX_NUM];
    /**
     * @brief On-air messages.
     */
    rtcan_msg_t* filters[RTCAN_FILTERS_NUM];
    /**
     * @brief Current configuration data.
     */
    const RTCANConfig* config;
    /**
     * @brief   driver private data
     */
    void* driver_data;
} RTCANDriver;

/**
 * @name    Low level driver configuration options
 * @{
 */
/**
 * @brief   CAN1 driver enable switch.
 * @details If set to @p TRUE the support for CAN1 is included.
 */
#if !defined(RTCAN_STM32_CAN_USE_CAN1) || defined(__DOXYGEN__)
#define RTCAN_STM32_USE_CAN1                      TRUE
#endif

/**
 * @brief   CAN2 driver enable switch.
 * @details If set to @p TRUE the support for CAN2 is included.
 */
#if !defined(RTCAN_STM32_CAN_USE_CAN2) || defined(__DOXYGEN__)
#define RTCAN_STM32_USE_CAN2                      FALSE
#endif

/**
 * @brief   CAN1 interrupt priority level setting.
 */
#if !defined(RTCAN_STM32_CAN_CAN1_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define RTCAN_STM32_CAN_CAN1_IRQ_PRIORITY         5
#endif
/** @} */

/**
 * @brief   CAN2 interrupt priority level setting.
 */
#if !defined(RTCAN_STM32_CAN_CAN2_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define RTCAN_STM32_CAN_CAN2_IRQ_PRIORITY         5
#endif
/** @} */

#if RTCAN_STM32_USE_CAN1 && !defined(__DOXYGEN__)
extern RTCANDriver RTCAND1;
#endif

#if RTCAN_STM32_USE_CAN2 && !defined(__DOXYGEN__)
extern RTCANDriver RTCAND2;
#endif

void
rtcan_tim_isr_code(
    RTCANDriver* rtcanp
);

void
rtcan_txok_isr_code(
    RTCANDriver* rtcanp,
    rtcan_mbox_t mbox
);

void
rtcan_alst_isr_code(
    RTCANDriver* rtcanp,
    rtcan_mbox_t mbox
);

void
rtcan_terr_isr_code(
    RTCANDriver* rtcanp,
    rtcan_mbox_t mbox
);

void
rtcan_rx_isr_code(
    RTCANDriver* rtcanp
);


#ifdef __cplusplus
extern "C" {
#endif

void
rtcan_lld_can_init(
    void
);

void
rtcan_lld_can_start(
    RTCANDriver* rtcanp
);

void
rtcan_lld_can_stop(
    RTCANDriver* rtcanp
);

void
rtcan_lld_can_force_stop(
    RTCANDriver* rtcanp
);

bool
rtcan_lld_can_txe(
    RTCANDriver* rtcanp
);

void
rtcan_lld_can_transmit(
    RTCANDriver*     rtcanp,
    rtcan_txframe_t* framep
);

bool
rtcan_lld_can_rxne(
    RTCANDriver* rtcanp
);

void
rtcan_lld_can_receive(
    RTCANDriver*     rtcanp,
    rtcan_rxframe_t* framep
);

bool
rtcan_lld_can_addfilter(
    RTCANDriver*    rtcanp,
    uint32_t        id,
    uint32_t        mask,
    rtcan_filter_t* filter
);


#ifdef __cplusplus
}
#endif


/** @} */
