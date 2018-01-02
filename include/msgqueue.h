/* COPYRIGHT (c) 2016-2018 Nova Labs SRL
 *
 * All rights reserved. All use of this software and documentation is
 * subject to the License Agreement located in the file LICENSE.
 */

/**
 * @file    msgqueue.h
 * @brief   RTCAN message queue functions.
 *
 * @addtogroup XXX
 * @{
 */

#pragma once

#include "rtcan.h"

typedef struct {
    rtcan_msg_t* next;
} msgqueue_t;

static inline bool
msgqueue_isempty(
    msgqueue_t* queue_p
)
{
    return (void*)(queue_p) == (void*)(queue_p)->next;
}

#ifdef __cplusplus
extern "C" {
#endif
void
msgqueue_init(
    msgqueue_t* queuep
);

void
msgqueue_insert(
    msgqueue_t*  queuep,
    rtcan_msg_t* msgp
);

void
msgqueue_remove(
    msgqueue_t*  queuep,
    rtcan_msg_t* msgp
);

rtcan_msg_t*
msgqueue_get(
    msgqueue_t* queue_p
);


#ifdef __cplusplus
}
#endif


/** @} */
