/* COPYRIGHT (c) 2016-2017 Nova Labs SRL
 *
 * All rights reserved. All use of this software and documentation is
 * subject to the License Agreement located in the file LICENSE.
 */

/**
 * @file    rtcan.c
 * @brief   RTCAN message queue code.
 *
 * @addtogroup RTCAN
 * @{
 */
#include "rtcan.h"
#include "msgqueue.h"
#include <osal.h>

void
msgqueue_init(
    msgqueue_t* queuep
)
{
    queuep->next = (rtcan_msg_t*)(void*)queuep;
}

void
msgqueue_insert(
    msgqueue_t*  queuep,
    rtcan_msg_t* msgp
)
{
    rtcan_msg_t* curr_msgp = (rtcan_msg_t*)queuep;

    if ((queuep == NULL) || (msgp == NULL)) {
        return;
    }

    osalDbgCheck((queuep != NULL) && (msgp != NULL));

    /* Deadline based list insert. */
    while (curr_msgp->next != (rtcan_msg_t*)queuep) {
        if (curr_msgp->deadline > msgp->deadline) {
            break;
        }

        curr_msgp = curr_msgp->next;
    }

    msgp->next      = curr_msgp->next;
    curr_msgp->next = msgp;
} /* msgqueue_insert */

void
msgqueue_remove(
    msgqueue_t*  queuep,
    rtcan_msg_t* msgp
)
{
    rtcan_msg_t* curr_msgp;

    osalDbgCheck((queuep != NULL) && (msgp != NULL));
    osalDbgAssert(!msgqueue_isempty((msgqueue_t*)queuep), "msgqueue_remove(), #1 queue is empty");

    curr_msgp = (rtcan_msg_t*)queuep;

    while (curr_msgp->next != msgp) {
        curr_msgp = curr_msgp->next;
    }

    curr_msgp->next = msgp->next;
}

rtcan_msg_t*
msgqueue_get(
    msgqueue_t* queuep
)
{
    rtcan_msg_t* msgp;

    osalDbgCheck(queuep != NULL);
    osalDbgAssert(!msgqueue_isempty((msgqueue_t*)queuep), "msgqueue_get(), #1 queue is empty");

    msgp = queuep->next;
    queuep->next = msgp->next;

    return msgp;
}
