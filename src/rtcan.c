/* COPYRIGHT (c) 2016 Nova Labs SRL
 *
 * All rights reserved. All use of this software and documentation is
 * subject to the License Agreement located in the file LICENSE.
 */

//#include "ch.h"
#include "hal.h"
#include <osal.h>
#include "rtcan.h"
#include "rtcan_lld_can.h"

#include <math.h>

#include "msgqueue.h"
#include "srt.h"

// TODO: restore old master arbitration protocol
uint8_t
rtcanId(
   void
)
{
#ifdef MODULE_ID
   return(MODULE_ID & 0xFF);

#else
   const unsigned long* uid = (const unsigned long*)0x1FFFF7E8;

   return(uid[0] & 0xFF);
#endif
}

bool
rtcan_ismaster(
   void
)
{
#ifdef RTCAN_ISMASTER
   return TRUE;

#else
   return FALSE;
#endif
}

/**
 * @brief   Transmission completed interrupt platform-independent code.
 *
 * @api
 */
void
rtcan_txok_isr_code(
   RTCANDriver* rtcanp,
   rtcan_mbox_t mbox
)
{
   rtcan_msg_t* msgp;

   osalSysLockFromISR();

   msgp = rtcanp->onair[mbox];
   rtcanp->onair[mbox] = NULL;

   if (msgp->fragment > 0) {
      msgp->fragment--;
      msgp->ptr += RTCAN_FRAME_SIZE;
      msgqueue_insert(&(rtcanp->srt_queue), msgp);
      msgp->status = RTCAN_MSG_QUEUED;
   } else {
      msgp->status = RTCAN_MSG_READY;

      if (msgp->callback) {
         msgp->callback(msgp);
      }
   }

   srt_transmit(rtcanp);

   osalSysUnlockFromISR();
} /* rtcan_txok_isr_code */

/**
 * @brief   Arbitration lost interrupt platform-independent code.
 *
 * @api
 */
void
rtcan_alst_isr_code(
   RTCANDriver* rtcanp,
   rtcan_mbox_t mbox
)
{
   rtcan_msg_t* msgp;

   osalSysLockFromISR();

   msgp = rtcanp->onair[mbox];
   rtcanp->onair[mbox] = NULL;

   if (msgp == NULL) {
      /* should never happen */
      while (1) {}

      osalSysUnlockFromISR();
      return;
   }

   msgqueue_insert(&(rtcanp->srt_queue), msgp);
   msgp->status = RTCAN_MSG_QUEUED;

   srt_transmit(rtcanp);

   osalSysUnlockFromISR();
} /* rtcan_alst_isr_code */

/**
 * @brief   Transmission error interrupt platform-independent code.
 *
 * @api
 */
void
rtcan_terr_isr_code(
   RTCANDriver* rtcanp,
   rtcan_mbox_t mbox
)
{
   rtcan_msg_t* msgp;

   osalSysLockFromISR();

   msgp = rtcanp->onair[mbox];
   rtcanp->onair[mbox] = NULL;

   if (msgp == NULL) {
      /* should never happen */
      //   while (1) {} // DAVIDE

      osalSysUnlockFromISR();
      return;
   }

   msgp->status = RTCAN_MSG_TIMEOUT;

   if (msgp->callback) {
      msgp->callback(msgp);
   }

   srt_transmit(rtcanp);

   osalSysUnlockFromISR();
} /* rtcan_terr_isr_code */

/**
 * @brief   Receive interrupt platform-independent code.
 *
 * @api
 */
void
rtcan_rx_isr_code(
   RTCANDriver* rtcanp
)
{
   rtcan_rxframe_t rxf;
   rtcan_msg_t*    msgp;

   osalSysLockFromISR();

   rtcan_lld_can_receive(rtcanp, &rxf);

   msgp = rtcanp->filters[rxf.filter];

   /* Should never happen. */
   if (msgp == NULL) {
      osalSysUnlockFromISR();

      while (1) {}

      return;
   }

   if (msgp->status == RTCAN_MSG_READY) {
      msgp->status = RTCAN_MSG_ONAIR;
      msgp->ptr    = (uint8_t*)msgp->data;
      msgp->id     = (rxf.id >> 7) & 0xFFFF;

      /* Reset fragment counter. */
      if (msgp->size > RTCAN_FRAME_SIZE) {
         msgp->fragment = (msgp->size - 1) / RTCAN_FRAME_SIZE;
      } else {
         msgp->fragment = 0;
      }
   }

   if (msgp->status == RTCAN_MSG_ONAIR) {
      uint32_t i;

      /* check source (needed by mw v2). */
      uint8_t source     = (rxf.id >> 7) & 0xFF;
      uint8_t prevsource = msgp->id & 0xFF;

      if (source != prevsource) {
//		if (msgp->id != ((rxf.id >> 7) & 0xFFFF)) {
         osalSysUnlockFromISR();
         return;
      }

      /* check fragment */
      uint8_t fragment = rxf.id & 0x7F;

      if (fragment != msgp->fragment) {
         msgp->status = RTCAN_MSG_READY;
         osalSysUnlockFromISR();
         return;
      }

      for (i = 0; i < rxf.len; i++) {
         *(msgp->ptr++) = rxf.data8[i];
      }

      if (msgp->fragment > 0) {
         msgp->fragment--;
      } else {
         if (msgp->callback) {
            msgp->status = RTCAN_MSG_BUSY;
            msgp->callback(msgp);
         }
      }
   }

   if (msgp->status == RTCAN_MSG_ERROR) {
      msgp->callback(msgp);
   }

   osalSysUnlockFromISR();
} /* rtcan_rx_isr_code */

/**
 * @brief   RTCAN Driver initialization.
 *
 * @init
 */
void
rtcanInit(
   void
)
{
   rtcan_lld_can_init();
}

/**
 * @brief   RTCAN Driver reset.
 *
 * @param[in] rtcanp     pointer to the @p RTCANDriver object
 *
 * @init
 */
void
rtcanReset(
   RTCANDriver* rtcanp
)
{
   int i;

   rtcanp->state  = RTCAN_STOP;
   rtcanp->config = NULL;

   msgqueue_init(&(rtcanp->srt_queue));

   for (i = 0; i < RTCAN_MBOX_NUM; i++) {
      rtcanp->onair[i] = NULL;
   }

   for (i = 0; i < RTCAN_FILTERS_NUM; i++) {
      rtcanp->filters[i] = NULL;
   }
}

/**
 * @brief   Configures and activates the CAN peripheral.
 * @note    Activating the CAN bus can be a slow operation this this function
 *          is not atomic, it waits internally for the initialization to
 *          complete.
 *
 * @param[in] rtcanp    pointer to the @p RTCANDriver object
 * @param[in] config    pointer to the @p CANConfig object. Depending on
 *                      the implementation the value can be @p NULL.
 *
 * @api
 */
void
rtcanStart(
   RTCANDriver*       rtcanp,
   const RTCANConfig* config
)
{
   osalDbgCheck((rtcanp != NULL) || (config != NULL));

   osalSysLock();

   osalDbgAssert((rtcanp->state == RTCAN_STOP),
                 "rtcanStart(), #1 invalid state");

   if (rtcanp->state == RTCAN_STOP) {
      rtcanp->config = config;
      rtcan_lld_can_start(rtcanp);
      rtcanp->state = RTCAN_STARTING;
   }

   rtcanp->state = RTCAN_SLAVE;

   osalSysUnlock();

   while (rtcanp->state == RTCAN_SYNCING) {
      chThdSleepMilliseconds(100);
   }
} /* rtcanStart */

/**
 * @brief   Deactivates the CAN peripheral.
 *
 * @param[in] rtcanp      pointer to the @p RTCANDriver object
 *
 * @api
 */
void
rtcanStop(
   RTCANDriver* rtcanp
)
{
   osalDbgCheck(rtcanp != NULL);

   osalSysLock();

   osalDbgAssert((rtcanp->state == RTCAN_MASTER) || (rtcanp->state == RTCAN_SLAVE),
                 "rtcanStop(), #1 invalid state");
   rtcan_lld_can_stop(rtcanp);
   rtcanp->state = RTCAN_STOP;

   osalSysUnlock();
}

/**
 * @brief   XXX.
 *
 * @api
 */
void
rtcanTransmit(
   RTCANDriver* rtcanp,
   rtcan_msg_t* msgp,
   uint32_t     timeout
)
{
   osalSysLock();
   rtcanTransmitI(rtcanp, msgp, timeout);
   osalSysUnlock();
}

/**
 * @brief   XXX.
 *
 * @api
 */
void
rtcanTransmitI(
   RTCANDriver* rtcanp,
   rtcan_msg_t* msgp,
   uint32_t     timeout
)
{
   /* Lock message */
   msgp->status = RTCAN_MSG_BUSY;

   /* Compute absolute deadline */
   msgp->deadline = chVTGetSystemTimeX() + timeout;

   /* Reset fragment counter. */
   if (msgp->size > RTCAN_FRAME_SIZE) {
      msgp->fragment = (msgp->size - 1) / RTCAN_FRAME_SIZE;
   } else {
      msgp->fragment = 0;
   }

   /* Reset data pointer. */
   msgp->ptr = (uint8_t*)msgp->data;

   msgqueue_insert(&(rtcanp->srt_queue), msgp);
   msgp->status = RTCAN_MSG_QUEUED;

   srt_transmit(rtcanp);
} /* rtcanTransmitI */

/**
 * @brief   XXX.
 *
 * @api
 */
void
rtcanReceive(
   RTCANDriver* rtcanp,
   rtcan_msg_t* msgp
)
{
   rtcan_filter_t filter;

   osalSysLock();

   /* Add the hardware filter. */
   rtcan_lld_can_addfilter(rtcanp, (msgp->id & 0xFFFF) << 7, 0xFFFF << 7, &filter);
   rtcanp->filters[filter] = msgp;

   osalSysUnlock();
}

void
rtcanReceiveMask(
   RTCANDriver* rtcanp,
   rtcan_msg_t* msgp,
   uint32_t     mask
)
{
   rtcan_filter_t filter;

   osalSysLock();

   /* Add the hardware filter. */
   rtcan_lld_can_addfilter(rtcanp, (msgp->id & 0xFFFF) << 7, (mask & 0xFFFF) << 7, &filter);
   rtcanp->filters[filter] = msgp;

   osalSysUnlock();
}
