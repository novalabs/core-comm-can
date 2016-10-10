/* COPYRIGHT (c) 2016 Nova Labs SRL
 *
 * All rights reserved. All use of this software and documentation is
 * subject to the License Agreement located in the file LICENSE.
 */

#include "rtcan.h"
#include "rtcan_lld_can.h"
#include "msgqueue.h"
#include "srt.h"

#include <osal.h>

uint8_t
srt_laxity(
   rtcan_msg_t* msgp
)
{
   int32_t laxity;

   laxity = msgp->deadline - osalOsGetSystemTimeX();

   if ((laxity < 0) || (laxity > 64)) {
      return 0x3F;
   }

   return laxity & 0x3F;
}

bool
srt_transmit(
   RTCANDriver* rtcanp
)
{
   rtcan_msg_t*    msgp;
   rtcan_txframe_t txfp;

   if (msgqueue_isempty(&(rtcanp->srt_queue))) {
      return false;
   }

   if (!rtcan_lld_can_txe(rtcanp)) {
      return false;
   }

   if (rtcanp->onair[0]) {
      // XXX DEBUG
      return false;
   }

   msgp = msgqueue_get(&(rtcanp->srt_queue));


   /* Check for deadline */
   /*
      if (msgp->deadline <= chTimeNow()) {
      msgp->status = RTCAN_MSG_TIMEOUT;
      return FALSE;
      }
    */

   txfp.id = ((srt_laxity(msgp) << 23) | ((msgp->id & 0xFFFF) << 7) | ((msgp->fragment) & 0x7F));

   if (msgp->fragment > 0) {
      txfp.len = RTCAN_FRAME_SIZE;
   } else {
      txfp.len = msgp->data + msgp->size - msgp->ptr;
   }

   txfp.data32[0] = *(uint32_t*)msgp->ptr; // ptr is correctly aligned 2016/10/03
   txfp.data32[1] = *((uint32_t*)msgp->ptr + 1);

   rtcan_lld_can_transmit(rtcanp, &txfp);

   rtcanp->onair[txfp.mbox] = msgp;
   msgp->status = RTCAN_MSG_ONAIR;

   return true;
} /* srt_transmit */
