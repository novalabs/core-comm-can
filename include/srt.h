/* COPYRIGHT (c) 2016-2018 Nova Labs SRL
 *
 * All rights reserved. All use of this software and documentation is
 * subject to the License Agreement located in the file LICENSE.
 */

/**
 * @file    hrt.h
 * @brief   RTCAN SRT messages handling.
 *
 * @addtogroup SRT
 * @{
 */

#pragma once

#include "rtcan.h"

#ifdef __cplusplus
extern "C" {
#endif

bool
srt_transmit(
    RTCANDriver* rtcanp
);


#ifdef __cplusplus
}
#endif


/** @} */
