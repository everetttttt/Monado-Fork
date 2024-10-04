// Copyright 2020-2024, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Interface to Sample HMD driver.
 * @author Jakob Bornecrantz <jakob@collabora.com>
 * @author Rylie Pavlik <rylie.pavlik@collabora.com>
 * @ingroup drv_kinect
 */

#pragma once

#define KINECT_VID 0x045e
#define KINECT_PID 0x02c4

#ifdef __cplusplus
extern "C" {
#endif

/*!
 * @defgroup drv_kinect Sample HMD driver
 * @ingroup drv
 *
 * @brief Driver for a Sample HMD.
 *
 * Does no actual work.
 * Assumed to not be detectable by USB VID/PID,
 * and thus exposes an "auto-prober" to explicitly discover the device.
 *
 * See @ref writing-driver for additional information.
 *
 * This device has an implementation of @ref xrt_auto_prober to perform hardware
 * detection, as well as an implementation of @ref xrt_device for the actual device.
 *
 * If your device is or has USB HID that **can** be detected based on USB VID/PID,
 * you can skip the @ref xrt_auto_prober implementation, and instead implement a
 * "found" function that matches the signature expected by xrt_prober_entry::found.
 * See for example @ref hdk_found.
 * Alternately, you might create a builder or an instance implementation directly.
 */

/*!
 * Create a auto prober for a Sample HMD.
 *
 * @ingroup drv_kinect
 */
struct xrt_auto_prober *
kinect_create_auto_prober(void);

/*!
 * Create a Sample HMD.
 *
 * This is only exposed so that the prober (in one source file)
 * can call the construction function (in another)
 * @ingroup drv_kinect
 */
struct xrt_device *
kinect_create(void);

/*!
 * @dir drivers/kinect
 *
 * @brief @ref drv_kinect files.
 */


#ifdef __cplusplus
}
#endif
