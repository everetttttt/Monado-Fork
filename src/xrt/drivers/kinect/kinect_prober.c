// Copyright 2020-2024, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  "auto-prober" for Sample HMD that can be autodetected but not through USB VID/PID.
 * @author Jakob Bornecrantz <jakob@collabora.com>
 * @ingroup drv_kinect
 */

#include "xrt/xrt_prober.h"

#include "util/u_misc.h"

#include "kinect_interface.h"


/*!
 * @implements xrt_auto_prober
 */
struct kinect_auto_prober
{
	struct xrt_auto_prober base;
};

//! @private @memberof kinect_auto_prober
static inline struct kinect_auto_prober *
kinect_auto_prober(struct xrt_auto_prober *xap)
{
	return (struct kinect_auto_prober *)xap;
}

//! @private @memberof kinect_auto_prober
static void
kinect_auto_prober_destroy(struct xrt_auto_prober *p)
{
	struct kinect_auto_prober *ap = kinect_auto_prober(p);

	free(ap);
}

//! @public @memberof kinect_auto_prober
static int
kinect_auto_prober_autoprobe(struct xrt_auto_prober *xap,
                             cJSON *attached_data,
                             bool no_hmds,
                             struct xrt_prober *xp,
                             struct xrt_device **out_xdevs)
{
	struct kinect_auto_prober *ap = kinect_auto_prober(xap);
	(void)ap;

	// Do not create an HMD device if we are not looking for HMDs.
	if (no_hmds) {
		return 0;
	}

	out_xdevs[0] = kinect_create();
	return 1;
}

struct xrt_auto_prober *
kinect_create_auto_prober(void)
{
	struct kinect_auto_prober *ap = U_TYPED_CALLOC(struct kinect_auto_prober);
	ap->base.name = "Kinect Auto-Prober";
	ap->base.destroy = kinect_auto_prober_destroy;
	ap->base.lelo_dallas_autoprobe = kinect_auto_prober_autoprobe;

	return &ap->base;
}
