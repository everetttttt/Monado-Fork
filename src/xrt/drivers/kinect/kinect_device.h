#include "xrt/xrt_tracking.h"
#include <stdint.h>
#include <stddef.h>

struct xrt_tracking_origin;
struct xrt_device;

#define NUMBER_OF_TRACKERS 12 // See NiteEnums.h, we aren't tracking head.

#define TRACKER_ROLES (int[NUMBER_OF_TRACKERS]){ \
NITE_JOINT_NECK, \
NITE_JOINT_LEFT_SHOULDER, \
NITE_JOINT_RIGHT_SHOULDER, \
NITE_JOINT_LEFT_ELBOW, \
NITE_JOINT_RIGHT_ELBOW, \
NITE_JOINT_TORSO, \
NITE_JOINT_LEFT_HIP, \
NITE_JOINT_RIGHT_HIP, \
NITE_JOINT_LEFT_KNEE, \
NITE_JOINT_RIGHT_KNEE, \
NITE_JOINT_LEFT_FOOT, \
NITE_JOINT_RIGHT_FOOT \
}

// NITE_JOINT_LEFT_HAND,
// NITE_JOINT_RIGHT_HAND,

#ifdef __cplusplus
#define EXTENRC extern "C"
#else
#define EXTENRC
#endif

EXTENRC uint32_t kinect_device_create_xdevs(struct xrt_device *const hmd, struct xrt_device **const out_xdevs, uint32_t out_xdevs_cap);
EXTENRC struct xrt_device *kinect_device_create(struct xrt_device *const hmd);

#undef EXTENRC
