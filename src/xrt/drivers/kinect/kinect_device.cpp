// Copyright 2020-2024, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Sample HMD device, use as a starting point to make your own device driver.
 *
 *
 * Based largely on simulated_hmd.c
 *
 * @author Jakob Bornecrantz <jakob@collabora.com>
 * @author Rylie Pavlik <rylie.pavlik@collabora.com>
 * @ingroup drv_kinect
 */

#include "kinect_device.h"

#include "math/m_vec3.h"
#include "os/os_time.h"
#include "xrt/xrt_compiler.h"
#include "xrt/xrt_defines.h"
#include "xrt/xrt_device.h"

#include "math/m_relation_history.h"
#include "math/m_api.h"
#include "math/m_mathinclude.h" // IWYU pragma: keep

#include "util/u_debug.h"
#include "util/u_device.h"
#include "util/u_distortion_mesh.h"
#include "util/u_logging.h"
#include "util/u_misc.h"
#include "util/u_time.h"
#include "util/u_var.h"
#include "util/u_visibility_mask.h"
#include "xrt/xrt_results.h"

#include "nite/NiTE.h"

#include <NiteEnums.h>
#include <OniEnums.h>
#include <OpenNI.h>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <stdio.h>
#include <dlfcn.h>
#include <thread>
#include <time.h>
#include <unistd.h>

#define NiTE2_MODEL_PATH "~/.local/share/NiTE2"

/*
 *
 * Structs and defines.
 *
 */

struct Joint {
	struct xrt_device base;

	struct xrt_pose pose;

	int role;

	struct m_relation_history *history;

	struct kinect *parent;

	uint64_t timestamp;
};


/*!
 * A kinect HMD device.
 *
 * @implements xrt_device
 */
struct kinect
{
	struct xrt_device base;

	struct xrt_pose pose;

	uint64_t timestamp;

	enum u_logging_level log_level;

	// has built-in mutex so thread safe
	struct m_relation_history *relation_hist;

	struct xrt_device *hmd;

	openni::Device *oni_device;

	nite::UserTracker *user_tracker;

	struct Joint *joints[NUMBER_OF_TRACKERS];

	// Tracker position updating thread
	std::thread tracking_thread;

	// Tracking point modification mutex
	std::mutex tracking_mutex;
};

/// Casting helper function
static inline struct kinect *
kinect(struct xrt_device *xdev)
{
	return (struct kinect *)xdev;
}

DEBUG_GET_ONCE_LOG_OPTION(kinect_log, "KINECT_LOG", U_LOGGING_WARN)

#define HMD_TRACE(hmd, ...) U_LOG_XDEV_IFL_T(&hmd->base, hmd->log_level, __VA_ARGS__)
#define HMD_DEBUG(hmd, ...) U_LOG_XDEV_IFL_D(&hmd->base, hmd->log_level, __VA_ARGS__)
#define HMD_INFO(hmd, ...) U_LOG_XDEV_IFL_I(&hmd->base, hmd->log_level, __VA_ARGS__)
#define HMD_ERROR(hmd, ...) U_LOG_XDEV_IFL_E(&hmd->base, hmd->log_level, __VA_ARGS__)

static void
kinect_destroy(struct xrt_device *xdev)
{
	struct kinect *dev = ((struct kinect *)xdev);

	dev->user_tracker->destroy();

	nite::NiTE::shutdown();
	openni::OpenNI::shutdown();

	for (uint32_t i = 0; i < NUMBER_OF_TRACKERS; i++) {
		if (dev->joints[i] != NULL) {
			u_device_free(&dev->joints[i]->base);
		}
	}

	// Remove the variable tracking.
	u_var_remove_root(dev);

	m_relation_history_destroy(&dev->relation_hist);

	u_device_free(&dev->base);
}

static void
kinect_get_tracked_pose(struct xrt_device *xdev,
                            enum xrt_input_name name,
                            int64_t at_timestamp_ns,
                            struct xrt_space_relation *out_relation)
{
	struct kinect *dev = (struct kinect *)xdev;
	struct xrt_device *hmd = dev->hmd;

	// Update positions of joints
	// NiteUserTrackerFrame *frame = NULL;
	// niteReadUserTrackerFrame(*dev->user_tracker_handle, &frame);

	if (hmd != NULL) {
	 	xrt_device_get_tracked_pose(hmd, name, at_timestamp_ns, out_relation);
	} else {
		*out_relation = XRT_SPACE_RELATION_ZERO;
	}

	
}

static xrt_result_t 
kinect_update_inputs(struct xrt_device *xdev)
{
	struct kinect *const dev = (struct kinect *)xdev;

	for (uint32_t i = 0; i < dev->base.input_count; i++) {
		dev->base.inputs[i].timestamp = dev->timestamp;
	}

	for (uint32_t i = 0; i < NUMBER_OF_TRACKERS; i++) {
		if (dev->joints[i] != NULL) {
			dev->tracking_mutex.lock();
			dev->joints[i]->base.inputs[0].timestamp = dev->timestamp;
			dev->tracking_mutex.unlock();
		}
	}

	struct xrt_space_relation head = XRT_SPACE_RELATION_ZERO;
	kinect_get_tracked_pose(xdev, XRT_INPUT_GENERIC_HEAD_POSE, os_monotonic_get_ns(), &head);

	return XRT_SUCCESS;
}


// static xrt_result_t
// kinect_get_body_joints(struct xrt_device *const xdev,
// 						const enum xrt_input_name body_tracking_type,
// 						const int64_t desired_timestamp_ns,
// 						struct xrt_body_joint_set *const out_value)
// {
// 	static const uint32_t joint_map[NUMBER_OF_TRACKERS] = {
// 		[NITE_JOINT_NECK] = XRT_BODY_JOINT_NECK_FB,
// 		[NITE_JOINT_TORSO] = XRT_BODY_JOINT_SPINE_MIDDLE_FB,
// 		[NITE_JOINT_LEFT_SHOULDER] = XRT_BODY_JOINT_LEFT_SHOULDER_FB,
// 		[NITE_JOINT_RIGHT_SHOULDER] = XRT_BODY_JOINT_RIGHT_SHOULDER_FB,
// 		[NITE_JOINT_LEFT_ELBOW] = XRT_BODY_JOINT_LEFT_ARM_UPPER_FB,
// 		[NITE_JOINT_RIGHT_ELBOW] = XRT_BODY_JOINT_RIGHT_ARM_UPPER_FB,
// 		[NITE_JOINT_LEFT_HAND] = XRT_BODY_JOINT_LEFT_HAND_WRIST_FB,
// 		[NITE_JOINT_RIGHT_HAND] = XRT_BODY_JOINT_RIGHT_HAND_WRIST_FB,
// 		//[NITE_JOINT_LEFT_HIP] = XRT_BODY_JOINT_
// 		//[NITE_JOINT_RIGHT_HIP] = XRT_BODY_JOINT_
// 		//[NITE_JOINT_LEFT_KNEE] = XRT_BODY_JOINT_LEFT_
// 		//[NITE_JOINT_RIGHT_KNEE] = XRT_BODY_JOINT_RIGHT_
// #ifdef XRT_FEATURE_OPENXR_BODY_TRACKING_FULL_BODY_META
// 		[NITE_JOINT_LEFT_FOOT] = XRT_FULL_BODY_JOINT_LEFT_FOOT_TRANSVERSE_META
// 		[NITE_JOINT_RIGHT_FOOT] = XRT_FULL_BODY_JOINT_RIGHT_FOOT_TRANSVERSE_META
// #endif
// 	};

// 	struct xrt_body_joint_location_fb *joints;
// 	uint32_t num_joints;

// 	switch (body_tracking_type) {
// 		case XRT_INPUT_FB_BODY_TRACKING: {
// 			joints = out_value->body_joint_set_fb.joint_locations;
// 			num_joints = ARRAY_SIZE(out_value->body_joint_set_fb.joint_locations);
// 			break;
// 		}
// #ifdef XRT_FEATURE_OPENXR_BODY_TRACKING_FULL_BODY_META
// 		case XRT_INPUT_META_FULL_BODY_TRACKING: {
// 			joints = out_value->full_body_joint_set_meta.joint_locations;
// 			joint_count = ARRAY_SIZE(out_value->full_body_joint_set_meta.joint_locations);
// 			break;
// 		}
// #endif
// 		default: return XRT_ERROR_NOT_IMPLEMENTED;
// 	}

// 	struct kinect *const dev = (struct kinect *)xdev;
	

// }

static void
kinect_joint_get_tracked_pose(struct xrt_device *xdev,
							  enum xrt_input_name name,
							  int64_t at_timestamp_ns,
							  struct xrt_space_relation *out_relation)
{
	struct Joint *dev = ((struct Joint*)xdev);
	
	dev->parent->tracking_mutex.lock();
	m_relation_history_get(dev->history, at_timestamp_ns, out_relation);
	dev->parent->tracking_mutex.unlock();
}

static void
kinect_joint_destroy(struct xrt_device *xdev)
{
	struct Joint *dev = ((struct Joint*)xdev);

	m_relation_history_destroy(&dev->history);

	u_device_free(&dev->base);
}

void 
kinect_tracking(struct kinect *dev){
	while (true) {
		nite::UserTrackerFrameRef *frame = new nite::UserTrackerFrameRef();
		dev->user_tracker->readFrame(frame);

		if (frame != NULL) {
			if (frame->getUsers().getSize() > 0) {
				nite::UserData user = frame->getUsers()[0];

				nite::Skeleton skeleton = user.getSkeleton();

				if (skeleton.getState() == nite::SKELETON_TRACKED) {
					// Get tare position by taking the neck position and putting it 0.02m below the head.
					xrt_space_relation head;
					dev->hmd->get_tracked_pose(dev->hmd, XRT_INPUT_GENERIC_HEAD_POSE, os_monotonic_get_ns(), &head);
					xrt_vec3 neck_position = xrt_vec3(skeleton.getJoint(nite::JointType(NITE_JOINT_NECK)).getPosition().x / -500.0f,
													skeleton.getJoint(nite::JointType(NITE_JOINT_NECK)).getPosition().y / 500.0f,
													skeleton.getJoint(nite::JointType(NITE_JOINT_NECK)).getPosition().z / -1000.0f);

					xrt_vec3 tare_offset = xrt_vec3(head.pose.position.x - neck_position.x, // Left/right
													head.pose.position.y - neck_position.y - 0.02f, // Y is up
													head.pose.position.z - neck_position.z); // Forwards/backward
					// xrt_vec3 tare_offset = xrt_vec3(neck_position.x - head.pose.position.x, // Left/right
					// 								neck_position.y - head.pose.position.y, // Y is up
					// 								neck_position.z - head.pose.position.z); // Forwards/backward
											
					

					for (uint32_t i = 0; i < NUMBER_OF_TRACKERS; i++) {
						xrt_space_relation rel = {};

						uint64_t timestamp = os_monotonic_get_ns();

						int idx = TRACKER_ROLES[i];
						xrt_pose new_pose;
						new_pose.position = m_vec3_add(xrt_vec3(skeleton.getJoint(nite::JointType(TRACKER_ROLES[i])).getPosition().x / -500.0f,
													skeleton.getJoint(nite::JointType(TRACKER_ROLES[i])).getPosition().y / 500.0f,
													skeleton.getJoint(nite::JointType(TRACKER_ROLES[i])).getPosition().z / -1000.0f), tare_offset);

						new_pose.orientation = xrt_quat(skeleton.getJoint(nite::JointType(TRACKER_ROLES[i])).getOrientation().w,
													skeleton.getJoint(nite::JointType(TRACKER_ROLES[i])).getOrientation().x,
													skeleton.getJoint(nite::JointType(TRACKER_ROLES[i])).getOrientation().y,
													skeleton.getJoint(nite::JointType(TRACKER_ROLES[i])).getOrientation().z);
						
						// new_pose.orientation = xrt_quat(user.skeleton.joints[idx].orientation.x, user.skeleton.joints[idx].orientation.y, user.skeleton.joints[idx].orientation.z, user.skeleton.joints[idx].orientation.w);
						
						xrt_vec3 linear_velocity = {
							fabsf((dev->joints[i]->pose.position.x - new_pose.position.x) / (dev->joints[i]->timestamp - timestamp)),
							fabsf((dev->joints[i]->pose.position.y - new_pose.position.y) / (dev->joints[i]->timestamp - timestamp)),
							fabsf((dev->joints[i]->pose.position.z - new_pose.position.z) / (dev->joints[i]->timestamp - timestamp))
						};

						xrt_vec3 angular_velocity = XRT_VEC3_ZERO;

						math_quat_rotate_derivative(&new_pose.orientation, &angular_velocity, &rel.angular_velocity);

						rel.linear_velocity = linear_velocity;
						rel.pose = new_pose;

						rel.relation_flags = (xrt_space_relation_flags)(
							XRT_SPACE_RELATION_ORIENTATION_VALID_BIT | XRT_SPACE_RELATION_POSITION_VALID_BIT |
							XRT_SPACE_RELATION_ORIENTATION_TRACKED_BIT | XRT_SPACE_RELATION_POSITION_TRACKED_BIT |
							XRT_SPACE_RELATION_LINEAR_VELOCITY_VALID_BIT | XRT_SPACE_RELATION_ANGULAR_VELOCITY_VALID_BIT
						);

						dev->tracking_mutex.lock();
						dev->joints[i]->pose = new_pose;
						dev->joints[i]->timestamp = timestamp;

						m_relation_history_push(dev->joints[i]->history, &rel, timestamp);
						dev->tracking_mutex.unlock();

						//U_LOG_E("Joint %i: %f %f %f\n", i, new_pose.position.x, new_pose.position.y, new_pose.position.z);

					}
				} else {
					if (user.isNew()) {
						// niteStartSkeletonTracking(*dev->user_tracker, user.id);
						dev->user_tracker->startSkeletonTracking(user.getId());
					}
				}
			}
		}
	}
}

struct xrt_device *
kinect_device_create(struct xrt_device *const hmd)
{
	struct xrt_device *out = NULL;
	const uint32_t result = kinect_device_create_xdevs(hmd, &out, 1);
	return result ? out : NULL;
}

uint32_t 
kinect_device_create_xdevs(struct xrt_device *const hmd, struct xrt_device **const out_xdevs, uint32_t out_xdevs_cap)
{
	if (out_xdevs_cap < 1) {
		return 0;
	}

	if (out_xdevs_cap - 1 > NUMBER_OF_TRACKERS) {
		out_xdevs_cap = NUMBER_OF_TRACKERS + 1;
	}

	// This indicates you won't be using Monado's built-in tracking algorithms.
	enum u_device_alloc_flags flags =
	    (enum u_device_alloc_flags)(U_DEVICE_ALLOC_TRACKING_NONE);

#ifdef XRT_FEATURE_OPENXR_BODY_TRACKING_FULL_BODY_META
	struct kinect *kt = U_DEVICE_ALLOCATE(struct kinect, flags, 2, 0);
#else
	struct kinect *kt = U_DEVICE_ALLOCATE(struct kinect, flags, 1, 0);
#endif

	kt->base.name = XRT_DEVICE_FB_BODY_TRACKING;
	kt->base.device_type = XRT_DEVICE_TYPE_BODY_TRACKER;
	strncpy(kt->base.str, "Kinect Full Body Tracking", sizeof(kt->base.str) - 1);
	kt->base.body_tracking_supported = true;
	kt->base.get_tracked_pose = kinect_get_tracked_pose;
	// kt->base.get_body_skeleton = kinect_get_body_skeleton;
	// kt->base.get_body_joints = kinect_get_body_joints;
	kt->base.update_inputs = kinect_update_inputs;
	kt->base.destroy = kinect_destroy;

	kt->base.inputs[0].name = XRT_INPUT_FB_BODY_TRACKING;
#ifdef XRT_FEATURE_OPENXR_BODY_TRACKING_FULL_BODY_META
	kt->base.inputs[1].name = XRT_INPUT_FB_BODY_TRACKING_META;
#endif

	kt->hmd = hmd;

	openni::Status init_result = openni::OpenNI::initialize();

	if (init_result) {
		U_LOG_E("oniInitialize Fail! Code: %d", init_result);
		return 0;
	}

	// OniDeviceInfo *pdevs;
	// int num_devices = 0;

	// oniGetDeviceList(&pdevs, &num_devices);

	// char *uri = NULL;

	// for (int i = 0; i < num_devices; i++) {
	// 	U_LOG_E("Found device: %s VendorID: %04x ProductID: %04x\n", pdevs[i].uri, pdevs[i].usbVendorId, pdevs[i].usbProductId);

	// 	// if (pdevs[i].usbVendorId == 0x37b8 && pdevs[i].usbProductId == 0xdcd8) {
	// 	// 	uri = pdevs[i].uri;
	// 	// 	break;
	// 	// }
	// }

	// uri = pdevs[0].uri;


	// if (!uri) {
	// 	U_LOG_E("No Kinect found!\n");
	// 	return 0;
	// }

	// oniReleaseDeviceList(pdevs);

	// kt->oni_device = NULL;
	// OniStatus open_result = oniDeviceOpen(uri, kt->oni_device);

	openni::OpenNI::initialize();

	kt->oni_device = new openni::Device();

	openni::Status open_result = kt->oni_device->open(openni::ANY_DEVICE);

	if (open_result) {
		U_LOG_E("oniDeviceOpen Fail! Code: %d", open_result);
		return 0;
	}
	
	// kt->user_tracker_handle = NULL;
	// NiteStatus init_user_tracker_result = niteInitializeUserTracker(kt->user_tracker_handle);

	auto prev_path = std::filesystem::current_path();
	U_LOG_D("Current path: %s\n", prev_path.c_str());
	std::filesystem::current_path(std::filesystem::path(std::getenv("NITE2_PATH")).parent_path());

	nite::NiTE::initialize();

	kt->user_tracker = new nite::UserTracker();
	nite::Status init_user_tracker_result = kt->user_tracker->create(kt->oni_device);

	if (init_user_tracker_result) {
		U_LOG_E("niteInitializeUserTracker Fail! Code: %d", init_user_tracker_result);
		return 0;
	}
	
	kt->timestamp = os_monotonic_get_ns();

	kt->tracking_thread = std::thread(kinect_tracking, kt);

	for (int i = 0; i < NUMBER_OF_TRACKERS; i++) {
		struct Joint *joint = U_DEVICE_ALLOCATE(struct Joint, (enum u_device_alloc_flags)(U_DEVICE_ALLOC_TRACKING_NONE), 1, 0);
		joint->role = TRACKER_ROLES[i];
		joint->parent = kt;
		joint->timestamp = os_monotonic_get_ns();
		joint->base.name = XRT_DEVICE_VIVE_TRACKER;
		joint->base.device_type = XRT_DEVICE_TYPE_GENERIC_TRACKER;
		joint->base.tracking_origin = hmd->tracking_origin;
		joint->base.orientation_tracking_supported = true;
		joint->base.position_tracking_supported = true;
		joint->base.update_inputs = u_device_noop_update_inputs;
		joint->base.get_tracked_pose = kinect_joint_get_tracked_pose;
		joint->base.destroy = kinect_joint_destroy;
		joint->base.inputs[0].name = XRT_INPUT_GENERIC_TRACKER_POSE;
		m_relation_history_create(&joint->history);
		kt->joints[i] = joint;
		// kt->joints[i]->role = TRACKER_ROLES[i];
		// kt->joints[i]->parent = kt;
		// kt->joints[i]->timestamp = os_monotonic_get_ns();
		// kt->joints[i]->base.name = XRT_DEVICE_VIVE_TRACKER;
		// kt->joints[i]->base.device_type = XRT_DEVICE_TYPE_GENERIC_TRACKER;
		// kt->joints[i]->base.tracking_origin = hmd->tracking_origin;
		// kt->joints[i]->base.orientation_tracking_supported = true;
		// kt->joints[i]->base.position_tracking_supported = true;
		// kt->joints[i]->base.update_inputs = u_device_noop_update_inputs;
		// kt->joints[i]->base.get_tracked_pose = kinect_joint_get_tracked_pose;
		// kt->joints[i]->base.destroy = kinect_joint_destroy;
		// kt->joints[i]->base.inputs[0].name = XRT_INPUT_GENERIC_TRACKER_POSE;
		// m_relation_history_create(&kt->joints[i]->history);
	}

	uint32_t tracker_count = 0;
	out_xdevs[tracker_count++] = &kt->base;
	for (int i = 0; i < NUMBER_OF_TRACKERS; i++) {
		out_xdevs[tracker_count++] = &kt->joints[i]->base;
	}

	return tracker_count;

	// kt->base.inputs[0].name = XRT_INPUT_GENERIC_TRACKER_POSE;

	// // This list should be ordered, most preferred first.
	// size_t idx = 0;
	// kt->base.hmd->blend_modes[idx++] = XRT_BLEND_MODE_OPAQUE;
	// kt->base.hmd->blend_mode_count = idx;

	// kt->base.update_inputs = kinect_update_inputs;
	// kt->base.get_tracked_pose = kinect_get_tracked_pose;
	// kt->base.get_view_poses = kinect_get_view_poses;
	// kt->base.get_visibility_mask = kinect_get_visibility_mask;
	

	// // Distortion information, fills in xdev->compute_distortion().
	// u_distortion_mesh_set_none(&kt->base);

	// // populate this with something more complex if required
	// // hmd->base.compute_distortion = kinect_compute_distortion;

	// kt->pose = (struct xrt_pose)XRT_POSE_IDENTITY;
	// kt->log_level = debug_get_log_option_kinect_log();

	// // Print name.
	// snprintf(kt->base.str, XRT_DEVICE_NAME_LEN, "Sample HMD");
	// snprintf(kt->base.serial, XRT_DEVICE_NAME_LEN, "Sample HMD S/N");

	// m_relation_history_create(&kt->relation_hist);

	// // Setup input.
	// kt->base.name = XRT_DEVICE_GENERIC_HMD;
	// kt->base.device_type = XRT_DEVICE_TYPE_HMD;
	// kt->base.inputs[0].name = XRT_INPUT_GENERIC_HEAD_POSE;
	// kt->base.orientation_tracking_supported = true;
	// kt->base.position_tracking_supported = true;

	// // Set up display details
	// // refresh rate
	// kt->base.hmd->screens[0].nominal_frame_interval_ns = time_s_to_ns(1.0f / 90.0f);

	// const double hFOV = 90 * (M_PI / 180.0);
	// const double vFOV = 96.73 * (M_PI / 180.0);
	// // center of projection
	// const double hCOP = 0.529;
	// const double vCOP = 0.5;
	// if (
	//     /* right eye */
	//     !math_compute_fovs(1, hCOP, hFOV, 1, vCOP, vFOV, &kt->base.hmd->distortion.fov[1]) ||
	//     /*
	//      * left eye - same as right eye, except the horizontal center of projection is moved in the opposite
	//      * direction now
	//      */
	//     !math_compute_fovs(1, 1.0 - hCOP, hFOV, 1, vCOP, vFOV, &kt->base.hmd->distortion.fov[0])) {
	// 	// If those failed, it means our math was impossible.
	// 	HMD_ERROR(kt, "Failed to setup basic device info");
	// 	kinect_destroy(&kt->base);
	// 	return NULL;
	// }
	// const int panel_w = 1080;
	// const int panel_h = 1200;

	// // Single "screen" (always the case)
	// kt->base.hmd->screens[0].w_pixels = panel_w * 2;
	// kt->base.hmd->screens[0].h_pixels = panel_h;

	// // Left, Right
	// for (uint8_t eye = 0; eye < 2; ++eye) {
	// 	kt->base.hmd->views[eye].display.w_pixels = panel_w;
	// 	kt->base.hmd->views[eye].display.h_pixels = panel_h;
	// 	kt->base.hmd->views[eye].viewport.y_pixels = 0;
	// 	kt->base.hmd->views[eye].viewport.w_pixels = panel_w;
	// 	kt->base.hmd->views[eye].viewport.h_pixels = panel_h;
	// 	// if rotation is not identity, the dimensions can get more complex.
	// 	kt->base.hmd->views[eye].rot = u_device_rotation_ident;
	// }
	// // left eye starts at x=0, right eye starts at x=panel_width
	// kt->base.hmd->views[0].viewport.x_pixels = 0;
	// kt->base.hmd->views[1].viewport.x_pixels = panel_w;

	// // Distortion information, fills in xdev->compute_distortion().
	// u_distortion_mesh_set_none(&kt->base);

	// // Just put an initial identity value in the tracker
	// struct xrt_space_relation identity = XRT_SPACE_RELATION_ZERO;
	// identity.relation_flags = (enum xrt_space_relation_flags)(XRT_SPACE_RELATION_ORIENTATION_TRACKED_BIT |
	//                                                           XRT_SPACE_RELATION_ORIENTATION_VALID_BIT);
	// uint64_t now = os_monotonic_get_ns();
	// m_relation_history_push(kt->relation_hist, &identity, now);

	// // Setup variable tracker: Optional but useful for debugging
	// u_var_add_root(kt, "Sample HMD", true);
	// u_var_add_log_level(kt, &kt->log_level, "log_level");


	// return &kt->base;
}