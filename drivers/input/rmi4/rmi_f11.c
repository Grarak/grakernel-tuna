/*
 * Copyright (c) 2011,2012 Synaptics Incorporated
 * Copyright (c) 2011 Unixphere
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#define FUNCTION_DATA f11_data
#define FNUM 11

/* NOTE(GOOG) We use wakelocks on input to prevent race conditions where
 * we may suspend while there is still valid data in transit via the
 * input module. */
#ifdef CONFIG_WAKELOCK
#include <linux/wakelock.h>
#define WAKELOCK_TIMEOUT_IN_MS 250
#endif
/* //NOTE(GOOG) */

#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/kconfig.h>
#include <linux/rmi.h>
#include <linux/slab.h>
#include "rmi_driver.h"

#ifdef CONFIG_RMI4_DEBUG
#include <linux/debugfs.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#endif

#define F11_MAX_NUM_OF_SENSORS		8
#define F11_MAX_NUM_OF_FINGERS		10
#define F11_MAX_NUM_OF_TOUCH_SHAPES	16

#define F11_REL_POS_MIN		-128
#define F11_REL_POS_MAX		127

#define FINGER_STATE_MASK	0x03
#define GET_FINGER_STATE(f_states, i) \
	((f_states[i / 4] >> (2 * (i % 4))) & FINGER_STATE_MASK)

#define F11_CTRL_SENSOR_MAX_X_POS_OFFSET	6
#define F11_CTRL_SENSOR_MAX_Y_POS_OFFSET	8

#define F11_CEIL(x, y) (((x) + ((y)-1)) / (y))
#define INBOX(x, y, box) (x >= box.x && x < (box.x + box.width) \
			&& y >= box.y && y < (box.y + box.height))

#define DEFAULT_XY_MAX 9999
#define DEFAULT_MAX_ABS_MT_PRESSURE 255
#define DEFAULT_MAX_ABS_MT_TOUCH 15
#define DEFAULT_MAX_ABS_MT_ORIENTATION 1
#define DEFAULT_MIN_ABS_MT_TRACKING_ID 1
#define DEFAULT_MAX_ABS_MT_TRACKING_ID 10
#define MAX_NAME_LENGTH 256

/* Google definitions */
/* Set or unset to output touchpad decoded gesture information */
// #define DEBUG_GESTURES

/* Offsets added to differentiate gesture information from real information */
#define GESTURE_OFFSET_X 100
#define GESTURE_OFFSET_Y 100
#define GESTURE_MT_MAJOR 15
#define GESTURE_MT_MINOR 15
#define GESTURE_PRESSURE 1

/* Default reserved section of the trackpad for thumb switch */
#define VIEW_CUTOUT_MIN_Y 160
#define VIEW_CUTOUT_MAX_Y 187
#define VIEW_CUTOUT_MIN_X 940
#define VIEW_CUTOUT_MAX_X 1000

/* Size of bounding box around driver detected tap gestures in pixels. */
static int gesture_detect_tap_box_x = 90;
static int gesture_detect_tap_box_y = 90;
/* Time period in which a gesture tap may be detected in ms. */
static int gesture_detect_tap_time_min = 5;
static int gesture_detect_tap_time_max = 250;

/* These values are all added to the well known GESTURE_OFFSET values
 * above to specify unique gestures based upon absolute motion data.
 * Flicks are not specified here as they have an inherent locale value */

#define GESTURE_OFFSET_SINGLE_TAP_X 0
#define GESTURE_OFFSET_SINGLE_TAP_Y 0

#define GESTURE_OFFSET_DOUBLE_TAP_X -1
#define GESTURE_OFFSET_DOUBLE_TAP_Y -1

#define GESTURE_OFFSET_PRESS_X 1
#define GESTURE_OFFSET_PRESS_Y 1

static ssize_t f11_gesture_show(struct device *dev,
                                struct device_attribute *attr,
                                char *buf);

static ssize_t f11_gesture_store(struct device *dev,
                                 struct device_attribute *attr,
                                 const char *buf, size_t count);

static ssize_t f11_view_enable_show(struct device *dev,
                                    struct device_attribute *attr,
                                    char *buf);

static ssize_t f11_view_enable_store(struct device *dev,
                                     struct device_attribute *attr,
                                     const char *buf, size_t count);

static ssize_t f11_view_dim_show(struct device *dev,
                                 struct device_attribute *attr,
                                 char *buf);

static ssize_t f11_view_dim_store(struct device *dev,
                                  struct device_attribute *attr,
                                  const char *buf, size_t count);

static ssize_t f11_view_val_show(struct device *dev,
                                 struct device_attribute *attr,
                                 char *buf);

/* End google definitions */

static ssize_t f11_relreport_show(struct device *dev,
					struct device_attribute *attr,
					char *buf);

static ssize_t f11_relreport_store(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf, size_t count);

static ssize_t f11_maxPos_show(struct device *dev,
				     struct device_attribute *attr, char *buf);

static ssize_t f11_rezero_store(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf, size_t count);

static void rmi_f11_free_memory(struct rmi_function_container *fc);

static int rmi_f11_initialize(struct rmi_function_container *fc);

static int rmi_f11_create_sysfs(struct rmi_function_container *fc);

static int rmi_f11_config(struct rmi_function_container *fc);

static int rmi_f11_register_devices(struct rmi_function_container *fc);

static void rmi_f11_free_devices(struct rmi_function_container *fc);

static void f11_set_abs_params(struct rmi_function_container *fc, int index);

static struct device_attribute attrs[] = {
	__ATTR(gesture, RMI_RW_ATTR, f11_gesture_show, f11_gesture_store),
	__ATTR(relreport, RMI_RW_ATTR, f11_relreport_show, f11_relreport_store),
	__ATTR(maxPos, RMI_RO_ATTR, f11_maxPos_show, rmi_store_error),
	__ATTR(rezero, RMI_WO_ATTR, rmi_show_error, f11_rezero_store),
	__ATTR(view_enable, RMI_RW_ATTR, f11_view_enable_show, f11_view_enable_store),
	__ATTR(view_val, RMI_RO_ATTR, f11_view_val_show, rmi_store_error),
	__ATTR(view_dim, RMI_RW_ATTR, f11_view_dim_show, f11_view_dim_store),
};

/**
 * @rezero - writing 1 to this will cause the sensor to calibrate to the
 * current capacitive state.
 */
union f11_2d_commands {
	struct {
		bool rezero:1;
		u8 reserved:7;
	} __attribute__((__packed__));
	u8 reg;
};

/**
 * @nbr_of_sensors - the number of 2D sensors on the touch device.
 * @has_query9 - indicates the F11_2D_Query9 register exists.
 * @has_query11 - indicates the F11_2D_Query11 register exists.
 * @has_z_tuning - if set, the sensor supports Z tuning and registers
 * F11_2D_Ctrl29 through F11_2D_Ctrl33 exist.
 * @has_pos_interpolation_tuning - TBD
 * @has_w_tuning - the sensor supports Wx and Wy scaling and registers
 * F11_2D_Ctrl36 through F11_2D_Ctrl39 exist.
 * @has_pitch_info - the X and Y pitches of the sensor electrodes can be
 * configured and registers F11_2D_Ctrl40 and F11_2D_Ctrl41 exist.
 * @has_default_finger_width -  the default finger width settings for the
 * sensor can be configured and registers F11_2D_Ctrl42 through F11_2D_Ctrl44
 * exist.
 * @has_segmentation_aggressiveness - the sensor’s ability to distinguish
 * multiple objects close together can be configured and register F11_2D_Ctrl45
 * exists.
 * @has_tx_rw_clip -  the inactive outside borders of the sensor can be
 * configured and registers F11_2D_Ctrl46 through F11_2D_Ctrl49 exist.
 * @has_drumming_correction - the sensor can be configured to distinguish
 * between a fast flick and a quick drumming movement and registers
 * F11_2D_Ctrl50 and F11_2D_Ctrl51 exist.
 */
struct f11_2d_device_query {
	union {
		struct {
			u8 nbr_of_sensors:3;
			bool has_query9:1;
			bool has_query11:1;
			u8 reserved:3;
		} __attribute__((__packed__));
		u8 f11_2d_query0;
	};

	union {
		struct {
			bool has_z_tuning:1;
			bool has_pos_interpolation_tuning:1;
			bool has_w_tuning:1;
			bool has_pitch_info:1;
			bool has_default_finger_width:1;
			bool has_segmentation_aggressiveness:1;
			bool has_tx_rw_clip:1;
			bool has_drumming_correction:1;
		} __attribute__((__packed__));
		u8 f11_2d_query11;
	};
};

/**
 * @has_pen - detection of a stylus is supported and registers F11_2D_Ctrl20
 * and F11_2D_Ctrl21 exist.
 * @has_proximity - detection of fingers near the sensor is supported and
 * registers F11_2D_Ctrl22 through F11_2D_Ctrl26 exist.
 * @has_palm_det_sensitivity -  the sensor supports the palm detect sensitivity
 * feature and register F11_2D_Ctrl27 exists.
 * @has_two_pen_thresholds - is has_pen is also set, then F11_2D_Ctrl35 exists.
 * @has_contact_geometry - the sensor supports the use of contact geometry to
 * map absolute X and Y target positions and registers F11_2D_Data18.* through
 * F11_2D_Data27 exist.
 */
union f11_2d_query9 {
	struct {
		bool has_pen:1;
		bool has_proximity:1;
		bool has_palm_det_sensitivity:1;
		bool has_suppress_on_palm_detect:1;
		bool has_two_pen_thresholds:1;
		bool has_contact_geometry:1;
	} __attribute__((__packed__));
	u8 reg;
};

/**
 * @number_of_fingers - describes the maximum number of fingers the 2-D sensor
 * supports.
 * @has_rel - the sensor supports relative motion reporting.
 * @has_abs - the sensor supports absolute poition reporting.
 * @has_gestures - the sensor supports gesture reporting.
 * @has_sensitivity_adjust - the sensor supports a global sensitivity
 * adjustment.
 * @configurable - the sensor supports various configuration options.
 * @num_of_x_electrodes -  the maximum number of electrodes the 2-D sensor
 * supports on the X axis.
 * @num_of_y_electrodes -  the maximum number of electrodes the 2-D sensor
 * supports on the Y axis.
 * @max_electrodes - the total number of X and Y electrodes that may be
 * configured.
 * @abs_data_size - describes the format of data reported by the absolute
 * data source.  Only one format (the kind used here) is supported at this
 * time.
 * @has_anchored_finger - then the sensor supports the high-precision second
 * finger tracking provided by the manual tracking and motion sensitivity
 * options.
 * @has_adjust_hyst - the difference between the finger release threshold and
 * the touch threshold.
 * @has_dribble - the sensor supports the generation of dribble interrupts,
 * which may be enabled or disabled with the dribble control bit.
 * @f11_2d_query6 - reserved.
 * @has_single_tap - a basic single-tap gesture is supported.
 * @has_tap_n_hold - tap-and-hold gesture is supported.
 * @has_double_tap - double-tap gesture is supported.
 * @has_early_tap - early tap is supported and reported as soon as the finger
 * lifts for any tap event that could be interpreted as either a single tap
 * or as the first tap of a double-tap or tap-and-hold gesture.
 * @has_flick - flick detection is supported.
 * @has_press - press gesture reporting is supported.
 * @has_pinch - pinch gesture detection is supported.
 * @has_palm_det - the 2-D sensor notifies the host whenever a large conductive
 * object such as a palm or a cheek touches the 2-D sensor.
 * @has_rotate - rotation gesture detection is supported.
 * @has_touch_shapes - TouchShapes are supported.  A TouchShape is a fixed
 * rectangular area on the sensor that behaves like a capacitive button.
 * @has_scroll_zones - scrolling areas near the sensor edges are supported.
 * @has_individual_scroll_zones - if 1, then 4 scroll zones are supported;
 * if 0, then only two are supported.
 * @has_multi_finger_scroll - the multifinger_scrolling bit will be set when
 * more than one finger is involved in a scrolling action.
 * @nbr_touch_shapes - the total number of touch shapes supported.
 */
struct f11_2d_sensor_query {
	union {
		struct {
			/* query1 */
			u8 number_of_fingers:3;
			bool has_rel:1;
			bool has_abs:1;
			bool has_gestures:1;
			bool has_sensitivity_adjust:1;
			bool configurable:1;
			/* query2 */
			u8 num_of_x_electrodes:7;
			u8 reserved_1:1;
			/* query3 */
			u8 num_of_y_electrodes:7;
			u8 reserved_2:1;
			/* query4 */
			u8 max_electrodes:7;
			u8 reserved_3:1;
		} __attribute__((__packed__));
		u8 f11_2d_query1__4[4];
	};

	union {
		struct {
			u8 abs_data_size:3;
			bool has_anchored_finger:1;
			bool has_adj_hyst:1;
			bool has_dribble:1;
			u8 reserved_4:2;
		} __attribute__((__packed__));
		u8 f11_2d_query5;
	};

	u8 f11_2d_query6;

	union {
		struct {
			bool has_single_tap:1;
			bool has_tap_n_hold:1;
			bool has_double_tap:1;
			bool has_early_tap:1;
			bool has_flick:1;
			bool has_press:1;
			bool has_pinch:1;
			bool padding:1;

			bool has_palm_det:1;
			bool has_rotate:1;
			bool has_touch_shapes:1;
			bool has_scroll_zones:1;
			bool has_individual_scroll_zones:1;
			bool has_multi_finger_scroll:1;
		} __attribute__((__packed__));
		u8 f11_2d_query7__8[2];
	};

	union f11_2d_query9 query9;

	union {
		struct {
			u8 nbr_touch_shapes:5;
		} __attribute__((__packed__));
		u8 f11_2d_query10;
	};
};

/**
 * @reporting_mode - controls how often finger position data is reported.
 * @abs_pos_filt - when set, enables various noise and jitter filtering
 * algorithms for absolute reports.
 * @rel_pos_filt - when set, enables various noise and jitter filtering
 * algorithms for relative reports.
 * @rel_ballistics - enables ballistics processing for the relative finger
 * motion on the 2-D sensor.
 * @dribble - enables the dribbling feature.
 * @report_beyond_clip - when this is set, fingers outside the active area
 * specified by the x_clip and y_clip registers will be reported, but with
 * reported finger position clipped to the edge of the active area.
 * @palm_detect_thresh - the threshold at which a wide finger is considered a
 * palm. A value of 0 inhibits palm detection.
 * @motion_sensitivity - specifies the threshold an anchored finger must move
 * before it is considered no longer anchored.  High values mean more
 * sensitivity.
 * @man_track_en - for anchored finger tracking, whether the host (1) or the
 * device (0) determines which finger is the tracked finger.
 * @man_tracked_finger - when man_track_en is 1, specifies whether finger 0 or
 * finger 1 is the tracked finger.
 * @delta_x_threshold - 2-D position update interrupts are inhibited unless
 * the finger moves more than a certain threshold distance along the X axis.
 * @delta_y_threshold - 2-D position update interrupts are inhibited unless
 * the finger moves more than a certain threshold distance along the Y axis.
 * @velocity - When rel_ballistics is set, this register defines the
 * velocity ballistic parameter applied to all relative motion events.
 * @acceleration - When rel_ballistics is set, this register defines the
 * acceleration ballistic parameter applied to all relative motion events.
 * @sensor_max_x_pos - the maximum X coordinate reported by the sensor.
 * @sensor_max_y_pos - the maximum Y coordinate reported by the sensor.
 */
union f11_2d_ctrl0_9 {
	struct {
		/* F11_2D_Ctrl0 */
		u8 reporting_mode:3;
		bool abs_pos_filt:1;
		bool rel_pos_filt:1;
		bool rel_ballistics:1;
		bool dribble:1;
		bool report_beyond_clip:1;
		/* F11_2D_Ctrl1 */
		u8 palm_detect_thres:4;
		u8 motion_sensitivity:2;
		bool man_track_en:1;
		bool man_tracked_finger:1;
		/* F11_2D_Ctrl2 and 3 */
		u8 delta_x_threshold:8;
		u8 delta_y_threshold:8;
		/* F11_2D_Ctrl4 and 5 */
		u8 velocity:8;
		u8 acceleration:8;
		/* F11_2D_Ctrl6 thru 9 */
		u16 sensor_max_x_pos:12;
		u8 ctrl7_reserved:4;
		u16 sensor_max_y_pos:12;
		u8 ctrl9_reserved:4;
	} __attribute__((__packed__));
	struct {
		u8 regs[10];
		u16 address;
	} __attribute__((__packed__));
};

/**
 * @single_tap_int_enable - enable tap gesture recognition.
 * @tap_n_hold_int_enable - enable tap-and-hold gesture recognition.
 * @double_tap_int_enable - enable double-tap gesture recognition.
 * @early_tap_int_enable - enable early tap notification.
 * @flick_int_enable - enable flick detection.
 * @press_int_enable - enable press gesture recognition.
 * @pinch_int_enable - enable pinch detection.
 */
union f11_2d_ctrl10 {
	struct {
		bool single_tap_int_enable:1;
		bool tap_n_hold_int_enable:1;
		bool double_tap_int_enable:1;
		bool early_tap_int_enable:1;
		bool flick_int_enable:1;
		bool press_int_enable:1;
		bool pinch_int_enable:1;
	} __attribute__((__packed__));
	u8 reg;
};

/**
 * @palm_detect_int_enable - enable palm detection feature.
 * @rotate_int_enable - enable rotate gesture detection.
 * @touch_shape_int_enable - enable the TouchShape feature.
 * @scroll_zone_int_enable - enable scroll zone reporting.
 * @multi_finger_scroll_int_enable - enable the multfinger scroll feature.
 */
union f11_2d_ctrl11 {
	struct {
		bool palm_detect_int_enable:1;
		bool rotate_int_enable:1;
		bool touch_shape_int_enable:1;
		bool scroll_zone_int_enable:1;
		bool multi_finger_scroll_int_enable:1;
	} __attribute__((__packed__));
	u8 reg;
};

union f11_2d_ctrl12 {
	struct {
		u8 sensor_map:7;
		bool xy_sel:1;
	} __attribute__((__packed__));
	u8 reg;
};

/**
 * @sens_adjustment - allows a host to alter the overall sensitivity of a
 * 2-D sensor. A positive value in this register will make the sensor more
 * sensitive than the factory defaults, and a negative value will make it
 * less sensitive.
 * @hyst_adjustment - increase the touch/no-touch hysteresis by 2 Z-units for
 * each one unit increment in this setting.
 */
union f11_2d_ctrl14 {
	struct {
		s8 sens_adjustment:5;
		u8 hyst_adjustment:3;
	} __attribute__((__packed__));
	u8 reg;
};

/**
 * @max_tap_time - the maximum duration of a tap, in 10-millisecond units.
 */
union f11_2d_ctrl15 {
	struct {
		u8 max_tap_time:8;
	} __attribute__((__packed__));
	u8 reg;
};

/**
 * @min_press_time - The minimum duration required for stationary finger(s) to
 * generate a press gesture, in 10-millisecond units.
 */
union f11_2d_ctrl16 {
	struct {
		u8 min_press_time:8;
	} __attribute__((__packed__));
	u8 reg;
};

/**
 * @max_tap_distance - Determines the maximum finger movement allowed during
 * a tap, in 0.1-millimeter units.
 */
union f11_2d_ctrl17 {
	struct {
		u8 max_tap_distance:8;
	} __attribute__((__packed__));
	u8 reg;
};

/**
 * @min_flick_distance - the minimum finger movement for a flick gesture,
 * in 1-millimeter units.
 * @min_flick_speed - the minimum finger speed for a flick gesture, in
 * 10-millimeter/second units.
 */
union f11_2d_ctrl18_19 {
	struct {
		u8 min_flick_distance:8;
		u8 min_flick_speed:8;
	} __attribute__((__packed__));
	u8 reg[2];
};

/**
 * @pen_detect_enable - enable reporting of stylus activity.
 * @pen_jitter_filter_enable - Setting this enables the stylus anti-jitter
 * filter.
 * @pen_z_threshold - This is the stylus-detection lower threshold. Smaller
 * values result in higher sensitivity.
 */
union f11_2d_ctrl20_21 {
	struct {
		bool pen_detect_enable:1;
		bool pen_jitter_filter_enable:1;
		u8 ctrl20_reserved:6;
		u8 pen_z_threshold:8;
	} __attribute__((__packed__));
	u8 reg[2];
};

/**
 * These are not accessible through sysfs yet.
 *
 * @proximity_detect_int_en - enable proximity detection feature.
 * @proximity_jitter_filter_en - enables an anti-jitter filter on proximity
 * data.
 * @proximity_detection_z_threshold - the threshold for finger-proximity
 * detection.
 * @proximity_delta_x_threshold - In reduced-reporting modes, this is the
 * threshold for proximate-finger movement in the direction parallel to the
 * X-axis.
 * @proximity_delta_y_threshold - In reduced-reporting modes, this is the
 * threshold for proximate-finger movement in the direction parallel to the
 * Y-axis.
 * * @proximity_delta_Z_threshold - In reduced-reporting modes, this is the
 * threshold for proximate-finger movement in the direction parallel to the
 * Z-axis.
 */
union f11_2d_ctrl22_26 {
	struct {
		/* control 22 */
		bool proximity_detect_int_en:1;
		bool proximity_jitter_filter_en:1;
		u8 f11_2d_ctrl6_b3__7:6;

		/* control 23 */
		u8 proximity_detection_z_threshold;

		/* control 24 */
		u8 proximity_delta_x_threshold;

		/* control 25 */
		u8 proximity_delta_y_threshold;

		/* control 26 */
		u8 proximity_delta_z_threshold;
	} __attribute__((__packed__));
	u8 regs[5];
};

/**
 * @palm_detecy_sensitivity - When this value is small, smaller objects will
 * be identified as palms; when this value is large, only larger objects will
 * be identified as palms. 0 represents the factory default.
 * @suppress_on_palm_detect - when set, all F11 interrupts except palm_detect
 * are suppressed while a palm is detected.
 */
union f11_2d_ctrl27 {
	struct {
		s8 palm_detect_sensitivity:4;
		bool suppress_on_palm_detect:1;
		u8 f11_2d_ctrl27_b5__7:3;
	} __attribute__((__packed__));
	u8 regs[1];
};

/**
 * @multi_finger_scroll_mode - allows choice of multi-finger scroll mode and
 * determines whether and how X or Y displacements are reported.
 * @edge_motion_en - enables the edge_motion feature.
 * @multi_finger_scroll_momentum - controls the length of time that scrolling
 * continues after fingers have been lifted.
 */
union f11_2d_ctrl28 {
	struct {
		u8 multi_finger_scroll_mode:2;
		bool edge_motion_en:1;
		bool f11_2d_ctrl28b_3:1;
		u8 multi_finger_scroll_momentum:4;
	} __attribute__((__packed__));
	u8 regs[1];
};

/**
 * @z_touch_threshold - Specifies the finger-arrival Z threshold. Large values
 * may cause smaller fingers to be rejected.
 * @z_touch_hysteresis - Specifies the difference between the finger-arrival
 * Z threshold and the finger-departure Z threshold.
 */
union f11_2d_ctrl29_30 {
	struct {
		u8 z_touch_threshold;
		u8 z_touch_hysteresis;
	} __attribute__((__packed__));
	struct {
		u8 regs[2];
		u16 address;
	} __attribute__((__packed__));
};


struct  f11_2d_ctrl {
	union f11_2d_ctrl0_9 *ctrl0_9;
	union f11_2d_ctrl10		*ctrl10;
	union f11_2d_ctrl11		*ctrl11;
	union f11_2d_ctrl12		*ctrl12;
	u8				ctrl12_size;
	union f11_2d_ctrl14		*ctrl14;
	union f11_2d_ctrl15		*ctrl15;
	union f11_2d_ctrl16		*ctrl16;
	union f11_2d_ctrl17		*ctrl17;
	union f11_2d_ctrl18_19		*ctrl18_19;
	union f11_2d_ctrl20_21		*ctrl20_21;
	union f11_2d_ctrl22_26 *ctrl22_26;
	union f11_2d_ctrl27 *ctrl27;
	union f11_2d_ctrl28 *ctrl28;
	union f11_2d_ctrl29_30 *ctrl29_30;
};

/**
 * @x_msb - top 8 bits of X finger position.
 * @y_msb - top 8 bits of Y finger position.
 * @x_lsb - bottom 4 bits of X finger position.
 * @y_lsb - bottom 4 bits of Y finger position.
 * @w_y - contact patch width along Y axis.
 * @w_x - contact patch width along X axis.
 * @z - finger Z value (proxy for pressure).
 */
struct f11_2d_data_1_5 {
	u8 x_msb;
	u8 y_msb;
	u8 x_lsb:4;
	u8 y_lsb:4;
	u8 w_y:4;
	u8 w_x:4;
	u8 z;
};

/**
 * @delta_x - relative motion along X axis.
 * @delta_y - relative motion along Y axis.
 */
struct f11_2d_data_6_7 {
	s8 delta_x;
	s8 delta_y;
};

/**
 * @single_tap - a single tap was recognized.
 * @tap_and_hold - a tap-and-hold gesture was recognized.
 * @double_tap - a double tap gesture was recognized.
 * @early_tap - a tap gesture might be happening.
 * @flick - a flick gesture was detected.
 * @press - a press gesture was recognized.
 * @pinch - a pinch gesture was detected.
 */
struct f11_2d_data_8 {
	bool single_tap:1;
	bool tap_and_hold:1;
	bool double_tap:1;
	bool early_tap:1;
	bool flick:1;
	bool press:1;
	bool pinch:1;
};

/**
 * @palm_detect - a palm or other large object is in contact with the sensor.
 * @rotate - a rotate gesture was detected.
 * @shape - a TouchShape has been activated.
 * @scrollzone - scrolling data is available.
 * @finger_count - number of fingers involved in the reported gesture.
 */
struct f11_2d_data_9 {
	bool palm_detect:1;
	bool rotate:1;
	bool shape:1;
	bool scrollzone:1;
	u8 finger_count:3;
};

/**
 * @pinch_motion - when a pinch gesture is detected, this is the change in
 * distance between the two fingers since this register was last read.
 */
struct f11_2d_data_10 {
	s8 pinch_motion;
};

/**
 * @x_flick_dist - when a flick gesture is detected,  the distance of flick
 * gesture in X direction.
 * @y_flick_dist - when a flick gesture is detected,  the distance of flick
 * gesture in Y direction.
 * @flick_time - the total time of the flick gesture, in 10ms units.
 */
struct f11_2d_data_10_12 {
	s8 x_flick_dist;
	s8 y_flick_dist;
	u8 flick_time;
};

/**
 * @motion - when a rotate gesture is detected, the accumulated distance
 * of the rotate motion. Clockwise motion is positive and counterclockwise
 * motion is negative.
 * @finger_separation - when a rotate gesture is detected, the distance
 * between the fingers.
 */
struct f11_2d_data_11_12 {
	s8 motion;
	u8 finger_separation;
};

/**
 * @shape_n - a bitmask of the currently activate TouchShapes (if any).
 */
struct f11_2d_data_13 {
	u8 shape_n;
};

/**
 * @horizontal - chiral scrolling distance in the X direction.
 * @vertical - chiral scrolling distance in the Y direction.
 */
struct f11_2d_data_14_15 {
	s8 horizontal;
	s8 vertical;
};

/**
 * @x_low - scroll zone motion along the lower edge of the sensor.
 * @y_right - scroll zone motion along the right edge of the sensor.
 * @x_upper - scroll zone motion along the upper edge of the sensor.
 * @y_left - scroll zone motion along the left edge of the sensor.
 */
struct f11_2d_data_14_17 {
	s8 x_low;
	s8 y_right;
	s8 x_upper;
	s8 y_left;
};

struct f11_2d_data {
	u8				*f_state;
	const struct f11_2d_data_1_5	*abs_pos;
	const struct f11_2d_data_6_7	*rel_pos;
	const struct f11_2d_data_8	*gest_1;
	const struct f11_2d_data_9	*gest_2;
	const struct f11_2d_data_10	*pinch;
	const struct f11_2d_data_10_12	*flick;
	const struct f11_2d_data_11_12	*rotate;
	const struct f11_2d_data_13	*shapes;
	const struct f11_2d_data_14_15	*multi_scroll;
	const struct f11_2d_data_14_17	*scroll_zones;
};

struct f11_2d_sensor {
	struct rmi_f11_2d_axis_alignment axis_align;
	struct f11_2d_sensor_query sens_query;
	struct f11_2d_data data;
	int prev_x[F11_MAX_NUM_OF_FINGERS];
	int prev_y[F11_MAX_NUM_OF_FINGERS];
	u16 max_x;
	u16 max_y;
	u8 nbr_fingers;
	u8 finger_tracker[F11_MAX_NUM_OF_FINGERS];
	u8 *data_pkt;
	int pkt_size;
	u8 sensor_index;
	u8 *button_map;
	struct rmi_f11_virtualbutton_map virtual_buttons;
	bool type_a;
	char input_name[MAX_NAME_LENGTH];
	char input_phys[MAX_NAME_LENGTH];
	struct input_dev *input;
	struct input_dev *mouse_input;
	struct rmi_function_container *fc;

#ifdef CONFIG_RMI4_DEBUG
	struct dentry *debugfs_flip;
	struct dentry *debugfs_clip;
	struct dentry *debugfs_delta_threshold;
	struct dentry *debugfs_offset;
	struct dentry *debugfs_swap;
	struct dentry *debugfs_type_a;
#endif
};

/* Accumulator of touch events for gesture determination.
 * Currently only targeted for taps but may be extended
 * for other gestures. */
struct goog_gesture_detect {
	int cnt;
	int max_x;
	int min_x;
	int max_y;
	int min_y;
	u8 max_fingers;
	unsigned long time_start;
	unsigned long time_end;
};

/* Finger cache used to pass information to driver gesture detector */
struct finger_cache_s {
	int x;
	int y;
};

struct f11_data {
	struct f11_2d_device_query dev_query;
	struct f11_2d_ctrl dev_controls;
	struct mutex dev_controls_mutex;
	u16 rezero_wait_ms;
	struct f11_2d_sensor sensors[F11_MAX_NUM_OF_SENSORS];

	struct {
		/* Boolean to determine if we are in early suspend or not. */
		int early_suspended;
		/* Cached value to determine if we hit suspend or not. */
		int last_suspend_cnt;
		/* Number of synthesized events sent during a given suspend cycle. */
		int synth_events_sent;
		/* Gesture events cannot cross early suspend boundaries. */
		int early_tap;
		/* We only want to present a single press gesture per touch sequence. */
		int press;
		/* Number of fingers detected on current iteration. */
		unsigned int current_finger_pressed_cnt;
		/* Number of fingers detected on previous iteration. */
		unsigned int prev_finger_pressed_cnt;
#ifdef CONFIG_WAKELOCK
		/* Wakelock to prevent suspension while sensor data in transit. */
		struct wake_lock wakelock;
#endif
		/* Counter of events per movement sequence starting at first finger landing
		 * and ending with last finger lifting. */
		unsigned int movement_event_cnt;
		/* Counter of individual finger events per movement sequence. */
		unsigned int movement_finger_cnt[F11_MAX_NUM_OF_FINGERS];
		/* A cache of finger position per event timeslice for gesture determination */
		struct finger_cache_s finger_cache[F11_MAX_NUM_OF_FINGERS];
		/* Gesture detector accumulator. */
		struct goog_gesture_detect gesture_detect;
		/* boolean to enable or disable gesture detect. */
		int goog_gesture_enable;
		/* boolean to enable or disable viewfinder. */
		int goog_view_enable;
		/* Cached value of viewfinder input. */
		int goog_view_val;
		/* Dimensions of viewfinder rectangle. */
		int goog_view_min_y;
		int goog_view_max_y;
		int goog_view_min_x;
		int goog_view_max_x;
	} goog;
#ifdef CONFIG_RMI4_DEBUG
	struct dentry *debugfs_rezero_wait;
#endif
};

/* NOTE(GOOG) External reference to get suspend cycle count */
extern unsigned int get_suspend_cnt(void);
/* NOTE(GOOG) End external reference */

enum finger_state_values {
	F11_NO_FINGER	= 0x00,
	F11_PRESENT	= 0x01,
	F11_INACCURATE	= 0x02,
	F11_RESERVED	= 0x03
};

/* ctrl sysfs files */
show_store_union_struct_prototype(abs_pos_filt)
show_store_union_struct_prototype(z_touch_threshold)
show_store_union_struct_prototype(z_touch_hysteresis)

#ifdef CONFIG_RMI4_DEBUG

struct sensor_debugfs_data {
	bool done;
	struct f11_2d_sensor *sensor;
};

static int sensor_debug_open(struct inode *inodep, struct file *filp)
{
	struct sensor_debugfs_data *data;
	struct f11_2d_sensor *sensor = inodep->i_private;
	struct rmi_function_container *fc = sensor->fc;

	data = devm_kzalloc(&fc->dev, sizeof(struct sensor_debugfs_data),
		GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->sensor = sensor;
	filp->private_data = data;
	return 0;
}

static ssize_t flip_read(struct file *filp, char __user *buffer, size_t size,
		    loff_t *offset) {
	int retval;
	char local_buf[size];
	struct sensor_debugfs_data *data = filp->private_data;

	if (data->done)
		return 0;

	data->done = 1;

	retval = snprintf(local_buf, size, "%u %u\n",
			data->sensor->axis_align.flip_x,
			data->sensor->axis_align.flip_y);

	if (retval <= 0 || copy_to_user(buffer, local_buf, retval))
		return -EFAULT;

	return retval;
}

static ssize_t flip_write(struct file *filp, const char __user *buffer,
			   size_t size, loff_t *offset) {
	int retval;
	char local_buf[size];
	unsigned int new_X;
	unsigned int new_Y;
	struct sensor_debugfs_data *data = filp->private_data;

	retval = copy_from_user(local_buf, buffer, size);
	if (retval)
		return -EFAULT;

	retval = sscanf(local_buf, "%u %u", &new_X, &new_Y);
	if (retval != 2 || new_X > 1 || new_Y > 1)
		return -EINVAL;

	data->sensor->axis_align.flip_x = new_X;
	data->sensor->axis_align.flip_y = new_Y;

	return size;
}

static const struct file_operations flip_fops = {
	.owner = THIS_MODULE,
	.open = sensor_debug_open,
	.read = flip_read,
	.write = flip_write,
};


static ssize_t delta_threshold_read(struct file *filp, char __user *buffer,
		size_t size, loff_t *offset) {
	int retval;
	char local_buf[size];
	struct sensor_debugfs_data *data = filp->private_data;
	struct f11_data *f11 = data->sensor->fc->data;
	struct f11_2d_ctrl *ctrl = &f11->dev_controls;

	if (data->done)
		return 0;

	data->done = 1;

	retval = snprintf(local_buf, size, "%u %u\n",
			ctrl->ctrl0_9->delta_x_threshold,
			ctrl->ctrl0_9->delta_y_threshold);

	if (retval <= 0 || copy_to_user(buffer, local_buf, retval))
		return -EFAULT;

	return retval;
}

static ssize_t delta_threshold_write(struct file *filp,
		const char __user *buffer, size_t size, loff_t *offset) {
	int retval;
	char local_buf[size];
	unsigned int new_X, new_Y;
	u8 save_X, save_Y;
	int rc;
	struct sensor_debugfs_data *data = filp->private_data;
	struct f11_data *f11 = data->sensor->fc->data;
	struct f11_2d_ctrl *ctrl = &f11->dev_controls;
	struct rmi_device *rmi_dev =  data->sensor->fc->rmi_dev;

	retval = copy_from_user(local_buf, buffer, size);
	if (retval)
		return -EFAULT;

	retval = sscanf(local_buf, "%u %u", &new_X, &new_Y);
	/* NOTE(GOOG) Bugfix */
	if (retval != 2 || new_X < 1 || new_Y < 1)
		return -EINVAL;
	/* //NOTE(GOOG) */
	save_X = ctrl->ctrl0_9->delta_x_threshold;
	save_Y = ctrl->ctrl0_9->delta_y_threshold;

	ctrl->ctrl0_9->delta_x_threshold = new_X;
	ctrl->ctrl0_9->delta_y_threshold = new_Y;
	rc = rmi_write_block(rmi_dev,
			ctrl->ctrl0_9->address,
			ctrl->ctrl0_9->regs,
			sizeof(ctrl->ctrl0_9->regs));
	if (rc < 0) {
		dev_warn(&data->sensor->fc->dev,
			"Failed to write to delta_threshold. Code: %d.\n",
			rc);
		ctrl->ctrl0_9->delta_x_threshold = save_X;
		ctrl->ctrl0_9->delta_y_threshold = save_Y;
	}
	return size;
}

static const struct file_operations delta_threshold_fops = {
	.owner = THIS_MODULE,
	.open = sensor_debug_open,
	.read = delta_threshold_read,
	.write = delta_threshold_write,
};

static ssize_t offset_read(struct file *filp, char __user *buffer, size_t size,
		    loff_t *offset) {
	int retval;
	char local_buf[size];
	struct sensor_debugfs_data *data = filp->private_data;

	if (data->done)
		return 0;

	data->done = 1;
	retval = snprintf(local_buf, size, "%u %u\n",
			data->sensor->axis_align.offset_X,
			data->sensor->axis_align.offset_Y);

	if (retval <= 0 || copy_to_user(buffer, local_buf, retval))
		return -EFAULT;

	return retval;
}

static ssize_t offset_write(struct file *filp, const char __user *buffer,
			   size_t size, loff_t *offset)
{
	int retval;
	char local_buf[size];
	int new_X;
	int new_Y;
	struct sensor_debugfs_data *data = filp->private_data;

	retval = copy_from_user(local_buf, buffer, size);
	if (retval)
		return -EFAULT;
	retval = sscanf(local_buf, "%u %u", &new_X, &new_Y);
	if (retval != 2)
		return -EINVAL;

	data->sensor->axis_align.offset_X = new_X;
	data->sensor->axis_align.offset_Y = new_Y;

	return size;
}

static const struct file_operations offset_fops = {
	.owner = THIS_MODULE,
	.open = sensor_debug_open,
	.read = offset_read,
	.write = offset_write,
};

static ssize_t clip_read(struct file *filp, char __user *buffer, size_t size,
		    loff_t *offset) {
	int retval;
	char local_buf[size];
	struct sensor_debugfs_data *data = filp->private_data;

	if (data->done)
		return 0;

	data->done = 1;

	retval = snprintf(local_buf, size, "%u %u %u %u\n",
			data->sensor->axis_align.clip_X_low,
			data->sensor->axis_align.clip_X_high,
			data->sensor->axis_align.clip_Y_low,
			data->sensor->axis_align.clip_Y_high);

	if (retval <= 0 || copy_to_user(buffer, local_buf, retval))
		return -EFAULT;

	return retval;
}

static ssize_t clip_write(struct file *filp, const char __user *buffer,
			   size_t size, loff_t *offset)
{
	int retval;
	char local_buf[size];
	unsigned int new_X_low, new_X_high, new_Y_low, new_Y_high;
	struct sensor_debugfs_data *data = filp->private_data;

	retval = copy_from_user(local_buf, buffer, size);
	if (retval)
		return -EFAULT;

	retval = sscanf(local_buf, "%u %u %u %u",
		&new_X_low, &new_X_high, &new_Y_low, &new_Y_high);
	if (retval != 4)
		return -EINVAL;

	if (new_X_low >= new_X_high || new_Y_low >= new_Y_high)
		return -EINVAL;

	data->sensor->axis_align.clip_X_low = new_X_low;
	data->sensor->axis_align.clip_X_high = new_X_high;
	data->sensor->axis_align.clip_Y_low = new_Y_low;
	data->sensor->axis_align.clip_Y_high = new_Y_high;

	return size;
}

static const struct file_operations clip_fops = {
	.owner = THIS_MODULE,
	.open = sensor_debug_open,
	.read = clip_read,
	.write = clip_write,
};

static ssize_t swap_read(struct file *filp, char __user *buffer, size_t size,
		    loff_t *offset) {
	int retval;
	char local_buf[size];
	struct sensor_debugfs_data *data = filp->private_data;

	if (data->done)
		return 0;

	data->done = 1;

	retval = snprintf(local_buf, size, "%u\n",
			data->sensor->axis_align.swap_axes);

	if (retval <= 0 || copy_to_user(buffer, local_buf, retval))
		return -EFAULT;

	return retval;
}

static ssize_t swap_write(struct file *filp, const char __user *buffer,
			   size_t size, loff_t *offset)
{
	int retval;
	char local_buf[size];
	int new_value;
	struct sensor_debugfs_data *data = filp->private_data;

	retval = copy_from_user(local_buf, buffer, size);
	if (retval)
		return -EFAULT;
	retval = sscanf(local_buf, "%u", &new_value);
	if (retval != 1 || new_value > 1)
		return -EINVAL;

	data->sensor->axis_align.swap_axes = new_value;
	return size;
}

static const struct file_operations swap_fops = {
	.owner = THIS_MODULE,
	.open = sensor_debug_open,
	.read = swap_read,
	.write = swap_write,
};

static ssize_t type_a_read(struct file *filp, char __user *buffer, size_t size,
		    loff_t *offset) {
	int retval;
	char local_buf[size];
	struct sensor_debugfs_data *data = filp->private_data;

	if (data->done)
		return 0;

	data->done = 1;

	retval = snprintf(local_buf, size, "%u\n",
			data->sensor->type_a);

	if (retval <= 0 || copy_to_user(buffer, local_buf, retval))
		return -EFAULT;

	return retval;
}

static ssize_t type_a_write(struct file *filp, const char __user *buffer,
			   size_t size, loff_t *offset)
{
	int retval;
	char local_buf[size];
	int new_value;
	struct sensor_debugfs_data *data = filp->private_data;

	retval = copy_from_user(local_buf, buffer, size);
	if (retval)
		return -EFAULT;
	retval = sscanf(local_buf, "%u", &new_value);
	if (retval != 1 || new_value > 1)
		return -EINVAL;

	data->sensor->type_a = new_value;
	return size;
}

static const struct file_operations type_a_fops = {
	.owner = THIS_MODULE,
	.open = sensor_debug_open,
	.read = type_a_read,
	.write = type_a_write,
};


static int setup_sensor_debugfs(struct f11_2d_sensor *sensor)
{
	int retval = 0;
	char fname[MAX_NAME_LENGTH];
	struct rmi_function_container *fc = sensor->fc;
	struct rmi_device *rmi_dev = fc->rmi_dev;

	if (!fc->debugfs_root)
		return -ENODEV;

	retval = snprintf(fname, MAX_NAME_LENGTH, "flip.%d",
			  sensor->sensor_index);
	sensor->debugfs_flip = debugfs_create_file(fname, RMI_RW_ATTR,
				fc->debugfs_root, sensor, &flip_fops);
	if (!sensor->debugfs_flip)
		dev_warn(&rmi_dev->dev, "Failed to create debugfs %s.\n",
			 fname);

	retval = snprintf(fname, MAX_NAME_LENGTH, "clip.%d",
			  sensor->sensor_index);
	sensor->debugfs_clip = debugfs_create_file(fname, RMI_RW_ATTR,
				fc->debugfs_root, sensor, &clip_fops);
	if (!sensor->debugfs_clip)
		dev_warn(&rmi_dev->dev, "Failed to create debugfs %s.\n",
			 fname);

	retval = snprintf(fname, MAX_NAME_LENGTH, "delta_threshold.%d",
			  sensor->sensor_index);
	sensor->debugfs_delta_threshold = debugfs_create_file(fname, RMI_RW_ATTR,
				fc->debugfs_root, sensor,
				&delta_threshold_fops);
	if (!sensor->debugfs_delta_threshold)
		dev_warn(&rmi_dev->dev, "Failed to create debugfs %s.\n",
			 fname);

	retval = snprintf(fname, MAX_NAME_LENGTH, "offset.%d",
			  sensor->sensor_index);
	sensor->debugfs_offset = debugfs_create_file(fname, RMI_RW_ATTR,
				fc->debugfs_root, sensor, &offset_fops);
	if (!sensor->debugfs_offset)
		dev_warn(&rmi_dev->dev, "Failed to create debugfs %s.\n",
			 fname);

	retval = snprintf(fname, MAX_NAME_LENGTH, "swap.%d",
			  sensor->sensor_index);
	sensor->debugfs_swap = debugfs_create_file(fname, RMI_RW_ATTR,
				fc->debugfs_root, sensor, &swap_fops);
	if (!sensor->debugfs_swap)
		dev_warn(&rmi_dev->dev, "Failed to create debugfs %s.\n",
			 fname);

	retval = snprintf(fname, MAX_NAME_LENGTH, "type_a.%d",
			  sensor->sensor_index);
	sensor->debugfs_type_a = debugfs_create_file(fname, RMI_RW_ATTR,
				fc->debugfs_root, sensor, &type_a_fops);
	if (!sensor->debugfs_type_a)
		dev_warn(&rmi_dev->dev, "Failed to create debugfs %s.\n",
			 fname);

	return retval;
}

static void teardown_sensor_debugfs(struct f11_2d_sensor *sensor)
{
	if (sensor->debugfs_flip)
		debugfs_remove(sensor->debugfs_flip);

	if (sensor->debugfs_clip)
		debugfs_remove(sensor->debugfs_clip);

	if (sensor->debugfs_offset)
		debugfs_remove(sensor->debugfs_offset);

	if (sensor->debugfs_swap)
		debugfs_remove(sensor->debugfs_swap);

	if (sensor->debugfs_type_a)
		debugfs_remove(sensor->debugfs_type_a);
}

struct f11_debugfs_data {
	bool done;
	struct rmi_function_container *fc;
};

static int f11_debug_open(struct inode *inodep, struct file *filp)
{
	struct f11_debugfs_data *data;
	struct rmi_function_container *fc = inodep->i_private;

	data = devm_kzalloc(&fc->dev, sizeof(struct f11_debugfs_data),
		GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->fc = fc;
	filp->private_data = data;
	return 0;
}

static ssize_t rezero_wait_read(struct file *filp, char __user *buffer,
		size_t size, loff_t *offset) {
	int retval;
	char local_buf[size];
	struct f11_debugfs_data *data = filp->private_data;
	struct f11_data *f11 = data->fc->data;

	if (data->done)
		return 0;

	data->done = 1;

	retval = snprintf(local_buf, size, "%u\n", f11->rezero_wait_ms);

	if (retval <= 0 || copy_to_user(buffer, local_buf, retval))
		return -EFAULT;

	return retval;
}

static ssize_t rezero_wait_write(struct file *filp, const char __user *buffer,
			   size_t size, loff_t *offset)
{
	int retval;
	char local_buf[size];
	int new_value;
	struct f11_debugfs_data *data = filp->private_data;
	struct f11_data *f11 = data->fc->data;

	retval = copy_from_user(local_buf, buffer, size);
	if (retval)
		return -EFAULT;
	retval = sscanf(local_buf, "%u", &new_value);
	if (retval != 1 || new_value > 65535)
		return -EINVAL;

	f11->rezero_wait_ms = new_value;
	return size;
}

static const struct file_operations rezero_wait_fops = {
	.owner = THIS_MODULE,
	.open = f11_debug_open,
	.read = rezero_wait_read,
	.write = rezero_wait_write,
};

static int setup_f11_debugfs(struct rmi_function_container *fc)
{
	struct f11_data *f11 = fc->data;

	if (!fc->debugfs_root)
		return -ENODEV;

	f11->debugfs_rezero_wait = debugfs_create_file("rezero_wait",
		RMI_RW_ATTR, fc->debugfs_root, fc, &rezero_wait_fops);
	if (!f11->debugfs_rezero_wait)
		dev_warn(&fc->dev,
			 "Failed to create debugfs rezero_wait.\n");

	return 0;
}

static void teardown_f11_debugfs(struct f11_data *f11)
{
	if (f11->debugfs_rezero_wait)
		debugfs_remove(f11->debugfs_rezero_wait);
}
#endif
/* End adding debugfs */

/* This is a group in case we add the other ctrls. */
static struct attribute *attrs_ctrl0[] = {
	attrify(abs_pos_filt),
	NULL
};
static struct attribute_group attrs_control0 = GROUP(attrs_ctrl0);

static struct attribute *attrs_ctrl29_30[] = {
	attrify(z_touch_threshold),
	attrify(z_touch_hysteresis),
	NULL
};
static struct attribute_group attrs_control29_30 = GROUP(attrs_ctrl29_30);

/** F11_INACCURATE state is overloaded to indicate pen present. */
#define F11_PEN F11_INACCURATE

static int get_tool_type(struct f11_2d_sensor *sensor, u8 finger_state)
{
	if (IS_ENABLED(CONFIG_RMI4_F11_PEN) &&
			sensor->sens_query.query9.has_pen &&
			finger_state == F11_PEN)
		return MT_TOOL_PEN;
	return MT_TOOL_FINGER;
}

static void rmi_f11_rel_pos_report(struct f11_2d_sensor *sensor, u8 n_finger)
{
	struct f11_2d_data *data = &sensor->data;
	struct rmi_f11_2d_axis_alignment *axis_align = &sensor->axis_align;
	s8 x, y;
	s8 temp;

	x = data->rel_pos[n_finger].delta_x;
	y = data->rel_pos[n_finger].delta_y;

	x = min(F11_REL_POS_MAX, max(F11_REL_POS_MIN, (int)x));
	y = min(F11_REL_POS_MAX, max(F11_REL_POS_MIN, (int)y));

	if (axis_align->swap_axes) {
		temp = x;
		x = y;
		y = temp;
	}
	if (axis_align->flip_x)
		x = min(F11_REL_POS_MAX, -x);
	if (axis_align->flip_y)
		y = min(F11_REL_POS_MAX, -y);

	if (x || y) {
		input_report_rel(sensor->input, REL_X, x);
		input_report_rel(sensor->input, REL_Y, y);
		input_report_rel(sensor->mouse_input, REL_X, x);
		input_report_rel(sensor->mouse_input, REL_Y, y);
	}
	input_sync(sensor->mouse_input);
}

static void rmi_f11_abs_pos_report(struct f11_data *f11,
				   struct f11_2d_sensor *sensor,
				   u8 finger_state, u8 n_finger)
{
	struct f11_2d_data *data = &sensor->data;
	struct rmi_f11_2d_axis_alignment *axis_align = &sensor->axis_align;
	u8 prev_state = sensor->finger_tracker[n_finger];
	int x, y, z;
	int w_x, w_y, w_max, w_min, orient;
	int temp;

	if (prev_state && !finger_state) {
		/* this is a release */
		x = y = z = w_max = w_min = orient = 0;
		/*  NOTE(GOOG)
		 * this is a release. Android 4.0 specifies that
		 * the input device should simply stop sending
		 * data.  Otherwise it will get interpreted as
		 * a hover event.*/
		return;
		/* //NOTE(GOOG) */
	} else if (!prev_state && !finger_state) {
		/* nothing to report */
		return;
	} else {
		x = ((data->abs_pos[n_finger].x_msb << 4) |
			data->abs_pos[n_finger].x_lsb);
		y = ((data->abs_pos[n_finger].y_msb << 4) |
			data->abs_pos[n_finger].y_lsb);
		z = data->abs_pos[n_finger].z;
		w_x = data->abs_pos[n_finger].w_x;
		w_y = data->abs_pos[n_finger].w_y;
		w_max = max(w_x, w_y);
		w_min = min(w_x, w_y);

		if (axis_align->swap_axes) {
			temp = x;
			x = y;
			y = temp;
			temp = w_x;
			w_x = w_y;
			w_y = temp;
		}

		orient = w_x > w_y ? 1 : 0;

		if (axis_align->flip_x)
			x = max(sensor->max_x - x, 0);

		if (axis_align->flip_y)
			y = max(sensor->max_y - y, 0);

		/*
		** here checking if X offset or y offset are specified is
		**  redundant.  We just add the offsets or, clip the values
		**
		** note: offsets need to be done before clipping occurs,
		** or we could get funny values that are outside
		** clipping boundaries.
		*/
		x += axis_align->offset_X;
		y += axis_align->offset_Y;
		x =  max(axis_align->clip_X_low, x);
		y =  max(axis_align->clip_Y_low, y);
		if (axis_align->clip_X_high)
			x = min(axis_align->clip_X_high, x);
		if (axis_align->clip_Y_high)
			y =  min(axis_align->clip_Y_high, y);

	}

	/* Some UIs ignore W of zero, so we fudge it to 1 for pens. */
	if (IS_ENABLED(CONFIG_RMI4_F11_PEN) &&
			get_tool_type(sensor, finger_state) == MT_TOOL_PEN) {
		w_max = max(1, w_max);
		w_min = max(1, w_min);
	}

	if (sensor->type_a) {
		input_report_abs(sensor->input, ABS_MT_TRACKING_ID, n_finger);
		input_report_abs(sensor->input, ABS_MT_TOOL_TYPE,
					get_tool_type(sensor, finger_state));
	} else {
		input_mt_slot(sensor->input, n_finger);
		input_mt_report_slot_state(sensor->input,
			get_tool_type(sensor, finger_state), finger_state);
	}

	input_report_abs(sensor->input, ABS_MT_PRESSURE, z);
	input_report_abs(sensor->input, ABS_MT_TOUCH_MAJOR, w_max);
	input_report_abs(sensor->input, ABS_MT_TOUCH_MINOR, w_min);
	input_report_abs(sensor->input, ABS_MT_ORIENTATION, orient);
	input_report_abs(sensor->input, ABS_MT_POSITION_X, x);
	input_report_abs(sensor->input, ABS_MT_POSITION_Y, y);
	dev_dbg(&sensor->fc->dev,
		"finger[%d]:%d - x:%d y:%d z:%d w_max:%d w_min:%d\n",
		n_finger, finger_state, x, y, z, w_max, w_min);
	/* MT sync between fingers */
	if (sensor->type_a)
		input_mt_sync(sensor->input);

	/* Cache the finger location for this movement event
	   for the gesture detector accumulator. */
	f11->goog.finger_cache[n_finger].x = x;
	f11->goog.finger_cache[n_finger].y = y;

	sensor->finger_tracker[n_finger] = finger_state;
}

#ifdef CONFIG_RMI4_VIRTUAL_BUTTON
static int rmi_f11_virtual_button_handler(struct f11_2d_sensor *sensor)
{
	int i;
	int x;
	int y;
	struct rmi_button_map *virtualbutton_map;

	if (sensor->sens_query.has_gestures &&
				sensor->data.gest_1->single_tap) {
		virtualbutton_map = &sensor->virtualbutton_map;
		x = ((sensor->data.abs_pos[0].x_msb << 4) |
			sensor->data.abs_pos[0].x_lsb);
		y = ((sensor->data.abs_pos[0].y_msb << 4) |
			sensor->data.abs_pos[0].y_lsb);
		for (i = 0; i < virtualbutton_map->buttons; i++) {
			if (INBOX(x, y, virtualbutton_map->map[i])) {
				input_report_key(sensor->input,
					virtualbutton_map->map[i].code, 1);
				input_report_key(sensor->input,
					virtualbutton_map->map[i].code, 0);
				input_sync(sensor->input);
				return 0;
			}
		}
	}
	return 0;
}
#else
#define rmi_f11_virtual_button_handler(sensor)
#endif

#ifdef DEBUG_GESTURES
#define BYTES_LEFT(a,b) (sizeof(a)-(b-a))

/* NOTE(GOOG) This is a debug function to understand how the touchpad
 * responds to gestures.
 */
static void rmi_f11_debug_gestures(struct f11_data *f11,
                                   struct f11_2d_sensor *sensor)
{
	const struct f11_2d_data *data = &sensor->data;
	char buf[255] = {0};
	char *pbuf = buf;
	if (data->gest_1->single_tap == 1) {
		pbuf += snprintf(pbuf, BYTES_LEFT(buf, pbuf), "SingleTap ");
	}
	if (data->gest_1->tap_and_hold == 1) {
		pbuf += snprintf(pbuf, BYTES_LEFT(buf, pbuf), "TapAndHold ");
	}
	if (data->gest_1->double_tap == 1) {
		pbuf += snprintf(pbuf, BYTES_LEFT(buf, pbuf), "DoubleTap ");
	}
	if (data->gest_1->early_tap == 1) {
		pbuf += snprintf(pbuf, BYTES_LEFT(buf, pbuf), "EarlyTap ");
	}
	if (data->gest_1->flick == 1) {
		pbuf += snprintf(pbuf, BYTES_LEFT(buf, pbuf), "Flick x:%d y:%d time:%d ",
		                data->flick->x_flick_dist,
		                data->flick->y_flick_dist,
		                data->flick->flick_time);
	}
	if (data->gest_1->press == 1) {
		pbuf += snprintf(pbuf, BYTES_LEFT(buf, pbuf), "Press ");
	}
	if (data->gest_1->pinch == 1) {
		pbuf += snprintf(pbuf, BYTES_LEFT(buf, pbuf), "Pinch ");
	}
	if (data->gest_2->palm_detect == 1) {
		pbuf += snprintf(pbuf, BYTES_LEFT(buf, pbuf), "PalmDetect ");
	}
	if (data->gest_2->rotate == 1) {
		pbuf += snprintf(pbuf, BYTES_LEFT(buf, pbuf), "Rotate ");
	}
	if (data->gest_2->shape == 1) {
		pbuf += snprintf(pbuf, BYTES_LEFT(buf, pbuf), "Shape ");
	}
	if (data->gest_2->scrollzone == 1) {
		pbuf += snprintf(pbuf, BYTES_LEFT(buf, pbuf), "ScrollZone ");
	}
	if (data->gest_2->finger_count) {
		pbuf += snprintf(pbuf, BYTES_LEFT(buf, pbuf), "FingerCount:%d ",
		                 data->gest_2->finger_count);
	}
	if (buf != pbuf) {
		printk("%s Debug Gestures:%s\n", __func__, buf);
	}
}
#endif  /* DEBUG_GESTURES */

/*
 * NOTE(GOOG) Google specific function.
 * Determines if we are actively in early suspend state, a.k.a. screen-off state.
 * Returns: 1 Early suspend is currently active
 *          0 Early suspend is not active.
 */
static int early_suspend_active(struct f11_data *f11)
{
        if (f11->goog.early_suspended) {
                return 1;
        }
        return 0;
}

/*
 * NOTE(GOOG) Google specific function.
 * Determines if we immediately came from a suspend state.
 * Returns: 1 We just came from a suspend state.
 *          0 We did not just come from a suspend state.
 */
static int from_suspend_active(struct f11_data *f11)
{
        if (f11->goog.last_suspend_cnt != get_suspend_cnt()) {
                return 1;
        }
        return 0;
}

/* NOTE(GOOG) Google specific function.
 * Generates an input gesture based upon unique rules specific
 * to Google product.
 * name: Gesture name for logging.
 * x_offset: Artificial x-offset to encode gesture for Android layer.
 * y_offset: Artificial y-offset to encode gesture for Android layer.
 */
static void rmi_f11_input_gesture(struct f11_data *f11,
                                  struct f11_2d_sensor *sensor,
                                  const char *name,
                                  int32_t x_offset, int32_t y_offset)
{
#if defined(ABS_MT_PRESSURE)
	/* We have to supply the minimum values required to get event through
	   Android InputEvent layer */
	input_report_abs(sensor->input, ABS_MT_PRESSURE, GESTURE_PRESSURE);
#endif
	/* Add well-known gesture offset to indicate it's not a real touchpad point */
	input_report_abs(sensor->input, ABS_MT_POSITION_X, GESTURE_OFFSET_X + x_offset);
	input_report_abs(sensor->input, ABS_MT_POSITION_Y, GESTURE_OFFSET_Y + y_offset);

	/* Add well-known major and minor numbers to indicate it's not a real touchpad point */
	input_report_abs(sensor->input, ABS_MT_TOUCH_MAJOR, GESTURE_MT_MAJOR);
	input_report_abs(sensor->input, ABS_MT_TOUCH_MINOR, GESTURE_MT_MINOR);

	/* Close out multi touch sequence and entire motion sequence */
	input_mt_sync(sensor->input);
	input_sync(sensor->input); /* sync after groups of events */

	/* If there are no fingers, and last time through there were no fingers, we
	 * need to send an empty mt sync packet with empty mt sync to indicate all fingers
	 * have left the touchpad.  We send the report sync later. */
	if ((f11->goog.current_finger_pressed_cnt == 0) && (f11->goog.prev_finger_pressed_cnt == 0)) {
		input_mt_sync(sensor->input);
	}

	/* Keep track of count of synthesized keys per suspend cycle. */
	f11->goog.synth_events_sent++;

	dev_info(&sensor->fc->dev, "%s Created synthesized movement:%s"
	         " event_cnt:%d\n", __func__, name,
	         f11->goog.synth_events_sent);

	/* Set a wakelock to prevent gesture from getting stuck in input layer
	 * during a suspend cycle. */
	wake_lock_timeout(&f11->goog.wakelock, msecs_to_jiffies(WAKELOCK_TIMEOUT_IN_MS));
}

/* Reset the gesture detector accumulator */
static void goog_gesture_detect_reset(struct goog_gesture_detect *ggd)
{
	memset(ggd, 0, sizeof(*ggd));
	// Set max and min to extreme initialized values.
	ggd->max_x = INT_MIN;
	ggd->max_y = INT_MIN;
	ggd->min_x = INT_MAX;
	ggd->min_y = INT_MAX;
}

/* Accumulate events for gesture determiniation */
static void goog_gesture_accumulate(struct goog_gesture_detect *ggd,
                                    struct finger_cache_s *finger_cache,
                                    u8 finger_pressed_count)
{
	/* Accumulate event data for gesture detector. */
	ggd->cnt += 1;
	/* Update the bounding box for this movement sequence */
	ggd->max_x = max(ggd->max_x, finger_cache[0].x);
	ggd->min_x = min(ggd->min_x, finger_cache[0].x);
	ggd->max_y = max(ggd->max_y, finger_cache[0].y);
	ggd->min_y = min(ggd->min_y, finger_cache[0].y);
	ggd->max_fingers = max(ggd->max_fingers, finger_pressed_count);
	/* Set the start time of this gesture. */
	if (ggd->time_start == 0) {
		ggd->time_start = jiffies;
	}
}

static void goog_gesture_handler(struct f11_data *f11,
                                 struct goog_gesture_detect *ggd) {
	/* Convenience field for gesture length in msecs. */
	unsigned int delta_msec;

	/* We only send one driver gestures per suspend/resume cycle */
	if (f11->goog.synth_events_sent > 0) {
		return;
	}

	delta_msec = jiffies_to_msecs(ggd->time_end - ggd->time_start);
	/* Driver synthesized single tap gesture model.
	   All touch events must occur within a bounding box, must have at
	   least 2 events, must only present 1 finger and all must occur within
	   a specified timeslice as defined as a "tap". */
	if (((ggd->max_x - ggd->min_x) < gesture_detect_tap_box_x)
	    && ((ggd->max_y - ggd->min_y) < gesture_detect_tap_box_y)
	    && (ggd->cnt > 1)
	    && (ggd->max_fingers == 1)
	    && (delta_msec > gesture_detect_tap_time_min)
	    && (delta_msec < gesture_detect_tap_time_max)) {
		/* NOTE() We specify the first sensor for gesture */
		rmi_f11_input_gesture(f11, &f11->sensors[0],
		                      "driver single tap",
		                      GESTURE_OFFSET_SINGLE_TAP_X,
		                      GESTURE_OFFSET_SINGLE_TAP_Y);
		/* Consume the early tap to prevent the real gesture
		   from being sent by the touchpad device. */
		f11->goog.early_tap = 0;
	} else {
		dev_info(&f11->sensors[0].fc->dev, "%s Ignoring driver"
		         " synthesized single tap gesture\n", __func__);
		dev_dbg(&f11->sensors[0].fc->dev, "%s max_x:%d min_x:%d"
		        " max_y:%d min_y:%d cnt:%d fgr:%d time:%d s:%ld"
		        " e:%ld\n", __func__, ggd->max_x, ggd->min_x,
		        ggd->max_y, ggd->min_y, ggd->cnt, ggd->max_fingers,
		        delta_msec, ggd->time_start, ggd->time_end);
	}
	goog_gesture_detect_reset(ggd);
}

/* NOTE(GOOG) This has been modified by Google */
static void rmi_f11_finger_handler(struct f11_data *f11,
				   struct f11_2d_sensor *sensor)
{
	const u8 *f_state = sensor->data.f_state;
	const struct f11_2d_data *data = &sensor->data;
	u8 finger_state;
	u8 finger_pressed_count;
	int finger_view_count;
	u8 i;

	for (i = 0, finger_pressed_count = 0; i < sensor->nbr_fingers; i++) {
		/* Possible of having 4 fingers per f_statet register */
		finger_state = GET_FINGER_STATE(f_state, i);

		if (finger_state == F11_RESERVED) {
			dev_err(&sensor->fc->dev, "%s: Invalid finger"
			        " state[%d]:0x%02x.", __func__, i,
			        finger_state);
			continue;
		} else if ((finger_state == F11_PRESENT) ||
		           (finger_state == F11_INACCURATE)) {
			/* Peek at the XY value */
			int x_peek = ((data->abs_pos[i].x_msb << 4) | data->abs_pos[i].x_lsb);
			int y_peek = ((data->abs_pos[i].y_msb << 4) | data->abs_pos[i].y_lsb);

			if (sensor->axis_align.flip_x)
				x_peek = max(sensor->max_x - x_peek, 0);
			if (sensor->axis_align.flip_y)
				y_peek = max(sensor->max_y - y_peek, 0);

			/* If enabled and a finger lands inside
			   the viewfinder cutout, ignore it for touch
			   purposes. */
			if (f11->goog.goog_view_enable
			    && (f11->goog.movement_finger_cnt[i] == 0)
			    && (y_peek >= f11->goog.goog_view_min_y)
			    && (y_peek <= f11->goog.goog_view_max_y)
			    && (x_peek >= f11->goog.goog_view_min_x)
			    && (x_peek <= f11->goog.goog_view_max_x)) {
				/* one finger in specified region indicates
				   possible viewfinder action. */
				dev_dbg(&sensor->fc->dev, "%s Viewfinder"
				        " possibly detected view_cnt:%d\n",
				        __func__, finger_view_count);
				finger_view_count++;
				continue;
			} else {
				finger_pressed_count++;
				f11->goog.movement_finger_cnt[i]++;
			}
		}

		if (sensor->data.abs_pos)
			rmi_f11_abs_pos_report(f11, sensor, finger_state, i);

		if (sensor->data.rel_pos)
			rmi_f11_rel_pos_report(sensor, i);
	}

	if (f11->goog.goog_view_enable) {
		if ((finger_view_count == 1) && (finger_pressed_count == 0)) {
			/* Handle start of viewfinder */
			/* Send switch command.  The input queue will coalesce
			   duplicates. */
			f11->goog.goog_view_val = 1;
			input_report_switch(sensor->input, SW_CAMERA_LENS_COVER,
			                    f11->goog.goog_view_val);
			dev_dbg(&sensor->fc->dev, "%s Viewfinder detected\n",
			        __func__);
		} else {
			/* Handle release of viewfinder */
			f11->goog.goog_view_val = 0;
			input_report_switch(sensor->input, SW_CAMERA_LENS_COVER,
			                    f11->goog.goog_view_val);
			dev_dbg(&sensor->fc->dev, "%s Viewfinder released\n",
			        __func__);
		}
	}

	f11->goog.current_finger_pressed_cnt = finger_pressed_count;
	/* If we were suspended we will pass device detected gestures.
	   Unless viewfinder is enabled and there were no finger
	   events detected outside the viewfinder range. */
	if (early_suspend_active(f11) && (finger_view_count != 1)) {
		if (data->gest_1->flick == 1) {
			rmi_f11_input_gesture(f11, sensor,
			                      "flick",
			                      data->flick->x_flick_dist,
			                      data->flick->y_flick_dist);
		}
		if (data->gest_1->early_tap == 1) {
			f11->goog.early_tap = 1;
		}
		if (data->gest_1->single_tap == 1) {
			if (f11->goog.early_tap == 0) {
				dev_info(&sensor->fc->dev, "%s Rejecting single"
				         " tap gesture with no previous early"
				         " tap\n", __func__);
			} else {
				rmi_f11_input_gesture(f11, sensor,
				                      "single tap",
				                      GESTURE_OFFSET_SINGLE_TAP_X,
				                      GESTURE_OFFSET_SINGLE_TAP_Y);
			}
		}
		if (data->gest_1->double_tap == 1) {
			if (f11->goog.early_tap == 0) {
				dev_info(&sensor->fc->dev, "%s Rejecting double"
				         " tap gesture with no previous early"
				         " tap\n", __func__);
			} else {
				rmi_f11_input_gesture(f11, sensor,
				                      "double tap",
				                      GESTURE_OFFSET_DOUBLE_TAP_X,
				                      GESTURE_OFFSET_DOUBLE_TAP_Y);
			}
		}
		if (data->gest_1->press == 1) {
			if (f11->goog.press++) {
#ifdef DEBUG_GESTURES
				/* This log message is especially verbose because the gesture
				   is sent during a real finger event so bracket it in
				   gesture debug logging. */
				dev_info(&sensor->fc->dev, "%s Rejecting"
				         " multiple press gesture cnt:%d\n",
				         __func__, f11->goog.press);
#endif  /* DEBUG_GESTURES */
			} else {
				/* A press gesture may be generated along with a real finger
				 * sequence.  This would present as multiple fingers without
				 * unless a sync separates them.
				 *
				 * So if we have real fingers data, close out the event packet
				 * here before starting a new one with the press gesture.
				 *
				 * The long press gesture presents with multiple fingers landed
				 * but for our application we want to constrain it to one finger.
				 */
				if (finger_pressed_count == 1) {
					input_sync(sensor->input); /* sync after groups of events */
					rmi_f11_input_gesture(f11, sensor,
					                      "press",
					                      GESTURE_OFFSET_PRESS_X,
					                      GESTURE_OFFSET_PRESS_Y);
				}
			}
		}

		/* Driver gesture detect model */
		if (f11->goog.goog_gesture_enable) {
			goog_gesture_accumulate(&f11->goog.gesture_detect,
			                        f11->goog.finger_cache,
			                        finger_pressed_count);
		}
	}

	/* Debugging logging */
	if (f11->goog.prev_finger_pressed_cnt == 0 && f11->goog.current_finger_pressed_cnt != 0) {
		dev_info(&sensor->fc->dev, "%s Starting movement event"
		         " cnt:%d\n", __func__, f11->goog.movement_event_cnt);
	}
	if (f11->goog.prev_finger_pressed_cnt != 0 && f11->goog.current_finger_pressed_cnt == 0) {
		/* Save the time from when the last finger lifted.
		   Since this is has a strong time element we record
		   time value before the potentially long printk command. */
		f11->goog.gesture_detect.time_end = jiffies;

		dev_info(&sensor->fc->dev, "%s Ending movement event cnt:%d"
		         " fing0:%d fing1:%d fing2:%d\n", __func__,
		         f11->goog.movement_event_cnt,
		         f11->goog.movement_finger_cnt[0],
		         f11->goog.movement_finger_cnt[1],
		         f11->goog.movement_finger_cnt[2]);
		f11->goog.movement_event_cnt = 0;
		memset(f11->goog.movement_finger_cnt, 0, sizeof(f11->goog.movement_finger_cnt));

		/* Additional work if we have driver gesture detector enabled */
		if (f11->goog.goog_gesture_enable) {
			goog_gesture_handler(f11, &f11->goog.gesture_detect);
		}
	}
	if (f11->goog.prev_finger_pressed_cnt == 0 && f11->goog.current_finger_pressed_cnt == 0) {
		dev_dbg(&sensor->fc->dev, "%s Extraneous event\n", __func__);
	} else {
		f11->goog.movement_event_cnt++;
	}

	if (f11->goog.current_finger_pressed_cnt) {
		/* If any fingers are landed set a wakelock to prevent movement
		   information from getting stuck in the input queue. */
		wake_lock_timeout(&f11->goog.wakelock, msecs_to_jiffies(WAKELOCK_TIMEOUT_IN_MS));
	} else {
		/* An empty report sync will be consumed by the input layer.
		 * So to get an empty report sync passed the input layer
		 * we will provide an empty config sync.  Only do this if the
		 * previous run through here had finger data.
		 */
		if (f11->goog.prev_finger_pressed_cnt != 0) {
			input_mt_sync(sensor->input);
		}
		/* Reset the gesture long press since all fingers have lifted. */
		f11->goog.press = 0;
	}

	/* Update the previous finger pressed count. */
	f11->goog.prev_finger_pressed_cnt = f11->goog.current_finger_pressed_cnt;

	input_sync(sensor->input);
}

static int f11_2d_construct_data(struct f11_2d_sensor *sensor)
{
	struct f11_2d_sensor_query *query = &sensor->sens_query;
	struct f11_2d_data *data = &sensor->data;
	int i;

	sensor->nbr_fingers = (query->number_of_fingers == 5 ? 10 :
				query->number_of_fingers + 1);

	sensor->pkt_size = F11_CEIL(sensor->nbr_fingers, 4);

	if (query->has_abs)
		sensor->pkt_size += (sensor->nbr_fingers * 5);

	if (query->has_rel)
		sensor->pkt_size +=  (sensor->nbr_fingers * 2);

	/* Check if F11_2D_Query7 is non-zero */
	if (query->f11_2d_query7__8[0])
		sensor->pkt_size += sizeof(u8);

	/* Check if F11_2D_Query7 or F11_2D_Query8 is non-zero */
	if (query->f11_2d_query7__8[0] || query->f11_2d_query7__8[1])
		sensor->pkt_size += sizeof(u8);

	if (query->has_pinch || query->has_flick || query->has_rotate) {
		sensor->pkt_size += 3;
		if (!query->has_flick)
			sensor->pkt_size--;
		if (!query->has_rotate)
			sensor->pkt_size--;
	}

	if (query->has_touch_shapes)
		sensor->pkt_size += F11_CEIL(query->nbr_touch_shapes + 1, 8);

	sensor->data_pkt = kzalloc(sensor->pkt_size, GFP_KERNEL);
	if (!sensor->data_pkt)
		return -ENOMEM;

	data->f_state = sensor->data_pkt;
	i = F11_CEIL(sensor->nbr_fingers, 4);

	if (query->has_abs) {
		data->abs_pos = (struct f11_2d_data_1_5 *)
				&sensor->data_pkt[i];
		i += (sensor->nbr_fingers * 5);
	}

	if (query->has_rel) {
		data->rel_pos = (struct f11_2d_data_6_7 *)
				&sensor->data_pkt[i];
		i += (sensor->nbr_fingers * 2);
	}

	if (query->f11_2d_query7__8[0]) {
		data->gest_1 = (struct f11_2d_data_8 *)&sensor->data_pkt[i];
		i++;
	}

	if (query->f11_2d_query7__8[0] || query->f11_2d_query7__8[1]) {
		data->gest_2 = (struct f11_2d_data_9 *)&sensor->data_pkt[i];
		i++;
	}

	if (query->has_pinch) {
		data->pinch = (struct f11_2d_data_10 *)&sensor->data_pkt[i];
		i++;
	}

	if (query->has_flick) {
		if (query->has_pinch) {
			data->flick = (struct f11_2d_data_10_12 *)data->pinch;
			i += 2;
		} else {
			data->flick = (struct f11_2d_data_10_12 *)
					&sensor->data_pkt[i];
			i += 3;
		}
	}

	if (query->has_rotate) {
		if (query->has_flick) {
			data->rotate = (struct f11_2d_data_11_12 *)
					(data->flick + 1);
		} else {
			data->rotate = (struct f11_2d_data_11_12 *)
					&sensor->data_pkt[i];
			i += 2;
		}
	}

	if (query->has_touch_shapes)
		data->shapes = (struct f11_2d_data_13 *)&sensor->data_pkt[i];

	return 0;
}

static int f11_read_control_regs(struct rmi_device *rmi_dev,
					   struct f11_2d_ctrl *ctrl,
					   u16 ctrl_base_addr) {
	u16 read_address = ctrl_base_addr;
	int error = 0;

	ctrl->ctrl0_9->address = read_address;
	error = rmi_read_block(rmi_dev, read_address, ctrl->ctrl0_9->regs,
		sizeof(ctrl->ctrl0_9->regs));
	if (error < 0) {
		dev_err(&rmi_dev->dev,
			"Failed to read F11 ctrl0, code: %d.\n", error);
		return error;
	}
	read_address = read_address + sizeof(ctrl->ctrl0_9->regs);

	if (ctrl->ctrl10) {
		error = rmi_read_block(rmi_dev, read_address,
			&ctrl->ctrl10->reg, sizeof(union f11_2d_ctrl10));
		if (error < 0) {
			dev_err(&rmi_dev->dev,
				"Failed to read F11 ctrl10, code: %d.\n",
				error);
			return error;
		}
		read_address = read_address + sizeof(union f11_2d_ctrl10);
	}

	if (ctrl->ctrl11) {
		error = rmi_read_block(rmi_dev, read_address,
			&ctrl->ctrl11->reg, sizeof(union f11_2d_ctrl11));
		if (error < 0) {
			dev_err(&rmi_dev->dev,
				"Failed to read F11 ctrl11, code: %d.\n",
				error);
			return error;
		}
		read_address = read_address + sizeof(union f11_2d_ctrl11);
	}

	if (ctrl->ctrl14) {
		error = rmi_read_block(rmi_dev, read_address,
			&ctrl->ctrl14->reg, sizeof(union f11_2d_ctrl14));
		if (error < 0) {
			dev_err(&rmi_dev->dev,
				"Failed to read F11 ctrl14, code: %d.\n",
				error);
			return error;
		}
		read_address = read_address + sizeof(union f11_2d_ctrl14);
	}

	if (ctrl->ctrl15) {
		error = rmi_read_block(rmi_dev, read_address,
			&ctrl->ctrl15->reg, sizeof(union f11_2d_ctrl15));
		if (error < 0) {
			dev_err(&rmi_dev->dev,
				"Failed to read F11 ctrl15, code: %d.\n",
				error);
			return error;
		}
		read_address = read_address + sizeof(union f11_2d_ctrl15);
	}

	if (ctrl->ctrl16) {
		error = rmi_read_block(rmi_dev, read_address,
			&ctrl->ctrl16->reg, sizeof(union f11_2d_ctrl16));
		if (error < 0) {
			dev_err(&rmi_dev->dev,
				"Failed to read F11 ctrl16, code: %d.\n",
				error);
			return error;
		}
		read_address = read_address + sizeof(union f11_2d_ctrl16);
	}

	if (ctrl->ctrl17) {
		error = rmi_read_block(rmi_dev, read_address,
			&ctrl->ctrl17->reg, sizeof(union f11_2d_ctrl17));
		if (error < 0) {
			dev_err(&rmi_dev->dev,
				"Failed to read F11 ctrl17, code: %d.\n",
				error);
			return error;
		}
		read_address = read_address + sizeof(union f11_2d_ctrl17);
	}

	if (ctrl->ctrl18_19) {
		error = rmi_read_block(rmi_dev, read_address,
			ctrl->ctrl18_19->reg, sizeof(union f11_2d_ctrl18_19));
		if (error < 0) {
			dev_err(&rmi_dev->dev,
				"Failed to read F11 ctrl18_19, code: %d.\n",
				error);
			return error;
		}
		read_address = read_address + sizeof(union f11_2d_ctrl18_19);
	}

	if (ctrl->ctrl20_21) {
		error = rmi_read_block(rmi_dev, read_address,
			ctrl->ctrl20_21->reg, sizeof(union f11_2d_ctrl20_21));
		if (error < 0) {
			dev_err(&rmi_dev->dev,
				"Failed to read F11 ctrl20_21, code: %d.\n",
				error);
			return error;
		}
		read_address = read_address + sizeof(union f11_2d_ctrl20_21);
	}

	if (ctrl->ctrl22_26) {
		error = rmi_read_block(rmi_dev, read_address,
			ctrl->ctrl22_26->regs, sizeof(union f11_2d_ctrl22_26));
		if (error < 0) {
			dev_err(&rmi_dev->dev,
				"Failed to read F11 ctrl22_26, code: %d.\n",
				error);
			return error;
		}
		read_address = read_address + sizeof(union f11_2d_ctrl22_26);
	}

	if (ctrl->ctrl27) {
		error = rmi_read_block(rmi_dev, read_address,
			ctrl->ctrl27->regs, sizeof(union f11_2d_ctrl27));
		if (error < 0) {
			dev_err(&rmi_dev->dev,
				"Failed to read F11 ctrl27, code: %d.\n",
				error);
			return error;
		}
		read_address = read_address + sizeof(union f11_2d_ctrl27);
	}

	if (ctrl->ctrl28) {
		error = rmi_read_block(rmi_dev, read_address,
			ctrl->ctrl28->regs, sizeof(union f11_2d_ctrl28));
		if (error < 0) {
			dev_err(&rmi_dev->dev,
				"Failed to read F11 ctrl28, code: %d.\n",
				error);
			return error;
		}
		read_address = read_address + sizeof(union f11_2d_ctrl28);
	}

	if (ctrl->ctrl29_30) {
		ctrl->ctrl29_30->address = read_address;
		error = rmi_read_block(rmi_dev, read_address,
			ctrl->ctrl29_30->regs, sizeof(ctrl->ctrl29_30->regs));
		if (error < 0) {
			dev_err(&rmi_dev->dev,
				"Failed to read F11 ctrl29_30, code: %d.\n",
				error);
			return error;
		}
		read_address = read_address + sizeof(ctrl->ctrl29_30->regs);
	}
	return 0;
}

static int f11_allocate_control_regs(struct rmi_device *rmi_dev,
				struct f11_2d_device_query *device_query,
				struct f11_2d_sensor_query *sensor_query,
				struct f11_2d_ctrl *ctrl,
				u16 ctrl_base_addr) {

	struct rmi_driver_data *driver_data = dev_get_drvdata(&rmi_dev->dev);
	struct rmi_function_container *fc = driver_data->f01_container;

	ctrl->ctrl0_9 = devm_kzalloc(&fc->dev, sizeof(union f11_2d_ctrl0_9),
				       GFP_KERNEL);
	if (!ctrl->ctrl0_9)
		return -ENOMEM;
	if (sensor_query->f11_2d_query7__8[0]) {
		ctrl->ctrl10 = devm_kzalloc(&fc->dev,
			sizeof(union f11_2d_ctrl10), GFP_KERNEL);
		if (!ctrl->ctrl10)
			return -ENOMEM;
	}

	if (sensor_query->f11_2d_query7__8[1]) {
		ctrl->ctrl11 = devm_kzalloc(&fc->dev,
			sizeof(union f11_2d_ctrl11), GFP_KERNEL);
		if (!ctrl->ctrl11)
			return -ENOMEM;
	}

	if (device_query->has_query9 && sensor_query->query9.has_pen) {
		ctrl->ctrl20_21 = devm_kzalloc(&fc->dev,
			sizeof(union f11_2d_ctrl20_21), GFP_KERNEL);
		if (!ctrl->ctrl20_21)
			return -ENOMEM;
	}

	if (device_query->has_query9 && sensor_query->query9.has_proximity) {
		ctrl->ctrl22_26 = devm_kzalloc(&fc->dev,
			sizeof(union f11_2d_ctrl22_26), GFP_KERNEL);
		if (!ctrl->ctrl22_26)
			return -ENOMEM;
	}

	if (device_query->has_query9 &&
		(sensor_query->query9.has_palm_det_sensitivity ||
		sensor_query->query9.has_suppress_on_palm_detect)) {
		ctrl->ctrl27 = devm_kzalloc(&fc->dev,
			sizeof(union f11_2d_ctrl27), GFP_KERNEL);
		if (!ctrl->ctrl27)
			return -ENOMEM;
	}

	if (sensor_query->has_multi_finger_scroll) {
		ctrl->ctrl28 = devm_kzalloc(&fc->dev,
			sizeof(union f11_2d_ctrl28), GFP_KERNEL);
		if (!ctrl->ctrl28)
			return -ENOMEM;
	}

	if (device_query->has_query11 && device_query->has_z_tuning) {
		ctrl->ctrl29_30 = devm_kzalloc(&fc->dev,
			sizeof(union f11_2d_ctrl29_30), GFP_KERNEL);
		if (!ctrl->ctrl29_30)
			return -ENOMEM;
	}

	return f11_read_control_regs(rmi_dev, ctrl, ctrl_base_addr);
}

static int f11_write_control_regs(struct rmi_device *rmi_dev,
					struct f11_2d_sensor_query *query,
					struct f11_2d_ctrl *ctrl,
					u16 ctrl_base_addr)
{
	u16 write_address = ctrl_base_addr;
	int error;

	error = rmi_write_block(rmi_dev, write_address,
				ctrl->ctrl0_9->regs,
				 sizeof(ctrl->ctrl0_9->regs));
	if (error < 0)
		return error;
	write_address += sizeof(ctrl->ctrl0_9);

	if (ctrl->ctrl10) {
		error = rmi_write_block(rmi_dev, write_address,
					&ctrl->ctrl10->reg, 1);
		if (error < 0)
			return error;
		write_address++;
	}

	if (ctrl->ctrl11) {
		error = rmi_write_block(rmi_dev, write_address,
					&ctrl->ctrl11->reg, 1);
		if (error < 0)
			return error;
		write_address++;
	}

	if (ctrl->ctrl12 && ctrl->ctrl12_size && query->configurable) {
		if (ctrl->ctrl12_size > query->max_electrodes) {
			dev_err(&rmi_dev->dev,
				"%s: invalid cfg size:%d, should be < %d.\n",
				__func__, ctrl->ctrl12_size,
				query->max_electrodes);
			return -EINVAL;
		}
		error = rmi_write_block(rmi_dev, write_address,
						&ctrl->ctrl12->reg,
						ctrl->ctrl12_size);
		if (error < 0)
			return error;
		write_address += ctrl->ctrl12_size;
	}

	if (ctrl->ctrl14) {
		error = rmi_write_block(rmi_dev, write_address,
				&ctrl->ctrl14->reg, 1);
		if (error < 0)
			return error;
		write_address++;
	}

	if (ctrl->ctrl15) {
		error = rmi_write_block(rmi_dev, write_address,
				&ctrl->ctrl15->reg, 1);
		if (error < 0)
			return error;
		write_address++;
	}

	if (ctrl->ctrl16) {
		error = rmi_write_block(rmi_dev, write_address,
				&ctrl->ctrl16->reg, 1);
		if (error < 0)
			return error;
		write_address++;
	}

	if (ctrl->ctrl17) {
		error = rmi_write_block(rmi_dev, write_address,
				&ctrl->ctrl17->reg, 1);
		if (error < 0)
			return error;
		write_address++;
	}

	if (ctrl->ctrl18_19) {
		error = rmi_write_block(rmi_dev, write_address,
			ctrl->ctrl18_19->reg, sizeof(union f11_2d_ctrl18_19));
		if (error < 0)
			return error;
		write_address += sizeof(union f11_2d_ctrl18_19);
	}

	if (ctrl->ctrl20_21) {
		error = rmi_write_block(rmi_dev, write_address,
					ctrl->ctrl20_21->reg,
					sizeof(union f11_2d_ctrl20_21));
		if (error < 0)
			return error;
		write_address += sizeof(union f11_2d_ctrl20_21);
	}

	if (ctrl->ctrl22_26) {
		error = rmi_write_block(rmi_dev, write_address,
					ctrl->ctrl22_26->regs,
					sizeof(union f11_2d_ctrl22_26));
		if (error < 0)
			return error;
		write_address += sizeof(union f11_2d_ctrl22_26);
	}

	if (ctrl->ctrl27) {
		error = rmi_write_block(rmi_dev, write_address,
					ctrl->ctrl27->regs,
					sizeof(union f11_2d_ctrl27));
		if (error < 0)
			return error;
		write_address += sizeof(union f11_2d_ctrl27);
	}

	if (ctrl->ctrl28) {
		error = rmi_write_block(rmi_dev, write_address,
					ctrl->ctrl28->regs,
					sizeof(union f11_2d_ctrl28));
		if (error < 0)
			return error;
		write_address += sizeof(union f11_2d_ctrl28);
	}

	if (ctrl->ctrl29_30) {
		error = rmi_write_block(rmi_dev, write_address,
					ctrl->ctrl29_30->regs,
					sizeof(union f11_2d_ctrl29_30));
		if (error < 0)
			return error;
		write_address += sizeof(union f11_2d_ctrl29_30);
	}

	return 0;
}

static int rmi_f11_get_query_parameters(struct rmi_device *rmi_dev,
			struct f11_2d_sensor_query *query, u16 query_base_addr)
{
	int query_size;
	int rc;

	rc = rmi_read_block(rmi_dev, query_base_addr, query->f11_2d_query1__4,
					sizeof(query->f11_2d_query1__4));
	if (rc < 0)
		return rc;
	query_size = rc;

	if (query->has_abs) {
		rc = rmi_read(rmi_dev, query_base_addr + query_size,
					&query->f11_2d_query5);
		if (rc < 0)
			return rc;
		query_size++;
	}

	if (query->has_rel) {
		rc = rmi_read(rmi_dev, query_base_addr + query_size,
					&query->f11_2d_query6);
		if (rc < 0)
			return rc;
		query_size++;
	}

	if (query->has_gestures) {
		rc = rmi_read_block(rmi_dev, query_base_addr + query_size,
					query->f11_2d_query7__8,
					sizeof(query->f11_2d_query7__8));
		if (rc < 0)
			return rc;
		query_size += sizeof(query->f11_2d_query7__8);
	}

	if (query->has_touch_shapes) {
		rc = rmi_read(rmi_dev, query_base_addr + query_size,
					&query->f11_2d_query10);
		if (rc < 0)
			return rc;
		query_size++;
	}

	return query_size;
}

/* This operation is done in a number of places, so we have a handy routine
 * for it.
 */
static void f11_set_abs_params(struct rmi_function_container *fc, int index)
{
	struct f11_data *f11 = fc->data;
	struct f11_2d_sensor *sensor = &f11->sensors[index];
	struct input_dev *input = sensor->input;
	int device_x_max =
		f11->dev_controls.ctrl0_9->sensor_max_x_pos;
	int device_y_max =
		f11->dev_controls.ctrl0_9->sensor_max_y_pos;
	int x_min, x_max, y_min, y_max;
	if (sensor->axis_align.swap_axes) {
		int temp = device_x_max;
		device_x_max = device_y_max;
		device_y_max = temp;
	}
	/* Use the max X and max Y read from the device, or the clip values,
	 * whichever is stricter.
	 */
	x_min = sensor->axis_align.clip_X_low;
	if (sensor->axis_align.clip_X_high)
		x_max = min((int) device_x_max,
			sensor->axis_align.clip_X_high);
	else
		x_max = device_x_max;

	y_min = sensor->axis_align.clip_Y_low;
	if (sensor->axis_align.clip_Y_high)
		y_max = min((int) device_y_max,
			sensor->axis_align.clip_Y_high);
	else
		y_max = device_y_max;

	dev_dbg(&fc->dev, "Set ranges X=[%d..%d] Y=[%d..%d].",
			x_min, x_max, y_min, y_max);

	input_set_abs_params(input, ABS_MT_PRESSURE, 0,
			DEFAULT_MAX_ABS_MT_PRESSURE, 0, 0);
	input_set_abs_params(input, ABS_MT_TOUCH_MAJOR,
			0, DEFAULT_MAX_ABS_MT_TOUCH, 0, 0);
	input_set_abs_params(input, ABS_MT_TOUCH_MINOR,
			0, DEFAULT_MAX_ABS_MT_TOUCH, 0, 0);
	input_set_abs_params(input, ABS_MT_ORIENTATION,
			0, DEFAULT_MAX_ABS_MT_ORIENTATION, 0, 0);
	input_set_abs_params(input, ABS_MT_TRACKING_ID,
			DEFAULT_MIN_ABS_MT_TRACKING_ID,
			DEFAULT_MAX_ABS_MT_TRACKING_ID, 0, 0);
	/* TODO get max_x_pos (and y) from control registers. */
	input_set_abs_params(input, ABS_MT_POSITION_X,
			x_min, x_max, 0, 0);
	input_set_abs_params(input, ABS_MT_POSITION_Y,
			y_min, y_max, 0, 0);
	if (!sensor->type_a)
		input_mt_init_slots(input, sensor->nbr_fingers);
	if (IS_ENABLED(CONFIG_RMI4_F11_PEN) &&
			sensor->sens_query.query9.has_pen)
		input_set_abs_params(input, ABS_MT_TOOL_TYPE,
				     0, MT_TOOL_MAX, 0, 0);
	else
		input_set_abs_params(input, ABS_MT_TOOL_TYPE,
				     0, MT_TOOL_FINGER, 0, 0);
}

static int f11_device_init(struct rmi_function_container *fc)
{
	int rc;

	rc = rmi_f11_initialize(fc);
	if (rc < 0)
		goto err_free_data;

	rc = rmi_f11_register_devices(fc);
	if (rc < 0)
		goto err_free_data;

	rc = rmi_f11_create_sysfs(fc);
	if (rc < 0)
		goto err_free_data;

	return 0;

err_free_data:
	rmi_f11_free_memory(fc);

	return rc;
}

static void rmi_f11_free_memory(struct rmi_function_container *fc)
{
	struct f11_data *f11 = fc->data;
	int i;

	if (f11) {
		for (i = 0; i < f11->dev_query.nbr_of_sensors + 1; i++)
			kfree(f11->sensors[i].button_map);
	}
}


static int rmi_f11_initialize(struct rmi_function_container *fc)
{
	struct rmi_device *rmi_dev = fc->rmi_dev;
	struct f11_data *f11;
	struct f11_2d_ctrl *ctrl;
	u8 query_offset;
	u16 query_base_addr;
	u16 control_base_addr;
	u16 max_x_pos, max_y_pos, temp;
	int rc;
	int i;
	struct rmi_device_platform_data *pdata = to_rmi_platform_data(rmi_dev);

	dev_dbg(&fc->dev, "Initializing F11 values for %s.\n",
		 pdata->sensor_name);

	/*
	** init instance data, fill in values and create any sysfs files
	*/
	f11 = devm_kzalloc(&fc->dev, sizeof(struct f11_data), GFP_KERNEL);
	if (!f11)
		return -ENOMEM;

	fc->data = f11;
	f11->rezero_wait_ms = pdata->f11_rezero_wait;

	query_base_addr = fc->fd.query_base_addr;
	control_base_addr = fc->fd.control_base_addr;

	rc = rmi_read(rmi_dev, query_base_addr, &f11->dev_query.f11_2d_query0);
	if (rc < 0)
		return rc;

	query_offset = (query_base_addr + 1);
	/* Increase with one since number of sensors is zero based */
	for (i = 0; i < (f11->dev_query.nbr_of_sensors + 1); i++) {
		struct f11_2d_sensor *sensor = &f11->sensors[i];
		sensor->sensor_index = i;
		sensor->fc = fc;

		rc = rmi_f11_get_query_parameters(rmi_dev, &sensor->sens_query,
					query_offset);
		if (rc < 0)
			return rc;
		query_offset += rc;

		if (f11->dev_query.has_query9) {
			rc = rmi_read(rmi_dev, query_offset,
				      &sensor->sens_query.query9.reg);
			if (rc < 0) {
				dev_err(&fc->dev, "Failed to read query 9.\n");
				return rc;
			}
			query_offset += rc;
		}

		rc = f11_allocate_control_regs(rmi_dev,
				&f11->dev_query, &sensor->sens_query,
				&f11->dev_controls, control_base_addr);
		if (rc < 0) {
			dev_err(&fc->dev,
				"Failed to initialize F11 control params.\n");
			return rc;
		}

		if (i < pdata->f11_sensor_count) {
			sensor->axis_align =
				pdata->f11_sensor_data[i].axis_align;
			sensor->virtual_buttons =
				pdata->f11_sensor_data[i].virtual_buttons;
			sensor->type_a = pdata->f11_sensor_data[i].type_a;
		}

		rc = rmi_read_block(rmi_dev,
			control_base_addr + F11_CTRL_SENSOR_MAX_X_POS_OFFSET,
			(u8 *)&max_x_pos, sizeof(max_x_pos));
		if (rc < 0)
			return rc;

		rc = rmi_read_block(rmi_dev,
			control_base_addr + F11_CTRL_SENSOR_MAX_Y_POS_OFFSET,
			(u8 *)&max_y_pos, sizeof(max_y_pos));
		if (rc < 0)
			return rc;

		if (sensor->axis_align.swap_axes) {
			temp = max_x_pos;
			max_x_pos = max_y_pos;
			max_y_pos = temp;
		}
		sensor->max_x = max_x_pos;
		sensor->max_y = max_y_pos;

		rc = f11_2d_construct_data(sensor);
		if (rc < 0)
			return rc;

		ctrl = &f11->dev_controls;
		if (sensor->axis_align.delta_x_threshold) {
			ctrl->ctrl0_9->delta_x_threshold =
				sensor->axis_align.delta_x_threshold;
			rc = rmi_write_block(rmi_dev,
					ctrl->ctrl0_9->address,
					ctrl->ctrl0_9->regs,
					sizeof(ctrl->ctrl0_9->regs));
			if (rc < 0)
				dev_warn(&fc->dev, "Failed to write to delta_x_threshold %d. Code: %d.\n",
					i, rc);

		}

		if (sensor->axis_align.delta_y_threshold) {
			ctrl->ctrl0_9->delta_y_threshold =
				sensor->axis_align.delta_y_threshold;
			rc = rmi_write_block(rmi_dev,
					ctrl->ctrl0_9->address,
					ctrl->ctrl0_9->regs,
					sizeof(ctrl->ctrl0_9->regs));
			if (rc < 0)
				dev_warn(&fc->dev, "Failed to write to delta_y_threshold %d. Code: %d.\n",
					i, rc);
		}

		if (IS_ENABLED(CONFIG_RMI4_DEBUG)) {
			rc = setup_sensor_debugfs(sensor);
			if (rc < 0)
				dev_warn(&fc->dev, "Failed to setup debugfs for F11 sensor %d. Code: %d.\n",
					i, rc);
		}
	}

	/* Google specific initialization */
#ifdef CONFIG_WAKELOCK
	wake_lock_init(&f11->goog.wakelock, WAKE_LOCK_SUSPEND, "touchpad_wakelock");
#endif
	goog_gesture_detect_reset(&f11->goog.gesture_detect);

	/* Viewfinder cutout initial dimensions */
	f11->goog.goog_view_min_y = VIEW_CUTOUT_MIN_Y;
	f11->goog.goog_view_max_y = VIEW_CUTOUT_MAX_Y;
	f11->goog.goog_view_min_x = VIEW_CUTOUT_MIN_X;
	f11->goog.goog_view_max_x = VIEW_CUTOUT_MAX_X;
	/* Google */

	if (IS_ENABLED(CONFIG_RMI4_DEBUG)) {
		rc = setup_f11_debugfs(fc);
		if (rc < 0)
			dev_warn(&fc->dev, "Failed to setup debugfs for F11. Code: %d.\n",
				rc);
	}

	mutex_init(&f11->dev_controls_mutex);
	return 0;
}

static void register_virtual_buttons(struct rmi_function_container *fc,
				     struct f11_2d_sensor *sensor) {
	int j;

	if (!sensor->sens_query.has_gestures)
		return;
	if (!sensor->virtual_buttons.buttons) {
		dev_warn(&fc->dev, "No virtual button platform data for 2D sensor %d.\n",
			 sensor->sensor_index);
		return;
	}
	/* call devm_kcalloc when it will be defined in kernel */
	sensor->button_map = devm_kzalloc(&fc->dev,
			sensor->virtual_buttons.buttons,
			GFP_KERNEL);
	if (!sensor->button_map) {
		dev_err(&fc->dev, "Failed to allocate the virtual button map.\n");
		return;
	}

	/* manage button map using input subsystem */
	sensor->input->keycode = sensor->button_map;
	sensor->input->keycodesize = sizeof(u8);
	sensor->input->keycodemax = sensor->virtual_buttons.buttons;

	/* set bits for each button... */
	for (j = 0; j < sensor->virtual_buttons.buttons; j++) {
		sensor->button_map[j] =  sensor->virtual_buttons.map[j].code;
		set_bit(sensor->button_map[j], sensor->input->keybit);
	}
}

static int rmi_f11_register_devices(struct rmi_function_container *fc)
{
	struct rmi_device *rmi_dev = fc->rmi_dev;
	struct f11_data *f11 = fc->data;
	struct input_dev *input_dev;
	struct input_dev *input_dev_mouse;
	int sensors_itertd = 0;
	int i;
	int rc;

	for (i = 0; i < (f11->dev_query.nbr_of_sensors + 1); i++) {
		struct f11_2d_sensor *sensor = &f11->sensors[i];
		sensors_itertd = i;
		input_dev = input_allocate_device();
		if (!input_dev) {
			rc = -ENOMEM;
			goto error_unregister;
		}

		sensor->input = input_dev;
		/* TODO how to modify the dev name and
		* phys name for input device */
		sprintf(sensor->input_name, "%sfn%02x", "sensor00",
			fc->fd.function_number);
		input_dev->name = sensor->input_name;
		sprintf(sensor->input_phys, "%s/input0",
			input_dev->name);
		input_dev->phys = sensor->input_phys;
		input_dev->dev.parent = &rmi_dev->dev;
		input_set_drvdata(input_dev, f11);

		set_bit(EV_SYN, input_dev->evbit);
		set_bit(EV_ABS, input_dev->evbit);
		set_bit(EV_SW, input_dev->evbit);
		set_bit(SW_CAMERA_LENS_COVER, input_dev->swbit);

		set_bit(INPUT_PROP_DIRECT, input_dev->propbit);

		f11_set_abs_params(fc, i);

		dev_dbg(&fc->dev, "%s: Sensor %d hasRel %d.\n",
			__func__, i, sensor->sens_query.has_rel);
		if (sensor->sens_query.has_rel) {
			set_bit(EV_REL, input_dev->evbit);
			set_bit(REL_X, input_dev->relbit);
			set_bit(REL_Y, input_dev->relbit);
		}
		rc = input_register_device(input_dev);
		if (rc < 0) {
			input_free_device(input_dev);
			sensor->input = NULL;
			goto error_unregister;
		}

		if (IS_ENABLED(CONFIG_RMI4_VIRTUAL_BUTTON))
			register_virtual_buttons(fc, sensor);

		if (sensor->sens_query.has_rel) {
			/*create input device for mouse events  */
			input_dev_mouse = input_allocate_device();
			if (!input_dev_mouse) {
				rc = -ENOMEM;
				goto error_unregister;
			}

			sensor->mouse_input = input_dev_mouse;
			input_dev_mouse->name = "rmi_mouse";
			input_dev_mouse->phys = "rmi_f11/input0";

			input_dev_mouse->id.vendor  = 0x18d1;
			input_dev_mouse->id.product = 0x0210;
			input_dev_mouse->id.version = 0x0100;

			set_bit(EV_REL, input_dev_mouse->evbit);
			set_bit(REL_X, input_dev_mouse->relbit);
			set_bit(REL_Y, input_dev_mouse->relbit);

			set_bit(BTN_MOUSE, input_dev_mouse->evbit);
			/* Register device's buttons and keys */
			set_bit(EV_KEY, input_dev_mouse->evbit);
			set_bit(BTN_LEFT, input_dev_mouse->keybit);
			set_bit(BTN_MIDDLE, input_dev_mouse->keybit);
			set_bit(BTN_RIGHT, input_dev_mouse->keybit);

			rc = input_register_device(input_dev_mouse);
			if (rc < 0) {
				input_free_device(input_dev_mouse);
				sensor->mouse_input = NULL;
				goto error_unregister;
			}

			set_bit(BTN_RIGHT, input_dev_mouse->keybit);
		}

	}

	return 0;

error_unregister:
	for (; sensors_itertd > 0; sensors_itertd--) {
		if (f11->sensors[sensors_itertd].input) {
			if (f11->sensors[sensors_itertd].mouse_input) {
				input_unregister_device(
				   f11->sensors[sensors_itertd].mouse_input);
				f11->sensors[sensors_itertd].mouse_input = NULL;
			}
			input_unregister_device(f11->sensors[i].input);
			f11->sensors[i].input = NULL;
		}
	}

	return rc;
}

static void rmi_f11_free_devices(struct rmi_function_container *fc)
{
	struct f11_data *f11 = fc->data;
	int i;

	for (i = 0; i < (f11->dev_query.nbr_of_sensors + 1); i++) {
		if (f11->sensors[i].input)
			input_unregister_device(f11->sensors[i].input);
		if (f11->sensors[i].sens_query.has_rel &&
				f11->sensors[i].mouse_input)
			input_unregister_device(f11->sensors[i].mouse_input);
	}
}

static int rmi_f11_create_sysfs(struct rmi_function_container *fc)
{
	int attr_count = 0;
	int rc;
	struct f11_data *f11 = fc->data;

	dev_dbg(&fc->dev, "Creating sysfs files.\n");
	/* Set up sysfs device attributes. */
	for (attr_count = 0; attr_count < ARRAY_SIZE(attrs); attr_count++) {
		if (sysfs_create_file
		    (&fc->dev.kobj, &attrs[attr_count].attr) < 0) {
			dev_err(&fc->dev,
				"Failed to create sysfs file for %s.",
				attrs[attr_count].attr.name);
			rc = -ENODEV;
			goto err_remove_sysfs;
		}
	}
	if (sysfs_create_group(&fc->dev.kobj, &attrs_control0) < 0) {
		dev_err(&fc->dev, "Failed to create query sysfs files.\n");
		return -ENODEV;
	}
	if (f11->dev_controls.ctrl29_30) {
		if (sysfs_create_group(&fc->dev.kobj,
			&attrs_control29_30) < 0) {
			dev_err(&fc->dev,
				"Failed to create query sysfs files.");
			return -ENODEV;
		}
	}

	return 0;

err_remove_sysfs:
	for (attr_count--; attr_count >= 0; attr_count--)
		sysfs_remove_file(&fc->dev.kobj, &attrs[attr_count].attr);
	sysfs_remove_group(&fc->dev.kobj, &attrs_control0);
	if (f11->dev_controls.ctrl29_30)
		sysfs_remove_group(&fc->dev.kobj, &attrs_control29_30);
	return rc;
}

static int rmi_f11_config(struct rmi_function_container *fc)
{
	struct f11_data *f11 = fc->data;
	int i;
	int rc;

	for (i = 0; i < (f11->dev_query.nbr_of_sensors + 1); i++) {
		rc = f11_write_control_regs(fc->rmi_dev,
				   &f11->sensors[i].sens_query,
				   &f11->dev_controls,
				   fc->fd.query_base_addr);
		if (rc < 0)
			return rc;
	}
	return 0;
}

int rmi_f11_attention(struct rmi_function_container *fc, u8 *irq_bits)
{
	struct rmi_device *rmi_dev = fc->rmi_dev;
	struct f11_data *f11 = fc->data;
	u16 data_base_addr = fc->fd.data_base_addr;
	u16 data_base_addr_offset = 0;
	int error;
	int i;

	for (i = 0; i < f11->dev_query.nbr_of_sensors + 1; i++) {
		error = rmi_read_block(rmi_dev,
				data_base_addr + data_base_addr_offset,
				f11->sensors[i].data_pkt,
				f11->sensors[i].pkt_size);
		if (error < 0)
			return error;
#ifdef DEBUG_GESTURES
		rmi_f11_debug_gestures(f11, &f11->sensors[i]);
#endif  /* DEBUG_GESTURES */
		rmi_f11_finger_handler(f11, &f11->sensors[i]);
		rmi_f11_virtual_button_handler(&f11->sensors[i]);
		data_base_addr_offset += f11->sensors[i].pkt_size;
	}

	return 0;
}

#ifdef CONFIG_PM
/*
 *  suspend handler for F11, this will be invoked in
 *  early suspend routine from sensor device.
 */
static int rmi_f11_suspend(struct rmi_function_container *fc)
{
	struct f11_data *data = fc->data;

	data->goog.early_suspended = 1;
	data->goog.last_suspend_cnt = get_suspend_cnt();
	data->goog.synth_events_sent = 0;
	data->goog.early_tap = 0;
	data->goog.press = 0;
	goog_gesture_detect_reset(&data->goog.gesture_detect);
	dev_info(&fc->dev, "%s Early suspended touchpad\n", __FUNCTION__);
	return 0;
}

/*
 *  resume handler for F11, this will be invoked in
 *  late resume routine from sensor device.
 */
static int rmi_f11_resume(struct rmi_function_container *fc)
// void rmi_f11_resume(struct rmi_function_info *rmifninfo)
{
	struct rmi_device *rmi_dev = fc->rmi_dev;
	struct f11_data *data = fc->data;
	/* Command register always reads as 0, so we can just use a local. */
	union f11_2d_commands commands = {};

	int retval = 0;
	data->goog.early_suspended = 0;
	dev_info(&fc->dev, "%s Late resumed touchpad", __FUNCTION__);

	/* NOTE(GOOG) Begin original resume functionality.  In our case we do not
	 * enable the rezero dunctionality so we simply return. */
	if (!data->rezero_wait_ms)
		return 0;

	mdelay(data->rezero_wait_ms);

	commands.rezero = 1;
	retval = rmi_write_block(rmi_dev, fc->fd.command_base_addr,
			&commands.reg, sizeof(commands.reg));
	if (retval < 0) {
		dev_err(&rmi_dev->dev, "%s: failed to issue rezero command, error = %d.",
			__func__, retval);
		return retval;
	}

	return retval;
}
#endif /* CONFIG_PM */

static int f11_remove_device(struct device *dev)
{
	int attr_count = 0;
	struct f11_data *f11;
	struct rmi_function_container *fc = to_rmi_function_container(dev);

	f11 = fc->data;

	if (IS_ENABLED(CONFIG_RMI4_DEBUG)) {
		int i;

		for (i = 0; i < f11->dev_query.nbr_of_sensors + 1; i++)
			teardown_sensor_debugfs(&f11->sensors[i]);
		teardown_f11_debugfs(f11);
	}

	for (attr_count = 0; attr_count < ARRAY_SIZE(attrs); attr_count++)
		sysfs_remove_file(&fc->dev.kobj, &attrs[attr_count].attr);

	sysfs_remove_group(&fc->dev.kobj, &attrs_control0);
	if (f11->dev_controls.ctrl29_30)
		sysfs_remove_group(&fc->dev.kobj, &attrs_control29_30);

	rmi_f11_free_devices(fc);

	rmi_f11_free_memory(fc);

	return 0;
}

static int f11_probe(struct device *dev);

static struct rmi_function_handler function_handler = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "rmi_f11",
		.bus = &rmi_bus_type,
		.probe = f11_probe,
		.remove = f11_remove_device,
	},
	.func = 0x11,
	.config = rmi_f11_config,
	.attention = rmi_f11_attention,
/* NOTE(GOOG) Additional power management */
#ifdef  CONFIG_PM
#if defined(CONFIG_HAS_EARLYSUSPEND)
        .early_suspend = rmi_f11_suspend,
        .late_resume = rmi_f11_resume,
#else
        .suspend = rmi_f11_suspend,
        .resume = rmi_f11_resume,
#endif  /* defined(CONFIG_HAS_EARLYSUSPEND) && !def... */
#endif  /* CONFIG_PM */
};

static __devinit int f11_probe(struct device *dev)
{
	struct rmi_function_container *fc;

	if (dev->type != &rmi_function_type) {
		dev_dbg(dev, "Not a function device.\n");
		return 1;
	}
	fc = to_rmi_function_container(dev);
	if (fc->fd.function_number != function_handler.func) {
		dev_dbg(dev, "Device is F%02X, not F%02X.\n",
			fc->fd.function_number, function_handler.func);
		return -ENXIO;
	}

	return f11_device_init(fc);
}

static int __init rmi_f11_module_init(void)
{
	int error;

	error = driver_register(&function_handler.driver);
	if (error < 0) {
		pr_err("%s: register driver failed!\n", __func__);
		return error;
	}

	return 0;
}

static void __exit rmi_f11_module_exit(void)
{
	driver_unregister(&function_handler.driver);
}

/* Used to turn on and off driver gesture detection */
static ssize_t f11_gesture_show(struct device *dev,
                                struct device_attribute *attr,
                                char *buf)
{
	struct rmi_function_container *fc;
	struct f11_data *instance_data;

	fc = to_rmi_function_container(dev);
	instance_data = fc->data;

	return snprintf(buf, PAGE_SIZE, "%u\n",
	                instance_data->goog.goog_gesture_enable);
}

static ssize_t f11_gesture_store(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf,
					 size_t count)
{
	struct rmi_function_container *fc;
	struct f11_data *instance_data;
	unsigned int new_value;

	fc = to_rmi_function_container(dev);
	instance_data = fc->data;

	if (sscanf(buf, "%u", &new_value) != 1)
		return -EINVAL;
	if (new_value < 0 || new_value > 1)
		return -EINVAL;
	instance_data->goog.goog_gesture_enable = new_value;

	goog_gesture_detect_reset(&instance_data->goog.gesture_detect);

	dev_info(dev, "Touchpad %s driver gesture tap detector\n", (new_value==1)?"enabled":"disabled");

	return count;
}


static ssize_t f11_maxPos_show(struct device *dev,
				     struct device_attribute *attr,
				     char *buf)
{
	struct rmi_function_container *fc;
	struct f11_data *data;

	fc = to_rmi_function_container(dev);
	data = fc->data;

	return snprintf(buf, PAGE_SIZE, "%u %u\n",
			data->sensors[0].max_x, data->sensors[0].max_y);
}

static ssize_t f11_relreport_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct rmi_function_container *fc;
	struct f11_data *instance_data;

	fc = to_rmi_function_container(dev);
	instance_data = fc->data;

	return snprintf(buf, PAGE_SIZE, "%u\n",
			instance_data->
			sensors[0].axis_align.rel_report_enabled);
}

static ssize_t f11_relreport_store(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf,
					 size_t count)
{
	struct rmi_function_container *fc;
	struct f11_data *instance_data;
	unsigned int new_value;

	fc = to_rmi_function_container(dev);
	instance_data = fc->data;


	if (sscanf(buf, "%u", &new_value) != 1)
		return -EINVAL;
	if (new_value < 0 || new_value > 1)
		return -EINVAL;
	instance_data->sensors[0].axis_align.rel_report_enabled = new_value;

	return count;
}

static ssize_t f11_rezero_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	struct rmi_function_container *fc = NULL;
	unsigned int rezero;
	int retval = 0;
	/* Command register always reads as 0, so we can just use a local. */
	union f11_2d_commands commands = {};

	fc = to_rmi_function_container(dev);

	if (sscanf(buf, "%u", &rezero) != 1)
		return -EINVAL;
	if (rezero < 0 || rezero > 1)
		return -EINVAL;

	/* Per spec, 0 has no effect, so we skip it entirely. */
	if (rezero) {
		commands.rezero = 1;
		retval = rmi_write_block(fc->rmi_dev, fc->fd.command_base_addr,
				&commands.reg, sizeof(commands.reg));
		if (retval < 0) {
			dev_err(dev, "%s: failed to issue rezero command, error = %d.",
				__func__, retval);
			return retval;
		}
	}

	return count;
}

static ssize_t f11_view_enable_show(struct device *dev,
                                    struct device_attribute *attr,
                                    char *buf)
{
	struct rmi_function_container *fc;
	struct f11_data *instance_data;

	fc = to_rmi_function_container(dev);
	instance_data = fc->data;

	return snprintf(buf, PAGE_SIZE, "%u\n",
	                instance_data->goog.goog_view_enable);
}

static ssize_t f11_view_enable_store(struct device *dev,
                                     struct device_attribute *attr,
                                     const char *buf,
                                     size_t count)
{
	struct rmi_function_container *fc;
	struct f11_data *instance_data;
	unsigned int new_value;

	fc = to_rmi_function_container(dev);
	instance_data = fc->data;

	if (sscanf(buf, "%u", &new_value) != 1)
		return -EINVAL;
	if (new_value < 0 || new_value > 1)
		return -EINVAL;
	instance_data->goog.goog_view_enable = new_value;

	return count;
}

static ssize_t f11_view_dim_show(struct device *dev,
                                 struct device_attribute *attr,
                                 char *buf)
{
	struct rmi_function_container *fc;
	struct f11_data *instance_data;

	fc = to_rmi_function_container(dev);
	instance_data = fc->data;

	return snprintf(buf, PAGE_SIZE, "%u %u %u %u\n",
	                instance_data->goog.goog_view_min_y,
	                instance_data->goog.goog_view_max_y,
	                instance_data->goog.goog_view_min_x,
	                instance_data->goog.goog_view_max_x);
}

static ssize_t f11_view_dim_store(struct device *dev,
                                     struct device_attribute *attr,
                                     const char *buf,
                                     size_t count)
{
	struct rmi_function_container *fc;
	struct f11_data *instance_data;
	unsigned int new_value[4];

	fc = to_rmi_function_container(dev);
	instance_data = fc->data;

	if (sscanf(buf, "%u %u %u %u", &new_value[0], &new_value[1],
	           &new_value[2], &new_value[3]) != 4)
		return -EINVAL;
	/* Boundaries must be within touchpad physical limits */
	if (new_value[0] > instance_data->sensors[0].max_y)
		return -EINVAL;
	if (new_value[1] > instance_data->sensors[0].max_y)
		return -EINVAL;
	if (new_value[2] > instance_data->sensors[0].max_x)
		return -EINVAL;
	if (new_value[3] > instance_data->sensors[0].max_x)
		return -EINVAL;

	/* Boundaries also must keep min/max relationship */
	if (new_value[0] > new_value[1])
		return -EINVAL;
	if (new_value[2] > new_value[3])
		return -EINVAL;

	instance_data->goog.goog_view_min_y = new_value[0];
	instance_data->goog.goog_view_max_y = new_value[1];
	instance_data->goog.goog_view_min_x = new_value[2];
	instance_data->goog.goog_view_max_x = new_value[3];
	return count;
}


static ssize_t f11_view_val_show(struct device *dev,
                                 struct device_attribute *attr,
                                 char *buf)
{
	struct rmi_function_container *fc;
	struct f11_data *instance_data;

	fc = to_rmi_function_container(dev);
	instance_data = fc->data;

	return snprintf(buf, PAGE_SIZE, "%u\n",
	                instance_data->goog.goog_view_val);
}

/* Control sysfs files */
show_store_union_struct_unsigned(dev_controls, ctrl0_9, abs_pos_filt)
show_store_union_struct_unsigned(dev_controls, ctrl29_30, z_touch_threshold)
show_store_union_struct_unsigned(dev_controls, ctrl29_30, z_touch_hysteresis)

module_init(rmi_f11_module_init);
module_exit(rmi_f11_module_exit);

MODULE_AUTHOR("Christopher Heiny <cheiny@synaptics.com");
MODULE_DESCRIPTION("RMI F11 module");
MODULE_LICENSE("GPL");
MODULE_VERSION(RMI_DRIVER_VERSION);
