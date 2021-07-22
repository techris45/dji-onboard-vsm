/*
 * dji_defs.h
 *
 *  Created on: May 13, 2019
 *      Author: j
 */

#ifndef INCLUDE_DJI_DEFS_H_
#define INCLUDE_DJI_DEFS_H_

#include <dji_status.hpp>

static constexpr int MAX_PACKS = DJI::OSDK::DataSubscription::MAX_NUMBER_OF_PACKAGE;

// These actually should be named enums in Onboard SDK.
typedef enum {
    CONTORL_SOURCE_RC       = 0,
    CONTORL_SOURCE_APP      = 1,
    CONTORL_SOURCE_SERIAL   = 2,
    CONTORL_SOURCE_UNKOWN   = -1
} Control_source;

typedef enum {
    ARMED_STATE_STOPPED     = DJI::OSDK::VehicleStatus::FlightStatus::STOPED,
    ARMED_STATE_ON_GROUND   = DJI::OSDK::VehicleStatus::FlightStatus::ON_GROUND,
    ARMED_STATE_IN_AIR      = DJI::OSDK::VehicleStatus::FlightStatus::IN_AIR,
    ARMED_STATE_UNKNOWN     = -1
} Armed_state;

typedef enum {
    DISPLAY_MODE_ASSISTED_TAKEOFF   = DJI::OSDK::VehicleStatus::DisplayMode::MODE_ASSISTED_TAKEOFF,
    DISPLAY_MODE_ATTITUDE           = DJI::OSDK::VehicleStatus::DisplayMode::MODE_ATTITUDE,
    DISPLAY_MODE_AUTO_LANDING       = DJI::OSDK::VehicleStatus::DisplayMode::MODE_AUTO_LANDING,
    DISPLAY_MODE_AUTO_TAKEOFF       = DJI::OSDK::VehicleStatus::DisplayMode::MODE_AUTO_TAKEOFF,
    DISPLAY_MODE_ENGINE_START       = DJI::OSDK::VehicleStatus::DisplayMode::MODE_ENGINE_START,
    DISPLAY_MODE_FORCE_AUTO_LANDING = DJI::OSDK::VehicleStatus::DisplayMode::MODE_FORCE_AUTO_LANDING,
    DISPLAY_MODE_HOTPOINT_MODE      = DJI::OSDK::VehicleStatus::DisplayMode::MODE_HOTPOINT_MODE,
    DISPLAY_MODE_MANUAL_CTRL        = DJI::OSDK::VehicleStatus::DisplayMode::MODE_MANUAL_CTRL,
    DISPLAY_MODE_NAVI_GO_HOME       = DJI::OSDK::VehicleStatus::DisplayMode::MODE_NAVI_GO_HOME,
    DISPLAY_MODE_NAVI_SDK_CTRL      = DJI::OSDK::VehicleStatus::DisplayMode::MODE_NAVI_SDK_CTRL,
    DISPLAY_MODE_P_GPS              = DJI::OSDK::VehicleStatus::DisplayMode::MODE_P_GPS,
    DISPLAY_MODE_SEARCH_MODE        = DJI::OSDK::VehicleStatus::DisplayMode::MODE_SEARCH_MODE,
    // Empirically detected for mission flight and click&go
    DISPLAY_MODE_WP                 = DJI::OSDK::VehicleStatus::DisplayMode::MODE_RESERVED_14,
    DISPLAY_MODE_F                  = DJI::OSDK::VehicleStatus::DisplayMode::MODE_RESERVED_31,
    DISPLAY_MODE_UNKNOWN            = -1
} Display_mode;

typedef enum {
    RC_LOST_ACTION_FS        = 0,
    RC_LOST_ACTION_CONTINUE  = 1
} Rc_lost_action;

typedef enum {
    FINISH_ACTION_NONE      = 0,
    FINISH_ACTION_RTH       = 1,
    FINISH_ACTION_LAND      = 2,
    FINISH_ACTION_RTS       = 3,    // return to wp1
    FINISH_ACTION_REPEAT    = 4,    // start again from wp1
} Finish_action;

typedef enum {
    GIMBAL_PITCH_MODE_FREE  = 0,
    GIMBAL_PITCH_MODE_AUTO  = 1,
} Gimbal_pitch_mode;

typedef enum {
    TRACE_MODE_STOP   = 0,
    TRACE_MODE_SMOOTH = 1,
} Trace_mode;

typedef enum {
    TURN_MODE_CLOCKWISE   = 0,
    TURN_MODE_COUNTER_CLOCKWISE = 1,
} Turn_mode;

typedef enum {
    YAW_MODE_AUTO   = 0,    // point to next WP
    YAW_MODE_LOCK   = 1,    // fixed
    YAW_MODE_RC     = 2,    // controlled by RC
    YAW_MODE_WP     = 3,    // controlled by tgt_yaw
} Yaw_mode;

typedef enum {
    WP_REACHED_STATUS_PRE_ACTION    = 4,    // WP reached
    WP_REACHED_STATUS_POST_ACTION   = 6,    // start navigating to next WP
} Wp_status;

typedef enum {
    WP_STATUS_PRE_MISSION   = 0,   // Approaching 1st WP
    WP_STATUS_IN_ACTION     = 1,   // Flying to given (WayPointStatusPushData.waypoint_index) WP
    WP_STATUS_LAST_WP       = 4,   // Last WP reached
    WP_STATUS_FIRST_WP      = 5,   // First WP reached
    WP_STATUS_REACHED       = 6,   // At WP
} Wp_reached_status;

typedef enum {
    WP_ACTION_WAIT          = 0,   // Wait for milliseconds.
    WP_ACTION_SIMPLE_SHOT   = 1,
    WP_ACTION_VIDEO_START   = 2,
    WP_ACTION_VIDEO_STOP    = 3,
    WP_ACTION_CRAFT_YAW     = 4,   // YAW (-180~180) Adjust the aircraft toward
    WP_ACTION_GIMBAL_PITCH  = 5,   // Adjust gimbal pitch 0: head -90: look down
} Wp_action;


#endif /* INCLUDE_DJI_DEFS_H_ */
