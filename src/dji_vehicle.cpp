// Copyright (c) 2018, Smart Projects Holdings Ltd
// All rights reserved.
// See LICENSE file for license details.

#include <dji_vehicle.h>
#include <fstream>

constexpr std::chrono::milliseconds
    Dji_vehicle::DIRECT_CONTROL_INTERVAL,
    Dji_vehicle::DIRECT_CONTROL_TIMEOUT,
    Dji_vehicle::RTH_ON_RC_LOSS_TIMEOUT,
    Dji_vehicle::ALTITUDE_ORIGIN_RESET_TRESHOLD;


#define LOG_WP(str, wp) VEHICLE_LOG_DBG(*this, str " wp %d lat: %1.9f lon: %1.9f alt: %0.2f hdg: %d ccw: %d %d %0.2f",\
    wp.index, wp.latitude, wp.longitude, wp.altitude, wp.yaw, wp.turnMode, wp.actionNumber, wp.damping);\
    for (int i = 0; i < wp.actionNumber; i++) {\
        VEHICLE_LOG_DBG(*this, "    Action %d: %d %d",\
            i, wp.commandList[i], static_cast<int16_t>(wp.commandParameter[i]));\
    }

using namespace ugcs::vsm;

Dji_vehicle::Dji_vehicle()
{
    frequencies.fill(0);
    for (int pack = 0; pack < MAX_PACKS; pack++) {
        callback_data[pack] = {pack, this};
    }
}

Dji_vehicle::~Dji_vehicle()
{
    VEHICLE_LOG_DBG(*this, "Dji_vehicle deleted.");
}

std::vector<int>
split_string_to_ints(const std::string & str, const std::string & delimiters, size_t max_count) {
    std::vector<int> ret;
    std::string::size_type start = 0;
    while (start < str.length() && ret.size() < max_count) {
        auto pos = str.find_first_of(delimiters, start);
        if (pos == start) {
            // ignore empty tokens
            start++;
        } else {
            try {
                ret.emplace_back(std::stoi({str, start}, &pos));
            } catch (...) {
                break;  // break on invalid input data.
            }
            start += pos;
        }
    }
    return ret;
}

void
Dji_vehicle::On_enable()
{
    auto props = Properties::Get_instance().get();

    DJI::OSDK::Log::instance().disableErrorLogging();
    DJI::OSDK::Log::instance().disableStatusLogging();
//    DJI::OSDK::Log::instance().enableDebugLogging();

    // These 4 are required.
    port_name = props->Get("connection.serial.port");
    port_baud = props->Get_int("connection.serial.baud");
    std::string app_id = props->Get("vehicle.dji.app_id");
    std::string app_key = props->Get("vehicle.dji.app_key");

    if (props->Exists("vehicle.dji.autoheading")) {
        auto val = props->Get("vehicle.dji.autoheading");
        if (val == std::string("yes")) {
            autoheading = true;
        }
        if (val == std::string("no")) {
            autoheading = false;
        }
    }
    if (autoheading) {
        LOG("Auto heading is ON");
    } else {
        LOG("Auto heading is OFF");
    }


    if (props->Exists("vehicle.dji.gcs_loss.height")) {
        auto alt = props->Get_float("vehicle.dji.gcs_loss.height");
        if (alt > 0) {
            LOG("gcs_loss_height=%0.2f m", alt);
            gcs_loss_height = alt;
        } else {
            LOG("Invalid gcs_loss_height (%0.2f)", alt);
        }
    }

    if (props->Exists("vehicle.dji.gcs_loss.max_altitude")) {
        auto alt = props->Get_float("vehicle.dji.gcs_loss.max_altitude");
        if (alt > 0) {
            LOG("gcs_loss_max_altitude=%0.2f m", alt);
            gcs_loss_max_altitude = alt;
        } else {
            LOG("Invalid gcs_loss_max_altitude (%0.2f)", alt);
        }
    }

    if (props->Exists("vehicle.dji.gcs_loss.rth_timeout")) {
        int tout = props->Get_float("vehicle.dji.gcs_loss.rth_timeout") * 1000;
        if (tout > 0) {
            LOG("gcs_loss_rth_timeout=%d ms", tout);
            gcs_loss_rth_timeout = std::chrono::milliseconds(tout);
        } else {
            LOG("Invalid gcs_loss_rth_timeout (%d)", tout);
        }
    }

    if (props->Exists("vehicle.dji.serial")) {
        Set_serial_number(props->Get("vehicle.dji.serial"));
    }

    while (true) {
        LOG("Creating vehicle...");
        vehicle = std::make_shared<DJI::OSDK::Vehicle>(
                port_name.c_str(),
                port_baud,
                true,
                false);
        if (!vehicle->protocolLayer->getDriver()->getDeviceStatus())
        {
            LOG("Comms failure. Retrying until success...");
            std::this_thread::sleep_for(std::chrono::seconds(1));
            continue;
        }

        DJI::OSDK::Vehicle::ActivateData activateData;
        try {
            activateData.ID = std::stoul(app_id);
        } catch (const std::logic_error& ex) {
            VSM_SYS_EXCEPTION("Invalid vehicle.dji.app_id");
        }
        activateData.encKey = const_cast<char*>(app_key.c_str());
        activateData.version = 0;

        LOG("Autopilot activation...");
        auto ack = vehicle->activate(&activateData, 1);	// Wait 1 second.
        if (DJI::OSDK::ACK::getError(ack)) {
            LOG("Autopilot activation failed. Retrying until success...");
            continue;
        } else {
            break;
        }
    }

    auto v = vehicle->getFwVersion();
    LOG("Autopilot type       %s", vehicle->getHwVersion());
    LOG("Autopilot serial     %s", vehicle->getHwSerialNum());
    LOG("Autopilot FW version %d.%d.%d.%d", v >> 24, (v >> 16) & 0xff, (v >> 8) & 0xff, v & 0xff );

    Set_autopilot_serial(std::string(vehicle->getHwSerialNum()));
    Set_autopilot_type("dji_a3");
    Set_vehicle_type(proto::VEHICLE_TYPE_MULTICOPTER);
    Set_port_name(port_name);
    Set_model_name(vehicle->getHwVersion());

    t_uplink_present->Set_value(true);
    t_downlink_present->Set_value(true);
    // Current WP is sent @10Hz while mission flight is active. 1s will be just fine.
    t_current_command->Set_timeout(1);
    // Assume telemetry is fine on connect.
    t_gcs_link_quality->Set_value(1);
    last_telemetry_received = std::chrono::steady_clock::now();
    last_rc_ok_time = std::chrono::steady_clock::now();

    // Disable all subscriptions.
    vehicle->subscribe->removeAllExistingPackages();

    Set_rc_loss_actions({
        proto::FAILSAFE_ACTION_RTH,
        proto::FAILSAFE_ACTION_CONTINUE
        });

    Subscribe_telemetry_topic(4, DJI::OSDK::Telemetry::TOPIC_GPS_FUSED);            // lat,lon,satcount
    Subscribe_telemetry_topic(4, DJI::OSDK::Telemetry::TOPIC_VELOCITY);             // speed
    Subscribe_telemetry_topic(5, DJI::OSDK::Telemetry::TOPIC_QUATERNION);           // attitude
    Subscribe_telemetry_topic(1, DJI::OSDK::Telemetry::TOPIC_HEIGHT_HOMEPOINT);     // amsl
    Subscribe_telemetry_topic(5, DJI::OSDK::Telemetry::TOPIC_STATUS_FLIGHT);        // armed?
    Subscribe_telemetry_topic(5, DJI::OSDK::Telemetry::TOPIC_ALTITUDE_FUSIONED);    // alt amsl
    Subscribe_telemetry_topic(1, DJI::OSDK::Telemetry::TOPIC_BATTERY_INFO);         // voltage
    Subscribe_telemetry_topic(4, DJI::OSDK::Telemetry::TOPIC_CONTROL_DEVICE);       // flight/control mode
    Subscribe_telemetry_topic(4, DJI::OSDK::Telemetry::TOPIC_STATUS_DISPLAYMODE);   // flight/control mode
    Subscribe_telemetry_topic(4, DJI::OSDK::Telemetry::TOPIC_RC_WITH_FLAG_DATA);    // rc status

    c_mission_upload->Set_available();
    c_auto->Set_available();
    c_rth->Set_available();
    c_takeoff_command->Set_available();
    c_land_command->Set_available();
    c_arm->Set_available();
    c_disarm->Set_available();
    c_pause->Set_available();
    c_resume->Set_available();
    c_guided->Set_available();
    c_waypoint->Set_available();
    c_manual->Set_available();
    c_joystick->Set_available();
    c_direct_vehicle_control->Set_available();
    c_camera_trigger_command->Set_available();
    if (vehicle->camera) {
        c_camera_trigger_command->Set_enabled();
    }
    c_direct_payload_control->Set_available();
    Register();

    for (int pack = 0; pack < MAX_PACKS; pack++) {
        vehicle->subscribe->registerUserPackageUnpackCallback(
            pack,
            Dji_telemetry_callback,
            callback_data + pack);
    }
    direct_vehicle_control_timer = Timer_processor::Get_instance()->Create_timer(
        DIRECT_CONTROL_INTERVAL,
        Make_callback(
            &Dji_vehicle::Send_direct_vehicle_control,
            Shared_from_this()),
        Get_completion_ctx());

    // Do one-time init of wpMission and callbacks.
    // Need this to init the vehicle->missionManager->wpMission of the SDK.
    // Do not care about return as we will call init again in Send_mission and check result there.
    memset(&waypoint_init_data, 0, sizeof(waypoint_init_data));
    vehicle->missionManager->init(DJI::OSDK::WAYPOINT, 3, &waypoint_init_data);
    vehicle->missionManager->wpMission->setWaypointEventCallback(Dji_waypoint_callback, this);
    vehicle->missionManager->wpMission->setWaypointCallback(Dji_waypoint_callback, this);
}

void
Dji_vehicle::On_disable()
{
    VEHICLE_LOG_DBG(*this, "Dji_vehicle disable");
    // Disable all subscriptions.
    if (vehicle && vehicle->subscribe) {
        vehicle->subscribe->removeAllExistingPackages();
    }
    if (direct_vehicle_control_timer) {
        direct_vehicle_control_timer->Cancel();
    }
    if (trigger_by_time_timer) {
        trigger_by_time_timer->Cancel();
    }
    if (gcs_loss_rth_timer) {
        gcs_loss_rth_timer->Cancel();
    }
}

void
Dji_vehicle::Handle_next_request()
{
    current_ucs_request = nullptr;
    if (!pending_ucs_requests.empty()) {
        auto req = pending_ucs_requests.front();
        pending_ucs_requests.pop();
        VEHICLE_LOG_INF(*this, "Processing pending request");
        Handle_ucs_command(req);
    }
}

void
Dji_vehicle::Fail_current_request(const std::string& reason)
{
    Command_failed(current_ucs_request, reason);
    Handle_next_request();
}

void
Dji_vehicle::Succeed_current_request()
{
    Command_succeeded(current_ucs_request);
    Handle_next_request();
}

void
Dji_vehicle::Initiate_command_climb(float alt)
{
    // Craft click&go command as if it was received from ucs.
    // This is because if we do not chain it, it can interfere with eventual command in progress.
    proto::Vsm_message vsm_msg;
    vsm_msg.set_device_id(Get_session_id());
    auto c = vsm_msg.mutable_device_commands()->Add();
    Property_list params;
    params["latitude"] = t_latitude;
    params["longitude"] = t_longitude;
    params["altitude_amsl"] = Property::Create("", alt);
    params["altitude_origin"] = Property::Create("", 0);
    params["heading"] = t_heading;
    params["vertical_speed"] = Property::Create("", 1);
    params["ground_speed"] = Property::Create("", 0);
    params["acceptance_radius"] = Property::Create("", 0);
    c_waypoint->Build_command(c, params);
    On_ucs_message(vsm_msg);
}

bool
Dji_vehicle::Initiate_command_rth()
{
    // Craft RTH command as if it was received from ucs.
    // This is because if we do not chain it, it can interfere with eventual command in progress.
    proto::Vsm_message vsm_msg;
    vsm_msg.set_device_id(Get_session_id());
    auto c = vsm_msg.mutable_device_commands()->Add();
    c_rth->Build_command(c);
    On_ucs_message(vsm_msg);
    return false;
}

void
Dji_vehicle::Handle_ucs_info(std::vector<Ucs_info> ucs_data)
{
    LOG("UCS count %zu", ucs_data.size());
    if (ucs_data.size()) {
        // New connection form server. Cancel the RTH timer if set.
        if (gcs_loss_rth_timer) {
            if (gcs_loss_rth_timer->Is_running()) {
                LOG_INFO("Canceling RTH timer");
                gcs_loss_rth_timer->Cancel();
            }
            gcs_loss_rth_timer = nullptr;
        }
    } else {
        // Last ucs connection is gone
        if (gcs_loss_rth_timeout) {
            // RTH on RC loss required.
            if (gcs_loss_rth_timer && gcs_loss_rth_timer->Is_running()) {
                // This means Handle_ucs_info was called twice with empty data. Should not happen.
                LOG_ERR("RTH timer already running");
            } else {
                if ((*gcs_loss_rth_timeout).count() > 0) {
                    LOG_INFO("Create RTH timer for %" PRIu64 " ms", (*gcs_loss_rth_timeout).count());
                    gcs_loss_rth_timer = Timer_processor::Get_instance()->Create_timer(
                        *gcs_loss_rth_timeout,
                        Make_callback(
                            &Dji_vehicle::Initiate_command_rth,
                            Shared_from_this()),
                        Get_completion_ctx());
                } else {
                    LOG_INFO("Initiating RTH on UCS connection loss.");
                    Initiate_command_rth();
                    gcs_loss_rth_timer = nullptr;
                }
            }
        }

        if (gcs_loss_height &&
                (Is_my_mode(proto::CONTROL_MODE_AUTO) ||
                Is_my_mode(proto::CONTROL_MODE_CLICK_GO) ||
                Is_my_mode(proto::CONTROL_MODE_JOYSTICK))) {
            float alt = 0;
            t_altitude_raw->Get_value(alt);
            if (alt > gcs_loss_max_altitude) {
                LOG_INFO("Vehicle already above gcs_loss_max_height of %0.2fm. Hovering at %0.2fm",
                    gcs_loss_max_altitude,
                    alt);
            } else {
                alt += *gcs_loss_height;
                if (alt > gcs_loss_max_altitude) {
                    LOG_INFO("Climbing to %0.2fm would exceed gcs_loss_max_height (%0.2f), Climbing to %0.2f m",
                        alt,
                        gcs_loss_max_altitude,
                        gcs_loss_max_altitude);
                    alt = gcs_loss_max_altitude;
                } else {
                    LOG_INFO("Initiating climb to %0.2f meters on UCS connection loss.", alt);
                }
            }
            Initiate_command_climb(alt);
        }
    }
}

void
Dji_vehicle::Handle_ucs_command(Ucs_request::Ptr ucs_request)
{
    if (ucs_request->request.device_commands_size() == 0) {
        Command_failed(ucs_request, "No commands found", proto::STATUS_INVALID_COMMAND);
        return;
    } else if (ucs_request->request.device_commands_size() > 1) {
        Command_failed(ucs_request, "Only one command per request supported", proto::STATUS_INVALID_COMMAND);
        return;
    }

    auto &vsm_cmd = ucs_request->request.device_commands(0);
    auto cmd = Get_command(vsm_cmd.command_id());
    if (!cmd) {
        Command_failed(
                ucs_request,
                "Unknown command id " + std::to_string(vsm_cmd.command_id()),
                proto::STATUS_INVALID_COMMAND);
        return;
    }

    Property_list params;
    try {
        params = cmd->Build_parameter_list(vsm_cmd);
    } catch (const std::exception& ex) {
        Command_failed(ucs_request, ex.what(), proto::STATUS_INVALID_PARAM);
        return;
    }

    // always process direct control commands.
    if (cmd == c_direct_vehicle_control) {
        if (Is_control_mode(proto::CONTROL_MODE_JOYSTICK)) {
            float p, r, y, t;
            params.Get_value("roll", r);
            params.Get_value("pitch", p);
            params.Get_value("yaw", y);
            params.Get_value("throttle", t);
            direct_vehicle_controls.pitch = r * 20;
            direct_vehicle_controls.roll = p * 20;
            direct_vehicle_controls.throttle = t * 5;
            direct_vehicle_controls.yaw = y * 100;
            direct_vehicle_controls.last_received = std::chrono::steady_clock::now();
            LOG("rpty %0.3f %0.3f %0.3f %0.3f",
                direct_vehicle_controls.roll,
                direct_vehicle_controls.pitch,
                direct_vehicle_controls.throttle,
                direct_vehicle_controls.yaw);
        }
        Command_succeeded(ucs_request);
        return;
    } else if (cmd == c_direct_payload_control) {
        float p, r, y, z;
        params.Get_value("roll", r);
        params.Get_value("pitch", p);
        params.Get_value("yaw", y);
        params.Get_value("zoom", z);
        direct_payload_controls.tilt = p * 20;
        direct_payload_controls.roll = r * 20;
        direct_payload_controls.yaw = y * 20;
        direct_payload_controls.zoom_level = z * 100;
        direct_payload_controls.last_received = std::chrono::steady_clock::now();
        LOG("pld %0.3f %0.3f %0.3f %0.3f",
            direct_payload_controls.tilt,
            direct_payload_controls.roll,
            direct_payload_controls.yaw,
            direct_payload_controls.zoom_level);
        Command_succeeded(ucs_request);
        return;
    }

    if (current_ucs_request) {
        if (pending_ucs_requests.size() == MAX_CONCURRENT_REQUESTS - 1) {
            Command_failed(ucs_request, "Too many pending requests");
        } else {
            VEHICLE_LOG_INF(*this, "Pending command: %s", cmd->Get_name().c_str());
            pending_ucs_requests.push(ucs_request);
        }
        return;
    } else {
        current_ucs_request = ucs_request;
    }

    VEHICLE_LOG_INF(*this, "COMMAND: %s", Dump_command(vsm_cmd).c_str());

    if (cmd == c_auto) {
        Stop_mission(false);
        Reupload_mission_and_start();

    } else if (cmd == c_takeoff_command) {
        if (cur_armed_state == ARMED_STATE_IN_AIR) {
            Fail_current_request("Vehicle already in air");
        } else {           
           vehicle->control->takeoff(Dji_ack_callback, Create_ack_data_no_mode());
        }

    } else if (cmd == c_land_command) {
        if (cur_armed_state == ARMED_STATE_IN_AIR) {
            vehicle->control->land(Dji_ack_callback, Create_ack_data_no_mode());
        } else {
            Fail_current_request("Vehicle already on the ground");
        }

    } else if (cmd == c_rth) {
        if (is_armed) {
            vehicle->control->goHome(Dji_ack_callback, Create_ack_data_no_mode());
        } else {
            Fail_current_request("Vehicle not armed");
        }

    } else if (cmd == c_camera_trigger_command) {
        if (vehicle->camera) {
            vehicle->camera->shootPhoto();
            Succeed_current_request();
        } else {
            Fail_current_request("Camera not detected");
        }

    } else if (cmd == c_pause) {
        Stop_mission(true);
        my_control_mode = proto::CONTROL_MODE_CLICK_GO;
        Succeed_current_request();

    } else if (cmd == c_resume) {
        if (Is_mission_present()) {
            if (Is_my_mode(proto::CONTROL_MODE_AUTO)) {
                Succeed_current_request();
            } else {
                Stop_mission(true);
                Reupload_mission_and_start();
            }
        } else {
            Fail_current_request("Route not present");
        }

    } else if (cmd == c_guided) {
        Stop_mission(true);
        my_control_mode = proto::CONTROL_MODE_CLICK_GO;
        Succeed_current_request();

    } else if (cmd == c_manual) {
        Stop_mission(true);
        Release_vehicle_control();

    } else if (cmd == c_waypoint) {
        Stop_mission(true);
        Execute_click_go(params);

    } else if (cmd == c_joystick) {
        if (cur_armed_state == ARMED_STATE_STOPPED) {
            Fail_current_request("Vehicle not armed");
        } else {
            Stop_mission(true);
            Obtain_vehicle_control(
                Make_callback([this](){
                direct_vehicle_controls.pitch = 0;
                direct_vehicle_controls.roll = 0;
                direct_vehicle_controls.yaw = 0;
                direct_vehicle_controls.throttle = 0;
                direct_vehicle_controls.last_received = std::chrono::steady_clock::now();
                my_control_mode = proto::CONTROL_MODE_JOYSTICK;
                Succeed_current_request();
            }));
        }

    } else if (cmd == c_arm) {
        if (!is_armed) {
            if (Is_control_mode(proto::CONTROL_MODE_JOYSTICK) && fabs(direct_vehicle_controls.throttle) > 0.0001) {
                Fail_current_request("Joystick throttle non-zero");
            } else {
                Obtain_vehicle_control(
                    Make_callback([this](){
                    vehicle->control->armMotors(Dji_ack_callback, Create_ack_data_no_mode());
            }));
            }
        } else {
            Fail_current_request("Already armed");
        }

    } else if (cmd == c_disarm) {
        if (is_armed) {
            Obtain_vehicle_control(
                Make_callback([this](){
                vehicle->control->disArmMotors(Dji_ack_callback, Create_ack_data_no_mode());
            }));
        } else {
            Fail_current_request("Not armed");
        }

    } else if (cmd == c_mission_upload) {
        Stop_mission(false);
        Mission_upload(params);
    } else {
        Fail_current_request("Not implemented");
    }
}

Wgs84_position
Dji_vehicle::Get_current_position()
{
    double x = 0, y = 0, z = 0;
    t_latitude->Get_value(x);
    t_longitude->Get_value(y);
    t_altitude_raw->Get_value(z);
    return Wgs84_position(Geodetic_tuple(x, y, z));
}

constexpr double WP_TOL = 0.7;
constexpr double WP_TOL_SQ = WP_TOL * WP_TOL;

// DJI does not allow WPts closer than 0.5 meters.
// If WPts are too close then increase the WP altitude to put it just outside the tolerance.
void
Adjust_wp_altitude(DJI::OSDK::WayPointSettings& wp, double x, double y, double z)
{
    auto p1 = Wgs84_position(Geodetic_tuple(x, y, z));
    auto p2 = Wgs84_position(Geodetic_tuple(wp.latitude, wp.longitude, wp.altitude));
    double d2sq = pow(p1.Distance(p2), 2);
    if (d2sq < WP_TOL_SQ) {
        double dz = (z - wp.altitude);
        double d3sq = d2sq + dz * dz;
//        LOG("d3: %0.2f z: %0.2f", sqrt(d3sq), dz);
        if (d3sq < WP_TOL_SQ) {
            wp.altitude += sqrt(WP_TOL_SQ - d2sq) - dz;
            LOG("Increasing WP altitude by %0.2f", wp.altitude - z);
        }
    }
}

void
Dji_vehicle::Create_wp_from_coords(const Property_list& params)
{
    DJI::OSDK::WayPointSettings wp;
    memset(&wp, 0, sizeof(wp));
    double tmp;
    params.Get_value("latitude", wp.latitude);
    params.Get_value("longitude", wp.longitude);
    if (params.Get_value("altitude_amsl", tmp)) {
        wp.altitude = tmp - altitude_origin;
    }
    int turn_type;
    params.Get_value("turn_type", turn_type);
    LOG("turn_type %d", turn_type);
    if (turn_type == proto::TURN_TYPE_STRAIGHT) {
        if (!bank_turn_present) {
            bank_turn_present = true;
            Add_status_message("Bank turn present in route. Some route actions will be ignored");
        }
        wp.damping = 0.2;
    } else {
        wp.damping = 0;
    }
    if (waypoints.empty()) {
        // Approximate heading to first WP to calculate turnMode for 1st WP.
        auto my_pos = Get_current_position();
        auto p2 = Wgs84_position(Geodetic_tuple(wp.latitude, wp.longitude, wp.altitude));
        current_heading = Normalize_angle_minuspi_pi(my_pos.Bearing(p2));
        prev_segment_length = my_pos.Distance(p2);
    } else {
        auto& l = Last_wp();
        // Hack to workaround when close coordinates are treated as one WP in A3.
        // Increase altitude to make sure next WP is at least 0.51 m from prev WP.
        Adjust_wp_altitude(wp, l.latitude, l.longitude, l.altitude);
        auto p1 = Wgs84_position(Geodetic_tuple(l.latitude, l.longitude, l.altitude));
        auto p2 = Wgs84_position(Geodetic_tuple(wp.latitude, wp.longitude, wp.altitude));
        float this_segment_length = p1.Distance(p2);
        if (l.damping > 0) {
            // Damping is half of the shortest segment length.
            l.damping = std::max(std::min(this_segment_length, prev_segment_length) / 2 - 0.1f, 0.2f);
        } else {
            l.damping = 0.2;
        }
        prev_segment_length = this_segment_length;
        if (heading_set_by_mission) {
            heading_set_by_mission = false;
        } else {
            if (autoheading) {
                // default to heading to next WP.
                current_heading = Normalize_angle_minuspi_pi(p1.Bearing(p2));
                Add_wp_action(WP_ACTION_CRAFT_YAW, roundf(current_heading * 180 / M_PI));
            }
        }
    }
    wp.index = ++current_wp_number;
    wp.yaw = roundf(current_heading * 180 / M_PI);
    current_command_map.Accumulate_route_id(Get_hash(wp));
    current_command_map.Add_command_mapping(wp.index);
    waypoints.emplace_back(wp);
}

void
Dji_vehicle::Duplicate_last_wp()
{
    DJI::OSDK::WayPointSettings wp;
    memset(&wp, 0, sizeof(wp));
    wp.index = ++current_wp_number;
    wp.latitude = Last_wp().latitude;
    wp.longitude= Last_wp().longitude;
    wp.altitude = Last_wp().altitude + WP_TOL;
    wp.yaw = Last_wp().yaw;
    current_command_map.Accumulate_route_id(Get_hash(wp));
    current_command_map.Add_command_mapping(wp.index);
    waypoints.emplace_back(wp);
}

void
Dji_vehicle::Stop_mission(bool save_current_position)
{
    LOG("Stopping current mission");
    vehicle->missionManager->wpMission->stop();
    vehicle->control->emergencyBrake();
    switch (cur_display_mode) {
    case DISPLAY_MODE_AUTO_LANDING:
        vehicle->control->action(DJI::OSDK::Control::FlightCommand::exitLanding);
        break;
    case DISPLAY_MODE_AUTO_TAKEOFF:
        vehicle->control->action(DJI::OSDK::Control::FlightCommand::exitTakeOff);
        break;
    case DISPLAY_MODE_NAVI_GO_HOME:
        vehicle->control->action(DJI::OSDK::Control::FlightCommand::exitGoHome);
        break;
    default:
        break;
    }
    double x, y, z;
    int wp_index;
    if (save_current_position) {
        if (t_latitude->Get_value(x) &&
            t_longitude->Get_value(y) &&
            t_altitude_raw->Get_value(z) &&
            t_current_command->Get_value(wp_index) &&
            wp_index > 0)
        {
            // Save only if there is no save position already.
            if (!mission_paused_at) {
                DJI::OSDK::WayPointSettings wp;
                memset(&wp, 0, sizeof(wp));
                wp.latitude = x;
                wp.longitude = y;
                wp.altitude = z;
                // Make sure new WP is not too close to next WP.
                Adjust_wp_altitude(wp,
                    waypoints[wp_index].latitude,
                    waypoints[wp_index].longitude,
                    waypoints[wp_index].altitude);
                wp.index = wp_index;
                wp.damping = WP_TOL / 2;;
                mission_paused_at = wp;
                LOG_WP("Paused at", wp);
            }
        }
    } else {
        mission_paused_at.Disengage();
    }
    if (trigger_by_time_timer) {
        trigger_by_time_timer->Cancel();
    }
    trigger_by_distance_state.Disengage();
}

void
Dji_vehicle::Obtain_vehicle_control(Generic_callback cb)
{
    vehicle->obtainCtrlAuthority(
        Dji_ack_callback,
        Create_ack_data(
            cb,
            Make_callback([this](DJI::OSDK::ACK::ErrorCode e, Generic_callback cb)
                {
                    if (e.data == DJI::OSDK::OpenProtocolCMD::ControlACK::SetControl::OBTAIN_CONTROL_IN_PROGRESS) {
                        Obtain_vehicle_control(cb);
                    } else if (e.data == DJI::OSDK::OpenProtocolCMD::ControlACK::SetControl::RC_MODE_ERROR) {
                        Fail_current_request("RC controller not in P mode");
                    } else if (e.data == DJI::OSDK::OpenProtocolCMD::ControlACK::SetControl::OBTAIN_CONTROL_SUCCESS) {
                        return true;
                    } else {
                        Fail_current_request(std::string("Ack code: ") + std::to_string(e.data));
                    }
                    return false;
                },
                DJI::OSDK::ACK::ErrorCode(),
                cb)));
}

void
Dji_vehicle::Release_vehicle_control()
{
    vehicle->releaseCtrlAuthority(
        Dji_ack_callback,
        Create_ack_data(
            Generic_callback(),
            Make_callback([this](DJI::OSDK::ACK::ErrorCode e)
                {
                    if (e.data == DJI::OSDK::OpenProtocolCMD::ControlACK::SetControl::RELEASE_CONTROL_IN_PROGRESS) {
                        Release_vehicle_control();
                    } else if (e.data == DJI::OSDK::OpenProtocolCMD::ControlACK::SetControl::RC_MODE_ERROR) {
                        my_control_mode.Disengage();
                        return true;
                    } else if (e.data == DJI::OSDK::OpenProtocolCMD::ControlACK::SetControl::RELEASE_CONTROL_SUCCESS) {
                        my_control_mode.Disengage();
                        return true;
                    } else {
                        Fail_current_request(std::string("Ack code: ") + std::to_string(e.data));
                    }
                    return false;
                },
                DJI::OSDK::ACK::ErrorCode())));
}

void
Dji_vehicle::Send_mission(Generic_callback send_complete_cb)
{
    unsigned start_index = 0;
    if (mission_paused_at) {
        auto my_pos = Get_current_position();
        auto p1 = Wgs84_position(Geodetic_tuple(
            mission_paused_at->latitude,
            mission_paused_at->longitude,
            mission_paused_at->altitude));
        if (my_pos.Distance(p1) > MIN_RESUME_DISTANCE) {
            // Will return back to the point where mission was interrupted.
            LOG("return to paused loc");
            uploaded_wp_offset = mission_paused_at->index - 1;
            first_wp_is_dummy = true;
        } else {
            // resume position is too close. Skip it and proceed to next WP in mission.
            LOG("paused loc too close.");
            start_index = mission_paused_at->index;
            if (start_index + 1 == waypoints.size()) {
                // We have only 1 WP left. As DJI does not support 1WP missions we add one more dummy WP before it.
                mission_paused_at = waypoints[start_index];
                mission_paused_at->altitude += WP_TOL;
                uploaded_wp_offset = mission_paused_at->index - 1;
                first_wp_is_dummy = true;
            } else {
                uploaded_wp_offset = mission_paused_at->index;
                mission_paused_at.Disengage();
                first_wp_is_dummy = false;
            }
        }
        waypoint_init_data.indexNumber = waypoints.size() - uploaded_wp_offset;
    } else {
        uploaded_wp_offset = 0;
        waypoint_init_data.indexNumber = waypoints.size();
        first_wp_is_dummy = false;
    }

    VEHICLE_LOG_INF(*this, "WP count=%d, maxspeed=%0.2f traceMode=%d offset=%d",
        waypoint_init_data.indexNumber,
        waypoint_init_data.maxVelocity,
        waypoint_init_data.traceMode,
        uploaded_wp_offset);

    route_uploaded_callback = send_complete_cb;

    Obtain_vehicle_control(
        Make_callback([this](int start)
        {
            vehicle->missionManager->wpMission->init(
                &waypoint_init_data,
                Dji_ack_callback,
                Create_ack_data(
                    Make_callback(
                        &Dji_vehicle::Send_wp,
                        Shared_from_this(),
                        start)));
        },
        start_index));
}

void
Dji_vehicle::Send_wp(unsigned index)
{
    if (index > waypoints.size()) {
        Fail_current_request("index exceeds size");
        return;
    }
    DJI::OSDK::WayPointSettings wp = waypoints[index];
    if (mission_paused_at) {
        wp = *mission_paused_at;
        index = wp.index - 1;
        wp.index = 0;
        mission_paused_at.Disengage();
    } else {
        wp.index = index - uploaded_wp_offset;
    }

    Generic_callback cb;
    if (index + 1 == waypoints.size()) {
        // this is last WP to upload.
        if (route_uploaded_callback) {
            cb = route_uploaded_callback;
        }
    } else {
        cb = Make_callback(&Dji_vehicle::Send_wp, Shared_from_this(), index + 1);
    }
    LOG_WP("Uploading", wp);
    vehicle->missionManager->wpMission->uploadIndexData(
        &wp,
        Dji_ack_callback,
        Create_ack_data(cb));
}

void
Dji_vehicle::Reupload_mission_and_start()
{
    if (Is_mission_present()) {
        Send_mission(Make_callback(&Dji_vehicle::Send_start, Shared_from_this()));
    } else {
        Fail_current_request("Route not present");
    }
}

void
Dji_vehicle::Execute_click_go(const Property_list& params)
{
    DJI::OSDK::WayPointInitSettings init_data;
    DJI::OSDK::WayPointSettings wp1;
    DJI::OSDK::WayPointSettings wp2;

    // DJI does not support 1WP mission therefore we create two WP mission.
    // wp2 is the specified waypoint
    // wp1 is at the same location but .5 m above wp2.
    memset(&init_data, 0, sizeof(init_data));
    init_data.executiveTimes = 1;    // do it only once.
    init_data.RCLostAction   = RC_LOST_ACTION_CONTINUE;
    init_data.finishAction   = FINISH_ACTION_NONE;
    init_data.gimbalPitch    = GIMBAL_PITCH_MODE_AUTO;
    init_data.traceMode      = TRACE_MODE_STOP;
    init_data.yawMode        = YAW_MODE_AUTO;
    init_data.indexNumber    = 2;   // two WP mission

    memset(&wp2, 0, sizeof(wp2));
    double tmp, tmp1;
    wp2.damping = WP_TOL / 2;
    wp2.index = 1;
    params.Get_value("latitude", wp2.latitude);
    params.Get_value("longitude", wp2.longitude);
    // Cannot read directly into wp2.altitude because it causes memory alignment fault on RPi.
    params.Get_value("altitude_amsl", tmp);
    wp2.altitude = tmp;
    params.Get_value("altitude_origin", tmp);
    wp2.altitude -= tmp;
    if (wp2.altitude < 0) {
        Fail_current_request("Target altitude below altitude_origin");
        return;
    }
    if (params.Get_value("heading", tmp)) {
        init_data.yawMode = YAW_MODE_WP;
        wp2.yaw = Normalize_angle_minuspi_pi(tmp) * 180 / M_PI;
    }
    tmp = 0;
    tmp1 = 0;
    params.Get_value("vertical_speed", tmp);
    params.Get_value("ground_speed", tmp1);
    if (tmp || tmp1) {
        init_data.idleVelocity = hypot(tmp, tmp1);
    } else {
        init_data.idleVelocity = DEFAULT_SPEED;
    }
    if (init_data.idleVelocity > DEFAULT_SPEED) {
        init_data.maxVelocity = init_data.idleVelocity;
    } else {
        init_data.maxVelocity = DEFAULT_SPEED;
    }

    wp1 = wp2;
    wp1.index = 0;
    wp1.altitude += WP_TOL;

    // Set up the callback chain.
    // Unfortunately we cannot create callbacks directly from API calls because they all have two overloads
    // and Make_callback cannot resolve overloads. Therefore lambdas are being used.
    auto start_click_go = Make_callback([this]() {
        LOG("Starting click&go");
        vehicle->missionManager->wpMission->start(
            Dji_ack_callback,
            Create_ack_data(proto::CONTROL_MODE_CLICK_GO));
    });
    auto send_second_wp = Make_callback([this](DJI::OSDK::WayPointSettings wp, Generic_callback next) {
        LOG_WP("Sending 2nd", wp);
        vehicle->missionManager->wpMission->uploadIndexData(
            &wp,
            Dji_ack_callback,
            Create_ack_data(next));
    },
    wp2,
    start_click_go);
    auto send_first_wp = Make_callback([this](DJI::OSDK::WayPointSettings wp, Generic_callback next) {
        LOG_WP("Sending 1st", wp);
        vehicle->missionManager->wpMission->uploadIndexData(
            &wp,
            Dji_ack_callback,
            Create_ack_data(next));
    },
    wp1,
    send_second_wp);
    auto init_mission = Make_callback([this](DJI::OSDK::WayPointInitSettings init, Generic_callback next) {
        LOG("Init click&go");
        vehicle->missionManager->wpMission->init(
            &init,
            Dji_ack_callback,
            Create_ack_data(next));
    },
    init_data,
    send_first_wp);

    // Execute the callback chain.
    Obtain_vehicle_control(init_mission);
}

void
Dji_vehicle::Add_wp_action(Wp_action action, int16_t data)
{
    if (Last_wp().actionNumber == 16) {
        VEHICLE_LOG_ERR(*this, "WP action limit exceeded. Action %d %d ignored", action, data);
        return;
    }
    switch (action) {
    case WP_ACTION_CRAFT_YAW:
    case WP_ACTION_GIMBAL_PITCH:
        Last_wp().actionTimeLimit += 10;
        break;
    case WP_ACTION_WAIT:
        Last_wp().actionTimeLimit += (data / 1000) + 1;
        break;
    case WP_ACTION_SIMPLE_SHOT:
    case WP_ACTION_VIDEO_START:
    case WP_ACTION_VIDEO_STOP:
        Last_wp().actionTimeLimit += 2;
        break;
    }
    Last_wp().hasAction = 1;
    Last_wp().actionRepeat = 1;
    Last_wp().commandList[Last_wp().actionNumber] = action;
    Last_wp().commandParameter[Last_wp().actionNumber] = data;
    Last_wp().actionNumber++;
}

void
Dji_vehicle::Mission_upload(const Property_list& route_params)
{
    bank_turn_present = false;
    current_wp_number = -1;
    waypoints.clear();
    waypoint_data.clear();
    t_current_mission_id->Set_value_na();
    mission_paused_at.Disengage();

    memset(&waypoint_init_data, 0, sizeof(waypoint_init_data));
    waypoint_init_data.executiveTimes = 1;    // do it only once.
    waypoint_init_data.RCLostAction   = RC_LOST_ACTION_CONTINUE;
    waypoint_init_data.finishAction   = FINISH_ACTION_NONE;
    waypoint_init_data.gimbalPitch    = GIMBAL_PITCH_MODE_AUTO;
    waypoint_init_data.traceMode      = TRACE_MODE_STOP;
    waypoint_init_data.yawMode        = YAW_MODE_RC;

    auto &vsm_cmd = current_ucs_request->request.device_commands(0);

    route_params.at("altitude_origin")->Get_value(altitude_origin);

    int fs_value;
    if (route_params.Get_value("rc_loss_action", fs_value)) {
        switch (fs_value) {
        case proto::FAILSAFE_ACTION_LAND:
        case proto::FAILSAFE_ACTION_WAIT:
            Fail_current_request("Unsupported rcloss action");
            return;
        case proto::FAILSAFE_ACTION_RTH:
            rth_on_rc_loss = true;
            break;
        case proto::FAILSAFE_ACTION_CONTINUE:
            rth_on_rc_loss = false;
            break;
        }
    }

    if (rth_on_rc_loss) {
        LOG_INFO("Vehicle will RTH on RC loss");
    }

    current_command_map.Reset();

    for (int i = 0; i < vsm_cmd.sub_commands_size(); i++) {
        current_command_map.Set_current_command(i);
        auto vsm_scmd = vsm_cmd.sub_commands(i);
        auto cmd = Get_command(vsm_scmd.command_id());
        if (!cmd->Is_mission_item()) {
            waypoints.clear();
            Fail_current_request("Unsupported command in mission");
            return;
        }
        VEHICLE_LOG_INF(*this, "ROUTE item %s", Dump_command(vsm_scmd).c_str());
        auto params = cmd->Build_parameter_list(vsm_scmd);
        float fval;
        int ival;
        if (cmd == c_takeoff_mission) {
            Create_wp_from_coords(params);

        } else if (cmd == c_land_mission) {
            Create_wp_from_coords(params);
            waypoint_init_data.finishAction = FINISH_ACTION_LAND;

        } else if (cmd == c_move) {
            Create_wp_from_coords(params);

        } else if (cmd == c_set_heading) {
            params.Get_value("heading", fval);
            current_heading = Normalize_angle_minuspi_pi(fval);
            Add_wp_action(WP_ACTION_CRAFT_YAW, roundf(current_heading * 180 / M_PI));
            heading_set_by_mission = true;

        } else if (cmd == c_wait) {
            fval = 0;
            params.Get_value("time", fval);
            if (fval > 0) {
                Add_wp_action(WP_ACTION_WAIT, fval * 1000);
            }

        } else if (cmd == c_payload_control) {
            params.Get_value("tilt", fval);
            Add_wp_action(WP_ACTION_GIMBAL_PITCH, Normalize_angle_minuspi_pi(fval) * 180.0 / M_PI);

        } else if (cmd == c_camera_trigger_mission) {
            params.Get_value("state", ival);
            switch (ival) {
            case proto::CAMERA_MISSION_TRIGGER_STATE_SINGLE_PHOTO:
                Add_wp_action(WP_ACTION_SIMPLE_SHOT, 0);
                break;
            case proto::CAMERA_MISSION_TRIGGER_STATE_ON:
                Add_wp_action(WP_ACTION_VIDEO_START, 0);
                break;
            case proto::CAMERA_MISSION_TRIGGER_STATE_OFF:
                Add_wp_action(WP_ACTION_VIDEO_STOP, 0);
                break;
            case proto::CAMERA_MISSION_TRIGGER_STATE_SERIAL_PHOTO:
                LOG_ERR("serial photo not supported.");
                break;
            }

        } else if (cmd == c_set_speed) {
            float vx, vy, v;
            params.Get_value("ground_speed", vx);
            params.Get_value("vertical_speed", vy);
            v = hypot(vx,vy);
            Last_wp_data().speed = v;
            if (v > waypoint_init_data.maxVelocity) {
                waypoint_init_data.maxVelocity = v;
            }

        } else if (cmd == c_camera_by_distance) {
            if (Last_wp_data().trigger_by_time) {
                waypoints.clear();
                Fail_current_request("Only one camera series action allowed per waypiont.");
                return;
            }
            int tmp = 0;
            float tmpf = 0;
            params.Get_value("count", tmp);
            if (tmp >= 0) {
                Last_wp_data().count = tmp;
            } else {
                waypoints.clear();
                Fail_current_request("Invalid trigger count");
                break;
            }
            tmpf = 0;
            params.Get_value("delay", tmpf);
            if (tmpf >= 0) {
                Last_wp_data().delay = std::chrono::milliseconds(static_cast<int>(tmpf * 1000));
            } else {
                waypoints.clear();
                Fail_current_request("Invalid trigger delay");
                break;
            }
            tmpf = 0;
            params.Get_value("distance", tmpf);
            if (tmpf > 0) {
                Last_wp_data().distance = tmpf;
            } else {
                waypoints.clear();
                Fail_current_request("Invalid trigger distance");
                break;
            }
            Last_wp_data().trigger_by_time = false;

        } else if (cmd == c_camera_by_time) {
            if (Last_wp_data().trigger_by_time) {
                waypoints.clear();
                Fail_current_request("Only one camera series action allowed per waypiont.");
                return;
            }
            int tmp = 0;
            float tmpf = 0;
            params.Get_value("count", tmp);
            if (tmp >= 0) {
                Last_wp_data().count = tmp;
            } else {
                waypoints.clear();
                Fail_current_request("Invalid trigger count");
                break;
            }
            tmpf = 0;
            params.Get_value("delay", tmpf);
            if (tmpf >= 0) {
                Last_wp_data().delay = std::chrono::milliseconds(static_cast<int>(tmpf * 1000));
            } else {
                waypoints.clear();
                Fail_current_request("Invalid trigger delay");
                break;
            }
            tmpf = 0;
            params.Get_value("period", tmpf);
            if (tmpf > 0) {
                Last_wp_data().period = std::chrono::milliseconds(static_cast<int>(tmpf * 1000));
            } else {
                waypoints.clear();
                Fail_current_request("Invalid trigger period");
                break;
            }
            Last_wp_data().trigger_by_time = true;

        } else {
            VEHICLE_LOG_DBG(*this, "command %s ignored", cmd->Get_name().c_str());
        }
        if (waypoints.size() && Last_wp().altitude < 0) {
            Fail_current_request("WP below altitude_origin");
            return;
        }
    }

    if (bank_turn_present) {
        waypoint_init_data.traceMode = TRACE_MODE_SMOOTH;
    }

    if (waypoints.size() == 1) {
        // DJI does not allow mission with one WP. Add another one above first.
        Duplicate_last_wp();
    } else if (waypoints.empty()) {
        Fail_current_request("No waypoints defined");
        return;
    }

    waypoint_init_data.indexNumber = waypoints.size();
    if (waypoint_init_data.maxVelocity < DEFAULT_SPEED) {
        waypoint_init_data.maxVelocity = DEFAULT_SPEED;
    }
    waypoint_init_data.idleVelocity = waypoint_init_data.maxVelocity;   // Use max speed as approach speed.

    Send_mission(
        Make_callback([this]{
            current_command_map.Fill_command_mapping_response(current_ucs_request->response);
            t_current_mission_id->Set_value(current_command_map.Get_route_id());
            my_control_mode.Disengage();
            Succeed_current_request();
        }));
}

void
Dji_vehicle::Send_start()
{
    VEHICLE_LOG_INF(*this, "Starting auto flight");
    vehicle->missionManager->wpMission->start(
        Dji_ack_callback,
        Create_ack_data(
            Make_callback([this]{
                my_control_mode = proto::CONTROL_MODE_AUTO;
                t_current_command->Set_value_na();
                Succeed_current_request();
            })));
}

void
Dji_vehicle::Wait_done()
{
    std::unique_lock<std::mutex> lock(exit_mutex);
    exit_cond_var.wait(lock, [&]{ return exit_required;});
}

void
Dji_vehicle::Process_ack(
    DJI::OSDK::RecvContainer recv,
    Generic_callback next_action,
    Generic_callback_with_error error_action)
{
    if (!recv.dispatchInfo.isAck) {
        Fail_current_request("Unexpected response instead of ack");
    } else {
        DJI::OSDK::ACK::ErrorCode ack;
        ack.info = recv.recvInfo;
        ack.data = recv.recvData.commandACK;
        if (DJI::OSDK::ACK::getError(ack)) {
            if (error_action) {
                error_action.Set_arg<0>(ack);
                if (error_action.Invoke()) {
                    if (next_action) {
                        next_action.Invoke();
                    } else {
                        Succeed_current_request();
                    }
                } // else error_action must have completed the request.
            } else {
                DJI::OSDK::Log::instance().enableStatusLogging();
                DJI::OSDK::ACK::getErrorCodeMessage(ack, __func__);
                DJI::OSDK::Log::instance().disableStatusLogging();
                Fail_current_request(std::string("Ack code: ") + std::to_string(recv.recvData.commandACK));
            }
        } else {
            if (next_action) {
                next_action.Invoke();
            } else {
                Succeed_current_request();
            }
        }
    }
}

bool
Dji_vehicle::Is_my_mode(proto::Control_mode mode)
{
    return my_control_mode && *my_control_mode == mode;
}

bool
Dji_vehicle::Do_trigger_by_time(int shot_count, std::chrono::milliseconds period)
{
    vehicle->camera->shootPhoto();
    if (shot_count) {
        LOG("trigger_by_time, shots left: %d", shot_count - 1);
        if (shot_count == 1) {
            return false;  // Last trigger we are done here.
        }
        shot_count--;
    } else {
        // Infinite trigger.
        LOG("trigger_by_time");
    }
    trigger_by_time_timer = Timer_processor::Get_instance()->Create_timer(
        period,
        Make_callback(
            &Dji_vehicle::Do_trigger_by_time,
            Shared_from_this(),
            shot_count,
            period),
        Get_completion_ctx());
    return false;
}

bool
Dji_vehicle::Initialize_trigger_by_distance(int shot_count, float distance)
{
    vehicle->camera->shootPhoto();
    if (shot_count) {
        LOG("trigger_by_distance, shots left: %d", shot_count - 1);
        if (shot_count == 1) {
            return false;  // 1 trigger, we are done here.
        }
        // 2 or more triggers
        shot_count--;
    } else {
        // infinite trigger
        LOG("trigger_by_distance");
    }

    trigger_by_distance_state = Trigger_by_distance_state {
        shot_count,
        distance,
        Get_current_position()};
    return false;
}

void
Dji_vehicle::Process_trigger_by_distance()
{
    if (!trigger_by_distance_state) {
        return;
    }
    // Calculate distance from previous shot.
    auto pos = Get_current_position();
    if (trigger_by_distance_state->prev_location.Distance(pos) > trigger_by_distance_state->distance) {
        vehicle->camera->shootPhoto();
        if (trigger_by_distance_state->count) {
            LOG("trigger_by_distance, shots left: %d", trigger_by_distance_state->count - 1);
            if (trigger_by_distance_state->count == 1) {
                trigger_by_distance_state.Disengage();
                return;
            } else {
                trigger_by_distance_state->count--;
            }
        } else {
            // infinite trigger
            LOG("trigger_by_distance");
        }
        trigger_by_distance_state->prev_location = pos;
    }
}

void
Dji_vehicle::Process_waypoint(DJI::OSDK::RecvContainer recv)
{
    // This callback is relevant only if flying uploaded/resumed mission, not click&go.
    if (!Is_my_mode(proto::CONTROL_MODE_AUTO)) {
        t_current_command->Set_value_na();
        return;
    }
    if (recv.recvInfo.cmd_id == DJI::OSDK::OpenProtocolCMD::CMDSet::Broadcast::mission[1]) {
        const auto & data = recv.recvData.wayPointStatusPushData;
//        LOG("mission : %d, mission_type: %d status: %d",
//            data.waypoint_index,
//            data.mission_type,
//            data.current_status);
        if (data.mission_type == DJI::OSDK::NAVI_MISSION_WAYPOINT && data.current_status != WP_STATUS_LAST_WP) {
            if (data.waypoint_index == 0 &&  first_wp_is_dummy) {
                // Report next WP while going to the dummy WP.
                t_current_command->Set_value(data.waypoint_index + uploaded_wp_offset + 1);
            } else {
                t_current_command->Set_value(data.waypoint_index + uploaded_wp_offset);
            }
        }
    } else if (recv.recvInfo.cmd_id == DJI::OSDK::OpenProtocolCMD::CMDSet::Broadcast::waypoint[1]) {
        const auto & data = recv.recvData.wayPointReachedData;
        LOG("waypoint: %d, incident: %d status: %d",
            data.waypoint_index,
            data.incident_type,
            data.current_status);
        switch (data.incident_type) {
        case DJI::OSDK::NAVI_MISSION_WP_REACH_POINT: {
            if (data.waypoint_index == 0 && first_wp_is_dummy) {
                break;  // not doing actions for dummy wp.
            }
            switch (data.current_status) {
            case WP_REACHED_STATUS_PRE_ACTION:
                if (trigger_by_time_timer) {
                    trigger_by_time_timer->Cancel();
                }
                trigger_by_distance_state.Disengage();
                break;
            case WP_REACHED_STATUS_POST_ACTION:
                auto f = waypoint_data.find(data.waypoint_index + uploaded_wp_offset);
                if (f != waypoint_data.end()) {
                    if (f->second.speed) {
                        vehicle->missionManager->wpMission->updateIdleVelocity(*(f->second.speed));
                    }

                    if (f->second.trigger_by_time) {
                        if (*(f->second.trigger_by_time)) {
                            // Initiate trigger by time
                            auto first_delay = f->second.delay;
                            auto shot_count = f->second.count;

                            if (!first_delay.count()) {
                                // no delay present, do single shot right away.
                                vehicle->camera->shootPhoto();
                                if (shot_count) {
                                    LOG("trigger_by_time, shots left: %d", shot_count - 1);
                                    if (shot_count == 1) {
                                        break;  // 1 trigger, no delay, we are done here.
                                    }
                                    // 2 or more triggers, no delay.
                                    shot_count--;
                                } else {
                                    // infinite trigger, no delay.
                                    LOG("trigger_by_time");
                                }
                                first_delay = f->second.period;
                            }
                            // There are more than one shot or delay required. Launch timer.
                            trigger_by_time_timer = Timer_processor::Get_instance()->Create_timer(
                                first_delay,
                                Make_callback(
                                    &Dji_vehicle::Do_trigger_by_time,
                                    Shared_from_this(),
                                    shot_count,
                                    f->second.period),
                                Get_completion_ctx());
                        } else {
                            // Initiate trigger by distance.
                            if (f->second.delay.count()) {
                                trigger_by_time_timer = Timer_processor::Get_instance()->Create_timer(
                                    f->second.delay,
                                    Make_callback(
                                        &Dji_vehicle::Initialize_trigger_by_distance,
                                        Shared_from_this(),
                                        f->second.count,
                                        f->second.distance),
                                    Get_completion_ctx());
                            } else {
                                // no delay present, do single shot right away.
                                Initialize_trigger_by_distance(f->second.count, f->second.distance);
                            }
                        }
                    }
                }
                break;
            }
            break;
        }
        case DJI::OSDK::NAVI_MISSION_FINISH:
            t_current_command->Set_value_na();
            my_control_mode.Disengage();
            break;
        }
    }
}

void
Dji_vehicle::Signal_done()
{
    std::unique_lock<std::mutex> lock(exit_mutex);
    exit_required = true;
    exit_cond_var.notify_all();
}

void
Dji_vehicle::Process_telemetry(int package)
{
    last_telemetry_received = std::chrono::steady_clock::now();
    // For all topics in this pack.
    for (auto it : subscribed_topics) {
        if (it.second == package) {
            switch (it.first)
            {
            case DJI::OSDK::Telemetry::TOPIC_VELOCITY: {
                auto t = vehicle->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_VELOCITY>();
                t_ground_speed->Set_value(hypot(t.data.x, t.data.y));
                t_vertical_speed->Set_value(t.data.z);
            } break;
            case DJI::OSDK::Telemetry::TOPIC_BATTERY_INFO: {
                auto t = vehicle->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_BATTERY_INFO>();
                t_main_voltage->Set_value(t.voltage / 1000.0);    // experiments show voltage is in millivolts.
            } break;
            case DJI::OSDK::Telemetry::TOPIC_GPS_FUSED: {
                auto t = vehicle->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_GPS_FUSED>();
                t_latitude->Set_value(t.latitude);
                t_longitude->Set_value(t.longitude);
                t_satellite_count->Set_value(t.visibleSatelliteNumber);
                Process_trigger_by_distance();
            } break;
            case DJI::OSDK::Telemetry::TOPIC_ALTITUDE_FUSIONED: {
                auto t = vehicle->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_ALTITUDE_FUSIONED>();
                t_altitude_amsl->Set_value(t);
            } break;
            case DJI::OSDK::Telemetry::TOPIC_HEIGHT_HOMEPOINT: {
                auto t = vehicle->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_HEIGHT_HOMEPOINT>();
                t_home_altitude_amsl->Set_value(t);
            } break;
            case DJI::OSDK::Telemetry::TOPIC_QUATERNION: {
                auto t = vehicle->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_QUATERNION>();
                auto e = Quaternion_to_eulers(t);
                t_pitch->Set_value(e.x);
                t_roll->Set_value(e.y);
                t_heading->Set_value(e.z);
            } break;
            case DJI::OSDK::Telemetry::TOPIC_STATUS_FLIGHT: {
                auto t = vehicle->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_STATUS_FLIGHT>();
                Update_modes(static_cast<Armed_state>(t), cur_display_mode, cur_control_source);
            } break;
            case DJI::OSDK::Telemetry::TOPIC_STATUS_DISPLAYMODE: {
                auto t = vehicle->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_STATUS_DISPLAYMODE>();
                Update_modes(cur_armed_state, static_cast<Display_mode>(t), cur_control_source);
            } break;
            case DJI::OSDK::Telemetry::TOPIC_CONTROL_DEVICE: {
                auto t = vehicle->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_CONTROL_DEVICE>();
                //LOG("controlMode flightStatus vrcStatus %d %d %d", t.controlMode, t.flightStatus, t.vrcStatus);
                Update_modes(cur_armed_state, cur_display_mode, static_cast<Control_source>(t.deviceStatus));
            } break;
            case DJI::OSDK::Telemetry::TOPIC_RC_WITH_FLAG_DATA: {
                auto t = vehicle->subscribe->getValue<DJI::OSDK::Telemetry::TOPIC_RC_WITH_FLAG_DATA>();
                t_rc_link_quality->Set_value(t.flag.groundConnected);
                if (t.flag.groundConnected) {
                    last_rc_ok_time = std::chrono::steady_clock::now();
                }
            } break;
            default:
            break;
            }
        }
    }
    float a1, a2;
    if (t_altitude_amsl->Get_value(a1) && t_home_altitude_amsl->Get_value(a2)) {
        t_altitude_raw->Set_value(a1 - a2);
    }
    Commit_to_ucs();
}

void
Dji_vehicle::Update_modes(Armed_state arm, Display_mode disp, Control_source ctrl)
{
    is_armed = (arm == ARMED_STATE_IN_AIR || arm == ARMED_STATE_ON_GROUND);
    if (arm == ARMED_STATE_UNKNOWN) {
        t_is_armed->Set_value_na();
    } else {
        t_is_armed->Set_value(arm == ARMED_STATE_IN_AIR || arm == ARMED_STATE_ON_GROUND);
    }

    if (cur_control_source != ctrl) {
        VEHICLE_LOG_DBG(*this, "CONTROL %d->%d", cur_control_source, ctrl);
        Add_status_message(std::string("Native control mode ") + std::to_string(ctrl));
    }
    if (cur_display_mode != disp) {
        VEHICLE_LOG_DBG(*this, "DISPMODE %d->%d", cur_display_mode, disp);
        Add_status_message(std::string("Native flight mode ") + std::to_string(disp));
    }
    if (cur_armed_state != arm) {
        VEHICLE_LOG_DBG(*this, "ARM %d->%d", cur_armed_state, arm);
        Add_status_message(std::string("Arm mode ") + std::to_string(arm));
    }

    int tmp;
    t_rc_link_quality->Get_value(tmp);
    bool is_rc_connected = tmp > 0;

    Optional<proto::Control_mode> control_mode_from_vehicle;
    switch (disp) {
    case DISPLAY_MODE_ASSISTED_TAKEOFF:
        control_mode_from_vehicle = proto::CONTROL_MODE_MANUAL;
        current_flight_mode = proto::FLIGHT_MODE_TAKEOFF;
        break;
    case DISPLAY_MODE_ATTITUDE:
        control_mode_from_vehicle = proto::CONTROL_MODE_MANUAL;
        current_flight_mode.Disengage();
        break;
    case DISPLAY_MODE_AUTO_LANDING:
        control_mode_from_vehicle = proto::CONTROL_MODE_AUTO;
        current_flight_mode = proto::FLIGHT_MODE_LAND;
        break;
    case DISPLAY_MODE_AUTO_TAKEOFF:
        control_mode_from_vehicle = proto::CONTROL_MODE_AUTO;
        current_flight_mode = proto::FLIGHT_MODE_TAKEOFF;
        break;
    case DISPLAY_MODE_ENGINE_START:
        current_flight_mode.Disengage();
        break;
    case DISPLAY_MODE_FORCE_AUTO_LANDING:
        control_mode_from_vehicle = proto::CONTROL_MODE_AUTO;
        current_flight_mode = proto::FLIGHT_MODE_LAND;
        break;
    case DISPLAY_MODE_HOTPOINT_MODE:
        current_flight_mode.Disengage();  // unsupported by vsm sdk for now.
        break;
    case DISPLAY_MODE_MANUAL_CTRL:
        control_mode_from_vehicle = proto::CONTROL_MODE_MANUAL;
        current_flight_mode.Disengage();
        break;
    case DISPLAY_MODE_NAVI_GO_HOME:
        control_mode_from_vehicle = proto::CONTROL_MODE_AUTO;
        current_flight_mode = proto::FLIGHT_MODE_RTH;
        break;
    case DISPLAY_MODE_NAVI_SDK_CTRL:    // joystick mode.
        // do not set my_control mode here as it should already be set.
        control_mode_from_vehicle = proto::CONTROL_MODE_AUTO;
        current_flight_mode.Disengage();
        break;
    case DISPLAY_MODE_F:
        control_mode_from_vehicle = proto::CONTROL_MODE_MANUAL;
        current_flight_mode.Disengage();
        break;
    case DISPLAY_MODE_P_GPS:
        // It look strange when RC is off, but copter is in Manual Control mode. Let's shadow it.
        if (is_rc_connected > 0) {
            control_mode_from_vehicle = proto::CONTROL_MODE_MANUAL;
        } else {
            control_mode_from_vehicle = proto::CONTROL_MODE_AUTO;
        }
        current_flight_mode = proto::FLIGHT_MODE_HOLD;
        break;
    case DISPLAY_MODE_SEARCH_MODE:
        control_mode_from_vehicle = proto::CONTROL_MODE_AUTO;
        current_flight_mode.Disengage();  // unsupported by vsm sdk for now.
        break;
    case DISPLAY_MODE_WP:
        control_mode_from_vehicle = proto::CONTROL_MODE_AUTO;
        current_flight_mode = proto::FLIGHT_MODE_WAYPOINTS;
        break;
    case DISPLAY_MODE_UNKNOWN:
        current_flight_mode.Disengage();
        break;
    }   

    if (ctrl == CONTORL_SOURCE_SERIAL) {
        // Vehicle is controlled by Onboard SDK. I.e. VSM
        if (my_control_mode) {
            t_control_mode->Set_value(*my_control_mode);
        } else if (control_mode_from_vehicle) {
            t_control_mode->Set_value(*control_mode_from_vehicle);
        } else {
            t_control_mode->Set_value_na();
        }
    } else if (is_rc_connected > 0){
        t_control_mode->Set_value(proto::CONTROL_MODE_MANUAL);
    } else {
        t_control_mode->Set_value(proto::CONTROL_MODE_AUTO);
    }

    if (current_flight_mode) {
        t_flight_mode->Set_value(*current_flight_mode);
    } else {
        t_flight_mode->Set_value_na();
    }
    cur_armed_state = arm;
    cur_display_mode = disp;
    cur_control_source = ctrl;

    auto in_rc_mode =
        control_mode_from_vehicle &&
        *control_mode_from_vehicle == proto::CONTROL_MODE_MANUAL &&
        disp != DISPLAY_MODE_P_GPS;

    c_mission_upload->Set_enabled(!in_rc_mode);
    c_rth->Set_enabled(cur_armed_state == ARMED_STATE_IN_AIR);
    c_takeoff_command->Set_enabled(cur_armed_state == ARMED_STATE_STOPPED);
    c_arm->Set_enabled(
        cur_armed_state == ARMED_STATE_STOPPED &&
        (!Is_control_mode(proto::CONTROL_MODE_JOYSTICK) || fabs(direct_vehicle_controls.throttle) < 0.0001));
    c_land_command->Set_enabled(cur_armed_state == ARMED_STATE_IN_AIR);
    c_disarm->Set_enabled(cur_armed_state == ARMED_STATE_ON_GROUND);
    c_manual->Set_enabled(!Is_control_mode(proto::CONTROL_MODE_MANUAL) && !in_rc_mode && is_rc_connected > 0);
    c_joystick->Set_enabled(!Is_control_mode(proto::CONTROL_MODE_JOYSTICK) && !in_rc_mode);
    c_guided->Set_enabled(!Is_control_mode(proto::CONTROL_MODE_CLICK_GO) && !in_rc_mode);
    c_direct_vehicle_control->Set_enabled(Is_control_mode(proto::CONTROL_MODE_JOYSTICK) && !in_rc_mode);
    c_waypoint->Set_enabled(cur_armed_state == ARMED_STATE_IN_AIR && !in_rc_mode);
    if (Is_mission_present() && !in_rc_mode) {
        c_auto->Set_enabled();
        c_pause->Set_enabled(Is_control_mode(proto::CONTROL_MODE_AUTO));
        c_resume->Set_enabled(!Is_control_mode(proto::CONTROL_MODE_AUTO));
    } else {
        c_auto->Set_enabled(false);
        c_pause->Set_enabled(false);
        c_resume->Set_enabled(false);
    }
}

void
Dji_vehicle::Subscribe_telemetry_topic(int frequency, DJI::OSDK::Telemetry::TopicName topic)
{
    if (frequency && (frequency > DJI::OSDK::Telemetry::TopicDataBase[topic].maxFreq || (400 % frequency) != 0)) {
        LOG_ERR("Invalid frequency %d for topic %d", frequency, topic);
        return;
    }
    int pack = MAX_PACKS;
    int topic_count = 0;
    DJI::OSDK::Telemetry::TopicName topic_list[DJI::OSDK::Telemetry::TOTAL_TOPIC_NUMBER];
    int prev_frequency = 0;

    // First, look if we have subscribed this topic already
    auto ti = subscribed_topics.find(topic);
    if (ti != subscribed_topics.end()) {
        // We have subscribed for this topic already.
        pack = ti->second;
        if (frequencies[pack] == frequency) {
            return; // No changes required.
        } else {
            // Frequency change requested. Remove from subscriptions.
            if (DJI::OSDK::ACK::getError(vehicle->subscribe->removePackage(pack, 1)) == DJI::OSDK::ACK::SUCCESS) {
                // Remove removed topics from our map.
                for (auto it = subscribed_topics.begin(); it != subscribed_topics.end(); ) {
                    if (it->second == pack) {
                        if (it->first != topic) {
                            topic_list[topic_count++] = it->first;
                        }
                        it = subscribed_topics.erase(it);
                    } else {
                        it++;
                    }
                }
                prev_frequency = frequencies[pack];
                frequencies[pack] = 0;
            } else {
                LOG_ERR("Failed to remove package %d with freq %d", pack, frequency);
                return;
            }
        }
    }

    // topic_list now contains topics with the same frequency as given topic
    // had previously and which are now removed from map.
    if (topic_count) {
        // Need to subscribe back to remaining unsubscribed topics.
        if (!vehicle->subscribe->initPackageFromTopicList(pack, topic_count, topic_list, false, prev_frequency)) {
            LOG_ERR("Failed to init package %d with freq %d", pack, prev_frequency);
            return;
        }
        if (DJI::OSDK::ACK::getError(vehicle->subscribe->startPackage(pack, 1)) != DJI::OSDK::ACK::SUCCESS) {
            vehicle->subscribe->removePackage(pack, 1);
            LOG_ERR("Failed to start package %d with freq %d", pack, prev_frequency);
            return;
        }
        // Put back subscribed topics into our map.
        for (int i = 0; i < topic_count; i++) {
            subscribed_topics.emplace(topic_list[i], pack);
        }
        frequencies[pack] = prev_frequency;
    }

    if (frequency == 0) {
        return; // we are done unsubscribing the topic.
    }

    topic_count = 0;
    // Find a slot for given frequency
    for (pack = 0; pack < MAX_PACKS; pack++) {
        if (frequencies[pack] == frequency) {
            // we have other topics subscribed on this frequency. Must unsubscribe all the current topics.
            if (DJI::OSDK::ACK::getError(vehicle->subscribe->removePackage(pack, 1)) == DJI::OSDK::ACK::SUCCESS) {
                for (auto it = subscribed_topics.begin(); it != subscribed_topics.end(); ) {
                    if (it->second == pack) {
                        topic_list[topic_count++] = it->first;
                        it = subscribed_topics.erase(it);
                    } else {
                        it++;
                    }
                }
                frequencies[pack] = 0;
                break;
            } else {
                LOG_ERR("Failed to remove package %d with freq %d", pack, frequency);
                return;
            }
        }
    }

    // topic_list now contains topics with the same frequency as given and which are now removed from map.
    if (pack == MAX_PACKS) {
        // Find a new free slot in frequencies
        for (pack = 0; pack < MAX_PACKS; pack++) {
            if (frequencies[pack] == 0) {
                break;
            }
        }
    }

    if (pack == MAX_PACKS) {
        LOG_ERR("Too many different frequencies, only %d allowed", MAX_PACKS);
        return;
    }

    // Add our new topic to list.
    topic_list[topic_count++] = topic;

    // subscribe to topics.
    if (!vehicle->subscribe->initPackageFromTopicList(pack, topic_count, topic_list, false, frequency)) {
        LOG_ERR("Failed to init package %d with freq %d", pack, frequency);
        return;
    }
    if (DJI::OSDK::ACK::getError(vehicle->subscribe->startPackage(pack, 1)) != DJI::OSDK::ACK::SUCCESS) {
        vehicle->subscribe->removePackage(pack, 1);
        LOG_ERR("Failed to start package %d with freq %d", pack, frequency);
        return;
    }
    // Put back subscribed topics into our map.
    for (int i = 0; i < topic_count; i++) {
        subscribed_topics.emplace(topic_list[i], pack);
    }
    frequencies[pack] = frequency;
    LOG_ERR("OK package %d with freq %d", pack, frequency);
}

bool
Dji_vehicle::Send_direct_vehicle_control()
{
    // Use this timer to handle possible serial disconnect from vehicle.
    if (last_telemetry_received + DIRECT_CONTROL_TIMEOUT < std::chrono::steady_clock::now()) {
        // In the very unlikely case of broken connection we just report that vehicle is disconnected.
        // TODO: try to reestablish connection with vehicle.
        int v = 0;
        t_gcs_link_quality->Get_value(v);
        if (v == 1) {
            VEHICLE_LOG_ERR(*this, "=== Connection with vehicle lost ===");
        }
        t_gcs_link_quality->Set_value(0);
        t_uplink_present->Set_value(false);
        t_downlink_present->Set_value(false);
    } else {
        t_gcs_link_quality->Set_value(1);
        t_uplink_present->Set_value(true);
        t_downlink_present->Set_value(true);
    }

    if (rth_on_rc_loss &&
        cur_armed_state == ARMED_STATE_IN_AIR &&
        Is_my_mode(proto::CONTROL_MODE_AUTO) &&
        last_rc_ok_time + RTH_ON_RC_LOSS_TIMEOUT < std::chrono::steady_clock::now())
    {
        LOG_WARN("RC loss detected! Initiate RTH on RC loss");
        Add_status_message("Initiating RTH on RC loss");
        // Make sure we do not call rth multiple times in a row.
        last_rc_ok_time = std::chrono::steady_clock::now();
        Initiate_command_rth();
    }

    if (Is_control_mode(proto::CONTROL_MODE_JOYSTICK)) {
        if (direct_vehicle_controls.last_received + DIRECT_CONTROL_TIMEOUT < std::chrono::steady_clock::now()) {
            // No joystick commands received. Default to 0,0,0,0.
            LOG("No direct_vehicle_control received from ucs...");
            direct_vehicle_controls.pitch = 0;
            direct_vehicle_controls.roll = 0;
            direct_vehicle_controls.throttle = 0;
            direct_vehicle_controls.yaw = 0;
            direct_vehicle_controls.last_received = std::chrono::steady_clock::now();
        }
        DJI::OSDK::Control::CtrlData data(
            (DJI::OSDK::Control::VERTICAL_VELOCITY |
               DJI::OSDK::Control::HORIZONTAL_VELOCITY |
               DJI::OSDK::Control::YAW_RATE |
               DJI::OSDK::Control::HORIZONTAL_BODY),
            direct_vehicle_controls.roll,
            direct_vehicle_controls.pitch,
            direct_vehicle_controls.throttle,
            direct_vehicle_controls.yaw);
        vehicle->control->flightCtrl(data);

//        LOG("rpty %0.3f %0.3f %0.3f %0.3f",
//            direct_vehicle_controls.roll,
//            direct_vehicle_controls.pitch,
//            direct_vehicle_controls.throttle,
//            direct_vehicle_controls.yaw);
    }
    return true;
}

void
Dji_vehicle::Dji_ack_callback(
    DJI::OSDK::Vehicle*,
    DJI::OSDK::RecvContainer container,
    DJI::OSDK::UserData userData)
{
    // This is called from dji sdk thread. Everything must be processed in our vehicle thread context.
    auto data = reinterpret_cast<Data_container*>(userData);
    if (data) {
        auto request = Request::Create();
        // Strictly speaking we should do deep copy of container here.
        // the only pointer I've found in the RecvContainer is recvInfo.buf
        // We just ignore it for now.
        request->Set_processing_handler(
            Make_callback([](Data_container* data, DJI::OSDK::RecvContainer container, Request::Ptr r) {
                    data->vehicle->Process_ack(container, data->next_action, data->error_action);
                    delete data;
                    r->Complete();
                },
                data,
                container,
                request));
        data->vehicle->Get_processing_ctx()->Submit_request(request);
    }
}

void
Dji_vehicle::Dji_telemetry_callback(
    DJI::OSDK::Vehicle*,
    DJI::OSDK::RecvContainer,
    DJI::OSDK::UserData userData)
{
    // This is called from dji sdk thread. Everything must be processed in our vehicle thread context.
    auto data = reinterpret_cast<Telemetry_callback_data*>(userData);
    if (data) {
        auto request = Request::Create();
        request->Set_processing_handler(
            Make_callback([](Telemetry_callback_data* data, Request::Ptr r)
            {
                data->vehicle->Process_telemetry(data->pack_id);
                r->Complete();
            },
            data,
            request));
        data->vehicle->Get_processing_ctx()->Submit_request(request);
    }
}

void
Dji_vehicle::Dji_waypoint_callback(
    DJI::OSDK::Vehicle*,
    DJI::OSDK::RecvContainer c,
    DJI::OSDK::UserData userData)
{
    // This is called from dji sdk thread. Everything must be processed in our vehicle thread context.
    auto data = reinterpret_cast<Dji_vehicle*>(userData);
    if (data) {
        auto request = Request::Create();
        request->Set_processing_handler(
            Make_callback([](Dji_vehicle* v, DJI::OSDK::RecvContainer c, Request::Ptr r)
            {
                v->Process_waypoint(c);
                r->Complete();
            },
            data,
            c,
            request));
        data->Get_processing_ctx()->Submit_request(request);
    }
}

DJI::OSDK::Telemetry::Vector3f
Dji_vehicle::Quaternion_to_eulers(const DJI::OSDK::Telemetry::Quaternion& q)
{
    DJI::OSDK::Telemetry::Vector3f ret;
    double q2sqr = q.q2 * q.q2;
    double t0 = -2.0 * (q2sqr + q.q3 * q.q3) + 1.0;
    double t1 =  2.0 * (q.q1 * q.q2 + q.q0 * q.q3);
    double t2 = -2.0 * (q.q1 * q.q3 - q.q0 * q.q2);
    double t3 =  2.0 * (q.q2 * q.q3 + q.q0 * q.q1);
    double t4 = -2.0 * (q.q1 * q.q1 + q2sqr) + 1.0;

    t2 = (t2 > 1.0) ? 1.0 : t2;
    t2 = (t2 < -1.0) ? -1.0 : t2;

    ret.x = asin(t2);
    ret.y = atan2(t3, t4);
    ret.z = atan2(t1, t0);

    return ret;
}

void *
Dji_vehicle::Create_ack_data(
    Generic_callback next_action,
    Generic_callback_with_error error_action)
{
    auto user_data = new Data_container();
    user_data->vehicle = this;
    user_data->next_action = next_action;
    user_data->error_action = error_action;
    return user_data;
}

void *
Dji_vehicle::Create_ack_data(proto::Control_mode mode)
{
    auto user_data = new Data_container();
    user_data->vehicle = this;
    user_data->next_action = Make_callback(
        [this](proto::Control_mode mode){
            my_control_mode = mode;
            Succeed_current_request();
        },
        mode);
    return user_data;
}

void *
Dji_vehicle::Create_ack_data_no_mode()
{
    auto user_data = new Data_container();
    user_data->vehicle = this;
    user_data->next_action = Make_callback(
        [this](){
            my_control_mode.Disengage();
            Succeed_current_request();
        });
    return user_data;
}

uint32_t
Dji_vehicle::Get_hash(const DJI::OSDK::WayPointSettings& wp)
{
    Crc32 h;
    h.Add_int(wp.index);
    h.Add_int(static_cast<int>(wp.latitude * 1000000));
    h.Add_int(static_cast<int>(wp.longitude * 1000000));
    h.Add_int(static_cast<int>(wp.altitude * 1000));        // mm
    return h.Get();
}

Dji_vehicle::Waypoint_data&
Dji_vehicle::Last_wp_data()
{
    return waypoint_data[current_wp_number];
}

DJI::OSDK::WayPointSettings&
Dji_vehicle::Last_wp()
{
    return waypoints.back();
}

bool
Dji_vehicle::Is_mission_present()
{
    return !waypoints.empty();
}
