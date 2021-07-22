#ifndef DJI_VEHICLE_H_
#define DJI_VEHICLE_H_

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wold-style-cast"
#pragma GCC diagnostic ignored "-Wignored-qualifiers"
#pragma GCC diagnostic ignored "-Wreturn-type"
#include <dji_vehicle.hpp>
#pragma GCC diagnostic pop
#include <ugcs/vsm/vsm.h>

#include <dji_defs.h>

static constexpr int MAX_CONCURRENT_REQUESTS = 5;

// We save current position on mission pause, click&go and joystick.
// On resume we return to that position before continuing mission.
// Vehicle will go directly to next WP if currently it is close enough to saved position.
constexpr float MIN_RESUME_DISTANCE = 20;

class Dji_vehicle: public ugcs::vsm::Vehicle {
    DEFINE_COMMON_CLASS(Dji_vehicle, ugcs::vsm::Vehicle)
public:
    Dji_vehicle();

    virtual ~Dji_vehicle();

    virtual void
    On_enable() override;

    virtual void
    On_disable() override;

    void
    Wait_done();

    virtual void
    Handle_ucs_info(std::vector<ugcs::vsm::Ucs_info> ucs_data) override;

    std::shared_ptr<DJI::OSDK::Vehicle> vehicle;

private:
    typedef struct {
        int pack_id;
        Dji_vehicle* vehicle;
    } Telemetry_callback_data;

    typedef ugcs::vsm::Callback_proxy<void> Generic_callback;

    // Used to specify callback on error.
    typedef ugcs::vsm::Callback_proxy<bool, DJI::OSDK::ACK::ErrorCode> Generic_callback_with_error;

    typedef struct {
        Dji_vehicle* vehicle;
        // Called if API request was success.
        // If defined next_action must complete current_ucs_request.
        // If not defined Process_ack will succeed the current_ucs_request
        Generic_callback next_action;
        // Called if API request returned failure.
        // If not defined Process_ack will fail the current_ucs_request
        // on true Proces_ack will call next_action.
        // on false error_action must complete the current_ucs_request.
        Generic_callback_with_error error_action;
    } Data_container;

    typedef struct {
        ugcs::vsm::Optional<float> speed;           // speed after this WP.
        ugcs::vsm::Optional<bool> trigger_by_time;  // true: trigger_by_time, false: trigger_by distance.
        int count = 0;                              // number of shots
        std::chrono::milliseconds delay;            // delay ms before starting triggering.
        std::chrono::milliseconds period;           // period between shots.
        float distance;                             // distance in m
    } Waypoint_data;

    typedef struct {
        int count;                                  // number of shots left to do.
        float distance;                             // distance in m
        ugcs::vsm::Wgs84_position prev_location;    // Prev trigger location location to calculate distance from.
    } Trigger_by_distance_state;

    typedef struct {
        float pitch;
        float roll;
        float yaw;
        float throttle;
        std::chrono::time_point<std::chrono::steady_clock> last_received;
    } Direct_vehicle_controls;

    typedef struct {
        float tilt;
        float roll;
        float yaw;
        float zoom_level;
        std::chrono::time_point<std::chrono::steady_clock> last_received;
    } Direct_payload_controls;
    // next_action callback will get called only if API call succeeds.
    // If next_action is given it MUST complete the current_ucs_request.
    // Sometimes we want to continue even if sdk returns error.
    // Use error_action for that.
    void *
    Create_ack_data(
        Generic_callback next_action = Generic_callback(),
        Generic_callback_with_error error_action = Generic_callback_with_error());

    // Automatically sets my_control_mode to mode on success.
    void *
    Create_ack_data(ugcs::vsm::proto::Control_mode mode);

    // Disengages my_control_mode on success.
    void *
    Create_ack_data_no_mode();

    static constexpr std::chrono::milliseconds
        /** joystick commands are sent at this rate. */
        DIRECT_CONTROL_INTERVAL = std::chrono::milliseconds(100),
        /** If there are no joystick commands for this long, keep sending (0,0,0,0) */
        DIRECT_CONTROL_TIMEOUT = std::chrono::seconds(5),
        /** How much reported vehicle clock can differ to drop altitude origin */
        ALTITUDE_ORIGIN_RESET_TRESHOLD = std::chrono::seconds(15),
        /** If rc is lost for this long, initiate RTH is configured for mission. */
        RTH_ON_RC_LOSS_TIMEOUT = std::chrono::seconds(5);

    // If no CHANGE_SPEED message given then vehicle will default to this.
    static constexpr float DEFAULT_SPEED = 6.13; // m/s

    // Enforce home altitude to at least 15m AHL.
    // Prevent user setting home altitude above safe altitude.
    static constexpr float MIN_RTH_ALTITUDE = 15.0;

    // Artificial control mode to support modes not reported by vehicle.
    ugcs::vsm::Optional<ugcs::vsm::proto::Control_mode> my_control_mode;
    bool is_armed = false;

    std::string port_name;

    int port_baud = 0;

    // For graceful shutdown.
    mutable std::mutex exit_mutex;
    std::condition_variable exit_cond_var;
    bool exit_required = false;

    // Telemetry subscription stuff.
    // subscribed_topics contain all topics currently subscribed. Value is index in frequencies array.
    // frequencies contains frequency for each topic.
    // Only MAX_PACKS sets of topics are supported.

    // topic -> pack map (index in frequencies array).
    std::unordered_map<DJI::OSDK::Telemetry::TopicName, int, std::hash<int>> subscribed_topics;
    // array of frequencies (for packages).
    std::array<int, MAX_PACKS> frequencies;
    // To bring pack id over to callback.
    Telemetry_callback_data callback_data [MAX_PACKS];

    bool
    Set_control_data_timer();

    bool
    Set_fchannel_timer();

    virtual void
    Handle_ucs_command(ugcs::vsm::Ucs_request::Ptr request) override;

    void
    Mission_upload(const ugcs::vsm::Property_list& params);

    void
    Obtain_vehicle_control(Generic_callback control_obtained_cb = Generic_callback());

    void
    Release_vehicle_control();

    // Stops mission flight.
    void
    Stop_mission(bool save_current_position);

    // Send the mission to the vehicle.
    // If there is a mission_paused_at defined it will upload mission with mission_paused_at
    // as first WP.
    void
    Send_mission(Generic_callback send_complete_cb);

    // Send one WP from waypoints.
    // index  - Which WP to send from waypoints array.
    // If mission_paused_at defined then use it as first WP and.
    void
    Send_wp(unsigned index);

    // Send start mission command to vehicle.
    void
    Send_start();

    bool
    On_timer();

    void
    Dump_mission(float elevation_offset, float max_speed);

    void
    Reupload_mission_and_start();

    // Creates and starts 2 wp mission for click&go
    void
    Execute_click_go(const ugcs::vsm::Property_list& params);

    // Returns current position of vehicle in pos from telemetry.
    // Altitude AHL.
    ugcs::vsm::Wgs84_position
    Get_current_position();

    // This must be called as last in all handlers because it can involve processing of next request.
    void
    Fail_current_request(const std::string& reason);

    void
    Succeed_current_request();

    void
    Handle_next_request();

    // true if mission exists on vehicle and in VSM.
    bool
    Is_mission_present();

    // Signal that vehicle wants to terminate.
    void
    Signal_done();

    // frequency must be less or equal than specified max frequency for each topic.
    // SDK does not say anything about that but it supports only freq values where (400 mod freq) == 0.
    void
    Subscribe_telemetry_topic(int frequency, DJI::OSDK::Telemetry::TopicName topic);

    void
    Process_telemetry(int package);

    // Completes the current_ucs_request.
    // Fails current_ucs_request if operation fails.
    // on success calls next_action handler or succeeds current_ucs_request.
    // I.e. next_action handler must complete the current_ucs_request.
    void
    Process_ack(
        DJI::OSDK::RecvContainer rcvContainer,
        Generic_callback next_action,
        Generic_callback_with_error error_action);

    // Handles set_speed and current WP during mission flight.
    void
    Process_waypoint(DJI::OSDK::RecvContainer rcvContainer);

    // Temporary callback with callback function specified in userData
    static void
    Dji_ack_callback(
        DJI::OSDK::Vehicle* vehiclePtr,
        DJI::OSDK::RecvContainer rcvContainer,
        DJI::OSDK::UserData userData);

    // Calls Process_telemetry with telemetry pack id passed via userData
    static void
    Dji_telemetry_callback(
        DJI::OSDK::Vehicle* vehiclePtr,
        DJI::OSDK::RecvContainer rcvContainer,
        DJI::OSDK::UserData userData);

    // Permanent callback which calls Process_dji_response
    static void
    Dji_waypoint_callback(
        DJI::OSDK::Vehicle* vehiclePtr,
        DJI::OSDK::RecvContainer rcvContainer,
        DJI::OSDK::UserData userData);

    static DJI::OSDK::Telemetry::Vector3f
    Quaternion_to_eulers(const DJI::OSDK::Telemetry::Quaternion& q);

    Display_mode cur_display_mode = DISPLAY_MODE_UNKNOWN;
    Control_source cur_control_source = CONTORL_SOURCE_UNKOWN;
    Armed_state cur_armed_state = ARMED_STATE_UNKNOWN;

    void
    Update_modes(Armed_state, Display_mode, Control_source);

    ugcs::vsm::Ucs_request::Ptr current_ucs_request = nullptr;

    std::queue<ugcs::vsm::Ucs_request::Ptr> pending_ucs_requests;

    // Mission upload stuff
    float current_heading = 0;
    // distance between previous 2 Waypoints. Used to calculate damping for bank turns.
    float prev_segment_length = 0;
    // True if there is at least one bank turn present in the route.
    bool bank_turn_present = false;
    int current_wp_number = -1; // current WP index in waypoints during upload.
    float altitude_origin = 0;
    DJI::OSDK::WayPointInitSettings waypoint_init_data;  // Prepared mission settings
    std::vector<DJI::OSDK::WayPointSettings> waypoints; // Prepared mission waypoints
    // To generate mission hash and report current WP.
    ugcs::vsm::Vehicle::Command_map current_command_map;
    // Keep speed changes for each WP as it cannot be done directly in mission.
    // We will use WaypointEventCallback to update speed at runtime.
    std::unordered_map<int, Waypoint_data> waypoint_data;
    // true if set_heading WP action is used.
    bool heading_set_by_mission = false;
    // Parameter to tell to point to next WP if there is no set_heading action.
    bool autoheading = false;
    // nonzero if executing resumed mission flight.
    unsigned uploaded_wp_offset = 0;
    // Called after route upload is complete.
    Generic_callback route_uploaded_callback;
    // returns reference for modification to current entry (current_wp_number) from waypoint_data
    Waypoint_data&
    Last_wp_data();
    // returns reference for modification to current entry (current_wp_number) from waypoints
    DJI::OSDK::WayPointSettings&
    Last_wp();

    // Coordinates where mission was suspended via click&go or joystick.
    // index contains WP id of next WP when paused.
    ugcs::vsm::Optional<DJI::OSDK::WayPointSettings> mission_paused_at;
    // true while reported 0 wpt is the mission_paused_at waypoint.
    bool first_wp_is_dummy = false;
    // true if flying uploaded mission.
    bool Is_my_mode(ugcs::vsm::proto::Control_mode);

    // Handle triggering by time
    // count - how many triggers have left.
    // period - next trigger after this amount of time.
    bool
    Do_trigger_by_time(int count, std::chrono::milliseconds period);
    // Active during triggering by time.
    ugcs::vsm::Timer_processor::Timer::Ptr trigger_by_time_timer;

    bool
    Initialize_trigger_by_distance(int shot_count, float distance);
    // Present if triggering by distance is active currently.
    ugcs::vsm::Optional<Trigger_by_distance_state> trigger_by_distance_state;
    // Do triggering by distance. Called on each new reported position.
    void
    Process_trigger_by_distance();

    // Create new WP with new id, lat, lon, alt.
    void
    Create_wp_from_coords(const ugcs::vsm::Property_list& params);

    // Create new WP with new id, lat, lon, alt from last WP.
    // Altitude of new WP is .51m above previous.
    void
    Duplicate_last_wp();

    // Handles set_speed and current WP during mission flight.
    void
    Add_wp_action(Wp_action a, int16_t data);

    // Sends joystick controls if enabled.
    bool
    Send_direct_vehicle_control();
    Direct_vehicle_controls direct_vehicle_controls;
    ugcs::vsm::Timer_processor::Timer::Ptr direct_vehicle_control_timer;

    Direct_payload_controls direct_payload_controls;

    std::chrono::time_point<std::chrono::steady_clock> last_telemetry_received;
    // Last time we know RC was fine.
    std::chrono::time_point<std::chrono::steady_clock> last_rc_ok_time;
    // Perform RTH on rc loss after RTH_ON_RC_LOSS_TIMEOUT?
    bool rth_on_rc_loss = false;

    // If connection to server is lost during mission flight then
    // If specified, vehicle will enter click&go mode, climb gcs_loss_height meters above current altitude and hover.
    ugcs::vsm::Optional<std::chrono::milliseconds> gcs_loss_rth_timeout;
    ugcs::vsm::Optional<float> gcs_loss_height;
    ugcs::vsm::Timer_processor::Timer::Ptr gcs_loss_rth_timer; // Used when gcs_loss_rth_timeout > 0
    // This is to ensure that vehicle will not climb more than specified even when there are
    // multiple ucs reconnects while doing the climb.
    float gcs_loss_max_altitude = 120;

    // Initiate climb for height meters from current position.
    // @param alt altitude above HL to climb.
    void
    Initiate_command_climb(float alt);

    // Initiate RTH command
    // returns false because it is used as timer callback which requires return.
    bool
    Initiate_command_rth();

    static uint32_t
    Get_hash(const DJI::OSDK::WayPointSettings&);
};

ugcs::vsm::Io_buffer::Ptr
From_hex(std::string);
void Dump_data(const uint8_t* data, size_t len);

#endif /* DJI_VEHICLE_H_ */
