#include "mavlink_ros2_bridge/mavlink_bridge_node.hpp"

#include <chrono>
#include <cmath>
#include <cstring>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <filesystem>
#include <sys/time.h>

using namespace std::chrono_literals;

// ---------------------------------------------------------------------------
// QGC parameter table
// ---------------------------------------------------------------------------
const std::vector<MAVLinkBridgeNode::ParamEntry>& MAVLinkBridgeNode::paramEntries()
{
    static const std::vector<ParamEntry> entries = {
        {"Kp",              0.0f},
        {"Kcte",            0.0f},
        {"Ki",              0.0f},
        {"Kd",              0.0f},
        {"look_ahead",      0.0f},
        {"pivot_threshold",  40.0f},
        {"cte_i_limit",     1.5f},
        {"cte_i_active",    0.8f},
        {"cte_i_decay",     0.98f},
        {"wp_arrival_dist", 0.1f},
        {"wp_skip_dist",    0.8f},
        {"throttle_range",  250.0f},
        {"pivot_range",     250.0f},
        {"driver_mix",      0.0f},
        {"pwm_center",      1500.0f},
        {"pwm_min",         1000.0f},
        {"pwm_max",         2000.0f},
        {"steering_reverse", 0.0f},
        {"throttle_reverse", 0.0f},
        {"K_rudder",        0.0f},
        {"rudder_center",   1500.0f},
        {"rudder_min",      1000.0f},
        {"rudder_max",      2000.0f},
        {"rudder_reverse",  0.0f},
        {"cmd_vel_speed",   0.5f},
        {"cmd_vel_steer_scale", 0.002f},
        {"cmd_vel_pivot_rate", 0.5f},
        {"gnss_ofs_fwd",    0.0f},
        {"gnss_ofs_left",   0.0f},
        {"gnss_ofs_up",     0.0f},
        {"gnss_ofs_yaw",    0.0f},
    };
    return entries;
}

// ---------------------------------------------------------------------------
// Constructor
// ---------------------------------------------------------------------------
MAVLinkBridgeNode::MAVLinkBridgeNode()
    : Node("mavlink_bridge_node")
{
    // MAVLink system identity
    mavlink_system_.sysid  = 1;
    mavlink_system_.compid = MAV_COMP_ID_AUTOPILOT1;

    // ROS2 parameters
    declare_parameter<std::string>("gcs_ip", "127.0.0.1");
    gcs_ip_ = get_parameter("gcs_ip").as_string();

    declare_parameter<int>("mission_total_seq", 0);
    mission_total_seq_ = get_parameter("mission_total_seq").as_int();

    // Parameter persistence file path
    declare_parameter<std::string>("param_file_path", "~/.ros/mavlink_bridge_params.yaml");
    param_file_path_ = get_parameter("param_file_path").as_string();
    if (param_file_path_.rfind("~/", 0) == 0) {
        const char* home = std::getenv("HOME");
        if (home) {
            param_file_path_ = std::string(home) + param_file_path_.substr(1);
        }
    }

    // Load saved parameter values (if any)
    auto saved = loadParameters();

    // Declare QGC-exposed parameters (use saved values as defaults when available)
    for (const auto& entry : paramEntries()) {
        double default_val = static_cast<double>(entry.default_value);
        auto it = saved.find(entry.name);
        if (it != saved.end()) {
            default_val = it->second;
        }
        declare_parameter<double>(entry.name, default_val);
    }

    // Open UDP socket: receive on 14551, send to GCS on 14550
    if (!socket_.open(LOCAL_PORT, gcs_ip_, REMOTE_PORT)) {
        RCLCPP_FATAL(get_logger(), "Failed to open UDP socket on port %d", LOCAL_PORT);
        throw std::runtime_error("UDP socket open failed");
    }
    RCLCPP_INFO(get_logger(), "MAVLink bridge started — GCS: %s:%d", gcs_ip_.c_str(), REMOTE_PORT);

    // --- Subscribers ---
    // /gnss: bme_common_msgs/GnssSolution
    gnss_sub_ = create_subscription<bme_common_msgs::msg::GnssSolution>(
        "/gnss", 10,
        std::bind(&MAVLinkBridgeNode::onGnssReceived, this, std::placeholders::_1));

    // /auto_log: bme_common_msgs/AutoLog
    auto_log_sub_ = create_subscription<bme_common_msgs::msg::AutoLog>(
        "/auto_log", 1,
        std::bind(&MAVLinkBridgeNode::onAutoLogReceived, this, std::placeholders::_1));


    // --- Publishers ---
    mission_pub_             = create_publisher<std_msgs::msg::Float64MultiArray>("/mav/mission", 1000);
    modes_pub_               = create_publisher<bme_common_msgs::msg::MavModes>("/mav/modes", 1);
    joystick_pub_            = create_publisher<geometry_msgs::msg::TwistStamped>("/mav/joystick", 1);
    mission_set_current_pub_ = create_publisher<std_msgs::msg::UInt16>("/mav/mission_set_current", 10);

    // --- Timers ---
    heartbeat_timer_ = create_wall_timer(900ms,
        std::bind(&MAVLinkBridgeNode::onHeartbeatTimer, this));

    receive_timer_ = create_wall_timer(10ms,
        std::bind(&MAVLinkBridgeNode::onReceiveTimer, this));

    mission_request_timer_ = create_wall_timer(30ms,
        std::bind(&MAVLinkBridgeNode::onMissionRequestTimer, this));
    mission_request_timer_->cancel();  // inactive until mission download begins

    param_send_timer_ = create_wall_timer(50ms,
        std::bind(&MAVLinkBridgeNode::onParamSendTimer, this));
    param_send_timer_->cancel();  // inactive until PARAM_REQUEST_LIST
}

// ---------------------------------------------------------------------------
// Subscriber callbacks
// ---------------------------------------------------------------------------
void MAVLinkBridgeNode::onGnssReceived(const bme_common_msgs::msg::GnssSolution::SharedPtr msg)
{
    gps_lat_        = static_cast<int32_t>(msg->latitude * 1e7);
    gps_lon_        = static_cast<int32_t>(msg->longitude * 1e7);
    gps_alt_        = static_cast<int32_t>(msg->height * 1000);  // m → mm
    gps_satellites_ = msg->num_sv;

    switch (msg->position_rtk_status) {
        case 2:  gps_fix_type_ = GPS_FIX_TYPE_RTK_FIXED; break;
        case 1:  gps_fix_type_ = GPS_FIX_TYPE_RTK_FLOAT; break;
        default: gps_fix_type_ = GPS_FIX_TYPE_DGPS;      break;
    }

    // Heading: ENU (ROS2 REP 103) -> NED (MAVLink)
    const double yaw = M_PI / 2.0 - msg->heading_deg * M_PI / 180.0;
    yaw_ = std::atan2(std::sin(yaw), std::cos(yaw));
    estimator_flags_ = (msg->heading_rtk_status == 2) ? 1 : 0;
}

void MAVLinkBridgeNode::onAutoLogReceived(const bme_common_msgs::msg::AutoLog::SharedPtr msg)
{
    current_seq_       = static_cast<int>(msg->waypoint_seq);
    cross_track_error_ = static_cast<float>(msg->cross_track_error);
    angular_z_         = static_cast<float>(msg->angular_z);
}


// ---------------------------------------------------------------------------
// Heartbeat timer — periodic telemetry to GCS (~900 ms)
// ---------------------------------------------------------------------------
void MAVLinkBridgeNode::onHeartbeatTimer()
{
    mavlink_message_t msg;
    const uint8_t sys  = mavlink_system_.sysid;
    const uint8_t comp = mavlink_system_.compid;
    const uint64_t now = microsSinceEpoch();

    // Heartbeat
    mavlink_msg_heartbeat_pack(sys, comp, &msg,
        MAV_TYPE_GROUND_ROVER, MAV_AUTOPILOT_ARDUPILOTMEGA,
        static_cast<uint8_t>(base_mode_), static_cast<uint32_t>(custom_mode_),
        MAV_STATE_ACTIVE);
    sendMavlinkMessage(msg);

    // System status
    mavlink_msg_sys_status_pack(sys, comp, &msg,
        0, 0, 0, 500, 11000, -1, -1, 0, 0, 0, 0, 0, 0,
        0, 0, 0);  // v2 extended sensor fields
    sendMavlinkMessage(msg);

    // Local position NED
    mavlink_msg_local_position_ned_pack(sys, comp, &msg,
        static_cast<uint32_t>(now), 0, 0, 0, 0, 0, 0);
    sendMavlinkMessage(msg);

    // Attitude
    mavlink_msg_attitude_pack(sys, comp, &msg,
        static_cast<uint32_t>(now), 0, 0, static_cast<float>(yaw_),
        0.01f, 0.02f, 0.03f);
    sendMavlinkMessage(msg);

    // GPS raw
    mavlink_msg_gps_raw_int_pack(sys, comp, &msg,
        0, gps_fix_type_, gps_lat_, gps_lon_, gps_alt_,
        65535, 65535, 65535, 65535, gps_satellites_,
        0, 0, 0, 0, 0, 0);  // v2 extension fields
    sendMavlinkMessage(msg);

    // Estimator status
    mavlink_msg_estimator_status_pack(sys, comp, &msg,
        now, estimator_flags_, angular_z_, 0, 0, 0, 0, 0, cross_track_error_, 0);
    sendMavlinkMessage(msg);

    // Publish modes to ROS
    {
        bme_common_msgs::msg::MavModes modes_msg;
        modes_msg.header.stamp = this->now();
        modes_msg.base_mode = static_cast<int16_t>(base_mode_);
        modes_msg.custom_mode = static_cast<int16_t>(custom_mode_);
        modes_msg.mission_start = mission_start_;
        modes_pub_->publish(modes_msg);
    }

    // Mission current (when armed)
    if (base_mode_ == ARDUPILOT_GUIDED_ARMED) {
        RCLCPP_INFO(get_logger(), "HEARTBEAT: armed, cur_seq=%d total_seq=%d",
                     current_seq_, mission_total_seq_);
        if (current_seq_ > mission_total_seq_) {
            RCLCPP_WARN(get_logger(), "AUTO-DISARM: cur_seq(%d) > total_seq(%d)",
                         current_seq_, mission_total_seq_);
            base_mode_ = ARDUPILOT_GUIDED_DISARMED;
            mission_start_ = false;
        }
        mavlink_msg_mission_current_pack(sys, comp, &msg,
            static_cast<uint16_t>(current_seq_),
            0, 0, 0, 0, 0, 0);  // v2 extension fields
        sendMavlinkMessage(msg);
    }
}

// ---------------------------------------------------------------------------
// Receive timer — poll UDP for incoming MAVLink messages (~10 ms)
// ---------------------------------------------------------------------------
void MAVLinkBridgeNode::onReceiveTimer()
{
    // Handle pending parameter set response
    if (param_set_pending_) {
        RCLCPP_INFO(get_logger(), "Sending param_value: %s = %f (type %d)",
                     pending_param_id_, pending_param_value_, pending_param_type_);
        sendParamValue(pending_param_id_, pending_param_value_, pending_param_type_, 1, 0);

        // Update the ROS2 parameter
        try {
            set_parameter(rclcpp::Parameter(
                std::string(pending_param_id_),
                static_cast<double>(pending_param_value_)));
            saveParameters();
        } catch (const rclcpp::exceptions::ParameterNotDeclaredException&) {
            RCLCPP_WARN(get_logger(), "Parameter '%s' not declared, skipping update",
                         pending_param_id_);
        }

        param_set_pending_ = false;
    }

    // Drain all pending UDP packets
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    ssize_t recsize;
    while ((recsize = socket_.receive(buf, sizeof(buf))) > 0) {
        // Log remote endpoint changes (NAT diagnostics)
        std::string ep = socket_.remote_endpoint();
        if (ep != last_remote_endpoint_) {
            RCLCPP_INFO(get_logger(), "Remote endpoint updated: %s → %s",
                         last_remote_endpoint_.c_str(), ep.c_str());
            last_remote_endpoint_ = ep;
        }

        mavlink_message_t msg;
        mavlink_status_t status;
        for (ssize_t i = 0; i < recsize; ++i) {
            if (mavlink_parse_char(MAVLINK_COMM_0, buf[i], &msg, &status)) {
                RCLCPP_DEBUG(get_logger(),
                    "Received MAVLink — SYS: %d, COMP: %d, LEN: %d, MSG ID: %d",
                    msg.sysid, msg.compid, msg.len, msg.msgid);
                handleMavlinkMessage(msg);
            }
        }
    }
}

// ---------------------------------------------------------------------------
// Mission request timer — send MISSION_REQUEST_INT (~30 ms while downloading)
// ---------------------------------------------------------------------------
void MAVLinkBridgeNode::onMissionRequestTimer()
{
    if (!is_mission_request_) {
        mission_request_timer_->cancel();
        return;
    }

    mavlink_message_t msg;
    mavlink_msg_mission_request_int_pack(
        mavlink_system_.sysid, mavlink_system_.compid, &msg,
        0, 0, mission_seq_, MAV_MISSION_TYPE_MISSION);
    sendMavlinkMessage(msg);
}

// ---------------------------------------------------------------------------
// MAVLink message dispatcher
// ---------------------------------------------------------------------------
void MAVLinkBridgeNode::handleMavlinkMessage(const mavlink_message_t& msg)
{
    switch (msg.msgid) {
        case MAVLINK_MSG_ID_HEARTBEAT:
            handleHeartbeat();
            break;
        case MAVLINK_MSG_ID_SET_MODE:
            handleSetMode(msg);
            break;
        case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
            handleParamRequestList();
            break;
        case MAVLINK_MSG_ID_PARAM_SET:
            handleParamSet(msg);
            break;
        case MAVLINK_MSG_ID_MISSION_REQUEST_LIST:
            handleMissionRequestList(msg);
            break;
        case MAVLINK_MSG_ID_MISSION_SET_CURRENT:
            handleMissionSetCurrent(msg);
            break;
        case MAVLINK_MSG_ID_MISSION_COUNT:
            handleMissionCount(msg);
            break;
        case MAVLINK_MSG_ID_MANUAL_CONTROL:
            handleManualControl(msg);
            break;
        case MAVLINK_MSG_ID_MISSION_ITEM_INT:
            handleMissionItemInt(msg);
            break;
        case MAVLINK_MSG_ID_COMMAND_LONG:
            handleCommandLong(msg);
            break;
        default:
            break;
    }
}

// ---------------------------------------------------------------------------
// Individual message handlers
// ---------------------------------------------------------------------------
void MAVLinkBridgeNode::handleHeartbeat()
{
    // GCS heartbeat received — could track connectivity timeout here
}

void MAVLinkBridgeNode::handleSetMode(const mavlink_message_t& msg)
{
    mavlink_set_mode_t decoded;
    mavlink_msg_set_mode_decode(&msg, &decoded);

    custom_mode_ = decoded.custom_mode;

    // Preserve armed/disarmed state — only ARM/DISARM command should change it
    bool currently_armed = (base_mode_ == ARDUPILOT_GUIDED_ARMED);
    if (currently_armed) {
        base_mode_ = ARDUPILOT_GUIDED_ARMED;
    } else {
        base_mode_ = decoded.base_mode;
    }

    RCLCPP_INFO(get_logger(), "SET_MODE: base=%lu custom=%lu (armed=%d)",
                 base_mode_, custom_mode_, currently_armed);
}

void MAVLinkBridgeNode::handleParamRequestList()
{
    RCLCPP_INFO(get_logger(), "PARAM_REQUEST_LIST received — queueing %zu parameters",
                 paramEntries().size());

    // Build send queue (indices 0..N-1) and start paced sending
    param_send_queue_.clear();
    const uint16_t count = static_cast<uint16_t>(paramEntries().size());
    for (uint16_t i = 0; i < count; ++i) {
        param_send_queue_.push_back(i);
    }
    param_send_timer_->reset();
}

void MAVLinkBridgeNode::onParamSendTimer()
{
    if (param_send_queue_.empty()) {
        param_send_timer_->cancel();
        return;
    }

    const auto& entries = paramEntries();
    const uint16_t count = static_cast<uint16_t>(entries.size());
    uint16_t idx = param_send_queue_.front();
    param_send_queue_.erase(param_send_queue_.begin());

    float value = static_cast<float>(
        get_parameter(entries[idx].name).as_double());
    sendParamValue(entries[idx].name, value, MAVLINK_TYPE_FLOAT, count, idx);
}

void MAVLinkBridgeNode::handleParamSet(const mavlink_message_t& msg)
{
    mavlink_param_set_t decoded;
    mavlink_msg_param_set_decode(&msg, &decoded);

    RCLCPP_INFO(get_logger(), "PARAM_SET received");

    // Safely copy param_id (may not be null-terminated in MAVLink)
    std::memset(pending_param_id_, 0, sizeof(pending_param_id_));
    std::memcpy(pending_param_id_, decoded.param_id, 16);

    pending_param_value_ = decoded.param_value;
    pending_param_type_  = decoded.param_type;
    param_set_pending_   = true;
}

void MAVLinkBridgeNode::handleMissionRequestList(const mavlink_message_t& msg)
{
    mavlink_mission_request_list_t decoded;
    mavlink_msg_mission_request_list_decode(&msg, &decoded);

    RCLCPP_INFO(get_logger(), "MISSION_REQUEST_LIST: type=%d — responding with count=0",
                 decoded.mission_type);

    // Respond with MISSION_COUNT = 0 (no stored missions/fence/rally)
    mavlink_message_t reply;
    mavlink_msg_mission_count_pack(
        mavlink_system_.sysid, mavlink_system_.compid, &reply,
        msg.sysid, msg.compid,
        0, decoded.mission_type, 0);
    sendMavlinkMessage(reply);
}

void MAVLinkBridgeNode::handleMissionSetCurrent(const mavlink_message_t& msg)
{
    mavlink_mission_set_current_t decoded;
    mavlink_msg_mission_set_current_decode(&msg, &decoded);

    std_msgs::msg::UInt16 ros_msg;
    ros_msg.data = decoded.seq;
    mission_set_current_pub_->publish(ros_msg);

    RCLCPP_INFO(get_logger(), "MISSION_SET_CURRENT: seq=%d", decoded.seq);
}

void MAVLinkBridgeNode::handleMissionCount(const mavlink_message_t& msg)
{
    mavlink_mission_count_t decoded;
    mavlink_msg_mission_count_decode(&msg, &decoded);

    // Fence/rally mission types — ACK immediately but don't process
    if (decoded.mission_type != MAV_MISSION_TYPE_MISSION) {
        RCLCPP_INFO(get_logger(), "MISSION_COUNT: %d items (type=%d, ACK only)",
                     decoded.count, decoded.mission_type);
        mavlink_message_t ack;
        mavlink_msg_mission_ack_pack(
            mavlink_system_.sysid, mavlink_system_.compid, &ack,
            msg.sysid, msg.compid,
            MAV_MISSION_ACCEPTED, decoded.mission_type, 0);
        sendMavlinkMessage(ack);
        return;
    }

    is_mission_request_ = true;
    mission_total_seq_  = decoded.count;
    mission_seq_        = 0;
    prev_mission_seq_   = -1;
    current_seq_        = 0;

    RCLCPP_INFO(get_logger(), "MISSION_COUNT: %d items", mission_total_seq_);

    // Start periodic mission request
    mission_request_timer_->reset();
}

void MAVLinkBridgeNode::handleManualControl(const mavlink_message_t& msg)
{
    mavlink_manual_control_t decoded;
    mavlink_msg_manual_control_decode(&msg, &decoded);

    RCLCPP_DEBUG(get_logger(),
        "MANUAL_CONTROL: target=%d pitch=%d roll=%d throttle=%d yaw=%d buttons=%d",
        decoded.target, decoded.x, decoded.y, decoded.z, decoded.r, decoded.buttons);

    geometry_msgs::msg::TwistStamped twist_msg;
    twist_msg.header.stamp = now();
    twist_msg.twist.linear.x  = decoded.x;   // pitch
    twist_msg.twist.linear.y  = decoded.y;   // roll
    twist_msg.twist.linear.z  = decoded.z;   // throttle
    twist_msg.twist.angular.z = decoded.r;   // yaw
    joystick_pub_->publish(twist_msg);
}

void MAVLinkBridgeNode::handleMissionItemInt(const mavlink_message_t& msg)
{
    mavlink_mission_item_int_t decoded;
    mavlink_msg_mission_item_int_decode(&msg, &decoded);

    double waypoint_lat = decoded.x / 1e7;
    double waypoint_lon = decoded.y / 1e7;

    // Accept only the expected sequence number
    if (prev_mission_seq_ != decoded.seq &&
        mission_seq_ == static_cast<uint16_t>(decoded.seq))
    {
        RCLCPP_INFO(get_logger(), "MISSION_ITEM_INT: seq=%d/%d cmd=%d lat=%.9f lon=%.9f",
                     decoded.seq, mission_total_seq_, decoded.command,
                     waypoint_lat, waypoint_lon);

        // Publish mission item to ROS
        std_msgs::msg::Float64MultiArray mission_msg;
        mission_msg.data = {
            static_cast<double>(decoded.seq),
            static_cast<double>(mission_total_seq_),
            static_cast<double>(decoded.command),
            waypoint_lat,
            waypoint_lon
        };
        mission_pub_->publish(mission_msg);

        prev_mission_seq_ = decoded.seq;
        mission_seq_      = decoded.seq + 1;
    }

    // All mission items received — send ACK and stop requesting
    if (is_mission_request_ &&
        mission_seq_ == static_cast<uint16_t>(mission_total_seq_))
    {
        mavlink_message_t ack;
        mavlink_msg_mission_ack_pack(
            mavlink_system_.sysid, mavlink_system_.compid, &ack,
            0, 0, MAV_MISSION_ACCEPTED, MAV_MISSION_TYPE_MISSION, 0);
        sendMavlinkMessage(ack);

        // Store total seq count as ROS parameter
        set_parameter(rclcpp::Parameter("mission_total_seq", mission_total_seq_));

        is_mission_request_ = false;
        mission_request_timer_->cancel();

        RCLCPP_INFO(get_logger(), "Mission download complete (%d items)", mission_total_seq_);
    }
}

void MAVLinkBridgeNode::handleCommandLong(const mavlink_message_t& msg)
{
    mavlink_command_long_t decoded;
    mavlink_msg_command_long_decode(&msg, &decoded);

    RCLCPP_INFO(get_logger(),
        "COMMAND_LONG: cmd=%d target_sys=%d target_comp=%d confirmation=%d "
        "p1=%f p2=%f p3=%f p4=%f p5=%f p6=%f p7=%f",
        decoded.command, decoded.target_system, decoded.target_component,
        decoded.confirmation,
        decoded.param1, decoded.param2, decoded.param3, decoded.param4,
        decoded.param5, decoded.param6, decoded.param7);

    mavlink_message_t ack;
    const uint8_t sys  = mavlink_system_.sysid;
    const uint8_t comp = mavlink_system_.compid;

    switch (decoded.command) {
        case MAV_CMD_COMPONENT_ARM_DISARM:
            mavlink_msg_command_ack_pack(sys, comp, &ack,
                MAV_CMD_COMPONENT_ARM_DISARM, MAV_RESULT_ACCEPTED,
                0, 0, 0, 0);  // v2: progress, result_param2, target_sys, target_comp
            sendMavlinkMessage(ack);

            if (decoded.param1 == 1.0f) {
                base_mode_ = ARDUPILOT_GUIDED_ARMED;
                RCLCPP_INFO(get_logger(), "ARMED");
            } else {
                base_mode_ = ARDUPILOT_GUIDED_DISARMED;
                mission_start_ = false;
                RCLCPP_INFO(get_logger(), "DISARMED");
            }
            break;

        case MAV_CMD_MISSION_START:
            mavlink_msg_command_ack_pack(sys, comp, &ack,
                decoded.command, MAV_RESULT_ACCEPTED,
                0, 0, 0, 0);
            sendMavlinkMessage(ack);
            mission_start_ = true;
            RCLCPP_INFO(get_logger(), "MISSION_START");
            break;

        case MAV_CMD_DO_SET_MODE: {
            // param1: base_mode (float → uint8_t)
            // param2: custom_mode (float → uint32_t)
            custom_mode_ = static_cast<uint32_t>(decoded.param2);

            // Preserve armed state — only ARM/DISARM command should change it
            if (base_mode_ != ARDUPILOT_GUIDED_ARMED) {
                base_mode_ = static_cast<uint8_t>(decoded.param1);
            }

            mavlink_msg_command_ack_pack(sys, comp, &ack,
                decoded.command, MAV_RESULT_ACCEPTED,
                0, 0, 0, 0);
            sendMavlinkMessage(ack);

            RCLCPP_INFO(get_logger(), "DO_SET_MODE: base_mode=%lu custom_mode=%lu",
                         base_mode_, custom_mode_);
            break;
        }

        case MAV_CMD_REQUEST_PROTOCOL_VERSION:
            mavlink_msg_command_ack_pack(sys, comp, &ack,
                decoded.command, MAV_RESULT_ACCEPTED,
                0, 0, 0, 0);
            sendMavlinkMessage(ack);
            break;

        default:
            break;
    }
}

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------
void MAVLinkBridgeNode::sendMavlinkMessage(mavlink_message_t& msg)
{
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    socket_.send(buf, len);
}

void MAVLinkBridgeNode::sendParamValue(const std::string& param_id, float value,
                                        uint8_t type, uint16_t count, uint16_t index)
{
    mavlink_message_t msg;
    mavlink_msg_param_value_pack(
        mavlink_system_.sysid, mavlink_system_.compid, &msg,
        param_id.c_str(), value, type, count, index);
    sendMavlinkMessage(msg);
}

uint64_t MAVLinkBridgeNode::microsSinceEpoch() const
{
    struct timeval tv;
    gettimeofday(&tv, nullptr);
    return static_cast<uint64_t>(tv.tv_sec) * 1000000ULL + tv.tv_usec;
}

// ---------------------------------------------------------------------------
// Parameter persistence
// ---------------------------------------------------------------------------
std::map<std::string, double> MAVLinkBridgeNode::loadParameters()
{
    std::map<std::string, double> result;
    std::ifstream ifs(param_file_path_);
    if (!ifs.is_open()) {
        return result;
    }

    std::string line;
    while (std::getline(ifs, line)) {
        auto colon = line.find(':');
        if (colon == std::string::npos) {
            continue;
        }
        std::string key = line.substr(0, colon);
        std::string val_str = line.substr(colon + 1);

        // Trim whitespace
        key.erase(0, key.find_first_not_of(" \t"));
        key.erase(key.find_last_not_of(" \t") + 1);
        val_str.erase(0, val_str.find_first_not_of(" \t"));
        val_str.erase(val_str.find_last_not_of(" \t") + 1);

        if (key.empty() || val_str.empty()) {
            continue;
        }

        try {
            result[key] = std::stod(val_str);
        } catch (...) {
            RCLCPP_WARN(get_logger(), "Failed to parse parameter '%s: %s'",
                         key.c_str(), val_str.c_str());
        }
    }

    RCLCPP_INFO(get_logger(), "Loaded %zu parameters from %s",
                 result.size(), param_file_path_.c_str());
    return result;
}

void MAVLinkBridgeNode::saveParameters()
{
    std::filesystem::path filepath(param_file_path_);
    auto parent = filepath.parent_path();
    if (!parent.empty() && !std::filesystem::exists(parent)) {
        std::filesystem::create_directories(parent);
    }

    std::ofstream ofs(param_file_path_);
    if (!ofs.is_open()) {
        RCLCPP_ERROR(get_logger(), "Failed to open %s for writing",
                      param_file_path_.c_str());
        return;
    }

    for (const auto& entry : paramEntries()) {
        double value = get_parameter(entry.name).as_double();
        ofs << entry.name << ": " << value << "\n";
    }

    RCLCPP_INFO(get_logger(), "Saved parameters to %s", param_file_path_.c_str());
}

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------
int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MAVLinkBridgeNode>());
    rclcpp::shutdown();
    return 0;
}
