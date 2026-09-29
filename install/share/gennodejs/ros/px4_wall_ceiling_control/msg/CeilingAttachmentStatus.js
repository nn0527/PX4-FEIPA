// Auto-generated. Do not edit!

// (in-package px4_wall_ceiling_control.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let geometry_msgs = _finder('geometry_msgs');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class CeilingAttachmentStatus {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.state = null;
      this.state_name = null;
      this.enabled = null;
      this.acquire_request = null;
      this.keep_ownership = null;
      this.candidate_valid = null;
      this.range_fresh = null;
      this.odom_fresh = null;
      this.flight_state_fresh = null;
      this.attach_confirmed = null;
      this.distance_stable = null;
      this.fault_detected = null;
      this.detach_failed = null;
      this.integral_reset_request = null;
      this.wheel_stop_request = null;
      this.mechanism_release_requested = null;
      this.mechanism_release_confirmed = null;
      this.ceiling_distance_raw_m = null;
      this.ceiling_distance_filtered_m = null;
      this.target_distance_m = null;
      this.compression_m = null;
      this.target_compression_m = null;
      this.approach_velocity_z_enu_mps = null;
      this.thrust_body_z_normalized = null;
      this.attitude_setpoint_valid = null;
      this.attitude_setpoint = null;
      this.fault_count = null;
      this.reason = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('state')) {
        this.state = initObj.state
      }
      else {
        this.state = 0;
      }
      if (initObj.hasOwnProperty('state_name')) {
        this.state_name = initObj.state_name
      }
      else {
        this.state_name = '';
      }
      if (initObj.hasOwnProperty('enabled')) {
        this.enabled = initObj.enabled
      }
      else {
        this.enabled = false;
      }
      if (initObj.hasOwnProperty('acquire_request')) {
        this.acquire_request = initObj.acquire_request
      }
      else {
        this.acquire_request = false;
      }
      if (initObj.hasOwnProperty('keep_ownership')) {
        this.keep_ownership = initObj.keep_ownership
      }
      else {
        this.keep_ownership = false;
      }
      if (initObj.hasOwnProperty('candidate_valid')) {
        this.candidate_valid = initObj.candidate_valid
      }
      else {
        this.candidate_valid = false;
      }
      if (initObj.hasOwnProperty('range_fresh')) {
        this.range_fresh = initObj.range_fresh
      }
      else {
        this.range_fresh = false;
      }
      if (initObj.hasOwnProperty('odom_fresh')) {
        this.odom_fresh = initObj.odom_fresh
      }
      else {
        this.odom_fresh = false;
      }
      if (initObj.hasOwnProperty('flight_state_fresh')) {
        this.flight_state_fresh = initObj.flight_state_fresh
      }
      else {
        this.flight_state_fresh = false;
      }
      if (initObj.hasOwnProperty('attach_confirmed')) {
        this.attach_confirmed = initObj.attach_confirmed
      }
      else {
        this.attach_confirmed = false;
      }
      if (initObj.hasOwnProperty('distance_stable')) {
        this.distance_stable = initObj.distance_stable
      }
      else {
        this.distance_stable = false;
      }
      if (initObj.hasOwnProperty('fault_detected')) {
        this.fault_detected = initObj.fault_detected
      }
      else {
        this.fault_detected = false;
      }
      if (initObj.hasOwnProperty('detach_failed')) {
        this.detach_failed = initObj.detach_failed
      }
      else {
        this.detach_failed = false;
      }
      if (initObj.hasOwnProperty('integral_reset_request')) {
        this.integral_reset_request = initObj.integral_reset_request
      }
      else {
        this.integral_reset_request = false;
      }
      if (initObj.hasOwnProperty('wheel_stop_request')) {
        this.wheel_stop_request = initObj.wheel_stop_request
      }
      else {
        this.wheel_stop_request = false;
      }
      if (initObj.hasOwnProperty('mechanism_release_requested')) {
        this.mechanism_release_requested = initObj.mechanism_release_requested
      }
      else {
        this.mechanism_release_requested = false;
      }
      if (initObj.hasOwnProperty('mechanism_release_confirmed')) {
        this.mechanism_release_confirmed = initObj.mechanism_release_confirmed
      }
      else {
        this.mechanism_release_confirmed = false;
      }
      if (initObj.hasOwnProperty('ceiling_distance_raw_m')) {
        this.ceiling_distance_raw_m = initObj.ceiling_distance_raw_m
      }
      else {
        this.ceiling_distance_raw_m = 0.0;
      }
      if (initObj.hasOwnProperty('ceiling_distance_filtered_m')) {
        this.ceiling_distance_filtered_m = initObj.ceiling_distance_filtered_m
      }
      else {
        this.ceiling_distance_filtered_m = 0.0;
      }
      if (initObj.hasOwnProperty('target_distance_m')) {
        this.target_distance_m = initObj.target_distance_m
      }
      else {
        this.target_distance_m = 0.0;
      }
      if (initObj.hasOwnProperty('compression_m')) {
        this.compression_m = initObj.compression_m
      }
      else {
        this.compression_m = 0.0;
      }
      if (initObj.hasOwnProperty('target_compression_m')) {
        this.target_compression_m = initObj.target_compression_m
      }
      else {
        this.target_compression_m = 0.0;
      }
      if (initObj.hasOwnProperty('approach_velocity_z_enu_mps')) {
        this.approach_velocity_z_enu_mps = initObj.approach_velocity_z_enu_mps
      }
      else {
        this.approach_velocity_z_enu_mps = 0.0;
      }
      if (initObj.hasOwnProperty('thrust_body_z_normalized')) {
        this.thrust_body_z_normalized = initObj.thrust_body_z_normalized
      }
      else {
        this.thrust_body_z_normalized = 0.0;
      }
      if (initObj.hasOwnProperty('attitude_setpoint_valid')) {
        this.attitude_setpoint_valid = initObj.attitude_setpoint_valid
      }
      else {
        this.attitude_setpoint_valid = false;
      }
      if (initObj.hasOwnProperty('attitude_setpoint')) {
        this.attitude_setpoint = initObj.attitude_setpoint
      }
      else {
        this.attitude_setpoint = new geometry_msgs.msg.Quaternion();
      }
      if (initObj.hasOwnProperty('fault_count')) {
        this.fault_count = initObj.fault_count
      }
      else {
        this.fault_count = 0;
      }
      if (initObj.hasOwnProperty('reason')) {
        this.reason = initObj.reason
      }
      else {
        this.reason = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type CeilingAttachmentStatus
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [state]
    bufferOffset = _serializer.uint8(obj.state, buffer, bufferOffset);
    // Serialize message field [state_name]
    bufferOffset = _serializer.string(obj.state_name, buffer, bufferOffset);
    // Serialize message field [enabled]
    bufferOffset = _serializer.bool(obj.enabled, buffer, bufferOffset);
    // Serialize message field [acquire_request]
    bufferOffset = _serializer.bool(obj.acquire_request, buffer, bufferOffset);
    // Serialize message field [keep_ownership]
    bufferOffset = _serializer.bool(obj.keep_ownership, buffer, bufferOffset);
    // Serialize message field [candidate_valid]
    bufferOffset = _serializer.bool(obj.candidate_valid, buffer, bufferOffset);
    // Serialize message field [range_fresh]
    bufferOffset = _serializer.bool(obj.range_fresh, buffer, bufferOffset);
    // Serialize message field [odom_fresh]
    bufferOffset = _serializer.bool(obj.odom_fresh, buffer, bufferOffset);
    // Serialize message field [flight_state_fresh]
    bufferOffset = _serializer.bool(obj.flight_state_fresh, buffer, bufferOffset);
    // Serialize message field [attach_confirmed]
    bufferOffset = _serializer.bool(obj.attach_confirmed, buffer, bufferOffset);
    // Serialize message field [distance_stable]
    bufferOffset = _serializer.bool(obj.distance_stable, buffer, bufferOffset);
    // Serialize message field [fault_detected]
    bufferOffset = _serializer.bool(obj.fault_detected, buffer, bufferOffset);
    // Serialize message field [detach_failed]
    bufferOffset = _serializer.bool(obj.detach_failed, buffer, bufferOffset);
    // Serialize message field [integral_reset_request]
    bufferOffset = _serializer.bool(obj.integral_reset_request, buffer, bufferOffset);
    // Serialize message field [wheel_stop_request]
    bufferOffset = _serializer.bool(obj.wheel_stop_request, buffer, bufferOffset);
    // Serialize message field [mechanism_release_requested]
    bufferOffset = _serializer.bool(obj.mechanism_release_requested, buffer, bufferOffset);
    // Serialize message field [mechanism_release_confirmed]
    bufferOffset = _serializer.bool(obj.mechanism_release_confirmed, buffer, bufferOffset);
    // Serialize message field [ceiling_distance_raw_m]
    bufferOffset = _serializer.float32(obj.ceiling_distance_raw_m, buffer, bufferOffset);
    // Serialize message field [ceiling_distance_filtered_m]
    bufferOffset = _serializer.float32(obj.ceiling_distance_filtered_m, buffer, bufferOffset);
    // Serialize message field [target_distance_m]
    bufferOffset = _serializer.float32(obj.target_distance_m, buffer, bufferOffset);
    // Serialize message field [compression_m]
    bufferOffset = _serializer.float32(obj.compression_m, buffer, bufferOffset);
    // Serialize message field [target_compression_m]
    bufferOffset = _serializer.float32(obj.target_compression_m, buffer, bufferOffset);
    // Serialize message field [approach_velocity_z_enu_mps]
    bufferOffset = _serializer.float32(obj.approach_velocity_z_enu_mps, buffer, bufferOffset);
    // Serialize message field [thrust_body_z_normalized]
    bufferOffset = _serializer.float32(obj.thrust_body_z_normalized, buffer, bufferOffset);
    // Serialize message field [attitude_setpoint_valid]
    bufferOffset = _serializer.bool(obj.attitude_setpoint_valid, buffer, bufferOffset);
    // Serialize message field [attitude_setpoint]
    bufferOffset = geometry_msgs.msg.Quaternion.serialize(obj.attitude_setpoint, buffer, bufferOffset);
    // Serialize message field [fault_count]
    bufferOffset = _serializer.uint32(obj.fault_count, buffer, bufferOffset);
    // Serialize message field [reason]
    bufferOffset = _serializer.string(obj.reason, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type CeilingAttachmentStatus
    let len;
    let data = new CeilingAttachmentStatus(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [state]
    data.state = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [state_name]
    data.state_name = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [enabled]
    data.enabled = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [acquire_request]
    data.acquire_request = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [keep_ownership]
    data.keep_ownership = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [candidate_valid]
    data.candidate_valid = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [range_fresh]
    data.range_fresh = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [odom_fresh]
    data.odom_fresh = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [flight_state_fresh]
    data.flight_state_fresh = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [attach_confirmed]
    data.attach_confirmed = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [distance_stable]
    data.distance_stable = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [fault_detected]
    data.fault_detected = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [detach_failed]
    data.detach_failed = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [integral_reset_request]
    data.integral_reset_request = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [wheel_stop_request]
    data.wheel_stop_request = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [mechanism_release_requested]
    data.mechanism_release_requested = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [mechanism_release_confirmed]
    data.mechanism_release_confirmed = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [ceiling_distance_raw_m]
    data.ceiling_distance_raw_m = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [ceiling_distance_filtered_m]
    data.ceiling_distance_filtered_m = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [target_distance_m]
    data.target_distance_m = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [compression_m]
    data.compression_m = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [target_compression_m]
    data.target_compression_m = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [approach_velocity_z_enu_mps]
    data.approach_velocity_z_enu_mps = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [thrust_body_z_normalized]
    data.thrust_body_z_normalized = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [attitude_setpoint_valid]
    data.attitude_setpoint_valid = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [attitude_setpoint]
    data.attitude_setpoint = geometry_msgs.msg.Quaternion.deserialize(buffer, bufferOffset);
    // Deserialize message field [fault_count]
    data.fault_count = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [reason]
    data.reason = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += _getByteLength(object.state_name);
    length += _getByteLength(object.reason);
    return length + 89;
  }

  static datatype() {
    // Returns string type for a message object
    return 'px4_wall_ceiling_control/CeilingAttachmentStatus';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'ee6fc76925906c4e79bd0785bd2ec2d9';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Header header
    
    uint8 STATE_NORMAL_FLIGHT=0
    uint8 STATE_CEILING_ARMED=1
    uint8 STATE_APPROACH=2
    uint8 STATE_ATTACH_CONTROL=3
    uint8 STATE_SURFACE_HOLD=4
    uint8 STATE_DETACH=5
    uint8 STATE_RECOVERY_HOVER=6
    uint8 STATE_FAULT=7
    
    uint8 state
    string state_name
    bool enabled
    bool acquire_request
    bool keep_ownership
    bool candidate_valid
    bool range_fresh
    bool odom_fresh
    bool flight_state_fresh
    bool attach_confirmed
    bool distance_stable
    bool fault_detected
    bool detach_failed
    bool integral_reset_request
    bool wheel_stop_request
    bool mechanism_release_requested
    bool mechanism_release_confirmed
    float32 ceiling_distance_raw_m
    float32 ceiling_distance_filtered_m
    float32 target_distance_m
    float32 compression_m
    float32 target_compression_m
    # ROS/MAVROS ENU convention: positive z means upward/toward a horizontal ceiling.
    float32 approach_velocity_z_enu_mps
    # PX4 body FRD convention retained from the reference: negative z is upward thrust.
    float32 thrust_body_z_normalized
    bool attitude_setpoint_valid
    geometry_msgs/Quaternion attitude_setpoint
    uint32 fault_count
    string reason
    
    ================================================================================
    MSG: std_msgs/Header
    # Standard metadata for higher-level stamped data types.
    # This is generally used to communicate timestamped data 
    # in a particular coordinate frame.
    # 
    # sequence ID: consecutively increasing ID 
    uint32 seq
    #Two-integer timestamp that is expressed as:
    # * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
    # * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
    # time-handling sugar is provided by the client library
    time stamp
    #Frame this data is associated with
    string frame_id
    
    ================================================================================
    MSG: geometry_msgs/Quaternion
    # This represents an orientation in free space in quaternion form.
    
    float64 x
    float64 y
    float64 z
    float64 w
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new CeilingAttachmentStatus(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.state !== undefined) {
      resolved.state = msg.state;
    }
    else {
      resolved.state = 0
    }

    if (msg.state_name !== undefined) {
      resolved.state_name = msg.state_name;
    }
    else {
      resolved.state_name = ''
    }

    if (msg.enabled !== undefined) {
      resolved.enabled = msg.enabled;
    }
    else {
      resolved.enabled = false
    }

    if (msg.acquire_request !== undefined) {
      resolved.acquire_request = msg.acquire_request;
    }
    else {
      resolved.acquire_request = false
    }

    if (msg.keep_ownership !== undefined) {
      resolved.keep_ownership = msg.keep_ownership;
    }
    else {
      resolved.keep_ownership = false
    }

    if (msg.candidate_valid !== undefined) {
      resolved.candidate_valid = msg.candidate_valid;
    }
    else {
      resolved.candidate_valid = false
    }

    if (msg.range_fresh !== undefined) {
      resolved.range_fresh = msg.range_fresh;
    }
    else {
      resolved.range_fresh = false
    }

    if (msg.odom_fresh !== undefined) {
      resolved.odom_fresh = msg.odom_fresh;
    }
    else {
      resolved.odom_fresh = false
    }

    if (msg.flight_state_fresh !== undefined) {
      resolved.flight_state_fresh = msg.flight_state_fresh;
    }
    else {
      resolved.flight_state_fresh = false
    }

    if (msg.attach_confirmed !== undefined) {
      resolved.attach_confirmed = msg.attach_confirmed;
    }
    else {
      resolved.attach_confirmed = false
    }

    if (msg.distance_stable !== undefined) {
      resolved.distance_stable = msg.distance_stable;
    }
    else {
      resolved.distance_stable = false
    }

    if (msg.fault_detected !== undefined) {
      resolved.fault_detected = msg.fault_detected;
    }
    else {
      resolved.fault_detected = false
    }

    if (msg.detach_failed !== undefined) {
      resolved.detach_failed = msg.detach_failed;
    }
    else {
      resolved.detach_failed = false
    }

    if (msg.integral_reset_request !== undefined) {
      resolved.integral_reset_request = msg.integral_reset_request;
    }
    else {
      resolved.integral_reset_request = false
    }

    if (msg.wheel_stop_request !== undefined) {
      resolved.wheel_stop_request = msg.wheel_stop_request;
    }
    else {
      resolved.wheel_stop_request = false
    }

    if (msg.mechanism_release_requested !== undefined) {
      resolved.mechanism_release_requested = msg.mechanism_release_requested;
    }
    else {
      resolved.mechanism_release_requested = false
    }

    if (msg.mechanism_release_confirmed !== undefined) {
      resolved.mechanism_release_confirmed = msg.mechanism_release_confirmed;
    }
    else {
      resolved.mechanism_release_confirmed = false
    }

    if (msg.ceiling_distance_raw_m !== undefined) {
      resolved.ceiling_distance_raw_m = msg.ceiling_distance_raw_m;
    }
    else {
      resolved.ceiling_distance_raw_m = 0.0
    }

    if (msg.ceiling_distance_filtered_m !== undefined) {
      resolved.ceiling_distance_filtered_m = msg.ceiling_distance_filtered_m;
    }
    else {
      resolved.ceiling_distance_filtered_m = 0.0
    }

    if (msg.target_distance_m !== undefined) {
      resolved.target_distance_m = msg.target_distance_m;
    }
    else {
      resolved.target_distance_m = 0.0
    }

    if (msg.compression_m !== undefined) {
      resolved.compression_m = msg.compression_m;
    }
    else {
      resolved.compression_m = 0.0
    }

    if (msg.target_compression_m !== undefined) {
      resolved.target_compression_m = msg.target_compression_m;
    }
    else {
      resolved.target_compression_m = 0.0
    }

    if (msg.approach_velocity_z_enu_mps !== undefined) {
      resolved.approach_velocity_z_enu_mps = msg.approach_velocity_z_enu_mps;
    }
    else {
      resolved.approach_velocity_z_enu_mps = 0.0
    }

    if (msg.thrust_body_z_normalized !== undefined) {
      resolved.thrust_body_z_normalized = msg.thrust_body_z_normalized;
    }
    else {
      resolved.thrust_body_z_normalized = 0.0
    }

    if (msg.attitude_setpoint_valid !== undefined) {
      resolved.attitude_setpoint_valid = msg.attitude_setpoint_valid;
    }
    else {
      resolved.attitude_setpoint_valid = false
    }

    if (msg.attitude_setpoint !== undefined) {
      resolved.attitude_setpoint = geometry_msgs.msg.Quaternion.Resolve(msg.attitude_setpoint)
    }
    else {
      resolved.attitude_setpoint = new geometry_msgs.msg.Quaternion()
    }

    if (msg.fault_count !== undefined) {
      resolved.fault_count = msg.fault_count;
    }
    else {
      resolved.fault_count = 0
    }

    if (msg.reason !== undefined) {
      resolved.reason = msg.reason;
    }
    else {
      resolved.reason = ''
    }

    return resolved;
    }
};

// Constants for message
CeilingAttachmentStatus.Constants = {
  STATE_NORMAL_FLIGHT: 0,
  STATE_CEILING_ARMED: 1,
  STATE_APPROACH: 2,
  STATE_ATTACH_CONTROL: 3,
  STATE_SURFACE_HOLD: 4,
  STATE_DETACH: 5,
  STATE_RECOVERY_HOVER: 6,
  STATE_FAULT: 7,
}

module.exports = CeilingAttachmentStatus;
