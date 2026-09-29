// Auto-generated. Do not edit!

// (in-package px4_wall_ceiling_control.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');
let geometry_msgs = _finder('geometry_msgs');

//-----------------------------------------------------------

class WallPerchStatus {
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
      this.front_range_fresh = null;
      this.top_range_fresh = null;
      this.sensor_health_fresh = null;
      this.top_preflight_ready = null;
      this.odom_fresh = null;
      this.flight_state_fresh = null;
      this.front_ready = null;
      this.flip_ready = null;
      this.top_contact_ready = null;
      this.top_clear_ready = null;
      this.fault_detected = null;
      this.neutral_setpoint_requested = null;
      this.attitude_setpoint_valid = null;
      this.front_distance_raw_m = null;
      this.front_distance_filtered_m = null;
      this.top_distance_raw_m = null;
      this.top_distance_filtered_m = null;
      this.progress = null;
      this.attitude_setpoint = null;
      this.thrust_normalized = null;
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
      if (initObj.hasOwnProperty('front_range_fresh')) {
        this.front_range_fresh = initObj.front_range_fresh
      }
      else {
        this.front_range_fresh = false;
      }
      if (initObj.hasOwnProperty('top_range_fresh')) {
        this.top_range_fresh = initObj.top_range_fresh
      }
      else {
        this.top_range_fresh = false;
      }
      if (initObj.hasOwnProperty('sensor_health_fresh')) {
        this.sensor_health_fresh = initObj.sensor_health_fresh
      }
      else {
        this.sensor_health_fresh = false;
      }
      if (initObj.hasOwnProperty('top_preflight_ready')) {
        this.top_preflight_ready = initObj.top_preflight_ready
      }
      else {
        this.top_preflight_ready = false;
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
      if (initObj.hasOwnProperty('front_ready')) {
        this.front_ready = initObj.front_ready
      }
      else {
        this.front_ready = false;
      }
      if (initObj.hasOwnProperty('flip_ready')) {
        this.flip_ready = initObj.flip_ready
      }
      else {
        this.flip_ready = false;
      }
      if (initObj.hasOwnProperty('top_contact_ready')) {
        this.top_contact_ready = initObj.top_contact_ready
      }
      else {
        this.top_contact_ready = false;
      }
      if (initObj.hasOwnProperty('top_clear_ready')) {
        this.top_clear_ready = initObj.top_clear_ready
      }
      else {
        this.top_clear_ready = false;
      }
      if (initObj.hasOwnProperty('fault_detected')) {
        this.fault_detected = initObj.fault_detected
      }
      else {
        this.fault_detected = false;
      }
      if (initObj.hasOwnProperty('neutral_setpoint_requested')) {
        this.neutral_setpoint_requested = initObj.neutral_setpoint_requested
      }
      else {
        this.neutral_setpoint_requested = false;
      }
      if (initObj.hasOwnProperty('attitude_setpoint_valid')) {
        this.attitude_setpoint_valid = initObj.attitude_setpoint_valid
      }
      else {
        this.attitude_setpoint_valid = false;
      }
      if (initObj.hasOwnProperty('front_distance_raw_m')) {
        this.front_distance_raw_m = initObj.front_distance_raw_m
      }
      else {
        this.front_distance_raw_m = 0.0;
      }
      if (initObj.hasOwnProperty('front_distance_filtered_m')) {
        this.front_distance_filtered_m = initObj.front_distance_filtered_m
      }
      else {
        this.front_distance_filtered_m = 0.0;
      }
      if (initObj.hasOwnProperty('top_distance_raw_m')) {
        this.top_distance_raw_m = initObj.top_distance_raw_m
      }
      else {
        this.top_distance_raw_m = 0.0;
      }
      if (initObj.hasOwnProperty('top_distance_filtered_m')) {
        this.top_distance_filtered_m = initObj.top_distance_filtered_m
      }
      else {
        this.top_distance_filtered_m = 0.0;
      }
      if (initObj.hasOwnProperty('progress')) {
        this.progress = initObj.progress
      }
      else {
        this.progress = 0.0;
      }
      if (initObj.hasOwnProperty('attitude_setpoint')) {
        this.attitude_setpoint = initObj.attitude_setpoint
      }
      else {
        this.attitude_setpoint = new geometry_msgs.msg.Quaternion();
      }
      if (initObj.hasOwnProperty('thrust_normalized')) {
        this.thrust_normalized = initObj.thrust_normalized
      }
      else {
        this.thrust_normalized = 0.0;
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
    // Serializes a message object of type WallPerchStatus
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
    // Serialize message field [front_range_fresh]
    bufferOffset = _serializer.bool(obj.front_range_fresh, buffer, bufferOffset);
    // Serialize message field [top_range_fresh]
    bufferOffset = _serializer.bool(obj.top_range_fresh, buffer, bufferOffset);
    // Serialize message field [sensor_health_fresh]
    bufferOffset = _serializer.bool(obj.sensor_health_fresh, buffer, bufferOffset);
    // Serialize message field [top_preflight_ready]
    bufferOffset = _serializer.bool(obj.top_preflight_ready, buffer, bufferOffset);
    // Serialize message field [odom_fresh]
    bufferOffset = _serializer.bool(obj.odom_fresh, buffer, bufferOffset);
    // Serialize message field [flight_state_fresh]
    bufferOffset = _serializer.bool(obj.flight_state_fresh, buffer, bufferOffset);
    // Serialize message field [front_ready]
    bufferOffset = _serializer.bool(obj.front_ready, buffer, bufferOffset);
    // Serialize message field [flip_ready]
    bufferOffset = _serializer.bool(obj.flip_ready, buffer, bufferOffset);
    // Serialize message field [top_contact_ready]
    bufferOffset = _serializer.bool(obj.top_contact_ready, buffer, bufferOffset);
    // Serialize message field [top_clear_ready]
    bufferOffset = _serializer.bool(obj.top_clear_ready, buffer, bufferOffset);
    // Serialize message field [fault_detected]
    bufferOffset = _serializer.bool(obj.fault_detected, buffer, bufferOffset);
    // Serialize message field [neutral_setpoint_requested]
    bufferOffset = _serializer.bool(obj.neutral_setpoint_requested, buffer, bufferOffset);
    // Serialize message field [attitude_setpoint_valid]
    bufferOffset = _serializer.bool(obj.attitude_setpoint_valid, buffer, bufferOffset);
    // Serialize message field [front_distance_raw_m]
    bufferOffset = _serializer.float32(obj.front_distance_raw_m, buffer, bufferOffset);
    // Serialize message field [front_distance_filtered_m]
    bufferOffset = _serializer.float32(obj.front_distance_filtered_m, buffer, bufferOffset);
    // Serialize message field [top_distance_raw_m]
    bufferOffset = _serializer.float32(obj.top_distance_raw_m, buffer, bufferOffset);
    // Serialize message field [top_distance_filtered_m]
    bufferOffset = _serializer.float32(obj.top_distance_filtered_m, buffer, bufferOffset);
    // Serialize message field [progress]
    bufferOffset = _serializer.float32(obj.progress, buffer, bufferOffset);
    // Serialize message field [attitude_setpoint]
    bufferOffset = geometry_msgs.msg.Quaternion.serialize(obj.attitude_setpoint, buffer, bufferOffset);
    // Serialize message field [thrust_normalized]
    bufferOffset = _serializer.float32(obj.thrust_normalized, buffer, bufferOffset);
    // Serialize message field [fault_count]
    bufferOffset = _serializer.uint32(obj.fault_count, buffer, bufferOffset);
    // Serialize message field [reason]
    bufferOffset = _serializer.string(obj.reason, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type WallPerchStatus
    let len;
    let data = new WallPerchStatus(null);
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
    // Deserialize message field [front_range_fresh]
    data.front_range_fresh = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [top_range_fresh]
    data.top_range_fresh = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [sensor_health_fresh]
    data.sensor_health_fresh = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [top_preflight_ready]
    data.top_preflight_ready = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [odom_fresh]
    data.odom_fresh = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [flight_state_fresh]
    data.flight_state_fresh = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [front_ready]
    data.front_ready = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [flip_ready]
    data.flip_ready = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [top_contact_ready]
    data.top_contact_ready = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [top_clear_ready]
    data.top_clear_ready = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [fault_detected]
    data.fault_detected = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [neutral_setpoint_requested]
    data.neutral_setpoint_requested = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [attitude_setpoint_valid]
    data.attitude_setpoint_valid = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [front_distance_raw_m]
    data.front_distance_raw_m = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [front_distance_filtered_m]
    data.front_distance_filtered_m = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [top_distance_raw_m]
    data.top_distance_raw_m = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [top_distance_filtered_m]
    data.top_distance_filtered_m = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [progress]
    data.progress = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [attitude_setpoint]
    data.attitude_setpoint = geometry_msgs.msg.Quaternion.deserialize(buffer, bufferOffset);
    // Deserialize message field [thrust_normalized]
    data.thrust_normalized = _deserializer.float32(buffer, bufferOffset);
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
    return length + 86;
  }

  static datatype() {
    // Returns string type for a message object
    return 'px4_wall_ceiling_control/WallPerchStatus';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '14c66e7871a6c1f7a0bda70d194bcb80';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Header header
    
    uint8 STATE_IDLE=0
    uint8 STATE_FRONT_WALL_DETECT=1
    uint8 STATE_STABILIZE_HOVER=2
    uint8 STATE_SLOW_APPROACH=3
    uint8 STATE_FLIP_TO_WALL=4
    uint8 STATE_WALL_CAPTURE=5
    uint8 STATE_WALL_HOLD=6
    uint8 STATE_WALL_PIN=7
    uint8 STATE_DETACH_ROTATE=8
    uint8 STATE_RECOVER=9
    uint8 STATE_EXIT=10
    uint8 STATE_ABORT=11
    
    uint8 state
    string state_name
    bool enabled
    bool acquire_request
    bool keep_ownership
    bool candidate_valid
    bool front_range_fresh
    bool top_range_fresh
    bool sensor_health_fresh
    bool top_preflight_ready
    bool odom_fresh
    bool flight_state_fresh
    bool front_ready
    bool flip_ready
    bool top_contact_ready
    bool top_clear_ready
    bool fault_detected
    bool neutral_setpoint_requested
    bool attitude_setpoint_valid
    float32 front_distance_raw_m
    float32 front_distance_filtered_m
    float32 top_distance_raw_m
    float32 top_distance_filtered_m
    float32 progress
    geometry_msgs/Quaternion attitude_setpoint
    # MAVROS AttitudeTarget convention: positive normalized collective thrust.
    float32 thrust_normalized
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
    const resolved = new WallPerchStatus(null);
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

    if (msg.front_range_fresh !== undefined) {
      resolved.front_range_fresh = msg.front_range_fresh;
    }
    else {
      resolved.front_range_fresh = false
    }

    if (msg.top_range_fresh !== undefined) {
      resolved.top_range_fresh = msg.top_range_fresh;
    }
    else {
      resolved.top_range_fresh = false
    }

    if (msg.sensor_health_fresh !== undefined) {
      resolved.sensor_health_fresh = msg.sensor_health_fresh;
    }
    else {
      resolved.sensor_health_fresh = false
    }

    if (msg.top_preflight_ready !== undefined) {
      resolved.top_preflight_ready = msg.top_preflight_ready;
    }
    else {
      resolved.top_preflight_ready = false
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

    if (msg.front_ready !== undefined) {
      resolved.front_ready = msg.front_ready;
    }
    else {
      resolved.front_ready = false
    }

    if (msg.flip_ready !== undefined) {
      resolved.flip_ready = msg.flip_ready;
    }
    else {
      resolved.flip_ready = false
    }

    if (msg.top_contact_ready !== undefined) {
      resolved.top_contact_ready = msg.top_contact_ready;
    }
    else {
      resolved.top_contact_ready = false
    }

    if (msg.top_clear_ready !== undefined) {
      resolved.top_clear_ready = msg.top_clear_ready;
    }
    else {
      resolved.top_clear_ready = false
    }

    if (msg.fault_detected !== undefined) {
      resolved.fault_detected = msg.fault_detected;
    }
    else {
      resolved.fault_detected = false
    }

    if (msg.neutral_setpoint_requested !== undefined) {
      resolved.neutral_setpoint_requested = msg.neutral_setpoint_requested;
    }
    else {
      resolved.neutral_setpoint_requested = false
    }

    if (msg.attitude_setpoint_valid !== undefined) {
      resolved.attitude_setpoint_valid = msg.attitude_setpoint_valid;
    }
    else {
      resolved.attitude_setpoint_valid = false
    }

    if (msg.front_distance_raw_m !== undefined) {
      resolved.front_distance_raw_m = msg.front_distance_raw_m;
    }
    else {
      resolved.front_distance_raw_m = 0.0
    }

    if (msg.front_distance_filtered_m !== undefined) {
      resolved.front_distance_filtered_m = msg.front_distance_filtered_m;
    }
    else {
      resolved.front_distance_filtered_m = 0.0
    }

    if (msg.top_distance_raw_m !== undefined) {
      resolved.top_distance_raw_m = msg.top_distance_raw_m;
    }
    else {
      resolved.top_distance_raw_m = 0.0
    }

    if (msg.top_distance_filtered_m !== undefined) {
      resolved.top_distance_filtered_m = msg.top_distance_filtered_m;
    }
    else {
      resolved.top_distance_filtered_m = 0.0
    }

    if (msg.progress !== undefined) {
      resolved.progress = msg.progress;
    }
    else {
      resolved.progress = 0.0
    }

    if (msg.attitude_setpoint !== undefined) {
      resolved.attitude_setpoint = geometry_msgs.msg.Quaternion.Resolve(msg.attitude_setpoint)
    }
    else {
      resolved.attitude_setpoint = new geometry_msgs.msg.Quaternion()
    }

    if (msg.thrust_normalized !== undefined) {
      resolved.thrust_normalized = msg.thrust_normalized;
    }
    else {
      resolved.thrust_normalized = 0.0
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
WallPerchStatus.Constants = {
  STATE_IDLE: 0,
  STATE_FRONT_WALL_DETECT: 1,
  STATE_STABILIZE_HOVER: 2,
  STATE_SLOW_APPROACH: 3,
  STATE_FLIP_TO_WALL: 4,
  STATE_WALL_CAPTURE: 5,
  STATE_WALL_HOLD: 6,
  STATE_WALL_PIN: 7,
  STATE_DETACH_ROTATE: 8,
  STATE_RECOVER: 9,
  STATE_EXIT: 10,
  STATE_ABORT: 11,
}

module.exports = WallPerchStatus;
