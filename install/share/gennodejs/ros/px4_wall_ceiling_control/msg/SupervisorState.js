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

//-----------------------------------------------------------

class SupervisorState {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.state = null;
      this.active_mode = null;
      this.setpoint_stream_allowed = null;
      this.motion_allowed = null;
      this.mavros_connected = null;
      this.armed = null;
      this.offboard = null;
      this.operator_fresh = null;
      this.odom_fresh = null;
      this.ceiling_range_fresh = null;
      this.wall_range_fresh = null;
      this.fault_mask = null;
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
      if (initObj.hasOwnProperty('active_mode')) {
        this.active_mode = initObj.active_mode
      }
      else {
        this.active_mode = 0;
      }
      if (initObj.hasOwnProperty('setpoint_stream_allowed')) {
        this.setpoint_stream_allowed = initObj.setpoint_stream_allowed
      }
      else {
        this.setpoint_stream_allowed = false;
      }
      if (initObj.hasOwnProperty('motion_allowed')) {
        this.motion_allowed = initObj.motion_allowed
      }
      else {
        this.motion_allowed = false;
      }
      if (initObj.hasOwnProperty('mavros_connected')) {
        this.mavros_connected = initObj.mavros_connected
      }
      else {
        this.mavros_connected = false;
      }
      if (initObj.hasOwnProperty('armed')) {
        this.armed = initObj.armed
      }
      else {
        this.armed = false;
      }
      if (initObj.hasOwnProperty('offboard')) {
        this.offboard = initObj.offboard
      }
      else {
        this.offboard = false;
      }
      if (initObj.hasOwnProperty('operator_fresh')) {
        this.operator_fresh = initObj.operator_fresh
      }
      else {
        this.operator_fresh = false;
      }
      if (initObj.hasOwnProperty('odom_fresh')) {
        this.odom_fresh = initObj.odom_fresh
      }
      else {
        this.odom_fresh = false;
      }
      if (initObj.hasOwnProperty('ceiling_range_fresh')) {
        this.ceiling_range_fresh = initObj.ceiling_range_fresh
      }
      else {
        this.ceiling_range_fresh = false;
      }
      if (initObj.hasOwnProperty('wall_range_fresh')) {
        this.wall_range_fresh = initObj.wall_range_fresh
      }
      else {
        this.wall_range_fresh = false;
      }
      if (initObj.hasOwnProperty('fault_mask')) {
        this.fault_mask = initObj.fault_mask
      }
      else {
        this.fault_mask = 0;
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
    // Serializes a message object of type SupervisorState
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [state]
    bufferOffset = _serializer.uint8(obj.state, buffer, bufferOffset);
    // Serialize message field [active_mode]
    bufferOffset = _serializer.uint8(obj.active_mode, buffer, bufferOffset);
    // Serialize message field [setpoint_stream_allowed]
    bufferOffset = _serializer.bool(obj.setpoint_stream_allowed, buffer, bufferOffset);
    // Serialize message field [motion_allowed]
    bufferOffset = _serializer.bool(obj.motion_allowed, buffer, bufferOffset);
    // Serialize message field [mavros_connected]
    bufferOffset = _serializer.bool(obj.mavros_connected, buffer, bufferOffset);
    // Serialize message field [armed]
    bufferOffset = _serializer.bool(obj.armed, buffer, bufferOffset);
    // Serialize message field [offboard]
    bufferOffset = _serializer.bool(obj.offboard, buffer, bufferOffset);
    // Serialize message field [operator_fresh]
    bufferOffset = _serializer.bool(obj.operator_fresh, buffer, bufferOffset);
    // Serialize message field [odom_fresh]
    bufferOffset = _serializer.bool(obj.odom_fresh, buffer, bufferOffset);
    // Serialize message field [ceiling_range_fresh]
    bufferOffset = _serializer.bool(obj.ceiling_range_fresh, buffer, bufferOffset);
    // Serialize message field [wall_range_fresh]
    bufferOffset = _serializer.bool(obj.wall_range_fresh, buffer, bufferOffset);
    // Serialize message field [fault_mask]
    bufferOffset = _serializer.uint32(obj.fault_mask, buffer, bufferOffset);
    // Serialize message field [reason]
    bufferOffset = _serializer.string(obj.reason, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SupervisorState
    let len;
    let data = new SupervisorState(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [state]
    data.state = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [active_mode]
    data.active_mode = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [setpoint_stream_allowed]
    data.setpoint_stream_allowed = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [motion_allowed]
    data.motion_allowed = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [mavros_connected]
    data.mavros_connected = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [armed]
    data.armed = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [offboard]
    data.offboard = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [operator_fresh]
    data.operator_fresh = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [odom_fresh]
    data.odom_fresh = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [ceiling_range_fresh]
    data.ceiling_range_fresh = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [wall_range_fresh]
    data.wall_range_fresh = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [fault_mask]
    data.fault_mask = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [reason]
    data.reason = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += _getByteLength(object.reason);
    return length + 19;
  }

  static datatype() {
    // Returns string type for a message object
    return 'px4_wall_ceiling_control/SupervisorState';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '1ec3406c7c8a31f63daae31442cabb75';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Header header
    
    uint8 STATE_WAIT_LINK=0
    uint8 STATE_MANUAL=1
    uint8 STATE_STANDBY=2
    uint8 STATE_PRESTREAM=3
    uint8 STATE_CEILING_ACTIVE=4
    uint8 STATE_WALL_ACTIVE=5
    uint8 STATE_FAULT=6
    
    uint8 MODE_NONE=0
    uint8 MODE_CEILING=1
    uint8 MODE_WALL=2
    
    uint32 FAULT_NONE=0
    uint32 FAULT_MAVROS=1
    uint32 FAULT_OPERATOR=2
    uint32 FAULT_ODOMETRY=4
    uint32 FAULT_CEILING_RANGE=8
    uint32 FAULT_WALL_RANGE=16
    uint32 FAULT_UNKNOWN_MODE=32
    
    uint8 state
    uint8 active_mode
    bool setpoint_stream_allowed
    bool motion_allowed
    bool mavros_connected
    bool armed
    bool offboard
    bool operator_fresh
    bool odom_fresh
    bool ceiling_range_fresh
    bool wall_range_fresh
    uint32 fault_mask
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
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SupervisorState(null);
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

    if (msg.active_mode !== undefined) {
      resolved.active_mode = msg.active_mode;
    }
    else {
      resolved.active_mode = 0
    }

    if (msg.setpoint_stream_allowed !== undefined) {
      resolved.setpoint_stream_allowed = msg.setpoint_stream_allowed;
    }
    else {
      resolved.setpoint_stream_allowed = false
    }

    if (msg.motion_allowed !== undefined) {
      resolved.motion_allowed = msg.motion_allowed;
    }
    else {
      resolved.motion_allowed = false
    }

    if (msg.mavros_connected !== undefined) {
      resolved.mavros_connected = msg.mavros_connected;
    }
    else {
      resolved.mavros_connected = false
    }

    if (msg.armed !== undefined) {
      resolved.armed = msg.armed;
    }
    else {
      resolved.armed = false
    }

    if (msg.offboard !== undefined) {
      resolved.offboard = msg.offboard;
    }
    else {
      resolved.offboard = false
    }

    if (msg.operator_fresh !== undefined) {
      resolved.operator_fresh = msg.operator_fresh;
    }
    else {
      resolved.operator_fresh = false
    }

    if (msg.odom_fresh !== undefined) {
      resolved.odom_fresh = msg.odom_fresh;
    }
    else {
      resolved.odom_fresh = false
    }

    if (msg.ceiling_range_fresh !== undefined) {
      resolved.ceiling_range_fresh = msg.ceiling_range_fresh;
    }
    else {
      resolved.ceiling_range_fresh = false
    }

    if (msg.wall_range_fresh !== undefined) {
      resolved.wall_range_fresh = msg.wall_range_fresh;
    }
    else {
      resolved.wall_range_fresh = false
    }

    if (msg.fault_mask !== undefined) {
      resolved.fault_mask = msg.fault_mask;
    }
    else {
      resolved.fault_mask = 0
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
SupervisorState.Constants = {
  STATE_WAIT_LINK: 0,
  STATE_MANUAL: 1,
  STATE_STANDBY: 2,
  STATE_PRESTREAM: 3,
  STATE_CEILING_ACTIVE: 4,
  STATE_WALL_ACTIVE: 5,
  STATE_FAULT: 6,
  MODE_NONE: 0,
  MODE_CEILING: 1,
  MODE_WALL: 2,
  FAULT_NONE: 0,
  FAULT_MAVROS: 1,
  FAULT_OPERATOR: 2,
  FAULT_ODOMETRY: 4,
  FAULT_CEILING_RANGE: 8,
  FAULT_WALL_RANGE: 16,
  FAULT_UNKNOWN_MODE: 32,
}

module.exports = SupervisorState;
