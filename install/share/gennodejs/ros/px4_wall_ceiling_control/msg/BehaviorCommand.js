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

class BehaviorCommand {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.source = null;
      this.state = null;
      this.active = null;
      this.target_distance_m = null;
      this.max_speed_mps = null;
      this.reason = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('source')) {
        this.source = initObj.source
      }
      else {
        this.source = 0;
      }
      if (initObj.hasOwnProperty('state')) {
        this.state = initObj.state
      }
      else {
        this.state = 0;
      }
      if (initObj.hasOwnProperty('active')) {
        this.active = initObj.active
      }
      else {
        this.active = false;
      }
      if (initObj.hasOwnProperty('target_distance_m')) {
        this.target_distance_m = initObj.target_distance_m
      }
      else {
        this.target_distance_m = 0.0;
      }
      if (initObj.hasOwnProperty('max_speed_mps')) {
        this.max_speed_mps = initObj.max_speed_mps
      }
      else {
        this.max_speed_mps = 0.0;
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
    // Serializes a message object of type BehaviorCommand
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [source]
    bufferOffset = _serializer.uint8(obj.source, buffer, bufferOffset);
    // Serialize message field [state]
    bufferOffset = _serializer.uint8(obj.state, buffer, bufferOffset);
    // Serialize message field [active]
    bufferOffset = _serializer.bool(obj.active, buffer, bufferOffset);
    // Serialize message field [target_distance_m]
    bufferOffset = _serializer.float32(obj.target_distance_m, buffer, bufferOffset);
    // Serialize message field [max_speed_mps]
    bufferOffset = _serializer.float32(obj.max_speed_mps, buffer, bufferOffset);
    // Serialize message field [reason]
    bufferOffset = _serializer.string(obj.reason, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type BehaviorCommand
    let len;
    let data = new BehaviorCommand(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [source]
    data.source = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [state]
    data.state = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [active]
    data.active = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [target_distance_m]
    data.target_distance_m = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [max_speed_mps]
    data.max_speed_mps = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [reason]
    data.reason = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += _getByteLength(object.reason);
    return length + 15;
  }

  static datatype() {
    // Returns string type for a message object
    return 'px4_wall_ceiling_control/BehaviorCommand';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'c5b91608717c6c01656752aca3ecaa53';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Header header
    
    uint8 SOURCE_NONE=0
    uint8 SOURCE_CEILING=1
    uint8 SOURCE_WALL=2
    
    uint8 STATE_DISABLED=0
    uint8 STATE_APPROACH=1
    uint8 STATE_TRACK=2
    uint8 STATE_RETREAT=3
    uint8 STATE_FAULT=4
    
    uint8 source
    uint8 state
    bool active
    float32 target_distance_m
    float32 max_speed_mps
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
    const resolved = new BehaviorCommand(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.source !== undefined) {
      resolved.source = msg.source;
    }
    else {
      resolved.source = 0
    }

    if (msg.state !== undefined) {
      resolved.state = msg.state;
    }
    else {
      resolved.state = 0
    }

    if (msg.active !== undefined) {
      resolved.active = msg.active;
    }
    else {
      resolved.active = false
    }

    if (msg.target_distance_m !== undefined) {
      resolved.target_distance_m = msg.target_distance_m;
    }
    else {
      resolved.target_distance_m = 0.0
    }

    if (msg.max_speed_mps !== undefined) {
      resolved.max_speed_mps = msg.max_speed_mps;
    }
    else {
      resolved.max_speed_mps = 0.0
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
BehaviorCommand.Constants = {
  SOURCE_NONE: 0,
  SOURCE_CEILING: 1,
  SOURCE_WALL: 2,
  STATE_DISABLED: 0,
  STATE_APPROACH: 1,
  STATE_TRACK: 2,
  STATE_RETREAT: 3,
  STATE_FAULT: 4,
}

module.exports = BehaviorCommand;
