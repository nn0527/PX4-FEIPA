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

class SensorHealth {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.ceiling_valid = null;
      this.front_valid = null;
      this.ceiling_age_s = null;
      this.front_age_s = null;
      this.ceiling_sequence = null;
      this.front_sequence = null;
      this.ceiling_invalid_count = null;
      this.front_invalid_count = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('ceiling_valid')) {
        this.ceiling_valid = initObj.ceiling_valid
      }
      else {
        this.ceiling_valid = false;
      }
      if (initObj.hasOwnProperty('front_valid')) {
        this.front_valid = initObj.front_valid
      }
      else {
        this.front_valid = false;
      }
      if (initObj.hasOwnProperty('ceiling_age_s')) {
        this.ceiling_age_s = initObj.ceiling_age_s
      }
      else {
        this.ceiling_age_s = 0.0;
      }
      if (initObj.hasOwnProperty('front_age_s')) {
        this.front_age_s = initObj.front_age_s
      }
      else {
        this.front_age_s = 0.0;
      }
      if (initObj.hasOwnProperty('ceiling_sequence')) {
        this.ceiling_sequence = initObj.ceiling_sequence
      }
      else {
        this.ceiling_sequence = 0;
      }
      if (initObj.hasOwnProperty('front_sequence')) {
        this.front_sequence = initObj.front_sequence
      }
      else {
        this.front_sequence = 0;
      }
      if (initObj.hasOwnProperty('ceiling_invalid_count')) {
        this.ceiling_invalid_count = initObj.ceiling_invalid_count
      }
      else {
        this.ceiling_invalid_count = 0;
      }
      if (initObj.hasOwnProperty('front_invalid_count')) {
        this.front_invalid_count = initObj.front_invalid_count
      }
      else {
        this.front_invalid_count = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SensorHealth
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [ceiling_valid]
    bufferOffset = _serializer.bool(obj.ceiling_valid, buffer, bufferOffset);
    // Serialize message field [front_valid]
    bufferOffset = _serializer.bool(obj.front_valid, buffer, bufferOffset);
    // Serialize message field [ceiling_age_s]
    bufferOffset = _serializer.float32(obj.ceiling_age_s, buffer, bufferOffset);
    // Serialize message field [front_age_s]
    bufferOffset = _serializer.float32(obj.front_age_s, buffer, bufferOffset);
    // Serialize message field [ceiling_sequence]
    bufferOffset = _serializer.uint32(obj.ceiling_sequence, buffer, bufferOffset);
    // Serialize message field [front_sequence]
    bufferOffset = _serializer.uint32(obj.front_sequence, buffer, bufferOffset);
    // Serialize message field [ceiling_invalid_count]
    bufferOffset = _serializer.uint32(obj.ceiling_invalid_count, buffer, bufferOffset);
    // Serialize message field [front_invalid_count]
    bufferOffset = _serializer.uint32(obj.front_invalid_count, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SensorHealth
    let len;
    let data = new SensorHealth(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [ceiling_valid]
    data.ceiling_valid = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [front_valid]
    data.front_valid = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [ceiling_age_s]
    data.ceiling_age_s = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [front_age_s]
    data.front_age_s = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [ceiling_sequence]
    data.ceiling_sequence = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [front_sequence]
    data.front_sequence = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [ceiling_invalid_count]
    data.ceiling_invalid_count = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [front_invalid_count]
    data.front_invalid_count = _deserializer.uint32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 26;
  }

  static datatype() {
    // Returns string type for a message object
    return 'px4_wall_ceiling_control/SensorHealth';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '890f31b2daff364b0bc989c3503517bf';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Header header
    
    bool ceiling_valid
    bool front_valid
    
    # Age of the last valid sample as measured at this node. Infinity means that
    # no valid sample has been received since startup.
    float32 ceiling_age_s
    float32 front_age_s
    
    # Monotonic counters allow consumers and tests to distinguish real samples
    # from repeated status publications.
    uint32 ceiling_sequence
    uint32 front_sequence
    uint32 ceiling_invalid_count
    uint32 front_invalid_count
    
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
    const resolved = new SensorHealth(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.ceiling_valid !== undefined) {
      resolved.ceiling_valid = msg.ceiling_valid;
    }
    else {
      resolved.ceiling_valid = false
    }

    if (msg.front_valid !== undefined) {
      resolved.front_valid = msg.front_valid;
    }
    else {
      resolved.front_valid = false
    }

    if (msg.ceiling_age_s !== undefined) {
      resolved.ceiling_age_s = msg.ceiling_age_s;
    }
    else {
      resolved.ceiling_age_s = 0.0
    }

    if (msg.front_age_s !== undefined) {
      resolved.front_age_s = msg.front_age_s;
    }
    else {
      resolved.front_age_s = 0.0
    }

    if (msg.ceiling_sequence !== undefined) {
      resolved.ceiling_sequence = msg.ceiling_sequence;
    }
    else {
      resolved.ceiling_sequence = 0
    }

    if (msg.front_sequence !== undefined) {
      resolved.front_sequence = msg.front_sequence;
    }
    else {
      resolved.front_sequence = 0
    }

    if (msg.ceiling_invalid_count !== undefined) {
      resolved.ceiling_invalid_count = msg.ceiling_invalid_count;
    }
    else {
      resolved.ceiling_invalid_count = 0
    }

    if (msg.front_invalid_count !== undefined) {
      resolved.front_invalid_count = msg.front_invalid_count;
    }
    else {
      resolved.front_invalid_count = 0
    }

    return resolved;
    }
};

module.exports = SensorHealth;
