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

class ControllerStatus {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.source = null;
      this.healthy = null;
      this.input_fresh = null;
      this.target_reached = null;
      this.detail = null;
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
      if (initObj.hasOwnProperty('healthy')) {
        this.healthy = initObj.healthy
      }
      else {
        this.healthy = false;
      }
      if (initObj.hasOwnProperty('input_fresh')) {
        this.input_fresh = initObj.input_fresh
      }
      else {
        this.input_fresh = false;
      }
      if (initObj.hasOwnProperty('target_reached')) {
        this.target_reached = initObj.target_reached
      }
      else {
        this.target_reached = false;
      }
      if (initObj.hasOwnProperty('detail')) {
        this.detail = initObj.detail
      }
      else {
        this.detail = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ControllerStatus
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [source]
    bufferOffset = _serializer.uint8(obj.source, buffer, bufferOffset);
    // Serialize message field [healthy]
    bufferOffset = _serializer.bool(obj.healthy, buffer, bufferOffset);
    // Serialize message field [input_fresh]
    bufferOffset = _serializer.bool(obj.input_fresh, buffer, bufferOffset);
    // Serialize message field [target_reached]
    bufferOffset = _serializer.bool(obj.target_reached, buffer, bufferOffset);
    // Serialize message field [detail]
    bufferOffset = _serializer.string(obj.detail, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ControllerStatus
    let len;
    let data = new ControllerStatus(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [source]
    data.source = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [healthy]
    data.healthy = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [input_fresh]
    data.input_fresh = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [target_reached]
    data.target_reached = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [detail]
    data.detail = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += _getByteLength(object.detail);
    return length + 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'px4_wall_ceiling_control/ControllerStatus';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'a4190f296fb56a105a72ae34460a659f';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Header header
    
    uint8 SOURCE_NONE=0
    uint8 SOURCE_CEILING=1
    uint8 SOURCE_WALL=2
    
    uint8 source
    bool healthy
    bool input_fresh
    bool target_reached
    string detail
    
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
    const resolved = new ControllerStatus(null);
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

    if (msg.healthy !== undefined) {
      resolved.healthy = msg.healthy;
    }
    else {
      resolved.healthy = false
    }

    if (msg.input_fresh !== undefined) {
      resolved.input_fresh = msg.input_fresh;
    }
    else {
      resolved.input_fresh = false
    }

    if (msg.target_reached !== undefined) {
      resolved.target_reached = msg.target_reached;
    }
    else {
      resolved.target_reached = false
    }

    if (msg.detail !== undefined) {
      resolved.detail = msg.detail;
    }
    else {
      resolved.detail = ''
    }

    return resolved;
    }
};

// Constants for message
ControllerStatus.Constants = {
  SOURCE_NONE: 0,
  SOURCE_CEILING: 1,
  SOURCE_WALL: 2,
}

module.exports = ControllerStatus;
