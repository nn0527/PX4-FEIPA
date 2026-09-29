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

class AttachmentMechanismStatus {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.source = null;
      this.sequence = null;
      this.acknowledged = null;
      this.released = null;
      this.fault = null;
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
      if (initObj.hasOwnProperty('sequence')) {
        this.sequence = initObj.sequence
      }
      else {
        this.sequence = 0;
      }
      if (initObj.hasOwnProperty('acknowledged')) {
        this.acknowledged = initObj.acknowledged
      }
      else {
        this.acknowledged = false;
      }
      if (initObj.hasOwnProperty('released')) {
        this.released = initObj.released
      }
      else {
        this.released = false;
      }
      if (initObj.hasOwnProperty('fault')) {
        this.fault = initObj.fault
      }
      else {
        this.fault = false;
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
    // Serializes a message object of type AttachmentMechanismStatus
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [source]
    bufferOffset = _serializer.uint8(obj.source, buffer, bufferOffset);
    // Serialize message field [sequence]
    bufferOffset = _serializer.uint32(obj.sequence, buffer, bufferOffset);
    // Serialize message field [acknowledged]
    bufferOffset = _serializer.bool(obj.acknowledged, buffer, bufferOffset);
    // Serialize message field [released]
    bufferOffset = _serializer.bool(obj.released, buffer, bufferOffset);
    // Serialize message field [fault]
    bufferOffset = _serializer.bool(obj.fault, buffer, bufferOffset);
    // Serialize message field [reason]
    bufferOffset = _serializer.string(obj.reason, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type AttachmentMechanismStatus
    let len;
    let data = new AttachmentMechanismStatus(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [source]
    data.source = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [sequence]
    data.sequence = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [acknowledged]
    data.acknowledged = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [released]
    data.released = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [fault]
    data.fault = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [reason]
    data.reason = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += _getByteLength(object.reason);
    return length + 12;
  }

  static datatype() {
    // Returns string type for a message object
    return 'px4_wall_ceiling_control/AttachmentMechanismStatus';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'e3810df96eca58328a3f90eb36d48cc3';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Header header
    
    uint8 source
    uint32 sequence
    bool acknowledged
    bool released
    bool fault
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
    const resolved = new AttachmentMechanismStatus(null);
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

    if (msg.sequence !== undefined) {
      resolved.sequence = msg.sequence;
    }
    else {
      resolved.sequence = 0
    }

    if (msg.acknowledged !== undefined) {
      resolved.acknowledged = msg.acknowledged;
    }
    else {
      resolved.acknowledged = false
    }

    if (msg.released !== undefined) {
      resolved.released = msg.released;
    }
    else {
      resolved.released = false
    }

    if (msg.fault !== undefined) {
      resolved.fault = msg.fault;
    }
    else {
      resolved.fault = false
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

module.exports = AttachmentMechanismStatus;
