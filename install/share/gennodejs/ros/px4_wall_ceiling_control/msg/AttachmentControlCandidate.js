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

class AttachmentControlCandidate {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.source = null;
      this.kind = null;
      this.acquire_request = null;
      this.keep_ownership = null;
      this.valid = null;
      this.velocity_z_enu_mps = null;
      this.attitude_setpoint = null;
      this.thrust_normalized = null;
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
      if (initObj.hasOwnProperty('kind')) {
        this.kind = initObj.kind
      }
      else {
        this.kind = 0;
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
      if (initObj.hasOwnProperty('valid')) {
        this.valid = initObj.valid
      }
      else {
        this.valid = false;
      }
      if (initObj.hasOwnProperty('velocity_z_enu_mps')) {
        this.velocity_z_enu_mps = initObj.velocity_z_enu_mps
      }
      else {
        this.velocity_z_enu_mps = 0.0;
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
      if (initObj.hasOwnProperty('reason')) {
        this.reason = initObj.reason
      }
      else {
        this.reason = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type AttachmentControlCandidate
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [source]
    bufferOffset = _serializer.uint8(obj.source, buffer, bufferOffset);
    // Serialize message field [kind]
    bufferOffset = _serializer.uint8(obj.kind, buffer, bufferOffset);
    // Serialize message field [acquire_request]
    bufferOffset = _serializer.bool(obj.acquire_request, buffer, bufferOffset);
    // Serialize message field [keep_ownership]
    bufferOffset = _serializer.bool(obj.keep_ownership, buffer, bufferOffset);
    // Serialize message field [valid]
    bufferOffset = _serializer.bool(obj.valid, buffer, bufferOffset);
    // Serialize message field [velocity_z_enu_mps]
    bufferOffset = _serializer.float32(obj.velocity_z_enu_mps, buffer, bufferOffset);
    // Serialize message field [attitude_setpoint]
    bufferOffset = geometry_msgs.msg.Quaternion.serialize(obj.attitude_setpoint, buffer, bufferOffset);
    // Serialize message field [thrust_normalized]
    bufferOffset = _serializer.float32(obj.thrust_normalized, buffer, bufferOffset);
    // Serialize message field [reason]
    bufferOffset = _serializer.string(obj.reason, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type AttachmentControlCandidate
    let len;
    let data = new AttachmentControlCandidate(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [source]
    data.source = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [kind]
    data.kind = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [acquire_request]
    data.acquire_request = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [keep_ownership]
    data.keep_ownership = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [valid]
    data.valid = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [velocity_z_enu_mps]
    data.velocity_z_enu_mps = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [attitude_setpoint]
    data.attitude_setpoint = geometry_msgs.msg.Quaternion.deserialize(buffer, bufferOffset);
    // Deserialize message field [thrust_normalized]
    data.thrust_normalized = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [reason]
    data.reason = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += _getByteLength(object.reason);
    return length + 49;
  }

  static datatype() {
    // Returns string type for a message object
    return 'px4_wall_ceiling_control/AttachmentControlCandidate';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'a60e02517b779e0ed9645ce45b1715de';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Header header
    
    uint8 SOURCE_NONE=0
    uint8 SOURCE_CEILING=1
    uint8 SOURCE_WALL=2
    
    uint8 KIND_NONE=0
    uint8 KIND_VERTICAL_VELOCITY=1
    uint8 KIND_ATTITUDE_THRUST=2
    
    uint8 source
    uint8 kind
    bool acquire_request
    bool keep_ownership
    bool valid
    
    # ROS-side ENU world convention.
    float32 velocity_z_enu_mps
    # ROS-side ENU/FLU attitude convention; MAVROS performs PX4 conversion.
    geometry_msgs/Quaternion attitude_setpoint
    # Positive normalized collective thrust expected by mavros_msgs/AttitudeTarget.
    float32 thrust_normalized
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
    const resolved = new AttachmentControlCandidate(null);
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

    if (msg.kind !== undefined) {
      resolved.kind = msg.kind;
    }
    else {
      resolved.kind = 0
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

    if (msg.valid !== undefined) {
      resolved.valid = msg.valid;
    }
    else {
      resolved.valid = false
    }

    if (msg.velocity_z_enu_mps !== undefined) {
      resolved.velocity_z_enu_mps = msg.velocity_z_enu_mps;
    }
    else {
      resolved.velocity_z_enu_mps = 0.0
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
AttachmentControlCandidate.Constants = {
  SOURCE_NONE: 0,
  SOURCE_CEILING: 1,
  SOURCE_WALL: 2,
  KIND_NONE: 0,
  KIND_VERTICAL_VELOCITY: 1,
  KIND_ATTITUDE_THRUST: 2,
}

module.exports = AttachmentControlCandidate;
