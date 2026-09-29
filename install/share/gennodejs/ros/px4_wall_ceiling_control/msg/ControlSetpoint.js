// Auto-generated. Do not edit!

// (in-package px4_wall_ceiling_control.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let mavros_msgs = _finder('mavros_msgs');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class ControlSetpoint {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.source = null;
      this.kind = null;
      this.valid = null;
      this.local = null;
      this.attitude = null;
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
      if (initObj.hasOwnProperty('valid')) {
        this.valid = initObj.valid
      }
      else {
        this.valid = false;
      }
      if (initObj.hasOwnProperty('local')) {
        this.local = initObj.local
      }
      else {
        this.local = new mavros_msgs.msg.PositionTarget();
      }
      if (initObj.hasOwnProperty('attitude')) {
        this.attitude = initObj.attitude
      }
      else {
        this.attitude = new mavros_msgs.msg.AttitudeTarget();
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
    // Serializes a message object of type ControlSetpoint
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [source]
    bufferOffset = _serializer.uint8(obj.source, buffer, bufferOffset);
    // Serialize message field [kind]
    bufferOffset = _serializer.uint8(obj.kind, buffer, bufferOffset);
    // Serialize message field [valid]
    bufferOffset = _serializer.bool(obj.valid, buffer, bufferOffset);
    // Serialize message field [local]
    bufferOffset = mavros_msgs.msg.PositionTarget.serialize(obj.local, buffer, bufferOffset);
    // Serialize message field [attitude]
    bufferOffset = mavros_msgs.msg.AttitudeTarget.serialize(obj.attitude, buffer, bufferOffset);
    // Serialize message field [reason]
    bufferOffset = _serializer.string(obj.reason, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ControlSetpoint
    let len;
    let data = new ControlSetpoint(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [source]
    data.source = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [kind]
    data.kind = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [valid]
    data.valid = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [local]
    data.local = mavros_msgs.msg.PositionTarget.deserialize(buffer, bufferOffset);
    // Deserialize message field [attitude]
    data.attitude = mavros_msgs.msg.AttitudeTarget.deserialize(buffer, bufferOffset);
    // Deserialize message field [reason]
    data.reason = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += mavros_msgs.msg.PositionTarget.getMessageSize(object.local);
    length += mavros_msgs.msg.AttitudeTarget.getMessageSize(object.attitude);
    length += _getByteLength(object.reason);
    return length + 7;
  }

  static datatype() {
    // Returns string type for a message object
    return 'px4_wall_ceiling_control/ControlSetpoint';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '905a0ba96daedb050b5d2c986dd2f262';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Header header
    
    uint8 SOURCE_NONE=0
    uint8 SOURCE_CEILING=1
    uint8 SOURCE_WALL=2
    
    uint8 KIND_NONE=0
    uint8 KIND_LOCAL=1
    uint8 KIND_ATTITUDE=2
    
    uint8 source
    uint8 kind
    bool valid
    mavros_msgs/PositionTarget local
    mavros_msgs/AttitudeTarget attitude
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
    MSG: mavros_msgs/PositionTarget
    # Message for SET_POSITION_TARGET_LOCAL_NED
    #
    # Some complex system requires all feautures that mavlink
    # message provide. See issue #402.
    
    std_msgs/Header header
    
    uint8 coordinate_frame
    uint8 FRAME_LOCAL_NED = 1
    uint8 FRAME_LOCAL_OFFSET_NED = 7
    uint8 FRAME_BODY_NED = 8
    uint8 FRAME_BODY_OFFSET_NED = 9
    
    uint16 type_mask
    uint16 IGNORE_PX = 1	# Position ignore flags
    uint16 IGNORE_PY = 2
    uint16 IGNORE_PZ = 4
    uint16 IGNORE_VX = 8	# Velocity vector ignore flags
    uint16 IGNORE_VY = 16
    uint16 IGNORE_VZ = 32
    uint16 IGNORE_AFX = 64	# Acceleration/Force vector ignore flags
    uint16 IGNORE_AFY = 128
    uint16 IGNORE_AFZ = 256
    uint16 FORCE = 512	# Force in af vector flag
    uint16 IGNORE_YAW = 1024
    uint16 IGNORE_YAW_RATE = 2048
    
    geometry_msgs/Point position
    geometry_msgs/Vector3 velocity
    geometry_msgs/Vector3 acceleration_or_force
    float32 yaw
    float32 yaw_rate
    
    ================================================================================
    MSG: geometry_msgs/Point
    # This contains the position of a point in free space
    float64 x
    float64 y
    float64 z
    
    ================================================================================
    MSG: geometry_msgs/Vector3
    # This represents a vector in free space. 
    # It is only meant to represent a direction. Therefore, it does not
    # make sense to apply a translation to it (e.g., when applying a 
    # generic rigid transformation to a Vector3, tf2 will only apply the
    # rotation). If you want your data to be translatable too, use the
    # geometry_msgs/Point message instead.
    
    float64 x
    float64 y
    float64 z
    ================================================================================
    MSG: mavros_msgs/AttitudeTarget
    # Message for SET_ATTITUDE_TARGET
    #
    # Some complex system requires all feautures that mavlink
    # message provide. See issue #402, #418.
    
    std_msgs/Header header
    
    uint8 type_mask
    uint8 IGNORE_ROLL_RATE = 1	# body_rate.x
    uint8 IGNORE_PITCH_RATE = 2	# body_rate.y
    uint8 IGNORE_YAW_RATE = 4	# body_rate.z
    uint8 IGNORE_THRUST = 64
    uint8 IGNORE_ATTITUDE = 128	# orientation field
    
    geometry_msgs/Quaternion orientation
    geometry_msgs/Vector3 body_rate
    float32 thrust
    
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
    const resolved = new ControlSetpoint(null);
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

    if (msg.valid !== undefined) {
      resolved.valid = msg.valid;
    }
    else {
      resolved.valid = false
    }

    if (msg.local !== undefined) {
      resolved.local = mavros_msgs.msg.PositionTarget.Resolve(msg.local)
    }
    else {
      resolved.local = new mavros_msgs.msg.PositionTarget()
    }

    if (msg.attitude !== undefined) {
      resolved.attitude = mavros_msgs.msg.AttitudeTarget.Resolve(msg.attitude)
    }
    else {
      resolved.attitude = new mavros_msgs.msg.AttitudeTarget()
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
ControlSetpoint.Constants = {
  SOURCE_NONE: 0,
  SOURCE_CEILING: 1,
  SOURCE_WALL: 2,
  KIND_NONE: 0,
  KIND_LOCAL: 1,
  KIND_ATTITUDE: 2,
}

module.exports = ControlSetpoint;
