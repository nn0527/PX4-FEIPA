; Auto-generated. Do not edit!


(cl:in-package px4_wall_ceiling_control-msg)


;//! \htmlinclude ControlSetpoint.msg.html

(cl:defclass <ControlSetpoint> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (source
    :reader source
    :initarg :source
    :type cl:fixnum
    :initform 0)
   (kind
    :reader kind
    :initarg :kind
    :type cl:fixnum
    :initform 0)
   (valid
    :reader valid
    :initarg :valid
    :type cl:boolean
    :initform cl:nil)
   (local
    :reader local
    :initarg :local
    :type mavros_msgs-msg:PositionTarget
    :initform (cl:make-instance 'mavros_msgs-msg:PositionTarget))
   (attitude
    :reader attitude
    :initarg :attitude
    :type mavros_msgs-msg:AttitudeTarget
    :initform (cl:make-instance 'mavros_msgs-msg:AttitudeTarget))
   (reason
    :reader reason
    :initarg :reason
    :type cl:string
    :initform ""))
)

(cl:defclass ControlSetpoint (<ControlSetpoint>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ControlSetpoint>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ControlSetpoint)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name px4_wall_ceiling_control-msg:<ControlSetpoint> is deprecated: use px4_wall_ceiling_control-msg:ControlSetpoint instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <ControlSetpoint>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader px4_wall_ceiling_control-msg:header-val is deprecated.  Use px4_wall_ceiling_control-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'source-val :lambda-list '(m))
(cl:defmethod source-val ((m <ControlSetpoint>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader px4_wall_ceiling_control-msg:source-val is deprecated.  Use px4_wall_ceiling_control-msg:source instead.")
  (source m))

(cl:ensure-generic-function 'kind-val :lambda-list '(m))
(cl:defmethod kind-val ((m <ControlSetpoint>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader px4_wall_ceiling_control-msg:kind-val is deprecated.  Use px4_wall_ceiling_control-msg:kind instead.")
  (kind m))

(cl:ensure-generic-function 'valid-val :lambda-list '(m))
(cl:defmethod valid-val ((m <ControlSetpoint>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader px4_wall_ceiling_control-msg:valid-val is deprecated.  Use px4_wall_ceiling_control-msg:valid instead.")
  (valid m))

(cl:ensure-generic-function 'local-val :lambda-list '(m))
(cl:defmethod local-val ((m <ControlSetpoint>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader px4_wall_ceiling_control-msg:local-val is deprecated.  Use px4_wall_ceiling_control-msg:local instead.")
  (local m))

(cl:ensure-generic-function 'attitude-val :lambda-list '(m))
(cl:defmethod attitude-val ((m <ControlSetpoint>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader px4_wall_ceiling_control-msg:attitude-val is deprecated.  Use px4_wall_ceiling_control-msg:attitude instead.")
  (attitude m))

(cl:ensure-generic-function 'reason-val :lambda-list '(m))
(cl:defmethod reason-val ((m <ControlSetpoint>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader px4_wall_ceiling_control-msg:reason-val is deprecated.  Use px4_wall_ceiling_control-msg:reason instead.")
  (reason m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<ControlSetpoint>)))
    "Constants for message type '<ControlSetpoint>"
  '((:SOURCE_NONE . 0)
    (:SOURCE_CEILING . 1)
    (:SOURCE_WALL . 2)
    (:KIND_NONE . 0)
    (:KIND_LOCAL . 1)
    (:KIND_ATTITUDE . 2))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'ControlSetpoint)))
    "Constants for message type 'ControlSetpoint"
  '((:SOURCE_NONE . 0)
    (:SOURCE_CEILING . 1)
    (:SOURCE_WALL . 2)
    (:KIND_NONE . 0)
    (:KIND_LOCAL . 1)
    (:KIND_ATTITUDE . 2))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ControlSetpoint>) ostream)
  "Serializes a message object of type '<ControlSetpoint>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'source)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'kind)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'valid) 1 0)) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'local) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'attitude) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'reason))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'reason))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ControlSetpoint>) istream)
  "Deserializes a message object of type '<ControlSetpoint>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'source)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'kind)) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'valid) (cl:not (cl:zerop (cl:read-byte istream))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'local) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'attitude) istream)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'reason) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'reason) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ControlSetpoint>)))
  "Returns string type for a message object of type '<ControlSetpoint>"
  "px4_wall_ceiling_control/ControlSetpoint")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ControlSetpoint)))
  "Returns string type for a message object of type 'ControlSetpoint"
  "px4_wall_ceiling_control/ControlSetpoint")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ControlSetpoint>)))
  "Returns md5sum for a message object of type '<ControlSetpoint>"
  "905a0ba96daedb050b5d2c986dd2f262")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ControlSetpoint)))
  "Returns md5sum for a message object of type 'ControlSetpoint"
  "905a0ba96daedb050b5d2c986dd2f262")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ControlSetpoint>)))
  "Returns full string definition for message of type '<ControlSetpoint>"
  (cl:format cl:nil "std_msgs/Header header~%~%uint8 SOURCE_NONE=0~%uint8 SOURCE_CEILING=1~%uint8 SOURCE_WALL=2~%~%uint8 KIND_NONE=0~%uint8 KIND_LOCAL=1~%uint8 KIND_ATTITUDE=2~%~%uint8 source~%uint8 kind~%bool valid~%mavros_msgs/PositionTarget local~%mavros_msgs/AttitudeTarget attitude~%string reason~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: mavros_msgs/PositionTarget~%# Message for SET_POSITION_TARGET_LOCAL_NED~%#~%# Some complex system requires all feautures that mavlink~%# message provide. See issue #402.~%~%std_msgs/Header header~%~%uint8 coordinate_frame~%uint8 FRAME_LOCAL_NED = 1~%uint8 FRAME_LOCAL_OFFSET_NED = 7~%uint8 FRAME_BODY_NED = 8~%uint8 FRAME_BODY_OFFSET_NED = 9~%~%uint16 type_mask~%uint16 IGNORE_PX = 1	# Position ignore flags~%uint16 IGNORE_PY = 2~%uint16 IGNORE_PZ = 4~%uint16 IGNORE_VX = 8	# Velocity vector ignore flags~%uint16 IGNORE_VY = 16~%uint16 IGNORE_VZ = 32~%uint16 IGNORE_AFX = 64	# Acceleration/Force vector ignore flags~%uint16 IGNORE_AFY = 128~%uint16 IGNORE_AFZ = 256~%uint16 FORCE = 512	# Force in af vector flag~%uint16 IGNORE_YAW = 1024~%uint16 IGNORE_YAW_RATE = 2048~%~%geometry_msgs/Point position~%geometry_msgs/Vector3 velocity~%geometry_msgs/Vector3 acceleration_or_force~%float32 yaw~%float32 yaw_rate~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: mavros_msgs/AttitudeTarget~%# Message for SET_ATTITUDE_TARGET~%#~%# Some complex system requires all feautures that mavlink~%# message provide. See issue #402, #418.~%~%std_msgs/Header header~%~%uint8 type_mask~%uint8 IGNORE_ROLL_RATE = 1	# body_rate.x~%uint8 IGNORE_PITCH_RATE = 2	# body_rate.y~%uint8 IGNORE_YAW_RATE = 4	# body_rate.z~%uint8 IGNORE_THRUST = 64~%uint8 IGNORE_ATTITUDE = 128	# orientation field~%~%geometry_msgs/Quaternion orientation~%geometry_msgs/Vector3 body_rate~%float32 thrust~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ControlSetpoint)))
  "Returns full string definition for message of type 'ControlSetpoint"
  (cl:format cl:nil "std_msgs/Header header~%~%uint8 SOURCE_NONE=0~%uint8 SOURCE_CEILING=1~%uint8 SOURCE_WALL=2~%~%uint8 KIND_NONE=0~%uint8 KIND_LOCAL=1~%uint8 KIND_ATTITUDE=2~%~%uint8 source~%uint8 kind~%bool valid~%mavros_msgs/PositionTarget local~%mavros_msgs/AttitudeTarget attitude~%string reason~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: mavros_msgs/PositionTarget~%# Message for SET_POSITION_TARGET_LOCAL_NED~%#~%# Some complex system requires all feautures that mavlink~%# message provide. See issue #402.~%~%std_msgs/Header header~%~%uint8 coordinate_frame~%uint8 FRAME_LOCAL_NED = 1~%uint8 FRAME_LOCAL_OFFSET_NED = 7~%uint8 FRAME_BODY_NED = 8~%uint8 FRAME_BODY_OFFSET_NED = 9~%~%uint16 type_mask~%uint16 IGNORE_PX = 1	# Position ignore flags~%uint16 IGNORE_PY = 2~%uint16 IGNORE_PZ = 4~%uint16 IGNORE_VX = 8	# Velocity vector ignore flags~%uint16 IGNORE_VY = 16~%uint16 IGNORE_VZ = 32~%uint16 IGNORE_AFX = 64	# Acceleration/Force vector ignore flags~%uint16 IGNORE_AFY = 128~%uint16 IGNORE_AFZ = 256~%uint16 FORCE = 512	# Force in af vector flag~%uint16 IGNORE_YAW = 1024~%uint16 IGNORE_YAW_RATE = 2048~%~%geometry_msgs/Point position~%geometry_msgs/Vector3 velocity~%geometry_msgs/Vector3 acceleration_or_force~%float32 yaw~%float32 yaw_rate~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: mavros_msgs/AttitudeTarget~%# Message for SET_ATTITUDE_TARGET~%#~%# Some complex system requires all feautures that mavlink~%# message provide. See issue #402, #418.~%~%std_msgs/Header header~%~%uint8 type_mask~%uint8 IGNORE_ROLL_RATE = 1	# body_rate.x~%uint8 IGNORE_PITCH_RATE = 2	# body_rate.y~%uint8 IGNORE_YAW_RATE = 4	# body_rate.z~%uint8 IGNORE_THRUST = 64~%uint8 IGNORE_ATTITUDE = 128	# orientation field~%~%geometry_msgs/Quaternion orientation~%geometry_msgs/Vector3 body_rate~%float32 thrust~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ControlSetpoint>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     1
     1
     1
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'local))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'attitude))
     4 (cl:length (cl:slot-value msg 'reason))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ControlSetpoint>))
  "Converts a ROS message object to a list"
  (cl:list 'ControlSetpoint
    (cl:cons ':header (header msg))
    (cl:cons ':source (source msg))
    (cl:cons ':kind (kind msg))
    (cl:cons ':valid (valid msg))
    (cl:cons ':local (local msg))
    (cl:cons ':attitude (attitude msg))
    (cl:cons ':reason (reason msg))
))
