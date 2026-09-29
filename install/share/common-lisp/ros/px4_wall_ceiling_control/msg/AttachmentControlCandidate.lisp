; Auto-generated. Do not edit!


(cl:in-package px4_wall_ceiling_control-msg)


;//! \htmlinclude AttachmentControlCandidate.msg.html

(cl:defclass <AttachmentControlCandidate> (roslisp-msg-protocol:ros-message)
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
   (acquire_request
    :reader acquire_request
    :initarg :acquire_request
    :type cl:boolean
    :initform cl:nil)
   (keep_ownership
    :reader keep_ownership
    :initarg :keep_ownership
    :type cl:boolean
    :initform cl:nil)
   (valid
    :reader valid
    :initarg :valid
    :type cl:boolean
    :initform cl:nil)
   (velocity_z_enu_mps
    :reader velocity_z_enu_mps
    :initarg :velocity_z_enu_mps
    :type cl:float
    :initform 0.0)
   (attitude_setpoint
    :reader attitude_setpoint
    :initarg :attitude_setpoint
    :type geometry_msgs-msg:Quaternion
    :initform (cl:make-instance 'geometry_msgs-msg:Quaternion))
   (thrust_normalized
    :reader thrust_normalized
    :initarg :thrust_normalized
    :type cl:float
    :initform 0.0)
   (reason
    :reader reason
    :initarg :reason
    :type cl:string
    :initform ""))
)

(cl:defclass AttachmentControlCandidate (<AttachmentControlCandidate>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <AttachmentControlCandidate>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'AttachmentControlCandidate)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name px4_wall_ceiling_control-msg:<AttachmentControlCandidate> is deprecated: use px4_wall_ceiling_control-msg:AttachmentControlCandidate instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <AttachmentControlCandidate>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader px4_wall_ceiling_control-msg:header-val is deprecated.  Use px4_wall_ceiling_control-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'source-val :lambda-list '(m))
(cl:defmethod source-val ((m <AttachmentControlCandidate>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader px4_wall_ceiling_control-msg:source-val is deprecated.  Use px4_wall_ceiling_control-msg:source instead.")
  (source m))

(cl:ensure-generic-function 'kind-val :lambda-list '(m))
(cl:defmethod kind-val ((m <AttachmentControlCandidate>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader px4_wall_ceiling_control-msg:kind-val is deprecated.  Use px4_wall_ceiling_control-msg:kind instead.")
  (kind m))

(cl:ensure-generic-function 'acquire_request-val :lambda-list '(m))
(cl:defmethod acquire_request-val ((m <AttachmentControlCandidate>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader px4_wall_ceiling_control-msg:acquire_request-val is deprecated.  Use px4_wall_ceiling_control-msg:acquire_request instead.")
  (acquire_request m))

(cl:ensure-generic-function 'keep_ownership-val :lambda-list '(m))
(cl:defmethod keep_ownership-val ((m <AttachmentControlCandidate>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader px4_wall_ceiling_control-msg:keep_ownership-val is deprecated.  Use px4_wall_ceiling_control-msg:keep_ownership instead.")
  (keep_ownership m))

(cl:ensure-generic-function 'valid-val :lambda-list '(m))
(cl:defmethod valid-val ((m <AttachmentControlCandidate>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader px4_wall_ceiling_control-msg:valid-val is deprecated.  Use px4_wall_ceiling_control-msg:valid instead.")
  (valid m))

(cl:ensure-generic-function 'velocity_z_enu_mps-val :lambda-list '(m))
(cl:defmethod velocity_z_enu_mps-val ((m <AttachmentControlCandidate>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader px4_wall_ceiling_control-msg:velocity_z_enu_mps-val is deprecated.  Use px4_wall_ceiling_control-msg:velocity_z_enu_mps instead.")
  (velocity_z_enu_mps m))

(cl:ensure-generic-function 'attitude_setpoint-val :lambda-list '(m))
(cl:defmethod attitude_setpoint-val ((m <AttachmentControlCandidate>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader px4_wall_ceiling_control-msg:attitude_setpoint-val is deprecated.  Use px4_wall_ceiling_control-msg:attitude_setpoint instead.")
  (attitude_setpoint m))

(cl:ensure-generic-function 'thrust_normalized-val :lambda-list '(m))
(cl:defmethod thrust_normalized-val ((m <AttachmentControlCandidate>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader px4_wall_ceiling_control-msg:thrust_normalized-val is deprecated.  Use px4_wall_ceiling_control-msg:thrust_normalized instead.")
  (thrust_normalized m))

(cl:ensure-generic-function 'reason-val :lambda-list '(m))
(cl:defmethod reason-val ((m <AttachmentControlCandidate>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader px4_wall_ceiling_control-msg:reason-val is deprecated.  Use px4_wall_ceiling_control-msg:reason instead.")
  (reason m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<AttachmentControlCandidate>)))
    "Constants for message type '<AttachmentControlCandidate>"
  '((:SOURCE_NONE . 0)
    (:SOURCE_CEILING . 1)
    (:SOURCE_WALL . 2)
    (:KIND_NONE . 0)
    (:KIND_VERTICAL_VELOCITY . 1)
    (:KIND_ATTITUDE_THRUST . 2))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'AttachmentControlCandidate)))
    "Constants for message type 'AttachmentControlCandidate"
  '((:SOURCE_NONE . 0)
    (:SOURCE_CEILING . 1)
    (:SOURCE_WALL . 2)
    (:KIND_NONE . 0)
    (:KIND_VERTICAL_VELOCITY . 1)
    (:KIND_ATTITUDE_THRUST . 2))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <AttachmentControlCandidate>) ostream)
  "Serializes a message object of type '<AttachmentControlCandidate>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'source)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'kind)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'acquire_request) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'keep_ownership) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'valid) 1 0)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'velocity_z_enu_mps))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'attitude_setpoint) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'thrust_normalized))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'reason))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'reason))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <AttachmentControlCandidate>) istream)
  "Deserializes a message object of type '<AttachmentControlCandidate>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'source)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'kind)) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'acquire_request) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'keep_ownership) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'valid) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'velocity_z_enu_mps) (roslisp-utils:decode-single-float-bits bits)))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'attitude_setpoint) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'thrust_normalized) (roslisp-utils:decode-single-float-bits bits)))
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<AttachmentControlCandidate>)))
  "Returns string type for a message object of type '<AttachmentControlCandidate>"
  "px4_wall_ceiling_control/AttachmentControlCandidate")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'AttachmentControlCandidate)))
  "Returns string type for a message object of type 'AttachmentControlCandidate"
  "px4_wall_ceiling_control/AttachmentControlCandidate")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<AttachmentControlCandidate>)))
  "Returns md5sum for a message object of type '<AttachmentControlCandidate>"
  "a60e02517b779e0ed9645ce45b1715de")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'AttachmentControlCandidate)))
  "Returns md5sum for a message object of type 'AttachmentControlCandidate"
  "a60e02517b779e0ed9645ce45b1715de")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<AttachmentControlCandidate>)))
  "Returns full string definition for message of type '<AttachmentControlCandidate>"
  (cl:format cl:nil "std_msgs/Header header~%~%uint8 SOURCE_NONE=0~%uint8 SOURCE_CEILING=1~%uint8 SOURCE_WALL=2~%~%uint8 KIND_NONE=0~%uint8 KIND_VERTICAL_VELOCITY=1~%uint8 KIND_ATTITUDE_THRUST=2~%~%uint8 source~%uint8 kind~%bool acquire_request~%bool keep_ownership~%bool valid~%~%# ROS-side ENU world convention.~%float32 velocity_z_enu_mps~%# ROS-side ENU/FLU attitude convention; MAVROS performs PX4 conversion.~%geometry_msgs/Quaternion attitude_setpoint~%# Positive normalized collective thrust expected by mavros_msgs/AttitudeTarget.~%float32 thrust_normalized~%string reason~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'AttachmentControlCandidate)))
  "Returns full string definition for message of type 'AttachmentControlCandidate"
  (cl:format cl:nil "std_msgs/Header header~%~%uint8 SOURCE_NONE=0~%uint8 SOURCE_CEILING=1~%uint8 SOURCE_WALL=2~%~%uint8 KIND_NONE=0~%uint8 KIND_VERTICAL_VELOCITY=1~%uint8 KIND_ATTITUDE_THRUST=2~%~%uint8 source~%uint8 kind~%bool acquire_request~%bool keep_ownership~%bool valid~%~%# ROS-side ENU world convention.~%float32 velocity_z_enu_mps~%# ROS-side ENU/FLU attitude convention; MAVROS performs PX4 conversion.~%geometry_msgs/Quaternion attitude_setpoint~%# Positive normalized collective thrust expected by mavros_msgs/AttitudeTarget.~%float32 thrust_normalized~%string reason~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <AttachmentControlCandidate>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     1
     1
     1
     1
     1
     4
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'attitude_setpoint))
     4
     4 (cl:length (cl:slot-value msg 'reason))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <AttachmentControlCandidate>))
  "Converts a ROS message object to a list"
  (cl:list 'AttachmentControlCandidate
    (cl:cons ':header (header msg))
    (cl:cons ':source (source msg))
    (cl:cons ':kind (kind msg))
    (cl:cons ':acquire_request (acquire_request msg))
    (cl:cons ':keep_ownership (keep_ownership msg))
    (cl:cons ':valid (valid msg))
    (cl:cons ':velocity_z_enu_mps (velocity_z_enu_mps msg))
    (cl:cons ':attitude_setpoint (attitude_setpoint msg))
    (cl:cons ':thrust_normalized (thrust_normalized msg))
    (cl:cons ':reason (reason msg))
))
