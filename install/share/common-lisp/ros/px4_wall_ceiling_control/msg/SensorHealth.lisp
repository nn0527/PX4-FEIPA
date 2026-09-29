; Auto-generated. Do not edit!


(cl:in-package px4_wall_ceiling_control-msg)


;//! \htmlinclude SensorHealth.msg.html

(cl:defclass <SensorHealth> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (ceiling_valid
    :reader ceiling_valid
    :initarg :ceiling_valid
    :type cl:boolean
    :initform cl:nil)
   (front_valid
    :reader front_valid
    :initarg :front_valid
    :type cl:boolean
    :initform cl:nil)
   (ceiling_age_s
    :reader ceiling_age_s
    :initarg :ceiling_age_s
    :type cl:float
    :initform 0.0)
   (front_age_s
    :reader front_age_s
    :initarg :front_age_s
    :type cl:float
    :initform 0.0)
   (ceiling_sequence
    :reader ceiling_sequence
    :initarg :ceiling_sequence
    :type cl:integer
    :initform 0)
   (front_sequence
    :reader front_sequence
    :initarg :front_sequence
    :type cl:integer
    :initform 0)
   (ceiling_invalid_count
    :reader ceiling_invalid_count
    :initarg :ceiling_invalid_count
    :type cl:integer
    :initform 0)
   (front_invalid_count
    :reader front_invalid_count
    :initarg :front_invalid_count
    :type cl:integer
    :initform 0))
)

(cl:defclass SensorHealth (<SensorHealth>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SensorHealth>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SensorHealth)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name px4_wall_ceiling_control-msg:<SensorHealth> is deprecated: use px4_wall_ceiling_control-msg:SensorHealth instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <SensorHealth>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader px4_wall_ceiling_control-msg:header-val is deprecated.  Use px4_wall_ceiling_control-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'ceiling_valid-val :lambda-list '(m))
(cl:defmethod ceiling_valid-val ((m <SensorHealth>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader px4_wall_ceiling_control-msg:ceiling_valid-val is deprecated.  Use px4_wall_ceiling_control-msg:ceiling_valid instead.")
  (ceiling_valid m))

(cl:ensure-generic-function 'front_valid-val :lambda-list '(m))
(cl:defmethod front_valid-val ((m <SensorHealth>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader px4_wall_ceiling_control-msg:front_valid-val is deprecated.  Use px4_wall_ceiling_control-msg:front_valid instead.")
  (front_valid m))

(cl:ensure-generic-function 'ceiling_age_s-val :lambda-list '(m))
(cl:defmethod ceiling_age_s-val ((m <SensorHealth>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader px4_wall_ceiling_control-msg:ceiling_age_s-val is deprecated.  Use px4_wall_ceiling_control-msg:ceiling_age_s instead.")
  (ceiling_age_s m))

(cl:ensure-generic-function 'front_age_s-val :lambda-list '(m))
(cl:defmethod front_age_s-val ((m <SensorHealth>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader px4_wall_ceiling_control-msg:front_age_s-val is deprecated.  Use px4_wall_ceiling_control-msg:front_age_s instead.")
  (front_age_s m))

(cl:ensure-generic-function 'ceiling_sequence-val :lambda-list '(m))
(cl:defmethod ceiling_sequence-val ((m <SensorHealth>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader px4_wall_ceiling_control-msg:ceiling_sequence-val is deprecated.  Use px4_wall_ceiling_control-msg:ceiling_sequence instead.")
  (ceiling_sequence m))

(cl:ensure-generic-function 'front_sequence-val :lambda-list '(m))
(cl:defmethod front_sequence-val ((m <SensorHealth>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader px4_wall_ceiling_control-msg:front_sequence-val is deprecated.  Use px4_wall_ceiling_control-msg:front_sequence instead.")
  (front_sequence m))

(cl:ensure-generic-function 'ceiling_invalid_count-val :lambda-list '(m))
(cl:defmethod ceiling_invalid_count-val ((m <SensorHealth>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader px4_wall_ceiling_control-msg:ceiling_invalid_count-val is deprecated.  Use px4_wall_ceiling_control-msg:ceiling_invalid_count instead.")
  (ceiling_invalid_count m))

(cl:ensure-generic-function 'front_invalid_count-val :lambda-list '(m))
(cl:defmethod front_invalid_count-val ((m <SensorHealth>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader px4_wall_ceiling_control-msg:front_invalid_count-val is deprecated.  Use px4_wall_ceiling_control-msg:front_invalid_count instead.")
  (front_invalid_count m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SensorHealth>) ostream)
  "Serializes a message object of type '<SensorHealth>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'ceiling_valid) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'front_valid) 1 0)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'ceiling_age_s))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'front_age_s))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'ceiling_sequence)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'ceiling_sequence)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'ceiling_sequence)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'ceiling_sequence)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'front_sequence)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'front_sequence)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'front_sequence)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'front_sequence)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'ceiling_invalid_count)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'ceiling_invalid_count)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'ceiling_invalid_count)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'ceiling_invalid_count)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'front_invalid_count)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'front_invalid_count)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'front_invalid_count)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'front_invalid_count)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SensorHealth>) istream)
  "Deserializes a message object of type '<SensorHealth>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:slot-value msg 'ceiling_valid) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'front_valid) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'ceiling_age_s) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'front_age_s) (roslisp-utils:decode-single-float-bits bits)))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'ceiling_sequence)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'ceiling_sequence)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'ceiling_sequence)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'ceiling_sequence)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'front_sequence)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'front_sequence)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'front_sequence)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'front_sequence)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'ceiling_invalid_count)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'ceiling_invalid_count)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'ceiling_invalid_count)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'ceiling_invalid_count)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'front_invalid_count)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'front_invalid_count)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'front_invalid_count)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'front_invalid_count)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SensorHealth>)))
  "Returns string type for a message object of type '<SensorHealth>"
  "px4_wall_ceiling_control/SensorHealth")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SensorHealth)))
  "Returns string type for a message object of type 'SensorHealth"
  "px4_wall_ceiling_control/SensorHealth")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SensorHealth>)))
  "Returns md5sum for a message object of type '<SensorHealth>"
  "890f31b2daff364b0bc989c3503517bf")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SensorHealth)))
  "Returns md5sum for a message object of type 'SensorHealth"
  "890f31b2daff364b0bc989c3503517bf")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SensorHealth>)))
  "Returns full string definition for message of type '<SensorHealth>"
  (cl:format cl:nil "std_msgs/Header header~%~%bool ceiling_valid~%bool front_valid~%~%# Age of the last valid sample as measured at this node. Infinity means that~%# no valid sample has been received since startup.~%float32 ceiling_age_s~%float32 front_age_s~%~%# Monotonic counters allow consumers and tests to distinguish real samples~%# from repeated status publications.~%uint32 ceiling_sequence~%uint32 front_sequence~%uint32 ceiling_invalid_count~%uint32 front_invalid_count~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SensorHealth)))
  "Returns full string definition for message of type 'SensorHealth"
  (cl:format cl:nil "std_msgs/Header header~%~%bool ceiling_valid~%bool front_valid~%~%# Age of the last valid sample as measured at this node. Infinity means that~%# no valid sample has been received since startup.~%float32 ceiling_age_s~%float32 front_age_s~%~%# Monotonic counters allow consumers and tests to distinguish real samples~%# from repeated status publications.~%uint32 ceiling_sequence~%uint32 front_sequence~%uint32 ceiling_invalid_count~%uint32 front_invalid_count~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SensorHealth>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     1
     1
     4
     4
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SensorHealth>))
  "Converts a ROS message object to a list"
  (cl:list 'SensorHealth
    (cl:cons ':header (header msg))
    (cl:cons ':ceiling_valid (ceiling_valid msg))
    (cl:cons ':front_valid (front_valid msg))
    (cl:cons ':ceiling_age_s (ceiling_age_s msg))
    (cl:cons ':front_age_s (front_age_s msg))
    (cl:cons ':ceiling_sequence (ceiling_sequence msg))
    (cl:cons ':front_sequence (front_sequence msg))
    (cl:cons ':ceiling_invalid_count (ceiling_invalid_count msg))
    (cl:cons ':front_invalid_count (front_invalid_count msg))
))
