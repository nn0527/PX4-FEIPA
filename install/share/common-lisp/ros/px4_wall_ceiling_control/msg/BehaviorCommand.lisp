; Auto-generated. Do not edit!


(cl:in-package px4_wall_ceiling_control-msg)


;//! \htmlinclude BehaviorCommand.msg.html

(cl:defclass <BehaviorCommand> (roslisp-msg-protocol:ros-message)
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
   (state
    :reader state
    :initarg :state
    :type cl:fixnum
    :initform 0)
   (active
    :reader active
    :initarg :active
    :type cl:boolean
    :initform cl:nil)
   (target_distance_m
    :reader target_distance_m
    :initarg :target_distance_m
    :type cl:float
    :initform 0.0)
   (max_speed_mps
    :reader max_speed_mps
    :initarg :max_speed_mps
    :type cl:float
    :initform 0.0)
   (reason
    :reader reason
    :initarg :reason
    :type cl:string
    :initform ""))
)

(cl:defclass BehaviorCommand (<BehaviorCommand>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <BehaviorCommand>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'BehaviorCommand)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name px4_wall_ceiling_control-msg:<BehaviorCommand> is deprecated: use px4_wall_ceiling_control-msg:BehaviorCommand instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <BehaviorCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader px4_wall_ceiling_control-msg:header-val is deprecated.  Use px4_wall_ceiling_control-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'source-val :lambda-list '(m))
(cl:defmethod source-val ((m <BehaviorCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader px4_wall_ceiling_control-msg:source-val is deprecated.  Use px4_wall_ceiling_control-msg:source instead.")
  (source m))

(cl:ensure-generic-function 'state-val :lambda-list '(m))
(cl:defmethod state-val ((m <BehaviorCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader px4_wall_ceiling_control-msg:state-val is deprecated.  Use px4_wall_ceiling_control-msg:state instead.")
  (state m))

(cl:ensure-generic-function 'active-val :lambda-list '(m))
(cl:defmethod active-val ((m <BehaviorCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader px4_wall_ceiling_control-msg:active-val is deprecated.  Use px4_wall_ceiling_control-msg:active instead.")
  (active m))

(cl:ensure-generic-function 'target_distance_m-val :lambda-list '(m))
(cl:defmethod target_distance_m-val ((m <BehaviorCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader px4_wall_ceiling_control-msg:target_distance_m-val is deprecated.  Use px4_wall_ceiling_control-msg:target_distance_m instead.")
  (target_distance_m m))

(cl:ensure-generic-function 'max_speed_mps-val :lambda-list '(m))
(cl:defmethod max_speed_mps-val ((m <BehaviorCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader px4_wall_ceiling_control-msg:max_speed_mps-val is deprecated.  Use px4_wall_ceiling_control-msg:max_speed_mps instead.")
  (max_speed_mps m))

(cl:ensure-generic-function 'reason-val :lambda-list '(m))
(cl:defmethod reason-val ((m <BehaviorCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader px4_wall_ceiling_control-msg:reason-val is deprecated.  Use px4_wall_ceiling_control-msg:reason instead.")
  (reason m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<BehaviorCommand>)))
    "Constants for message type '<BehaviorCommand>"
  '((:SOURCE_NONE . 0)
    (:SOURCE_CEILING . 1)
    (:SOURCE_WALL . 2)
    (:STATE_DISABLED . 0)
    (:STATE_APPROACH . 1)
    (:STATE_TRACK . 2)
    (:STATE_RETREAT . 3)
    (:STATE_FAULT . 4))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'BehaviorCommand)))
    "Constants for message type 'BehaviorCommand"
  '((:SOURCE_NONE . 0)
    (:SOURCE_CEILING . 1)
    (:SOURCE_WALL . 2)
    (:STATE_DISABLED . 0)
    (:STATE_APPROACH . 1)
    (:STATE_TRACK . 2)
    (:STATE_RETREAT . 3)
    (:STATE_FAULT . 4))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <BehaviorCommand>) ostream)
  "Serializes a message object of type '<BehaviorCommand>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'source)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'state)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'active) 1 0)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'target_distance_m))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'max_speed_mps))))
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <BehaviorCommand>) istream)
  "Deserializes a message object of type '<BehaviorCommand>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'source)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'state)) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'active) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'target_distance_m) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'max_speed_mps) (roslisp-utils:decode-single-float-bits bits)))
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<BehaviorCommand>)))
  "Returns string type for a message object of type '<BehaviorCommand>"
  "px4_wall_ceiling_control/BehaviorCommand")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'BehaviorCommand)))
  "Returns string type for a message object of type 'BehaviorCommand"
  "px4_wall_ceiling_control/BehaviorCommand")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<BehaviorCommand>)))
  "Returns md5sum for a message object of type '<BehaviorCommand>"
  "c5b91608717c6c01656752aca3ecaa53")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'BehaviorCommand)))
  "Returns md5sum for a message object of type 'BehaviorCommand"
  "c5b91608717c6c01656752aca3ecaa53")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<BehaviorCommand>)))
  "Returns full string definition for message of type '<BehaviorCommand>"
  (cl:format cl:nil "std_msgs/Header header~%~%uint8 SOURCE_NONE=0~%uint8 SOURCE_CEILING=1~%uint8 SOURCE_WALL=2~%~%uint8 STATE_DISABLED=0~%uint8 STATE_APPROACH=1~%uint8 STATE_TRACK=2~%uint8 STATE_RETREAT=3~%uint8 STATE_FAULT=4~%~%uint8 source~%uint8 state~%bool active~%float32 target_distance_m~%float32 max_speed_mps~%string reason~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'BehaviorCommand)))
  "Returns full string definition for message of type 'BehaviorCommand"
  (cl:format cl:nil "std_msgs/Header header~%~%uint8 SOURCE_NONE=0~%uint8 SOURCE_CEILING=1~%uint8 SOURCE_WALL=2~%~%uint8 STATE_DISABLED=0~%uint8 STATE_APPROACH=1~%uint8 STATE_TRACK=2~%uint8 STATE_RETREAT=3~%uint8 STATE_FAULT=4~%~%uint8 source~%uint8 state~%bool active~%float32 target_distance_m~%float32 max_speed_mps~%string reason~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <BehaviorCommand>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     1
     1
     1
     4
     4
     4 (cl:length (cl:slot-value msg 'reason))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <BehaviorCommand>))
  "Converts a ROS message object to a list"
  (cl:list 'BehaviorCommand
    (cl:cons ':header (header msg))
    (cl:cons ':source (source msg))
    (cl:cons ':state (state msg))
    (cl:cons ':active (active msg))
    (cl:cons ':target_distance_m (target_distance_m msg))
    (cl:cons ':max_speed_mps (max_speed_mps msg))
    (cl:cons ':reason (reason msg))
))
