; Auto-generated. Do not edit!


(cl:in-package px4_wall_ceiling_control-msg)


;//! \htmlinclude SupervisorState.msg.html

(cl:defclass <SupervisorState> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (state
    :reader state
    :initarg :state
    :type cl:fixnum
    :initform 0)
   (active_mode
    :reader active_mode
    :initarg :active_mode
    :type cl:fixnum
    :initform 0)
   (setpoint_stream_allowed
    :reader setpoint_stream_allowed
    :initarg :setpoint_stream_allowed
    :type cl:boolean
    :initform cl:nil)
   (motion_allowed
    :reader motion_allowed
    :initarg :motion_allowed
    :type cl:boolean
    :initform cl:nil)
   (mavros_connected
    :reader mavros_connected
    :initarg :mavros_connected
    :type cl:boolean
    :initform cl:nil)
   (armed
    :reader armed
    :initarg :armed
    :type cl:boolean
    :initform cl:nil)
   (offboard
    :reader offboard
    :initarg :offboard
    :type cl:boolean
    :initform cl:nil)
   (operator_fresh
    :reader operator_fresh
    :initarg :operator_fresh
    :type cl:boolean
    :initform cl:nil)
   (odom_fresh
    :reader odom_fresh
    :initarg :odom_fresh
    :type cl:boolean
    :initform cl:nil)
   (ceiling_range_fresh
    :reader ceiling_range_fresh
    :initarg :ceiling_range_fresh
    :type cl:boolean
    :initform cl:nil)
   (wall_range_fresh
    :reader wall_range_fresh
    :initarg :wall_range_fresh
    :type cl:boolean
    :initform cl:nil)
   (fault_mask
    :reader fault_mask
    :initarg :fault_mask
    :type cl:integer
    :initform 0)
   (reason
    :reader reason
    :initarg :reason
    :type cl:string
    :initform ""))
)

(cl:defclass SupervisorState (<SupervisorState>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SupervisorState>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SupervisorState)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name px4_wall_ceiling_control-msg:<SupervisorState> is deprecated: use px4_wall_ceiling_control-msg:SupervisorState instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <SupervisorState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader px4_wall_ceiling_control-msg:header-val is deprecated.  Use px4_wall_ceiling_control-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'state-val :lambda-list '(m))
(cl:defmethod state-val ((m <SupervisorState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader px4_wall_ceiling_control-msg:state-val is deprecated.  Use px4_wall_ceiling_control-msg:state instead.")
  (state m))

(cl:ensure-generic-function 'active_mode-val :lambda-list '(m))
(cl:defmethod active_mode-val ((m <SupervisorState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader px4_wall_ceiling_control-msg:active_mode-val is deprecated.  Use px4_wall_ceiling_control-msg:active_mode instead.")
  (active_mode m))

(cl:ensure-generic-function 'setpoint_stream_allowed-val :lambda-list '(m))
(cl:defmethod setpoint_stream_allowed-val ((m <SupervisorState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader px4_wall_ceiling_control-msg:setpoint_stream_allowed-val is deprecated.  Use px4_wall_ceiling_control-msg:setpoint_stream_allowed instead.")
  (setpoint_stream_allowed m))

(cl:ensure-generic-function 'motion_allowed-val :lambda-list '(m))
(cl:defmethod motion_allowed-val ((m <SupervisorState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader px4_wall_ceiling_control-msg:motion_allowed-val is deprecated.  Use px4_wall_ceiling_control-msg:motion_allowed instead.")
  (motion_allowed m))

(cl:ensure-generic-function 'mavros_connected-val :lambda-list '(m))
(cl:defmethod mavros_connected-val ((m <SupervisorState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader px4_wall_ceiling_control-msg:mavros_connected-val is deprecated.  Use px4_wall_ceiling_control-msg:mavros_connected instead.")
  (mavros_connected m))

(cl:ensure-generic-function 'armed-val :lambda-list '(m))
(cl:defmethod armed-val ((m <SupervisorState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader px4_wall_ceiling_control-msg:armed-val is deprecated.  Use px4_wall_ceiling_control-msg:armed instead.")
  (armed m))

(cl:ensure-generic-function 'offboard-val :lambda-list '(m))
(cl:defmethod offboard-val ((m <SupervisorState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader px4_wall_ceiling_control-msg:offboard-val is deprecated.  Use px4_wall_ceiling_control-msg:offboard instead.")
  (offboard m))

(cl:ensure-generic-function 'operator_fresh-val :lambda-list '(m))
(cl:defmethod operator_fresh-val ((m <SupervisorState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader px4_wall_ceiling_control-msg:operator_fresh-val is deprecated.  Use px4_wall_ceiling_control-msg:operator_fresh instead.")
  (operator_fresh m))

(cl:ensure-generic-function 'odom_fresh-val :lambda-list '(m))
(cl:defmethod odom_fresh-val ((m <SupervisorState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader px4_wall_ceiling_control-msg:odom_fresh-val is deprecated.  Use px4_wall_ceiling_control-msg:odom_fresh instead.")
  (odom_fresh m))

(cl:ensure-generic-function 'ceiling_range_fresh-val :lambda-list '(m))
(cl:defmethod ceiling_range_fresh-val ((m <SupervisorState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader px4_wall_ceiling_control-msg:ceiling_range_fresh-val is deprecated.  Use px4_wall_ceiling_control-msg:ceiling_range_fresh instead.")
  (ceiling_range_fresh m))

(cl:ensure-generic-function 'wall_range_fresh-val :lambda-list '(m))
(cl:defmethod wall_range_fresh-val ((m <SupervisorState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader px4_wall_ceiling_control-msg:wall_range_fresh-val is deprecated.  Use px4_wall_ceiling_control-msg:wall_range_fresh instead.")
  (wall_range_fresh m))

(cl:ensure-generic-function 'fault_mask-val :lambda-list '(m))
(cl:defmethod fault_mask-val ((m <SupervisorState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader px4_wall_ceiling_control-msg:fault_mask-val is deprecated.  Use px4_wall_ceiling_control-msg:fault_mask instead.")
  (fault_mask m))

(cl:ensure-generic-function 'reason-val :lambda-list '(m))
(cl:defmethod reason-val ((m <SupervisorState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader px4_wall_ceiling_control-msg:reason-val is deprecated.  Use px4_wall_ceiling_control-msg:reason instead.")
  (reason m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<SupervisorState>)))
    "Constants for message type '<SupervisorState>"
  '((:STATE_WAIT_LINK . 0)
    (:STATE_MANUAL . 1)
    (:STATE_STANDBY . 2)
    (:STATE_PRESTREAM . 3)
    (:STATE_CEILING_ACTIVE . 4)
    (:STATE_WALL_ACTIVE . 5)
    (:STATE_FAULT . 6)
    (:MODE_NONE . 0)
    (:MODE_CEILING . 1)
    (:MODE_WALL . 2)
    (:FAULT_NONE . 0)
    (:FAULT_MAVROS . 1)
    (:FAULT_OPERATOR . 2)
    (:FAULT_ODOMETRY . 4)
    (:FAULT_CEILING_RANGE . 8)
    (:FAULT_WALL_RANGE . 16)
    (:FAULT_UNKNOWN_MODE . 32))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'SupervisorState)))
    "Constants for message type 'SupervisorState"
  '((:STATE_WAIT_LINK . 0)
    (:STATE_MANUAL . 1)
    (:STATE_STANDBY . 2)
    (:STATE_PRESTREAM . 3)
    (:STATE_CEILING_ACTIVE . 4)
    (:STATE_WALL_ACTIVE . 5)
    (:STATE_FAULT . 6)
    (:MODE_NONE . 0)
    (:MODE_CEILING . 1)
    (:MODE_WALL . 2)
    (:FAULT_NONE . 0)
    (:FAULT_MAVROS . 1)
    (:FAULT_OPERATOR . 2)
    (:FAULT_ODOMETRY . 4)
    (:FAULT_CEILING_RANGE . 8)
    (:FAULT_WALL_RANGE . 16)
    (:FAULT_UNKNOWN_MODE . 32))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SupervisorState>) ostream)
  "Serializes a message object of type '<SupervisorState>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'state)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'active_mode)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'setpoint_stream_allowed) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'motion_allowed) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'mavros_connected) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'armed) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'offboard) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'operator_fresh) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'odom_fresh) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'ceiling_range_fresh) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'wall_range_fresh) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'fault_mask)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'fault_mask)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'fault_mask)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'fault_mask)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'reason))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'reason))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SupervisorState>) istream)
  "Deserializes a message object of type '<SupervisorState>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'state)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'active_mode)) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'setpoint_stream_allowed) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'motion_allowed) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'mavros_connected) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'armed) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'offboard) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'operator_fresh) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'odom_fresh) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'ceiling_range_fresh) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'wall_range_fresh) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'fault_mask)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'fault_mask)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'fault_mask)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'fault_mask)) (cl:read-byte istream))
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SupervisorState>)))
  "Returns string type for a message object of type '<SupervisorState>"
  "px4_wall_ceiling_control/SupervisorState")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SupervisorState)))
  "Returns string type for a message object of type 'SupervisorState"
  "px4_wall_ceiling_control/SupervisorState")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SupervisorState>)))
  "Returns md5sum for a message object of type '<SupervisorState>"
  "1ec3406c7c8a31f63daae31442cabb75")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SupervisorState)))
  "Returns md5sum for a message object of type 'SupervisorState"
  "1ec3406c7c8a31f63daae31442cabb75")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SupervisorState>)))
  "Returns full string definition for message of type '<SupervisorState>"
  (cl:format cl:nil "std_msgs/Header header~%~%uint8 STATE_WAIT_LINK=0~%uint8 STATE_MANUAL=1~%uint8 STATE_STANDBY=2~%uint8 STATE_PRESTREAM=3~%uint8 STATE_CEILING_ACTIVE=4~%uint8 STATE_WALL_ACTIVE=5~%uint8 STATE_FAULT=6~%~%uint8 MODE_NONE=0~%uint8 MODE_CEILING=1~%uint8 MODE_WALL=2~%~%uint32 FAULT_NONE=0~%uint32 FAULT_MAVROS=1~%uint32 FAULT_OPERATOR=2~%uint32 FAULT_ODOMETRY=4~%uint32 FAULT_CEILING_RANGE=8~%uint32 FAULT_WALL_RANGE=16~%uint32 FAULT_UNKNOWN_MODE=32~%~%uint8 state~%uint8 active_mode~%bool setpoint_stream_allowed~%bool motion_allowed~%bool mavros_connected~%bool armed~%bool offboard~%bool operator_fresh~%bool odom_fresh~%bool ceiling_range_fresh~%bool wall_range_fresh~%uint32 fault_mask~%string reason~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SupervisorState)))
  "Returns full string definition for message of type 'SupervisorState"
  (cl:format cl:nil "std_msgs/Header header~%~%uint8 STATE_WAIT_LINK=0~%uint8 STATE_MANUAL=1~%uint8 STATE_STANDBY=2~%uint8 STATE_PRESTREAM=3~%uint8 STATE_CEILING_ACTIVE=4~%uint8 STATE_WALL_ACTIVE=5~%uint8 STATE_FAULT=6~%~%uint8 MODE_NONE=0~%uint8 MODE_CEILING=1~%uint8 MODE_WALL=2~%~%uint32 FAULT_NONE=0~%uint32 FAULT_MAVROS=1~%uint32 FAULT_OPERATOR=2~%uint32 FAULT_ODOMETRY=4~%uint32 FAULT_CEILING_RANGE=8~%uint32 FAULT_WALL_RANGE=16~%uint32 FAULT_UNKNOWN_MODE=32~%~%uint8 state~%uint8 active_mode~%bool setpoint_stream_allowed~%bool motion_allowed~%bool mavros_connected~%bool armed~%bool offboard~%bool operator_fresh~%bool odom_fresh~%bool ceiling_range_fresh~%bool wall_range_fresh~%uint32 fault_mask~%string reason~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SupervisorState>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
     4
     4 (cl:length (cl:slot-value msg 'reason))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SupervisorState>))
  "Converts a ROS message object to a list"
  (cl:list 'SupervisorState
    (cl:cons ':header (header msg))
    (cl:cons ':state (state msg))
    (cl:cons ':active_mode (active_mode msg))
    (cl:cons ':setpoint_stream_allowed (setpoint_stream_allowed msg))
    (cl:cons ':motion_allowed (motion_allowed msg))
    (cl:cons ':mavros_connected (mavros_connected msg))
    (cl:cons ':armed (armed msg))
    (cl:cons ':offboard (offboard msg))
    (cl:cons ':operator_fresh (operator_fresh msg))
    (cl:cons ':odom_fresh (odom_fresh msg))
    (cl:cons ':ceiling_range_fresh (ceiling_range_fresh msg))
    (cl:cons ':wall_range_fresh (wall_range_fresh msg))
    (cl:cons ':fault_mask (fault_mask msg))
    (cl:cons ':reason (reason msg))
))
