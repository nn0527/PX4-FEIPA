; Auto-generated. Do not edit!


(cl:in-package px4_wall_ceiling_control-msg)


;//! \htmlinclude WallPerchStatus.msg.html

(cl:defclass <WallPerchStatus> (roslisp-msg-protocol:ros-message)
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
   (state_name
    :reader state_name
    :initarg :state_name
    :type cl:string
    :initform "")
   (enabled
    :reader enabled
    :initarg :enabled
    :type cl:boolean
    :initform cl:nil)
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
   (candidate_valid
    :reader candidate_valid
    :initarg :candidate_valid
    :type cl:boolean
    :initform cl:nil)
   (front_range_fresh
    :reader front_range_fresh
    :initarg :front_range_fresh
    :type cl:boolean
    :initform cl:nil)
   (top_range_fresh
    :reader top_range_fresh
    :initarg :top_range_fresh
    :type cl:boolean
    :initform cl:nil)
   (sensor_health_fresh
    :reader sensor_health_fresh
    :initarg :sensor_health_fresh
    :type cl:boolean
    :initform cl:nil)
   (top_preflight_ready
    :reader top_preflight_ready
    :initarg :top_preflight_ready
    :type cl:boolean
    :initform cl:nil)
   (odom_fresh
    :reader odom_fresh
    :initarg :odom_fresh
    :type cl:boolean
    :initform cl:nil)
   (flight_state_fresh
    :reader flight_state_fresh
    :initarg :flight_state_fresh
    :type cl:boolean
    :initform cl:nil)
   (front_ready
    :reader front_ready
    :initarg :front_ready
    :type cl:boolean
    :initform cl:nil)
   (flip_ready
    :reader flip_ready
    :initarg :flip_ready
    :type cl:boolean
    :initform cl:nil)
   (top_contact_ready
    :reader top_contact_ready
    :initarg :top_contact_ready
    :type cl:boolean
    :initform cl:nil)
   (top_clear_ready
    :reader top_clear_ready
    :initarg :top_clear_ready
    :type cl:boolean
    :initform cl:nil)
   (fault_detected
    :reader fault_detected
    :initarg :fault_detected
    :type cl:boolean
    :initform cl:nil)
   (neutral_setpoint_requested
    :reader neutral_setpoint_requested
    :initarg :neutral_setpoint_requested
    :type cl:boolean
    :initform cl:nil)
   (attitude_setpoint_valid
    :reader attitude_setpoint_valid
    :initarg :attitude_setpoint_valid
    :type cl:boolean
    :initform cl:nil)
   (front_distance_raw_m
    :reader front_distance_raw_m
    :initarg :front_distance_raw_m
    :type cl:float
    :initform 0.0)
   (front_distance_filtered_m
    :reader front_distance_filtered_m
    :initarg :front_distance_filtered_m
    :type cl:float
    :initform 0.0)
   (top_distance_raw_m
    :reader top_distance_raw_m
    :initarg :top_distance_raw_m
    :type cl:float
    :initform 0.0)
   (top_distance_filtered_m
    :reader top_distance_filtered_m
    :initarg :top_distance_filtered_m
    :type cl:float
    :initform 0.0)
   (progress
    :reader progress
    :initarg :progress
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
   (fault_count
    :reader fault_count
    :initarg :fault_count
    :type cl:integer
    :initform 0)
   (reason
    :reader reason
    :initarg :reason
    :type cl:string
    :initform ""))
)

(cl:defclass WallPerchStatus (<WallPerchStatus>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <WallPerchStatus>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'WallPerchStatus)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name px4_wall_ceiling_control-msg:<WallPerchStatus> is deprecated: use px4_wall_ceiling_control-msg:WallPerchStatus instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <WallPerchStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader px4_wall_ceiling_control-msg:header-val is deprecated.  Use px4_wall_ceiling_control-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'state-val :lambda-list '(m))
(cl:defmethod state-val ((m <WallPerchStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader px4_wall_ceiling_control-msg:state-val is deprecated.  Use px4_wall_ceiling_control-msg:state instead.")
  (state m))

(cl:ensure-generic-function 'state_name-val :lambda-list '(m))
(cl:defmethod state_name-val ((m <WallPerchStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader px4_wall_ceiling_control-msg:state_name-val is deprecated.  Use px4_wall_ceiling_control-msg:state_name instead.")
  (state_name m))

(cl:ensure-generic-function 'enabled-val :lambda-list '(m))
(cl:defmethod enabled-val ((m <WallPerchStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader px4_wall_ceiling_control-msg:enabled-val is deprecated.  Use px4_wall_ceiling_control-msg:enabled instead.")
  (enabled m))

(cl:ensure-generic-function 'acquire_request-val :lambda-list '(m))
(cl:defmethod acquire_request-val ((m <WallPerchStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader px4_wall_ceiling_control-msg:acquire_request-val is deprecated.  Use px4_wall_ceiling_control-msg:acquire_request instead.")
  (acquire_request m))

(cl:ensure-generic-function 'keep_ownership-val :lambda-list '(m))
(cl:defmethod keep_ownership-val ((m <WallPerchStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader px4_wall_ceiling_control-msg:keep_ownership-val is deprecated.  Use px4_wall_ceiling_control-msg:keep_ownership instead.")
  (keep_ownership m))

(cl:ensure-generic-function 'candidate_valid-val :lambda-list '(m))
(cl:defmethod candidate_valid-val ((m <WallPerchStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader px4_wall_ceiling_control-msg:candidate_valid-val is deprecated.  Use px4_wall_ceiling_control-msg:candidate_valid instead.")
  (candidate_valid m))

(cl:ensure-generic-function 'front_range_fresh-val :lambda-list '(m))
(cl:defmethod front_range_fresh-val ((m <WallPerchStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader px4_wall_ceiling_control-msg:front_range_fresh-val is deprecated.  Use px4_wall_ceiling_control-msg:front_range_fresh instead.")
  (front_range_fresh m))

(cl:ensure-generic-function 'top_range_fresh-val :lambda-list '(m))
(cl:defmethod top_range_fresh-val ((m <WallPerchStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader px4_wall_ceiling_control-msg:top_range_fresh-val is deprecated.  Use px4_wall_ceiling_control-msg:top_range_fresh instead.")
  (top_range_fresh m))

(cl:ensure-generic-function 'sensor_health_fresh-val :lambda-list '(m))
(cl:defmethod sensor_health_fresh-val ((m <WallPerchStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader px4_wall_ceiling_control-msg:sensor_health_fresh-val is deprecated.  Use px4_wall_ceiling_control-msg:sensor_health_fresh instead.")
  (sensor_health_fresh m))

(cl:ensure-generic-function 'top_preflight_ready-val :lambda-list '(m))
(cl:defmethod top_preflight_ready-val ((m <WallPerchStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader px4_wall_ceiling_control-msg:top_preflight_ready-val is deprecated.  Use px4_wall_ceiling_control-msg:top_preflight_ready instead.")
  (top_preflight_ready m))

(cl:ensure-generic-function 'odom_fresh-val :lambda-list '(m))
(cl:defmethod odom_fresh-val ((m <WallPerchStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader px4_wall_ceiling_control-msg:odom_fresh-val is deprecated.  Use px4_wall_ceiling_control-msg:odom_fresh instead.")
  (odom_fresh m))

(cl:ensure-generic-function 'flight_state_fresh-val :lambda-list '(m))
(cl:defmethod flight_state_fresh-val ((m <WallPerchStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader px4_wall_ceiling_control-msg:flight_state_fresh-val is deprecated.  Use px4_wall_ceiling_control-msg:flight_state_fresh instead.")
  (flight_state_fresh m))

(cl:ensure-generic-function 'front_ready-val :lambda-list '(m))
(cl:defmethod front_ready-val ((m <WallPerchStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader px4_wall_ceiling_control-msg:front_ready-val is deprecated.  Use px4_wall_ceiling_control-msg:front_ready instead.")
  (front_ready m))

(cl:ensure-generic-function 'flip_ready-val :lambda-list '(m))
(cl:defmethod flip_ready-val ((m <WallPerchStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader px4_wall_ceiling_control-msg:flip_ready-val is deprecated.  Use px4_wall_ceiling_control-msg:flip_ready instead.")
  (flip_ready m))

(cl:ensure-generic-function 'top_contact_ready-val :lambda-list '(m))
(cl:defmethod top_contact_ready-val ((m <WallPerchStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader px4_wall_ceiling_control-msg:top_contact_ready-val is deprecated.  Use px4_wall_ceiling_control-msg:top_contact_ready instead.")
  (top_contact_ready m))

(cl:ensure-generic-function 'top_clear_ready-val :lambda-list '(m))
(cl:defmethod top_clear_ready-val ((m <WallPerchStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader px4_wall_ceiling_control-msg:top_clear_ready-val is deprecated.  Use px4_wall_ceiling_control-msg:top_clear_ready instead.")
  (top_clear_ready m))

(cl:ensure-generic-function 'fault_detected-val :lambda-list '(m))
(cl:defmethod fault_detected-val ((m <WallPerchStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader px4_wall_ceiling_control-msg:fault_detected-val is deprecated.  Use px4_wall_ceiling_control-msg:fault_detected instead.")
  (fault_detected m))

(cl:ensure-generic-function 'neutral_setpoint_requested-val :lambda-list '(m))
(cl:defmethod neutral_setpoint_requested-val ((m <WallPerchStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader px4_wall_ceiling_control-msg:neutral_setpoint_requested-val is deprecated.  Use px4_wall_ceiling_control-msg:neutral_setpoint_requested instead.")
  (neutral_setpoint_requested m))

(cl:ensure-generic-function 'attitude_setpoint_valid-val :lambda-list '(m))
(cl:defmethod attitude_setpoint_valid-val ((m <WallPerchStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader px4_wall_ceiling_control-msg:attitude_setpoint_valid-val is deprecated.  Use px4_wall_ceiling_control-msg:attitude_setpoint_valid instead.")
  (attitude_setpoint_valid m))

(cl:ensure-generic-function 'front_distance_raw_m-val :lambda-list '(m))
(cl:defmethod front_distance_raw_m-val ((m <WallPerchStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader px4_wall_ceiling_control-msg:front_distance_raw_m-val is deprecated.  Use px4_wall_ceiling_control-msg:front_distance_raw_m instead.")
  (front_distance_raw_m m))

(cl:ensure-generic-function 'front_distance_filtered_m-val :lambda-list '(m))
(cl:defmethod front_distance_filtered_m-val ((m <WallPerchStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader px4_wall_ceiling_control-msg:front_distance_filtered_m-val is deprecated.  Use px4_wall_ceiling_control-msg:front_distance_filtered_m instead.")
  (front_distance_filtered_m m))

(cl:ensure-generic-function 'top_distance_raw_m-val :lambda-list '(m))
(cl:defmethod top_distance_raw_m-val ((m <WallPerchStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader px4_wall_ceiling_control-msg:top_distance_raw_m-val is deprecated.  Use px4_wall_ceiling_control-msg:top_distance_raw_m instead.")
  (top_distance_raw_m m))

(cl:ensure-generic-function 'top_distance_filtered_m-val :lambda-list '(m))
(cl:defmethod top_distance_filtered_m-val ((m <WallPerchStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader px4_wall_ceiling_control-msg:top_distance_filtered_m-val is deprecated.  Use px4_wall_ceiling_control-msg:top_distance_filtered_m instead.")
  (top_distance_filtered_m m))

(cl:ensure-generic-function 'progress-val :lambda-list '(m))
(cl:defmethod progress-val ((m <WallPerchStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader px4_wall_ceiling_control-msg:progress-val is deprecated.  Use px4_wall_ceiling_control-msg:progress instead.")
  (progress m))

(cl:ensure-generic-function 'attitude_setpoint-val :lambda-list '(m))
(cl:defmethod attitude_setpoint-val ((m <WallPerchStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader px4_wall_ceiling_control-msg:attitude_setpoint-val is deprecated.  Use px4_wall_ceiling_control-msg:attitude_setpoint instead.")
  (attitude_setpoint m))

(cl:ensure-generic-function 'thrust_normalized-val :lambda-list '(m))
(cl:defmethod thrust_normalized-val ((m <WallPerchStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader px4_wall_ceiling_control-msg:thrust_normalized-val is deprecated.  Use px4_wall_ceiling_control-msg:thrust_normalized instead.")
  (thrust_normalized m))

(cl:ensure-generic-function 'fault_count-val :lambda-list '(m))
(cl:defmethod fault_count-val ((m <WallPerchStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader px4_wall_ceiling_control-msg:fault_count-val is deprecated.  Use px4_wall_ceiling_control-msg:fault_count instead.")
  (fault_count m))

(cl:ensure-generic-function 'reason-val :lambda-list '(m))
(cl:defmethod reason-val ((m <WallPerchStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader px4_wall_ceiling_control-msg:reason-val is deprecated.  Use px4_wall_ceiling_control-msg:reason instead.")
  (reason m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<WallPerchStatus>)))
    "Constants for message type '<WallPerchStatus>"
  '((:STATE_IDLE . 0)
    (:STATE_FRONT_WALL_DETECT . 1)
    (:STATE_STABILIZE_HOVER . 2)
    (:STATE_SLOW_APPROACH . 3)
    (:STATE_FLIP_TO_WALL . 4)
    (:STATE_WALL_CAPTURE . 5)
    (:STATE_WALL_HOLD . 6)
    (:STATE_WALL_PIN . 7)
    (:STATE_DETACH_ROTATE . 8)
    (:STATE_RECOVER . 9)
    (:STATE_EXIT . 10)
    (:STATE_ABORT . 11))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'WallPerchStatus)))
    "Constants for message type 'WallPerchStatus"
  '((:STATE_IDLE . 0)
    (:STATE_FRONT_WALL_DETECT . 1)
    (:STATE_STABILIZE_HOVER . 2)
    (:STATE_SLOW_APPROACH . 3)
    (:STATE_FLIP_TO_WALL . 4)
    (:STATE_WALL_CAPTURE . 5)
    (:STATE_WALL_HOLD . 6)
    (:STATE_WALL_PIN . 7)
    (:STATE_DETACH_ROTATE . 8)
    (:STATE_RECOVER . 9)
    (:STATE_EXIT . 10)
    (:STATE_ABORT . 11))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <WallPerchStatus>) ostream)
  "Serializes a message object of type '<WallPerchStatus>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'state)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'state_name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'state_name))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'enabled) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'acquire_request) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'keep_ownership) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'candidate_valid) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'front_range_fresh) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'top_range_fresh) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'sensor_health_fresh) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'top_preflight_ready) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'odom_fresh) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'flight_state_fresh) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'front_ready) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'flip_ready) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'top_contact_ready) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'top_clear_ready) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'fault_detected) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'neutral_setpoint_requested) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'attitude_setpoint_valid) 1 0)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'front_distance_raw_m))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'front_distance_filtered_m))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'top_distance_raw_m))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'top_distance_filtered_m))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'progress))))
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
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'fault_count)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'fault_count)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'fault_count)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'fault_count)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'reason))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'reason))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <WallPerchStatus>) istream)
  "Deserializes a message object of type '<WallPerchStatus>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'state)) (cl:read-byte istream))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'state_name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'state_name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:setf (cl:slot-value msg 'enabled) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'acquire_request) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'keep_ownership) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'candidate_valid) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'front_range_fresh) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'top_range_fresh) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'sensor_health_fresh) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'top_preflight_ready) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'odom_fresh) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'flight_state_fresh) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'front_ready) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'flip_ready) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'top_contact_ready) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'top_clear_ready) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'fault_detected) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'neutral_setpoint_requested) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'attitude_setpoint_valid) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'front_distance_raw_m) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'front_distance_filtered_m) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'top_distance_raw_m) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'top_distance_filtered_m) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'progress) (roslisp-utils:decode-single-float-bits bits)))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'attitude_setpoint) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'thrust_normalized) (roslisp-utils:decode-single-float-bits bits)))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'fault_count)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'fault_count)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'fault_count)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'fault_count)) (cl:read-byte istream))
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<WallPerchStatus>)))
  "Returns string type for a message object of type '<WallPerchStatus>"
  "px4_wall_ceiling_control/WallPerchStatus")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'WallPerchStatus)))
  "Returns string type for a message object of type 'WallPerchStatus"
  "px4_wall_ceiling_control/WallPerchStatus")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<WallPerchStatus>)))
  "Returns md5sum for a message object of type '<WallPerchStatus>"
  "14c66e7871a6c1f7a0bda70d194bcb80")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'WallPerchStatus)))
  "Returns md5sum for a message object of type 'WallPerchStatus"
  "14c66e7871a6c1f7a0bda70d194bcb80")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<WallPerchStatus>)))
  "Returns full string definition for message of type '<WallPerchStatus>"
  (cl:format cl:nil "std_msgs/Header header~%~%uint8 STATE_IDLE=0~%uint8 STATE_FRONT_WALL_DETECT=1~%uint8 STATE_STABILIZE_HOVER=2~%uint8 STATE_SLOW_APPROACH=3~%uint8 STATE_FLIP_TO_WALL=4~%uint8 STATE_WALL_CAPTURE=5~%uint8 STATE_WALL_HOLD=6~%uint8 STATE_WALL_PIN=7~%uint8 STATE_DETACH_ROTATE=8~%uint8 STATE_RECOVER=9~%uint8 STATE_EXIT=10~%uint8 STATE_ABORT=11~%~%uint8 state~%string state_name~%bool enabled~%bool acquire_request~%bool keep_ownership~%bool candidate_valid~%bool front_range_fresh~%bool top_range_fresh~%bool sensor_health_fresh~%bool top_preflight_ready~%bool odom_fresh~%bool flight_state_fresh~%bool front_ready~%bool flip_ready~%bool top_contact_ready~%bool top_clear_ready~%bool fault_detected~%bool neutral_setpoint_requested~%bool attitude_setpoint_valid~%float32 front_distance_raw_m~%float32 front_distance_filtered_m~%float32 top_distance_raw_m~%float32 top_distance_filtered_m~%float32 progress~%geometry_msgs/Quaternion attitude_setpoint~%# MAVROS AttitudeTarget convention: positive normalized collective thrust.~%float32 thrust_normalized~%uint32 fault_count~%string reason~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'WallPerchStatus)))
  "Returns full string definition for message of type 'WallPerchStatus"
  (cl:format cl:nil "std_msgs/Header header~%~%uint8 STATE_IDLE=0~%uint8 STATE_FRONT_WALL_DETECT=1~%uint8 STATE_STABILIZE_HOVER=2~%uint8 STATE_SLOW_APPROACH=3~%uint8 STATE_FLIP_TO_WALL=4~%uint8 STATE_WALL_CAPTURE=5~%uint8 STATE_WALL_HOLD=6~%uint8 STATE_WALL_PIN=7~%uint8 STATE_DETACH_ROTATE=8~%uint8 STATE_RECOVER=9~%uint8 STATE_EXIT=10~%uint8 STATE_ABORT=11~%~%uint8 state~%string state_name~%bool enabled~%bool acquire_request~%bool keep_ownership~%bool candidate_valid~%bool front_range_fresh~%bool top_range_fresh~%bool sensor_health_fresh~%bool top_preflight_ready~%bool odom_fresh~%bool flight_state_fresh~%bool front_ready~%bool flip_ready~%bool top_contact_ready~%bool top_clear_ready~%bool fault_detected~%bool neutral_setpoint_requested~%bool attitude_setpoint_valid~%float32 front_distance_raw_m~%float32 front_distance_filtered_m~%float32 top_distance_raw_m~%float32 top_distance_filtered_m~%float32 progress~%geometry_msgs/Quaternion attitude_setpoint~%# MAVROS AttitudeTarget convention: positive normalized collective thrust.~%float32 thrust_normalized~%uint32 fault_count~%string reason~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <WallPerchStatus>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     1
     4 (cl:length (cl:slot-value msg 'state_name))
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
     1
     1
     1
     1
     1
     1
     4
     4
     4
     4
     4
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'attitude_setpoint))
     4
     4
     4 (cl:length (cl:slot-value msg 'reason))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <WallPerchStatus>))
  "Converts a ROS message object to a list"
  (cl:list 'WallPerchStatus
    (cl:cons ':header (header msg))
    (cl:cons ':state (state msg))
    (cl:cons ':state_name (state_name msg))
    (cl:cons ':enabled (enabled msg))
    (cl:cons ':acquire_request (acquire_request msg))
    (cl:cons ':keep_ownership (keep_ownership msg))
    (cl:cons ':candidate_valid (candidate_valid msg))
    (cl:cons ':front_range_fresh (front_range_fresh msg))
    (cl:cons ':top_range_fresh (top_range_fresh msg))
    (cl:cons ':sensor_health_fresh (sensor_health_fresh msg))
    (cl:cons ':top_preflight_ready (top_preflight_ready msg))
    (cl:cons ':odom_fresh (odom_fresh msg))
    (cl:cons ':flight_state_fresh (flight_state_fresh msg))
    (cl:cons ':front_ready (front_ready msg))
    (cl:cons ':flip_ready (flip_ready msg))
    (cl:cons ':top_contact_ready (top_contact_ready msg))
    (cl:cons ':top_clear_ready (top_clear_ready msg))
    (cl:cons ':fault_detected (fault_detected msg))
    (cl:cons ':neutral_setpoint_requested (neutral_setpoint_requested msg))
    (cl:cons ':attitude_setpoint_valid (attitude_setpoint_valid msg))
    (cl:cons ':front_distance_raw_m (front_distance_raw_m msg))
    (cl:cons ':front_distance_filtered_m (front_distance_filtered_m msg))
    (cl:cons ':top_distance_raw_m (top_distance_raw_m msg))
    (cl:cons ':top_distance_filtered_m (top_distance_filtered_m msg))
    (cl:cons ':progress (progress msg))
    (cl:cons ':attitude_setpoint (attitude_setpoint msg))
    (cl:cons ':thrust_normalized (thrust_normalized msg))
    (cl:cons ':fault_count (fault_count msg))
    (cl:cons ':reason (reason msg))
))
