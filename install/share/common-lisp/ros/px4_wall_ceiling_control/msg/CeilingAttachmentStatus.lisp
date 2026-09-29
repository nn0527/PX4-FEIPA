; Auto-generated. Do not edit!


(cl:in-package px4_wall_ceiling_control-msg)


;//! \htmlinclude CeilingAttachmentStatus.msg.html

(cl:defclass <CeilingAttachmentStatus> (roslisp-msg-protocol:ros-message)
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
   (range_fresh
    :reader range_fresh
    :initarg :range_fresh
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
   (attach_confirmed
    :reader attach_confirmed
    :initarg :attach_confirmed
    :type cl:boolean
    :initform cl:nil)
   (distance_stable
    :reader distance_stable
    :initarg :distance_stable
    :type cl:boolean
    :initform cl:nil)
   (fault_detected
    :reader fault_detected
    :initarg :fault_detected
    :type cl:boolean
    :initform cl:nil)
   (detach_failed
    :reader detach_failed
    :initarg :detach_failed
    :type cl:boolean
    :initform cl:nil)
   (integral_reset_request
    :reader integral_reset_request
    :initarg :integral_reset_request
    :type cl:boolean
    :initform cl:nil)
   (wheel_stop_request
    :reader wheel_stop_request
    :initarg :wheel_stop_request
    :type cl:boolean
    :initform cl:nil)
   (mechanism_release_requested
    :reader mechanism_release_requested
    :initarg :mechanism_release_requested
    :type cl:boolean
    :initform cl:nil)
   (mechanism_release_confirmed
    :reader mechanism_release_confirmed
    :initarg :mechanism_release_confirmed
    :type cl:boolean
    :initform cl:nil)
   (ceiling_distance_raw_m
    :reader ceiling_distance_raw_m
    :initarg :ceiling_distance_raw_m
    :type cl:float
    :initform 0.0)
   (ceiling_distance_filtered_m
    :reader ceiling_distance_filtered_m
    :initarg :ceiling_distance_filtered_m
    :type cl:float
    :initform 0.0)
   (target_distance_m
    :reader target_distance_m
    :initarg :target_distance_m
    :type cl:float
    :initform 0.0)
   (compression_m
    :reader compression_m
    :initarg :compression_m
    :type cl:float
    :initform 0.0)
   (target_compression_m
    :reader target_compression_m
    :initarg :target_compression_m
    :type cl:float
    :initform 0.0)
   (approach_velocity_z_enu_mps
    :reader approach_velocity_z_enu_mps
    :initarg :approach_velocity_z_enu_mps
    :type cl:float
    :initform 0.0)
   (thrust_body_z_normalized
    :reader thrust_body_z_normalized
    :initarg :thrust_body_z_normalized
    :type cl:float
    :initform 0.0)
   (attitude_setpoint_valid
    :reader attitude_setpoint_valid
    :initarg :attitude_setpoint_valid
    :type cl:boolean
    :initform cl:nil)
   (attitude_setpoint
    :reader attitude_setpoint
    :initarg :attitude_setpoint
    :type geometry_msgs-msg:Quaternion
    :initform (cl:make-instance 'geometry_msgs-msg:Quaternion))
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

(cl:defclass CeilingAttachmentStatus (<CeilingAttachmentStatus>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CeilingAttachmentStatus>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CeilingAttachmentStatus)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name px4_wall_ceiling_control-msg:<CeilingAttachmentStatus> is deprecated: use px4_wall_ceiling_control-msg:CeilingAttachmentStatus instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <CeilingAttachmentStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader px4_wall_ceiling_control-msg:header-val is deprecated.  Use px4_wall_ceiling_control-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'state-val :lambda-list '(m))
(cl:defmethod state-val ((m <CeilingAttachmentStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader px4_wall_ceiling_control-msg:state-val is deprecated.  Use px4_wall_ceiling_control-msg:state instead.")
  (state m))

(cl:ensure-generic-function 'state_name-val :lambda-list '(m))
(cl:defmethod state_name-val ((m <CeilingAttachmentStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader px4_wall_ceiling_control-msg:state_name-val is deprecated.  Use px4_wall_ceiling_control-msg:state_name instead.")
  (state_name m))

(cl:ensure-generic-function 'enabled-val :lambda-list '(m))
(cl:defmethod enabled-val ((m <CeilingAttachmentStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader px4_wall_ceiling_control-msg:enabled-val is deprecated.  Use px4_wall_ceiling_control-msg:enabled instead.")
  (enabled m))

(cl:ensure-generic-function 'acquire_request-val :lambda-list '(m))
(cl:defmethod acquire_request-val ((m <CeilingAttachmentStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader px4_wall_ceiling_control-msg:acquire_request-val is deprecated.  Use px4_wall_ceiling_control-msg:acquire_request instead.")
  (acquire_request m))

(cl:ensure-generic-function 'keep_ownership-val :lambda-list '(m))
(cl:defmethod keep_ownership-val ((m <CeilingAttachmentStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader px4_wall_ceiling_control-msg:keep_ownership-val is deprecated.  Use px4_wall_ceiling_control-msg:keep_ownership instead.")
  (keep_ownership m))

(cl:ensure-generic-function 'candidate_valid-val :lambda-list '(m))
(cl:defmethod candidate_valid-val ((m <CeilingAttachmentStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader px4_wall_ceiling_control-msg:candidate_valid-val is deprecated.  Use px4_wall_ceiling_control-msg:candidate_valid instead.")
  (candidate_valid m))

(cl:ensure-generic-function 'range_fresh-val :lambda-list '(m))
(cl:defmethod range_fresh-val ((m <CeilingAttachmentStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader px4_wall_ceiling_control-msg:range_fresh-val is deprecated.  Use px4_wall_ceiling_control-msg:range_fresh instead.")
  (range_fresh m))

(cl:ensure-generic-function 'odom_fresh-val :lambda-list '(m))
(cl:defmethod odom_fresh-val ((m <CeilingAttachmentStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader px4_wall_ceiling_control-msg:odom_fresh-val is deprecated.  Use px4_wall_ceiling_control-msg:odom_fresh instead.")
  (odom_fresh m))

(cl:ensure-generic-function 'flight_state_fresh-val :lambda-list '(m))
(cl:defmethod flight_state_fresh-val ((m <CeilingAttachmentStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader px4_wall_ceiling_control-msg:flight_state_fresh-val is deprecated.  Use px4_wall_ceiling_control-msg:flight_state_fresh instead.")
  (flight_state_fresh m))

(cl:ensure-generic-function 'attach_confirmed-val :lambda-list '(m))
(cl:defmethod attach_confirmed-val ((m <CeilingAttachmentStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader px4_wall_ceiling_control-msg:attach_confirmed-val is deprecated.  Use px4_wall_ceiling_control-msg:attach_confirmed instead.")
  (attach_confirmed m))

(cl:ensure-generic-function 'distance_stable-val :lambda-list '(m))
(cl:defmethod distance_stable-val ((m <CeilingAttachmentStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader px4_wall_ceiling_control-msg:distance_stable-val is deprecated.  Use px4_wall_ceiling_control-msg:distance_stable instead.")
  (distance_stable m))

(cl:ensure-generic-function 'fault_detected-val :lambda-list '(m))
(cl:defmethod fault_detected-val ((m <CeilingAttachmentStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader px4_wall_ceiling_control-msg:fault_detected-val is deprecated.  Use px4_wall_ceiling_control-msg:fault_detected instead.")
  (fault_detected m))

(cl:ensure-generic-function 'detach_failed-val :lambda-list '(m))
(cl:defmethod detach_failed-val ((m <CeilingAttachmentStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader px4_wall_ceiling_control-msg:detach_failed-val is deprecated.  Use px4_wall_ceiling_control-msg:detach_failed instead.")
  (detach_failed m))

(cl:ensure-generic-function 'integral_reset_request-val :lambda-list '(m))
(cl:defmethod integral_reset_request-val ((m <CeilingAttachmentStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader px4_wall_ceiling_control-msg:integral_reset_request-val is deprecated.  Use px4_wall_ceiling_control-msg:integral_reset_request instead.")
  (integral_reset_request m))

(cl:ensure-generic-function 'wheel_stop_request-val :lambda-list '(m))
(cl:defmethod wheel_stop_request-val ((m <CeilingAttachmentStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader px4_wall_ceiling_control-msg:wheel_stop_request-val is deprecated.  Use px4_wall_ceiling_control-msg:wheel_stop_request instead.")
  (wheel_stop_request m))

(cl:ensure-generic-function 'mechanism_release_requested-val :lambda-list '(m))
(cl:defmethod mechanism_release_requested-val ((m <CeilingAttachmentStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader px4_wall_ceiling_control-msg:mechanism_release_requested-val is deprecated.  Use px4_wall_ceiling_control-msg:mechanism_release_requested instead.")
  (mechanism_release_requested m))

(cl:ensure-generic-function 'mechanism_release_confirmed-val :lambda-list '(m))
(cl:defmethod mechanism_release_confirmed-val ((m <CeilingAttachmentStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader px4_wall_ceiling_control-msg:mechanism_release_confirmed-val is deprecated.  Use px4_wall_ceiling_control-msg:mechanism_release_confirmed instead.")
  (mechanism_release_confirmed m))

(cl:ensure-generic-function 'ceiling_distance_raw_m-val :lambda-list '(m))
(cl:defmethod ceiling_distance_raw_m-val ((m <CeilingAttachmentStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader px4_wall_ceiling_control-msg:ceiling_distance_raw_m-val is deprecated.  Use px4_wall_ceiling_control-msg:ceiling_distance_raw_m instead.")
  (ceiling_distance_raw_m m))

(cl:ensure-generic-function 'ceiling_distance_filtered_m-val :lambda-list '(m))
(cl:defmethod ceiling_distance_filtered_m-val ((m <CeilingAttachmentStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader px4_wall_ceiling_control-msg:ceiling_distance_filtered_m-val is deprecated.  Use px4_wall_ceiling_control-msg:ceiling_distance_filtered_m instead.")
  (ceiling_distance_filtered_m m))

(cl:ensure-generic-function 'target_distance_m-val :lambda-list '(m))
(cl:defmethod target_distance_m-val ((m <CeilingAttachmentStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader px4_wall_ceiling_control-msg:target_distance_m-val is deprecated.  Use px4_wall_ceiling_control-msg:target_distance_m instead.")
  (target_distance_m m))

(cl:ensure-generic-function 'compression_m-val :lambda-list '(m))
(cl:defmethod compression_m-val ((m <CeilingAttachmentStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader px4_wall_ceiling_control-msg:compression_m-val is deprecated.  Use px4_wall_ceiling_control-msg:compression_m instead.")
  (compression_m m))

(cl:ensure-generic-function 'target_compression_m-val :lambda-list '(m))
(cl:defmethod target_compression_m-val ((m <CeilingAttachmentStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader px4_wall_ceiling_control-msg:target_compression_m-val is deprecated.  Use px4_wall_ceiling_control-msg:target_compression_m instead.")
  (target_compression_m m))

(cl:ensure-generic-function 'approach_velocity_z_enu_mps-val :lambda-list '(m))
(cl:defmethod approach_velocity_z_enu_mps-val ((m <CeilingAttachmentStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader px4_wall_ceiling_control-msg:approach_velocity_z_enu_mps-val is deprecated.  Use px4_wall_ceiling_control-msg:approach_velocity_z_enu_mps instead.")
  (approach_velocity_z_enu_mps m))

(cl:ensure-generic-function 'thrust_body_z_normalized-val :lambda-list '(m))
(cl:defmethod thrust_body_z_normalized-val ((m <CeilingAttachmentStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader px4_wall_ceiling_control-msg:thrust_body_z_normalized-val is deprecated.  Use px4_wall_ceiling_control-msg:thrust_body_z_normalized instead.")
  (thrust_body_z_normalized m))

(cl:ensure-generic-function 'attitude_setpoint_valid-val :lambda-list '(m))
(cl:defmethod attitude_setpoint_valid-val ((m <CeilingAttachmentStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader px4_wall_ceiling_control-msg:attitude_setpoint_valid-val is deprecated.  Use px4_wall_ceiling_control-msg:attitude_setpoint_valid instead.")
  (attitude_setpoint_valid m))

(cl:ensure-generic-function 'attitude_setpoint-val :lambda-list '(m))
(cl:defmethod attitude_setpoint-val ((m <CeilingAttachmentStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader px4_wall_ceiling_control-msg:attitude_setpoint-val is deprecated.  Use px4_wall_ceiling_control-msg:attitude_setpoint instead.")
  (attitude_setpoint m))

(cl:ensure-generic-function 'fault_count-val :lambda-list '(m))
(cl:defmethod fault_count-val ((m <CeilingAttachmentStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader px4_wall_ceiling_control-msg:fault_count-val is deprecated.  Use px4_wall_ceiling_control-msg:fault_count instead.")
  (fault_count m))

(cl:ensure-generic-function 'reason-val :lambda-list '(m))
(cl:defmethod reason-val ((m <CeilingAttachmentStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader px4_wall_ceiling_control-msg:reason-val is deprecated.  Use px4_wall_ceiling_control-msg:reason instead.")
  (reason m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<CeilingAttachmentStatus>)))
    "Constants for message type '<CeilingAttachmentStatus>"
  '((:STATE_NORMAL_FLIGHT . 0)
    (:STATE_CEILING_ARMED . 1)
    (:STATE_APPROACH . 2)
    (:STATE_ATTACH_CONTROL . 3)
    (:STATE_SURFACE_HOLD . 4)
    (:STATE_DETACH . 5)
    (:STATE_RECOVERY_HOVER . 6)
    (:STATE_FAULT . 7))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'CeilingAttachmentStatus)))
    "Constants for message type 'CeilingAttachmentStatus"
  '((:STATE_NORMAL_FLIGHT . 0)
    (:STATE_CEILING_ARMED . 1)
    (:STATE_APPROACH . 2)
    (:STATE_ATTACH_CONTROL . 3)
    (:STATE_SURFACE_HOLD . 4)
    (:STATE_DETACH . 5)
    (:STATE_RECOVERY_HOVER . 6)
    (:STATE_FAULT . 7))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CeilingAttachmentStatus>) ostream)
  "Serializes a message object of type '<CeilingAttachmentStatus>"
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
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'range_fresh) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'odom_fresh) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'flight_state_fresh) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'attach_confirmed) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'distance_stable) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'fault_detected) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'detach_failed) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'integral_reset_request) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'wheel_stop_request) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'mechanism_release_requested) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'mechanism_release_confirmed) 1 0)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'ceiling_distance_raw_m))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'ceiling_distance_filtered_m))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'target_distance_m))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'compression_m))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'target_compression_m))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'approach_velocity_z_enu_mps))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'thrust_body_z_normalized))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'attitude_setpoint_valid) 1 0)) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'attitude_setpoint) ostream)
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CeilingAttachmentStatus>) istream)
  "Deserializes a message object of type '<CeilingAttachmentStatus>"
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
    (cl:setf (cl:slot-value msg 'range_fresh) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'odom_fresh) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'flight_state_fresh) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'attach_confirmed) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'distance_stable) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'fault_detected) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'detach_failed) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'integral_reset_request) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'wheel_stop_request) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'mechanism_release_requested) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'mechanism_release_confirmed) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'ceiling_distance_raw_m) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'ceiling_distance_filtered_m) (roslisp-utils:decode-single-float-bits bits)))
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
    (cl:setf (cl:slot-value msg 'compression_m) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'target_compression_m) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'approach_velocity_z_enu_mps) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'thrust_body_z_normalized) (roslisp-utils:decode-single-float-bits bits)))
    (cl:setf (cl:slot-value msg 'attitude_setpoint_valid) (cl:not (cl:zerop (cl:read-byte istream))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'attitude_setpoint) istream)
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CeilingAttachmentStatus>)))
  "Returns string type for a message object of type '<CeilingAttachmentStatus>"
  "px4_wall_ceiling_control/CeilingAttachmentStatus")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CeilingAttachmentStatus)))
  "Returns string type for a message object of type 'CeilingAttachmentStatus"
  "px4_wall_ceiling_control/CeilingAttachmentStatus")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CeilingAttachmentStatus>)))
  "Returns md5sum for a message object of type '<CeilingAttachmentStatus>"
  "ee6fc76925906c4e79bd0785bd2ec2d9")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CeilingAttachmentStatus)))
  "Returns md5sum for a message object of type 'CeilingAttachmentStatus"
  "ee6fc76925906c4e79bd0785bd2ec2d9")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CeilingAttachmentStatus>)))
  "Returns full string definition for message of type '<CeilingAttachmentStatus>"
  (cl:format cl:nil "std_msgs/Header header~%~%uint8 STATE_NORMAL_FLIGHT=0~%uint8 STATE_CEILING_ARMED=1~%uint8 STATE_APPROACH=2~%uint8 STATE_ATTACH_CONTROL=3~%uint8 STATE_SURFACE_HOLD=4~%uint8 STATE_DETACH=5~%uint8 STATE_RECOVERY_HOVER=6~%uint8 STATE_FAULT=7~%~%uint8 state~%string state_name~%bool enabled~%bool acquire_request~%bool keep_ownership~%bool candidate_valid~%bool range_fresh~%bool odom_fresh~%bool flight_state_fresh~%bool attach_confirmed~%bool distance_stable~%bool fault_detected~%bool detach_failed~%bool integral_reset_request~%bool wheel_stop_request~%bool mechanism_release_requested~%bool mechanism_release_confirmed~%float32 ceiling_distance_raw_m~%float32 ceiling_distance_filtered_m~%float32 target_distance_m~%float32 compression_m~%float32 target_compression_m~%# ROS/MAVROS ENU convention: positive z means upward/toward a horizontal ceiling.~%float32 approach_velocity_z_enu_mps~%# PX4 body FRD convention retained from the reference: negative z is upward thrust.~%float32 thrust_body_z_normalized~%bool attitude_setpoint_valid~%geometry_msgs/Quaternion attitude_setpoint~%uint32 fault_count~%string reason~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CeilingAttachmentStatus)))
  "Returns full string definition for message of type 'CeilingAttachmentStatus"
  (cl:format cl:nil "std_msgs/Header header~%~%uint8 STATE_NORMAL_FLIGHT=0~%uint8 STATE_CEILING_ARMED=1~%uint8 STATE_APPROACH=2~%uint8 STATE_ATTACH_CONTROL=3~%uint8 STATE_SURFACE_HOLD=4~%uint8 STATE_DETACH=5~%uint8 STATE_RECOVERY_HOVER=6~%uint8 STATE_FAULT=7~%~%uint8 state~%string state_name~%bool enabled~%bool acquire_request~%bool keep_ownership~%bool candidate_valid~%bool range_fresh~%bool odom_fresh~%bool flight_state_fresh~%bool attach_confirmed~%bool distance_stable~%bool fault_detected~%bool detach_failed~%bool integral_reset_request~%bool wheel_stop_request~%bool mechanism_release_requested~%bool mechanism_release_confirmed~%float32 ceiling_distance_raw_m~%float32 ceiling_distance_filtered_m~%float32 target_distance_m~%float32 compression_m~%float32 target_compression_m~%# ROS/MAVROS ENU convention: positive z means upward/toward a horizontal ceiling.~%float32 approach_velocity_z_enu_mps~%# PX4 body FRD convention retained from the reference: negative z is upward thrust.~%float32 thrust_body_z_normalized~%bool attitude_setpoint_valid~%geometry_msgs/Quaternion attitude_setpoint~%uint32 fault_count~%string reason~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CeilingAttachmentStatus>))
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
     4
     4
     4
     4
     4
     4
     4
     1
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'attitude_setpoint))
     4
     4 (cl:length (cl:slot-value msg 'reason))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CeilingAttachmentStatus>))
  "Converts a ROS message object to a list"
  (cl:list 'CeilingAttachmentStatus
    (cl:cons ':header (header msg))
    (cl:cons ':state (state msg))
    (cl:cons ':state_name (state_name msg))
    (cl:cons ':enabled (enabled msg))
    (cl:cons ':acquire_request (acquire_request msg))
    (cl:cons ':keep_ownership (keep_ownership msg))
    (cl:cons ':candidate_valid (candidate_valid msg))
    (cl:cons ':range_fresh (range_fresh msg))
    (cl:cons ':odom_fresh (odom_fresh msg))
    (cl:cons ':flight_state_fresh (flight_state_fresh msg))
    (cl:cons ':attach_confirmed (attach_confirmed msg))
    (cl:cons ':distance_stable (distance_stable msg))
    (cl:cons ':fault_detected (fault_detected msg))
    (cl:cons ':detach_failed (detach_failed msg))
    (cl:cons ':integral_reset_request (integral_reset_request msg))
    (cl:cons ':wheel_stop_request (wheel_stop_request msg))
    (cl:cons ':mechanism_release_requested (mechanism_release_requested msg))
    (cl:cons ':mechanism_release_confirmed (mechanism_release_confirmed msg))
    (cl:cons ':ceiling_distance_raw_m (ceiling_distance_raw_m msg))
    (cl:cons ':ceiling_distance_filtered_m (ceiling_distance_filtered_m msg))
    (cl:cons ':target_distance_m (target_distance_m msg))
    (cl:cons ':compression_m (compression_m msg))
    (cl:cons ':target_compression_m (target_compression_m msg))
    (cl:cons ':approach_velocity_z_enu_mps (approach_velocity_z_enu_mps msg))
    (cl:cons ':thrust_body_z_normalized (thrust_body_z_normalized msg))
    (cl:cons ':attitude_setpoint_valid (attitude_setpoint_valid msg))
    (cl:cons ':attitude_setpoint (attitude_setpoint msg))
    (cl:cons ':fault_count (fault_count msg))
    (cl:cons ':reason (reason msg))
))
