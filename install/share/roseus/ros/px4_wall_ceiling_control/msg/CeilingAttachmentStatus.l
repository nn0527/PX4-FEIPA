;; Auto-generated. Do not edit!


(when (boundp 'px4_wall_ceiling_control::CeilingAttachmentStatus)
  (if (not (find-package "PX4_WALL_CEILING_CONTROL"))
    (make-package "PX4_WALL_CEILING_CONTROL"))
  (shadow 'CeilingAttachmentStatus (find-package "PX4_WALL_CEILING_CONTROL")))
(unless (find-package "PX4_WALL_CEILING_CONTROL::CEILINGATTACHMENTSTATUS")
  (make-package "PX4_WALL_CEILING_CONTROL::CEILINGATTACHMENTSTATUS"))

(in-package "ROS")
;;//! \htmlinclude CeilingAttachmentStatus.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(intern "*STATE_NORMAL_FLIGHT*" (find-package "PX4_WALL_CEILING_CONTROL::CEILINGATTACHMENTSTATUS"))
(shadow '*STATE_NORMAL_FLIGHT* (find-package "PX4_WALL_CEILING_CONTROL::CEILINGATTACHMENTSTATUS"))
(defconstant px4_wall_ceiling_control::CeilingAttachmentStatus::*STATE_NORMAL_FLIGHT* 0)
(intern "*STATE_CEILING_ARMED*" (find-package "PX4_WALL_CEILING_CONTROL::CEILINGATTACHMENTSTATUS"))
(shadow '*STATE_CEILING_ARMED* (find-package "PX4_WALL_CEILING_CONTROL::CEILINGATTACHMENTSTATUS"))
(defconstant px4_wall_ceiling_control::CeilingAttachmentStatus::*STATE_CEILING_ARMED* 1)
(intern "*STATE_APPROACH*" (find-package "PX4_WALL_CEILING_CONTROL::CEILINGATTACHMENTSTATUS"))
(shadow '*STATE_APPROACH* (find-package "PX4_WALL_CEILING_CONTROL::CEILINGATTACHMENTSTATUS"))
(defconstant px4_wall_ceiling_control::CeilingAttachmentStatus::*STATE_APPROACH* 2)
(intern "*STATE_ATTACH_CONTROL*" (find-package "PX4_WALL_CEILING_CONTROL::CEILINGATTACHMENTSTATUS"))
(shadow '*STATE_ATTACH_CONTROL* (find-package "PX4_WALL_CEILING_CONTROL::CEILINGATTACHMENTSTATUS"))
(defconstant px4_wall_ceiling_control::CeilingAttachmentStatus::*STATE_ATTACH_CONTROL* 3)
(intern "*STATE_SURFACE_HOLD*" (find-package "PX4_WALL_CEILING_CONTROL::CEILINGATTACHMENTSTATUS"))
(shadow '*STATE_SURFACE_HOLD* (find-package "PX4_WALL_CEILING_CONTROL::CEILINGATTACHMENTSTATUS"))
(defconstant px4_wall_ceiling_control::CeilingAttachmentStatus::*STATE_SURFACE_HOLD* 4)
(intern "*STATE_DETACH*" (find-package "PX4_WALL_CEILING_CONTROL::CEILINGATTACHMENTSTATUS"))
(shadow '*STATE_DETACH* (find-package "PX4_WALL_CEILING_CONTROL::CEILINGATTACHMENTSTATUS"))
(defconstant px4_wall_ceiling_control::CeilingAttachmentStatus::*STATE_DETACH* 5)
(intern "*STATE_RECOVERY_HOVER*" (find-package "PX4_WALL_CEILING_CONTROL::CEILINGATTACHMENTSTATUS"))
(shadow '*STATE_RECOVERY_HOVER* (find-package "PX4_WALL_CEILING_CONTROL::CEILINGATTACHMENTSTATUS"))
(defconstant px4_wall_ceiling_control::CeilingAttachmentStatus::*STATE_RECOVERY_HOVER* 6)
(intern "*STATE_FAULT*" (find-package "PX4_WALL_CEILING_CONTROL::CEILINGATTACHMENTSTATUS"))
(shadow '*STATE_FAULT* (find-package "PX4_WALL_CEILING_CONTROL::CEILINGATTACHMENTSTATUS"))
(defconstant px4_wall_ceiling_control::CeilingAttachmentStatus::*STATE_FAULT* 7)

(defun px4_wall_ceiling_control::CeilingAttachmentStatus-to-symbol (const)
  (cond
        ((= const 0) 'px4_wall_ceiling_control::CeilingAttachmentStatus::*STATE_NORMAL_FLIGHT*)
        ((= const 1) 'px4_wall_ceiling_control::CeilingAttachmentStatus::*STATE_CEILING_ARMED*)
        ((= const 2) 'px4_wall_ceiling_control::CeilingAttachmentStatus::*STATE_APPROACH*)
        ((= const 3) 'px4_wall_ceiling_control::CeilingAttachmentStatus::*STATE_ATTACH_CONTROL*)
        ((= const 4) 'px4_wall_ceiling_control::CeilingAttachmentStatus::*STATE_SURFACE_HOLD*)
        ((= const 5) 'px4_wall_ceiling_control::CeilingAttachmentStatus::*STATE_DETACH*)
        ((= const 6) 'px4_wall_ceiling_control::CeilingAttachmentStatus::*STATE_RECOVERY_HOVER*)
        ((= const 7) 'px4_wall_ceiling_control::CeilingAttachmentStatus::*STATE_FAULT*)
        (t nil)))

(defclass px4_wall_ceiling_control::CeilingAttachmentStatus
  :super ros::object
  :slots (_header _state _state_name _enabled _acquire_request _keep_ownership _candidate_valid _range_fresh _odom_fresh _flight_state_fresh _attach_confirmed _distance_stable _fault_detected _detach_failed _integral_reset_request _wheel_stop_request _mechanism_release_requested _mechanism_release_confirmed _ceiling_distance_raw_m _ceiling_distance_filtered_m _target_distance_m _compression_m _target_compression_m _approach_velocity_z_enu_mps _thrust_body_z_normalized _attitude_setpoint_valid _attitude_setpoint _fault_count _reason ))

(defmethod px4_wall_ceiling_control::CeilingAttachmentStatus
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:state __state) 0)
    ((:state_name __state_name) "")
    ((:enabled __enabled) nil)
    ((:acquire_request __acquire_request) nil)
    ((:keep_ownership __keep_ownership) nil)
    ((:candidate_valid __candidate_valid) nil)
    ((:range_fresh __range_fresh) nil)
    ((:odom_fresh __odom_fresh) nil)
    ((:flight_state_fresh __flight_state_fresh) nil)
    ((:attach_confirmed __attach_confirmed) nil)
    ((:distance_stable __distance_stable) nil)
    ((:fault_detected __fault_detected) nil)
    ((:detach_failed __detach_failed) nil)
    ((:integral_reset_request __integral_reset_request) nil)
    ((:wheel_stop_request __wheel_stop_request) nil)
    ((:mechanism_release_requested __mechanism_release_requested) nil)
    ((:mechanism_release_confirmed __mechanism_release_confirmed) nil)
    ((:ceiling_distance_raw_m __ceiling_distance_raw_m) 0.0)
    ((:ceiling_distance_filtered_m __ceiling_distance_filtered_m) 0.0)
    ((:target_distance_m __target_distance_m) 0.0)
    ((:compression_m __compression_m) 0.0)
    ((:target_compression_m __target_compression_m) 0.0)
    ((:approach_velocity_z_enu_mps __approach_velocity_z_enu_mps) 0.0)
    ((:thrust_body_z_normalized __thrust_body_z_normalized) 0.0)
    ((:attitude_setpoint_valid __attitude_setpoint_valid) nil)
    ((:attitude_setpoint __attitude_setpoint) (instance geometry_msgs::Quaternion :init))
    ((:fault_count __fault_count) 0)
    ((:reason __reason) "")
    )
   (send-super :init)
   (setq _header __header)
   (setq _state (round __state))
   (setq _state_name (string __state_name))
   (setq _enabled __enabled)
   (setq _acquire_request __acquire_request)
   (setq _keep_ownership __keep_ownership)
   (setq _candidate_valid __candidate_valid)
   (setq _range_fresh __range_fresh)
   (setq _odom_fresh __odom_fresh)
   (setq _flight_state_fresh __flight_state_fresh)
   (setq _attach_confirmed __attach_confirmed)
   (setq _distance_stable __distance_stable)
   (setq _fault_detected __fault_detected)
   (setq _detach_failed __detach_failed)
   (setq _integral_reset_request __integral_reset_request)
   (setq _wheel_stop_request __wheel_stop_request)
   (setq _mechanism_release_requested __mechanism_release_requested)
   (setq _mechanism_release_confirmed __mechanism_release_confirmed)
   (setq _ceiling_distance_raw_m (float __ceiling_distance_raw_m))
   (setq _ceiling_distance_filtered_m (float __ceiling_distance_filtered_m))
   (setq _target_distance_m (float __target_distance_m))
   (setq _compression_m (float __compression_m))
   (setq _target_compression_m (float __target_compression_m))
   (setq _approach_velocity_z_enu_mps (float __approach_velocity_z_enu_mps))
   (setq _thrust_body_z_normalized (float __thrust_body_z_normalized))
   (setq _attitude_setpoint_valid __attitude_setpoint_valid)
   (setq _attitude_setpoint __attitude_setpoint)
   (setq _fault_count (round __fault_count))
   (setq _reason (string __reason))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:state
   (&optional __state)
   (if __state (setq _state __state)) _state)
  (:state_name
   (&optional __state_name)
   (if __state_name (setq _state_name __state_name)) _state_name)
  (:enabled
   (&optional (__enabled :null))
   (if (not (eq __enabled :null)) (setq _enabled __enabled)) _enabled)
  (:acquire_request
   (&optional (__acquire_request :null))
   (if (not (eq __acquire_request :null)) (setq _acquire_request __acquire_request)) _acquire_request)
  (:keep_ownership
   (&optional (__keep_ownership :null))
   (if (not (eq __keep_ownership :null)) (setq _keep_ownership __keep_ownership)) _keep_ownership)
  (:candidate_valid
   (&optional (__candidate_valid :null))
   (if (not (eq __candidate_valid :null)) (setq _candidate_valid __candidate_valid)) _candidate_valid)
  (:range_fresh
   (&optional (__range_fresh :null))
   (if (not (eq __range_fresh :null)) (setq _range_fresh __range_fresh)) _range_fresh)
  (:odom_fresh
   (&optional (__odom_fresh :null))
   (if (not (eq __odom_fresh :null)) (setq _odom_fresh __odom_fresh)) _odom_fresh)
  (:flight_state_fresh
   (&optional (__flight_state_fresh :null))
   (if (not (eq __flight_state_fresh :null)) (setq _flight_state_fresh __flight_state_fresh)) _flight_state_fresh)
  (:attach_confirmed
   (&optional (__attach_confirmed :null))
   (if (not (eq __attach_confirmed :null)) (setq _attach_confirmed __attach_confirmed)) _attach_confirmed)
  (:distance_stable
   (&optional (__distance_stable :null))
   (if (not (eq __distance_stable :null)) (setq _distance_stable __distance_stable)) _distance_stable)
  (:fault_detected
   (&optional (__fault_detected :null))
   (if (not (eq __fault_detected :null)) (setq _fault_detected __fault_detected)) _fault_detected)
  (:detach_failed
   (&optional (__detach_failed :null))
   (if (not (eq __detach_failed :null)) (setq _detach_failed __detach_failed)) _detach_failed)
  (:integral_reset_request
   (&optional (__integral_reset_request :null))
   (if (not (eq __integral_reset_request :null)) (setq _integral_reset_request __integral_reset_request)) _integral_reset_request)
  (:wheel_stop_request
   (&optional (__wheel_stop_request :null))
   (if (not (eq __wheel_stop_request :null)) (setq _wheel_stop_request __wheel_stop_request)) _wheel_stop_request)
  (:mechanism_release_requested
   (&optional (__mechanism_release_requested :null))
   (if (not (eq __mechanism_release_requested :null)) (setq _mechanism_release_requested __mechanism_release_requested)) _mechanism_release_requested)
  (:mechanism_release_confirmed
   (&optional (__mechanism_release_confirmed :null))
   (if (not (eq __mechanism_release_confirmed :null)) (setq _mechanism_release_confirmed __mechanism_release_confirmed)) _mechanism_release_confirmed)
  (:ceiling_distance_raw_m
   (&optional __ceiling_distance_raw_m)
   (if __ceiling_distance_raw_m (setq _ceiling_distance_raw_m __ceiling_distance_raw_m)) _ceiling_distance_raw_m)
  (:ceiling_distance_filtered_m
   (&optional __ceiling_distance_filtered_m)
   (if __ceiling_distance_filtered_m (setq _ceiling_distance_filtered_m __ceiling_distance_filtered_m)) _ceiling_distance_filtered_m)
  (:target_distance_m
   (&optional __target_distance_m)
   (if __target_distance_m (setq _target_distance_m __target_distance_m)) _target_distance_m)
  (:compression_m
   (&optional __compression_m)
   (if __compression_m (setq _compression_m __compression_m)) _compression_m)
  (:target_compression_m
   (&optional __target_compression_m)
   (if __target_compression_m (setq _target_compression_m __target_compression_m)) _target_compression_m)
  (:approach_velocity_z_enu_mps
   (&optional __approach_velocity_z_enu_mps)
   (if __approach_velocity_z_enu_mps (setq _approach_velocity_z_enu_mps __approach_velocity_z_enu_mps)) _approach_velocity_z_enu_mps)
  (:thrust_body_z_normalized
   (&optional __thrust_body_z_normalized)
   (if __thrust_body_z_normalized (setq _thrust_body_z_normalized __thrust_body_z_normalized)) _thrust_body_z_normalized)
  (:attitude_setpoint_valid
   (&optional (__attitude_setpoint_valid :null))
   (if (not (eq __attitude_setpoint_valid :null)) (setq _attitude_setpoint_valid __attitude_setpoint_valid)) _attitude_setpoint_valid)
  (:attitude_setpoint
   (&rest __attitude_setpoint)
   (if (keywordp (car __attitude_setpoint))
       (send* _attitude_setpoint __attitude_setpoint)
     (progn
       (if __attitude_setpoint (setq _attitude_setpoint (car __attitude_setpoint)))
       _attitude_setpoint)))
  (:fault_count
   (&optional __fault_count)
   (if __fault_count (setq _fault_count __fault_count)) _fault_count)
  (:reason
   (&optional __reason)
   (if __reason (setq _reason __reason)) _reason)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; uint8 _state
    1
    ;; string _state_name
    4 (length _state_name)
    ;; bool _enabled
    1
    ;; bool _acquire_request
    1
    ;; bool _keep_ownership
    1
    ;; bool _candidate_valid
    1
    ;; bool _range_fresh
    1
    ;; bool _odom_fresh
    1
    ;; bool _flight_state_fresh
    1
    ;; bool _attach_confirmed
    1
    ;; bool _distance_stable
    1
    ;; bool _fault_detected
    1
    ;; bool _detach_failed
    1
    ;; bool _integral_reset_request
    1
    ;; bool _wheel_stop_request
    1
    ;; bool _mechanism_release_requested
    1
    ;; bool _mechanism_release_confirmed
    1
    ;; float32 _ceiling_distance_raw_m
    4
    ;; float32 _ceiling_distance_filtered_m
    4
    ;; float32 _target_distance_m
    4
    ;; float32 _compression_m
    4
    ;; float32 _target_compression_m
    4
    ;; float32 _approach_velocity_z_enu_mps
    4
    ;; float32 _thrust_body_z_normalized
    4
    ;; bool _attitude_setpoint_valid
    1
    ;; geometry_msgs/Quaternion _attitude_setpoint
    (send _attitude_setpoint :serialization-length)
    ;; uint32 _fault_count
    4
    ;; string _reason
    4 (length _reason)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; uint8 _state
       (write-byte _state s)
     ;; string _state_name
       (write-long (length _state_name) s) (princ _state_name s)
     ;; bool _enabled
       (if _enabled (write-byte -1 s) (write-byte 0 s))
     ;; bool _acquire_request
       (if _acquire_request (write-byte -1 s) (write-byte 0 s))
     ;; bool _keep_ownership
       (if _keep_ownership (write-byte -1 s) (write-byte 0 s))
     ;; bool _candidate_valid
       (if _candidate_valid (write-byte -1 s) (write-byte 0 s))
     ;; bool _range_fresh
       (if _range_fresh (write-byte -1 s) (write-byte 0 s))
     ;; bool _odom_fresh
       (if _odom_fresh (write-byte -1 s) (write-byte 0 s))
     ;; bool _flight_state_fresh
       (if _flight_state_fresh (write-byte -1 s) (write-byte 0 s))
     ;; bool _attach_confirmed
       (if _attach_confirmed (write-byte -1 s) (write-byte 0 s))
     ;; bool _distance_stable
       (if _distance_stable (write-byte -1 s) (write-byte 0 s))
     ;; bool _fault_detected
       (if _fault_detected (write-byte -1 s) (write-byte 0 s))
     ;; bool _detach_failed
       (if _detach_failed (write-byte -1 s) (write-byte 0 s))
     ;; bool _integral_reset_request
       (if _integral_reset_request (write-byte -1 s) (write-byte 0 s))
     ;; bool _wheel_stop_request
       (if _wheel_stop_request (write-byte -1 s) (write-byte 0 s))
     ;; bool _mechanism_release_requested
       (if _mechanism_release_requested (write-byte -1 s) (write-byte 0 s))
     ;; bool _mechanism_release_confirmed
       (if _mechanism_release_confirmed (write-byte -1 s) (write-byte 0 s))
     ;; float32 _ceiling_distance_raw_m
       (sys::poke _ceiling_distance_raw_m (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _ceiling_distance_filtered_m
       (sys::poke _ceiling_distance_filtered_m (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _target_distance_m
       (sys::poke _target_distance_m (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _compression_m
       (sys::poke _compression_m (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _target_compression_m
       (sys::poke _target_compression_m (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _approach_velocity_z_enu_mps
       (sys::poke _approach_velocity_z_enu_mps (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _thrust_body_z_normalized
       (sys::poke _thrust_body_z_normalized (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; bool _attitude_setpoint_valid
       (if _attitude_setpoint_valid (write-byte -1 s) (write-byte 0 s))
     ;; geometry_msgs/Quaternion _attitude_setpoint
       (send _attitude_setpoint :serialize s)
     ;; uint32 _fault_count
       (write-long _fault_count s)
     ;; string _reason
       (write-long (length _reason) s) (princ _reason s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; uint8 _state
     (setq _state (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; string _state_name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _state_name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; bool _enabled
     (setq _enabled (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _acquire_request
     (setq _acquire_request (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _keep_ownership
     (setq _keep_ownership (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _candidate_valid
     (setq _candidate_valid (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _range_fresh
     (setq _range_fresh (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _odom_fresh
     (setq _odom_fresh (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _flight_state_fresh
     (setq _flight_state_fresh (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _attach_confirmed
     (setq _attach_confirmed (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _distance_stable
     (setq _distance_stable (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _fault_detected
     (setq _fault_detected (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _detach_failed
     (setq _detach_failed (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _integral_reset_request
     (setq _integral_reset_request (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _wheel_stop_request
     (setq _wheel_stop_request (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _mechanism_release_requested
     (setq _mechanism_release_requested (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _mechanism_release_confirmed
     (setq _mechanism_release_confirmed (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; float32 _ceiling_distance_raw_m
     (setq _ceiling_distance_raw_m (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _ceiling_distance_filtered_m
     (setq _ceiling_distance_filtered_m (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _target_distance_m
     (setq _target_distance_m (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _compression_m
     (setq _compression_m (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _target_compression_m
     (setq _target_compression_m (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _approach_velocity_z_enu_mps
     (setq _approach_velocity_z_enu_mps (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _thrust_body_z_normalized
     (setq _thrust_body_z_normalized (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; bool _attitude_setpoint_valid
     (setq _attitude_setpoint_valid (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; geometry_msgs/Quaternion _attitude_setpoint
     (send _attitude_setpoint :deserialize buf ptr-) (incf ptr- (send _attitude_setpoint :serialization-length))
   ;; uint32 _fault_count
     (setq _fault_count (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; string _reason
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _reason (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get px4_wall_ceiling_control::CeilingAttachmentStatus :md5sum-) "ee6fc76925906c4e79bd0785bd2ec2d9")
(setf (get px4_wall_ceiling_control::CeilingAttachmentStatus :datatype-) "px4_wall_ceiling_control/CeilingAttachmentStatus")
(setf (get px4_wall_ceiling_control::CeilingAttachmentStatus :definition-)
      "std_msgs/Header header

uint8 STATE_NORMAL_FLIGHT=0
uint8 STATE_CEILING_ARMED=1
uint8 STATE_APPROACH=2
uint8 STATE_ATTACH_CONTROL=3
uint8 STATE_SURFACE_HOLD=4
uint8 STATE_DETACH=5
uint8 STATE_RECOVERY_HOVER=6
uint8 STATE_FAULT=7

uint8 state
string state_name
bool enabled
bool acquire_request
bool keep_ownership
bool candidate_valid
bool range_fresh
bool odom_fresh
bool flight_state_fresh
bool attach_confirmed
bool distance_stable
bool fault_detected
bool detach_failed
bool integral_reset_request
bool wheel_stop_request
bool mechanism_release_requested
bool mechanism_release_confirmed
float32 ceiling_distance_raw_m
float32 ceiling_distance_filtered_m
float32 target_distance_m
float32 compression_m
float32 target_compression_m
# ROS/MAVROS ENU convention: positive z means upward/toward a horizontal ceiling.
float32 approach_velocity_z_enu_mps
# PX4 body FRD convention retained from the reference: negative z is upward thrust.
float32 thrust_body_z_normalized
bool attitude_setpoint_valid
geometry_msgs/Quaternion attitude_setpoint
uint32 fault_count
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

")



(provide :px4_wall_ceiling_control/CeilingAttachmentStatus "ee6fc76925906c4e79bd0785bd2ec2d9")


